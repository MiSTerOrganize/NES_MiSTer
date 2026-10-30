// hd3_add.sv - stage 4 additions (HDPACK_DESIGN.md "8a"): the sprites <addition>s insert.
//
// Runs once per frame, after hd3_dict has resolved the frame (frame_done: every record's merged
// addition list is in the dictionary's pool), and before RENDER draws it. It reproduces
// hdref.process_additions as LANES (the storage scheme modelled by tools/sim3/addhw.py):
//
// for each source line y = 0..239:
//   - sprite sources: the line's valid fetches whose record has additions. A fetch's members are
//     the pixels where sprites are shown and the fetch is one of the first four valid fetches
//     covering the pixel; its rank there is k. Members of one rank are cut into runs of
//     consecutive pixels (the column x - X steps by one).
//   - BG sources, only on a line Mesen scans (process_bg: the tile row starts, the scroll t
//     changed, or the previous scanned line had a BG tile with additions): pixels whose BG slot
//     is a span with additions, k = 4, cut into runs of consecutive pixels whose column steps by
//     one and whose tile is the same (CHR index, palette, row).
//   - each run and each addition of its record (file order) makes a lane: the run moved by
//     (dx, dy) = (ox, oy), each negated when the source is mirrored that way, and clipped to the
//     screen. A line's lanes are sorted by (-dx, k, addition index) and appended, in that order,
//     to their target line. Two lanes with an equal key cover disjoint pixels (same dx and
//     target line, different source pixels), so their order never shows.
//   - pixel (0,0): virtual line 240 = line 239's lanes from source lines < 239 starting at x 0,
//     then line 0's lanes from source line 239 starting at x 0 (Mesen copies each line's first
//     pixel over pixel (0,0) before the line, so pixel (0,0) ends with line 239's).
//
// A reader gives pixel (x, y) its captured sprites, then the lanes of line y (240 for (0,0))
// covering x, in slot order, up to 4 in all. A line's NL lanes read in one clock, like
// hd3_fmap's sprite table, so RENDER can merge them on any line (spriteNearby).
//
// Record ids and addition lists come from hd3_dict; the rest of a line from the frame record
// in DDR3, read only as far as it is needed:
//   - a frame none of whose records has additions is skipped whole (a sweep of the records'
//     addition counts, one a clock): every list stays empty;
//   - every line: its header qword (1 burst), for the scroll t of process_bg and the span and
//     fetch counts; which fetches and spans are sources comes from hd3_dict (its fetch-valid
//     bit is the record's own);
//   - a line with a sprite source: the fetch area (4 bursts, X and attributes of every fetch,
//     for the ranks) and the pixel area (16 bursts, "sprites shown");
//   - a scanned line with a BG source: the pixel area (BG slot and column) and the span area
//     (9 bursts, CHR line addresses).
//
// Limits: NB lanes from one source line, NL on one target line (recorded frames: at most 8 on
// a line), and time: `abort` (raised by the stage-3 controller when RENDER needs the bank)
// stops issuing reads, waits for the outstanding bursts to return - a late beat must not land
// in the next frame's plan - and ends. Past any of these `ovf` is set with done: the frame is
// drawn without HD lookups (8o).

module hd3_add #(
	parameter integer RAW = 11,
	parameter integer NL  = 16,             // lanes a line holds (one-clock read width)
	parameter integer NB  = 64,             // lanes one source line can emit
	parameter [27:0]  REC_BASE = 28'hC000000
)
(
	input             clk,
	input             reset,

	input             start,            // pulse: build the lanes of frame-record bank `bank`
	input             bank,
	input             add_en,           // the pack has additions
	input             abort,            // out of time (RENDER needs the bank): end with ovf
	output reg        busy = 0,
	output reg        done = 0,         // pulse
	output reg        ovf = 0,          // the frame's lanes are incomplete (valid with done)

	// hd3_dict, bank `bank` (one clock latency)
	output      [7:0] r_y,
	output reg  [5:0] r_slot = 0,
	input     [RAW:0] r_span,           // {valid, id}
	output reg  [3:0] r_fetch = 0,
	input     [RAW:0] r_spr,            // {valid, id}
	output reg [RAW-1:0] r_rid = 0,
	input      [95:0] r_key,            // {palette, body}
	input     [RAW:0] r_nrec,           // records of bank `bank`
	output reg [RAW-1:0] a_rid = 0,
	input      [16:0] a_m,              // {count 7, start 10}
	output reg  [9:0] a_pi = 0,
	input  [47+RAW:0] a_pe,             // {file index 16, oy 16, ox 16, added id}

	// DDR3 read port: 8-qword bursts, data in issue order
	output reg        rd_req = 0,
	output reg [27:3] rd_addr = 0,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid,

	// reader: line 0..240 (240 = pixel (0,0)), one clock latency
	input       [7:0] rl_y,
	output reg  [6:0] rl_n = 0,         // lanes on the line
	output [NL*(RAW+21)-1:0] rl_q       // NL x lane, slot 0 at the bottom
);

// lane: {from source line 239, bgp, vm, hm, row 3, column of x0 3, added id, length-1 3, x0 8}
localparam integer LW = RAW + 21;
localparam integer KW = 17 + 3 + 16;     // sort key: {-dx + 2^16, k, addition index}
localparam integer BW = KW + 8 + LW;     // buffer entry: {key, target line, lane}
localparam integer LS = $clog2(NL);

reg [8:0] y = 0;
assign r_y = y[7:0];

// ------------------------------------------------------------------------------------------
// per-line lane tables: NL slots, a whole line read in one clock; lane counts
// ------------------------------------------------------------------------------------------
reg  [7:0] lw_a = 0;
reg [LW-1:0] lw_d = 0;
reg [NL-1:0] lw_e = 0;
reg  [7:0] lr_a = 0;
wire [7:0] l_ra = busy ? lr_a : rl_y;
reg [NL*LW-1:0] lq;
assign rl_q = lq;
genvar gi;
generate for (gi = 0; gi < NL; gi = gi + 1) begin : lane
	reg [LW-1:0] m [256];
	always @(posedge clk) begin
		if (lw_e[gi]) m[lw_a] <= lw_d;
		lq[LW * gi +: LW] <= m[l_ra];
	end
end endgenerate

reg  [6:0] cnt [256];
reg  [7:0] cw_a = 0, cr_a = 0;
reg  [6:0] cw_d = 0, cr_q = 0;
reg        cw_e = 0;
always @(posedge clk) begin
	if (cw_e) cnt[cw_a] <= cw_d;
	cr_q <= cnt[cr_a];
	rl_n <= cnt[rl_y];
end

// ------------------------------------------------------------------------------------------
// one source line's lanes, sorted at the end of the line
// ------------------------------------------------------------------------------------------
reg [BW-1:0] lbuf [NB];
reg  [6:0] bw_a = 0, br_a = 0;
reg [BW-1:0] bw_d = 0, br_q = 0;
reg        bw_e = 0;
always @(posedge clk) begin
	if (bw_e) lbuf[bw_a[$clog2(NB)-1:0]] <= bw_d;
	br_q <= lbuf[br_a[$clog2(NB)-1:0]];
end

// the line's pixel area: per pixel pair {col1 3, shown1, slot1 6, col0 3, shown0, slot0 6}
reg [19:0] pm [128];
reg  [6:0] pmw_a = 0, pmr_a = 0;
reg [19:0] pmw_d = 0, pmr_q = 0;
reg        pmw_e = 0;
always @(posedge clk) begin
	if (pmw_e) pm[pmw_a] <= pmw_d;
	pmr_q <= pm[pmr_a];
end

// ------------------------------------------------------------------------------------------
// the line read from DDR3
// ------------------------------------------------------------------------------------------
reg [63:0] q0 = 0;                       // line header qword 0
reg [15:0] fv = 0;                       // fetch valid
reg  [7:0] fx [16];                      // fetch X
reg  [2:0] fa [16];                      // fetch {vm, hm, bgp}
reg  [2:0] frow [16];                    // fetch tile row
reg [21:0] slin [34];                    // span CHR line address (low 3 bits = row)

// burst plan: phase 0 = the header (1 burst); phase 1 = bursts b0..nbur-1 of
// {fetch area 0..3, pixel area 4..19, span area 20..28}
reg        ph = 0;
reg  [4:0] nbur = 0;                     // end of this phase's bursts
reg  [4:0] bi = 0;                       // next to issue
reg  [4:0] bj = 0;                       // next to complete
reg  [2:0] bb = 0;                       // beat within the returning burst
reg  [3:0] outst = 0;
wire [24:0] rbase = REC_BASE[27:3] + {7'd0, bank, 17'd0};
wire [24:0] hbase = rbase + 25'h08000 + {9'd0, y[7:0], 8'd0};
wire [24:0] pbase = rbase + {10'd0, y[7:0], 7'd0};
wire  [4:0] bi_p = bi - 5'd4;            // pixel burst
wire  [4:0] bi_s = bi - 5'd20;           // span burst
wire  [4:0] bj_p = bj - 5'd4;
wire  [4:0] bj_s = bj - 5'd20;
wire [24:0] i_addr = !ph       ? hbase
                   : bi < 5'd4  ? hbase + 25'd80 + {17'd0, bi[3:0], 3'd0}
                   : bi < 5'd20 ? pbase + {17'd0, bi_p[3:0], 3'd0}
                   :              hbase + 25'd8 + {17'd0, bi_s[3:0], 3'd0};

wire       fetching = (bj != nbur);
// DDR3 plan requests from the state machine (declared before use: Quartus / ModelSim require it)
reg        st_plan = 0, st_ph = 0, st_issue = 0;
reg  [4:0] st_nb = 0, st_b0 = 0;
always @(posedge clk) begin
	pmw_e <= 0;
	if (rd_req && rd_ack) rd_req <= 0;
	if (fetching && bi != nbur && (!rd_req || rd_ack) && outst < 4'd8 && st_issue) begin
		rd_req  <= 1;
		rd_addr <= i_addr;
		bi      <= bi + 5'd1;
	end
	if (rd_valid) begin
		bb <= bb + 3'd1;
		if (!ph) begin
			if (bb == 3'd0) q0 <= rd_data;
		end else if (bj < 5'd4) begin
			if (!bb[0]) begin
				// fetch bj * 4 + bb / 2: its qa
				fv[{bj[1:0], bb[2:1]}]   <= rd_data[54];
				fx[{bj[1:0], bb[2:1]}]   <= rd_data[45:38];
				fa[{bj[1:0], bb[2:1]}]   <= {rd_data[53], rd_data[52], rd_data[51]};
				frow[{bj[1:0], bb[2:1]}] <= rd_data[18:16];
			end
		end else if (bj < 5'd20) begin
			pmw_e <= 1;
			pmw_a <= {bj_p[3:0], bb};
			pmw_d <= {rd_data[44:42], rd_data[38:32], rd_data[12:10], rd_data[6:0]};
		end else if (!bb[0] && {bj_s[3:0], bb[2:1]} < 6'd34) begin
			slin[{bj_s[3:0], bb[2:1]}] <= rd_data[21:0];
		end
		if (bb == 3'd7) bj <= bj + 5'd1;
	end
	case ({rd_req && rd_ack, rd_valid && bb == 3'd7})
	2'b10: outst <= outst + 4'd1;
	2'b01: outst <= outst - 4'd1;
	default: ;
	endcase
	if (st_plan) begin
		ph <= st_ph; nbur <= st_nb; bi <= st_b0; bj <= st_b0; bb <= 0;
		if (!st_ph) fv <= 0;                 // a new line: no fetch until its area is read
	end
	if (reset) begin
		rd_req <= 0; outst <= 0; bi <= 0; bj <= 0; nbur <= 0;
	end
end

// ------------------------------------------------------------------------------------------
// state
// ------------------------------------------------------------------------------------------
localparam [5:0]
	S_IDLE = 0,  S_CLR = 1,   S_LINE = 2,  S_HDR = 3,   S_LT = 4,
	S_SF = 5,    S_SFW = 6,   S_SFR = 7,   S_SFA = 8,   S_SFM = 9,
	S_BG = 10,   S_BGW = 11,  S_BGR = 12,  S_BGA = 13,  S_BGM = 14,  S_PIX = 15,
	S_SJ = 16,   S_PX = 17,   S_PXW = 18,  S_PXR = 19,  S_RUNS = 20,
	S_ENT = 21,  S_ENTW = 22, S_LANE = 23,
	S_BPX = 24,  S_BPXW = 25, S_BPXR = 26, S_BRUN = 27,
	S_SORT = 28, S_SC = 29,   S_SCL = 30,  S_AP0 = 31,  S_AP1 = 32,  S_AP2 = 33,
	S_V0 = 34,   S_V1 = 35,   S_V2 = 36,   S_V3 = 37,   S_FIN = 38,  S_PRE = 39,
	S_ABT = 40;
reg  [5:0] st = S_IDLE;
reg  [5:0] ret = S_IDLE;

// (DDR3 plan requests from the state machine: declared above the fetch block, which reads them)

// the frame sweep: does any record have additions? (a_m arrives two clocks after a_rid)
reg [RAW:0] pr_i = 0;
reg        pr_v1 = 0, pr_v2 = 0, pr_hit = 0;

reg  [8:0] cy = 0;
reg  [4:0] nfet = 0;
reg  [5:0] nspan = 0;
reg  [2:0] ys3 = 0;
reg [14:0] tmpv = 0;
reg        pbg = 0, scan_bg = 0;

// sprite sources: per fetch, its record's addition list (count 0 = not a source)
reg [16:0] fam [16];
reg        need_px = 0;
// BG sources: per span
reg [16:0] am [34];
reg [31:0] spal [34];
reg  [5:0] sj = 0;
reg        any_bg = 0, bg_hit = 0;

// the run being emitted
reg  [6:0] src_cnt = 0;
reg  [9:0] src_start = 0;
reg  [2:0] src_row = 0, src_k = 0;
reg        src_hm = 0, src_vm = 0, src_bgp = 0;
reg  [7:0] run_x0 = 0;
reg  [3:0] run_n = 0;
reg  [2:0] run_c0 = 0;
reg  [6:0] ei = 0;

// sprite runs of fetch j
reg  [4:0] j = 0;
reg  [3:0] xi = 0;
reg  [7:0] rx0 [8];
reg  [3:0] rn  [8];
reg  [2:0] rk  [8];
reg  [2:0] rc0 [8];
reg  [3:0] nruns = 0, ri = 0;
reg        open = 0;
wire [8:0] xcur = {1'b0, fx[j[3:0]]} + {5'd0, xi};
// fx goes into rank() as a packed vector: Quartus 17 (VRFX) crashes elaborating a memory read
// inside a function ("read to RAM wasn't mapped to a specific read port", 2026-10-21)
wire [127:0] fx_v;
generate for (gi = 0; gi < 16; gi = gi + 1) begin : fxp
	assign fx_v[8 * gi +: 8] = fx[gi];
end endgenerate
function [4:0] rank(input [15:0] v, input [4:0] jj, input [8:0] x, input [127:0] fxs);
	integer i;
	reg [8:0] xs;
	begin
		rank = 0;
		for (i = 0; i < 16; i = i + 1)
			if (i < jj && v[i]) begin
				xs = {1'b0, fxs[8 * i +: 8]};
				if (x >= xs && x <= xs + 9'd7) rank = rank + 5'd1;
			end
	end
endfunction
wire [4:0] rk_cur = rank(fv, j, xcur, fx_v);
wire       shown  = xcur[0] ? pmr_q[16] : pmr_q[6];

// BG runs
reg  [8:0] bx = 0;
reg        bopen = 0, b_pend = 0;
reg  [7:0] b_x0 = 0;
reg  [3:0] b_n = 0;
reg  [5:0] b_sl = 0, p_sl = 0;
reg  [2:0] b_c0 = 0, b_cl = 0, p_c = 0;
wire [5:0] q_sl = bx[0] ? pmr_q[15:10] : pmr_q[5:0];
wire [2:0] q_c  = bx[0] ? pmr_q[19:17] : pmr_q[9:7];
wire       q_src = q_sl != 6'd63 && q_sl < nspan && am[q_sl][16:10] != 7'd0;
// same Mesen tile: CHR index (address bits 19:4), palette, row
wire       q_same = q_sl == b_sl || (spal[q_sl] == spal[b_sl] &&
                    slin[q_sl][19:4] == slin[b_sl][19:4] && slin[q_sl][2:0] == slin[b_sl][2:0]);
// the column must step by one WITHOUT wrapping: 7 -> 0 is the next tile, a new run (a run is
// at most one tile wide, which is also what its 3-bit length holds)
wire       q_ext = bopen && b_cl != 3'd7 && q_c == b_cl + 3'd1 && q_same;

// lane from the current run and entry
wire [15:0] e_idx = a_pe[47+RAW:32+RAW];
wire [15:0] e_oy  = a_pe[31+RAW:16+RAW];
wire [15:0] e_ox  = a_pe[15+RAW:RAW];
wire [RAW-1:0] e_aid = a_pe[RAW-1:0];
wire signed [17:0] ox_s = {{2{e_ox[15]}}, e_ox};
wire signed [17:0] oy_s = {{2{e_oy[15]}}, e_oy};
wire signed [17:0] dxs  = src_hm ? -ox_s : ox_s;
wire signed [17:0] dys  = src_vm ? -oy_s : oy_s;
wire signed [17:0] tys  = $signed({9'd0, y}) + dys;
wire signed [17:0] txs  = $signed({10'd0, run_x0}) + dxs;
wire signed [17:0] txe  = txs + $signed({14'd0, run_n}) - 18'sd1;
wire        lane_ok = tys >= 0 && tys < 18'sd240 && txe >= 0 && txs < 18'sd256;
wire  [7:0] l_x0  = (txs < 0) ? 8'd0 : txs[7:0];
wire  [7:0] l_xe  = (txe > 18'sd255) ? 8'd255 : txe[7:0];
wire  [7:0] l_dn  = l_xe - l_x0;
wire  [7:0] l_dc  = l_x0 - txs[7:0];
wire  [2:0] l_c0  = run_c0 + l_dc[2:0];
wire [17:0] l_ndx = 18'sd65536 - dxs;
wire [LW-1:0] lane_v = {y == 9'd239, src_bgp, src_vm, src_hm, src_row, l_c0, e_aid,
                        l_dn[2:0], l_x0};

// sort
reg  [6:0] nl = 0, emitted = 0;
reg [NB-1:0] used = 0;
reg  [6:0] si = 0, sp = 0, best = 0;
reg        sv = 0, hb = 0;
reg [BW-1:0] be = 0;
wire [KW-1:0] bk = be[BW-1 -: KW];
wire [KW-1:0] ck = br_q[BW-1 -: KW];

// line 240
reg  [6:0] vs = 0, vn = 0, v240 = 0;
reg        vph = 0;
wire [LW-1:0] vlane = lq[LW * vs[LS-1:0] +: LW];

always @(posedge clk) begin
	done    <= 0;
	lw_e    <= 0;
	cw_e    <= 0;
	bw_e    <= 0;
	st_plan <= 0;

	case (st)
	S_IDLE: if (start) begin
		busy <= 1;
		ovf  <= 0;
		cy   <= 0;
		st   <= S_CLR;
	end

	// empty every line's list (lines 0..240)
	S_CLR: begin
		cw_e <= 1; cw_a <= cy[7:0]; cw_d <= 0;
		cy <= cy + 9'd1;
		if (cy == 9'd240) begin
			y      <= 0;
			pr_i   <= 0;
			pr_v1  <= 0;
			pr_v2  <= 0;
			pr_hit <= 0;
			st     <= add_en ? S_PRE : S_FIN;
		end
	end

	// any record of the frame with additions? one a clock; none: every list stays empty
	S_PRE: begin
		pr_v1 <= 0;
		if (pr_i < r_nrec) begin
			a_rid <= pr_i[RAW-1:0];
			pr_v1 <= 1;
			pr_i  <= pr_i + 1'd1;
		end
		pr_v2 <= pr_v1;
		if (pr_v2 && a_m[16:10] != 7'd0) pr_hit <= 1;
		if (pr_hit) st <= S_LINE;                // found one: build the lanes
		else if (pr_i == r_nrec && !pr_v1 && !pr_v2) st <= S_FIN;
	end

	// ---- read the line header ----
	S_LINE: begin
		if (y == 9'd240) st <= S_V0;
		else begin
			st_plan  <= 1; st_ph <= 0; st_b0 <= 5'd0; st_nb <= 5'd1;
			st_issue <= 1;
			st       <= S_HDR;
		end
	end
	S_HDR: if (!st_plan && bj == nbur) begin
		st_issue <= 0;
		st       <= S_LT;
	end
	S_LT: begin
		nfet  <= (q0[55:48] > 8'd16) ? 5'd16 : q0[52:48];
		nspan <= (q0[39:32] > 8'd34) ? 6'd34 : q0[37:32];
		// Mesen's process_bg: the tile row starts, the scroll changed, or the previous
		// scanned line had a BG tile with additions (line 0 always scans)
		if (y == 0 || (ys3 + y[2:0]) == 3'd0 || tmpv != q0[14:0]) begin
			ys3     <= q0[14:12];
			tmpv    <= q0[14:0];
			scan_bg <= 1;
		end else
			scan_bg <= pbg;
		pbg     <= 0;
		nl      <= 0;
		emitted <= 0;
		used    <= 0;
		need_px <= 0;
		any_bg  <= 0;
		bg_hit  <= 0;
		j       <= 0;
		st      <= S_SF;
	end

	// ---- which fetches are sources ----
	// (an invalid fetch has no record: hd3_dict's valid bit is the fetch's own)
	S_SF: begin
		if (j == nfet) begin
			sj <= 0;
			st <= scan_bg ? S_BG : S_PIX;
		end else begin
			r_fetch <= j[3:0];
			st      <= S_SFW;
		end
	end
	S_SFW: st <= S_SFR;
	S_SFR: begin
		if (!r_spr[RAW]) begin
			fam[j[3:0]] <= 0;
			j  <= j + 5'd1;
			st <= S_SF;
		end else begin
			a_rid <= r_spr[RAW-1:0];
			st    <= S_SFA;
		end
	end
	S_SFA: st <= S_SFM;
	S_SFM: begin
		fam[j[3:0]] <= a_m;
		if (a_m[16:10] != 7'd0) need_px <= 1;
		j  <= j + 5'd1;
		st <= S_SF;
	end

	// ---- which spans are sources (scanned lines) ----
	S_BG: begin
		if (sj == nspan) st <= S_PIX;
		else begin
			r_slot <= sj;
			st     <= S_BGW;
		end
	end
	S_BGW: st <= S_BGR;
	S_BGR: begin
		if (!r_span[RAW]) begin
			am[sj] <= 0;
			sj <= sj + 6'd1;
			st <= S_BG;
		end else begin
			a_rid <= r_span[RAW-1:0];
			r_rid <= r_span[RAW-1:0];
			st    <= S_BGA;
		end
	end
	S_BGA: st <= S_BGM;
	S_BGM: begin
		am[sj]   <= a_m;
		spal[sj] <= r_key[95:64];
		if (a_m[16:10] != 7'd0) any_bg <= 1;
		sj <= sj + 6'd1;
		st <= S_BG;
	end

	// ---- only on a line with a source: the fetch area (sprite sources), the pixel area, and
	// the span area (BG sources) ----
	S_PIX: begin
		if (!need_px && !any_bg)
			st <= S_SORT;
		else begin
			st_plan  <= 1; st_ph <= 1;
			st_b0    <= need_px ? 5'd0 : 5'd4;
			st_nb    <= any_bg ? 5'd29 : 5'd20;
			st_issue <= 1;
			st       <= S_SJ;
		end
	end
	S_SJ: if (!st_plan && bj == nbur && !pmw_e) begin
		st_issue <= 0;
		ri       <= 0;
		nruns    <= 0;
		// j = -1: S_RUNS (ri == nruns) advances to fetch 0
		j        <= 5'd31;
		st       <= S_RUNS;
	end

	// ---- sprite sources: fetch j's members, then its runs ----
	S_RUNS: begin
		if (ri != nruns) begin
			run_x0 <= rx0[ri[2:0]];
			run_n  <= rn[ri[2:0]];
			src_k  <= rk[ri[2:0]];
			run_c0 <= rc0[ri[2:0]];
			ei     <= 0;
			ret    <= S_RUNS;
			ri     <= ri + 4'd1;
			st     <= S_ENT;
		end else if (j + 5'd1 >= nfet) begin          // (j = 31 wraps to 0)
			// sprites done: BG runs on a scanned line with BG sources
			bx     <= 0;
			bopen  <= 0;
			b_pend <= 0;
			st     <= any_bg ? S_BPX : S_SORT;
		end else begin
			j <= j + 5'd1;
			if (fam[j[3:0] + 4'd1][16:10] == 7'd0)
				st <= S_RUNS;                    // not a source: next fetch (ri == nruns)
			else begin
				src_cnt   <= fam[j[3:0] + 4'd1][16:10];
				src_start <= fam[j[3:0] + 4'd1][9:0];
				src_row   <= frow[j[3:0] + 4'd1];
				src_vm    <= fa[j[3:0] + 4'd1][2];
				src_hm    <= fa[j[3:0] + 4'd1][1];
				src_bgp   <= fa[j[3:0] + 4'd1][0];
				xi        <= 0;
				nruns     <= 0;
				ri        <= 0;
				open      <= 0;
				st        <= S_PX;
			end
		end
	end
	S_PX: begin
		if (xi == 4'd8 || xcur > 9'd255) st <= S_RUNS;
		else begin
			pmr_a <= xcur[7:1];
			st    <= S_PXW;
		end
	end
	S_PXW: st <= S_PXR;
	S_PXR: begin
		if (shown && rk_cur < 5'd4) begin
			if (open && rk[nruns[2:0] - 3'd1] == rk_cur[2:0])
				rn[nruns[2:0] - 3'd1] <= rn[nruns[2:0] - 3'd1] + 4'd1;
			else begin
				rx0[nruns[2:0]] <= xcur[7:0];
				rn[nruns[2:0]]  <= 4'd1;
				rk[nruns[2:0]]  <= rk_cur[2:0];
				rc0[nruns[2:0]] <= xi[2:0];
				nruns <= nruns + 4'd1;
			end
			open <= 1;
		end else
			open <= 0;
		xi <= xi + 4'd1;
		st <= S_PX;
	end

	// ---- every addition of the run's record (file order): a lane each ----
	S_ENT: begin
		if (ei == src_cnt) st <= ret;
		else begin
			a_pi <= src_start + {3'd0, ei};
			st   <= S_ENTW;
		end
	end
	S_ENTW: st <= S_LANE;
	S_LANE: begin
		if (lane_ok) begin
			if (nl == NB[6:0]) ovf <= 1;
			else begin
				bw_e <= 1;
				bw_a <= nl;
				bw_d <= {l_ndx[16:0], src_k, e_idx, tys[7:0], lane_v};
				nl   <= nl + 7'd1;
			end
		end
		ei <= ei + 7'd1;
		st <= S_ENT;
	end

	// ---- BG sources: pixels whose slot is a span with additions ----
	S_BPX: begin
		if (b_pend) begin
			// the pixel that ended the emitted run starts its own
			b_pend <= 0;
			bopen  <= 1;
			b_x0   <= bx[7:0];
			b_n    <= 4'd1;
			b_sl   <= p_sl;
			b_c0   <= p_c;
			b_cl   <= p_c;
			bx     <= bx + 9'd1;
		end else if (bx == 9'd256) begin
			if (bopen) begin
				bopen <= 0;
				ret   <= S_BPX;
				st    <= S_BRUN;
			end else begin
				pbg <= bg_hit;
				st  <= S_SORT;
			end
		end else begin
			pmr_a <= bx[7:1];
			st    <= S_BPXW;
		end
	end
	S_BPXW: st <= S_BPXR;
	S_BPXR: begin
		if (q_src) begin
			bg_hit <= 1;
			if (q_ext) begin
				b_n  <= b_n + 4'd1;
				b_cl <= q_c;
				bx   <= bx + 9'd1;
				st   <= S_BPX;
			end else if (bopen) begin
				// this pixel starts a new run once the open one is emitted
				bopen  <= 0;
				b_pend <= 1;
				p_sl   <= q_sl;
				p_c    <= q_c;
				st     <= S_BRUN;
			end else begin
				bopen <= 1;
				b_x0  <= bx[7:0];
				b_n   <= 4'd1;
				b_sl  <= q_sl;
				b_c0  <= q_c;
				b_cl  <= q_c;
				bx    <= bx + 9'd1;
				st    <= S_BPX;
			end
		end else if (bopen) begin
			bopen <= 0;
			bx    <= bx + 9'd1;
			st    <= S_BRUN;
		end else begin
			bx <= bx + 9'd1;
			st <= S_BPX;
		end
	end
	S_BRUN: begin
		src_cnt   <= am[b_sl][16:10];
		src_start <= am[b_sl][9:0];
		src_row   <= slin[b_sl][2:0];
		src_k     <= 3'd4;
		src_hm    <= 0;
		src_vm    <= 0;
		src_bgp   <= 0;
		run_x0    <= b_x0;
		run_n     <= b_n;
		run_c0    <= b_c0;
		ei        <= 0;
		ret       <= S_BPX;
		st        <= S_ENT;
	end

	// ---- the line's lanes, smallest key first, appended to their target lines ----
	S_SORT: begin
		if (emitted == nl) begin
			y  <= y + 9'd1;
			st <= S_LINE;
		end else begin
			si <= 0; sv <= 0; hb <= 0;
			br_a <= 0;
			st <= S_SC;
		end
	end
	// one entry a clock: br_q holds entry sp while entry si + 1 is read
	S_SC: begin
		if (sv && !used[sp[$clog2(NB)-1:0]] && (!hb || ck < bk)) begin
			hb   <= 1;
			best <= sp;
			be   <= br_q;
		end
		if (si == nl) begin
			if (!sv) st <= S_SCL;
			sv <= 0;
		end else begin
			br_a <= si + 7'd1;
			sp   <= si;
			sv   <= 1;
			si   <= si + 7'd1;
		end
	end
	S_SCL: begin
		used[best[$clog2(NB)-1:0]] <= 1;
		emitted <= emitted + 7'd1;
		cr_a    <= be[LW+7:LW];
		st      <= S_AP0;
	end
	S_AP0: st <= S_AP1;
	S_AP1: begin
		if (cr_q == NL[6:0]) ovf <= 1;
		else begin
			lw_e[cr_q[LS-1:0]] <= 1;
			lw_a <= be[LW+7:LW];
			lw_d <= be[LW-1:0];
			cw_e <= 1;
			cw_a <= be[LW+7:LW];
			cw_d <= cr_q + 7'd1;
		end
		st <= S_AP2;
	end
	S_AP2: st <= S_SORT;                     // the count write lands before the next read

	// ---- line 240: pixel (0,0) ----
	S_V0: begin
		vph  <= 0;
		v240 <= 0;
		lr_a <= 8'd239;
		cr_a <= 8'd239;
		st   <= S_V1;
	end
	S_V1: st <= S_V2;
	S_V2: begin
		vn <= cr_q;
		vs <= 0;
		st <= S_V3;
	end
	S_V3: begin
		if (vs == vn) begin
			if (vph) begin
				cw_e <= 1; cw_a <= 8'd240; cw_d <= v240;
				st   <= S_FIN;
			end else begin
				vph  <= 1;
				lr_a <= 8'd0;
				cr_a <= 8'd0;
				st   <= S_V1;
			end
		end else begin
			// line 239: lanes from source lines < 239; line 0: lanes from source line 239;
			// both starting at x 0
			if (vlane[7:0] == 8'd0 && vlane[LW-1] == vph) begin
				if (v240 == NL[6:0]) ovf <= 1;
				else begin
					lw_e[v240[LS-1:0]] <= 1;
					lw_a <= 8'd240;
					lw_d <= vlane;
					v240 <= v240 + 7'd1;
				end
			end
			vs <= vs + 7'd1;
		end
	end

	S_FIN: begin
		busy <= 0;
		done <= 1;
		st   <= S_IDLE;
	end

	// out of time: no new reads; finish once the outstanding bursts have returned
	S_ABT: if (!rd_req && outst == 4'd0 && !st_plan) st <= S_FIN;

	default: st <= S_IDLE;
	endcase

	if (busy && abort && st != S_ABT && st != S_FIN) begin
		ovf      <= 1;
		st_issue <= 0;
		st       <= S_ABT;
	end

	if (reset) begin
		st       <= S_IDLE;
		busy     <= 0;
		ovf      <= 0;
		st_issue <= 0;
	end
end

endmodule
