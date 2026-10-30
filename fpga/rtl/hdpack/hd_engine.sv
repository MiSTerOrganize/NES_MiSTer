// hd_engine.sv - Mesen HD pack renderer for the NES core (test version, 2x, CHR-ROM only).
//
// Everything runs on the core clock (21.477 MHz). Per NES line, three stages run in parallel
// on three different lines:
//   BUILD  line L+1 : the PPU tap writes the BG tile table, sprite table and pixel descriptors
//   FETCH  line L   : look every tile up in the hash table, burst-read hits from the DDR3 atlas
//   OUTPUT line L-1 : two 512-pixel output lines (31 kHz), composited in Mesen's draw order
// A FETCH that is not finished when the next line starts is abandoned: its unfetched tiles
// simply draw as the original NES pixels.  Nothing here can stall the NES.
//
// Tap bus layout (from rtl/ppu.sv, LSB first):
//   [0] bg_we  [6:1] bg_slot  [28:7] bg_lin  [30:29] bg_pal
//   [31] spr_we  [34:32] spr_idx  [89:35] spr_data {valid, attr[7:0], x[7:0], lin[21:0], pix1, pix2}
//   [90] px_we  [98:91] px_x  [104:99] px_slot  [107:105] px_fx  [111:108] px_bgc {pal, cidx}
//   [112] px_bg_on  [113] px_spr_on  [114] line_done  [123:115] line_y  [315:124] palette RAM
//   (bits 351:316 carry stage-2 fields used only by rtl/hdpack/hd_stage2.sv; this module takes 319:0)
//
// Pack format, hash function and entry layout: tools/hdp_convert.py (must match exactly).

module hd_engine #(parameter [27:0] ATLAS_BASE = 28'h8000000)  // byte offset in the 0x30000000 window
(
	input             clk,
	input             reset,
	input             enable,          // OSD "HD Pack" on

	input     [319:0] tap,

	// pack loading (OSD file slot 4)
	input             load_active,
	input             load_wr,
	input      [24:0] load_addr,
	input       [7:0] load_data,
	output            load_wait,

	// DDR3: 16-qword burst reads
	output reg        rd_req = 0,
	output reg [27:3] rd_addr,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid,
	// DDR3: single-qword writes (pack loading only)
	output reg        wr_req = 0,
	output reg [27:3] wr_addr,
	output reg [63:0] wr_data,
	input             wr_ack,

	output            active,          // HD picture valid: pack loaded and option on
	output reg  [7:0] R,
	output reg  [7:0] G,
	output reg  [7:0] B,
	output reg        HS,
	output reg        VS,
	output reg        HB,
	output reg        VB
);

localparam MAXPROBE = 4'd15;

// ------------------------------------------------------------------------------------------
// Tap unpack
// ------------------------------------------------------------------------------------------
wire        t_bg_we     = tap[0];
wire  [5:0] t_bg_slot   = tap[6:1];
wire [21:0] t_bg_lin    = tap[28:7];
wire  [1:0] t_bg_pal    = tap[30:29];
wire        t_spr_we    = tap[31];
wire  [2:0] t_spr_idx   = tap[34:32];
wire [54:0] t_spr_data  = tap[89:35];
wire        t_px_we     = tap[90];
wire  [7:0] t_px_x      = tap[98:91];
wire  [5:0] t_px_slot   = tap[104:99];
wire  [2:0] t_px_fx     = tap[107:105];
wire  [3:0] t_px_bgc    = tap[111:108];
wire        t_px_bg_on  = tap[112];
wire        t_px_spr_on = tap[113];
wire        t_line_done = tap[114];
wire  [8:0] t_line_y    = tap[123:115];
wire [191:0] t_pal      = tap[315:124];

// ------------------------------------------------------------------------------------------
// Pack loader
// ------------------------------------------------------------------------------------------
reg         pack_ok = 0;
reg   [3:0] magic;
reg   [7:0] hdr_ver, hdr_scale, hdr_flags;
reg         legacy_key = 0;   // header flag bit 0: pack version < 100 keys (no colour 0, no sprite kind)
reg  [63:0] qasm;
reg  [23:0] pasm;
reg         old_load;

reg  [23:0] sdpal [64];
reg         sdpal_we;
reg   [5:0] sdpal_wa;
reg  [23:0] sdpal_wd;

reg  [63:0] htab [8192];
reg         ht_we;
reg  [12:0] ht_wa;
reg  [63:0] ht_wd;

wire [24:0] pal_off   = load_addr - 25'h40;
wire [24:0] tab_off   = load_addr - 25'h140;
wire [24:0] atlas_off = load_addr - 25'h10400;
wire [63:0] qword     = {load_data, qasm[63:8]};

assign load_wait = wr_req;

always @(posedge clk) begin
	sdpal_we <= 0;
	ht_we    <= 0;
	old_load <= load_active;

	if (wr_ack) wr_req <= 0;

	if (load_active & ~old_load) begin
		pack_ok <= 0;
		magic   <= 0;
	end

	if (load_active & load_wr) begin
		qasm <= qword;
		case (load_addr)
			25'd0: magic[0] <= (load_data == "N");
			25'd1: magic[1] <= (load_data == "H");
			25'd2: magic[2] <= (load_data == "D");
			25'd3: magic[3] <= (load_data == "P");
			25'd4: hdr_ver   <= load_data;
			25'd6: hdr_scale <= load_data;
			25'd7: hdr_flags <= load_data;
			default: ;
		endcase

		if (load_addr >= 25'h40 && load_addr < 25'h140) begin
			case (pal_off[1:0])
				2'd0: pasm[23:16] <= load_data;
				2'd1: pasm[15:8]  <= load_data;
				2'd2: begin
					sdpal_we <= 1;
					sdpal_wa <= pal_off[7:2];
					sdpal_wd <= {pasm[23:8], load_data};
				end
				default: ;
			endcase
		end

		if (load_addr >= 25'h140 && load_addr < 25'h10140 && tab_off[2:0] == 3'd7) begin
			ht_we <= 1;
			ht_wa <= tab_off[15:3];
			ht_wd <= qword;
		end

		if (load_addr >= 25'h10400 && atlas_off[2:0] == 3'd7) begin
			wr_req  <= 1;
			wr_addr <= ATLAS_BASE[27:3] + atlas_off[24:3];
			wr_data <= qword;
		end
	end

	if (~load_active & old_load) begin
		pack_ok    <= (&magic) && hdr_ver == 8'd2 && hdr_scale == 8'd2 && hdr_flags[7:1] == 7'd0;
		legacy_key <= hdr_flags[0];
	end

	if (reset) wr_req <= 0;
end

always @(posedge clk) if (sdpal_we) sdpal[sdpal_wa] <= sdpal_wd;
always @(posedge clk) if (ht_we)    htab[ht_wa]     <= ht_wd;

assign active = enable & pack_ok;

// ------------------------------------------------------------------------------------------
// BUILD: line buffers written by the tap
// ------------------------------------------------------------------------------------------
reg  [1:0] bbuf = 2'd0, jbuf = 2'd1, obuf = 2'd2;
reg        jpar = 0, opar = 1;
reg  [6:0] build_n = 0;                 // BG tiles written for the line being built

reg [23:0] bgtab [256];             // {buf, slot} -> {lin[21:0], pal[1:0]}
always @(posedge clk) if (t_bg_we) bgtab[{bbuf, t_bg_slot}] <= {t_bg_lin, t_bg_pal};

reg [15:0] pixd [1024];             // {buf, x} -> {spr_on, bg_on, bgc[3:0], fx[2:0], slot[5:0]}
always @(posedge clk) if (t_px_we) pixd[{bbuf, t_px_x}] <= {1'b0, t_px_spr_on, t_px_bg_on, t_px_bgc, t_px_fx, t_px_slot};

reg [54:0] build_spr [8];
always @(posedge clk) if (t_spr_we) build_spr[t_spr_idx] <= t_spr_data;

// per-line state handed down the pipeline at every line_done
reg [54:0] job_spr [8], out_spr [8];
reg [191:0] job_pal, out_pal;
reg  [8:0] job_y = 9'd511, out_y = 9'd511;
reg  [6:0] job_n;
reg        job_start;
reg  [1:0] job_tag = 0;

integer k;

always @(posedge clk) begin
	job_start <= 0;
	if (t_bg_we && {1'b0, t_bg_slot} >= build_n) build_n <= {1'b0, t_bg_slot} + 7'd1;

	if (t_line_done) begin
		obuf <= jbuf;  jbuf <= bbuf;  bbuf <= obuf;
		opar <= jpar;  jpar <= ~jpar;
		job_n   <= (t_bg_we && {1'b0, t_bg_slot} >= build_n) ? {1'b0, t_bg_slot} + 7'd1 : build_n;
		build_n <= 0;
		job_pal <= t_pal;   out_pal <= job_pal;
		job_y   <= t_line_y; out_y  <= job_y;
		for (k = 0; k < 8; k = k + 1) begin
			job_spr[k] <= build_spr[k];
			out_spr[k] <= job_spr[k];
		end
		job_start <= 1;
		job_tag   <= job_tag + 2'd1;
	end
end

// ------------------------------------------------------------------------------------------
// FETCH: hash lookups, then DDR3 burst reads into the line caches
// ------------------------------------------------------------------------------------------
reg [63:0] ht_q;
reg [12:0] ht_ra;
always @(posedge clk) ht_q <= htab[ht_ra];

reg [23:0] bg_q;
reg  [7:0] bg_ra;
always @(posedge clk) bg_q <= bgtab[bg_ra];

typedef enum bit [2:0] { F_IDLE, F_RD, F_WAIT, F_KEY, F_PROBE, F_PWAIT, F_PCHK } fstate_t;
fstate_t fs = F_IDLE;

reg  [6:0] fe;                      // entry: 0..job_n-1 = BG slot, 64..71 = sprite 0..7
reg        ftry;                    // 0 = exact key, 1 = default key
reg  [3:0] fprobe;
reg [17:0] f_tile;
reg  [1:0] f_kind;
reg [23:0] f_pal;
reg  [2:0] f_fy;
reg [12:0] f_hash;

wire f_is_spr = fe[6];
wire [54:0] f_spr = job_spr[fe[2:0]];

function [5:0] pal_entry(input [191:0] p, input [4:0] n);
	pal_entry = p[n*6 +: 6];
endfunction

// Packs older than version 100 key a tile on colours 1-3 only: no colour 0 for background
// tiles and no sprite marker (Mesen HdNesPpu.h). The converter sets header flag bit 0.
wire [1:0] spr_kind = legacy_key ? 2'd0 : 2'd1;
wire [5:0] bg_c0    = legacy_key ? 6'd0 : pal_entry(job_pal, 5'd0);

// CRC-13, poly 0x1CF5, init 0, MSB first, over {kind, pal, tile} (44 bits).
// Unrolled by synthesis into a pure XOR network. Must match tools/hdp_convert.py hash_key.
function [12:0] hd_hash(input [17:0] tile, input [1:0] kind, input [23:0] pal);
	reg [43:0] v;
	reg [12:0] r;
	reg        fb;
	integer    i;
	begin
		v = {kind, pal, tile};
		r = 13'd0;
		for (i = 43; i >= 0; i = i - 1) begin
			fb = r[12] ^ v[i];
			r  = {r[11:0], 1'b0};
			if (fb) r = r ^ 13'h1CF5;
		end
		hd_hash = r;
	end
endfunction

// request FIFO (lookup -> DDR issue)
reg [33:0] rq_mem [64];             // {tag[1:0], dest[6:0], qaddr[27:3]}
reg  [5:0] rq_wp = 0, rq_rp = 0;
wire       rq_empty = (rq_wp == rq_rp);
reg        rq_push;
reg [33:0] rq_din;
always @(posedge clk) if (rq_push) rq_mem[rq_wp] <= rq_din;

// in-flight FIFO (issued bursts awaiting data)
reg  [8:0] if_mem [8];              // {tag[1:0], dest[6:0]}
reg  [8:0] rd_info;                 // {tag, dest} of the request being presented
reg  [2:0] if_wp = 0, if_rp = 0;
reg  [3:0] beat = 0;

// hit flags per cache parity
reg [127:0] bghit = 0;              // {parity, slot}
reg  [15:0] sphit = 0;              // {parity, sprite}

// caches: BG {par, slot, row, pair} -> 2 pixels;  sprite k {par, row, pair} -> 2 pixels
reg [63:0] bgc [2048];
reg        bgc_we;
reg [10:0] bgc_wa;
reg [63:0] bgc_wd;
always @(posedge clk) if (bgc_we) bgc[bgc_wa] <= bgc_wd;

reg  [7:0] spc_we;
reg  [4:0] spc_wa;
reg [63:0] spc_wd;

always @(posedge clk) begin
	rq_push <= 0;
	bgc_we  <= 0;
	spc_we  <= 0;

	// ---- lookup state machine ----
	case (fs)
		F_IDLE: ;

		F_RD: begin
			if (!f_is_spr && fe >= job_n) begin
				fe <= 7'd64;                      // BG done, go to sprites
			end else if (f_is_spr && fe[3]) begin
				fs <= F_IDLE;                     // all 8 sprites done
			end else begin
				bg_ra <= {jbuf, fe[5:0]};
				fs    <= F_WAIT;
			end
		end

		F_WAIT: fs <= F_KEY;

		F_KEY: begin
			ftry   <= 0;
			fprobe <= 0;
			if (f_is_spr) begin
				f_tile <= {2'd0, f_spr[35:20]};
				f_fy   <= f_spr[18:16];
				f_kind <= spr_kind;
				f_pal  <= {6'd0,
				           pal_entry(job_pal, {1'b1, f_spr[47:46], 2'd1}),
				           pal_entry(job_pal, {1'b1, f_spr[47:46], 2'd2}),
				           pal_entry(job_pal, {1'b1, f_spr[47:46], 2'd3})};
				f_hash <= hd_hash({2'd0, f_spr[35:20]}, spr_kind,
				                  {6'd0,
				                   pal_entry(job_pal, {1'b1, f_spr[47:46], 2'd1}),
				                   pal_entry(job_pal, {1'b1, f_spr[47:46], 2'd2}),
				                   pal_entry(job_pal, {1'b1, f_spr[47:46], 2'd3})});
				fs <= (f_spr[54] && f_spr[37:36] == 2'b10) ? F_PROBE : F_RD;
				if (!(f_spr[54] && f_spr[37:36] == 2'b10)) fe <= fe + 7'd1;
			end else begin
				f_tile <= {2'd0, bg_q[21:6]};
				f_fy   <= bg_q[4:2];
				f_kind <= 2'd0;
				f_pal  <= {bg_c0,
				           pal_entry(job_pal, {1'b0, bg_q[1:0], 2'd1}),
				           pal_entry(job_pal, {1'b0, bg_q[1:0], 2'd2}),
				           pal_entry(job_pal, {1'b0, bg_q[1:0], 2'd3})};
				f_hash <= hd_hash({2'd0, bg_q[21:6]}, 2'd0,
				                  {bg_c0,
				                   pal_entry(job_pal, {1'b0, bg_q[1:0], 2'd1}),
				                   pal_entry(job_pal, {1'b0, bg_q[1:0], 2'd2}),
				                   pal_entry(job_pal, {1'b0, bg_q[1:0], 2'd3})});
				fs <= (bg_q[23:22] == 2'b10) ? F_PROBE : F_RD;
				if (bg_q[23:22] != 2'b10) fe <= fe + 7'd1;
			end
		end

		F_PROBE: begin
			ht_ra <= f_hash + {9'd0, fprobe};
			fs    <= F_PWAIT;
		end

		F_PWAIT: fs <= F_PCHK;

		F_PCHK: begin
			if (ht_q[63] && ht_q[17:0] == f_tile && ht_q[19:18] == f_kind && ht_q[43:20] == f_pal) begin
				// hit: queue the two needed rows (fy*2, fy*2+1) = 128 bytes = 16 qwords
				rq_push <= 1;
				rq_din  <= {job_tag, fe, ATLAS_BASE[27:3] + {ht_q[59:44], 7'd0} + {f_fy, 4'd0}};
				fe <= fe + 7'd1;
				fs <= F_RD;
			end else if (!ht_q[63] || fprobe == MAXPROBE) begin
				if (!ftry) begin
					ftry   <= 1;
					fprobe <= 0;
					f_kind <= 2'd2;
					f_pal  <= 24'd0;
					f_hash <= hd_hash(f_tile, 2'd2, 24'd0);
					fs     <= F_PROBE;
				end else begin
					fe <= fe + 7'd1;
					fs <= F_RD;
				end
			end else begin
				fprobe <= fprobe + 4'd1;
				fs     <= F_PROBE;
			end
		end

		default: fs <= F_IDLE;
	endcase

	if (rq_push) rq_wp <= rq_wp + 6'd1;

	// ---- new line: start a fresh job, abandon the old one ----
	if (job_start) begin
		fe    <= 0;
		fs    <= active ? F_RD : F_IDLE;
		rq_rp <= rq_wp + (rq_push ? 6'd1 : 6'd0);   // drop queued, unissued requests
		if (jpar) bghit[127:64] <= 0; else bghit[63:0] <= 0;
		if (jpar) sphit[15:8] <= 0;   else sphit[7:0]  <= 0;
	end

	// ---- issue DDR3 bursts ----
	// The queue entry is popped when the request is raised, and its {tag, dest} travel with the
	// request, so a job_start flush in between cannot mis-attribute the burst.
	if (rd_req) begin
		if (rd_ack) begin
			rd_req <= 0;
			if_mem[if_wp] <= rd_info;
			if_wp <= if_wp + 3'd1;
		end
	end else if (!rq_empty && !job_start && (if_wp - if_rp) < 3'd4) begin
		rd_req  <= 1;
		rd_addr <= rq_mem[rq_rp][24:0];
		rd_info <= rq_mem[rq_rp][33:25];
		rq_rp   <= rq_rp + 6'd1;
	end

	// ---- receive burst data into the caches ----
	if (rd_valid) begin
		beat <= beat + 4'd1;
		if (if_mem[if_rp][8:7] == job_tag) begin
			if (!if_mem[if_rp][6]) begin
				bgc_we <= 1;
				bgc_wa <= {jpar, if_mem[if_rp][5:0], beat[3], beat[2:0]};
				bgc_wd <= rd_data;
			end else begin
				spc_we[if_mem[if_rp][2:0]] <= 1;
				spc_wa <= {jpar, beat[3], beat[2:0]};
				spc_wd <= rd_data;
			end
			if (beat == 4'd15) begin
				if (!if_mem[if_rp][6]) bghit[{jpar, if_mem[if_rp][5:0]}] <= 1;
				else                   sphit[{jpar, if_mem[if_rp][2:0]}] <= 1;
			end
		end
		if (beat == 4'd15) if_rp <= if_rp + 3'd1;
	end

	if (reset) begin
		fs <= F_IDLE; rd_req <= 0; rq_wp <= 0; rq_rp <= 0; if_wp <= 0; if_rp <= 0; beat <= 0;
	end
end

// ------------------------------------------------------------------------------------------
// OUTPUT: two 512-pixel lines per NES line, anchored to line_done
// ------------------------------------------------------------------------------------------
reg [10:0] hcnt;
always @(posedge clk) hcnt <= t_line_done ? 11'd0 : (hcnt == 11'd1363 ? 11'd0 : hcnt + 11'd1);

wire       o_sub = (hcnt >= 11'd682);
wire [10:0] o_pxw = o_sub ? (hcnt - 11'd682) : hcnt;
wire  [9:0] o_px  = o_pxw[9:0];
wire       o_act = (o_px < 10'd512) && (out_y < 9'd240);

// stage 0 -> 1: descriptor read
reg [15:0] pd_q;
always @(posedge clk) pd_q <= pixd[{obuf, o_px[8:1]}];

reg  [7:0] s1_x;
reg        s1_sx, s1_t, s1_act, s1_hb, s1_hs, s1_vb, s1_vs;
always @(posedge clk) begin
	s1_x   <= o_px[8:1];
	s1_sx  <= o_px[0];
	s1_t   <= o_sub;
	s1_act <= o_act;
	s1_hb  <= (o_px >= 10'd512);
	s1_hs  <= (o_px >= 10'd544) && (o_px < 10'd608);
	s1_vb  <= (out_y >= 9'd240);
	s1_vs  <= (out_y >= 9'd245) && (out_y < 9'd247);
end

// stage 1 -> 2: cache reads
wire [5:0] d_slot   = pd_q[5:0];
wire [2:0] d_fx     = pd_q[8:6];
wire [3:0] d_bgc    = pd_q[12:9];
wire       d_bg_on  = pd_q[13];
wire       d_spr_on = pd_q[14];

reg [63:0] bgc_q;
always @(posedge clk) bgc_q <= bgc[{opar, d_slot, s1_t, d_fx}];

reg  [3:0] s2_bgc;
reg        s2_bg_on, s2_bghit, s2_sx, s2_act, s2_hb, s2_hs, s2_vb, s2_vs;
reg  [7:0] s2_sphit;

// per-sprite results, packed: sprite g at [g] / [2g+1:2g] / [32g+31:32g]
wire   [7:0] sp_draw, sp_nes_opq, sp_prio;
wire  [15:0] sp_ci, sp_pal;
wire [255:0] sp_tex;

genvar g;
generate for (g = 0; g < 8; g = g + 1) begin : g_spr
	wire [54:0] sd  = out_spr[g];
	wire  [8:0] rel = {1'b0, s1_x} - {1'b0, sd[45:38]};                // no wrap past x = 255
	wire        cov = sd[54] && d_spr_on && (rel < 9'd8);
	wire  [3:0] col = {rel[2:0], s1_sx} ^ (sd[52] ? 4'hF : 4'h0);      // attr[6] = hflip
	wire        row = s1_t ^ sd[53];                                    // attr[7] = vflip

	reg  [63:0] spc [32];
	reg  [63:0] spc_q;
	always @(posedge clk) if (spc_we[g]) spc[spc_wa] <= spc_wd;
	always @(posedge clk) spc_q <= spc[{opar, row, col[3:1]}];

	reg       cov_r, half_r, prio_r;
	reg [1:0] ci_r, pal_r;
	always @(posedge clk) begin
		cov_r  <= cov;
		half_r <= col[0];
		prio_r <= sd[51];                                               // attr[5] = behind BG
		ci_r   <= {sd[rel[2:0]], sd[8 + rel[2:0]]};                      // {pix2, pix1} at rel
		pal_r  <= sd[47:46];
	end

	wire [31:0] tex = half_r ? spc_q[63:32] : spc_q[31:0];
	assign sp_tex[g*32 +: 32] = tex;
	assign sp_ci[g*2 +: 2]    = ci_r;
	assign sp_pal[g*2 +: 2]   = pal_r;
	assign sp_prio[g]         = prio_r;
	assign sp_nes_opq[g]      = cov_r && (ci_r != 2'd0);
	assign sp_draw[g]         = cov_r && (s2_sphit[g] ? tex[31] : (ci_r != 2'd0));
end endgenerate

always @(posedge clk) begin
	s2_bgc   <= d_bgc;
	s2_bg_on <= d_bg_on;
	s2_bghit <= bghit[{opar, d_slot}];
	s2_sphit <= opar ? sphit[15:8] : sphit[7:0];
	s2_sx    <= s1_sx;
	s2_act   <= s1_act;
	s2_hb    <= s1_hb;  s2_hs <= s1_hs;  s2_vb <= s1_vb;  s2_vs <= s1_vs;
end

// stage 2 -> 3: composite (Mesen order as a priority pick)
wire [31:0] bg_tex = s2_sx ? bgc_q[63:32] : bgc_q[31:0];
wire        bg_draw = s2_bg_on && (s2_bghit ? bg_tex[31] : (s2_bgc[1:0] != 2'd0));
wire  [7:0] s2_prio = sp_prio;

// lowest-index behind-BG sprite whose NES pixel is opaque masks all front sprites after it
wire [7:0] behind_opq = sp_nes_opq & s2_prio;
wire [7:0] front_ok;
generate for (g = 0; g < 8; g = g + 1) begin : g_front
	if (g == 0) assign front_ok[g] = 1'b1;
	else        assign front_ok[g] = ~|behind_opq[g-1:0];
end endgenerate
wire [7:0] front_draw  = sp_draw & ~s2_prio & front_ok;
wire [7:0] behind_draw = sp_draw &  s2_prio;

// lowest index wins (sprite 0 is on top, as in Mesen's last-to-first draw)
reg  [2:0] win_k;
reg        win_spr, win_any_spr;
integer j;
always @(*) begin
	win_k = 3'd0; win_spr = 0;
	for (j = 7; j >= 0; j = j - 1) if (front_draw[j]) begin win_k = j[2:0]; win_spr = 1; end
	win_any_spr = win_spr;
	if (!win_spr && !bg_draw)
		for (j = 7; j >= 0; j = j - 1) if (behind_draw[j]) begin win_k = j[2:0]; win_any_spr = 1; end
end

reg        s3_tex;
reg [23:0] s3_rgb;
reg  [4:0] s3_pidx;
reg        s3_act, s3_hb, s3_hs, s3_vb, s3_vs;
always @(posedge clk) begin
	s3_act <= s2_act; s3_hb <= s2_hb; s3_hs <= s2_hs; s3_vb <= s2_vb; s3_vs <= s2_vs;
	if (win_any_spr) begin
		s3_tex  <= s2_sphit[win_k];
		s3_rgb  <= sp_tex[win_k*32 +: 24];
		s3_pidx <= {1'b1, sp_pal[win_k*2 +: 2], sp_ci[win_k*2 +: 2]};
	end else if (bg_draw) begin
		s3_tex  <= s2_bghit;
		s3_rgb  <= bg_tex[23:0];
		s3_pidx <= {1'b0, s2_bgc};
	end else begin
		s3_tex  <= 0;
		s3_rgb  <= 24'd0;
		s3_pidx <= 5'd0;                                                // backdrop
	end
end

// stage 3 -> 4: palette RAM entry -> SD colour lookup
reg  [5:0] s4_nes;
reg        s4_tex, s4_act, s4_hb, s4_hs, s4_vb, s4_vs;
reg [23:0] s4_rgb;
always @(posedge clk) begin
	s4_nes <= pal_entry(out_pal, s3_pidx);
	s4_tex <= s3_tex; s4_rgb <= s3_rgb;
	s4_act <= s3_act; s4_hb <= s3_hb; s4_hs <= s3_hs; s4_vb <= s3_vb; s4_vs <= s3_vs;
end

reg [23:0] sd_q;
always @(posedge clk) sd_q <= sdpal[s4_nes];

reg        s5_tex, s5_act, s5_hb, s5_hs, s5_vb, s5_vs;
reg [23:0] s5_rgb;
always @(posedge clk) begin
	s5_tex <= s4_tex; s5_rgb <= s4_rgb;
	s5_act <= s4_act; s5_hb <= s4_hb; s5_hs <= s4_hs; s5_vb <= s4_vb; s5_vs <= s4_vs;
end

always @(posedge clk) begin
	{R, G, B} <= !s5_act ? 24'd0 : s5_tex ? {s5_rgb[7:0], s5_rgb[15:8], s5_rgb[23:16]} : sd_q;
	HS <= s5_hs;
	VS <= s5_vs;
	HB <= s5_hb;
	VB <= s5_vb;
end

endmodule
