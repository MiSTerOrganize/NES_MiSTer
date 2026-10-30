// hd3_resolve.sv - stage 3 RESOLVE: the frame's rule lists filtered by the frame conditions
// (HDPACK_DESIGN.md "hd3_resolve").
//
// Runs once per frame, in vblank, after hd3_dict has resolved every record of the frame's bank
// to a rule list and hd_watch holds the frame's snapshot. For each record id the list its key
// resolved to is filtered ONCE (records sharing a list share the result): keep the rules whose
// frame conditions pass, in order, stopping after the first kept rule with nothing left to
// test (tools/hwmodel.HwModel.filter_list). A rewrite-sensitive frame condition is not
// evaluated here - Mesen memoises it at first use, so its answer can depend on lookups made
// earlier in the frame - the rule keeps its frame list pointer and render tests it. Every other
// frame condition is evaluated at most once (memo), on the frame as captured:
//   memoryCheck / memoryCheckConstant   the watched-address snapshot (hd_watch value port)
//   frameRange                          the frame number (hd3_dict's copy of the trailer)
//   tileAtPosition / spriteAtPosition   the record at a position, rebuilt from the frame
//                                       record in DDR3 exactly as tools/hdref.build_records
//
// Additions (8a): background selection runs before them (hdref.render selects, then
// process_additions), so it sees the frame as captured. A record list is filtered as RENDER
// would first test it, after additions: when the pack has additions, pixel (0,0) holds line
// 239's pixel 0 record (Mesen's lineFirstPixel copy), and a position's sprite list is its
// captured sprites then hd3_add's lanes of its line covering it (line 240 for pixel (0,0)), up
// to four. An added sprite has the added key: in a CHR-ROM game TileIndex = its body and a zero
// fingerprint; in a CHR-RAM game no TileIndex, so no tile condition matches it (hdref.tmatch).
// The answer a background memoised stays (Mesen memoises at first use). hd3_add must be done
// before this module starts.
//
// Outputs, per bank (render reads them through r_*):
//   record table [bank, record id] = {kind, last prefix, count, start, list offset / 8}
//                kind 0 no list, 1 filtered (count rules from start in the rule store),
//                2 unfiltered (the rule store was full: render walks the DDR3 list itself)
//   rule store   [bank, i] = {prefix, walk list pointer, frame list pointer or 0, payload}
//
// Memories have one write port and registered reads (the Quartus block-RAM shape); the line
// buffers are 16 x 64 bits.

module hd3_resolve #(
	parameter [27:0] REC_BASE = 28'hC000000,   // frame-record buffer 0 (byte offset)
	parameter        RAW = 11,                  // record id width
	parameter        RSW = 10,                  // rule store: 2^RSW rules per bank
	parameter        RS_MAX = 1 << RSW,         // rules a bank may use (a test lowers it)
	parameter        RTW = 2 + 1 + 11 + RSW + 25,
	parameter integer NL = 16                   // hd3_add lanes per line (its NL)
)
(
	input             clk,
	input             reset,

	// pack and game configuration
	input             game_ram,
	input             legacy,
	input      [27:3] rules_base,       // section 1 (rule lists)
	input      [27:3] conds_base,       // section 2 (conditions, 32 bytes each)
	input      [27:3] clist_base,       // section 3 (condition lists, u32 words)
	input      [27:3] fb_base,          // section 10 (fallback map, i32 per tile)
	input      [31:0] fb_len,
	input      [27:3] bgl_base,         // section 5 (background level table, 40 x 8 bytes)
	input      [27:0] bgi_base,         // section 13 (background value index), BYTE address
	input             add_en,           // the pack has additions (8a)

	// job: filter bank `bank` (its records resolved, its snapshot complete)
	input             start,
	input             bank,
	output reg        busy = 0,
	output reg        done = 0,         // pulse
	output reg        overflow = 0,     // sticky: a rule store filled (render walks DDR3)
	output reg        bg_lin = 0,       // sticky: an indexed level was scanned linearly
	output reg        bg_warn = 0,      // sticky: a background result could not be represented
	                                    // (record index >= 2^20, or a rewrite-sensitive
	                                    // condition whose memo index is beyond the memo)

	// hd3_dict read port: one clock latency
	output reg        d_bank = 0,
	output reg [RAW-1:0] d_rid = 0,
	input      [95:0] d_key,
	input      [63:0] d_res,
	input     [RAW:0] d_nrec,
	input      [31:0] d_frame,

	// hd_watch value port: one clock latency
	output reg        v_bank = 0,
	output reg  [7:0] v_idx = 0,
	input       [7:0] v_q,

	// DDR3 read port (8-qword bursts, in order)
	output reg        rd_req = 0,
	output reg [27:3] rd_addr,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid,

	// hd3_add's lanes of line a_y (one clock latency): {.., added id, length-1 3, x0 8}
	output reg  [7:0] a_y = 0,
	input       [6:0] a_n,
	input [NL*(RAW+21)-1:0] a_q,

	// render / testbench read ports: one clock latency
	input             r_bank,
	input   [RAW-1:0] r_rid,
	output reg [RTW-1:0] r_rec,         // {kind 2, lastp, count 11, start RSW, offq 25}
	input   [RSW-1:0] r_ri,
	output reg [72:0] r_rule,           // {prefix, wptr 20, fptr 20, payload 32}
	input             g_bank,           // background selection: {valid, record index 20}
	input       [5:0] g_lvl,            //   per level (0-39) of bank g_bank (render's frame,
	output reg [20:0] g_q,              //   not r_bank: resolve may already be on the next)
	input             s_bank,           // memo seed (8i): {valid, raw value (before negation)}
	input      [10:0] s_a,              //   per memo index - the rewrite-sensitive conditions
	output reg  [1:0] s_q               //   background selection evaluated; render's memo
	                                    //   sweep loads it
);

// ==========================================================================================
// memories
// ==========================================================================================
reg [RTW-1:0] rt [2 << RAW];                     // record table, {bank, rid}
reg  [RAW:0] rt_wa;
reg [RTW-1:0] rt_wd;
reg         rt_we;
always @(posedge clk) begin
	if (rt_we) rt[rt_wa] <= rt_wd;
	r_rec <= rt[{r_bank, r_rid}];
end

reg  [72:0] rs [2 << RSW];                       // rule store, {bank, index}
reg  [RSW:0] rs_wa;
reg  [72:0] rs_wd;
reg         rs_we;
always @(posedge clk) begin
	if (rs_we) rs[rs_wa] <= rs_wd;
	r_rule <= rs[{r_bank, r_ri}];
end

// list table, per job: {valid, offq 25, kind 2, lastp, count 11, start RSW}
localparam LTW = 1 + 25 + RTW - 25;
reg [LTW-1:0] lt [2048];
reg  [10:0] lt_ra, lt_wa;
reg [LTW-1:0] lt_wd, lt_q;
reg         lt_we;
always @(posedge clk) begin
	if (lt_we) lt[lt_wa] <= lt_wd;
	lt_q <= lt[lt_ra];
end

// condition memo, per job: {done, value} by the condition's memo index
reg   [1:0] memo [2048];
reg  [10:0] mm_ra, mm_wa;
reg   [1:0] mm_wd, mm_q;
reg         mm_we;
always @(posedge clk) begin
	if (mm_we) memo[mm_wa] <= mm_wd;
	mm_q <= memo[mm_ra];
end

// condition-list results, per job: the pass/fail of a whole frame-condition list depends only
// on its pointer within one frame (every condition is memoised or deferred), so a list shared
// by many rules (Metroid's title keys: hundreds of rules on two lists) is walked once.
// {valid, list pointer 28, pass, deferred conditions present}, direct-mapped
reg  [30:0] flm [1024];
reg   [9:0] fl_ra, fl_wa, fl_h;
reg  [30:0] fl_wd, fl_q;
reg         fl_we;
always @(posedge clk) begin
	if (fl_we) flm[fl_wa] <= fl_wd;
	fl_q <= flm[fl_ra];
end

// background selection, per bank: {valid, record index} per level
reg  [20:0] bgsel [128];
reg   [6:0] bs_wa;
reg  [20:0] bs_wd;
reg         bs_we;
always @(posedge clk) begin
	if (bs_we) bgsel[bs_wa] <= bs_wd;
	g_q <= bgsel[{g_bank, g_lvl}];
end

// memo seed, per bank: {valid, value} by memo index (cleared with the memo)
reg   [1:0] seed [4096];
reg  [11:0] sd_wa;
reg   [1:0] sd_wd;
reg         sd_we;
always @(posedge clk) begin
	if (sd_we) seed[sd_wa] <= sd_wd;
	s_q <= seed[{s_bank, s_a}];
end

// candidate bitmap of one indexed level: {level + 1, 32 entry bits} per word, entries up to
// 32,767 (corpus max 20,488); a word whose tag is not this level's is empty. Tags cleared per job.
reg  [37:0] bm [1024];
reg   [9:0] bm_ra, bm_wa;
reg  [37:0] bm_wd, bm_q;
reg         bm_we;
always @(posedge clk) begin
	if (bm_we) bm[bm_wa] <= bm_wd;
	bm_q <= bm[bm_ra];
end

// ==========================================================================================
// fetch unit: four 8-qword line buffers in front of the DDR3 port, one per stream so they do
// not evict each other: 0 the rule list, 1 condition-list words, 2 condition records, 3 the
// random reads (frame record, fallback map, a list's last rule).
// A request (fq_req pulse with fq_line, fq_addr) is answered by fq_ack with fq_q.
// ==========================================================================================
reg         fq_req = 0;
reg   [1:0] fq_line = 0;
reg  [27:3] fq_addr;
reg         fq_ack = 0;

typedef enum bit [2:0] { FT_IDLE, FT_HIT, FT_ACK, FT_WAIT, FT_FILL } ft_t;
ft_t ft = FT_IDLE;
reg   [1:0] ft_line;
reg  [27:3] ft_addr;
reg   [2:0] ft_beat;
reg  [27:3] tag [4];
reg   [3:0] tv = 0;
reg         ft_inv;

reg  [63:0] lb [32];
reg   [4:0] lb_ra;
reg  [63:0] lb_q;
wire        lb_we = ft == FT_FILL && rd_valid;
always @(posedge clk) begin
	if (lb_we) lb[{ft_line, ft_beat}] <= rd_data;
	lb_q <= lb[lb_ra];
end
wire [63:0] fq_q = lb_q;

wire [27:3] ft_d = fq_addr - tag[fq_line];

always @(posedge clk) begin
	fq_ack <= 0;
	case (ft)
		FT_IDLE: if (fq_req) begin
			ft_line <= fq_line;
			ft_addr <= fq_addr;
			if (tv[fq_line] && ft_d < 25'd8) begin
				lb_ra <= {fq_line, ft_d[5:3]};
				ft    <= FT_HIT;
			end else begin
				rd_req      <= 1;
				rd_addr     <= fq_addr;
				tv[fq_line] <= 0;
				ft          <= FT_WAIT;
			end
		end
		FT_HIT: ft <= FT_ACK;                     // lb_q valid next clock
		FT_ACK: begin fq_ack <= 1; ft <= FT_IDLE; end
		FT_WAIT: if (rd_ack) begin
			rd_req  <= 0;
			ft_beat <= 0;
			ft      <= FT_FILL;
		end
		FT_FILL: if (rd_valid) begin
			ft_beat <= ft_beat + 3'd1;
			if (ft_beat == 3'd7) begin
				tag[ft_line] <= ft_addr;
				tv[ft_line]  <= 1;
				lb_ra        <= {ft_line, 3'd0};
				ft           <= FT_HIT;
			end
		end
		default: ft <= FT_IDLE;
	endcase
	if (ft_inv) tv <= 0;
	if (reset) begin
		ft <= FT_IDLE; rd_req <= 0; tv <= 0;
	end
end

// ==========================================================================================
// main state machine
// ==========================================================================================
typedef enum bit [6:0] {
	S_IDLE, S_CLR, S_REC, S_REC1, S_RES, S_LT, S_LT1, S_LT2,
	S_LHDR, S_LASTP, S_RA, S_RB, S_FL, S_FL1,
	S_CW, S_CW1, S_C0, S_C0A, S_MEMO, S_C1, S_C2, S_C3, S_EVAL,
	S_MEM, S_MEM1, S_MEM2, S_DIV,
	S_PIX, S_HDR0, S_HDR1, S_HDR2, S_HDR3, S_SPAN, S_SPANB, S_SPR, S_SPRB, S_TM, S_FB,
	S_TMEND, S_CDONE, S_KEEP, S_NEXT, S_LEND, S_FIN,
	S_LN, S_LNW, S_LN1, S_LNK, S_LNK1,                // the lanes of a position's line (8a)
	// background selection (8i)
	S_GL, S_GL1, S_GH, S_GSC, S_GSG, S_GSG1, S_GTV, S_GTV1, S_GTV2, S_GCRC,
	S_GP0, S_GP1, S_GP2, S_GP3, S_GCL, S_GCL1, S_GBM, S_GBM1, S_GBM2, S_GSN,
	S_GSC0, S_GSC1, S_GSC2, S_GE, S_GF, S_GSEL, S_GEND
} st_t;
st_t st = S_IDLE;

reg         bk;
reg  [RAW:0] rid, nrec;
reg  [31:0] frame;
reg  [11:0] clr;
reg  [24:0] offq;
reg  [10:0] h;
reg  [27:3] lbase;                  // the list header's qword address
reg  [31:0] nrules, ri;
reg         lastp;
reg  [RSW:0] rs_n;                  // rules stored in this bank so far
reg [RSW-1:0] lstart;
reg  [10:0] lcount;
reg   [1:0] lkind;
reg  [31:0] payload, fptr, wptr;
reg         pfx;
reg         later;                  // the rule kept a rewrite-sensitive frame condition
reg  [27:0] cp;                     // condition-list byte pointer
reg         clast;
reg  [23:0] ci;
reg  [63:0] c0, c1, c2, c3;         // the condition record
reg         res;                    // condition result
reg         ev;                     // res was evaluated now (negate it, memoise it)
reg   [7:0] ma;
reg  [16:0] rem;
reg  [31:0] dq;
reg   [5:0] dn;
reg   [7:0] px, py;                 // tile/sprite at position
reg   [7:0] b0;
reg  [63:0] h0;
reg [191:0] hpal;
reg   [6:0] fi, nfet;
reg   [2:0] ncov;
reg  [63:0] qa;
reg  [15:0] t_idx;
reg  [63:0] t_fp;
reg         t_spr;
// 8a: the position's lanes
localparam integer LW = RAW + 21;
reg   [7:0] ay;                     // the lane line (240 for pixel (0,0))
reg   [6:0] lj;                     // next lane to test
reg         t_lane;                 // the record being matched is an added sprite
reg  [31:0] lpal;                   //   its palette
reg         t_hi;                   //   its body is not a 16-bit TileIndex
reg  [63:0] lbody;                  //   its body
reg         bgm;                    // evaluating background entries, not a record's list (8i)
wire        lanes = add_en && !bgm;  // record lists are filtered as after additions
wire [LW-1:0] ln = a_q[LW * lj[$clog2(NL)-1:0] +: LW];
wire  [7:0] ln_d = px - ln[7:0];

// background selection (8i) - bgm is declared above, before `lanes` reads it
reg   [5:0] lvl;
reg  [31:0] ioff;
reg  [31:0] nsig, sk;               // signatures of the level, current one
reg [447:0] sgb;                    // 56 bytes read around the signature record
reg   [2:0] sgn;                    // qwords of it read
reg  [27:0] sga;                    // the signature record's byte address
reg   [3:0] tj;                     // term
reg [127:0] vals;                   // the masked snapshot values (zero past the terms)
reg  [31:0] crc;
reg   [4:0] ck;
reg  [31:0] pp;                     // probe count
reg  [63:0] pq0;
reg  [27:0] ea;                     // the table entry's byte address
reg  [27:0] la;                     // the candidate list's byte address
reg  [31:0] lcnt, li;
reg  [31:0] cand;
reg   [9:0] wmin, wmax, sw;
reg  [31:0] sbits;
reg         lin;                    // this level falls back to a linear scan
reg  [31:0] gri;                    // the entry being evaluated

wire [447:0] sg_sh  = sgb >> {sga[2:0], 3'b000};
wire [383:0] sg     = sg_sh[383:0];                 // the 48-byte signature record
wire   [7:0] sg_n   = sg[7:0];
wire  [31:0] sg_toff = sg[8*34 +: 32];
wire  [31:0] sg_nb   = sg[8*38 +: 32];
wire  [31:0] sg_maxp = sg[8*42 +: 32];
wire  [31:0] g_bkt   = (crc + pp) & (sg_nb - 32'd1);                 // bucket being probed
wire  [27:0] g_ea    = bgi_base + sg_toff[27:0] + {g_bkt[22:0], 5'd0}; // its 32-byte entry
wire  [27:0] g_sga   = bgi_base + ioff[27:0] + 28'd4 + sk[27:0] * 28'd48;
wire  [27:0] g_cnt_a = bgi_base + ioff[27:0];                       // the signature count
wire  [27:0] g_la4   = la + {li[25:0], 2'b00};                       // candidate li's word
wire  [31:0] g_cw    = g_la4[2] ? fq_q[63:32] : fq_q[31:0];
reg          vdiff;                 // the probed entry holds other values

// the pack format's CRC-32 (poly 0x04C11DB7, init 0, MSB first, no reflection, no final
// XOR - tools/hdp3_convert.py crc32, the same form as hd3_lookup's crc96), one byte MSB
// first. NOT zlib's CRC-32.
function [31:0] crc_byte(input [31:0] c, input [7:0] d);
	integer i;
	reg [31:0] r;
	begin
		r = c;
		for (i = 7; i >= 0; i = i - 1)
			r = {r[30:0], 1'b0} ^ ((r[31] ^ d[i]) ? 32'h04C11DB7 : 32'd0);
		crc_byte = r;
	end
endfunction

function [4:0] lowbit(input [31:0] v);
	integer i;
	begin
		lowbit = 0;
		for (i = 31; i >= 0; i = i - 1) if (v[i]) lowbit = i[4:0];
	end
endfunction

function [5:0] c6(input [191:0] p, input [4:0] n);
	c6 = p[6 * n +: 6];
endfunction

function cmp8(input [7:0] a, input [7:0] b, input [3:0] op);
	case (op)
		4'd0: cmp8 = a == b;
		4'd1: cmp8 = a != b;
		4'd2: cmp8 = a >  b;
		4'd3: cmp8 = a <  b;
		4'd4: cmp8 = a <= b;
		4'd5: cmp8 = a >= b;
		default: cmp8 = 1'b0;
	endcase
endfunction

// condition fields
wire  [7:0] c_type = c0[7:0];
wire        c_neg  = c0[8];
wire        c_ign  = c0[9];
wire        c_data = c0[10];
wire  [3:0] c_op   = c0[15:12];
wire  [7:0] c_mask = c0[23:16];
wire [10:0] c_memo = c0[42:32];

// this bank's frame record, as qword addresses
wire [27:3] rec_q  = REC_BASE[27:3] + {7'd0, bk, 17'd0};
wire [27:3] line_q = rec_q + 25'h8000 + {9'd0, py, 8'd0};          // line py's header

// the record's palette word, from the line's palette RAM (hdref.build_records)
wire  [2:0] grp     = t_spr ? {1'b1, qa[47:46]} : {1'b0, qa[33:32]};
wire  [7:0] top     = legacy ? 8'h00 : (t_spr ? 8'hFF : {2'b00, c6(hpal, 5'd0)});
wire [31:0] rec_pal = {top, 2'b00, c6(hpal, {grp, 2'd1}), 2'b00, c6(hpal, {grp, 2'd2}),
                       2'b00, c6(hpal, {grp, 2'd3})};
wire        tm_pal  = c_ign || (t_lane ? lpal : rec_pal) == c3[31:0];

// hd3_dict result word: {done, via[1:0], list offset / 8 [24:0], ...}
wire  [1:0] r_via = d_res[62:61];
wire [24:0] r_off = d_res[60:36];

// the list's entry, as the record table and list table store it
wire [RTW-1:0] lentry = {lkind, lastp, lcount, lstart, offq};

wire [27:3] word_q = cp[27:3];
wire [31:0] word   = cp[2] ? fq_q[63:32] : fq_q[31:0];

always @(posedge clk) begin
	done   <= 0;
	fq_req <= 0;
	ft_inv <= 0;
	rt_we  <= 0;
	rs_we  <= 0;
	lt_we  <= 0;
	mm_we  <= 0;
	fl_we  <= 0;
	bs_we  <= 0;
	sd_we  <= 0;
	bm_we  <= 0;

	case (st)
		S_IDLE: if (start) begin
			bk     <= bank;
			d_bank <= bank;
			v_bank <= bank;
			busy   <= 1;
			clr    <= 0;
			ft_inv <= 1;
			st     <= S_CLR;
		end

		// clear the list table, the memo, this bank's memo seed and the bitmap tags (2048
		// clocks)
		S_CLR: begin
			lt_we <= 1; lt_wa <= clr[10:0]; lt_wd <= 0;
			mm_we <= 1; mm_wa <= clr[10:0]; mm_wd <= 0;
			fl_we <= 1; fl_wa <= clr[9:0];  fl_wd <= 0;
			sd_we <= 1; sd_wa <= {bk, clr[10:0]}; sd_wd <= 0;
			bm_we <= 1; bm_wa <= clr[9:0];  bm_wd <= 0;
			clr   <= clr + 12'd1;
			if (clr == 12'd2047) begin
				rid   <= 0;
				rs_n  <= 0;
				nrec  <= d_nrec;
				frame <= d_frame;
				lvl   <= 0;
				bgm   <= 1;
				st    <= S_GL;
			end
		end

		// ==================================================================================
		// background selection (hdref.select_backgrounds), BEFORE the record lists: per level
		// the first entry whose frame conditions all pass. Mesen memoises a frame condition at
		// first use and this is the first use for every condition a background tests, so a
		// rewrite-sensitive one is evaluated here (nothing is rewritten yet) and its value
		// seeds render's memo. Candidates are evaluated in ascending entry order and the
		// level stops at the first that passes - exactly the entries Mesen evaluates.
		// ==================================================================================
		S_GL: begin
			if (lvl == 6'd40) begin
				bgm <= 0;
				st  <= S_REC;
			end else begin
				fq_req <= 1; fq_line <= 2'd3; fq_addr <= bgl_base + {19'd0, lvl};
				st     <= S_GL1;
			end
		end
		S_GL1: if (fq_ack) begin
			ioff <= fq_q[63:32];
			lin  <= 0;
			if (fq_q[31:0] == 32'd0) begin                   // no background at this level
				bs_we <= 1; bs_wa <= {bk, lvl}; bs_wd <= 0;
				st    <= S_GEND;
			end else begin
				lbase  <= rules_base + fq_q[27:3];
				fq_req <= 1; fq_line <= 2'd0; fq_addr <= rules_base + fq_q[27:3];
				st     <= S_GH;
			end
		end
		S_GH: if (fq_ack) begin
			nrules <= fq_q[31:0];
			bs_wd  <= 0;                                     // unselected unless one passes
			if (fq_q[31:0] == 32'd0) begin
				bs_we <= 1; bs_wa <= {bk, lvl};
				st    <= S_GEND;
			end else if (ioff == 32'd0) begin                // unindexed: every entry in order
				gri <= 0;
				st  <= S_GE;
			end else begin
				fq_req  <= 1; fq_line <= 2'd3;
				fq_addr <= g_cnt_a[27:3];
				wmin    <= 10'h3FF; wmax <= 0;
				st      <= S_GSC;
			end
		end
		S_GSC: if (fq_ack) begin
			nsig <= g_cnt_a[2] ? fq_q[63:32] : fq_q[31:0];
			sk   <= 0;
			st   <= S_GSN;
		end

		// ---- next signature: read its 48-byte record ----
		S_GSN: begin
			if (sk == nsig) begin
				if (lin) begin
					bg_lin <= 1;
					gri    <= 0;
					st     <= S_GE;
				end else if (wmin > wmax) begin            // no candidate
					bs_we <= 1; bs_wa <= {bk, lvl};
					st    <= S_GEND;
				end else begin
					sw <= wmin;
					st <= S_GSC0;
				end
			end else begin
				sga    <= g_sga;
				sgn    <= 0;
				fq_req <= 1; fq_line <= 2'd3; fq_addr <= g_sga[27:3];
				st     <= S_GSG;
			end
		end
		S_GSG: if (fq_ack) begin
			sgb[64 * sgn +: 64] <= fq_q;
			if (sgn == 3'd6) begin
				tj   <= 0;
				vals <= 0;
				st   <= S_GTV;
			end else begin
				sgn    <= sgn + 3'd1;
				fq_req <= 1; fq_line <= 2'd3; fq_addr <= sga[27:3] + {22'd0, sgn} + 25'd1;
			end
		end
		// the masked snapshot value of each term (hd_watch value port: one-clock latency)
		S_GTV: begin
			if ({4'd0, tj} >= sg_n) begin
				crc <= 32'd0;
				ck  <= 0;
				st  <= S_GCRC;
			end else begin
				v_idx <= sg[8 * (2 + 2 * tj) +: 8];
				st    <= S_GTV1;
			end
		end
		S_GTV1: st <= S_GTV2;
		S_GTV2: begin
			vals[8 * tj +: 8] <= v_q & sg[8 * (3 + 2 * tj) +: 8];
			tj <= tj + 4'd1;
			st <= S_GTV;
		end
		S_GCRC: begin
			crc <= crc_byte(crc, vals[8 * ck[3:0] +: 8]);
			ck  <= ck + 5'd1;
			if (ck == 5'd15) begin
				pp <= 0;
				st <= S_GP0;
			end
		end

		// ---- probe the signature's table (linear probing; an invalid entry ends it) ----
		S_GP0: begin
			ea     <= g_ea;
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= g_ea[27:3];
			st     <= S_GP1;
		end
		S_GP1: if (fq_ack) begin
			pq0    <= fq_q;                                  // value bytes 0-7
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= ea[27:3] + 25'd1;
			st     <= S_GP2;
		end
		S_GP2: if (fq_ack) begin
			vdiff  <= {fq_q, pq0} != vals;                   // bytes 8-15; used if valid
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= ea[27:3] + 25'd2;
			st     <= S_GP3;
		end
		S_GP3: if (fq_ack) begin
			la   <= bgi_base + fq_q[27:0];
			lcnt <= fq_q[63:32];
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= ea[27:3] + 25'd3;
			st     <= S_GCL;
		end
		S_GCL: if (fq_ack) begin
			if (fq_q[31:0] == 32'd0) begin                   // invalid: no entry for these values
				sk <= sk + 32'd1;
				st <= S_GSN;
			end else if (vdiff) begin                        // another value set: next probe
				if (pp == sg_maxp) begin
					sk <= sk + 32'd1;
					st <= S_GSN;
				end else begin
					pp <= pp + 32'd1;
					st <= S_GP0;
				end
			end else begin
				li <= 0;
				st <= S_GCL1;
			end
		end

		// ---- the matching entry's candidates into the bitmap ----
		S_GCL1: begin
			if (li == lcnt) begin
				sk <= sk + 32'd1;
				st <= S_GSN;
			end else begin
				fq_req <= 1; fq_line <= 2'd3; fq_addr <= g_la4[27:3];
				st     <= S_GBM;
			end
		end
		S_GBM: if (fq_ack) begin
			cand <= g_cw;
			li   <= li + 32'd1;
			if (g_cw >= 32'd32768) begin
				lin <= 1;                                    // beyond the bitmap: scan linearly
				st  <= S_GCL1;
			end else begin
				bm_ra <= g_cw[14:5];
				st    <= S_GBM1;
			end
		end
		S_GBM1: st <= S_GBM2;                                // bm_q valid next clock
		S_GBM2: begin
			bm_we <= 1; bm_wa <= cand[14:5];
			bm_wd <= {lvl + 6'd1, ((bm_q[37:32] == lvl + 6'd1) ? bm_q[31:0] : 32'd0) |
			          (32'd1 << cand[4:0])};
			if (cand[14:5] < wmin) wmin <= cand[14:5];
			if (cand[14:5] > wmax) wmax <= cand[14:5];
			st <= S_GCL1;
		end

		// ---- scan the bitmap in ascending entry order ----
		S_GSC0: begin
			if (sw > wmax) begin
				bs_we <= 1; bs_wa <= {bk, lvl};               // none passed
				st    <= S_GEND;
			end else begin
				bm_ra <= sw;
				st    <= S_GSC1;
			end
		end
		S_GSC1: st <= S_GSC2;
		S_GSC2: begin
			sbits <= (bm_q[37:32] == lvl + 6'd1) ? bm_q[31:0] : 32'd0;
			if (bm_q[37:32] != lvl + 6'd1 || bm_q[31:0] == 32'd0) begin
				if (sw == wmax) begin
					bs_we <= 1; bs_wa <= {bk, lvl};
					st    <= S_GEND;
				end else begin
					sw <= sw + 10'd1;
					st <= S_GSC0;
				end
			end else begin
				gri <= {17'd0, sw, lowbit(bm_q[31:0])};
				st  <= S_GE;
			end
		end

		// ---- evaluate entry gri (the rule read and condition states are shared) ----
		S_GE: begin
			if (gri >= nrules) begin                         // linear scan ran out
				bs_we <= 1; bs_wa <= {bk, lvl};
				st    <= S_GEND;
			end else begin
				ri     <= gri;
				fq_req <= 1; fq_line <= 2'd0; fq_addr <= lbase + 25'd1 + {gri[23:0], 1'b0};
				st     <= S_RA;
			end
		end
		// the entry failed: next one
		S_GF: begin
			if (ioff == 32'd0 || lin) begin
				gri <= gri + 32'd1;
				st  <= S_GE;
			end else begin
				sbits[gri[4:0]] <= 1'b0;
				if ((sbits & ~(32'd1 << gri[4:0])) != 32'd0)
					gri <= {17'd0, sw, lowbit(sbits & ~(32'd1 << gri[4:0]))};
				if ((sbits & ~(32'd1 << gri[4:0])) != 32'd0)
					st <= S_GE;
				else if (sw == wmax) begin
					bs_we <= 1; bs_wa <= {bk, lvl};
					st    <= S_GEND;
				end else begin
					sw <= sw + 10'd1;
					st <= S_GSC0;
				end
			end
		end
		S_GSEL: begin
			bs_we <= 1; bs_wa <= {bk, lvl}; bs_wd <= {1'b1, payload[19:0]};
			if (payload[31:20] != 12'd0) bg_warn <= 1;
			st    <= S_GEND;
		end
		S_GEND: begin
			lvl <= lvl + 6'd1;
			st  <= S_GL;
		end

		// ---- next record: its resolution from hd3_dict ----
		S_REC: begin
			if (rid == nrec) begin
				busy <= 0;
				done <= 1;
				st   <= S_IDLE;
			end else begin
				d_rid <= rid[RAW-1:0];
				st    <= S_REC1;
			end
		end
		S_REC1: st <= S_RES;                      // d_res valid next clock
		S_RES: begin
			offq <= r_off;
			if (r_via == 2'd0) begin              // no key, no list
				rt_we <= 1; rt_wa <= {bk, rid[RAW-1:0]}; rt_wd <= 0;
				st    <= S_FIN;
			end else begin
				h  <= r_off[10:0] ^ r_off[21:11] ^ {8'd0, r_off[24:22]};
				st <= S_LT;
			end
		end

		// ---- has this list been filtered already? (list table, linear probing) ----
		S_LT:  begin lt_ra <= h; st <= S_LT1; end
		S_LT1: st <= S_LT2;                       // lt_q valid next clock
		S_LT2: begin
			if (lt_q[LTW-1] && lt_q[LTW-2 -: 25] == offq) begin
				rt_we <= 1; rt_wa <= {bk, rid[RAW-1:0]};
				rt_wd <= {lt_q[LTW-27:0], offq};
				st    <= S_FIN;
			end else if (lt_q[LTW-1]) begin
				h  <= h + 11'd1;
				st <= S_LT;
			end else begin
				lbase  <= rules_base + offq;
				lstart <= rs_n[RSW-1:0];
				lcount <= 0;
				lkind  <= 2'd1;
				lastp  <= 0;
				fq_req <= 1; fq_line <= 2'd0; fq_addr <= rules_base + offq;
				st     <= S_LHDR;
			end
		end

		// list header: rule count; then the last rule's w3 (prefix of the whole list)
		S_LHDR: if (fq_ack) begin
			nrules <= fq_q[31:0];
			ri     <= 0;
			if (fq_q[31:0] == 32'd0)
				st <= S_LEND;
			else begin
				fq_req  <= 1; fq_line <= 2'd3;
				fq_addr <= lbase + {fq_q[23:0], 1'b0};       // qB of rule n-1 = header + 2n
				st      <= S_LASTP;
			end
		end
		S_LASTP: if (fq_ack) begin
			lastp   <= fq_q[57];                             // w3 bit 25
			fq_req  <= 1; fq_line <= 2'd0; fq_addr <= lbase + 25'd1;
			st      <= S_RA;
		end

		// rule ri: qA = {w1 frame list, w0 payload}, qB = {w3 flags, w2 walk list}
		S_RA: if (fq_ack) begin
			payload <= fq_q[31:0];
			fptr    <= fq_q[63:32];
			fq_req  <= 1; fq_line <= 2'd0; fq_addr <= lbase + 25'd2 + {ri[23:0], 1'b0};
			st      <= S_RB;
		end
		S_RB: if (fq_ack) begin
			wptr  <= fq_q[31:0];
			pfx   <= fq_q[57];
			later <= 0;
			if (fptr == 32'd0)
				st <= S_KEEP;
			else begin
				fl_h  <= fptr[11:2] ^ fptr[21:12];
				fl_ra <= fptr[11:2] ^ fptr[21:12];
				cp    <= {clist_base, 3'd0} + fptr[27:0];
				// a background entry's list is walked without the list-result cache: there a
				// rewrite-sensitive condition is evaluated, not deferred, so its cached result
				// would wrongly tell a record's list "nothing deferred"
				st    <= bgm ? S_CW : S_FL;
			end
		end
		S_FL: st <= S_FL1;                           // fl_q valid next clock
		S_FL1: begin
			if (fl_q[30] && fl_q[29:2] == fptr[27:0] && fptr[31:28] == 4'd0) begin
				later <= fl_q[0];
				st    <= fl_q[1] ? S_KEEP : S_NEXT;      // this list was walked already
			end else
				st <= S_CW;
		end

		// ---- the rule's frame condition list, one word at a time ----
		S_CW: begin
			fq_req <= 1; fq_line <= 2'd1; fq_addr <= word_q;
			st     <= S_CW1;
		end
		S_CW1: if (fq_ack) begin
			ci     <= word[23:0];
			clast  <= word[31];
			fq_req <= 1; fq_line <= 2'd2; fq_addr <= conds_base + {word[22:0], 2'b00};
			st     <= S_C0;
		end
		S_C0: if (fq_ack) begin
			c0 <= fq_q;
			if (fq_q[24] && !bgm) begin               // rewrite-sensitive: render tests it
				later <= 1;
				res   <= 1;
				ev    <= 0;
				st    <= S_CDONE;
			end else if (fq_q[63:43] != 21'd0) begin
				// memo index beyond the memo (2048; corpus max 1,182): evaluate, unmemoised
				fq_req <= 1; fq_line <= 2'd2; fq_addr <= conds_base + {ci[22:0], 2'b01};
				st     <= S_C1;
			end else begin
				mm_ra <= fq_q[42:32];
				st    <= S_C0A;
			end
		end
		S_C0A: st <= S_MEMO;                         // mm_q valid next clock
		S_MEMO: begin
			if (mm_q[1]) begin                        // memoised: the final value
				res <= mm_q[0];
				ev  <= 0;
				st  <= S_CDONE;
			end else begin
				fq_req <= 1; fq_line <= 2'd2; fq_addr <= conds_base + {ci[22:0], 2'b01};
				st     <= S_C1;
			end
		end
		S_C1: if (fq_ack) begin
			c1 <= fq_q;
			fq_req <= 1; fq_line <= 2'd2; fq_addr <= conds_base + {ci[22:0], 2'b10};
			st <= S_C2;
		end
		S_C2: if (fq_ack) begin
			c2 <= fq_q;
			fq_req <= 1; fq_line <= 2'd2; fq_addr <= conds_base + {ci[22:0], 2'b11};
			st <= S_C3;
		end
		S_C3: if (fq_ack) begin
			c3 <= fq_q;
			ev <= 1;
			st <= S_EVAL;
		end

		// ---- evaluate one frame condition (hdref.eval_cond) ----
		S_EVAL: begin
			case (c_type)
				8'd0, 8'd1: begin                     // memoryCheck(Constant)
					v_idx <= c1[7:0];
					st    <= S_MEM;
				end
				8'd2: begin                           // frameRange: A != 0 and frame % A >= B
					if (c1[15:0] == 16'd0) begin
						res <= 0;
						st  <= S_CDONE;
					end else begin
						rem <= 0;
						dq  <= frame;
						dn  <= 0;
						st  <= S_DIV;
					end
				end
				8'd3, 8'd4: begin                     // tile / sprite at position Y*256+X
					t_lane <= 0;
					t_hi   <= 0;
					if (c3[63:32] >= 32'd61440) begin
						res <= 0;
						st  <= S_CDONE;
					end else begin : at_pos
						// after additions pixel (0,0) is line 239's pixel 0; its lanes are line 240
						reg [7:0] yy;
						yy = (lanes && c3[47:32] == 16'd0) ? 8'd239 : c3[47:40];
						px <= c3[39:32];
						py <= yy;
						ay <= (lanes && c3[47:32] == 16'd0) ? 8'd240 : c3[47:40];
						fq_req  <= 1; fq_line <= 2'd3;
						fq_addr <= rec_q + {10'd0, yy, 7'd0} + {18'd0, c3[39:33]};
						st <= S_PIX;
					end
				end
				default: begin                        // not a frame condition
					res <= 0;
					st  <= S_CDONE;
				end
			endcase
		end

		// memoryCheck: a & mask OP b & mask; memoryCheckConstant: a & mask OP value
		S_MEM:  begin v_idx <= c1[15:8]; st <= S_MEM1; end
		S_MEM1: begin ma <= v_q; st <= S_MEM2; end       // v_q = slot A
		S_MEM2: begin                                    // v_q = slot B
			res <= (c_type == 8'd0) ? cmp8(ma & c_mask, v_q & c_mask, c_op)
			                        : cmp8(ma & c_mask, c1[15:8], c_op);
			st  <= S_CDONE;
		end

		// frame % A by restoring division
		S_DIV: begin
			if ({rem[15:0], dq[31]} >= {1'b0, c1[15:0]})
				rem <= {rem[15:0], dq[31]} - {1'b0, c1[15:0]};
			else
				rem <= {rem[15:0], dq[31]};
			dq <= {dq[30:0], 1'b0};
			dn <= dn + 6'd1;
			if (dn == 6'd31) st <= S_TM;
		end

		// ---- the record at a position, as hdref.build_records rebuilds it ----
		S_PIX: if (fq_ack) begin
			b0 <= px[0] ? fq_q[39:32] : fq_q[7:0];
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= line_q;
			st <= S_HDR0;
		end
		S_HDR0: if (fq_ack) begin
			h0 <= fq_q;
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= line_q + 25'd1;
			st <= S_HDR1;
		end
		S_HDR1: if (fq_ack) begin
			hpal[63:0] <= fq_q;
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= line_q + 25'd2;
			st <= S_HDR2;
		end
		S_HDR2: if (fq_ack) begin
			hpal[127:64] <= fq_q;
			fq_req <= 1; fq_line <= 2'd3; fq_addr <= line_q + 25'd3;
			st <= S_HDR3;
		end
		S_HDR3: if (fq_ack) begin
			hpal[191:128] <= fq_q;
			res <= 0;
			if (c_type == 8'd3) begin
				// tileAtPosition: the pixel's BG record, none if NoTile
				t_spr <= 0;
				if (b0[5:0] == 6'd63 || {2'b00, b0[5:0]} >= ((h0[39:32] > 8'd34) ? 8'd34 : h0[39:32]))
					st <= S_CDONE;
				else begin
					fq_req  <= 1; fq_line <= 2'd3;
					fq_addr <= line_q + 25'd8 + {18'd0, b0[5:0], 1'b0};
					st <= S_SPAN;
				end
			end else begin
				// spriteAtPosition: the valid fetches covering the pixel, in order, at most 4,
				// only where sprites are shown at the dot
				t_spr <= 1;
				fi    <= 0;
				ncov  <= 0;
				nfet  <= (h0[55:48] > 8'd64) ? 7'd64 : h0[54:48];
				if (!b0[6] || h0[55:48] == 8'd0)
					st <= lanes ? S_LN : S_CDONE;     // no captured sprite: the lanes alone
				else begin
					fq_req  <= 1; fq_line <= 2'd3;
					fq_addr <= line_q + 25'd80;
					st <= S_SPR;
				end
			end
		end
		S_SPAN: if (fq_ack) begin
			qa <= fq_q;
			fq_req  <= 1; fq_line <= 2'd3;
			fq_addr <= line_q + 25'd9 + {18'd0, b0[5:0], 1'b0};
			st <= S_SPANB;
		end
		S_SPANB: if (fq_ack) begin
			t_idx <= qa[19:4];                            // (CHR linear & 0xFFFFF) >> 4
			t_fp  <= game_ram ? fq_q : 64'd0;              // CHR-ROM: fingerprint of 16 zeros
			st    <= S_TM;
		end
		S_SPR: if (fq_ack) begin
			qa <= fq_q;
			if (fq_q[54] && px >= fq_q[45:38] && px - fq_q[45:38] <= 8'd7) begin
				fq_req  <= 1; fq_line <= 2'd3;
				fq_addr <= line_q + 25'd81 + {17'd0, fi, 1'b0};
				st <= S_SPRB;
			end else if (fi + 7'd1 == nfet)
				st <= lanes ? S_LN : S_CDONE;             // res is 0
			else begin
				fi <= fi + 7'd1;
				fq_req  <= 1; fq_line <= 2'd3;
				fq_addr <= line_q + 25'd82 + {17'd0, fi, 1'b0};
			end
		end
		S_SPRB: if (fq_ack) begin
			t_idx <= qa[35:20];
			t_fp  <= game_ram ? fq_q : 64'd0;
			ncov  <= ncov + 3'd1;
			st    <= S_TM;
		end

		// ---- tmatch (hdref.tmatch), or the end of frameRange ----
		S_TM: begin
			if (c_type == 8'd2) begin
				res <= rem[15:0] >= c1[31:16];
				st  <= S_CDONE;
			end else if (!tm_pal)
				st <= S_TMEND;                            // res stays 0 for this record
			else if (c_data) begin
				res <= t_fp == c2;
				st  <= S_TMEND;
			end else if (t_hi) begin                      // an added body beyond 16 bits: no
				res <= lbody == c2;                       // fallback entry, only the body itself
				st  <= S_TMEND;
			end else if ({48'd0, t_idx} == c2) begin
				res <= 1;
				st  <= S_TMEND;
			end else if ({16'd0, t_idx} < fb_len) begin
				// fallback(idx) == tile? (hdref.fallback: the map entry, -1 beyond its end)
				fq_req  <= 1; fq_line <= 2'd3;
				fq_addr <= fb_base + {10'd0, t_idx[15:1]};
				st <= S_FB;
			end else
				st <= S_TMEND;
		end
		S_FB: if (fq_ack) begin
			res <= {{32{t_idx[0] ? fq_q[63] : fq_q[31]}}, (t_idx[0] ? fq_q[63:32] : fq_q[31:0])} == c2;
			st  <= S_TMEND;
		end
		S_TMEND: begin
			// a tile condition is decided by its one record; a sprite condition by any of
			// the covering records - the captured ones, then the lanes (8a), four in all
			if (!t_spr || res || ncov == 3'd4)
				st <= S_CDONE;
			else if (t_lane) begin
				lj <= lj + 7'd1;
				st <= S_LN1;
			end else if (fi + 7'd1 == nfet)
				st <= lanes ? S_LN : S_CDONE;
			else begin
				fi <= fi + 7'd1;
				fq_req  <= 1; fq_line <= 2'd3;
				fq_addr <= line_q + 25'd82 + {17'd0, fi, 1'b0};
				st <= S_SPR;
			end
		end

		// ---- 8a: the lanes of the position's line covering it, in slot order ----
		S_LN: begin
			a_y    <= ay;
			lj     <= 0;
			t_lane <= 1;
			st     <= S_LNW;
		end
		S_LNW: st <= S_LN1;                           // a_q / a_n valid next clock
		S_LN1: begin
			if (lj >= a_n)
				st <= S_CDONE;                            // res is 0
			else if (px >= ln[7:0] && ln_d <= {5'd0, ln[10:8]}) begin
				d_rid <= ln[11 +: RAW];
				st    <= S_LNK;
			end else
				lj <= lj + 7'd1;
		end
		S_LNK: st <= S_LNK1;                          // d_key valid next clock
		S_LNK1: begin
			ncov  <= ncov + 3'd1;
			lpal  <= d_key[95:64];
			t_idx <= d_key[15:0];
			t_hi  <= d_key[63:16] != 48'd0;
			lbody <= d_key[63:0];
			t_fp  <= 64'd0;                           // a CHR-ROM key: fingerprint of zeros
			// CHR-RAM: no TileIndex, never a match; a body of -1 is NoTile (hdref.tmatch)
			st    <= (game_ram || d_key[63:0] == 64'hFFFF_FFFF_FFFF_FFFF) ? S_TMEND : S_TM;
		end

		// ---- a condition's final value ----
		S_CDONE: begin
			if (ev && c0[63:43] == 21'd0) begin
				mm_we <= 1; mm_wa <= c_memo; mm_wd <= {1'b1, res ^ c_neg};
			end
			// background selection fixes a rewrite-sensitive condition's memo: seed render's.
			// Render's memo holds the result BEFORE negation (it negates on read), so the seed
			// does too - unlike this module's own memo above.
			if (bgm && ev && c0[24]) begin
				if (c0[63:43] == 21'd0) begin
					sd_we <= 1; sd_wa <= {bk, c_memo}; sd_wd <= {1'b1, res};
				end else
					bg_warn <= 1;
			end
			if (!(ev ? res ^ c_neg : res)) begin
				if (!bgm) begin fl_we <= 1; fl_wa <= fl_h; fl_wd <= {1'b1, fptr[27:0], 1'b0, 1'b0}; end
				st    <= S_NEXT;                          // the rule fails
			end else if (clast) begin
				if (!bgm) begin fl_we <= 1; fl_wa <= fl_h; fl_wd <= {1'b1, fptr[27:0], 1'b1, later}; end
				st    <= S_KEEP;                          // every frame condition passed
			end else begin
				cp <= cp + 28'd4;
				st <= S_CW;
			end
		end

		// ---- keep the rule ----
		S_KEEP: begin
			if (bgm)
				st <= S_GSEL;                             // the level's first passing entry
			else if ({{(31-RSW){1'b0}}, rs_n} >= RS_MAX) begin // rule store full
				lkind    <= 2'd2;
				overflow <= 1;
				st       <= S_LEND;
			end else begin
				rs_we <= 1; rs_wa <= {bk, rs_n[RSW-1:0]};
				rs_wd <= {pfx, wptr[19:0], later ? fptr[19:0] : 20'd0, payload};
				rs_n   <= rs_n + 1'd1;
				lcount <= lcount + 11'd1;
				st     <= (wptr == 32'd0 && !later) ? S_LEND : S_NEXT;
			end
		end
		S_NEXT: begin
			if (bgm)
				st <= S_GF;                               // the entry failed
			else if (ri + 32'd1 == nrules)
				st <= S_LEND;
			else begin
				ri      <= ri + 32'd1;
				fq_req  <= 1; fq_line <= 2'd0; fq_addr <= lbase + 25'd3 + {ri[23:0], 1'b0};
				st      <= S_RA;
			end
		end

		// ---- the list is done: list table + record table ----
		S_LEND: begin
			lt_we <= 1; lt_wa <= h; lt_wd <= {1'b1, offq, lentry[RTW-1:25]};
			rt_we <= 1; rt_wa <= {bk, rid[RAW-1:0]}; rt_wd <= lentry;
			st    <= S_FIN;
		end
		S_FIN: begin
			rid <= rid + 1'd1;
			st  <= S_REC;
		end

		default: st <= S_IDLE;
	endcase

	if (reset) begin
		st <= S_IDLE; busy <= 0;
	end
end

endmodule
