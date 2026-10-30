// hd3_paint.sv - the stage-3 HD picture painted from a display list the ARM writes.
//
// The ARM (tools/armdec/hdarm4.c) takes every decision Mesen's HD pack renderer takes and turns
// them into a list of 64-bit paint commands per NES line, in Mesen's composite order. This module
// only executes the list: a later command paints over an earlier one where they overlap. It has
// no sprite lists, no rule walk, no condition logic.
//
// Command (u64, little-endian in DDR3):  [1:0] op  0 BLIT  1 FILL  2 FX  3 EOL
//   [9:2] x0  [17:10] len-1  [19:18] blend mode (0 alpha, 1 add, 2 subtract)
//   BLIT: [22:20] tile row offy  [23] vm  [24] hm  [27:25] offx of the first pixel  [51:28] block
//   FILL: [43:20] RGB (opaque)
//   FX:   [2] grayscale  [5:3] emphasis
// A BLIT paints len NES pixels of one tile row of one 16x16 RGBA block; that tile row is block
// bytes 128*offy .. +127 (both HD rows). Lines run from ov_top to 239 - ov_bottom, one EOL each.
//
// Structure (one clock domain, the HD clock):
//   fetch   reads the list into a 128-command FIFO in 8-qword bursts (reader client L) until it
//           has received the frame's last EOL.
//   front   looks each BLIT's tile row up in a 256-row direct-mapped cache (2 x 16 KB of block
//           rows) and queues the command. A miss refills the row in place (client B, two
//           bursts); it is held while any queued or painting command still uses that row.
//   paint   one NES pixel (2x2 HD pixels, four blend lanes) per clock into the line buffer being
//           painted. Two line buffers: one is painted while the other is read out.
//   output  hd3_render's grayscale/emphasis stage, one NES pixel every two clocks.
// Pacing is hd3_render's: a line is started only while line_ok, which the engine derives from
// line_y and the scan-out position.

module hd3_paint #(
	parameter QD = 8                         // paint queue depth
)
(
	input             clk,
	input             reset,
	input             start,                 // pulse: paint one frame
	input      [24:0] list_base,             // qword address of the frame's display list
	input      [24:0] list_end,              // one past the last qword the ARM has published;
	                                         // a burst is read only when all 8 of its qwords
	                                         // lie below it (the ARM publishes in multiples
	                                         // of 8, padding after the frame's last EOL)
	input      [24:0] blocks_base,           // qword address of the pack's block section
	input       [7:0] ov_top,
	input       [7:0] ov_bottom,
	output reg        busy = 0,
	output reg        done = 0,              // pulse: every line of the frame has been output

	// reader client L: the display list
	output reg        l_rd_req = 0,
	output reg [24:0] l_rd_addr = 0,
	input             l_rd_ack,
	input      [63:0] rd_data,
	input             l_rd_valid,

	// reader client B: block rows
	output reg        b_rd_req = 0,
	output reg [24:0] b_rd_addr = 0,
	input             b_rd_ack,
	input             b_rd_valid,

	input             line_ok,
	output      [7:0] line_y,

	output reg        out_v = 0,
	output reg  [7:0] out_y = 0,
	output reg  [7:0] out_x = 0,
	output reg [95:0] out_px = 0
);

// $readmemh paths as hd3_render: Quartus (and a Verilator build that runs from the project
// folder, HD3_HEX_PROJECT - the full-core simulation) resolve against the project folder; a
// unit testbench runs from rtl/hdpack. A wrong path only warns and leaves the ROM all zeros.
`ifdef VERILATOR
`ifndef HD3_HEX_PROJECT
`define HD3P_HEX_BARE
`endif
`endif
`ifdef HD3P_HEX_BARE
`define HD3P_HEX_ER  "hd3_emph_r.hex"
`define HD3P_HEX_EG  "hd3_emph_g.hex"
`define HD3P_HEX_EB  "hd3_emph_b.hex"
`else
`define HD3P_HEX_ER  "rtl/hdpack/hd3_emph_r.hex"
`define HD3P_HEX_EG  "rtl/hdpack/hd3_emph_g.hex"
`define HD3P_HEX_EB  "rtl/hdpack/hd3_emph_b.hex"
`endif

localparam [1:0] OP_BLIT = 2'd0, OP_FILL = 2'd1, OP_FX = 2'd2, OP_EOL = 2'd3;
localparam QW = $clog2(QD);

// ------------------------------------------------------------------------------------------
// state shared between the blocks
// ------------------------------------------------------------------------------------------
reg        run = 0;                          // a frame is in progress
reg  [7:0] f_last = 0;                       // last line of the frame
wire [7:0] f_lines = f_last - ov_top + 8'd1;

// command FIFO
reg [63:0] cq [128];
reg  [6:0] cq_wp = 0, cq_rp = 0;
reg  [7:0] cq_n = 0;
reg  [7:0] cf_res = 0;                       // beats requested and not yet received
reg  [7:0] cf_eol = 0;                       // EOLs received
reg        cf_stop = 1;
reg        cq_pop = 0;
reg [63:0] cq_q;

// block-row cache
reg [255:0] cready = 0;                      // the row's data is in the cache
reg        cr_set_v = 0;                     // set cready next clock (after the last write lands)
reg  [7:0] cr_set_s = 0;

// paint queue
reg  [63:0] pq_c [QD];
reg  [QD*8-1:0] pq_s = 0;                    // set of each entry, flat (compared in parallel)
reg  [QD-1:0] pq_b = 0;                      // entry is a BLIT (holds a cache row)
reg  [QW-1:0] pq_wp = 0, pq_rp = 0;
reg  [QW:0]   pq_n = 0;
reg           pq_push = 0;
reg  [63:0]   pq_pc = 0;
reg   [7:0]   pq_ps = 0;
reg           pq_pb = 0;
reg           pq_pop = 0;

// miss queue {set 8, key 27} and refill return queue
reg [34:0] mq [8];
reg  [2:0] mq_wp = 0, mq_rp = 0;
reg  [3:0] mq_n = 0;
reg        mq_push = 0, mq_pop = 0;
reg [34:0] mq_d = 0;
reg  [7:0] rq_s [8];
reg  [2:0] rq_wp = 0, rq_rp = 0;
reg  [3:0] rq_n = 0;

// painter
reg        pe_has = 0;
reg [63:0] pe_c = 0;
reg  [7:0] pe_set = 0;
reg        pe_blit = 0;
reg  [8:0] pe_j = 0;
reg        pb = 0;                           // buffer being painted
reg  [7:0] py = 0;                           // line being painted
reg        p_go = 0;                         // the line has been started (line_ok seen)
reg        p_fin = 1;                        // every line of the frame painted
reg  [1:0] ob_full = 0;                      // buffer holds a finished line to output
reg  [7:0] ob_y [2];
reg  [3:0] ob_fx [2];                        // {emph 3, gray}
reg  [3:0] p_fx = 0;
reg        s1_v = 0;
reg        s2_v = 0, s3_v = 0;                   // the blend stages (see s2/s3 below)

// output
reg        ob = 0;                           // buffer being output
reg  [8:0] oe_n = 0;
reg  [7:0] oe_x = 0;
reg        oe_rd1 = 0;
reg  [7:0] oe_x1 = 0;
reg        oe_release = 0;
reg  [7:0] o_done = 0;

assign line_y = py;

// ==========================================================================================
// fetch: the list into the command FIFO
// ==========================================================================================
always @(posedge clk) begin
	if (l_rd_valid) cq[cq_wp] <= rd_data;
	cq_q <= cq[cq_rp];
end

always @(posedge clk) begin
	if (l_rd_valid) cq_wp <= cq_wp + 7'd1;
	if (cq_pop)     cq_rp <= cq_rp + 7'd1;
	cq_n <= cq_n + {7'd0, l_rd_valid} - {7'd0, cq_pop};
	if (l_rd_valid && rd_data[1:0] == OP_EOL) cf_eol <= cf_eol + 8'd1;
	if (l_rd_ack) begin
		l_rd_req  <= 0;
		l_rd_addr <= l_rd_addr + 25'd8;
		cf_res    <= cf_res + 8'd8 - {7'd0, l_rd_valid};
	end else begin
		cf_res <= cf_res - {7'd0, l_rd_valid};
		if (run && !cf_stop && !l_rd_req && ({1'b0, cq_n} + {1'b0, cf_res}) <= 9'd120 &&
		    {1'b0, l_rd_addr} + 26'd8 <= {1'b0, list_end})
			l_rd_req <= 1;
	end
	if (run && cf_eol >= f_lines) cf_stop <= 1;
	if (start) begin
		cq_wp <= 0; cq_rp <= 0; cq_n <= 0; cf_eol <= 0; cf_stop <= 0;
		l_rd_addr <= list_base;
	end
	if (reset) begin
		l_rd_req <= 0; cf_res <= 0; cq_wp <= 0; cq_rp <= 0; cq_n <= 0; cf_eol <= 0; cf_stop <= 1;
	end
end

// ==========================================================================================
// block-row cache: HD row 0 and HD row 1 of each cached tile row, 256 rows x 8 qwords;
// qword q holds HD pixels 2q (bits 31:0) and 2q+1 (bits 63:32), each {A, B, G, R}
// ==========================================================================================
reg [63:0] crow0 [2048];
reg [63:0] crow1 [2048];
reg [27:0] ctag  [256];                      // {valid, block 24, offy 3}
reg  [7:0] ctag_a = 0;
reg [27:0] ctag_q;
reg        ctag_we = 0;
reg  [7:0] ctag_wa = 0;
reg [27:0] ctag_wd = 0;
always @(posedge clk) begin
	ctag_q <= ctag[ctag_a];
	if (ctag_we) ctag[ctag_wa] <= ctag_wd;
end

reg        cw_v = 0;
reg [10:0] cw_a = 0;
reg        cw_row = 0;
reg [63:0] cw_d = 0;
always @(posedge clk) if (cw_v) begin
	if (cw_row) crow1[cw_a] <= cw_d; else crow0[cw_a] <= cw_d;
end

// a set is in use while a queued or painting BLIT holds it (front end's set only)
reg  [7:0] fe_set = 0;
reg        fe_busy;
always @(*) begin : busyck
	integer i;
	fe_busy = pe_has && pe_blit && pe_set == fe_set;
	for (i = 0; i < QD; i = i + 1)
		if (pq_b[i] && pq_s[i*8 +: 8] == fe_set) fe_busy = 1'b1;
end

// ==========================================================================================
// front end
// ==========================================================================================
localparam [2:0] F_CLR = 3'd0, F_IDLE = 3'd1, F_CMD = 3'd2, F_TAG = 3'd3, F_DEC = 3'd4;
reg  [2:0] fst = F_CLR;
reg  [8:0] fclr = 0;
reg [63:0] fe_c = 0;
reg [26:0] fe_key = 0;
reg  [7:0] fe_eol = 0;                       // EOLs passed on this frame

wire [26:0] key_of = {cq_q[51:28], cq_q[22:20]};
wire [26:0] kh     = key_of ^ (key_of >> 7);
wire        fe_hit  = ctag_q[27] && ctag_q[26:0] == fe_key;
wire        fe_miss = fst == F_DEC && !fe_hit && !fe_busy && mq_n < 4'd7;

always @(posedge clk) begin
	cq_pop  <= 0;
	ctag_we <= 0;
	pq_push <= 0;
	mq_push <= 0;
	case (fst)
	F_CLR: begin
		ctag_we <= 1; ctag_wa <= fclr[7:0]; ctag_wd <= 28'd0;
		fclr <= fclr + 9'd1;
		if (fclr == 9'd255) fst <= F_IDLE;
	end
	F_IDLE:
		// cq_q is the FIFO head (registered read of cq_rp, written at least a clock earlier)
		if (run && fe_eol != f_lines && cq_n != 0 && !cq_pop && pq_n < QD - 1) fst <= F_CMD;
	F_CMD: begin
		fe_c   <= cq_q;
		cq_pop <= 1;
		if (cq_q[1:0] == OP_BLIT) begin
			fe_key <= key_of;
			fe_set <= kh[7:0];
			ctag_a <= kh[7:0];
			fst    <= F_TAG;
		end else begin
			pq_push <= 1; pq_pc <= cq_q; pq_ps <= 8'd0; pq_pb <= 0;
			if (cq_q[1:0] == OP_EOL) fe_eol <= fe_eol + 8'd1;
			fst <= F_IDLE;
		end
	end
	F_TAG: fst <= F_DEC;                     // ctag_q is valid next clock
	F_DEC:
		if (fe_hit) begin
			pq_push <= 1; pq_pc <= fe_c; pq_ps <= fe_set; pq_pb <= 1;
			fst <= F_IDLE;
		end else if (fe_miss) begin
			ctag_we <= 1; ctag_wa <= fe_set; ctag_wd <= {1'b1, fe_key};
			mq_push <= 1; mq_d <= {fe_set, fe_key};
			pq_push <= 1; pq_pc <= fe_c; pq_ps <= fe_set; pq_pb <= 1;
			fst <= F_IDLE;
		end
	default: fst <= F_IDLE;
	endcase
	if (start) fe_eol <= 0;
	if (reset) begin fst <= F_CLR; fclr <= 0; fe_eol <= 0; end
end

// paint queue bookkeeping
always @(posedge clk) begin
	if (pq_push) begin
		pq_c[pq_wp] <= pq_pc; pq_s[pq_wp*8 +: 8] <= pq_ps;
		pq_wp <= pq_wp + 1'b1;
	end
	if (pq_push) pq_b[pq_wp] <= pq_pb;
	if (pq_pop)  pq_b[pq_rp] <= 1'b0;         // never the same entry as the push (QD-1 cap)
	if (pq_pop)  pq_rp <= pq_rp + 1'b1;
	pq_n <= pq_n + {{QW{1'b0}}, pq_push} - {{QW{1'b0}}, pq_pop};
	if (reset) begin pq_wp <= 0; pq_rp <= 0; pq_n <= 0; pq_b <= 0; end
end

// ==========================================================================================
// refill (client B): two bursts per miss; this client's beats return in issue order
// ==========================================================================================
reg        bi_second = 0;                    // the first burst of the miss-queue head is issued
reg  [3:0] rb_cnt = 0;                       // beats of the head pair received
wire [34:0] mq_h = mq[mq_rp];
wire [26:0] mq_hk = mq_h[26:0];
wire [24:0] mq_addr = blocks_base + {mq_hk[20:3], 7'd0} + {18'd0, mq_hk[2:0], 4'd0};

always @(posedge clk) begin
	mq_pop   <= 0;
	cw_v     <= 0;
	cr_set_v <= 0;
	if (mq_push) begin mq[mq_wp] <= mq_d; mq_wp <= mq_wp + 3'd1; end
	mq_n <= mq_n + {3'd0, mq_push} - {3'd0, mq_pop};

	// issue: the return queue entry is pushed with the FIRST burst (its beats may come back
	// before the second burst is even acknowledged)
	if (b_rd_ack) begin
		b_rd_req <= 0;
		if (!bi_second) begin
			bi_second <= 1;
			rq_s[rq_wp] <= mq_h[34:27]; rq_wp <= rq_wp + 3'd1;
		end else begin
			bi_second <= 0;
			mq_pop <= 1; mq_rp <= mq_rp + 3'd1;
		end
	end else if (!b_rd_req && !mq_pop && mq_n != 0 && (bi_second || rq_n < 4'd6)) begin
		b_rd_req  <= 1;
		b_rd_addr <= bi_second ? mq_addr + 25'd8 : mq_addr;
	end

	// return
	if (b_rd_valid) begin
		cw_v   <= 1;
		cw_row <= rb_cnt[3];
		cw_a   <= {rq_s[rq_rp], rb_cnt[2:0]};
		cw_d   <= rd_data;
		rb_cnt <= rb_cnt + 4'd1;
		if (rb_cnt == 4'd15) begin
			cr_set_v <= 1; cr_set_s <= rq_s[rq_rp];
			rq_rp <= rq_rp + 3'd1;
		end
	end
	rq_n <= rq_n + {3'd0, (b_rd_ack && !bi_second)} - {3'd0, (b_rd_valid && rb_cnt == 4'd15)};
	// the last qword is written the clock after cw_v: ready one clock after that
	if (cr_set_v) cready[cr_set_s] <= 1'b1;
	if (fe_miss) cready[fe_set] <= 1'b0;
	if (reset) begin
		b_rd_req <= 0; bi_second <= 0; mq_wp <= 0; mq_rp <= 0; mq_n <= 0;
		rq_wp <= 0; rq_rp <= 0; rq_n <= 0; rb_cnt <= 0; cready <= 0;
	end
end

// ==========================================================================================
// line buffers: 2 buffers x 4 lanes {r0c0, r0c1, r1c0, r1c1}, each 256 x RGB
// ==========================================================================================
wire  [7:0] pe_x;
reg   [7:0] lbr_a [2];
wire [23:0] lbr_q [2][4];
reg         lbw_v = 0;
reg         lbw_b = 0;
reg   [7:0] lbw_a = 0;
reg  [95:0] lbw_d = 0;
// a buffer is read by the output while it holds a finished line, otherwise by the painter
always @(*) begin
	lbr_a[0] = (ob == 1'b0 && ob_full[0]) ? oe_x : pe_x;
	lbr_a[1] = (ob == 1'b1 && ob_full[1]) ? oe_x : pe_x;
end
genvar gb, gl;
generate
	for (gb = 0; gb < 2; gb = gb + 1) begin : lbb
		for (gl = 0; gl < 4; gl = gl + 1) begin : lbl
			reg [23:0] m [256];
			reg [23:0] q;
			always @(posedge clk) begin
				q <= m[lbr_a[gb]];
				if (lbw_v && lbw_b == gb) m[lbw_a] <= lbw_d[gl*24 +: 24];
			end
			assign lbr_q[gb][gl] = q;
		end
	end
endgenerate

// ==========================================================================================
// paint
// ==========================================================================================
wire [1:0] pe_op   = pe_c[1:0];
wire [7:0] pe_x0   = pe_c[9:2];
wire [8:0] pe_len  = {1'b0, pe_c[17:10]} + 9'd1;
wire [1:0] pe_mode = pe_c[19:18];
wire       pe_vm   = pe_c[23];
wire       pe_hm   = pe_c[24];
wire [2:0] pe_ox0  = pe_c[27:25];
wire [2:0] pe_offx = pe_ox0 + pe_j[2:0];
wire [2:0] pe_col  = pe_hm ? ~pe_offx : pe_offx;
assign     pe_x    = pe_x0 + pe_j[7:0];

reg  [7:0] s1_x = 0;
reg        s1_fill = 0;
reg [23:0] s1_rgb = 0;
reg  [1:0] s1_mode = 0;
reg        s1_vm = 0, s1_hm = 0;
reg [63:0] c0_q, c1_q;
always @(posedge clk) begin
	c0_q <= crow0[{pe_set, pe_col}];
	c1_q <= crow1[{pe_set, pe_col}];
end

wire pe_ready = pe_has && (pe_op == OP_FILL || (pe_op == OP_BLIT && cready[pe_set]));
wire pe_issue = pe_ready && pe_j < pe_len;

always @(posedge clk) begin
	pq_pop <= 0;
	s1_v   <= 0;
	// next command: line started, and the previous span's writes landed (a new span may
	// read pixels the last one is still writing: 4 clocks from issue to write)
	if (!pe_has && !pq_pop && pq_n != 0 && p_go && !s1_v && !s2_v && !s3_v) begin
		pe_c    <= pq_c[pq_rp];
		pe_set  <= pq_s[pq_rp*8 +: 8];
		pe_blit <= pq_c[pq_rp][1:0] == OP_BLIT;
		pe_j    <= 0;
		pe_has  <= 1;
		pq_pop  <= 1;
	end
	if (pe_has) begin
		case (pe_op)
		OP_BLIT, OP_FILL:
			if (pe_issue) begin
				s1_v    <= 1;
				s1_x    <= pe_x;
				s1_fill <= pe_op == OP_FILL;
				s1_rgb  <= pe_c[43:20];
				s1_mode <= pe_mode;
				s1_vm   <= pe_vm;
				s1_hm   <= pe_hm;
				pe_j    <= pe_j + 9'd1;
				if (pe_j + 9'd1 == pe_len) pe_has <= 0;
			end
		OP_FX: begin p_fx <= {pe_c[5:3], pe_c[2]}; pe_has <= 0; end
		OP_EOL:
			// (the command-start guard already holds the EOL back until s1-s3 are empty, so
			// only lbw_v can still be set here; s1_v-s3_v stay as defence)
			if (!s1_v && !s2_v && !s3_v && !lbw_v) begin
				ob_full[pb] <= 1'b1; ob_y[pb] <= py; ob_fx[pb] <= p_fx;
				pb <= ~pb; p_fx <= 0; p_go <= 0; pe_has <= 0;
				if (py == f_last) p_fin <= 1; else py <= py + 8'd1;
			end
		endcase
	end
	if (run && !p_fin && !p_go && !pe_has && line_ok && !ob_full[pb]) p_go <= 1;
	if (oe_release) ob_full[ob] <= 1'b0;
	if (start) begin
		pb <= 0; py <= ov_top; p_go <= 0; p_fin <= 0; p_fx <= 0;
		f_last <= 8'd239 - ov_bottom;
	end
	if (reset) begin pe_has <= 0; s1_v <= 0; p_go <= 0; p_fin <= 1; ob_full <= 0; end
end

// blend one channel (p premultiplied source, o destination, a source alpha) in two clocks:
// bprep does the multiply and the alpha tests, bfin the mode select. In one clock (12 of
// these 8x8 multiplies in logic - the DSP blocks are full) it was -2.3 ns (2026-10-29).
// bprep = {a == FF, a == 0, ((256 - a) & 0xFF) * o >> 8, o, p}
function automatic [25:0] bprep(input [7:0] p, input [7:0] o, input [7:0] a);
	reg [15:0] t;
	begin
		t = {8'd0, 8'd0 - a} * {8'd0, o};
		bprep = {a == 8'hFF, a == 8'd0, t[15:8], o, p};
	end
endfunction
function automatic [7:0] bfin(input [25:0] q, input [1:0] m);
	reg [7:0] p, o;
	reg [8:0] s;
	begin
		p = q[7:0]; o = q[15:8];
		s = {1'b0, p} + {1'b0, o};
		case (m)
		2'd1:    bfin = s[8] ? 8'hFF : s[7:0];
		2'd2:    bfin = (o > p) ? o - p : 8'd0;
		default: bfin = q[25] ? p : q[24] ? o : p + q[23:16];
		endcase
	end
endfunction
// one pixel, s = {A, B, G, R}, o = {R, G, B}: the three channels' bprep, R first
function automatic [77:0] bpxprep(input [31:0] s, input [23:0] o);
	bpxprep = {bprep(s[7:0], o[23:16], s[31:24]), bprep(s[15:8], o[15:8], s[31:24]),
	           bprep(s[23:16], o[7:0], s[31:24])};
endfunction
function automatic [23:0] bpxfin(input [77:0] q, input [1:0] m);
	bpxfin = {bfin(q[77:52], m), bfin(q[51:26], m), bfin(q[25:0], m)};
endfunction

// s2: both RAM reads registered before the blend - the unregistered M10K output, the 8x8
// multiply and the mode select in one clk_f was -4.1 ns (2026-10-29). pb cannot change while
// a span is in flight (EOL waits for s1-s3/lbw to empty), so lbw_b may take it here.
reg  [7:0] s2_x = 0;
reg        s2_fill = 0;
reg [23:0] s2_rgb = 0;
reg  [1:0] s2_mode = 0;
reg        s2_vm = 0, s2_hm = 0;
reg [63:0] c0_r, c1_r;
reg [23:0] d_r [4];
always @(posedge clk) begin
	s2_v    <= s1_v;
	s2_x    <= s1_x;
	s2_fill <= s1_fill;
	s2_rgb  <= s1_rgb;
	s2_mode <= s1_mode;
	s2_vm   <= s1_vm;
	s2_hm   <= s1_hm;
	c0_r    <= c0_q;
	c1_r    <= c1_q;
	d_r[0]  <= lbr_q[pb][0];
	d_r[1]  <= lbr_q[pb][1];
	d_r[2]  <= lbr_q[pb][2];
	d_r[3]  <= lbr_q[pb][3];
	if (reset) s2_v <= 0;
end

wire [63:0] hr0 = s2_vm ? c1_r : c0_r;       // HD row 0 of the NES pixel
wire [63:0] hr1 = s2_vm ? c0_r : c1_r;
wire [31:0] w00 = s2_hm ? hr0[63:32] : hr0[31:0];
wire [31:0] w01 = s2_hm ? hr0[31:0]  : hr0[63:32];
wire [31:0] w10 = s2_hm ? hr1[63:32] : hr1[31:0];
wire [31:0] w11 = s2_hm ? hr1[31:0]  : hr1[63:32];

// s3: the multiplies and alpha tests, registered; the mode select is the next clock
reg  [7:0] s3_x = 0;
reg        s3_fill = 0;
reg [23:0] s3_rgb = 0;
reg  [1:0] s3_mode = 0;
reg [77:0] s3_q [4];                          // pixels w00, w01, w10, w11
always @(posedge clk) begin
	s3_v    <= s2_v;
	s3_x    <= s2_x;
	s3_fill <= s2_fill;
	s3_rgb  <= s2_rgb;
	s3_mode <= s2_mode;
	s3_q[0] <= bpxprep(w00, d_r[0]);
	s3_q[1] <= bpxprep(w01, d_r[1]);
	s3_q[2] <= bpxprep(w10, d_r[2]);
	s3_q[3] <= bpxprep(w11, d_r[3]);
	if (reset) s3_v <= 0;
end

always @(posedge clk) begin
	lbw_v <= s3_v;
	lbw_b <= pb;
	lbw_a <= s3_x;
	lbw_d <= s3_fill ? {4{s3_rgb}}
	                 : {bpxfin(s3_q[3], s3_mode), bpxfin(s3_q[2], s3_mode),
	                    bpxfin(s3_q[1], s3_mode), bpxfin(s3_q[0], s3_mode)};
	if (reset) lbw_v <= 0;
end

// ==========================================================================================
// output: the finished buffer through grayscale/emphasis to out_*
// ==========================================================================================
localparam OF_LOG2 = 3;
localparam integer OF_D = 1 << OF_LOG2;
localparam [OF_LOG2:0] OF_LIM = OF_D[OF_LOG2:0] - 2;   // stop issuing two entries short of full
reg  [115:0] of_m [OF_D];
reg  [OF_LOG2-1:0] of_wp = 0, of_rp = 0;
reg  [OF_LOG2:0]   of_n = 0;
reg          of_push = 0;
reg  [115:0] of_d = 0;
reg          os1 = 0, os2 = 0, os3 = 0;
wire [115:0] of_h = of_m[of_rp];
wire         of_pop = !os1 && of_n != 0;
wire         out_idle = of_n == 0 && !of_push && !os1 && !os2 && !os3;

always @(posedge clk) begin
	oe_rd1     <= 0;
	of_push    <= 0;
	oe_release <= 0;
	if (ob_full[ob] && oe_n < 9'd256 && !oe_rd1 && !oe_release && of_n < OF_LIM) begin
		oe_rd1 <= 1;
		oe_x1  <= oe_x;
		oe_x   <= oe_x + 8'd1;
		oe_n   <= oe_n + 9'd1;
	end
	if (oe_rd1) begin
		of_push <= 1;
		of_d <= {ob_fx[ob][3:1], ob_fx[ob][0], ob_y[ob], oe_x1,
		         lbr_q[ob][3], lbr_q[ob][2], lbr_q[ob][1], lbr_q[ob][0]};
	end
	if (ob_full[ob] && oe_n == 9'd256 && !oe_rd1 && !oe_release) oe_release <= 1;
	if (oe_release) begin
		ob <= ~ob; oe_n <= 0; oe_x <= 0;
		o_done <= o_done + 8'd1;
	end
	if (start) begin ob <= 0; oe_n <= 0; oe_x <= 0; o_done <= 0; end
	if (reset) begin oe_n <= 0; oe_x <= 0; ob <= 0; end
end

reg   [7:0] rom_er [2048];
reg   [7:0] rom_eg [2048];
reg   [7:0] rom_eb [2048];
initial begin
	$readmemh(`HD3P_HEX_ER, rom_er);
	$readmemh(`HD3P_HEX_EG, rom_eg);
	$readmemh(`HD3P_HEX_EB, rom_eb);
end
reg  [10:0] er_a = 0, eg_a = 0, eb_a = 0, er_b = 0, eg_b = 0, eb_b = 0;
reg   [7:0] er_q, eg_q, eb_q, er_r, eg_r, eb_r;
always @(posedge clk) begin
	er_q <= rom_er[er_a]; er_r <= rom_er[er_b];
	eg_q <= rom_eg[eg_a]; eg_r <= rom_eg[eg_b];
	eb_q <= rom_eb[eb_a]; eb_r <= rom_eb[eb_b];
end

function [7:0] avg3(input [23:0] c);
	reg  [9:0] s;
	reg [19:0] m;
	begin
		s = {2'b0, c[23:16]} + {2'b0, c[15:8]} + {2'b0, c[7:0]};
		m = s * 10'd683;
		avg3 = m[18:11];
	end
endfunction

reg  [95:0] o_in1 = 0;
reg  [15:0] o_yx1 = 0, o_yx2 = 0, o_yx3 = 0;
reg   [2:0] o_emph1 = 0;
reg         o_gray1 = 0;
reg  [47:0] o_res = 0;

always @(posedge clk) begin
	out_v <= 0;
	os1 <= of_pop; os2 <= os1; os3 <= os2;
	if (of_push) begin of_m[of_wp] <= of_d; of_wp <= of_wp + 1'b1; end
	if (of_pop) of_rp <= of_rp + 1'b1;
	of_n <= of_n + {{OF_LOG2{1'b0}}, of_push} - {{OF_LOG2{1'b0}}, of_pop};
	if (of_pop) begin
		er_a <= {of_h[115:113], of_h[112] ? avg3(of_h[23:0])  : of_h[23:16]};
		eg_a <= {of_h[115:113], of_h[112] ? avg3(of_h[23:0])  : of_h[15:8]};
		eb_a <= {of_h[115:113], of_h[112] ? avg3(of_h[23:0])  : of_h[7:0]};
		er_b <= {of_h[115:113], of_h[112] ? avg3(of_h[47:24]) : of_h[47:40]};
		eg_b <= {of_h[115:113], of_h[112] ? avg3(of_h[47:24]) : of_h[39:32]};
		eb_b <= {of_h[115:113], of_h[112] ? avg3(of_h[47:24]) : of_h[31:24]};
		o_in1 <= of_h[95:0]; o_yx1 <= of_h[111:96];
		o_emph1 <= of_h[115:113]; o_gray1 <= of_h[112];
	end else if (os1) begin
		er_a <= {o_emph1, o_gray1 ? avg3(o_in1[71:48]) : o_in1[71:64]};
		eg_a <= {o_emph1, o_gray1 ? avg3(o_in1[71:48]) : o_in1[63:56]};
		eb_a <= {o_emph1, o_gray1 ? avg3(o_in1[71:48]) : o_in1[55:48]};
		er_b <= {o_emph1, o_gray1 ? avg3(o_in1[95:72]) : o_in1[95:88]};
		eg_b <= {o_emph1, o_gray1 ? avg3(o_in1[95:72]) : o_in1[87:80]};
		eb_b <= {o_emph1, o_gray1 ? avg3(o_in1[95:72]) : o_in1[79:72]};
	end
	if (os1) o_yx2 <= o_yx1;
	if (os2) begin o_res <= {er_r, eg_r, eb_r, er_q, eg_q, eb_q}; o_yx3 <= o_yx2; end
	if (os3) begin
		out_v  <= 1;
		out_y  <= o_yx3[15:8];
		out_x  <= o_yx3[7:0];
		out_px <= {er_r, eg_r, eb_r, er_q, eg_q, eb_q, o_res};
	end
	if (reset) begin os1 <= 0; os2 <= 0; os3 <= 0; of_wp <= 0; of_rp <= 0; of_n <= 0; end
end

// ==========================================================================================
// frame control
// ==========================================================================================
always @(posedge clk) begin
	done <= 0;
	if (start) begin run <= 1; busy <= 1; end
	else if (run && p_fin && o_done == f_lines && out_idle && cf_res == 0 && !l_rd_req
	         && pq_n == 0 && mq_n == 0 && rq_n == 0 && !b_rd_req) begin
		run <= 0; busy <= 0; done <= 1;
	end
	if (reset) begin run <= 0; busy <= 0; end
end

endmodule
