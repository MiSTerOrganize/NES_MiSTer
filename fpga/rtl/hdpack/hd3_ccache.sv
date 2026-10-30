// hd3_ccache.sv - condition-record cache in front of one DDR3 read port (HDPACK_DESIGN.md
// "hd3_render"). A condition record is 32 bytes (4 qwords, section 2); a cache line is one
// DDR3 burst = 2 records. Direct-mapped on the record index, 2^LB lines; the pack's condition
// records never change while it is loaded, so lines are only invalidated by `flush`.
//
// `req` (one clock) with record index `ci`; `ack` (one clock) with the whole record in q.
// Hit: 2 clocks. Miss: one burst, then the answer. Memories: four lanes (qword 0..3 of a
// record), one write port each, registered read.

module hd3_ccache #(
	parameter LB = 8                     // 2^LB lines = 2^(LB+1) records
)
(
	input             clk,
	input             reset,
	input             flush,
	input      [27:3] base,              // section 2 (condition records), qwords

	input             req,
	input      [23:0] ci,
	output reg        ack = 0,
	output reg [255:0] q,                // {q3, q2, q1, q0}

	output reg        rd_req = 0,
	output reg [27:3] rd_addr,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid
);

localparam TW = 23 - LB;                 // tag: record index bits above (line, half)

reg  [TW-1:0] tag [1 << LB];
reg  [TW-1:0] tag_q;
reg  [LB-1:0] tag_wa;
reg  [TW-1:0] tag_wd;
reg           tag_we;

reg  [23:0] ci_r;
wire [LB-1:0] set_r = ci_r[LB:1];
wire [TW-1:0] tag_r = ci_r[23:LB+1];

// declared before the tag read uses them (Quartus / ModelSim require declaration before use)
localparam S_IDLE = 0, S_CMP = 2, S_FILL = 3, S_HIT = 1;
reg  [1:0] st = S_IDLE;

always @(posedge clk) begin
	if (tag_we) tag[tag_wa] <= tag_wd;
	tag_q <= tag[(st == S_IDLE) ? ci[LB:1] : set_r];   // live index while idle
end

// lanes: entry {set, half}
reg  [LB:0]  ln_ra, ln_wa;
reg  [63:0]  ln_wd;
reg   [3:0]  ln_we;
wire [255:0] ln_q;
genvar gi;
generate for (gi = 0; gi < 4; gi = gi + 1) begin : lane
	reg [63:0] m [2 << LB];
	reg [63:0] o;
	always @(posedge clk) begin
		if (ln_we[gi]) m[ln_wa] <= ln_wd;
		o <= m[ln_ra];
	end
	assign ln_q[64 * gi +: 64] = o;
end endgenerate

reg  [(1 << LB)-1:0] vld = 0;

reg  [2:0] beat;

always @(*) ln_ra = (st == S_IDLE) ? ci[LB:0] : ci_r[LB:0];

always @(posedge clk) begin
	ack    <= 0;
	tag_we <= 0;
	ln_we  <= 0;
	if (rd_ack) rd_req <= 0;

	case (st)
	S_IDLE: if (req) begin
		ci_r <= ci;
		st   <= S_CMP;                    // tag_q, lane data for ci now loading
	end
	S_CMP: begin
		if (vld[set_r] && tag_q == tag_r) begin
			q   <= ln_q;
			ack <= 1;
			st  <= S_IDLE;
		end else begin
			rd_req  <= 1;
			rd_addr <= base + {ci_r[22:1], 3'd0};          // record pair = 8 qwords
			beat    <= 0;
			st      <= S_FILL;           // (one request at a time: the line is unread until filled)
		end
	end
	S_FILL: if (rd_valid) begin
		ln_we[beat[1:0]] <= 1;
		ln_wa <= {set_r, beat[2]};
		ln_wd <= rd_data;
		if (beat[2] == ci_r[0]) q[64 * beat[1:0] +: 64] <= rd_data;   // the requested record
		beat  <= beat + 3'd1;
		if (beat == 3'd7) begin
			tag_we <= 1;
			tag_wa <= set_r;
			tag_wd <= tag_r;
			vld[set_r] <= 1;
			st  <= S_HIT;
		end
	end
	// answer one clock after the last fill beat, once its lane and tag writes have landed: a
	// request captured right after the ack then reads the filled line, not the old contents
	S_HIT: begin
		ack <= 1;
		st  <= S_IDLE;
	end
	default: st <= S_IDLE;
	endcase

	if (flush) vld <= 0;
	if (reset) begin
		st <= S_IDLE; rd_req <= 0; vld <= 0;
	end
end

endmodule
