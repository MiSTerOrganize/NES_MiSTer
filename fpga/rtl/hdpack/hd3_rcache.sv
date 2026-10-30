// hd3_rcache.sv - read-only qword cache in front of one DDR3 read port (HDPACK_DESIGN.md
// "hd3_render"). Direct-mapped, 2^LB lines of 8 qwords (one DDR3 burst each). The renderer
// reads pack data through it - condition records, condition-list words and, on the slow path,
// rule records - which never change while a pack is loaded, so a line is only invalidated by
// `flush` (pack load).
//
// One request at a time: `req` (one clock) with `addr`; `ack` (one clock) with `q` later.
// A hit answers in 2 clocks; a miss fills the line (one burst) and then answers.
// `byp` with `req`: an uncached read for data that changes between frames (the renderer's
// frame-record reads, 8k) - always one burst, answered from the beat, and neither the line's
// data nor its tag is written, so the cached pack lines are untouched.
// Memories: one write port, registered read (block RAM shape); the valid bits are flops so a
// flush is a single clock.

module hd3_rcache #(
	parameter LB = 8                     // 2^LB lines
)
(
	input             clk,
	input             reset,
	input             flush,

	input             req,
	input             byp,
	input      [27:3] addr,
	output reg        ack = 0,
	output reg [63:0] q,

	output reg        rd_req = 0,
	output reg [27:3] rd_addr,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid
);

localparam TW = 25 - 3 - LB;             // tag: qword address bits above line and word

reg  [TW-1:0] tag  [1 << LB];
reg  [TW-1:0] tag_q;
reg  [LB-1:0] tag_wa;
reg  [TW-1:0] tag_wd;
reg           tag_we;
// declared before the tag read uses them (Quartus / ModelSim require declaration before use)
reg  [27:3] addr_r;
localparam S_IDLE = 0, S_CMP = 2, S_FILL = 3, S_HIT = 4;
reg  [2:0] st = S_IDLE;
always @(posedge clk) begin
	if (tag_we) tag[tag_wa] <= tag_wd;
	tag_q <= tag[(st == S_IDLE) ? addr[LB+5:6] : addr_r[LB+5:6]];   // live address while idle
end

reg  [63:0] dat [8 << LB];
reg  [LB+2:0] dat_ra, dat_wa;
reg  [63:0] dat_wd, dat_q;
reg         dat_we;
always @(posedge clk) begin
	if (dat_we) dat[dat_wa] <= dat_wd;
	dat_q <= dat[dat_ra];
end

reg  [(1 << LB)-1:0] vld = 0;

reg         byp_r = 0;
wire [LB-1:0] set_r  = addr_r[LB+5:6];
wire [TW-1:0] tag_r  = addr_r[27:LB+6];

reg  [2:0] beat;

always @(*) dat_ra = st == S_IDLE ? {addr[LB+5:3]} : {addr_r[LB+5:3]};

always @(posedge clk) begin
	ack    <= 0;
	tag_we <= 0;
	dat_we <= 0;
	if (rd_ack) rd_req <= 0;

	case (st)
	S_IDLE: if (req) begin
		addr_r <= addr;
		byp_r  <= byp;
		st     <= S_CMP;                  // tag_q, dat_q for addr now loading
	end
	S_CMP: begin
		if (!byp_r && vld[set_r] && tag_q == tag_r) begin
			q   <= dat_q;
			ack <= 1;
			st  <= S_IDLE;
		end else begin
			rd_req  <= 1;
			rd_addr <= {addr_r[27:6], 3'd0};
			beat    <= 0;
			st      <= S_FILL;           // (one request at a time: the line is unread until filled)
		end
	end
	S_FILL: if (rd_valid) begin
		dat_we <= !byp_r;
		dat_wa <= {set_r, beat};
		dat_wd <= rd_data;
		if (beat == addr_r[5:3]) q <= rd_data;
		beat <= beat + 3'd1;
		if (beat == 3'd7) begin
			tag_we <= !byp_r;
			tag_wa <= set_r;
			tag_wd <= tag_r;
			if (!byp_r) vld[set_r] <= 1;
			st     <= S_HIT;
		end
	end
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
