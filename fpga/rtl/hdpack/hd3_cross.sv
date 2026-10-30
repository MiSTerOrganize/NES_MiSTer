// hd3_cross.sv - the core-clock / HD-clock crossings of the stage-3 HD engine.
//
// clk is the NES core clock (21.477 MHz) and clk_f the HD engine clock (85.909 MHz); both come
// from the same PLL with no phase shift and clk_f is exactly 4 x clk, so every edge of clk is
// also an edge of clk_f. The two are therefore RELATED clocks: Quartus times every path between
// them (derive_pll_clocks, as it already does for the SDRAM controller on clk_f), there is no
// metastability to filter, and a clk register read on clk_f is simply the value from the last
// clk edge. What still needs care is rate: a one-clock pulse on clk lasts four clk_f clocks, and
// a one-clock pulse on clk_f can fall between two clk edges. Both directions therefore carry
// events as toggles: the sender flips a bit, the receiver compares it with its own copy.

// hd3_xq: one entry from a valid/ready source on clk to a valid/ready sink on clk_f.
// i_ready is high while the entry is free; a qword taken on clk is offered on clk_f until the
// sink takes it, and the entry is free again from the next clk edge. One entry per clk at most -
// the capture writer and the pack loader both produce at most one qword per clk.
module hd3_xq #(parameter W = 97)
(
	input              clk,
	input              i_valid,
	input      [W-1:0] i_data,
	output             i_ready,

	input              clk_f,
	output             o_valid,
	output     [W-1:0] o_data,
	input              o_ready
);

reg [W-1:0] hold;
reg         put_t = 0;                  // clk: flips when an entry is taken in
reg         got_t = 0;                  // clk_f: flips when the entry is taken out

assign i_ready = ~(put_t ^ got_t);
assign o_valid =   put_t ^ got_t;
assign o_data  = hold;

always @(posedge clk)   if (i_valid & i_ready) begin hold <= i_data; put_t <= ~put_t; end
always @(posedge clk_f) if (o_valid & o_ready) got_t <= ~got_t;

endmodule


// hd3_ss_cross: the savestate channel (ddram.sv's interface: req a one-clock pulse per access,
// ready a one-clock pulse when a write is taken or read data is valid) from clk to clk_f.
// The request toggles rq_t on the clk edge that sees req; the first clk_f edge after it
// (still inside that clk period, so the savestate engine's clk-registered addr/din/be/rnw have
// not moved) latches the access and raises f_req for one clk_f clock. f_addr/f_din/f_be/f_rnw
// then stay steady until the next access (hd3_ddr reads ch1_addr when it issues the command,
// which can be that clock or later). The savestate engine waits for ready before its next
// access, so one access is in flight at a time.
module hd3_ss_cross
(
	input             clk,
	input      [27:1] addr,
	input      [63:0] din,
	input       [7:0] be,
	input             rnw,
	input             req,
	output reg [63:0] dout,
	output reg        ready = 0,

	input             clk_f,
	output reg [27:1] f_addr,
	output reg [63:0] f_din,
	output reg  [7:0] f_be,
	output reg        f_rnw,
	output reg        f_req = 0,
	input      [63:0] f_dout,
	input             f_ready
);

reg        rq_t = 0, rq_d = 0;          // request toggle (clk) and its copy (clk_f)
reg        rd_t = 0, rd_d = 0;          // ready toggle (clk_f) and its copy (clk)
reg [63:0] f_q;

always @(posedge clk) begin
	if (req) rq_t <= ~rq_t;
	rd_d  <= rd_t;
	ready <= rd_t ^ rd_d;
	if (rd_t ^ rd_d) dout <= f_q;
end

always @(posedge clk_f) begin
	rq_d  <= rq_t;
	f_req <= rq_t ^ rq_d;
	if (rq_t ^ rq_d) begin
		f_addr <= addr; f_din <= din; f_be <= be; f_rnw <= rnw;
	end
	if (f_ready) begin
		f_q  <= f_dout;
		rd_t <= ~rd_t;
	end
end

endmodule
