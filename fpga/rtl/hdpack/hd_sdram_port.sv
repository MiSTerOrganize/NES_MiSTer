// hd_sdram_port.sv - lends the HD engine single-byte reads on the SDRAM controller's third
// channel (rtl/sdram.sv ch2), which the core otherwise uses for battery saves and savestates.
//
// The controller starts an access on the RISING EDGE of a channel's rd, raises chN_busy when it
// takes the request and drops it with chN_dout valid (6 clk85 cycles = more than one core clock,
// so the core clock always sees it). ch0 and ch1 have priority, so the request may wait.
//
// A read is only started when nothing else can be using or about to use ch2:
//   allow  = !sleep_savestate && !bk_busy && !save_rd && !save_wr   (NES.sv)
//   and ch2_busy low.
// The battery-save path only starts while bk_busy, so it cannot start in the same clock; from
// the next clock on, `own` is folded into the save path's busy so it waits. A savestate raises
// sleep_savestate and then waits for the core to pause plus 100 clocks before its first SDRAM
// access; a read in flight here takes a handful of clocks, and NES.sv gives ch2 to this port
// while `own` is high, so it always finishes first.
//
// After the access, `own` stays high for one more clock with rd low, so the controller sees rd
// fall before the channel is handed back (another client whose rd is already high would
// otherwise produce no rising edge and its request would be lost).
//
// The controller remembers the last word each channel read and answers a repeat read of the
// same word without going to the SDRAM. That is only correct when one client owns a region;
// here the same word can be read again a frame later after the CPU or PPU changed it, so NES.sv
// sets sdram.ch2_nocache while `own` (rtl/sdram.sv).

module hd_sdram_port
(
	input             clk,
	input             reset,

	// from hd_watch: hold req until ack; q valid with ack
	input             req,
	input      [24:0] addr,
	output reg        ack = 0,
	output reg  [7:0] q = 0,

	input             allow,
	output reg        own = 0,
	output reg        rd = 0,
	output     [24:0] ch2_addr,
	input             ch2_busy,
	input       [7:0] ch2_dout,

	output reg        timeout = 0          // sticky: the controller never answered (should not happen)
);

localparam S_IDLE = 2'd0, S_TAKEN = 2'd1, S_DONE = 2'd2, S_GAP = 2'd3;
reg  [1:0] st = S_IDLE;
reg  [6:0] wd = 0;
reg [24:0] a  = 0;

assign ch2_addr = a;

always @(posedge clk) begin
	ack <= 0;
	case (st)
	S_IDLE: if (req && !ack && allow && !ch2_busy) begin
		a   <= addr;
		own <= 1;
		rd  <= 1;
		wd  <= 0;
		st  <= S_TAKEN;
	end
	S_TAKEN: begin                       // wait for the controller to take the request
		wd <= wd + 1'd1;
		if (ch2_busy) st <= S_DONE;
		else if (&wd) begin              // never taken: give up, answer 0
			timeout <= 1;
			rd  <= 0;
			ack <= 1;
			q   <= 0;
			st  <= S_GAP;
		end
	end
	S_DONE: if (!ch2_busy) begin
		rd  <= 0;
		ack <= 1;
		q   <= ch2_dout;
		st  <= S_GAP;
	end
	S_GAP: begin                         // rd low for one clock while still owned
		own <= 0;
		st  <= S_IDLE;
	end
	endcase

	if (reset) begin
		st  <= S_IDLE;
		own <= 0;
		rd  <= 0;
		ack <= 0;
	end
end

endmodule
