// hd_ddr.sv - DDR3 arbiter for the NES core with the HD pack engine (test version).
//
// Replaces rtl/ddram.sv. Channel 1 (savestates) keeps ddram.sv's exact interface and
// behaviour; two HD channels are added:
//   - HD write: single qword, used only while an HD pack is being loaded
//   - HD read : 16-qword bursts into the HD line caches, several may be outstanding
//   - frame-record write: single qword with byte enables (hd_capture, through its FIFO)
// Priority: savestates > HD write > HD read > frame-record write. Avalon semantics as ddram.sv: a command is taken
// on a clock edge where it is asserted and DDRAM_BUSY is low. Read data returns in order, so a
// small queue of {client, beats} routes every returning beat to its owner.

module hd_ddr
(
	input         DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	// savestates (same as ddram.sv)
	input  [27:1] ch1_addr,
	output reg [63:0] ch1_dout,
	input  [63:0] ch1_din,
	input         ch1_req,
	input         ch1_rnw,
	input   [7:0] ch1_be,
	output reg    ch1_ready,

	// HD pack burst reads
	input         rd_req,
	input  [27:3] rd_addr,
	output reg    rd_ack,
	output reg [63:0] rd_data,
	output reg    rd_valid,

	// HD pack writes
	input         wr_req,
	input  [27:3] wr_addr,
	input  [63:0] wr_data,
	output reg    wr_ack,

	// frame-record writes (hd_capture): valid/ready, byte enables; taken when nothing else is
	// waiting, cw_ready is high on the clock the qword is taken
	input         cw_valid,
	input  [27:3] cw_addr,
	input  [63:0] cw_data,
	input   [7:0] cw_be,
	output        cw_ready
);

reg        cmd_rd = 0, cmd_wr = 0;
reg  [7:0] cmd_burst;
reg [27:3] cmd_addr;
reg [63:0] cmd_din;
reg  [7:0] cmd_be;
reg        cmd_ss;                       // read belongs to the savestate channel

assign DDRAM_RD       = cmd_rd;
assign DDRAM_WE       = cmd_wr;
assign DDRAM_BURSTCNT = cmd_burst;
assign DDRAM_ADDR     = {4'b0011, cmd_addr};   // RAM at 0x30000000, as ddram.sv
assign DDRAM_DIN      = cmd_din;
assign DDRAM_BE       = cmd_rd ? 8'hFF : cmd_be;

wire taken = (cmd_rd | cmd_wr) & ~DDRAM_BUSY;

// return queue
reg  [5:0] rq [8];                       // {ss, beats[4:0]}
reg  [2:0] rq_wp = 0, rq_rp = 0;
reg  [4:0] rq_cnt = 0;                   // beats received for the head entry
wire       rq_empty = (rq_wp == rq_rp);

reg        ss_pend = 0;

wire rd_go = rd_req && (rq_wp - rq_rp) < 3'd6;
assign cw_ready = !cmd_rd && !cmd_wr && !ss_pend && !ch1_req && !wr_req && !rd_go && cw_valid;

always @(posedge DDRAM_CLK) begin
	rd_ack    <= 0;
	wr_ack    <= 0;
	ch1_ready <= 0;
	rd_valid  <= 0;

	ss_pend <= ss_pend | ch1_req;

	if (taken) begin
		if (cmd_rd) begin
			rq[rq_wp] <= {cmd_ss, cmd_burst[4:0]};
			rq_wp     <= rq_wp + 3'd1;
		end
		cmd_rd <= 0;
		cmd_wr <= 0;
	end else if (!cmd_rd && !cmd_wr) begin
		if (ss_pend || ch1_req) begin
			ss_pend   <= 0;
			cmd_addr  <= ch1_addr[27:3];
			cmd_din   <= ch1_din;
			cmd_be    <= ch1_be;
			cmd_burst <= 8'd1;
			cmd_ss    <= 1;
			if (ch1_rnw) cmd_rd <= 1;
			else begin
				cmd_wr    <= 1;
				ch1_ready <= 1;                  // ddram.sv acknowledges writes on issue
			end
		end else if (wr_req) begin
			cmd_addr  <= wr_addr;
			cmd_din   <= wr_data;
			cmd_be    <= 8'hFF;
			cmd_burst <= 8'd1;
			cmd_wr    <= 1;
			wr_ack    <= 1;
		end else if (rd_go) begin
			cmd_addr  <= rd_addr;
			cmd_burst <= 8'd16;
			cmd_ss    <= 0;
			cmd_rd    <= 1;
			rd_ack    <= 1;
		end else if (cw_valid) begin
			cmd_addr  <= cw_addr;
			cmd_din   <= cw_data;
			cmd_be    <= cw_be;
			cmd_burst <= 8'd1;
			cmd_wr    <= 1;
		end
	end

	if (DDRAM_DOUT_READY && !rq_empty) begin
		if (rq[rq_rp][5]) begin
			ch1_dout  <= DDRAM_DOUT;
			ch1_ready <= 1;
		end else begin
			rd_data   <= DDRAM_DOUT;
			rd_valid  <= 1;
		end
		if (rq_cnt + 5'd1 == rq[rq_rp][4:0]) begin
			rq_cnt <= 0;
			rq_rp  <= rq_rp + 3'd1;
		end else begin
			rq_cnt <= rq_cnt + 5'd1;
		end
	end
end

endmodule
