// hd3_ddr.sv - DDR3 arbiter for the stage-3 HD engine (replaces hd_ddr.sv when the MVP goes).
//
// Clients:
//   ch1    savestates, ddram.sv's interface and behaviour, unchanged (highest priority)
//   wr     pack-load writes (hd3_loader): single qword, only while a pack is loading
//   cw     frame-record writes (hd_capture): single qword with byte enables, valid/ready
//   rd[k]  NR read clients (hd3_dict, hd3_resolve, hd3_render, ...): 8-qword bursts, each with
//          the hd3_lookup handshake - req held until ack (a one-clock pulse on the clock the
//          burst is issued), then 8 rd_valid beats in order; several bursts may be
//          outstanding per client
// Order of service: ch1 > wr > {cw and the readers, alternating when both wait}; the readers
// take turns among themselves through their own round-robin pointer. A plain fixed priority
// would let the renderer's steady reads starve the capture writer (whose FIFO then overflows
// and the frame is lost) or let the vblank RESOLVE stall behind them. The readers' pointer is
// separate from the cw/reader alternation on purpose: with one pointer over {cw, readers}, a
// cw grant while the return queue is full (readers not eligible) moved the pointer back to
// reader 0, so reader 0 was served after every such grant and readers 1-2 starved (the unit
// test measured 12 other grants while one reader waited). Now a reader waits for at most
// NR - 1 other readers, and cw for at most one reader.
// Avalon semantics as ddram.sv: a command is taken on a clock edge where it is asserted and
// DDRAM_BUSY is low. Read data returns in issue order, so a queue of {client, beats} routes
// every returning beat to its owner.

module hd3_ddr #(
	parameter NR   = 3,                     // read clients
	parameter QD   = 8,                     // outstanding reads tracked (power of 2)
	parameter QW   = 3                      // log2(QD)
)
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

	// pack-load writes
	input         wr_req,
	input         wr_hi,                    // address in the 0x20000000 window (pack overflow)
	input  [27:3] wr_addr,
	input  [63:0] wr_data,
	output reg    wr_ack,

	// frame-record writes: cw_ready is high on the clock the qword is taken
	input         cw_valid,
	input  [27:3] cw_addr,
	input  [63:0] cw_data,
	input   [7:0] cw_be,
	output        cw_ready,

	// read clients, packed: client k uses rd_addr[k*25 +: 25]
	input     [NR-1:0] rd_req,
	input     [NR-1:0] rd_hi,               // client k's address is in the 0x20000000 window
	input  [NR*25-1:0] rd_addr,
	output reg [NR-1:0] rd_ack,
	output reg  [63:0] rd_data,             // shared; rd_valid[k] says whose beat it is
	output reg [NR-1:0] rd_valid
);

localparam KW = (NR > 8) ? 4 : (NR > 4) ? 3 : (NR > 2) ? 2 : 1;   // reader index width

reg        cmd_rd = 0, cmd_wr = 0;
reg  [7:0] cmd_burst;
reg [27:3] cmd_addr;
reg        cmd_hi = 0;                   // 0x20000000 window (the pack's overflow, audio only)
reg [63:0] cmd_din;
reg  [7:0] cmd_be;
reg        cmd_ss;                       // read belongs to the savestate channel
reg  [3:0] cmd_cl;                       // read client

assign DDRAM_RD       = cmd_rd;
assign DDRAM_WE       = cmd_wr;
assign DDRAM_BURSTCNT = cmd_burst;
// RAM at 0x30000000 as ddram.sv; cmd_hi selects 0x20000000 (used from 0x24000000: below it are
// the scaler and Linux framebuffers, HDPACK_DESIGN.md memory map)
assign DDRAM_ADDR     = {3'b001, ~cmd_hi, cmd_addr};
assign DDRAM_DIN      = cmd_din;
assign DDRAM_BE       = cmd_rd ? 8'hFF : cmd_be;

wire taken = (cmd_rd | cmd_wr) & ~DDRAM_BUSY;

// return queue: {ss, client 4, beats 4}
reg  [8:0] rq [QD];
reg [QW:0] rq_wp = 0, rq_rp = 0;
reg  [3:0] rq_cnt = 0;                   // beats received for the head entry
wire       rq_empty = (rq_wp == rq_rp);
wire       rq_room  = (rq_wp - rq_rp) < QD - 1;  // one kept free for a savestate read

reg        ss_pend = 0;
reg [KW-1:0] rr = NR[KW-1:0] - 1'd1;     // reader served last (so reader 0 goes first)
reg        cw_turn = 0;                  // cw's turn when cw and a reader both wait

// next reader: the first one after `last` that requests (only when the queue has room)
function automatic [KW:0] pick(input [KW-1:0] last, input [NR-1:0] rq_);
	integer i;
	integer s;
	reg found;
	begin
		pick = 0;
		found = 0;
		for (i = 1; i <= NR; i = i + 1) begin
			s = (32'(last) + i) % NR;
			if (!found && rq_[s]) begin
				pick = {1'b1, s[KW-1:0]};
				found = 1;
			end
		end
	end
endfunction

wire [KW:0] rnext = pick(rr, rd_req & {NR{rq_room}});
wire        idle  = !cmd_rd && !cmd_wr && !ss_pend && !ch1_req && !wr_req;
wire        take_cw = cw_valid && (!rnext[KW] || cw_turn);
assign cw_ready = idle && take_cw;

integer k;
always @(posedge DDRAM_CLK) begin
	rd_ack    <= 0;
	wr_ack    <= 0;
	ch1_ready <= 0;
	rd_valid  <= 0;

	ss_pend <= ss_pend | ch1_req;

	if (taken) begin
		if (cmd_rd) begin
			rq[rq_wp[QW-1:0]] <= {cmd_ss, cmd_cl, cmd_burst[3:0]};
			rq_wp <= rq_wp + 1'd1;
		end
		cmd_rd <= 0;
		cmd_wr <= 0;
	end else if (!cmd_rd && !cmd_wr) begin
		if (ss_pend || ch1_req) begin
			ss_pend   <= 0;
			cmd_hi    <= 0;
			cmd_addr  <= ch1_addr[27:3];
			cmd_din   <= ch1_din;
			cmd_be    <= ch1_be;
			cmd_burst <= 8'd1;
			cmd_ss    <= 1;
			cmd_cl    <= 0;
			if (ch1_rnw) cmd_rd <= 1;
			else begin
				cmd_wr    <= 1;
				ch1_ready <= 1;                  // ddram.sv acknowledges writes on issue
			end
		end else if (wr_req) begin
			cmd_hi    <= wr_hi;
			cmd_addr  <= wr_addr;
			cmd_din   <= wr_data;
			cmd_be    <= 8'hFF;
			cmd_burst <= 8'd1;
			cmd_wr    <= 1;
			wr_ack    <= 1;
		end else if (take_cw) begin             // cw (cw_ready is high on this clock)
			cw_turn   <= 0;
			cmd_hi    <= 0;
			cmd_addr  <= cw_addr;
			cmd_din   <= cw_data;
			cmd_be    <= cw_be;
			cmd_burst <= 8'd1;
			cmd_wr    <= 1;
		end else if (rnext[KW]) begin
			cw_turn <= 1;
			rr      <= rnext[KW-1:0];
			for (k = 0; k < NR; k = k + 1)
				if (rnext[KW-1:0] == k[KW-1:0]) begin
					cmd_hi    <= rd_hi[k];
					cmd_addr  <= rd_addr[k*25 +: 25];
					rd_ack[k] <= 1;
					cmd_cl    <= k[3:0];
				end
			cmd_burst <= 8'd8;
			cmd_ss    <= 0;
			cmd_rd    <= 1;
		end
	end

	if (DDRAM_DOUT_READY && !rq_empty) begin
		if (rq[rq_rp[QW-1:0]][8]) begin
			ch1_dout  <= DDRAM_DOUT;
			ch1_ready <= 1;
		end else begin
			rd_data <= DDRAM_DOUT;
			for (k = 0; k < NR; k = k + 1)
				if (rq[rq_rp[QW-1:0]][7:4] == k[3:0]) rd_valid[k] <= 1;
		end
		if (rq_cnt + 4'd1 == rq[rq_rp[QW-1:0]][3:0]) begin
			rq_cnt <= 0;
			rq_rp  <= rq_rp + 1'd1;
		end else begin
			rq_cnt <= rq_cnt + 4'd1;
		end
	end
end

endmodule
