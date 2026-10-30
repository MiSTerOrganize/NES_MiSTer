// hd3_romfix.sv - the FPGA half of the pack's <patch> (stage 5, HDPACK_DESIGN.md "Stage 5 - ROM
// patches as built"). Core clock.
//
// Since 2026-10-28 (OPEN_ITEMS item 11, owner decision: move ROM patching to the ARM) the
// hashing, the entry search and the build run on the ARM (tools/armdec/romfix_arm.c). This
// module keeps only what the ARM cannot do:
//   - stage every NES ROM download into DDR3 (SRC_Q) while it passes to the core unchanged;
//   - hold the core in reset from the end of a download until the ARM has answered (only when
//     a pack with a section 14 is active - otherwise there is nothing to wait for);
//   - replay a ROM into the core as a second download when the ARM asks: the patched ROM the
//     ARM built at TGT_Q, or the staged original. The core sees downloading rise, the bytes on
//     its loader port, downloading fall - exactly what loading the file from the OSD does, so
//     the mapper is chosen from the PATCHED header (Metroid's mmm.ips adds CHR-ROM).
//
// Handshake (two qwords in the mailbox page, HDPACK_DESIGN.md has the layout):
//   status  RF_Q     written by this module whenever something the ARM decides on changes
//                    (a new ROM is staged, the pack's `active` changes) - those bump aseq - and
//                    after every command it carries out (last_id, content), which do not.
//     [63:56] 0xA6  [55:48] rseq (+1 per ROM download)  [47:40] aseq  [39:32] last command id
//     [31] active  [30] section 14 present  [29:28] content  [27] staged  [23:0] ROM length
//   command RF_Q + 1 written by the ARM, carried out once (id != last id) and only while it is
//                    still current (its rseq and aseq are the status's):
//     [63:56] 0xC7  [55:48] rseq  [47:40] aseq  [39:32] id  [31:30] action (0 none, 1 replay
//     the target, 2 replay the original)  [29:27] result  [23:0] target length
// A download that ends with a section-14 pack active holds the core until a current command
// comes back, or TMO clocks pass (result 6: the ARM did not answer; the ROM runs unpatched and
// a later command can still replay the patched one).
//
// At power-up (S_INIT0..S_INIT2) the last session's status is read and this session's rseq
// starts 0x40 past its rseq, then both qwords are written 0. DDR3 keeps the last session's
// handshake across a core reload; without the jump its {rseq, aseq} would equal the first ROM's
// here (both restart at 1), so a command left in DDR3 - or an answer the ARM was still working
// out for the old status when the core reloaded - would be taken for this session's question
// and replay the previous game's patched ROM. The clear lets the ARM see the reload and forget
// the old session (romfix_arm.h).
//
// DDR3 is reached through a single-access port (the savestate crossing, shared through an
// arbiter in the engine): m_req pulses with the request, m_ready pulses with the answer. A new
// download can start while an access is in flight; its answer is swallowed (drop) and nothing
// new is asked until it has come back, so every later answer is credited to its own request.

module hd3_romfix #(
	parameter [27:3] SRC_Q  = 25'h1900000,     // byte 0xC800000: the staged ROM file
	parameter [27:3] TGT_Q  = 25'h1980000,     // byte 0xCC00000: the patched ROM (the ARM writes it)
	parameter [27:3] RF_Q   = 25'h1880008,     // byte 0xC400040: status; command at + 8
	parameter [22:0] MAXB   = 23'h400000,      // 4 MB each
`ifdef HD3_SIM_TMO
	parameter [25:0] TMO    = `HD3_SIM_TMO,    // simulation only: the timeout cases cannot wait 2^25
`else
	parameter [25:0] TMO    = 26'h2000000,     // core clocks to wait for the ARM: 2^25 at 21.48 MHz
	                                           // (NTSC) is ~1.56 s, at 26.6 MHz (PAL) ~1.26 s; the
	                                           // count pauses while a DDR3 access is in flight
`endif
	parameter [15:0] POLL_W = 16'd1024,        // command poll period while the core is held
	parameter [15:0] POLL_I = 16'd16384        // and while it runs
)
(
	input             clk,

	// the OSD download (hps_io), and whether it is an NES ROM (NES.sv type_nes)
	input             dl,
	input             is_rom,
	input             wr,
	input       [7:0] din,
	input      [24:0] addr,
	input       [7:0] ft,                     // hps_io's ioctl_index (filetype)
	output            stall,                  // to hps_io's ioctl_wait

	// what the core sees (pass-through, or the replayed ROM)
	output            o_dl,
	output            o_wr,
	output      [7:0] o_din,
	output     [24:0] o_addr,
	output      [7:0] o_ft,
	input             core_wait,              // the core's own ioctl_wait (loader writes)
	output            hold,                   // keep the NES in reset

	// the pack
	input             active,
	input      [31:0] plen,                   // section 14 length

	// DDR3
	output reg        m_req = 0,
	output reg        m_rnw = 1,
	output reg [27:3] m_addr = 0,
	output reg [63:0] m_din = 0,
	output reg  [7:0] m_be = 0,
	input      [63:0] m_dout,
	input             m_ready,

	output reg  [1:0] content = 0,            // what the core holds: 0 original, 1 patched, 2 unknown
	output reg  [2:0] result = 0              // last decision: 0 none, 1 patched, 2 no match,
	                                          // 3 refused (size/format), 4 CRC, 5 ROM too big,
	                                          // 6 the ARM did not answer
);

localparam [3:0]
	S_IDLE = 0, S_STAGE = 1, S_SFLUSH = 2, S_SFIN = 3, S_WAIT = 4, S_CHK = 5,
	S_RP0 = 6, S_RPQ = 7, S_RPB = 8, S_RPEND = 9, S_MEM = 10, S_INIT = 11, S_INIT2 = 12,
	S_INIT0 = 13;

reg  [3:0] st = S_INIT0, ret = S_IDLE;
reg [63:0] mq = 0;                         // last memory answer

// accesses in flight and answers to swallow (an access abandoned by a new download)
reg  [1:0] nout = 0, drop = 0;
wire [1:0] nout_live = nout + {1'b0, m_req} - {1'b0, m_ready};   // still to answer after now
wire       take = m_ready && drop == 2'd0;                       // this answer is ours

task automatic mem(input rnw, input [27:3] a, input [63:0] d, input [3:0] back);
	begin
		m_req <= 1; m_rnw <= rnw; m_addr <= a; m_din <= d; m_be <= rnw ? 8'h00 : 8'hFF;
		ret <= back; st <= S_MEM;
	end
endtask

// ------------------------------------------------------------------------------------------
// staging: bytes -> qword -> DDR3. A full qword waiting for its write holds hps_io.
// ------------------------------------------------------------------------------------------
reg  [23:0] L = 0;                         // bytes staged (= the hps_io address expected)
reg  [63:0] asmq = 0;
reg         wq = 0;                        // a full qword waits to be written
reg  [63:0] wq_d = 0;
reg  [20:0] wq_a = 0;
reg         bad = 0;                       // address gap or too big: never patch this ROM
reg         staged = 0;                    // SRC holds a complete ROM
reg         old_dl = 0, old_act = 0, act_dirty = 0, from_dl = 0;
reg   [7:0] ft_rom = 0;
reg   [7:0] rseq = 0, aseq = 0, last_id = 0;
reg  [25:0] tmo = 0;
reg  [15:0] poll = 0;
reg         init_done = 0;                 // both handshake qwords cleared this session
reg   [3:0] init_ret = S_IDLE;
wire        has14 = plen >= 32'd64;

// staging includes the clocks its own qword writes wait in S_MEM: bytes keep arriving then
wire        stg  = (st == S_STAGE) | (st == S_MEM && ret == S_STAGE);
wire        push = stg & dl & wr;
assign stall = stg & wq;

// ------------------------------------------------------------------------------------------
// replay
// ------------------------------------------------------------------------------------------
reg         rp = 0, rp_dl = 0, rp_wr = 0;
reg   [7:0] rp_b = 0;
reg  [24:0] rp_addr = 0;
reg         rp_tgt = 0;
reg  [23:0] rp_len = 0, rp_k = 0;
reg   [3:0] gap = 0;

assign o_dl   = rp ? rp_dl   : dl;
assign o_wr   = rp ? rp_wr   : wr;
assign o_din  = rp ? rp_b    : din;
assign o_addr = rp ? rp_addr : addr;
assign o_ft   = rp ? ft_rom  : ft;

wire replaying = (st >= S_RP0 && st <= S_RPEND) || (st == S_MEM && ret >= S_RP0 && ret <= S_RPEND);
// from a ROM download (whose own downloading already holds the core) until the ARM's answer. A
// pack-triggered command arrives while the game keeps running and holds only for the replay,
// so a pack without a usable patch for the running game never resets it
assign hold = (st != S_IDLE && from_dl) || replaying;

// the command in mq is current and new
wire cmd_ok = mq[63:56] == 8'hC7 && mq[55:48] == rseq && mq[47:40] == aseq && mq[39:32] != last_id;

// ------------------------------------------------------------------------------------------
always @(posedge clk) begin
	m_req <= 0;
	rp_wr <= 0;
	old_dl <= dl;
	old_act <= active;
	if (active != old_act) act_dirty <= 1;
	nout <= nout_live;
	if (m_ready && drop != 2'd0) drop <= drop - 2'd1;

	if (push) begin
		if (addr[23:0] != L || addr[24]) bad <= 1;
		if (L >= {1'b0, MAXB}) bad <= 1;
		if (L[2:0] == 3'd7 && wq) bad <= 1;       // hps_io ignored `stall`: a qword would be lost
		L <= L + 24'd1;
		if (L[2:0] == 3'd7) begin
			wq <= 1;
			wq_d <= {din, asmq[55:0]};
			wq_a <= L[23:3];
			asmq <= 0;
		end else begin
			asmq[{L[2:0], 3'b000} +: 8] <= din;
		end
	end

	// a new download of anything stops whatever was running (an access in flight is answered
	// later: swallow it)
	if (dl && !old_dl) begin
		drop <= nout_live;
		rp <= 0; rp_dl <= 0;
		if (content != 2'd0 && rp) content <= 2'd2;
		if (is_rom) begin
			st <= S_STAGE; staged <= 0; bad <= 0; L <= 0; wq <= 0; asmq <= 0; ft_rom <= ft;
			content <= 0; result <= 0; from_dl <= 1;
			rseq <= rseq + 8'd1;                  // commands for the previous ROM are void
		end else begin
			st <= S_IDLE;
		end
	end else if (drop != 2'd0) begin
		// an abandoned access has not come back yet: ask nothing new (the arbiter holds one
		// request, so a new one could replace it and its answer would never come)
	end else case (st)
	// power-up: read the last session's status, clear it, then the command (init_ret: where to
	// go after). rseq keeps any download already counted and moves 0x40 past the old session's
	S_INIT0: mem(1, RF_Q, 64'd0, S_INIT);
	S_INIT:  begin rseq <= rseq + mq[55:48] + 8'h40; mem(0, RF_Q, 64'd0, S_INIT2); end
	S_INIT2: begin init_done <= 1; mem(0, RF_Q + 25'd1, 64'd0, init_ret); end
	S_IDLE: begin
		from_dl <= 0;
		poll <= poll + 16'd1;
		if (!staged) begin
			act_dirty <= 0;
		end else if (!dl && act_dirty) begin
			// the pack changed: tell the ARM (its decision comes back as a command)
			act_dirty <= 0;
			aseq <= aseq + 8'd1;
			mem(0, RF_Q, {8'hA6, rseq, aseq + 8'd1, last_id, active, has14, content, staged, 3'd0, L},
			    S_IDLE);
		end else if (!dl && poll >= POLL_I) begin
			poll <= 0;
			mem(1, RF_Q + 25'd1, 64'd0, S_CHK);
		end
	end

	// ---- staging
	S_STAGE: begin
		if (wq) begin
			wq <= 0;
			if (!bad) mem(0, SRC_Q + {4'd0, wq_a}, wq_d, S_STAGE);
		end else if (!dl) st <= S_SFLUSH;
	end
	S_SFLUSH: begin                               // the last part qword
		if (L[2:0] != 3'd0 && !bad) mem(0, SRC_Q + {4'd0, L[23:3]}, asmq, S_SFIN);
		else st <= S_SFIN;
	end
	S_SFIN: if (!init_done) begin
		init_ret <= S_SFIN;                       // a download beat the power-up clear: clear first
		st <= S_INIT0;
	end else begin
		staged <= ~bad;
		act_dirty <= 0;                           // this status carries `active` as it is now
		aseq <= aseq + 8'd1;
		tmo <= 0; poll <= 0;
		if (bad) result <= 3'd5;
		else if (!(active && has14)) result <= 3'd2;
		// wait for the ARM only when it may patch; the status goes out either way, so a pack
		// loaded later can still have the ROM patched
		mem(0, RF_Q, {8'hA6, rseq, aseq + 8'd1, last_id, active, has14, content, ~bad, 3'd0, L},
		    (!bad && active && has14) ? S_WAIT : S_IDLE);
	end

	// ---- the core is held: poll for the ARM's command
	S_WAIT: begin
		tmo <= tmo + 26'd1;
		poll <= poll + 16'd1;
		if (tmo >= TMO) begin
			result <= 3'd6; st <= S_IDLE;
		end else if (act_dirty) begin             // the pack changed meanwhile: a new question
			act_dirty <= 0;
			aseq <= aseq + 8'd1;
			mem(0, RF_Q, {8'hA6, rseq, aseq + 8'd1, last_id, active, has14, content, staged, 3'd0, L},
			    S_WAIT);
		end else if (poll >= POLL_W) begin
			poll <= 0;
			mem(1, RF_Q + 25'd1, 64'd0, S_CHK);
		end
	end
	S_CHK: begin
		if (cmd_ok) begin
			last_id <= mq[39:32];
			result  <= mq[29:27];
			// a target length of 0, or past the target area, is not one the ARM writes: refuse it
			// (result 3) and keep what the core holds, rather than replay past TGT_Q's 4 MB
			if (mq[31:30] == 2'd1 && (mq[23:0] == 24'd0 || mq[23:0] > {1'b0, MAXB})) begin
				result <= 3'd3;
				mem(0, RF_Q, {8'hA6, rseq, aseq, mq[39:32], active, has14, content, staged,
				              3'd0, L}, S_IDLE);
			end else case (mq[31:30])
			2'd1: begin rp_tgt <= 1; rp_len <= mq[23:0]; st <= S_RP0; end
			2'd2: begin rp_tgt <= 0; rp_len <= L;         st <= S_RP0; end
			default: mem(0, RF_Q, {8'hA6, rseq, aseq, mq[39:32], active, has14, content, staged,
			                       3'd0, L}, S_IDLE);
			endcase
		end else st <= from_dl ? S_WAIT : S_IDLE;
	end

	// ---- replay: downloading high, one byte per loader write, downloading low
	S_RP0: begin
		rp <= 1; rp_dl <= 1; rp_addr <= 0; rp_k <= 0;
		content <= 2'd2;
		gap <= 4'd15;                              // let the core see downloading rise
		st <= S_RPQ;
	end
	S_RPQ: begin
		if (gap != 0) gap <= gap - 4'd1;
		else if (rp_k >= rp_len) st <= S_RPEND;
		else mem(1, (rp_tgt ? TGT_Q : SRC_Q) + {4'd0, rp_k[23:3]}, 64'd0, S_RPB);
	end
	S_RPB: begin
		if (gap != 0) gap <= gap - 4'd1;
		else if (!core_wait) begin
			rp_wr <= 1;
			rp_b <= mq[{rp_k[2:0], 3'b000} +: 8];
			rp_addr <= {1'b0, rp_k};
			rp_k <= rp_k + 24'd1;
			gap <= 4'd7;
			if (rp_k[2:0] == 3'd7 || rp_k + 24'd1 >= rp_len) st <= S_RPQ;
		end
	end
	S_RPEND: begin
		rp_addr <= {1'b0, rp_len};
		if (gap != 0) gap <= gap - 4'd1;
		else if (!core_wait) begin
			if (rp_dl) begin rp_dl <= 0; gap <= 4'd7; end
			else begin
				rp <= 0;
				content <= rp_tgt ? 2'd1 : 2'd0;
				// report it done (last_id, content); not a new question, so aseq stays
				mem(0, RF_Q, {8'hA6, rseq, aseq, last_id, active, has14, rp_tgt ? 2'd1 : 2'd0,
				              staged, 3'd0, L}, S_IDLE);
			end
		end
	end

	S_MEM: if (take) begin mq <= m_dout; st <= ret; end
	default: st <= S_IDLE;
	endcase
end

endmodule
