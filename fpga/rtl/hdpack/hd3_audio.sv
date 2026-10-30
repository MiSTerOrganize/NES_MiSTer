// hd3_audio.sv - stage-6 HD pack audio (HDPACK_DESIGN.md "Stage 6 - audio"), on clk_f: the FPGA
// half. Since 2026-10-28 (OPEN_ITEMS item 11, owner decision: move audio to the ARM) the ADPCM
// decoding and the volumes run on the ARM (tools/armdec/aud_arm.c); this module keeps what must
// answer the CPU at once or be sample exact.
//
// The device is Mesen's HdAudioDevice (MESEN_HD_RUNTIME_SPEC.md 8): seven write registers
// (options, control, BGM volume, SFX volume, album, play BGM, play SFX) and five read registers
// (status, revision 1, 'N', 'E', 'A'), at $4100-$4106 or, with the pack option
// alternateRegisterRange, written at $3002/$3012/.../$3062 and read at $4018/$4019. The device
// takes those addresses away from the mapper (and from the PPU for the $3x02 writes), as
// Mesen's SetAllowOverride does. It is ACTIVE only when the loaded pack carries section 15 and
// the HD option is on; when it is not, every bus and audio signal passes through unchanged.
//
// Here: the registers and the status the CPU reads; on pack load the entry table's key, frame
// count and loop frame are read into on-chip tables (512 x 18 + 512 x 58), so a play write is
// resolved by a binary search long before the CPU can read the status back (>= 3 CPU cycles)
// and a voice starts on the next 48 kHz tick; a voice tracker (which voices play, their frame,
// ends, loops, the SFX start order, Mesen's rules) for the status bits and the savestate; and
// the mix. Every write the device takes, every savestate restore and every pack reset goes to
// the ARM through an event log in DDR3, stamped with the tick it applies before (every 16
// ticks a status qword says how many ticks are done). The ARM runs the same device on that log
// and, for every tick, writes the volume-scaled sample of each voice in mix order (BGM, then
// the SFX in start order) into a PCM slot. The mix happens here, AUD_D ticks later: the NES
// sample (DC-blocked, at its HD level, delayed by the same AUD_D) then each voice, clamped
// after each source, as OggMixer does - the model's output sample for sample, AUD_D ticks
// late. A slot not written in time mixes no HD voice and counts as an underrun.
//
// DDR3 (window qwords; the ARM side is tools/armdec/aud_arm.h):
//   status  AST_Q  {8'hA7, 8'd0, events logged 16, ticks mixed 32}
//   events  EV_Q   event i at qwords 2(i mod 2048), +1: header, payload (payload written first)
//           header {8'hE1, i 16, kind 4, 4'd0, tick 32}
//           kinds: 1 write {reg 3, value 8}; 2 savestate restore (word A);
//                  3 pack reset {section length 32, 3'd0, section offset 29}
//                  4 state, every 2^SYNC_LOG2 ticks: payload = the savestate word A of now,
//                    header flags bit 0 = BGM paused. An ARM half started after the session
//                    began resumes from the latest one (the BGM at its frame); a fresh one
//                    ignores them.
//   PCM     PCM_Q  slot t mod 4096 = 8 qwords: {8'hD5, 24'd0, t}, then 4 x 2 x {R 16, L 16}
// The log is a FIFO in front of one write port: it waits rather than drops when full, so the
// ARM sees every event (a full FIFO for longer than a tick would lose that tick; 256 qwords
// cannot fill that way at CPU write rates).
//
// tools/audio/hdaudio.py is the reference model; tools/sim3/tb_audio (aud_arm.c linked in)
// checks the two agree sample for sample.

module hd3_audio #(
	parameter integer SFX_V     = 7,
	parameter [27:3]  PACK_Q    = 25'd0,         // qword of file byte 0, 0x30000000 window
	parameter [27:3]  OVF_Q     = 25'h0800000,   // qword of file byte 192 MB, 0x20000000 window
	parameter [28:0]  WIN_BYTES = 29'h0C000000,  // file bytes below this are in the main window
	parameter [27:3]  AST_Q     = 25'h188000A,   // byte 0xC400050: status for the ARM
	parameter [27:3]  EV_Q      = 25'h1A00000,   // byte 0xD000000: event log
	parameter [27:3]  PCM_Q     = 25'h1A02000,   // byte 0xD010000: PCM slots
	parameter integer AUD_D     = 512,           // ticks from a tick to its sound (1..2047): 10.7 ms
	parameter integer SYNC_LOG2 = 8              // a state event every 2^n ticks (late ARM start)
)
(
	input             clk,                       // core clock: the CPU bus
	input             clk_f,
	input             pal,                       // clk_f is 106.4 MHz rather than 85.9 MHz

	// configuration (core-clock registers that change only on a pack load / OSD change)
	input             enable,                    // HD option
	input             pack_ok,
	input      [28:0] aud_off,                   // section 15 file offset
	input      [31:0] aud_len,                   // its size, 0 = absent
	input             alt_regs,                  // pack options bit 1

	// CPU bus (core clock); hit_* and rdata are combinational from addr
	input      [15:0] addr,
	input       [7:0] wdata,
	input             wr_strobe,                 // one core clock per CPU write cycle
	output            hit_wr,                    // the device takes this write
	output            hit_rd,                    // the device drives this read
	output      [7:0] rdata,

	// audio
	input      [15:0] nes_sample,                // unsigned, nes.v's sample (core clock)
	output            active,
	output reg [15:0] out_l = 0,                 // signed, held between 48 kHz frames
	output reg [15:0] out_r = 0,
	output reg [15:0] underruns = 0,             // ticks whose PCM slot was not in (debug)
	output reg [15:0] ev_lost = 0,               // CPU writes dropped, queue full (debug, clk)
	output reg        frame_done = 0,            // one clk_f clock: out_l/out_r updated (tests)

	// savestate (core clock): two 64-bit savestate registers in NES.sv carry the state Mesen's
	// HdAudioDevice::Serialize keeps (album, volumes, loop option, the BGM and its frame):
	//   A: [63:56] album  [55:48] BGM volume  [47:40] SFX volume  [39] loop
	//      [38] BGM on  [37:29] its entry  [28:0] its next frame
	//   B: 0, not read. (It held the BGM's ADPCM state; the state at a frame is a function of
	//      the frame, which the ARM decodes to - aud_arm.c.)
	input             ss_save,                   // a save is reading the registers: hold ss_out
	input             ss_load,                   // a load is writing them: restore when it ends
	input      [63:0] ss_in_a,                   // the registers (the loaded state)
	input      [63:0] ss_in_b,
	output reg [63:0] ss_out_a = 64'h0080800000000000,   // the live state, for a save
	output     [63:0] ss_out_b,

	// DDR3 read client (clk_f): hd3_ddr's rd_* handshake, 8-qword bursts
	output reg        rd_req = 0,
	output reg        rd_hi = 0,                 // address is in the 0x20000000 window
	output reg [27:3] rd_addr = 0,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid,

	// DDR3 single-qword writes (clk_f): the event log and the status. aw_ready takes the qword.
	output reg        aw_valid = 0,
	output reg [27:3] aw_addr = 0,
	output reg [63:0] aw_data = 0,
	input             aw_ready
);

localparam NV = SFX_V + 1;                       // voice 0 = BGM

// SFX_V must be 7: ord/mv are 3 bits, the PCM slot holds 8 mix positions, and 4'(NV) is the mix
// loop's end. AUD_D must fit the 2,048-sample NES delay. Either wrong: no module to elaborate.
generate
	if (SFX_V != 7) begin : bad_SFX_V
		hd3_audio_SFX_V_must_be_7 stop();
	end
	if (AUD_D < 1 || AUD_D > 2047) begin : bad_AUD_D
		hd3_audio_AUD_D_must_be_1_to_2047 stop();
	end
endgenerate
assign ss_out_b = 64'd0;
wire unused_b = &{1'b0, ss_in_b};

// ------------------------------------------------------------------------------- config
reg        en_f = 0, alt_f = 0;
reg        pok_f /*verilator public_flat_rd*/ = 0;
reg [28:0] off_f = 0;
reg [31:0] len_f = 0;
reg        tab_ok /*verilator public_flat_rd*/ = 0;   // tables loaded, magic and count valid
reg  [9:0] n_ent = 0;                            // entries, 1..512
always @(posedge clk_f) begin
	en_f  <= enable;
	pok_f <= pack_ok & (aud_len != 0);
	off_f <= aud_off;
	len_f <= aud_len;
	alt_f <= alt_regs;
end
assign active = en_f & pok_f & tab_ok;

// ------------------------------------------------------------------------------- bus decode
wire def_a = addr[15:3] == 13'h0820 && addr[2:0] != 3'd7;      // $4100-$4106
wire alt_w = addr[15:8] == 8'h30 && addr[3:0] == 4'h2 && addr[7:4] <= 4'h6;
wire alt_r = addr[15:1] == 15'h200C;                             // $4018-$4019
// hit_* go combinationally into the CPU's bus (its data mux, then the T65), so they start from
// core-clock registers: from a clk_f register that path gets one clk_f period, 11.6 ns, and the
// 2026-10-29 first full compile missed it by 4.7 ns. enable / pack_ok / aud_len / alt_regs are
// core-clock registers already; tab_ok (clk_f, set when the tables have loaded) reaches clk
// through one register - related clocks, hd3_cross.sv - so the bus follows it one clk later.
reg en_c = 0, pok_c = 0, alt_c = 0, tab_c = 0;
always @(posedge clk) begin
	en_c  <= enable;
	pok_c <= pack_ok & (aud_len != 0);
	alt_c <= alt_regs;
	tab_c <= tab_ok;
end
wire active_c = en_c & pok_c & tab_c;
assign hit_wr = active_c & (alt_c ? alt_w : def_a);
assign hit_rd = active_c & (alt_c ? alt_r : def_a);

reg  st_bgm = 0, st_sfx = 0, st_err = 0;
assign rdata = (addr[2:0] == 3'd0) ? {5'd0, st_err, st_sfx, st_bgm} :
               (addr[2:0] == 3'd1) ? 8'd1 :
               (addr[2:0] == 3'd2) ? 8'h4E :
               (addr[2:0] == 3'd3) ? 8'h45 :
               (addr[2:0] == 3'd4) ? 8'h41 : 8'd0;

// write events: a 4-entry queue, written on clk, read on clk_f (related clocks: hd3_cross.sv).
// A read-modify-write instruction writes twice in consecutive CPU cycles, as Mesen sees it.
// It holds 3: a 4th write would make it read as empty and lose all four, so a write that finds it
// full is dropped instead (the register never sees it, the log never carries it: both halves
// skip the same write) and ev_lost counts it. Only a stalled write port can fill it - the FSM
// takes a write every few dozen clk_f clocks, the CPU writes at most once per 12 clk.
reg  [10:0] evq [4];                             // {reg, value}
reg   [1:0] ev_wp = 0;
reg   [1:0] ev_rp = 0;
always @(posedge clk)
	if (wr_strobe & hit_wr) begin
		if (ev_wp + 2'd1 == ev_rp) begin
			if (ev_lost != 16'hFFFF) ev_lost <= ev_lost + 16'd1;
		end else begin
			evq[ev_wp] <= {(addr > 16'h4100) ? addr[2:0] : addr[6:4], wdata};
			ev_wp <= ev_wp + 2'd1;
		end
	end
wire        ev_have = ev_wp != ev_rp;
wire [10:0] ev_head = evq[ev_rp];

// ------------------------------------------------------------------------------- entry tables
reg [17:0] keytab [512];                         // {does not decode, key}
reg [57:0] entab  [512];                         // {loop frame, frames}
reg  [8:0] kt_ra = 0, kt_wa = 0;
reg [17:0] kt_q, kt_wd;
reg [57:0] en_q, en_wd;
reg        kt_we = 0;
always @(posedge clk_f) begin
	if (kt_we) begin keytab[kt_wa] <= kt_wd; entab[kt_wa] <= en_wd; end
	kt_q <= keytab[kt_ra];
	en_q <= entab[kt_ra];
end

// ------------------------------------------------------------------------------- NES delay
// the NES sample of tick t, written at t and read at t + AUD_D: it meets that tick's voices.
// Stored signed (two's complement), so the power-up zeros of a block RAM and of its output
// register are silence: an initial value on either keeps Quartus from inferring the RAM (it
// built this 2,048 x 16 store from registers, ~32,000 of them).
// nes_sample is combinational from the APU's channels (its mixer), so it is registered on clk
// before clk_f reads it: straight into nd_wd it was a 19.8 ns path in an 11.6 ns window (the
// 2026-10-29 first full compile). One core clock (47 ns) later than before, far inside a tick.
reg [15:0] nes_c = 0;
always @(posedge clk) nes_c <= nes_sample;
reg [15:0] nesd [2048];
reg [10:0] nd_wa = 0, nd_ra = 0;
reg [15:0] nd_wd = 0, nd_q = 0, nd_q2 = 0;
reg        nd_we = 0;
always @(posedge clk_f) begin
	if (nd_we) nesd[nd_wa] <= nd_wd;
	nd_q  <= nesd[nd_ra];
	// one more register: the M10K output straight into S_NES's DC block, clamp and level was
	// -0.449 ns (2026-10-29). Valid two clocks after the tick sets nd_ra; S_NES is at least
	// four (S_TRK, S_TRKV, S_TRK, S_TRKC)
	nd_q2 <= nd_q;
end

// ------------------------------------------------------------------------------- log FIFO
// {addr, data}, in order: a header after its payload, the status after every event it counts.
// An entry is queued with its slot (wf_wa) and written one clock later; wf_cp follows wf_wp a
// clock behind, so the reader only ever sees an entry already in memory (wf_n, which counts the
// queued ones too, decides room).
reg [88:0] wf [256];
reg  [8:0] wf_wp = 0, wf_cp = 0, wf_rp = 0;
reg  [7:0] wf_wa = 0;
reg [88:0] wf_q;
reg        wf_rd = 0, wf_we = 0;
reg [88:0] wf_wd;
wire [8:0] wf_n = wf_wp - wf_rp;
wire       wf_room = wf_n <= 9'd253;             // room for a payload and its header
always @(posedge clk_f) begin
	if (wf_we) wf[wf_wa] <= wf_wd;
	wf_q <= wf[wf_rp[7:0]];
	wf_cp <= wf_wp;
end

// ------------------------------------------------------------------------------- voices
reg         v_on    [NV];
initial for (int j = 0; j < NV; j = j + 1) v_on[j] = 1'b0;   // nothing plays before a pack
reg         v_loop  [NV];
reg   [8:0] v_ent   [NV];
reg  [28:0] v_frames[NV];
reg  [28:0] v_lpos  [NV];
reg  [28:0] v_pos   [NV];

reg   [2:0] ord [SFX_V];                         // SFX voices in start order, oldest first
reg   [2:0] n_sfx = 0;

reg   [7:0] r_bvol = 8'd128, r_svol = 8'd128, r_album = 0;
reg         r_loop = 0, r_pause = 0;

function automatic [25:0] f2q(input [28:0] f);   // file byte -> {hi, qword}
	f2q = (f < WIN_BYTES) ? {1'b0, PACK_Q + f[27:3]}
	                      : {1'b1, OVF_Q + f[27:3] - WIN_BYTES[27:3]};
endfunction

function automatic signed [15:0] clamp16(input signed [19:0] v);
	clamp16 = (v > 20'sd32767) ? 16'sd32767 : (v < -20'sd32768) ? -16'sd32768 : v[15:0];
endfunction

// NES level relative to HD audio: 5/8 (x/2 + x/8, arithmetic shifts). Measured against Mesen
// (NesSoundMixer::GetOutputVolume * 4, HD audio added at full scale): the core's APU tables
// through a mapper that halves the sample (MMC1, UxROM and 50 others) are 1.43-1.76 x
// Mesen's level depending on the channel, so 5/8 lands every channel within -1.0..+0.8 dB.
function automatic signed [15:0] nes_level(input signed [15:0] c);
	nes_level = (c >>> 1) + (c >>> 3);
endfunction

// ------------------------------------------------------------------------------- savestate
// restore: requested on the core clock when a load ends, applied on clk_f in S_IDLE.
reg        rs_req = 0;
reg        ld_old = 0, rs_tog = 0;
reg  [2:0] rs_s = 0;
always @(posedge clk) begin
	ld_old <= ss_load;
	if (ld_old & ~ss_load) rs_tog <= ~rs_tog;
end

// save: clk_f snapshots the state into snap and toggles snap_tog; the core clock copies it into
// ss_out_a (unless a save is reading it) and acks. snap holds still until then.
reg [63:0] snap;
reg        snap_tog = 0, snap_busy = 0, snap_ack = 0;
reg  [2:0] snap_s = 0, ack_s = 0;
always @(posedge clk) begin
	snap_s <= {snap_s[1:0], snap_tog};
	if (snap_s[2] != snap_s[1]) begin
		if (!ss_save) ss_out_a <= snap;
		snap_ack <= snap_s[1];
	end
end

// ------------------------------------------------------------------------------- 48 kHz tick
reg  [26:0] tacc = 0;
wire [26:0] tmod = pal ? 27'd106406848 : 27'd85909091;
reg         tick_pend /*verilator public_flat_rd*/ = 0;
wire [26:0] tnext = tacc + 27'd48000;
always @(posedge clk_f) tacc <= (tnext >= tmod) ? tnext - tmod : tnext;

reg  [31:0] tcount = 0;                          // ticks mixed
reg  [10:0] since_rst = 0;                       // ticks mixed since the last pack reset (sat.)
reg  [15:0] ev_idx = 0;                          // events logged

// ------------------------------------------------------------------------------- DDR reads
localparam R_TAB = 1'b0, R_PCM = 1'b1;
reg        r_kind = 0;
reg        r_busy = 0;
reg  [2:0] r_beat = 0;
reg  [8:0] ld_burst = 0;                         // R_TAB bursts issued
reg [11:0] ld_q = 0;                             // R_TAB qwords received
reg        loading = 0;
// The tables are read in bursts from the 64-byte boundary at or below the section: an 8-qword
// burst from an unaligned start could run past 192 MB inside the main window (f2q maps only
// its first byte), while an aligned one never straddles it (WIN_BYTES is a multiple of 64).
// The first ld_skip qwords are before the section; lq counts the section's own.
wire [2:0]  ld_skip = off_f[5:3];
wire [11:0] lq = ld_q - {9'd0, ld_skip};
wire        ld_in = ld_q >= {9'd0, ld_skip};
wire [11:0] ld_end = {9'd0, ld_skip} + 12'd2 + {n_ent, 2'b00};   // qwords to receive
// PCM: the slot of tick pc_t, read after each tick for the next one. pc_ok is set on a read's
// last beat, from the tag latched on its first compared with pc_t THEN, so it is 1 only once
// the slot of the current pc_t has arrived whole: a read still in flight when its tick is mixed
// (an underrun) ends with pc_ok 0, as pc_t has moved on, and the next read cannot be mixed
// before its own last beat. (Comparing at the first beat let a late read's pc_done meet the next
// read's pc_ok and mix a half-overwritten slot.)
reg        pc_want = 0, pc_ok = 0, pc_done = 0;
reg [63:0] pc_tag = 0;
reg [31:0] pc_t = 0;
reg [31:0] pc_s [NV];                            // {R, L} per mix position

// ------------------------------------------------------------------------------- control FSM
localparam [3:0] S_IDLE = 0, S_SRCH = 1, S_SRCHW = 2, S_SRCHC = 3, S_TRK = 4, S_NES = 5,
                 S_MIX = 6, S_LOGP = 7, S_LOGH = 8, S_RSW = 9, S_RSC = 10, S_TRKC = 11,
                 S_TRKV = 12;
reg  [3:0] st /*verilator public_flat_rd*/ = S_IDLE;   // public: tb_audio times a pack reset
reg  [3:0] lg_ret = S_IDLE;

reg [16:0] s_key;
reg  [9:0] s_lo, s_hi;
reg  [8:0] s_mid;

reg  [3:0] m_i;                                  // tracker position (0 = BGM) / mix position
reg [NV-1:0] m_done;
reg [SFX_V-1:0] m_keep;                          // S_TRK -> S_TRKC: SFX position i plays on
reg  [2:0] t_v;                                  // S_TRK -> S_TRKV: the voice at m_i, its state
reg        t_play, t_loop;
reg [28:0] t_pos, t_fr, t_lp;
reg signed [15:0] acc_l, acc_r;
reg signed [19:0] dc_x = 0, dc_y = 0;
reg        m_pcm;                                // this tick's slot is in

reg  [3:0] lg_kind, lg_flags;
reg [63:0] lg_pay;
reg        rst_pend = 0, st_due = 0, sync_due = 0;

wire  [2:0] mv = (m_i == 4'd0) ? 3'd0 : ord[m_i[2:0] - 3'd1];
wire signed [15:0] nes_x = nd_q2;
wire  [8:0] rs_idx = ss_in_a[37:29];

// the snapshot a save reads (no BGM: its fields are 0, so a save does not depend on what
// played before)
wire [63:0] snap_now = {r_album, r_bvol, r_svol, r_loop, v_on[0], v_on[0] ? v_ent[0] : 9'd0,
                        v_on[0] ? v_pos[0] : 29'd0};

integer i;
reg old_pok /*verilator public_flat_rd*/ = 0;

always @(posedge clk_f) begin : main
	reg found;
	reg [28:0] p;
	reg [2:0]  v;
	reg [2:0]  k;
	reg signed [19:0] y;
	reg signed [15:0] sl, sr;

	kt_we <= 0;
	wf_we <= 0;
	nd_we <= 0;
	frame_done <= 0;
	if (rd_ack) rd_req <= 0;

	// ---- the log FIFO to the write port
	if (aw_valid && aw_ready) aw_valid <= 0;
	if (wf_rd) begin
		wf_rd <= 0;
		aw_valid <= 1;
		{aw_addr, aw_data} <= wf_q;
	end else if (!aw_valid && wf_cp != wf_rp) begin
		wf_rd <= 1;                              // wf_q holds wf[wf_rp] on the next clock
		wf_rp <= wf_rp + 9'd1;
	end

	// ---- DDR read data
	if (rd_valid) begin
		r_beat <= r_beat + 3'd1;
		if (r_beat == 3'd7) r_busy <= 0;
		if (r_kind == R_TAB) begin
			ld_q <= ld_q + 12'd1;
			if (!ld_in) begin
				// before the section (the burst started at the boundary below it)
			end else if (lq == 0) begin
				if (rd_data[31:0] != 32'h55414448 || rd_data[63:32] == 0 ||
				    rd_data[63:32] > 32'd512)
					loading <= 0;                    // not a valid section: stays inactive
				n_ent <= rd_data[41:32];
			end else if (lq < 12'd2 + {n_ent, 2'b00} && lq[1:0] == 2'd2) begin
				// (the bound keeps the last burst's spare qwords from wrapping onto entry 0)
				kt_wa <= lq[10:2];
				kt_wd[16:0] <= rd_data[16:0];
			end else if (lq < 12'd2 + {n_ent, 2'b00} && lq[1:0] == 2'd3) begin
				kt_we <= 1;
				kt_wd[17] <= rd_data[31:0] == 32'd0;
				en_wd <= {rd_data[60:32], rd_data[28:0]};
			end
		end else begin
			if (r_beat == 3'd0) pc_tag <= rd_data;
			else if (r_beat <= 3'd4) begin
				pc_s[{r_beat[1:0] - 2'd1, 1'b0}] <= rd_data[31:0];
				pc_s[{r_beat[1:0] - 2'd1, 1'b1}] <= rd_data[63:32];
			end
			if (r_beat == 3'd7) begin
				pc_done <= 1;
				pc_ok   <= pc_tag == {8'hD5, 24'd0, pc_t};   // pc_t now, not at the first beat
			end
		end
	end

	// ---- issue one read at a time: the tables on a pack load, else the next PCM slot
	if (!r_busy && !rd_req) begin
		if (loading) begin
			if (ld_burst == 0 || {ld_burst, 3'd0} < ld_end) begin
				{rd_hi, rd_addr} <= f2q({off_f[28:6], 6'd0} + {14'd0, ld_burst, 6'd0});
				rd_req <= 1; r_busy <= 1; r_beat <= 0; r_kind <= R_TAB;
				ld_burst <= ld_burst + 9'd1;
			end else if (ld_q >= ld_end) begin
				loading <= 0;
				tab_ok  <= 1;
			end
		end else if (pc_want) begin
			pc_want <= 0;
			rd_hi   <= 0;
			rd_addr <= PCM_Q + {10'd0, pc_t[11:0], 3'd0};
			rd_req <= 1; r_busy <= 1; r_beat <= 0; r_kind <= R_PCM;
		end
	end

	case (st)
	S_IDLE: begin
		// Anything that logs is taken only when the FIFO has room for its two entries, so
		// S_LOGP never waits: a write port that stalls long enough to fill the FIFO delays
		// events (the CPU-write queue then fills and ev_lost counts) but never the 48 kHz ticks.
		if (rst_pend && wf_room) begin
			rst_pend <= 0;
			lg_kind <= 4'd3; lg_flags <= 4'd0; lg_pay <= {len_f, 3'd0, off_f}; lg_ret <= S_IDLE;
			st <= S_LOGP;
		end else if (tick_pend && tab_ok) begin
			tick_pend <= 0;
			// the NES sample: this tick's goes in, the one AUD_D ticks back comes out
			nd_we <= 1; nd_wa <= tcount[10:0]; nd_wd <= {~nes_c[15], nes_c[14:0]};
			nd_ra <= 11'(tcount - AUD_D);
			m_pcm <= pc_done && pc_ok;
			if (since_rst >= 11'(AUD_D) && !(pc_done && pc_ok)) underruns <= underruns + 16'd1;
			m_i    <= 0;
			m_done <= 0;
			st <= S_TRK;
		end else if (rs_req && tab_ok && wf_room) begin
			// a savestate load ended: the registers, and the BGM at its saved frame (or none).
			// SFX and the pause flag are left as they are, as in Mesen.
			rs_req  <= 0;
			r_album <= ss_in_a[63:56];
			r_bvol  <= ss_in_a[55:48];
			r_svol  <= ss_in_a[47:40];
			r_loop  <= ss_in_a[39];
			kt_ra   <= rs_idx;
			st <= S_RSW;
		end else if (ev_have && tab_ok && wf_room) begin
			ev_rp <= ev_rp + 2'd1;
			case (ev_head[10:8])
				3'd0: begin r_loop <= ev_head[0]; v_loop[0] <= ev_head[0]; end
				3'd1: begin
					r_pause <= ev_head[0];
					if (ev_head[1]) v_on[0] <= 0;
					if (ev_head[2]) begin
						for (i = 1; i < NV; i = i + 1) v_on[i] <= 0;
						n_sfx <= 0;
					end
				end
				3'd2: r_bvol <= ev_head[7:0];
				3'd3: r_svol <= ev_head[7:0];
				3'd4: r_album <= ev_head[7:0];
				3'd5, 3'd6: begin
					s_key <= {ev_head[10:8] == 3'd6, r_album, ev_head[7:0]};
					s_lo  <= 0;
					s_hi  <= n_ent;
				end
				default: ;
			endcase
			lg_kind <= 4'd1; lg_flags <= 4'd0; lg_pay <= {53'd0, ev_head};
			lg_ret  <= (ev_head[10:8] == 3'd5 || ev_head[10:8] == 3'd6) ? S_SRCH : S_IDLE;
			st <= S_LOGP;
		end else if (sync_due && tab_ok && wf_room) begin
			// the state now, between ticks and after every event taken so far: stamped with
			// tcount, it is what a savestate saved at this tick would hold, plus the pause flag
			sync_due <= 0;
			lg_kind <= 4'd4; lg_flags <= {3'd0, r_pause}; lg_pay <= snap_now; lg_ret <= S_IDLE;
			st <= S_LOGP;
		end else if (st_due && wf_n <= 9'd254) begin
			st_due <= 0;
			wf_we <= 1; wf_wa <= wf_wp[7:0]; wf_wp <= wf_wp + 9'd1;
			wf_wd <= {AST_Q, 8'hA7, 8'd0, ev_idx, tcount};
		end
	end

	// the event log: the payload, then the header naming it (waits for room for both)
	S_LOGP: if (wf_n <= 9'd253) begin
		wf_we <= 1; wf_wa <= wf_wp[7:0]; wf_wp <= wf_wp + 9'd1;
		wf_wd <= {EV_Q + {13'd0, ev_idx[10:0], 1'b1}, lg_pay};
		st <= S_LOGH;
	end
	S_LOGH: begin
		wf_we <= 1; wf_wa <= wf_wp[7:0]; wf_wp <= wf_wp + 9'd1;
		wf_wd <= {EV_Q + {13'd0, ev_idx[10:0], 1'b0}, 8'hE1, ev_idx, lg_kind, lg_flags, tcount};
		ev_idx <= ev_idx + 16'd1;
		st <= lg_ret;
	end

	// a restore's BGM: its entry's frames and loop frame
	S_RSW: st <= S_RSC;
	S_RSC: begin
		if (ss_in_a[38] && {1'b0, rs_idx} < n_ent) begin
			v_on[0] <= 1; v_ent[0] <= rs_idx; v_loop[0] <= ss_in_a[39];
			v_frames[0] <= en_q[28:0]; v_lpos[0] <= en_q[57:29]; v_pos[0] <= ss_in_a[28:0];
		end else
			v_on[0] <= 0;
		lg_kind <= 4'd2; lg_flags <= 4'd0; lg_pay <= ss_in_a; lg_ret <= S_IDLE;
		st <= S_LOGP;
	end

	// binary search over [s_lo, s_hi)
	S_SRCH: begin
		if (s_lo >= s_hi) begin
			st_err <= 0;                          // undefined id: no change, error bit 0
			st <= S_IDLE;
		end else begin
			s_mid <= 9'((s_lo + s_hi) >> 1);
			kt_ra <= 9'((s_lo + s_hi) >> 1);
			st <= S_SRCHW;
		end
	end
	S_SRCHW: st <= S_SRCHC;
	S_SRCHC: begin
		if (kt_q[16:0] == s_key) begin
			st <= S_IDLE;
			if (!s_key[16]) begin                 // BGM: error bit = started
				st_err <= !kt_q[17];
				if (!kt_q[17]) begin
					v_on[0] <= 1; v_ent[0] <= s_mid; v_loop[0] <= r_loop;
					v_frames[0] <= en_q[28:0]; v_lpos[0] <= en_q[57:29]; v_pos[0] <= 29'd0;
				end
			end else begin                        // SFX: error bit = defined but undecodable
				st_err <= kt_q[17];
				if (!kt_q[17]) begin
					found = 0;
					v = 0;
					for (i = SFX_V; i >= 1; i = i - 1)
						if (!v_on[i]) begin found = 1; v = i[2:0]; end
					if (!found) begin             // all busy: replace the oldest
						v = ord[0];
						for (i = 0; i < SFX_V - 1; i = i + 1) ord[i] <= ord[i + 1];
						ord[SFX_V - 1] <= v;
					end else begin
						ord[n_sfx] <= v;
						n_sfx <= n_sfx + 3'd1;
					end
					v_on[v] <= 1; v_ent[v] <= s_mid; v_loop[v] <= 0;
					v_frames[v] <= en_q[28:0]; v_lpos[v] <= en_q[57:29]; v_pos[v] <= 29'd0;
				end
			end
		end else begin
			if (kt_q[16:0] < s_key) s_lo <= {1'b0, s_mid} + 10'd1;
			else                    s_hi <= {1'b0, s_mid};
			st <= S_SRCH;
		end
	end

	// the voice tracker, in mix order: what each voice plays this tick, which ones end
	S_TRK: begin
		if (m_i > {1'b0, n_sfx}) begin
			// which SFX positions play on, registered: the compaction below in one clock with
			// m_done[ord[i]] in it was the slowest clk_f path (-0.152 ns, 2026-10-29)
			for (i = 0; i < SFX_V; i = i + 1)
				m_keep[i] <= i < n_sfx && !m_done[ord[i]];
			st <= S_TRKC;
		end else begin
			// this position's voice, registered: selecting it (ord[m_i - 1], then an 8-way mux of
			// 29-bit fields) and the compares below in one clock was -4.3 ns (2026-10-29)
			t_v    <= mv;
			t_play <= v_on[mv] && !(mv == 3'd0 && r_pause);
			t_pos  <= v_pos[mv];
			t_fr   <= v_frames[mv];
			t_lp   <= v_lpos[mv];
			t_loop <= v_loop[mv];
			st <= S_TRKV;
		end
	end
	S_TRKV: begin
		if (!t_play) begin
			// nothing plays here
		end else if (t_pos >= t_fr && !t_loop) begin
			m_done[t_v] <= 1;                      // ended (loop turned off at its end)
		end else begin
			p = (t_pos >= t_fr) ? t_lp : t_pos;    // loop restart
			v_pos[t_v] <= p + 29'd1;
			if (p + 29'd1 >= t_fr && !t_loop) m_done[t_v] <= 1;
		end
		m_i <= m_i + 4'd1;
		st <= S_TRK;
	end

	// the SFX list without the voices that ended, in order; the ended voices go off
	S_TRKC: begin
		k = 0;
		for (i = 0; i < SFX_V; i = i + 1)
			if (m_keep[i]) begin ord[k] <= ord[i]; k = k + 3'd1; end
		n_sfx <= k;
		for (i = 0; i < NV; i = i + 1)
			if (m_done[i]) v_on[i] <= 0;
		st <= S_NES;
	end

	// the delayed NES sample (nd_q, read at the tick), DC-blocked, at its HD level
	S_NES: begin
		// all operands signed, or >>> turns into a logical shift
		y = 20'(nes_x) - dc_x + dc_y - (dc_y >>> 10);
		dc_x  <= 20'(nes_x);
		dc_y  <= y;
		acc_l <= nes_level(clamp16(y));
		acc_r <= nes_level(clamp16(y));
		m_i   <= 0;
		st <= S_MIX;
	end
	// then the voices of AUD_D ticks back, in mix order, clamped after each
	S_MIX: begin
		if (m_i == 4'(NV)) begin
			out_l <= acc_l;
			out_r <= acc_r;
			frame_done <= 1;
			tcount <= tcount + 32'd1;
			if (since_rst != 11'h7FF) since_rst <= since_rst + 11'd1;
			if (tcount[3:0] == 4'hF) st_due <= 1;
			if (&tcount[SYNC_LOG2-1:0]) sync_due <= 1;
			// the next tick's slot: the slot of tick tcount + 1 - AUD_D
			pc_t <= tcount + 32'd1 - AUD_D; pc_want <= 1; pc_done <= 0; pc_ok <= 0;
			st <= S_IDLE;
		end else begin
			sl = m_pcm ? pc_s[m_i[2:0]][15:0]  : 16'sd0;
			sr = m_pcm ? pc_s[m_i[2:0]][31:16] : 16'sd0;
			acc_l <= clamp16({{4{acc_l[15]}}, acc_l} + {{4{sl[15]}}, sl});
			acc_r <= clamp16({{4{acc_r[15]}}, acc_r} + {{4{sr[15]}}, sr});
			m_i <= m_i + 4'd1;
		end
	end
	default: st <= S_IDLE;
	endcase

	// after the FSM, so a tick that arrives on the clock the last one is taken is not lost
	if (tnext >= tmod) tick_pend <= 1;

	// status: BGM exists and is not paused; any SFX; the last play's error bit
	st_bgm <= v_on[0] & ~r_pause;
	found = 0;
	for (i = 1; i < NV; i = i + 1) found = found | v_on[i];
	st_sfx <= found;

	// savestate load: a request per load (core-clock toggle, synchronised here)
	rs_s <= {rs_s[1:0], rs_tog};
	if (rs_s[2] != rs_s[1]) rs_req <= 1;

	// savestate snapshot for the core clock, taken whenever the last one has been taken over.
	// Any clock will do: every field pair the snapshot uses is written in the same clock
	// (v_on/v_ent/v_pos, the restore's registers), and a save starts reading 100 core clocks
	// after the CPU stops (savestates.vhd SETTLECOUNT), long after its last write.
	ack_s <= {ack_s[1:0], snap_ack};
	if (snap_busy) begin
		if (ack_s[2] == snap_tog) snap_busy <= 0;
	end else begin
		snap      <= snap_now;
		snap_tog  <= ~snap_tog;
		snap_busy <= 1;
	end

	// a new pack: everything to Mesen's power-on values, then read the tables; the ARM is told
	// (a pack reset event) before any write of the new pack
	old_pok <= pok_f;
	if (pok_f & ~old_pok) begin
		tab_ok <= 0; loading <= 1; ld_burst <= 0; ld_q <= 0; n_ent <= 0;
		for (i = 0; i < NV; i = i + 1) v_on[i] <= 0;
		n_sfx <= 0; r_loop <= 0; r_pause <= 0;
		r_bvol <= 8'd128; r_svol <= 8'd128; r_album <= 0;
		st_err <= 0;
		ev_rp <= ev_wp;
		rs_req <= 0;                          // a load before this pack does not apply to it
		rst_pend <= 1; since_rst <= 0;
		// a tick being mixed finishes (its slot, its tcount, the next slot's read), so no tick is
		// lost and the reset is stamped after it; anything else stops here
		if (st != S_TRK && st != S_TRKV && st != S_TRKC && st != S_NES && st != S_MIX)
			st <= S_IDLE;
	end else if (!pok_f) begin
		tab_ok <= 0; loading <= 0;
	end
end

endmodule
