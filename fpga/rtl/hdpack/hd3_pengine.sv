// hd3_pengine.sv - the HD pack engine for the ARM display-list route (OPEN_ITEMS item 11,
// "THE ROUTE THAT CAN FIT"). Same ports as hd3_engine.sv, so it drops into NES.sv in its place.
//
// The ARM makes every decision Mesen's HD renderer makes (tools/armdec/hdarm4.c) and writes a
// display list per frame; this engine only captures, paints and scans out. Compared with
// hd3_engine, the dictionary, additions, resolve, frame maps and renderer are gone; hd3_paint
// takes the renderer's place on the same output interface (out_v/out_y/out_x/out_px,
// line_ok/line_y) and the same scan-out.
//
// ARM <-> FPGA, all through DDR3 (byte addresses in the 0x30000000 window):
//   0x3C000000 + 0x100000*b   frame record bank b (hd_capture, unchanged); trailer qword at
//                             +0xF0000 = the frame number of the record in that bank
//   +0xF0100 .. +0xF01FF      watched-address values of that frame, one byte per pack slot
//                             (written by this engine from hd_watch when the snapshot is done)
//   +0xF0200                  watch marker qword: [31:0] frame number, [40:32] value count;
//                             written after the values, so the ARM reads values only once the
//                             marker names the frame it is preparing. No snapshot is taken when
//                             the pack watches nothing, so then there is no marker either.
//   0x3C200000 + 0x80000*k    display-list buffer k = frame id mod 4 (64 Kqwords each); four
//                             buffers, so the ARM never rewrites a list the painter still reads
//   0x3C400000                mailbox qword, written by the ARM: [31:0] frame id (the ARM's own
//                             count, never 0), [55:32] qwords of that list published so far,
//                             [63:56] 0xA5. The ARM publishes in multiples of 8 qwords (padding
//                             after the last EOL); the painter reads only published bursts.
//   0x3C400008                pack-state qword, written by this engine: [31:0] generation (+1
//                             each time a pack becomes valid), [39:36] reject, [47:40] last load
//                             command carried out, [48] pack valid, [63:56] 0x5A. The ARM
//                             re-reads the pack (the .hdp at 0x30000000) when the generation
//                             moves, and stops publishing while it is not valid.
//   0x3C400040 / +0x48        ROM patcher status / command (hd3_romfix.sv, romfix_arm.h)
//   0x3C400050                HD audio status, written by this engine (hd3_audio.sv, aud_arm.h)
//   0x3C400060                pack load command, written by the ARM after it copies a pack into
//                             DDR3 (hd3_loader.sv: BEGIN / COMMIT, sequence, file size)
//   0x30000000 / 0x24000000   the pack: file bytes below 192 MB / from 192 MB on, written by
//                             the ARM (hdarm_daemon.c); hd3_loader reads its header back
//   0x3C800000 / 0x3CC00000   ROM patcher staged ROM / target, 4 MB each
//   0x3D000000                HD audio event log (2048 x 2 qwords)
//   0x3D010000                HD audio PCM slots, written by the ARM (4096 x 8 qwords)
// The engine polls the mailbox every 2048 clocks. A frame id it has not painted, with at least
// one burst published, starts the painter on that buffer; while the mailbox still names the
// frame being painted, list_end follows the published count, so the ARM may publish a list
// line by line or whole. Once the mailbox moves on, the frame being painted counts as fully
// published (the ARM finishes a list before starting the next).
// Frame ids the engine never started count as dropped.

module hd3_pengine #(
	parameter integer NL         = 16,
	parameter integer RS_MAX     = 1024,
	parameter integer ADD_BUDGET = 60000     // HD clocks; ~11 NES lines
)
(
	input             clk,
	input             clk_f,
	input             reset,                 // core clock: the NES reset
	input             enable,                // "HD Pack" OSD option

	// pack load (core clock). The ARM copies the pack into DDR3 and hd3_loader reads it back
	// (see its header), so the OSD file transfer is not used: these three inputs are unused
	// and load_wait is 0. They stay so the ports match hd3_engine's.
	input             load_active,
	input             load_wr,
	input       [7:0] load_data,
	output            load_wait,
	input             cart_no_chr_rom,

	// record writer (hd_stage2, core clock)
	input             cw_valid,
	input      [27:3] cw_addr,
	input      [63:0] cw_data,
	input       [7:0] cw_be,
	output            cw_ready,
	input      [31:0] frame_num,             // hd_stage2: N + 1 from the end of line 239 of N
	input             watch_done,            // hd_stage2: snapshot complete in bank ~frame_num[0]

	// watched-address list to hd_stage2 (core clock)
	output            lw_we,
	output      [7:0] lw_idx,
	output     [31:0] lw_addr,
	output      [8:0] list_n,

	// watch values from hd_stage2 (HD clock, one clock latency)
	output            w1_bank,
	output      [7:0] w1_idx,
	input       [7:0] w1_q,
	output            w2_bank,
	output      [7:0] w2_idx,
	input       [7:0] w2_q,

	// tap timing (core clock)
	input             line_done,             // tap[114]
	input       [8:0] line_y,                // tap[123:115]

	// savestates (core clock, ddram.sv's interface)
	input      [27:1] ss_addr,
	input      [63:0] ss_din,
	output     [63:0] ss_dout,
	input             ss_req,
	input             ss_rnw,
	input       [7:0] ss_be,
	output            ss_ack,

	// ROM patches (hd3_romfix): the OSD download in, what the core sees out
	input             rom_dl,                // hps_io ioctl_download
	input             rom_is_nes,            // NES.sv type_nes
	input             rom_wr,
	input       [7:0] rom_din,
	input      [24:0] rom_addr,
	input       [7:0] rom_ft,                // hps_io ioctl_index
	output            rom_stall,             // OR into hps_io's ioctl_wait
	output            rom_o_dl,
	output            rom_o_wr,
	output      [7:0] rom_o_din,
	output     [24:0] rom_o_addr,
	output      [7:0] rom_o_ft,
	input             rom_core_wait,
	output            rom_hold,              // keep the NES in reset
	output      [1:0] rom_content,
	output      [2:0] rom_result,

	// HD audio (stage 6, hd3_audio): CPU bus snoop/override and the mix (core clock)
	input             pal,                   // PAL timing: clk_f is 106.4 MHz
	input      [15:0] cpu_a,                 // CPU bus address of this cycle
	input       [7:0] cpu_dout,              // CPU write data
	input             cpu_wr_strobe,         // one clock per CPU write cycle
	output            aud_hit_wr,            // the device takes this write (mapper/PPU must not)
	output            aud_hit_rd,            // the device drives this read
	output      [7:0] aud_rdata,
	input      [15:0] nes_sample,            // nes.v's sample (unsigned)
	output            aud_active,            // pack audio on: AUDIO_S = 1, out_l/out_r
	output     [15:0] aud_l,
	output     [15:0] aud_r,
	input             aud_ss_save,           // savestate (core clock): hd3_audio's two registers
	input             aud_ss_load,
	input      [63:0] aud_ss_in_a,
	input      [63:0] aud_ss_in_b,
	output     [63:0] aud_ss_out_a,
	output     [63:0] aud_ss_out_b,

	// DDR3 (HD clock)
	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output     [28:0] DDRAM_ADDR,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY,
	output            DDRAM_RD,
	output     [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	output            DDRAM_WE,

	// picture (core clock)
	output            active,                // pack loaded and "HD Pack" on
	output            nolimit,               // the pack's disableSpriteLimit, while active
	output      [7:0] R,
	output      [7:0] G,
	output      [7:0] B,
	output            HS,
	output            VS,
	output            HB,
	output            VB,

	// status, for debug and the simulations
	output      [3:0] reject,                // why the last pack was refused (hd3_loader)
	output            underflow,             // sticky: a line was shown before it was drawn
	output reg [15:0] frames_drawn = 0,      // HD clock
	output reg [15:0] frames_dropped = 0,    // HD clock
	output reg [15:0] adds_aborted = 0,      // HD clock
	output reg        overrun = 0,           // sticky: capture began rewriting the bank in use
	output            dict_overflow,         // sticky (hd3_dict)
	output     [15:0] aud_underruns,         // hd3_audio: frames a voice had no data for (HD clock)
	output     [15:0] aud_ev_lost            // hd3_audio: CPU writes dropped, its queue full (core clock)
);

localparam integer RAW = 11;
localparam integer RSW = 10;

// ==========================================================================================
// pack loader (core clock)
// ==========================================================================================
wire [28:0] aud_off;
wire [31:0] aud_len;
wire        pack_ok;
wire  [7:0] ld_ack_seq;                  // the last load command carried out
wire        ld_st_t;                     // toggles when pack_ok / reject / ld_ack_seq change
wire        ld_req, ld_ready;            // the loader's reads (the single-access crossing)
wire [27:3] ld_addr;
wire [63:0] ssc_dout;
wire  [7:0] scale, flags;
wire [31:0] options, bucket_mask, hdr2c, hdr30, hdr34, fb_len, abucket_mask;
wire [15:0] max_probe, amax_probe;
wire [27:3] keys_base, rules_base, fb_base, akeys_base, alists_base;
wire [27:3] conds_base, clist_base, blocks_base, bgl_base, bgrec_base, pal_base;
wire [27:0] bgmap_base, ratio_base, bgi_base;
wire [31:0] patch_len;
wire  [7:0] ov_top, ov_right, ov_bottom, ov_left;
wire        game_ram, legacy, add_en, has_pal, cache_en, render_orig;

hd3_loader loader (
	.clk(clk), .reset(reset),
	.m_req(ld_req), .m_addr(ld_addr), .m_dout(ssc_dout), .m_ready(ld_ready),
	.pack_ok(pack_ok), .reject(reject), .ack_seq(ld_ack_seq), .st_t(ld_st_t),
	.scale(scale), .flags(flags), .options(options), .bucket_mask(bucket_mask),
	.max_probe(max_probe), .keys_base(keys_base), .rules_base(rules_base),
	.hdr_word0x2c(hdr2c), .hdr_word0x30(hdr30), .hdr_word0x34(hdr34),
	.cart_no_chr_rom(cart_no_chr_rom), .game_ram(game_ram), .legacy(legacy),
	.fb_base(fb_base), .fb_len(fb_len),
	.add_en(add_en), .akeys_base(akeys_base), .abucket_mask(abucket_mask),
	.amax_probe(amax_probe), .alists_base(alists_base),
	.ov_top(ov_top), .ov_right(ov_right), .ov_bottom(ov_bottom), .ov_left(ov_left),
	.has_pal(has_pal), .cache_en(cache_en), .render_orig(render_orig),
	.conds_base(conds_base), .clist_base(clist_base), .blocks_base(blocks_base),
	.bgl_base(bgl_base), .bgrec_base(bgrec_base), .bgmap_base(bgmap_base),
	.ratio_base(ratio_base), .pal_base(pal_base), .bgi_base(bgi_base),
	.patch_len(patch_len),
	.aud_off(aud_off), .aud_len(aud_len),
	.lw_we(lw_we), .lw_idx(lw_idx), .lw_addr(lw_addr)
);

assign active  = enable & pack_ok;
assign nolimit = active & options[0];
assign list_n  = pack_ok ? hdr34[8:0] : 9'd0;    // the loader refuses more than 256

// ==========================================================================================
// crossings into the HD clock
// ==========================================================================================
// the engine runs only while the HD picture is wanted; a new pack restarts it
reg reset_f = 1;
always @(posedge clk_f) reset_f <= reset | ~active;

assign load_wait = 1'b0;                 // no OSD file transfer to hold (see the ports)

// record writes
wire        cwf_v, cwf_r;
wire [96:0] cwf_d;
hd3_xq #(.W(97)) cwx (
	.clk(clk), .i_valid(cw_valid), .i_data({cw_addr, cw_data, cw_be}), .i_ready(cw_ready),
	.clk_f(clk_f), .o_valid(cwf_v), .o_data(cwf_d), .o_ready(cwf_r)
);
// the tap hd3_dict and hd3_fmap snoop: every record qword hd3_ddr takes
wire        tap_v  = cwf_v & cwf_r;
wire [27:3] tap_a  = cwf_d[96:72];
wire [63:0] tap_d  = cwf_d[71:8];
wire  [7:0] tap_be = cwf_d[7:0];

// ROM patches (stage 5): stages every ROM download and replays the ROM the ARM asks for (the ARM
// builds the patched one: tools/armdec/romfix_arm.c). Not gated by the pack: it must see the ROM
// whichever is loaded first.
wire        rf_req, rf_rnw, rf_ready;
wire [27:3] rf_addr;
wire [63:0] rf_din;
wire  [7:0] rf_be;
wire        ssc_ready;
hd3_romfix romfix (
	.clk(clk),
	.dl(rom_dl), .is_rom(rom_is_nes), .wr(rom_wr), .din(rom_din), .addr(rom_addr), .ft(rom_ft),
	.stall(rom_stall),
	.o_dl(rom_o_dl), .o_wr(rom_o_wr), .o_din(rom_o_din), .o_addr(rom_o_addr), .o_ft(rom_o_ft),
	.core_wait(rom_core_wait), .hold(rom_hold),
	.active(active), .plen(patch_len),
	.m_req(rf_req), .m_rnw(rf_rnw), .m_addr(rf_addr), .m_din(rf_din), .m_be(rf_be),
	.m_dout(ssc_dout), .m_ready(rf_ready),
	.content(rom_content), .result(rom_result)
);

// savestates, the ROM patcher and the pack loader share one single-access crossing. Each
// requests with a pulse and is answered with a pulse; a request waits (latched) while another
// is in flight. Priority: savestates, then the ROM patcher, then the loader (reads only).
localparam [1:0] OWN_SS = 2'd0, OWN_RF = 2'd1, OWN_LD = 2'd2;
reg         ssa_p = 0, rfa_p = 0, lda_p = 0, x_busy = 0, x_req = 0;
reg   [1:0] x_own = OWN_SS;
reg  [27:1] ssa_addr, x_addr;
reg  [63:0] ssa_din, x_din;
reg   [7:0] ssa_be, x_be;
reg         ssa_rnw, x_rnw;
reg  [27:3] rfa_addr, lda_addr;
reg  [63:0] rfa_din;
reg   [7:0] rfa_be;
reg         rfa_rnw;
always @(posedge clk) begin
	x_req <= 0;
	if (ss_req) begin ssa_p <= 1; ssa_addr <= ss_addr; ssa_din <= ss_din; ssa_be <= ss_be; ssa_rnw <= ss_rnw; end
	if (rf_req) begin rfa_p <= 1; rfa_addr <= rf_addr; rfa_din <= rf_din; rfa_be <= rf_be; rfa_rnw <= rf_rnw; end
	if (ld_req) begin lda_p <= 1; lda_addr <= ld_addr; end
	if (ssc_ready) x_busy <= 0;
	else if (!x_busy && !x_req) begin
		if (ssa_p) begin
			ssa_p <= 0; x_busy <= 1; x_own <= OWN_SS; x_req <= 1;
			x_addr <= ssa_addr; x_din <= ssa_din; x_be <= ssa_be; x_rnw <= ssa_rnw;
		end else if (rfa_p) begin
			rfa_p <= 0; x_busy <= 1; x_own <= OWN_RF; x_req <= 1;
			x_addr <= {rfa_addr, 2'b00}; x_din <= rfa_din; x_be <= rfa_be; x_rnw <= rfa_rnw;
		end else if (lda_p) begin
			lda_p <= 0; x_busy <= 1; x_own <= OWN_LD; x_req <= 1;
			x_addr <= {lda_addr, 2'b00}; x_din <= 64'd0; x_be <= 8'h00; x_rnw <= 1'b1;
		end
	end
end
assign ss_dout  = ssc_dout;
assign ss_ack   = ssc_ready & (x_own == OWN_SS);
assign rf_ready = ssc_ready & (x_own == OWN_RF);
assign ld_ready = ssc_ready & (x_own == OWN_LD);

// savestates
wire [27:1] ssf_addr;
wire [63:0] ssf_din, ssf_dout;
wire  [7:0] ssf_be;
wire        ssf_rnw, ssf_req, ssf_ready;
hd3_ss_cross ssx (
	.clk(clk), .addr(x_addr), .din(x_din), .be(x_be), .rnw(x_rnw), .req(x_req),
	.dout(ssc_dout), .ready(ssc_ready),
	.clk_f(clk_f), .f_addr(ssf_addr), .f_din(ssf_din), .f_be(ssf_be), .f_rnw(ssf_rnw),
	.f_req(ssf_req), .f_dout(ssf_dout), .f_ready(ssf_ready)
);

// watch snapshot complete: the bank and the frame number ride with a toggle. frame_num is
// N + 1 by then (it advances at the end of line 239 of N), so the snapshot is frame N's.
reg        wd_t = 0, wd_b = 0;
reg [31:0] wd_f = 0;
always @(posedge clk) if (watch_done) begin
	wd_b <= ~frame_num[0]; wd_f <= frame_num - 32'd1; wd_t <= ~wd_t;
end

// ==========================================================================================
// DDR3 (HD clock)
// ==========================================================================================
localparam integer NR = 4;      // 0 paint list, 1 paint block rows, 2 mailbox, 3 audio
wire [NR-1:0]    rd_req, rd_ack, rd_valid;
wire [NR*25-1:0] rd_addr;
wire [63:0]      rd_data;
wire             aud_rd_hi;
wire             aud_frame_done;      // hd3_audio's per-frame strobe: used only by tb_audio
wire [NR-1:0]    rd_hi = {aud_rd_hi, 3'd0};   // only audio reads the overflow window

// one write port for the frame records, the watch copier, the pack-state word and HD audio's
// event log, in that order of priority: records first (their FIFO must not back up while a line
// is captured); the copier runs after line 239, when records are idle; the pack word is rare;
// audio's log is a FIFO with a tick (~1,800 clocks) to spare
wire        aw_v;
wire [27:3] aw_a;
wire [63:0] aw_d;
wire        ww_v;
wire [27:3] ww_a;
wire [63:0] ww_d;
// pack-state word, byte 0x3C400008 (the qword after the mailbox), for the ARM program:
// [31:0] generation (+1 each time a pack becomes valid), [39:36] reject (hd3_loader; 0 =
// accepted), [47:40] sequence of the last load command carried out, [48] pack valid, [63:56]
// 0x5A. Written at power-up (so stale DDR3 contents never read as a pack: all zero but the
// 0x5A, which the ARM also reads as "freshly powered up") and every time the loader answers a
// command. The pack itself is the .hdp at byte 0x30000000 (file bytes from 192 MB on at
// 0x24000000), copied there by the ARM.
localparam [24:0] PACK_Q = 25'h1880001;
reg         pk_v = 1;
reg  [63:0] pk_d = {8'h5A, 56'd0};
reg   [2:0] pk_s = 0;                   // ld_st_t synchronised into the HD clock
reg  [31:0] pk_gen = 0;
wire        dw_ready;
wire        dw_valid = cwf_v | ww_v | pk_v | aw_v;
assign      cwf_r    = dw_ready & cwf_v;
wire        ww_take  = dw_ready & ~cwf_v & ww_v;
wire        pk_take  = dw_ready & ~cwf_v & ~ww_v & pk_v;
wire        aw_ready = dw_ready & ~cwf_v & ~ww_v & ~pk_v;   // taken on this clock when aw_v
always @(posedge clk_f) begin
	pk_s <= {pk_s[1:0], ld_st_t};
	if (pk_take) pk_v <= 0;
	// an answer after the take wins: the newer value is written. pack_ok, reject and
	// ld_ack_seq were set with the toggle and hold until the next command (milliseconds away),
	// so they are stable by the time the synchronised toggle edge arrives.
	if (pk_s[2] != pk_s[1]) begin
		pk_v    <= 1;
		pk_d    <= {8'h5A, 7'd0, pack_ok, ld_ack_seq, reject, 4'd0,
		            pack_ok ? pk_gen + 32'd1 : pk_gen};
		if (pack_ok) pk_gen <= pk_gen + 32'd1;
	end
end

hd3_ddr #(.NR(NR)) ddr (
	.DDRAM_CLK(clk_f), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD(DDRAM_RD), .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
	.ch1_addr(ssf_addr), .ch1_dout(ssf_dout), .ch1_din(ssf_din), .ch1_req(ssf_req),
	.ch1_rnw(ssf_rnw), .ch1_be(ssf_be), .ch1_ready(ssf_ready),
	.wr_req(1'b0), .wr_hi(1'b0), .wr_addr(25'd0), .wr_data(64'd0),   // the ARM writes the pack
	.wr_ack(),
	.cw_valid(dw_valid),
	.cw_addr(cwf_v ? cwf_d[96:72] : ww_v ? ww_a : pk_v ? PACK_Q : aw_a),
	.cw_data(cwf_v ? cwf_d[71:8]  : ww_v ? ww_d : pk_v ? pk_d   : aw_d),
	.cw_be  (cwf_v ? cwf_d[7:0]   : 8'hFF),
	.cw_ready(dw_ready),
	.rd_req(rd_req), .rd_hi(rd_hi), .rd_addr(rd_addr), .rd_ack(rd_ack), .rd_data(rd_data),
	.rd_valid(rd_valid)
);

// ==========================================================================================
// HD audio (stage 6): reader 3. Not gated by the picture's reset_f: Mesen's device keeps
// playing whatever the picture does; it stops only on a new pack.
// ==========================================================================================
hd3_audio audio (
	.clk(clk), .clk_f(clk_f), .pal(pal),
	.enable(enable), .pack_ok(pack_ok), .aud_off(aud_off), .aud_len(aud_len),
	.alt_regs(options[1]),
	.addr(cpu_a), .wdata(cpu_dout), .wr_strobe(cpu_wr_strobe),
	.hit_wr(aud_hit_wr), .hit_rd(aud_hit_rd), .rdata(aud_rdata),
	.nes_sample(nes_sample), .active(aud_active), .out_l(aud_l), .out_r(aud_r),
	.underruns(aud_underruns), .ev_lost(aud_ev_lost), .frame_done(aud_frame_done),
	.ss_save(aud_ss_save), .ss_load(aud_ss_load), .ss_in_a(aud_ss_in_a), .ss_in_b(aud_ss_in_b),
	.ss_out_a(aud_ss_out_a), .ss_out_b(aud_ss_out_b),
	.rd_req(rd_req[3]), .rd_hi(aud_rd_hi), .rd_addr(rd_addr[3*25 +: 25]), .rd_ack(rd_ack[3]),
	.rd_data(rd_data), .rd_valid(rd_valid[3]),
	.aw_valid(aw_v), .aw_addr(aw_a), .aw_data(aw_d), .aw_ready(aw_ready)
);

// ==========================================================================================
// watch copier (HD clock): hd_watch's values for the finished snapshot -> DDR3, then the marker
// ==========================================================================================
localparam [24:0] REC_Q  = 25'h1800000;   // byte 0xC000000: record bank 0 (bank 1: + 0x20000)
localparam [24:0] WVAL_Q = 25'h001E020;   // + byte 0xF0100 inside a bank
localparam [24:0] WMRK_Q = 25'h001E040;   // + byte 0xF0200

localparam [2:0] WW_IDLE = 3'd0, WW_SET = 3'd1, WW_GAP = 3'd2, WW_CAP = 3'd3, WW_PUT = 3'd4,
                 WW_MARK = 3'd5;
reg  [2:0] wst = WW_IDLE;
reg        wd_d = 0;
reg        wbk = 0;
reg [31:0] wfr = 0;
reg  [8:0] wn = 0, wi = 0;
reg [63:0] wacc = 0;
reg  [4:0] wq = 0;                        // qword being filled
reg        r_ww_v = 0;
reg [27:3] r_ww_a = 0;
reg [63:0] r_ww_d = 0;
reg  [7:0] r_w1_idx = 0;
assign ww_v = r_ww_v; assign ww_a = r_ww_a; assign ww_d = r_ww_d;
assign w1_bank = wbk;
assign w1_idx  = r_w1_idx;
assign w2_bank = 1'b0;
assign w2_idx  = 8'd0;
wire [24:0] wbase = REC_Q + {7'd0, wbk, 17'd0};

always @(posedge clk_f) begin
	wd_d <= wd_t;
	if (ww_take) r_ww_v <= 0;
	case (wst)
	WW_IDLE: if (wd_t ^ wd_d) begin
		wbk  <= wd_b;
		wfr  <= wd_f;
		wn   <= (list_n > 9'd256) ? 9'd256 : list_n;
		wi   <= 0;
		wq   <= 0;
		wacc <= 0;
		wst  <= (list_n == 9'd0) ? WW_MARK : WW_SET;
	end
	WW_SET: begin r_w1_idx <= wi[7:0]; wst <= WW_GAP; end
	WW_GAP: wst <= WW_CAP;                 // w1_q is hd_watch's register, valid two clocks on
	WW_CAP: begin
		wacc[8 * wi[2:0] +: 8] <= w1_q;
		wi <= wi + 9'd1;
		if (wi[2:0] == 3'd7 || wi + 9'd1 == wn) wst <= WW_PUT;
		else wst <= WW_SET;
	end
	WW_PUT: if (!r_ww_v) begin
		r_ww_v <= 1;
		r_ww_a <= wbase + WVAL_Q + {20'd0, wq};
		r_ww_d <= wacc;
		wacc   <= 0;
		wq     <= wq + 5'd1;
		wst    <= (wi == wn) ? WW_MARK : WW_SET;
	end
	WW_MARK: if (!r_ww_v) begin
		r_ww_v <= 1;
		r_ww_a <= wbase + WMRK_Q;
		r_ww_d <= {23'd0, wn, wfr};
		wst    <= WW_IDLE;
	end
	default: wst <= WW_IDLE;
	endcase
	if (reset_f) begin wst <= WW_IDLE; r_ww_v <= 0; end
end

// ==========================================================================================
// mailbox poller (HD clock): reader client 2, one 8-qword burst every 2048 clocks, first beat kept
// ==========================================================================================
localparam [24:0] LIST_Q = 25'h1840000;   // byte 0xC200000: list buffer 0
localparam [24:0] MBOX_Q = 25'h1880000;   // byte 0xC400000

reg        mb_req = 0;
reg [10:0] mb_tmr = 0;
reg  [3:0] mb_beats = 0;
reg        mb_seen = 0;
reg [63:0] mb_q = 0;
assign rd_req[2] = mb_req;
assign rd_addr[2*25 +: 25] = MBOX_Q;
always @(posedge clk_f) begin
	mb_tmr <= mb_tmr + 11'd1;
	if (mb_tmr == 11'd0 && !mb_req && mb_beats == 0) mb_req <= 1;
	if (rd_ack[2]) begin mb_req <= 0; mb_beats <= 4'd8; end
	if (rd_valid[2]) begin
		if (mb_beats == 4'd8) begin mb_q <= rd_data; mb_seen <= 1; end
		mb_beats <= mb_beats - 4'd1;
	end
	if (reset_f) begin mb_req <= 0; mb_beats <= 0; mb_seen <= 0; end
end
wire        mb_ok  = mb_seen && mb_q[63:56] == 8'hA5 && mb_q[31:0] != 32'd0;
wire [31:0] mb_fid = mb_q[31:0];
wire [23:0] mb_cnt = mb_q[55:32];

// ==========================================================================================
// painter (HD clock)
// ==========================================================================================
reg  [31:0] p_fid = 0;                  // frame id being (or last) painted
reg         p_any = 0;                  // a frame has been started since reset
wire [24:0] p_base = LIST_Q + {7'd0, p_fid[1:0], 16'd0};
wire [24:0] p_end  = (mb_ok && mb_fid == p_fid) ? p_base + {1'b0, mb_cnt}
                                                : p_base + 25'h0010000;   // whole buffer

wire        pt_busy, pt_done;
wire        out_v;
wire  [7:0] out_y, out_x;
wire [95:0] out_px;
wire        line_ok;
wire  [7:0] rn_line;
reg         r_pt_start = 0;

hd3_paint paint (
	.clk(clk_f), .reset(reset_f),
	.start(r_pt_start), .list_base(p_base), .list_end(p_end), .blocks_base(blocks_base),
	.ov_top(ov_top), .ov_bottom(ov_bottom),
	.busy(pt_busy), .done(pt_done),
	.l_rd_req(rd_req[0]), .l_rd_addr(rd_addr[0*25 +: 25]), .l_rd_ack(rd_ack[0]),
	.rd_data(rd_data), .l_rd_valid(rd_valid[0]),
	.b_rd_req(rd_req[1]), .b_rd_addr(rd_addr[1*25 +: 25]), .b_rd_ack(rd_ack[1]),
	.b_rd_valid(rd_valid[1]),
	.line_ok(line_ok), .line_y(rn_line),
	.out_v(out_v), .out_y(out_y), .out_x(out_x), .out_px(out_px)
);

// ==========================================================================================
// controller (HD clock)
// ==========================================================================================
localparam [1:0] C_IDLE = 2'd0, C_START = 2'd1, C_PAINT = 2'd2;
reg  [1:0] cst = C_IDLE;
reg  [1:0] tgt = 0;                     // scan-out frame id of the frame being painted

// scan-out position, read from the core clock (changes once per NES line)
wire  [8:0] disp_y;
wire  [1:0] dcount;

// the painter's progress, read by the scan-out
reg  [1:0] p_cur_t = 0, p_fin_t = 0;
reg  [8:0] p_cur_y = 0;
reg        p_fin_v = 0;
reg        p_run = 0;

wire [31:0] fid_gap = mb_fid - p_fid;   // frames the ARM published since the last one started

// no ADD stage and no dictionary on this route: adds_aborted keeps its initial 0
assign dict_overflow = 1'b0;

always @(posedge clk_f) begin
	r_pt_start <= 0;
	case (cst)
	C_IDLE: if (mb_ok && mb_cnt != 24'd0 && (!p_any || mb_fid != p_fid)) begin
		if (p_any && fid_gap > 32'd1)
			frames_dropped <= frames_dropped + fid_gap[15:0] - 16'd1;
		p_fid <= mb_fid;
		p_any <= 1;
		tgt   <= (disp_y >= 9'd240) ? dcount : dcount + 2'd1;
		cst   <= C_START;
	end
	C_START: begin                          // p_fid (hence list_base) settled last clock
		r_pt_start <= 1;
		p_cur_t <= tgt; p_cur_y <= 0; p_run <= 1;
		cst <= C_PAINT;
	end
	C_PAINT: begin
		if (out_v) p_cur_y <= {1'b0, out_y};
		if (pt_done) begin
			p_cur_y <= 9'd256;
			p_fin_t <= p_cur_t;
			p_fin_v <= 1;
			frames_drawn <= frames_drawn + 16'd1;
			cst <= C_IDLE;
		end
	end
	default: cst <= C_IDLE;
	endcase

	// overrun: the ARM side owns record reuse on this route (it copies a record out before the
	// capture two frames later rewrites that bank), so the engine has nothing to flag
	overrun <= 0;

	if (reset_f) begin
		cst <= C_IDLE; p_any <= 0; p_fin_v <= 0; p_cur_y <= 0; p_run <= 0;
	end
end

// a line may start once the ring slot it goes to has been shown: the frame painted is shown as
// frame id tgt; while the scan-out is still on the frame before (id tgt - 1), wait; in the
// blanking before frame tgt the first 8 lines may go; while frame tgt is shown up to line d,
// lines up to d + 7. A frame that is already late (the scan-out is past it) is painted at full
// speed.
wire [1:0] fdiff = tgt - dcount;
assign line_ok = (fdiff == 2'd0) ? ((disp_y < 9'd240) ? ({1'b0, rn_line} <= disp_y + 9'd7)
                                                       : (rn_line < 8'd8))
               : (fdiff != 2'd1);

// ==========================================================================================
// scan-out (core clock)
// ==========================================================================================
hd3_scanout scan (
	.clk_f(clk_f), .w_en(out_v), .w_y(out_y), .w_x(out_x), .w_px(out_px),
	.p_cur_t(p_cur_t), .p_cur_y(p_cur_y), .p_fin_t(p_fin_t), .p_fin_v(p_fin_v), .p_run(p_run),
	.clk(clk), .line_done(line_done), .line_y(line_y),
	.ov_top(ov_top), .ov_bottom(ov_bottom), .ov_left(ov_left), .ov_right(ov_right),
	.disp_y(disp_y), .dcount(dcount), .underflow(underflow),
	.R(R), .G(G), .B(B), .HS(HS), .VS(VS), .HB(HB), .VB(VB)
);

endmodule
