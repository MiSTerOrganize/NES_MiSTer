// hd3_engine.sv - the stage-3 HD pack engine (HDPACK_DESIGN.md "Integration (stage 3)").
// Replaces hd_engine.sv (the MVP) and hd_ddr.sv.
//
//   core clock (clk, 21.477 MHz)          HD clock (clk_f, 85.909 MHz = 4 x clk, same PLL)
//   ------------------------------        ----------------------------------------------------
//   hd3_loader  (.hdp from the OSD)  -->  hd3_ddr   (DDR3: savestates, pack, record, 7 readers)
//   hd_stage2's record writer (cw_*) -->  hd3_dict / hd3_fmap  (both snoop the record writes)
//   savestate channel (ss_*)         -->  hd3_add -> hd3_resolve -> hd3_render, frame by frame
//   hd_stage2's watch snapshot       <--  (read ports on clk_f, hd_watch's dual-clock store)
//   hd3_scanout (picture out)        <--  RENDER's 2x2 pixels into the 8-line ring
// The clocks are related (see hd3_cross.sv); every crossing is a hd3_cross.sv module, a
// dual-clock memory, or a register read in the other domain that only changes slowly.
//
// Frame sequence (the controller below): capture of frame N writes record bank N[0]; when
// hd3_dict reports every record of frame N resolved (frame_done), ADD then RESOLVE prepare it
// and RENDER draws it. The dictionary and lane read ports are owned by one client at a time -
// ADD, then RESOLVE, then RENDER - so a frame is taken up only once RENDER has finished the
// one before (the measured worst case leaves RENDER N-1 done by then; HDPACK_DESIGN.md 8m).
// RESOLVE also needs the watched-address values of frame N, so it waits for that snapshot
// (when the pack watches any address). ADD has a clock budget: at ADD_BUDGET it is aborted,
// which ends it with ovf, and the frame is drawn without lookups, as a dictionary overflow is.
// If a newer frame finishes capture before the controller took the older one up, the older one
// is dropped (frames_dropped).
//
// Pacing: RENDER tags its frame with the scan-out frame id it will be shown as and may start
// line y only when the ring slot y mod 8 has been shown (hd3_scanout, hd3_render line_ok).

module hd3_engine #(
	parameter integer NL         = 16,
	parameter integer RS_MAX     = 1024,
	parameter integer ADD_BUDGET = 60000     // HD clocks; ~11 NES lines
)
(
	input             clk,
	input             clk_f,
	input             reset,                 // core clock: the NES reset
	input             enable,                // "HD Pack" OSD option

	// pack load (core clock)
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
wire        ld_wr_req, ld_wr_hi;
wire [27:3] ld_wr_addr;
wire [28:0] aud_off;
wire [31:0] aud_len;
wire [63:0] ld_wr_data;
reg         ld_wr_ack = 0;
wire        pack_ok;
wire  [7:0] scale, flags;
wire [31:0] options, bucket_mask, hdr2c, hdr30, hdr34, fb_len, abucket_mask;
wire [15:0] max_probe, amax_probe;
wire [27:3] keys_base, rules_base, fb_base, akeys_base, alists_base;
wire [27:3] conds_base, clist_base, blocks_base, bgl_base, bgrec_base, pal_base;
wire [27:0] bgmap_base, ratio_base, bgi_base;
wire [31:0] patch_len;
wire  [7:0] ov_top, ov_right, ov_bottom, ov_left;
wire        game_ram, legacy, add_en, has_pal, cache_en, render_orig;

hd3_loader_stream loader (
	.clk(clk), .reset(reset),
	.load_active(load_active), .load_wr(load_wr), .load_data(load_data), .load_wait(load_wait),
	.wr_req(ld_wr_req), .wr_hi(ld_wr_hi), .wr_addr(ld_wr_addr), .wr_data(ld_wr_data),
	.wr_ack(ld_wr_ack),
	.pack_ok(pack_ok), .reject(reject),
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

// pack-load writes: the loader holds wr_req until wr_ack
wire        ldx_ready, ldf_v;
wire [89:0] ldf_d;                       // {hi, qword address, data}
wire        ld_take = ld_wr_req & ~ld_wr_ack & ldx_ready;
always @(posedge clk) ld_wr_ack <= ld_take;
wire        ddr_wr_ack;
hd3_xq #(.W(90)) ldx (
	.clk(clk), .i_valid(ld_wr_req & ~ld_wr_ack), .i_data({ld_wr_hi, ld_wr_addr, ld_wr_data}),
	.i_ready(ldx_ready),
	.clk_f(clk_f), .o_valid(ldf_v), .o_data(ldf_d), .o_ready(ddr_wr_ack)
);

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
wire [63:0] ssc_dout;
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

// savestates and the ROM patcher share one single-access crossing. Both request with a pulse
// and are answered with a pulse; a request waits (latched) while the other is in flight.
reg         ssa_p = 0, rfa_p = 0, x_busy = 0, x_own = 0, x_req = 0;
reg  [27:1] ssa_addr, x_addr;
reg  [63:0] ssa_din, x_din;
reg   [7:0] ssa_be, x_be;
reg         ssa_rnw, x_rnw;
reg  [27:3] rfa_addr;
reg  [63:0] rfa_din;
reg   [7:0] rfa_be;
reg         rfa_rnw;
always @(posedge clk) begin
	x_req <= 0;
	if (ss_req) begin ssa_p <= 1; ssa_addr <= ss_addr; ssa_din <= ss_din; ssa_be <= ss_be; ssa_rnw <= ss_rnw; end
	if (rf_req) begin rfa_p <= 1; rfa_addr <= rf_addr; rfa_din <= rf_din; rfa_be <= rf_be; rfa_rnw <= rf_rnw; end
	if (ssc_ready) x_busy <= 0;
	else if (!x_busy && !x_req) begin
		if (ssa_p) begin
			ssa_p <= 0; x_busy <= 1; x_own <= 0; x_req <= 1;
			x_addr <= ssa_addr; x_din <= ssa_din; x_be <= ssa_be; x_rnw <= ssa_rnw;
		end else if (rfa_p) begin
			rfa_p <= 0; x_busy <= 1; x_own <= 1; x_req <= 1;
			x_addr <= {rfa_addr, 2'b00}; x_din <= rfa_din; x_be <= rfa_be; x_rnw <= rfa_rnw;
		end
	end
end
assign ss_dout  = ssc_dout;
assign ss_ack   = ssc_ready & ~x_own;
assign rf_ready = ssc_ready &  x_own;

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

// watch snapshot complete: the bank rides with a toggle
reg wd_t = 0, wd_b = 0;
always @(posedge clk) if (watch_done) begin wd_b <= ~frame_num[0]; wd_t <= ~wd_t; end

// ==========================================================================================
// DDR3 (HD clock)
// ==========================================================================================
localparam integer NR = 8;      // 0 dict, 1 add, 2 resolve, 3 render L, 4 C, 5 E, 6 B, 7 audio
wire [NR-1:0]    rd_req, rd_ack, rd_valid;
wire [NR*25-1:0] rd_addr;
wire [63:0]      rd_data;
wire             aud_rd_hi;
wire             aud_frame_done;      // hd3_audio's per-frame strobe: used only by tb_audio
wire [NR-1:0]    rd_hi = {aud_rd_hi, 7'd0};   // only audio reads the overflow window

// one write port for the frame records and HD audio's event log, records first (their FIFO
// must not back up while a line is captured; audio's log is a FIFO with a tick to spare)
wire        aw_v;
wire [27:3] aw_a;
wire [63:0] aw_d;
wire        dw_ready;
assign      cwf_r    = dw_ready & cwf_v;
wire        aw_ready = dw_ready & ~cwf_v;          // taken on this clock when aw_v

hd3_ddr #(.NR(NR)) ddr (
	.DDRAM_CLK(clk_f), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD(DDRAM_RD), .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
	.ch1_addr(ssf_addr), .ch1_dout(ssf_dout), .ch1_din(ssf_din), .ch1_req(ssf_req),
	.ch1_rnw(ssf_rnw), .ch1_be(ssf_be), .ch1_ready(ssf_ready),
	.wr_req(ldf_v), .wr_hi(ldf_d[89]), .wr_addr(ldf_d[88:64]), .wr_data(ldf_d[63:0]),
	.wr_ack(ddr_wr_ack),
	.cw_valid(cwf_v | aw_v),
	.cw_addr(cwf_v ? cwf_d[96:72] : aw_a),
	.cw_data(cwf_v ? cwf_d[71:8]  : aw_d),
	.cw_be  (cwf_v ? cwf_d[7:0]   : 8'hFF),
	.cw_ready(dw_ready),
	.rd_req(rd_req), .rd_hi(rd_hi), .rd_addr(rd_addr), .rd_ack(rd_ack), .rd_data(rd_data),
	.rd_valid(rd_valid)
);

// ==========================================================================================
// HD audio (stage 6): reader 7. Not gated by the picture's reset_f: Mesen's device keeps
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
	.rd_req(rd_req[7]), .rd_hi(aud_rd_hi), .rd_addr(rd_addr[7*25 +: 25]), .rd_ack(rd_ack[7]),
	.rd_data(rd_data), .rd_valid(rd_valid[7]),
	.aw_valid(aw_v), .aw_addr(aw_a), .aw_data(aw_d), .aw_ready(aw_ready)
);

// ==========================================================================================
// dictionary, lanes, rule lists, frame maps, renderer (HD clock) - wired as hd3_render_tb
// ==========================================================================================
wire           frame_done, done_bank;
wire     [1:0] dict_fovf;
wire           ad_start, ad_bank, ad_abort, ad_busy, ad_done, ad_ovf;
wire           rs_start, rs_bank, rs_busy, rs_done, rs_overflow;
wire           rn_start, rn_bank, rn_busy, rn_done, rn_unsup;

wire           rs_dbank, rn_dbank;
wire [RAW-1:0] rs_drid, rn_drid;
wire    [95:0] r_key;
wire    [63:0] r_res;
wire     [7:0] rn_y;
wire     [5:0] rn_slot;
wire     [3:0] rn_fetch;
wire   [RAW:0] r_span, r_spr, r_nrec;
wire    [31:0] r_frame;
wire     [7:0] ad_y;
wire     [5:0] ad_slot;
wire     [3:0] ad_fetch;
wire [RAW-1:0] ad_rid, ad_arid;
wire     [9:0] ad_pi;
wire    [16:0] a_m;
wire [47+RAW:0] a_pe;
wire     [7:0] rs_ay, rn_ay;
wire     [6:0] rl_n;
wire [NL*(RAW+21)-1:0] rl_q;

hd3_dict #(.RAW(RAW)) dict (
	.clk(clk_f), .reset(reset_f),
	.game_ram(game_ram), .legacy(legacy),
	.keys_base(keys_base), .rules_base(rules_base),
	.bucket_mask(bucket_mask), .max_probe(max_probe),
	.fb_base(fb_base), .fb_len(fb_len),
	.add_en(add_en), .akeys_base(akeys_base), .abucket_mask(abucket_mask),
	.amax_probe(amax_probe), .alists_base(alists_base),
	.tap_v(tap_v), .tap_a(tap_a), .tap_d(tap_d),
	.rd_req(rd_req[0]), .rd_addr(rd_addr[0*25 +: 25]), .rd_ack(rd_ack[0]), .rd_data(rd_data),
	.rd_valid(rd_valid[0]),
	.r_bank(ad_busy ? ad_bank : rs_busy ? rs_dbank : rn_dbank),
	.r_rid(ad_busy ? ad_rid : rs_busy ? rs_drid : rn_drid),
	.r_key(r_key), .r_res(r_res),
	.r_y(ad_busy ? ad_y : rn_y), .r_slot(ad_busy ? ad_slot : rn_slot), .r_span(r_span),
	.r_fetch(ad_busy ? ad_fetch : rn_fetch), .r_spr(r_spr),
	.r_nrec(r_nrec), .r_frame(r_frame),
	.a_bank(ad_bank), .a_rid(ad_arid), .a_m(a_m), .a_pi(ad_pi), .a_pe(a_pe),
	.frame_done(frame_done), .done_bank(done_bank), .overflow(dict_overflow),
	.frame_ovf(dict_fovf)
);

hd3_add #(.RAW(RAW), .NL(NL)) add (
	.clk(clk_f), .reset(reset_f),
	.start(ad_start), .bank(ad_bank), .add_en(add_en), .abort(ad_abort), .busy(ad_busy),
	.done(ad_done), .ovf(ad_ovf),
	.r_y(ad_y), .r_slot(ad_slot), .r_span(r_span), .r_fetch(ad_fetch), .r_spr(r_spr),
	.r_rid(ad_rid), .r_key(r_key), .r_nrec(r_nrec),
	.a_rid(ad_arid), .a_m(a_m), .a_pi(ad_pi), .a_pe(a_pe),
	.rd_req(rd_req[1]), .rd_addr(rd_addr[1*25 +: 25]), .rd_ack(rd_ack[1]), .rd_data(rd_data),
	.rd_valid(rd_valid[1]),
	.rl_y(rs_busy ? rs_ay : rn_ay), .rl_n(rl_n), .rl_q(rl_q)
);

wire [RAW-1:0] q_rid;
wire [2+1+11+RSW+25-1:0] q_rec;
wire [RSW-1:0] q_ri;
wire    [72:0] q_rule;
wire           gs_bank, sd_bank;
wire     [5:0] gs_lvl;
wire    [20:0] gs_q;
wire    [10:0] sd_a;
wire     [1:0] sd_q;
wire           rs_bglin, rs_bgwarn;

hd3_resolve #(.RAW(RAW), .RSW(RSW), .RS_MAX(RS_MAX), .NL(NL)) resolve (
	.clk(clk_f), .reset(reset_f),
	.game_ram(game_ram), .legacy(legacy), .add_en(add_en), .d_key(r_key),
	.a_y(rs_ay), .a_n(rl_n), .a_q(rl_q),
	.rules_base(rules_base), .conds_base(conds_base), .clist_base(clist_base),
	.fb_base(fb_base), .fb_len(fb_len),
	.bgl_base(bgl_base), .bgi_base(bgi_base),
	.start(rs_start), .bank(rs_bank), .busy(rs_busy), .done(rs_done), .overflow(rs_overflow),
	.bg_lin(rs_bglin), .bg_warn(rs_bgwarn),
	.d_bank(rs_dbank), .d_rid(rs_drid), .d_res(r_res), .d_nrec(r_nrec), .d_frame(r_frame),
	.v_bank(w1_bank), .v_idx(w1_idx), .v_q(w1_q),
	.rd_req(rd_req[2]), .rd_addr(rd_addr[2*25 +: 25]), .rd_ack(rd_ack[2]), .rd_data(rd_data),
	.rd_valid(rd_valid[2]),
	.r_bank(rn_dbank), .r_rid(q_rid), .r_rec(q_rec), .r_ri(q_ri), .r_rule(q_rule),
	.g_bank(gs_bank), .g_lvl(gs_lvl), .g_q(gs_q), .s_bank(sd_bank), .s_a(sd_a), .s_q(sd_q)
);

wire  [14:0] p_a;
wire  [13:0] p_q;
wire   [7:0] l_y, s_y;
wire  [10:0] l_q;
wire [415:0] s_q;

hd3_fmap fmap (
	.clk(clk_f),
	.tap_v(tap_v), .tap_a(tap_a), .tap_d(tap_d), .tap_be(tap_be),
	.r_bank(rn_dbank),
	.p_a(p_a), .p_q(p_q), .l_y(l_y), .l_q(l_q), .s_y(s_y), .s_q(s_q)
);

// lanes incomplete for a bank (hd3_add ovf, or aborted): drawn without lookups
reg [1:0] add_ovf = 0;
always @(posedge clk_f) if (ad_done) add_ovf[ad_bank] <= ad_ovf;

wire        out_v;
wire  [7:0] out_y, out_x;
wire [95:0] out_px;
wire        line_ok;
wire  [7:0] rn_line;

hd3_render #(.RAW(RAW), .RSW(RSW), .NL(NL)) render (
	.clk(clk_f), .reset(reset_f),
	.game_ram(game_ram), .cache_en(cache_en), .render_orig(render_orig), .has_pal(has_pal),
	.ov_top(ov_top), .ov_bottom(ov_bottom), .ov_left(ov_left), .ov_right(ov_right),
	.rules_base(rules_base), .conds_base(conds_base), .clist_base(clist_base),
	.blocks_base(blocks_base), .pal_base(pal_base),
	.bgrec_base(bgrec_base), .bgmap_base(bgmap_base), .ratio_base(ratio_base),
	.add_en(add_en), .flush(reset_f),
	.start(rn_start), .bank(rn_bank), .frame_ovf(dict_fovf[rn_bank] || add_ovf[rn_bank]),
	.busy(rn_busy), .done(rn_done), .unsup(rn_unsup),
	.d_bank(rn_dbank), .d_rid(rn_drid), .d_key(r_key), .d_res(r_res),
	.d_y(rn_y), .d_slot(rn_slot), .d_span(r_span), .d_fetch(rn_fetch), .d_spr(r_spr),
	.d_frame(r_frame),
	.q_rid(q_rid), .q_rec(q_rec), .q_ri(q_ri), .q_rule(q_rule),
	.gs_bank(gs_bank), .gs_lvl(gs_lvl), .gs_q(gs_q), .sd_bank(sd_bank), .sd_a(sd_a), .sd_q(sd_q),
	.p_a(p_a), .p_q(p_q), .l_y(l_y), .l_q(l_q), .s_y(s_y), .s_q(s_q),
	.v_idx(w2_idx), .v_q(w2_q),
	.a_y(rn_ay), .a_n(rl_n), .a_q(rl_q),
	.l_rd_req(rd_req[3]), .l_rd_addr(rd_addr[3*25 +: 25]), .l_rd_ack(rd_ack[3]),
	.l_rd_data(rd_data), .l_rd_valid(rd_valid[3]),
	.c_rd_req(rd_req[4]), .c_rd_addr(rd_addr[4*25 +: 25]), .c_rd_ack(rd_ack[4]),
	.c_rd_data(rd_data), .c_rd_valid(rd_valid[4]),
	.e_rd_req(rd_req[5]), .e_rd_addr(rd_addr[5*25 +: 25]), .e_rd_ack(rd_ack[5]),
	.e_rd_data(rd_data), .e_rd_valid(rd_valid[5]),
	.b_rd_req(rd_req[6]), .b_rd_addr(rd_addr[6*25 +: 25]), .b_rd_ack(rd_ack[6]),
	.b_rd_data(rd_data), .b_rd_valid(rd_valid[6]),
	.out_v(out_v), .out_y(out_y), .out_x(out_x), .out_px(out_px),
	.line_ok(line_ok), .line_y(rn_line)
);
assign w2_bank = rn_dbank;

// ==========================================================================================
// controller (HD clock)
// ==========================================================================================
localparam [2:0] C_IDLE = 3'd0, C_ADD = 3'd1, C_WAITW = 3'd2, C_RES = 3'd3, C_REN = 3'd4;
reg  [2:0] cst = C_IDLE;
reg        pend_v = 0, pend_b = 0;      // a frame whose capture is complete, not taken up yet
reg        cur_b = 0;
reg  [1:0] watch_ok = 0;                // watched-address snapshot complete, per bank
reg        wd_d = 0;
reg [23:0] acnt;                        // ADD clocks; 24 bits so a test build can lift the budget
reg        r_ad_start = 0, r_rs_start = 0, r_rn_start = 0, r_ad_abort = 0;
reg  [1:0] tgt = 0;                     // scan-out frame id of the frame RENDER draws

// scan-out position, read from the core clock (changes once per NES line)
wire  [8:0] disp_y;
wire  [1:0] dcount;
wire        wneed = list_n != 9'd0;

// RENDER's progress, read by the scan-out
reg  [1:0] p_cur_t = 0, p_fin_t = 0;
reg  [8:0] p_cur_y = 0;
reg        p_fin_v = 0;
reg        p_run = 0;

assign ad_start = r_ad_start; assign ad_bank = cur_b; assign ad_abort = r_ad_abort;
assign rs_start = r_rs_start; assign rs_bank = cur_b;
assign rn_start = r_rn_start; assign rn_bank = cur_b;

always @(posedge clk_f) begin
	r_ad_start <= 0; r_rs_start <= 0; r_rn_start <= 0;
	wd_d <= wd_t;
	if (wd_t ^ wd_d) watch_ok[wd_b] <= 1;
	if (frame_done) begin
		if (pend_v) frames_dropped <= frames_dropped + 16'd1;
		pend_v <= 1; pend_b <= done_bank;
	end

	case (cst)
	C_IDLE: if (pend_v && !frame_done) begin
		cur_b  <= pend_b;
		pend_v <= 0;
		r_ad_start <= 1;
		acnt   <= 0;
		cst    <= C_ADD;
	end
	C_ADD: begin
		if (!r_ad_start && acnt != 24'hFFFFFF) acnt <= acnt + 24'd1;
		if (acnt >= 24'(ADD_BUDGET) && !r_ad_abort) begin
			r_ad_abort   <= 1;
			adds_aborted <= adds_aborted + 16'd1;
		end
		if (ad_done) begin
			r_ad_abort <= 0;
			cst <= C_WAITW;
		end
	end
	C_WAITW: if (!wneed || watch_ok[cur_b] || (wd_t ^ wd_d) && wd_b == cur_b) begin
		watch_ok[cur_b] <= 0;
		r_rs_start <= 1;
		cst <= C_RES;
	end
	C_RES: if (rs_done) begin
		tgt <= (disp_y >= 9'd240) ? dcount : dcount + 2'd1;
		r_rn_start <= 1;
		cst <= C_REN;
	end
	C_REN: begin
		if (r_rn_start) begin p_cur_t <= tgt; p_cur_y <= 0; p_run <= 1; end
		else if (out_v) p_cur_y <= {1'b0, out_y};
		if (rn_done) begin
			p_cur_y <= 9'd256;
			p_fin_t <= p_cur_t;
			p_fin_v <= 1;
			frames_drawn <= frames_drawn + 16'd1;
			cst <= C_IDLE;
		end
	end
	default: cst <= C_IDLE;
	endcase

	// capture writes frame M's records into bank M[0], so a record write into the bank still in
	// use means frame M - 2 is not finished when M begins (ADD + RESOLVE + RENDER over about a
	// frame): hd3_dict restarts the bank's keys from there, so whatever of M - 2 is still to
	// be prepared or drawn reads the new frame's dictionary. Its display period has ended by
	// then (it ends at scanline 241 of M - 1), so no wrong line is shown - the late lines were
	// already shown black (underflow) - but the frame is wasted work and delays M - 1.
	if (tap_v && tap_a[20] == cur_b && cst != C_IDLE) overrun <= 1;   // byte bit 20: the 1 MB bank

	if (reset_f) begin
		cst <= C_IDLE; pend_v <= 0; watch_ok <= 0; r_ad_abort <= 0;
		p_fin_v <= 0; p_cur_y <= 0; p_run <= 0;
	end
end

// a line may start once the ring slot it goes to has been shown: the frame RENDER draws is
// shown as frame id tgt; while the scan-out is still on the frame before (id tgt - 1), wait;
// in the blanking before frame tgt the first 8 lines may go; while frame tgt is shown up to
// line d, lines up to d + 7. A frame that is already late (the scan-out is past it) is drawn
// at full speed.
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
