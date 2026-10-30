// hd_stage2.sv - stage-2 HD pack blocks that run beside the MVP engine: the CHR-RAM
// fingerprint unit (hd_chrfp), the frame-record writer (hd_capture) and the watched-address
// snapshot (hd_watch, with hd_sdram_port for its SDRAM reads).
//
// Instantiated identically by NES.sv and the full-core simulation (tools/fullsim/sim_top.sv),
// so the glue between the PPU tap and these blocks exists in one place.
//
// Tap bus layout (rtl/ppu.sv assign hd_tap_bus): bits [315:0] as rtl/hdpack/hd_engine.sv
// documents, [319:316] zero, then
//   [320] px_branch_b  [335:321] px_t  [338:336] px_finex  [339] px_gray  [342:340] px_emph
//   [343] line_pre  [344] snap (scanline 240 dot 0)  [351:345] zero
//   [352] extra-sprite write  [355:353] its index in the extra group  [410:356] its entry
//   (same layout as [89:35])  [415:411] zero
//
// Extra sprites (the core's 16-sprite option) share the capture's one sprite port: the tap
// emits each extra entry one PPU dot after the main entry of the same fetch slot, so the two
// never coincide, and the record stores them at 8..15, after the 8 main fetches. That is the
// core's own priority order (SpriteSet: main 0-7 win over extra 8-15, each in fetch order).
//
// CHR RAM writes are taken from the PPU's own qualifier (nes.v hd_chr_wr = a PPU write with the
// mapper's CHR write enable and not a nametable access), because mappers place CHR RAM in
// different SDRAM regions (most at 0x200000, CPROM at 0x100000, an MMC3 variant at 0x3FF000);
// the low 15 bits of the linear address index the tile, as the tap's fetch addresses do.
// Savestate restores write SDRAM through their own channel (25-bit addresses); those are
// snooped by region, 0x0200000-0x02FFFFF, which covers every mapper except the two above.
//
// Watched addresses: the list comes from the format-3 pack loader (hd3_loader's lw_* port).
// Until stage 3 replaces the MVP loader with it, NES.sv ties the list off (list_n = 0, no
// snapshot work); fullsim drives it from a file. The snapshot is taken at the tap's snap pulse
// (scanline 240 dot 0) into value bank ~frame_num[0]: hd_capture advances frame_num at the end
// of line 239, so at the pulse frame_num is already N+1 for the frame N that just ended, and
// bank N[0] pairs the values with that frame's record buffer.

module hd_stage2
(
	input             clk,
	input             reset,          // PPU reset: frame numbering restarts at 1
	input             enable,         // "HD Pack" on

	input     [415:0] tap,
	input             chr_ram,        // mapper_flags[15]: the game has CHR RAM

	input             ppu_wr,         // CHR RAM write (nes.v hd_chr_wr)
	input      [21:0] ppu_addr,
	input       [7:0] ppu_data,
	input             ss_wr,          // savestate SDRAM write
	input      [24:0] ss_addr,
	input       [7:0] ss_data,

	output            cw_valid,
	output     [27:3] cw_addr,
	output     [63:0] cw_data,
	output      [7:0] cw_be,
	input             cw_ready,

	output            frame_done,
	output     [31:0] frame_num,
	output            overflow,

	// ---- watched-address snapshot ----
	input             lw_we,          // list entries (hd3_loader)
	input       [7:0] lw_idx,
	input      [31:0] lw_addr,
	input       [8:0] list_n,

	input             cpu_rd,         // nes.v cpumem_read / cpumem_write (rising edge = access)
	input             cpu_wr,
	input      [15:0] cpu_a,          // nes.v hd_cpu_a
	input      [24:0] cpu_lin,        // nes.v cpumem_addr
	input       [7:0] cpu_d,          // nes.v cpumem_dout
	input             vram_rd,        // nes.v ppumem_read / ppumem_write
	input             vram_wr,
	input      [13:0] vram_a,         // nes.v hd_ppu_a
	input      [21:0] vram_lin,       // nes.v ppumem_addr
	input       [7:0] vram_d,         // nes.v ppumem_dout

	input             sd_allow,       // ch2 is free of savestates and battery saves (NES.sv)
	output            sd_own,
	output            sd_rd,
	output     [24:0] sd_addr,
	input             sd_busy,
	input       [7:0] sd_dout,

	input             v_clk,          // the HD engine clock (stage 3); the read ports run on it
	input             v_bank,         // values for hd3_resolve
	input       [7:0] v_idx,
	output      [7:0] v_q,
	input             v2_bank,        // values for hd3_render
	input       [7:0] v2_idx,
	output      [7:0] v2_q,
	output            watch_done,     // pulse: a snapshot is complete in bank ~frame_num[0]
	output            watch_overflow,
	output            sd_timeout
);

wire        chr_wr_p = ppu_wr;
wire        chr_wr_s = ss_wr && (ss_addr[24:20] == 5'b00010);
wire        chr_wr   = chr_wr_p | chr_wr_s;
wire [14:0] chr_addr = chr_wr_p ? ppu_addr[14:0] : ss_addr[14:0];
wire  [7:0] chr_data = chr_wr_p ? ppu_data : ss_data;

wire [10:0] fp_tile;
wire [63:0] fp_q;
wire        fp_idle;

hd_chrfp #(.AW(15)) u_chrfp
(
	.clk     (clk),
	.reset   (reset),
	.wr      (chr_wr),
	.wr_addr (chr_addr),
	.wr_data (chr_data),
	.fp_tile (fp_tile),
	.fp_q    (fp_q),
	.idle    (fp_idle)
);

hd_capture #(.FP_AW(15)) u_capture
(
	.clk         (clk),
	.reset       (reset),
	.enable      (enable),
	.chr_ram     (chr_ram),

	.bg_we       (tap[0]),
	.bg_slot     (tap[6:1]),
	.bg_lin      (tap[28:7]),
	.bg_pal      (tap[30:29]),
	.spr_we      (tap[31] | tap[352]),
	.spr_idx     (tap[352] ? {3'd1, tap[355:353]} : {3'd0, tap[34:32]}),
	.spr_data    (tap[352] ? tap[410:356] : tap[89:35]),
	.px_we       (tap[90]),
	.px_x        (tap[98:91]),
	.px_slot     (tap[104:99]),
	.px_fx       (tap[107:105]),
	.px_bgc      (tap[111:108]),
	.px_bg_on    (tap[112]),
	.px_spr_on   (tap[113]),
	.line_done   (tap[114]),
	.line_y      (tap[123:115]),
	.pal_all     (tap[315:124]),
	.px_branch_b (tap[320]),
	.px_t        (tap[335:321]),
	.px_finex    (tap[338:336]),
	.px_gray     (tap[339]),
	.px_emph     (tap[342:340]),
	.line_pre    (tap[343]),

	.fp_tile     (fp_tile),
	.fp_q        (fp_q),

	.wq_valid    (cw_valid),
	.wq_addr     (cw_addr),
	.wq_data     (cw_data),
	.wq_be       (cw_be),
	.wq_ready    (cw_ready),

	.frame_done  (frame_done),
	.frame_num   (frame_num),
	.overflow    (overflow)
);

wire        sr_req, sr_ack;
wire [24:0] sr_addr;
wire  [7:0] sr_q;

hd_watch u_watch
(
	.clk      (clk),
	.reset    (reset),
	.enable   (enable),

	.lw_we    (lw_we),
	.lw_idx   (lw_idx),
	.lw_addr  (lw_addr),
	.list_n   (list_n),

	.cpu_rd   (cpu_rd),
	.cpu_wr   (cpu_wr),
	.cpu_a    (cpu_a),
	.cpu_lin  (cpu_lin),
	.cpu_d    (cpu_d),
	.ppu_rd   (vram_rd),
	.ppu_wr   (vram_wr),
	.ppu_a    (vram_a),
	.ppu_lin  (vram_lin),
	.ppu_d    (vram_d),

	.ss_wr    (ss_wr),
	.ss_addr  (ss_addr),
	.ss_d     (ss_data),

	.pal_all  (tap[315:124]),
	.snap     (tap[344]),
	.bank     (~frame_num[0]),

	.sr_req   (sr_req),
	.sr_addr  (sr_addr),
	.sr_ack   (sr_ack),
	.sr_q     (sr_q),

	.v_clk    (v_clk),
	.v_bank   (v_bank),
	.v_idx    (v_idx),
	.v_q      (v_q),
	.v2_bank  (v2_bank),
	.v2_idx   (v2_idx),
	.v2_q     (v2_q),

	.done     (watch_done),
	.overflow (watch_overflow)
);

hd_sdram_port u_sdport
(
	.clk      (clk),
	.reset    (1'b0),
	.req      (sr_req),
	.addr     (sr_addr),
	.ack      (sr_ack),
	.q        (sr_q),
	.allow    (sd_allow),
	.own      (sd_own),
	.rd       (sd_rd),
	.ch2_addr (sd_addr),
	.ch2_busy (sd_busy),
	.ch2_dout (sd_dout),
	.timeout  (sd_timeout)
);

endmodule
