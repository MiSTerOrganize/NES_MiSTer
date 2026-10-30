// hd3_loader.sv - the format-3 .hdp loader of the display-list route (hd3_pengine).
//
// The ARM copies the pack into DDR3 itself (hdarm_daemon.c, the pack loader thread): file bytes
// below MAX_BYTES (192 MB) at the pack window (qword PACK_Q, byte 0x30000000), the rest at the
// overflow window (0x24000000), where hd3_audio already maps them. Copying from the SD card runs
// at the card's speed (~17 MB/s, a 300 MB pack in ~18 s); the OSD file transfer this replaces
// (hd3_loader_stream.sv, still used by hd3_engine) ran at ~0.6 MB/s. The OSD entry "Load HD Pack"
// is a mount (SC1): MiSTer stores the chosen path in config/NES_HD.s1 and transfers nothing.
//
// The ARM drives this module through one command qword it writes beside the mailbox (CMD_Q,
// byte 0x3C400060):
//   [63:56] 0xC5   [55:48] op: 1 = BEGIN, 2 = COMMIT   [47:40] sequence   [31:0] file size
// The loader reads it every 2^POLL_W clocks through the engine's single-access DDR3 crossing. A
// command is carried out once: its sequence must differ from the last one carried out. The first
// read after power-up only records the sequence it finds: DDR3 keeps an old command across a
// core reload, and that one must not run again.
//   BEGIN   pack_ok -> 0 at once, so the engine, HD audio and the ROM patcher stop reading the
//           pack and the ARM may overwrite it. Answered at once.
//   COMMIT  reads the header (qwords 0..33: every word used, then the section table) and the
//           watched-address list (section 9, copied out on lw_* for hd_watch) back from DDR3,
//           checks them, and sets pack_ok.
// Each answer updates pack_ok, reject and ack_seq together and toggles st_t; hd3_pengine then
// writes the pack-state qword (0x3C400008) with them, which is how the ARM learns the outcome.
// An unknown op is answered with reject 12 and pack_ok 0.
//
// The checks are the streaming loader's, with "bytes received" replaced by the size in the
// command (the size of the file the ARM copied). A command written before the baseline read is
// taken as stale and never answered, so the ARM resends (with a new sequence) when no answer
// comes. The pack is accepted only if all hold:
//   magic "NHDP" (1), format version 3 (2), scale 1-10 (3), no flag bits beyond the six defined
//   (4), header file size == the size in the command (5), file size <= MAX_TOTAL (6), bucket
//   count a power of two and not 0 (7), the section table inside the file (the file is at least
//   0x110 bytes), every section's offset + size inside the file, its offset qword aligned, and
//   every section but 15 (audio) ending inside the MAX_BYTES window (8), at most 256 watched
//   addresses that fit in section 9 (10), and a consistent addition
//   table (11): additions (header 0x40) present only with section 16, section 16 = 64 bytes per
//   bucket with a power-of-two bucket count (header 0x4C) and section 17 present with it; no
//   additions = no table, count and probe 0. (Reject 9 was the streaming handshake's.)

module hd3_loader #(
	parameter [27:3] PACK_Q    = 25'd0,             // qword of byte 0 in the DDR3 window
	parameter [31:0] MAX_BYTES = 32'h0C000000,      // 192 MB, the pack window
	parameter [31:0] MAX_TOTAL = 32'h18000000,      // 384 MB: the window + the overflow region
	parameter [27:3] CMD_Q     = 25'h188000C,       // byte 0xC400060: the ARM's load command
	parameter integer POLL_W   = 17                 // read the command every 2^POLL_W clocks
)
(
	input             clk,
	input             reset,           // cancels a load in progress (pack_ok -> 0)

	// one-qword reads through the engine's single-access crossing (pulse in, pulse out)
	output reg        m_req = 0,
	output reg [27:3] m_addr = 0,
	input      [63:0] m_dout,
	input             m_ready,

	output reg        pack_ok = 0,
	output reg  [3:0] reject = 0,      // why the last COMMIT was refused (0 = accepted)
	output reg  [7:0] ack_seq = 0,     // sequence of the last command carried out
	output reg        st_t = 0,        // toggles when pack_ok / reject / ack_seq were updated

	// header fields used by the engine
	output     [7:0]  scale,
	output     [7:0]  flags,
	output    [31:0]  options,
	output    [31:0]  bucket_mask,
	output    [15:0]  max_probe,
	output    [27:3]  keys_base,
	output    [27:3]  rules_base,
	output    [31:0]  hdr_word0x2c,    // condition count
	output    [31:0]  hdr_word0x30,    // frame-condition count
	output    [31:0]  hdr_word0x34,    // watched address count

	input             cart_no_chr_rom, // the loaded game has no CHR-ROM (NES.sv: chrrom == 0)
	output            game_ram,        // tile keys are CHR-RAM fingerprints (not CHR-ROM indices)
	output            legacy,          // pack version < 100: no backdrop / 0xFF palette byte
	output    [27:3]  fb_base,         // section 10, the fallback map (i32 per CHR tile)
	output    [31:0]  fb_len,          // its entry count, header 0x44 (0 = no fallbacks)
	output            add_en,          // the pack has additions (section 16 present)
	output    [27:3]  akeys_base,      // section 16, the addition-key table
	output    [31:0]  abucket_mask,    // its bucket count (header 0x4C) - 1
	output    [15:0]  amax_probe,      // its maximum probe (header 0x50)
	output    [27:3]  alists_base,     // section 17, the lists
	output     [7:0]  ov_top,          // the overscan window (header 0x10-0x1C, clamped 0..255)
	output     [7:0]  ov_right,
	output     [7:0]  ov_bottom,
	output     [7:0]  ov_left,
	output            has_pal,         // flags bit 1: the pack carries palette sections
	output            cache_en,        // options bit 3 clear: the BG tile cache may be used
	output            render_orig,     // options bit 4 clear: original pixels where no HD tile
	output    [27:3]  conds_base,      // section 2, conditions
	output    [27:3]  clist_base,      // section 3, condition lists
	output    [27:3]  blocks_base,     // section 4, image blocks
	output    [27:3]  bgl_base,        // section 5, background levels
	output    [27:3]  bgrec_base,      // section 6, background records
	output    [27:0]  bgmap_base,      // section 7, background maps (bytes)
	output    [27:0]  ratio_base,      // section 8, scroll ratios (bytes)
	output    [27:3]  pal_base,        // section 12, palettes
	output    [27:0]  bgi_base,        // section 13, value index (bytes)
	output    [31:0]  patch_len,       // section 14, ROM patches: its length (0 = none)
	output    [28:0]  aud_off,         // section 15, audio: FILE offset (hd3_audio maps it)
	output    [31:0]  aud_len,         // its length, 0 = the pack has no audio

	// watched-address list (section 9), copied out on COMMIT, for hd_watch
	output reg        lw_we = 0,
	output reg  [7:0] lw_idx = 0,
	output reg [31:0] lw_addr = 0
);

// the header words kept (byte offset in the file)
reg  [31:0] h_magic, h_w04, h_opt, w_size, w_nbuck, w_maxp, h_2c, h_30, w_wcnt;
reg  [31:0] h_s0off, h_s1off, w_s9off, w_s9len, h_44, h_s10off;
reg  [31:0] w_nadd, w_anb, w_amp, h_s16off, w_s16len, h_s17off, w_s17len;
reg  [31:0] h_ov [4];          // 0x10 top, 0x14 right, 0x18 bottom, 0x1C left
reg  [31:0] h_s2off, h_s3off, h_s4off, h_s5off, h_s6off, h_s7off, h_s8off, h_s12off, h_s13off;
reg  [31:0] h_s14len, h_s15off, h_s15len;
reg         sec_bad;           // a section-table entry failed its check

assign scale        = h_w04[23:16];    // 0x06
assign flags        = h_w04[31:24];    // 0x07
assign options      = h_opt;           // 0x0C
assign bucket_mask  = w_nbuck - 32'd1;
assign max_probe    = w_maxp[15:0];
assign keys_base    = PACK_Q + h_s0off[27:3];    // 0x80: section 0 offset
assign rules_base   = PACK_Q + h_s1off[27:3];    // 0x88: section 1 offset
assign hdr_word0x2c = h_2c;
assign hdr_word0x30 = h_30;
assign hdr_word0x34 = w_wcnt;
// Tile keys: Mesen uses !HasChrRom for tiles and fallbacks (spec; HdNesPack), so a cart with
// BOTH CHR-ROM and CHR-RAM keys by tile index. The converter records the game's type in flags
// bit 4 when it knew it (bit 5); otherwise the loaded cart decides.
assign game_ram     = flags[5] ? flags[4] : cart_no_chr_rom;
assign legacy       = flags[0];
assign fb_base      = PACK_Q + h_s10off[27:3];     // 0xD0: section 10 offset
assign fb_len       = h_44;
assign add_en       = w_s16len != 32'd0;
assign akeys_base   = PACK_Q + h_s16off[27:3];     // 0x100: section 16 offset
assign abucket_mask = w_anb - 32'd1;
assign amax_probe   = w_amp[15:0];
assign alists_base  = PACK_Q + h_s17off[27:3];     // 0x108: section 17 offset

function [7:0] clamp8(input [31:0] v);              // i32 -> 0..255
	clamp8 = v[31] ? 8'd0 : (v[30:8] != 0) ? 8'd255 : v[7:0];
endfunction
assign ov_top       = clamp8(h_ov[0]);
assign ov_right     = clamp8(h_ov[1]);
assign ov_bottom    = clamp8(h_ov[2]);
assign ov_left      = clamp8(h_ov[3]);
assign has_pal      = flags[1];
assign cache_en     = ~h_opt[3];
assign render_orig  = ~h_opt[4];
assign conds_base   = PACK_Q + h_s2off[27:3];      // 0x90
assign clist_base   = PACK_Q + h_s3off[27:3];      // 0x98
assign blocks_base  = PACK_Q + h_s4off[27:3];      // 0xA0
assign bgl_base     = PACK_Q + h_s5off[27:3];      // 0xA8
assign bgrec_base   = PACK_Q + h_s6off[27:3];      // 0xB0
assign bgmap_base   = {PACK_Q, 3'b000} + h_s7off[27:0];   // 0xB8
assign ratio_base   = {PACK_Q, 3'b000} + h_s8off[27:0];   // 0xC0
assign pal_base     = PACK_Q + h_s12off[27:3];     // 0xE0
assign bgi_base     = {PACK_Q, 3'b000} + h_s13off[27:0];  // 0xE8
assign patch_len    = h_s14len;                    // 0xF4: section 14's length (0 = no patches)
assign aud_off      = h_s15off[28:0];              // 0xF8: section 15, audio (a file offset)
assign aud_len      = h_s15len;                    // 0xFC: its length (0 = no audio)

// section k = header qword 16 + k (offset [31:0], size [63:32]): it must lie inside the file and
// be qword aligned (the engine addresses sections in qwords); every section but 15 (audio, read
// through hd3_audio's own file-offset mapping) must also end inside the pack window.
wire [32:0] sec_top = {1'b0, m_dout[31:0]} + {1'b0, m_dout[63:32]};
wire        sec_err = sec_top > {1'b0, w_size} || m_dout[2:0] != 3'd0 ||
                      (hq != 6'd31 && sec_top > {1'b0, MAX_BYTES});

// the addition table (reject 11)
wire        anb_pow2 = w_anb != 0 && (w_anb & (w_anb - 1)) == 0;
wire        atab_ok  = (w_s16len == 0) ? (w_nadd == 0 && w_anb == 0 && w_amp == 0 && w_s17len == 0)
                                       : (anb_pow2 && w_anb <= 32'h03FFFFFF &&
                                          w_s16len == {w_anb[25:0], 6'd0} && w_s17len != 0);
// section 9 = watched addresses, u32 each (reject 10)
wire        wlist_ok = w_wcnt <= 32'd256 && {w_wcnt, 2'b00} <= {2'b00, w_s9len};

reg  [31:0] cmd_size = 0;      // the file size in the COMMIT being carried out
wire  [3:0] verdict = (h_magic != 32'h5044484E)                    ? 4'd1 :   // "NHDP"
                      (h_w04[15:0] != 16'd3)                       ? 4'd2 :
                      (scale == 8'd0 || scale > 8'd10)             ? 4'd3 :
                      (flags[7:6] != 2'd0)                         ? 4'd4 :
                      (cmd_size != w_size)                         ? 4'd5 :
                      (w_size > MAX_TOTAL)                         ? 4'd6 :
                      (w_nbuck == 0 || (w_nbuck & (w_nbuck - 1)) != 0) ? 4'd7 :
                      (sec_bad || w_size < 32'h110)                ? 4'd8 :   // + table in the file
                      (!wlist_ok)                                  ? 4'd10 :
                      (!atab_ok)                                   ? 4'd11 : 4'd0;

localparam [2:0] S_IDLE = 3'd0, S_CMD = 3'd1, S_HDR = 3'd2, S_W9 = 3'd3, S_W9B = 3'd4,
                 S_JUDGE = 3'd5;
reg  [2:0] st = S_IDLE;
reg [POLL_W-1:0] tmr = {POLL_W{1'b1}};   // the first read (the baseline) on the first clock
reg        busy = 0;           // a read is outstanding (a reset waits for it in S_IDLE, which
                               // ignores answers, so a cancelled read's answer is dropped)
reg        based = 0;         // the first command read after power-up was taken
reg  [7:0] last_seq = 0;       // the last command carried out (or found at power-up)
reg  [7:0] cmd_seq = 0;
reg  [5:0] hq = 0;             // header qword being read (0..33)
reg  [8:0] wi = 0;             // watch-list entry being copied
reg  [8:0] wn = 0;             // entries to copy: min(watched count, 256)
reg [31:0] w_hold = 0;         // the second entry of the qword just read

wire       take = m_ready;
wire [8:0] wn_new = (w_wcnt > 32'd256) ? 9'd256 : w_wcnt[8:0];

always @(posedge clk) begin
	m_req <= 0;
	lw_we <= 0;
	if (m_ready) busy <= 0;
	tmr <= tmr + 1'd1;

	case (st)
	S_IDLE: if (&tmr && !busy) begin
		m_req <= 1; m_addr <= CMD_Q; busy <= 1; st <= S_CMD;
	end

	S_CMD: if (take) begin
		st <= S_IDLE;
		if (!based) begin
			based    <= 1;
			last_seq <= m_dout[47:40];
		end
		else if (m_dout[63:56] == 8'hC5 && m_dout[47:40] != last_seq) begin
			last_seq <= m_dout[47:40];
			cmd_seq  <= m_dout[47:40];
			pack_ok  <= 0;
			if (m_dout[55:48] == 8'd2) begin                       // COMMIT
				cmd_size <= m_dout[31:0];
				sec_bad  <= 0;
				hq       <= 0;
				m_req    <= 1; m_addr <= PACK_Q; busy <= 1;
				st       <= S_HDR;
			end
			else begin                                              // BEGIN (or unknown)
				reject  <= (m_dout[55:48] == 8'd1) ? 4'd0 : 4'd12;
				ack_seq <= m_dout[47:40];
				st_t    <= ~st_t;
			end
		end
	end

	S_HDR: if (take) begin
		case (hq)
			6'd0:  begin h_magic  <= m_dout[31:0]; h_w04    <= m_dout[63:32]; end
			6'd1:  begin                           h_opt    <= m_dout[63:32]; end
			6'd2:  begin h_ov[0]  <= m_dout[31:0]; h_ov[1]  <= m_dout[63:32]; end
			6'd3:  begin h_ov[2]  <= m_dout[31:0]; h_ov[3]  <= m_dout[63:32]; end
			6'd4:  begin w_size   <= m_dout[31:0]; w_nbuck  <= m_dout[63:32]; end
			6'd5:  begin w_maxp   <= m_dout[31:0]; h_2c     <= m_dout[63:32]; end
			6'd6:  begin h_30     <= m_dout[31:0]; w_wcnt   <= m_dout[63:32]; end
			6'd8:  begin w_nadd   <= m_dout[31:0]; h_44     <= m_dout[63:32]; end
			6'd9:  begin                           w_anb    <= m_dout[63:32]; end
			6'd10: begin w_amp    <= m_dout[31:0];                            end
			6'd16: h_s0off  <= m_dout[31:0];
			6'd17: h_s1off  <= m_dout[31:0];
			6'd18: h_s2off  <= m_dout[31:0];
			6'd19: h_s3off  <= m_dout[31:0];
			6'd20: h_s4off  <= m_dout[31:0];
			6'd21: h_s5off  <= m_dout[31:0];
			6'd22: h_s6off  <= m_dout[31:0];
			6'd23: h_s7off  <= m_dout[31:0];
			6'd24: h_s8off  <= m_dout[31:0];
			6'd25: begin w_s9off  <= m_dout[31:0]; w_s9len  <= m_dout[63:32]; end
			6'd26: h_s10off <= m_dout[31:0];
			6'd28: h_s12off <= m_dout[31:0];
			6'd29: h_s13off <= m_dout[31:0];
			6'd30: h_s14len <= m_dout[63:32];
			6'd31: begin h_s15off <= m_dout[31:0]; h_s15len <= m_dout[63:32]; end
			6'd32: begin h_s16off <= m_dout[31:0]; w_s16len <= m_dout[63:32]; end
			6'd33: begin h_s17off <= m_dout[31:0]; w_s17len <= m_dout[63:32]; end
			default: ;
		endcase
		if (hq >= 6'd16 && sec_err) sec_bad <= 1;   // w_size (qword 4) is in by now
		if (hq != 6'd33) begin
			hq     <= hq + 6'd1;
			m_req  <= 1; m_addr <= m_addr + 25'd1; busy <= 1;
		end
		else begin
			wi <= 0;
			wn <= wn_new;
			if (wn_new == 0) st <= S_JUDGE;
			else begin
				m_req <= 1; m_addr <= PACK_Q + w_s9off[27:3]; busy <= 1;
				st    <= S_W9;
			end
		end
	end

	S_W9: if (take) begin                          // entry wi = the qword's low word
		lw_we   <= 1; lw_idx <= wi[7:0]; lw_addr <= m_dout[31:0];
		w_hold  <= m_dout[63:32];
		wi      <= wi + 9'd1;
		st      <= (wi + 9'd1 < wn) ? S_W9B : S_JUDGE;
	end

	S_W9B: begin                                   // entry wi = its high word
		lw_we   <= 1; lw_idx <= wi[7:0]; lw_addr <= w_hold;
		wi      <= wi + 9'd1;
		if (wi + 9'd1 < wn) begin
			m_req <= 1; m_addr <= m_addr + 25'd1; busy <= 1;
			st    <= S_W9;
		end
		else st <= S_JUDGE;
	end

	S_JUDGE: begin
		pack_ok <= verdict == 4'd0;
		reject  <= verdict;
		ack_seq <= cmd_seq;
		st_t    <= ~st_t;
		st      <= S_IDLE;
	end

	default: st <= S_IDLE;
	endcase

	if (reset) begin
		pack_ok <= 0;
		st      <= S_IDLE;
	end
end

endmodule
