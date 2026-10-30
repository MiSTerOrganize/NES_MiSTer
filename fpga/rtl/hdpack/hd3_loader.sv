// hd3_loader.sv - streams a format-3 .hdp from the OSD file load into DDR3 (stage 2).
//
// The file is stored verbatim from qword PACK_Q (byte 0 of the file at the window offset
// PACK_Q*8), so every section offset in the header is also a DDR3 offset from that base.
// Bytes from MAX_BYTES (192 MB) on go to the overflow window instead (wr_hi, from qword OVF_Q
// of the 0x20000000 region, i.e. 0x24000000): only section 15 (audio, laid out last by the
// converter) may reach there, and hd3_audio maps its file offsets the same way.
// The loader counts bytes itself: hps_io's ioctl_addr is 27 bits (128 MB) and the largest
// corpus pack is 157 MB, so the address from the framework cannot be trusted to reach the end.
//
// The header words the engine needs are kept in registers as the header streams past (the
// words used, not the whole 512-byte header: the full copy cost about 2,000 registers and a
// combinational bounds check over every entry about 1,000 LUTs - yosys estimate, 2026-10-15).
// Each section-table entry (18 of them, bytes 0x80-0x10F) is checked the moment its 8 bytes
// arrive; the file-size word (0x20) is complete long before the table. When the load ends the
// pack is accepted only if all of these hold, otherwise pack_ok stays 0 and the engine stays
// off (the NES picture is unaffected):
//   magic "NHDP", format version 3, scale 1-10, no flag bits beyond the six defined ones,
//   header file size == bytes received, file size <= MAX_TOTAL, bucket count a power of
//   two (and not 0), the whole section table received, every section's offset + size inside
//   the file and its offset qword aligned, every section but 15 (audio) ending inside the
//   MAX_BYTES pack window, at most 256 watched addresses that fit in section 9
//   (reject 10), and a consistent addition table (reject 11): additions (header 0x40) present
//   only with section 16, section 16 = 64 bytes per bucket with a power-of-two bucket count
//   (header 0x4C) and section 17 present with it; no additions = no table, count and probe 0.
//   A pack with additions and no table is a stale conversion: drawing it without its additions
//   would be silently wrong, so it is refused.
//
// Section 9 (the watched-address list) is also copied out as it streams past (lw_*), so
// hd_watch has the list without reading DDR3. It starts after the header, so the header
// fields that locate it are complete by then.
//
// DDR3 writes are single qwords; load_wait holds the file transfer while one is pending. A
// byte that arrives anyway (a broken handshake) would overwrite the pending qword, so it is
// recorded and the load is refused (reject 9) rather than accepted with corrupt data.

module hd3_loader #(
	parameter [27:3] PACK_Q    = 25'd0,             // qword of byte 0 in the DDR3 window
	parameter [31:0] MAX_BYTES = 32'h0C000000,      // 192 MB, the pack window
	parameter [31:0] MAX_TOTAL = 32'h18000000,      // 384 MB: the window + the overflow region
	parameter [27:3] OVF_Q     = 25'h0800000        // qword of file byte MAX_BYTES (wr_hi = 1)
)
(
	input             clk,
	input             reset,

	input             load_active,     // the .hdp file transfer is in progress
	input             load_wr,         // one byte on load_data
	input       [7:0] load_data,
	output            load_wait,

	output reg        wr_req = 0,
	output reg        wr_hi = 0,       // wr_addr is in the overflow window (file bytes >= 192 MB)
	output reg [27:3] wr_addr = 0,
	output reg [63:0] wr_data = 0,
	input             wr_ack,

	output reg        pack_ok = 0,
	output reg  [3:0] reject = 0,      // why the last load was refused (0 = accepted), for debug

	// header fields used by the engine
	output     [7:0]  scale,
	output     [7:0]  flags,
	output    [31:0]  options,
	output    [31:0]  bucket_mask,
	output    [15:0]  max_probe,
	output    [27:3]  keys_base,
	output    [27:3]  rules_base,
	output    [31:0]  hdr_word0x2c,    // condition count (kept for the stage-3 engine)
	output    [31:0]  hdr_word0x30,    // frame-condition count
	output    [31:0]  hdr_word0x34,    // watched address count

	// what hd3_dict / hd3_resolve take besides the key and rule tables
	input             cart_no_chr_rom, // the loaded game has no CHR-ROM (NES.sv: chrrom == 0)
	output            game_ram,        // tile keys are CHR-RAM fingerprints (not CHR-ROM indices)
	output            legacy,          // pack version < 100: no backdrop / 0xFF palette byte
	output    [27:3]  fb_base,         // section 10, the fallback map (i32 per CHR tile)
	output    [31:0]  fb_len,          // its entry count, header 0x44 (0 = no fallbacks)
	// the addition tables (8a), for hd3_dict / hd3_add
	output            add_en,          // the pack has additions (section 16 present)
	output    [27:3]  akeys_base,      // section 16, the addition-key table
	output    [31:0]  abucket_mask,    // its bucket count (header 0x4C) - 1
	output    [15:0]  amax_probe,      // its maximum probe (header 0x50)
	output    [27:3]  alists_base,     // section 17, the lists
	// what hd3_resolve / hd3_render take (stage 3): the overscan window (header 0x10-0x1C,
	// i32 each, clamped to 0..255 as hdref does), header flag/option bits, and the remaining
	// section bases - in qwords, or in bytes where the reader addresses inside a section by
	// byte (value index, background maps, scroll ratios)
	output     [7:0]  ov_top,
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
	output    [31:0]  patch_len,       // section 14, ROM patches: its length in bytes, 0 = the
	                                   // pack has none (hd3_romfix; the ARM reads the section)
	output    [28:0]  aud_off,         // section 15, audio: FILE offset (hd3_audio maps it)
	output    [31:0]  aud_len,         // its length, 0 = the pack has no audio

	// watched-address list (section 9), captured as it streams past, for hd_watch
	output reg        lw_we = 0,
	output reg  [7:0] lw_idx = 0,
	output reg [31:0] lw_addr = 0
);

reg  [28:0] cnt = 0;         // bytes received (up to MAX_TOTAL: 29 bits)
reg  [63:0] qasm;
reg         old_active = 0;
reg         ovf;             // more bytes than MAX_BYTES (the counter must not wrap into a match)
reg         herr;            // a byte arrived while a qword write was still pending
reg         sec_bad;         // a section-table entry failed its check as it arrived

// the header words kept (byte offset in the file)
reg  [31:0] h_magic, h_w04, h_opt, w_size, w_nbuck, w_maxp, h_2c, h_30, w_wcnt;
reg  [31:0] h_s0off, h_s1off, w_s9off, w_s9len, h_44, h_s10off;
reg  [31:0] w_nadd, w_anb, w_amp, h_s16off, w_s16len, h_s17off, w_s17len;
reg  [31:0] h_ov [4];          // 0x10 top, 0x14 right, 0x18 bottom, 0x1C left
reg  [31:0] h_s2off, h_s3off, h_s4off, h_s5off, h_s6off, h_s7off, h_s8off, h_s12off, h_s13off;
reg  [31:0] h_s14len, h_s15off, h_s15len;

assign load_wait = wr_req;

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
// BOTH CHR-ROM and CHR-RAM (an NES 2.0 header with both sizes, e.g. Metroid + mmm.ips) keys by
// tile index. The converter records the game's type in flags bit 4 when it knew it (bit 5,
// from the ROM or because every key is one kind); 39 corpus packs do not, and for those the
// loaded cart decides. NOT mapper_flags[15] (has_chr_ram): that is set for such a cart too.
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

// the header word / section-table entry that the current byte completes (little-endian)
wire [31:0] b_word  = {load_data, qasm[63:40]};
wire [63:0] b_entry = {load_data, qasm[63:8]};
wire        in_hdr  = cnt[28:9] == 0;                 // the 512-byte header
// section k (bytes 0x80+8k..+7, k = 0..17 = qwords 16..33): offset entry[31:0], size
// entry[63:32]; it must lie inside the file and be qword aligned (the engine addresses
// sections in qwords). Every section but 15 (audio, which hd3_audio reads through its own
// file-offset mapping) must also end inside the pack window: the engine's readers address
// the window only.
wire        sec_end = in_hdr & (cnt[8:3] >= 6'd16) & (cnt[8:3] < 6'd34) & (cnt[2:0] == 3'd7);
wire [32:0] sec_top = {1'b0, b_entry[31:0]} + {1'b0, b_entry[63:32]};
wire        sec_err = sec_top > {1'b0, w_size} || b_entry[2:0] != 3'd0 ||
                      (cnt[8:3] != 6'd31 && sec_top > {1'b0, MAX_BYTES});
wire        sections_ok = ~sec_bad & (cnt > 29'h10F);  // and the whole table arrived
// file qword -> DDR3: below MAX_BYTES in the pack window, above it in the overflow window
wire [25:0] fq      = cnt[28:3];
wire        f_hi    = fq >= MAX_BYTES[28:3];
wire [27:3] f_addr  = f_hi ? OVF_Q + 25'(fq - MAX_BYTES[28:3]) : PACK_Q + fq[24:0];

// the addition table (reject 11)
wire        anb_pow2 = w_anb != 0 && (w_anb & (w_anb - 1)) == 0;
wire        atab_ok  = (w_s16len == 0) ? (w_nadd == 0 && w_anb == 0 && w_amp == 0 && w_s17len == 0)
                                       : (anb_pow2 && w_anb <= 32'h03FFFFFF &&
                                          w_s16len == {w_anb[25:0], 6'd0} && w_s17len != 0);

// section 9 = watched addresses, u32 each; the header (bytes 0-255) is complete before it
wire [28:0] w_rel   = cnt - w_s9off[28:0];
wire        w_in9   = cnt[28:8] != 0 && {3'd0, cnt} >= w_s9off && w_rel < 29'd1024 &&
                      {5'd0, w_rel[28:2]} < w_wcnt;
wire        wlist_ok = w_wcnt <= 32'd256 && {w_wcnt, 2'b00} <= {2'b00, w_s9len};

always @(posedge clk) begin
	old_active <= load_active;
	if (wr_ack) wr_req <= 0;
	lw_we <= 0;
	if (load_active & load_wr & w_in9 & (w_rel[1:0] == 2'd3)) begin
		lw_we   <= 1;
		lw_idx  <= w_rel[9:2];
		lw_addr <= {load_data, qasm[63:40]};
	end

	if (load_active & ~old_active) begin
		pack_ok <= 0;
		cnt     <= 0;
		ovf     <= 0;
		herr    <= 0;
		sec_bad <= 0;
		h_magic <= 0;          // a file too short to carry a header is then refused (reject 1)
	end

	if (load_active & load_wr & wr_req & ~wr_ack) herr <= 1;

	if (load_active & load_wr) begin
		qasm <= {load_data, qasm[63:8]};
		if (in_hdr & (cnt[1:0] == 2'd3)) case (cnt[8:2])
			7'd0:  h_magic <= b_word;
			7'd1:  h_w04   <= b_word;
			7'd3:  h_opt   <= b_word;
			7'd4:  h_ov[0] <= b_word;
			7'd5:  h_ov[1] <= b_word;
			7'd6:  h_ov[2] <= b_word;
			7'd7:  h_ov[3] <= b_word;
			7'd8:  w_size  <= b_word;
			7'd9:  w_nbuck <= b_word;
			7'd10: w_maxp  <= b_word;
			7'd11: h_2c    <= b_word;
			7'd12: h_30    <= b_word;
			7'd13: w_wcnt  <= b_word;
			7'd16: w_nadd  <= b_word;
			7'd17: h_44    <= b_word;
			7'd19: w_anb   <= b_word;
			7'd20: w_amp   <= b_word;
			7'd32: h_s0off <= b_word;
			7'd34: h_s1off <= b_word;
			7'd36: h_s2off <= b_word;
			7'd38: h_s3off <= b_word;
			7'd40: h_s4off <= b_word;
			7'd42: h_s5off <= b_word;
			7'd44: h_s6off <= b_word;
			7'd46: h_s7off <= b_word;
			7'd48: h_s8off <= b_word;
			7'd56: h_s12off <= b_word;
			7'd58: h_s13off <= b_word;
			7'd61: h_s14len <= b_word;
			7'd62: h_s15off <= b_word;
			7'd63: h_s15len <= b_word;
			7'd50: w_s9off <= b_word;
			7'd51: w_s9len <= b_word;
			7'd52: h_s10off <= b_word;
			7'd64: h_s16off <= b_word;
			7'd65: w_s16len <= b_word;
			7'd66: h_s17off <= b_word;
			7'd67: w_s17len <= b_word;
			default: ;
		endcase
		if (sec_end & sec_err) sec_bad <= 1;
		if (cnt[2:0] == 3'd7) begin
			wr_req  <= 1;
			wr_hi   <= f_hi;
			wr_addr <= f_addr;
			wr_data <= {load_data, qasm[63:8]};
		end
		if (cnt == 29'h1FFFFFFF) ovf <= 1;
		else cnt <= cnt + 29'd1;
	end

	// end of the transfer: flush a final partial qword (zero-padded), then judge the header
	if (~load_active & old_active) begin
		if (cnt[2:0] != 3'd0) begin
			wr_req  <= 1;
			wr_hi   <= f_hi;
			wr_addr <= f_addr;
			wr_data <= qasm >> {~cnt[2:0] + 3'd1, 3'd0};
		end
		reject <= (h_magic != 32'h5044484E)                    ? 4'd1 :   // "NHDP"
		          (h_w04[15:0] != 16'd3)                       ? 4'd2 :
		          (scale == 8'd0 || scale > 8'd10)             ? 4'd3 :
		          (flags[7:6] != 2'd0)                         ? 4'd4 :
		          (ovf || {3'd0, cnt} != w_size)               ? 4'd5 :
		          (w_size > MAX_TOTAL)                         ? 4'd6 :
		          (w_nbuck == 0 || (w_nbuck & (w_nbuck - 1)) != 0) ? 4'd7 :
		          (!sections_ok)                               ? 4'd8 :
		          (herr)                                       ? 4'd9 :
		          (!wlist_ok)                                  ? 4'd10 :
		          (!atab_ok)                                   ? 4'd11 : 4'd0;
		pack_ok <= (h_magic == 32'h5044484E) && h_w04[15:0] == 16'd3 && scale != 0 && scale <= 8'd10 &&
		           flags[7:6] == 2'd0 && !ovf && {3'd0, cnt} == w_size && w_size <= MAX_TOTAL &&
		           w_nbuck != 0 && (w_nbuck & (w_nbuck - 1)) == 0 && sections_ok && !herr && wlist_ok &&
		           atab_ok;
	end

	if (reset) begin
		wr_req  <= 0;
		pack_ok <= 0;
	end
end

endmodule
