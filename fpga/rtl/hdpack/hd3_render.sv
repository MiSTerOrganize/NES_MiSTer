// hd3_render.sv - stage 3 RENDER (HDPACK_DESIGN.md "hd3_render").
//
// Draws frame N line by line while frame N+1 is captured, from:
//   hd3_dict    record ids per BG span and sprite fetch, record keys and resolutions
//   hd3_resolve each record's filtered rule list (rule store), or "walk it from DDR3"
//   hd3_fmap    the frame's pixel map and sprite table (conditions may name any pixel)
//   DDR3        the frame record (line headers, sprites, pixel words), condition records and
//               lists, rule lists (slow path), image blocks, the pack palette
// and outputs every pixel of the pack's overscan window as its 2x2 output block, after
// grayscale and emphasis (out_v / out_y / out_x / out_px), in raster order.
//
// Per line (lines and columns outside the window are not looked up, as Mesen):
//   LOAD   30 bursts: line block qwords 0..111 (header, palette RAM, spans, sprite fetches)
//          and the 128 pixel qwords; meanwhile each pixel's sprite list (the first four
//          valid fetches covering it, in fetch order, if sprites are shown there) is built
//          from hd3_fmap.
//   SCAN   BG instances: runs of up to 8 window pixels sharing a BG span (one record).
//   BG     each instance walked once with 8-bit pixel masks (hwmodel.walk). A rewrite-
//          sensitive condition met while walking a group restarts the instance one pixel at
//          a time, so the 8-pixel BG cache rule (which pixels are LOOKED UP, hence possibly
//          rewritten) is applied to a pixel before a later pixel of the group reads it. The
//          cache rule writes the final BG result and the frame-wide "looked up" bitmap.
//   SPR    per sprite fetch, runs of its window pixels that hold it in their list.
//   FETCH  the image rows of every result (one 2-row slot per (block, tile row)) stream in
//          on port L while the walks run.
//   BGPREP (8j, alongside the walks) per selected background, in level order: its scroll
//          ratios, the line's visibility test, the map words of the cells across the window
//          (port B), an image slot per non-transparent (block, tile row), and per 8-pixel
//          window group its pixel mask and its two cells (HDPACK_DESIGN.md "8i/8j").
//   COMP   per pixel, as hdref.render: backdrop, background layer 0, sprites behind, layer 1,
//          BG tile (or the original colour), layer 2, sprites in front, layer 3; 2x2 blend
//          (backgrounds Alpha / Add / Subtract, as hdref.blend_sub).
// Conditions (hdref.eval_cond): all 14 types. Tile/sprite conditions on other pixels read
// hd3_fmap + hd3_dict; Mesen's fallback rewrite is reproduced from lookup ORDER for the
// conditions the converter flags rewrite-sensitive (q0 bit 24 - only those can change);
// deferred (sensitive) frame conditions, and every frame condition on the slow path, are
// memoised per frame at first use.
//
// Dictionary overflow (HDPACK_DESIGN.md "8o"): a frame hd3_dict flagged `frame_ovf` is drawn
// with no HD lookups - every pixel keeps its original colour and no condition is evaluated.
//
// A tile condition given as an index in a CHR-RAM game (8k, no corpus pack has one) compares
// the target's TileIndex, read uncached from the frame record (T6/T11).
//
// Backgrounds are selected by hd3_resolve (8i: one record per level, before additions, and the
// memo seed of the rewrite-sensitive conditions that selection evaluated); this module draws
// them (8j). A dictionary-overflowed frame still draws its backgrounds.
//
// Additions (8a, hdref.process_additions): hd3_add has built the frame's LANES. A pixel's sprite
// list is its captured sprites, then the lanes of its line covering it (slot order), four in
// all; a list entry is {kind 2, index 6}: kind 0 a sprite fetch (16..31: line 239's fetches),
// 1 a lane of this line, 2 a lane of line 240. An added sprite is a copy of its source record
// with no colour of its own (sci 0) and the added key: it is walked like a sprite (its own
// run of pixels), drawn only when a rule matches, and seen by spriteAtPosition/Nearby. When the
// pack has additions, Mesen leaves line 239's first-pixel record over pixel (0,0), so on line 0:
// the line header fields (t, fine x, grayscale, emphasis) are line 239's, and pixel (0,0) is
// line 239's pixel 0 - its pixel word, its BG span (virtual slot 62), its sprite fetches
// (finfo 16..31, palette RAM l_pal239) - with the lanes of line 240. A condition naming pixel
// (0,0) reads that pixel too (its looked-up state stays pixel (0,0)'s).

module hd3_render #(
	parameter        RAW      = 11,
	parameter        RSW      = 10,
	parameter [27:0] REC_BASE = 28'hC000000,
	// image slots a line may allocate before tiles go uncached through the scratch slot (127).
	// Only a test build lowers it (tools/sim3/run_render.sh): no fixture line fills 127 slots.
	parameter integer NSLOT    = 127,
	// output FIFO depth = 2**OF_LOG2 finished pixels (8). Measured, it never holds more than one
	// (the output stage pops every 2 clocks, compose issues at most two back to back), so the
	// full-FIFO stall never fires at 8; a test build (OF_LOG2=1) makes it fire to check that
	// stalling loses nothing.
	parameter integer OF_LOG2  = 3,
	// simulation only: 1 prints a per-frame cycle profile, 2 also every line, 3 also clocks and
	// entries per state (over the frame, or only line PLINE when PLINE >= 0)
	parameter integer PROFILE  = 0,
	parameter integer PLINE    = -1,
	parameter integer NL       = 16     // hd3_add lanes per line (its NL)
)
(
	input             clk,
	input             reset,

	// pack configuration (stable while a pack is loaded)
	input             game_ram,
	input             cache_en,         // BG 8-pixel cache (options & 8 == 0)
	input             render_orig,      // draw the original BG colour (options & 16 == 0)
	input             has_pal,          // pack palette (section 12), else the 2C02 default
	input       [7:0] ov_top,           // overscan, already clamped to >= 0
	input       [7:0] ov_bottom,
	input       [7:0] ov_left,
	input       [7:0] ov_right,
	input      [27:3] rules_base,
	input      [27:3] conds_base,
	input      [27:3] clist_base,
	input      [27:3] blocks_base,
	input      [27:3] pal_base,
	input      [27:3] bgrec_base,       // backgrounds: section 6 records (32 bytes each)
	input      [27:0] bgmap_base,       //   section 7 cell maps (byte offset)
	input      [27:0] ratio_base,       //   section 8 ratio tables, 512 x int32 (byte offset)
	input             add_en,           // the pack has additions (8a)
	input             flush,            // pack (re)loaded: drop the condition caches

	input             start,
	input             bank,
	// hd3_dict's frame_ovf for `bank`: the frame's records are incomplete (more distinct records
	// than ids, or a capture timing fault). Such a frame is drawn with NO HD lookups - every BG
	// and sprite pixel as if its key had no rule (the original colours) - so no condition is
	// evaluated against records that were never given an id. Sampled at `start`.
	input             frame_ovf,
	output reg        busy = 0,
	output reg        done = 0,
	output reg        unsup = 0,        // sticky: a condition this design does not evaluate

	// hd3_dict read ports (one clock latency)
	output            d_bank,
	output reg [RAW-1:0] d_rid,
	input      [95:0] d_key,
	input      [63:0] d_res,
	output reg  [7:0] d_y,
	output reg  [5:0] d_slot,
	input     [RAW:0] d_span,
	output reg  [3:0] d_fetch,
	input     [RAW:0] d_spr,
	input      [31:0] d_frame,

	// hd3_resolve read ports (one clock latency)
	output reg [RAW-1:0] q_rid,
	input [2+1+11+RSW+25-1:0] q_rec,
	output reg [RSW-1:0] q_ri,
	input      [72:0] q_rule,
	// hd3_resolve background ports (one clock latency): the frame's selection per level
	// {valid, record index 20}, and the memo seed {valid, value before negation} that the
	// frame-start memo sweep loads
	output            gs_bank,
	output reg  [5:0] gs_lvl,
	input      [20:0] gs_q,
	output            sd_bank,
	output     [10:0] sd_a,
	input       [1:0] sd_q,

	// hd3_fmap read ports (one clock latency)
	output     [14:0] p_a,
	input      [13:0] p_q,
	output      [7:0] l_y,
	input      [10:0] l_q,
	output      [7:0] s_y,
	input     [415:0] s_q,

	// hd_watch value port (one clock latency)
	output reg  [7:0] v_idx,
	input       [7:0] v_q,

	// hd3_add's lanes of line a_y, 0..240 (one clock latency):
	// {from line 239, bgp, vm, hm, row 3, column of x0 3, added id, length-1 3, x0 8}
	output      [7:0] a_y,
	input       [6:0] a_n,
	input [NL*(RAW+21)-1:0] a_q,

	// DDR3 read ports: L = frame record, image rows, palette; C = condition lists and rule
	// lists (slow path); E = condition records; B = background records, ratios and cell maps
	output reg        l_rd_req = 0,
	output reg [27:3] l_rd_addr,
	input             l_rd_ack,
	input      [63:0] l_rd_data,
	input             l_rd_valid,
	output            c_rd_req,
	output     [27:3] c_rd_addr,
	input             c_rd_ack,
	input      [63:0] c_rd_data,
	input             c_rd_valid,
	output            e_rd_req,
	output     [27:3] e_rd_addr,
	input             e_rd_ack,
	input      [63:0] e_rd_data,
	input             e_rd_valid,
	output reg        b_rd_req = 0,
	output reg [27:3] b_rd_addr,
	input             b_rd_ack,
	input      [63:0] b_rd_data,
	input             b_rd_valid,

	// one window pixel: its 2x2 block {row1 col1, row1 col0, row0 col1, row0 col0}, RRGGBB
	output reg        out_v = 0,
	output reg  [7:0] out_y,
	output reg  [7:0] out_x,
	output reg [95:0] out_px,

	// display pacing (stage-3 integration): a line is started only while line_ok, which the
	// engine derives from line_y and the scan-out position so that the output ring is never
	// overwritten before it is shown; the render testbench ties it high
	input             line_ok,
	output      [7:0] line_y
);

// ==========================================================================================
// window
// ==========================================================================================
wire  [8:0] ybot9 = 9'd239 - {1'b0, ov_bottom};
wire  [8:0] xr9   = 9'd255 - {1'b0, ov_right};
wire        win_ok = !ybot9[8] && !xr9[8] && ({1'b0, ov_top} <= ybot9) &&
                     ({1'b0, ov_left} <= xr9);
wire  [7:0] ybot = ybot9[7:0];
wire  [7:0] xr   = xr9[7:0];
function in_win(input [7:0] yy, input [7:0] xx);
	in_win = yy >= ov_top && yy <= ybot && xx >= ov_left && xx <= xr;
endfunction

// ==========================================================================================
// frame / line state shared by the blocks
// ==========================================================================================
reg         fbank = 0;
reg         f_ovf = 0;               // this frame is drawn without lookups (see frame_ovf)
reg  [11:0] mc;                      // memo clear sweep index (S_MCLR)
reg  [31:0] frame;
reg   [7:0] y;
assign line_y = y;
assign d_bank = fbank;
assign gs_bank = fbank;
assign sd_bank = fbank;
assign sd_a = mc[10:0];

// line header (written by the port-L block)
reg  [14:0] l_t;                     // PPU t at the line's first pixel (background scroll)
reg   [2:0] l_finex;
reg         l_gray;
reg   [2:0] l_emph;
reg   [5:0] l_nspan;
reg   [4:0] l_nfet;
reg [191:0] l_pal;
function [5:0] pal6(input [191:0] p, input [4:0] n);
	pal6 = p[6 * n +: 6];
endfunction

// 8a: the line's lanes and, on line 0 when the pack has additions (vz), line 239's first pixel
localparam integer LW = RAW + 21;
wire        vz = add_en && y == 8'd0;
reg [NL*LW-1:0] ln_cur;              // lanes of line y, slot 0 at the bottom
reg [NL*LW-1:0] ln_240;              // lanes of line 240 (pixel (0,0)), loaded on every line
reg   [6:0] n_cur, n_240;
reg  [31:0] px239;                   // line 239's pixel 0 word
reg   [4:0] v_nfet;                  // line 239's sprite fetch count
reg   [5:0] v_nspan;                 // ... and BG span count
reg [191:0] l_pal239;                // ... and palette RAM
wire        v_slok = px239[5:0] != 6'd63 && px239[5:0] < v_nspan;
// pixel (0,0) on line 0: line 239's pixel 0, its BG span seen as virtual slot 62
wire [31:0] w00 = {px239[31:6], v_slok ? 6'd62 : 6'd63};
// a pixel word of the current line from the line pixel RAM (q = its qword)
function [31:0] pword(input [63:0] q, input [7:0] xx);
	pword = (vz && xx == 8'd0) ? w00 : (xx[0] ? q[63:32] : q[31:0]);
endfunction

// ==========================================================================================
// memories (one write port, registered read)
// ==========================================================================================
// pack palette: 32 x {entry 2i+1, entry 2i}, RRGGBB; default 2C02 palette ROM
reg  [47:0] palm [32];
reg  [23:0] pal_rom [64];
// $readmemh paths: Quartus resolves a relative name against the PROJECT folder (cf. video.sv's
// "rtl/tao.mif"); the Verilator testbenches run with the working folder set to rtl/hdpack.
// A bare name would make Quartus warn and synthesise these ROMs as all zeros.
// HD3_HEX_PROJECT: a Verilator build run from the project folder, as Quartus resolves the
// names (tools/fullsim) - it takes the project paths too.
`ifdef VERILATOR
`ifndef HD3_HEX_PROJECT
`define HD3_HEX_BARE
`endif
`endif
`ifdef HD3_HEX_BARE
`define HD3_HEX_PAL "hd3_pal2c02.hex"
`define HD3_HEX_ER  "hd3_emph_r.hex"
`define HD3_HEX_EG  "hd3_emph_g.hex"
`define HD3_HEX_EB  "hd3_emph_b.hex"
`else
`define HD3_HEX_PAL "rtl/hdpack/hd3_pal2c02.hex"
`define HD3_HEX_ER  "rtl/hdpack/hd3_emph_r.hex"
`define HD3_HEX_EG  "rtl/hdpack/hd3_emph_g.hex"
`define HD3_HEX_EB  "rtl/hdpack/hd3_emph_b.hex"
`endif
initial $readmemh(`HD3_HEX_PAL, pal_rom);
reg   [5:0] pc_a;                    // colour index being read
reg   [4:0] pmw_a;
reg  [47:0] pm_q, pmw_d;
reg  [23:0] prom_q;
reg         pmw_e = 0;
always @(posedge clk) begin
	if (pmw_e) palm[pmw_a] <= pmw_d;
	pm_q   <= palm[pc_a[5:1]];
	prom_q <= pal_rom[pc_a];
end
reg         dp_lo, dp2_lo;           // palette entry parity of the colour in the draw pipe
wire [23:0] pal_col = has_pal ? (dp2_lo ? pm_q[47:24] : pm_q[23:0]) : prom_q;

// line pixels: 128 x {pixel 2q+1, pixel 2q}, each {b3, b2, b1, b0}
reg  [63:0] lpix [128];
reg   [6:0] lp_a, lpw_a;
reg  [63:0] lp_q, lpw_d;
reg         lpw_e = 0;
always @(posedge clk) begin
	if (lpw_e) lpix[lpw_a] <= lpw_d;
	lp_q <= lpix[lp_a];
end

// tile row (lin & 7) per BG span
reg   [2:0] spoff [64];
reg   [5:0] so_a, sow_a;
reg   [2:0] so_q, sow_d;
reg         sow_e = 0;
always @(posedge clk) begin
	if (sow_e) spoff[sow_a] <= sow_d;
	so_q <= spoff[so_a];
end

// sprite fetches of the line: {valid, vm, hm, bgp, pal 2, X 8, row 3, pat1 8, pat0 8};
// 16..31 line 239's (pixel (0,0) on line 0, 8a)
reg  [32:0] finfo [32];
reg   [4:0] fi_a, fiw_a;
reg  [32:0] fi_q, fiw_d;
reg         fiw_e = 0;
always @(posedge clk) begin
	if (fiw_e) finfo[fiw_a] <= fiw_d;
	fi_q <= finfo[fi_a];
end

// sprite list per pixel (8a): {drawn 4, bgp 4, n 3, e3, e2, e1, e0}, an entry e = {kind 2,
// index 6} (kind 0 fetch, 1 lane of this line, 2 lane of line 240)
localparam integer PLW = 43, PL_N = 32, PL_B = 35, PL_D = 39;
reg [PLW-1:0] plist [256];
reg   [7:0] pl_a, plw_a;
reg [PLW-1:0] pl_q, plw_d;
reg         plw_e = 0;
always @(posedge clk) begin
	if (plw_e) plist[plw_a] <= plw_d;
	pl_q <= plist[pl_a];
end

// BG instances of the line: {x0 8, n-1 3, slot 6}
reg  [16:0] bglist [64];
reg   [5:0] bl_a, blw_a;
reg  [16:0] bl_q, blw_d;
reg         blw_e = 0;
always @(posedge clk) begin
	if (blw_e) bglist[blw_a] <= blw_d;
	bl_q <= bglist[bl_a];
end

// a lookup result: {found, transparent, uncached, slot 7, payload 32} = 42 bits
// BG walk results of one instance: {dis, result}
reg  [42:0] wres [8];
reg   [2:0] wr_a, wrw_a;
reg  [42:0] wr_q, wrw_d;
reg         wrw_e = 0;
always @(posedge clk) begin
	if (wrw_e) wres[wrw_a] <= wrw_d;
	wr_q <= wres[wr_a];
end

// final BG result per pixel (after the 8-pixel cache rule)
reg  [41:0] bgres [256];
reg   [7:0] bg_a, bgw_a;
reg  [41:0] bg_q, bgw_d;
reg         bgw_e = 0;
always @(posedge clk) begin
	if (bgw_e) bgres[bgw_a] <= bgw_d;
	bg_q <= bgres[bg_a];
end

// sprite results, {x, list position}
reg  [41:0] sprres [1024];
reg   [9:0] sr_a, srw_a;
reg  [41:0] sr_q, srw_d;
reg         srw_e = 0;
always @(posedge clk) begin
	if (srw_e) sprres[srw_a] <= srw_d;
	sr_q <= sprres[sr_a];
end

// image cache: 128 slots x 8 columns, one RAM per output row (slot 127 = scratch)
reg  [63:0] img0 [1024];
reg  [63:0] img1 [1024];
reg   [9:0] im_a, imw_a;
reg  [63:0] im0_q, im1_q, imw_d;
reg         imw0_e = 0, imw1_e = 0;
always @(posedge clk) begin
	if (imw0_e) img0[imw_a] <= imw_d;
	im0_q <= img0[im_a];
end
always @(posedge clk) begin
	if (imw1_e) img1[imw_a] <= imw_d;
	im1_q <= img1[im_a];
end

// frame-wide "BG pixel looked up" bitmap, even and odd x: {y, x / 2}
reg         lkb0 [32768];
reg         lkb1 [32768];
reg  [14:0] lk_a, lkw_a;
reg         lk0_q, lk1_q, lkw_d;
reg         lkw0_e = 0, lkw1_e = 0;
always @(posedge clk) begin
	if (lkw0_e) lkb0[lkw_a] <= lkw_d;
	lk0_q <= lkb0[lk_a];
end
always @(posedge clk) begin
	if (lkw1_e) lkb1[lkw_a] <= lkw_d;
	lk1_q <= lkb1[lk_a];
end

// deferred frame-condition memo: {valid, value} per memo index, cleared at every frame start
// (S_MCLR). A frame-parity tag is NOT enough: an entry never written in the previous frame
// still carries the tag of the frame before, so every other frame would read it as a hit.
reg   [1:0] dmemo [2048];
reg  [10:0] dm_a;
reg   [1:0] dm_q, dmw_d;
reg         dmw_e = 0;
always @(posedge clk) begin
	if (dmw_e) dmemo[dm_a] <= dmw_d;
	dm_q <= dmemo[dm_a];
end

// ==========================================================================================
// condition list / rule cache (port C) and condition record cache (port E)
// ==========================================================================================
reg         rc_req = 0;
reg         rc_byp = 0;              // with rc_req: an uncached frame-record read (8k)
reg  [27:3] rc_addr;
wire        rc_ack;
wire [63:0] rc_q;
hd3_rcache #(.LB(8)) rcache (
	.clk(clk), .reset(reset), .flush(flush),
	.req(rc_req), .byp(rc_byp), .addr(rc_addr), .ack(rc_ack), .q(rc_q),
	.rd_req(c_rd_req), .rd_addr(c_rd_addr), .rd_ack(c_rd_ack), .rd_data(c_rd_data),
	.rd_valid(c_rd_valid)
);

reg          cc_req = 0;
reg   [23:0] cc_ci;
reg   [23:0] m_ci;                  // the evaluated condition (cc_ci is shared with the prefetch)
wire         cc_ack;
wire [255:0] cc_q;
hd3_ccache #(.LB(8)) ccache (
	.clk(clk), .reset(reset), .flush(flush), .base(conds_base),
	.req(cc_req), .ci(cc_ci), .ack(cc_ack), .q(cc_q),
	.rd_req(e_rd_req), .rd_addr(e_rd_addr), .rd_ack(e_rd_ack), .rd_data(e_rd_data),
	.rd_valid(e_rd_valid)
);

// ==========================================================================================
// colour index (0 = transparent) of a sprite pixel: screen column c = x - X is bit c of
// both pattern bytes (hdref.build_records: sci = pix1 bit | pix2 bit << 1; pix1 = bits 15:8)
// ==========================================================================================
function [1:0] sci_of(input [7:0] pix1, input [7:0] pix2, input [7:0] sx, input [7:0] xx);
	reg [7:0] c;
	begin
		c = xx - sx;
		sci_of = {pix2[c[2:0]], pix1[c[2:0]]};
	end
endfunction

// ==========================================================================================
// sprite list of a pixel (hdref.build_records): the first four valid fetches covering x, in
// fetch order, as {lowest 3, drawn 4, bgp 4, n 3, f3 4, f2 4, f1 4, f0 4}. Fetch i of a line's
// 16-lane word: {valid, bgp, X 8, pix1 8, pix2 8}. drawn[j]: hdref.render looks entry j up
// (draws it): every behind-background sprite, and a front sprite whose index is below
// `lowest` (the first behind-background entry with an opaque pixel, 4 if none).
// ==========================================================================================
function [29:0] find4(input [415:0] sq, input [4:0] nf, input [7:0] xx);
	reg [15:0] cov;
	reg  [8:0] d;
	reg  [2:0] n;
	reg [15:0] f;
	reg  [3:0] bgp, opq, drawn;
	reg  [2:0] lowest;
	reg [25:0] ln;
	integer i;
	begin
		for (i = 0; i < 16; i = i + 1) begin
			d = {1'b0, xx} - {1'b0, sq[26 * i + 16 +: 8]};
			cov[i] = sq[26 * i + 25] && (i < nf) && !d[8] && (d[7:3] == 5'd0);
		end
		n = 0; f = 0; bgp = 0; opq = 0;
		for (i = 0; i < 16; i = i + 1)
			if (cov[i] && n < 3'd4) begin
				ln = sq[26 * i +: 26];
				f[4 * n[1:0] +: 4] = i[3:0];
				bgp[n[1:0]] = ln[24];
				opq[n[1:0]] = sci_of(ln[15:8], ln[7:0], ln[23:16], xx) != 2'd0;
				n = n + 3'd1;
			end
		lowest = 3'd4;
		for (i = 3; i >= 0; i = i - 1)
			if (bgp[i] && opq[i]) lowest = i[2:0];
		for (i = 0; i < 4; i = i + 1)
			drawn[i] = (i < n) && (bgp[i] || lowest > i[2:0]);
		find4 = {lowest, drawn, bgp, n, f};
	end
endfunction

// a lane covers pixel xx of its line
function lcov(input [LW-1:0] l, input [7:0] xx);
	reg [7:0] d;
	begin
		d = xx - l[7:0];
		lcov = xx >= l[7:0] && d <= {5'd0, l[10:8]};
	end
endfunction

// 8a (hdref.draw_additions appends to a pixel's list, sprites shown there or not): the
// captured list (none where sprites are not shown) then the lanes covering xx, four in all.
// An added sprite has no colour (sci 0), so it never sets `lowest`.
// Pixel list form (plist): entries {kind 2, index 6}, fetches offset by `fo`, lanes of kind lk.
function [PLW-1:0] mergep(input [29:0] fl, input shown, input [NL*LW-1:0] lq, input [6:0] nl,
                          input [7:0] xx, input [4:0] fo, input [1:0] lk);
	reg  [2:0] n, lowest;
	reg [31:0] e;
	reg  [3:0] bgp, drawn;
	reg [LW-1:0] l;
	integer i, k;
	begin
		n = 0; e = 0; bgp = 0; lowest = 3'd4;
		if (shown) begin
			n = fl[18:16]; bgp = fl[22:19]; lowest = fl[29:27];
			for (i = 0; i < 4; i = i + 1) e[8 * i +: 8] = {3'd0, fl[4 * i +: 4] + fo};
		end
		for (k = 0; k < NL; k = k + 1) begin
			l = lq[LW * k +: LW];
			if (k < nl && n < 3'd4 && lcov(l, xx)) begin
				e[8 * n[1:0] +: 8] = {lk, k[5:0]};
				bgp[n[1:0]] = l[LW-2];
				n = n + 3'd1;
			end
		end
		for (i = 0; i < 4; i = i + 1)
			drawn[i] = (i < n) && (bgp[i] || lowest > i[2:0]);
		mergep = {drawn, bgp, n, e};
	end
endfunction

// Target list form (conditions): {drawn 4, bgp 4, n 3, 4 x {lane, fetch index or added id}}
localparam integer TEW = RAW + 1;
localparam integer TLW = 11 + 4 * TEW;
function [TLW-1:0] merget(input [29:0] fl, input shown, input [NL*LW-1:0] lq, input [6:0] nl,
                          input [7:0] xx);
	reg  [2:0] n, lowest;
	reg [4*TEW-1:0] e;
	reg  [3:0] bgp, drawn;
	reg [LW-1:0] l;
	integer i, k;
	begin
		n = 0; e = 0; bgp = 0; lowest = 3'd4;
		if (shown) begin
			n = fl[18:16]; bgp = fl[22:19]; lowest = fl[29:27];
			for (i = 0; i < 4; i = i + 1) e[TEW * i +: TEW] = {{(TEW-4){1'b0}}, fl[4 * i +: 4]};
		end
		for (k = 0; k < NL; k = k + 1) begin
			l = lq[LW * k +: LW];
			if (k < nl && n < 3'd4 && lcov(l, xx)) begin
				e[TEW * n[1:0] +: TEW] = {1'b1, l[11 +: RAW]};
				bgp[n[1:0]] = l[LW-2];
				n = n + 3'd1;
			end
		end
		for (i = 0; i < 4; i = i + 1)
			drawn[i] = (i < n) && (bgp[i] || lowest > i[2:0]);
		merget = {drawn, bgp, n, e};
	end
endfunction

// ==========================================================================================
// port L: palette, line load, image rows (one always block owns all of it)
// ==========================================================================================
localparam [1:0] TG_PAL = 2'd0, TG_LINE = 2'd1, TG_IMG = 2'd2;
reg  [10:0] lq [8];                  // in-flight bursts in issue order: {type 2, index 8, row 1}
reg   [2:0] lq_w = 0, lq_r = 0;
reg   [3:0] lq_n = 0;
reg   [2:0] lbeat = 0;

reg         pal_kick = 0, ld_kick = 0;   // pulses from the sequencer
reg         pal_go = 0;
reg   [2:0] pal_i;
reg   [2:0] pal_rx;                  // palette bursts received
reg         ld_go = 0;
reg   [5:0] ld_i;
reg   [5:0] ld_rx;                   // line bursts received
// line bursts: 0..13 line block qwords 0..111, 14..29 pixel qwords; on line 0 when the pack has
// additions (vz) also 30 = line 239's first pixel burst, 31..44 = line 239's line block (8a)
wire  [5:0] ld_n = vz ? 6'd45 : 6'd30;
// image fetch FIFO {slot 7, block 18, tile row 3}
reg  [27:0] ifq [16];
reg   [3:0] ifq_w = 0, ifq_r = 0;
reg   [4:0] ifq_n = 0;
reg         ifq_row = 0;
reg   [6:0] img_out = 0;             // image bursts issued and not complete
reg         if_push = 0;             // from the sequencer
reg  [27:0] if_d;
wire [24:0] lbase = REC_BASE[27:3] + {7'd0, fbank, 17'd0};

wire        l_can   = (!l_rd_req || l_rd_ack) && lq_n < 4'd6;
wire        is_pal  = l_can && pal_go;
wire        is_ld   = l_can && !pal_go && ld_go;
wire        is_img  = l_can && !pal_go && !ld_go && ifq_n != 5'd0;
wire        l_issue = is_pal || is_ld || is_img;
wire        l_pop   = l_rd_valid && lbeat == 3'd7;
wire  [1:0] h_type  = lq[lq_r][10:9];
wire  [7:0] h_idx   = lq[lq_r][8:1];
wire  [6:0] h_q     = {h_idx[3:0], lbeat};      // line block qword (bursts 0..13)
wire [27:0] ifq_h   = ifq[ifq_r];

always @(posedge clk) begin
	lpw_e <= 0; sow_e <= 0; fiw_e <= 0; pmw_e <= 0; imw0_e <= 0; imw1_e <= 0;

	if (pal_kick) begin pal_go <= 1; pal_i <= 0; pal_rx <= 0; end
	if (ld_kick)  begin ld_go <= 1; ld_i <= 0; ld_rx <= 0; end

	if (l_rd_req && l_rd_ack) l_rd_req <= 0;
	if (is_pal) begin
		l_rd_req  <= 1;
		l_rd_addr <= pal_base + {19'd0, pal_i, 3'd0};
		lq[lq_w]  <= {TG_PAL, 5'd0, pal_i, 1'b0};
		if (pal_i == 3'd3) pal_go <= 0;
		pal_i <= pal_i + 3'd1;
	end else if (is_ld) begin
		l_rd_req  <= 1;
		l_rd_addr <= (ld_i < 6'd14) ? lbase + 25'h08000 + {9'd0, y, 8'd0} + {16'd0, ld_i, 3'd0} :
		             (ld_i < 6'd30) ? lbase + {10'd0, y, 7'd0} + {16'd0, ld_i - 6'd14, 3'd0} :
		             (ld_i == 6'd30) ? lbase + {10'd0, 8'd239, 7'd0} :
		                              lbase + 25'h08000 + {9'd0, 8'd239, 8'd0} + {16'd0, ld_i - 6'd31, 3'd0};
		lq[lq_w]  <= {TG_LINE, 2'd0, ld_i, 1'b0};
		if (ld_i == ld_n - 6'd1) ld_go <= 0;
		ld_i <= ld_i + 6'd1;
	end else if (is_img) begin
		l_rd_req  <= 1;
		// block = 1024 bytes = 128 qwords; output row r (0..15) = 8 qwords
		l_rd_addr <= blocks_base + {ifq_h[20:3], 7'd0} + {18'd0, ifq_h[2:0], ifq_row, 3'd0};
		lq[lq_w]  <= {TG_IMG, 1'b0, ifq_h[27:21], ifq_row};
		ifq_row   <= ~ifq_row;
		if (ifq_row) ifq_r <= ifq_r + 4'd1;
	end
	if (l_issue) lq_w <= lq_w + 3'd1;

	if (l_rd_valid) begin
		lbeat <= lbeat + 3'd1;
		case (h_type)
			TG_PAL: begin
				pmw_e <= 1;
				pmw_a <= {h_idx[1:0], lbeat};
				pmw_d <= {l_rd_data[55:32], l_rd_data[23:0]};
				if (lbeat == 3'd7) pal_rx <= pal_rx + 3'd1;
			end
			TG_LINE: begin
				if (h_idx[5:0] >= 6'd31) begin : vline
					// line 239's line block (8a): its header fields replace line 0's (Mesen's
					// line-0 first pixel is line 239's), its palette, the span of its pixel 0,
					// its fetches into finfo 16..31
					reg [6:0] vq;
					vq = {h_idx[3:0] - 4'd15, lbeat};   // h_idx 31..44 -> qword 0..111
					if (vq == 7'd0) begin
						l_t     <= l_rd_data[14:0];
						l_finex <= l_rd_data[17:15];
						l_gray  <= l_rd_data[18];
						l_emph  <= l_rd_data[21:19];
						v_nspan <= (l_rd_data[39:32] > 8'd34) ? 6'd34 : l_rd_data[37:32];
						v_nfet  <= (l_rd_data[55:48] > 8'd16) ? 5'd16 : l_rd_data[52:48];
					end
					if (vq == 7'd1) l_pal239[63:0]    <= l_rd_data;
					if (vq == 7'd2) l_pal239[127:64]  <= l_rd_data;
					if (vq == 7'd3) l_pal239[191:128] <= l_rd_data;
					if (vq == 7'd8 + {px239[5:0], 1'b0}) begin
						sow_e <= 1; sow_a <= 6'd62; sow_d <= l_rd_data[2:0];
					end
					if (vq >= 7'd80 && vq < 7'd112 && !lbeat[0]) begin
						fiw_e <= 1;
						fiw_a <= {1'b1, 4'(vq[6:1] - 6'd40)};
						fiw_d <= {l_rd_data[54], l_rd_data[53], l_rd_data[52], l_rd_data[51],
						          l_rd_data[47:46], l_rd_data[45:38], l_rd_data[18:16],
						          l_rd_data[15:8], l_rd_data[7:0]};
					end
				end else if (h_idx[5:0] == 6'd30) begin
					if (lbeat == 3'd0) px239 <= l_rd_data[31:0];
				end else if (h_idx[5:0] < 6'd14) begin
					if (h_q == 7'd0) begin
						l_t     <= l_rd_data[14:0];
						l_finex <= l_rd_data[17:15];
						l_gray  <= l_rd_data[18];
						l_emph  <= l_rd_data[21:19];
						l_nspan <= (l_rd_data[39:32] > 8'd34) ? 6'd34 : l_rd_data[37:32];
						l_nfet  <= (l_rd_data[55:48] > 8'd16) ? 5'd16 : l_rd_data[52:48];
					end
					if (h_q == 7'd1) l_pal[63:0]    <= l_rd_data;
					if (h_q == 7'd2) l_pal[127:64]  <= l_rd_data;
					if (h_q == 7'd3) l_pal[191:128] <= l_rd_data;
					if (h_q >= 7'd8 && h_q < 7'd76 && !lbeat[0]) begin
						sow_e <= 1;
						sow_a <= h_q[6:1] - 6'd4;
						sow_d <= l_rd_data[2:0];
					end
					if (h_q >= 7'd80 && h_q < 7'd112 && !lbeat[0]) begin
						fiw_e <= 1;
						fiw_a <= {1'b0, 4'(h_q[6:1] - 6'd40)};   // fetch (q - 80) / 2
						fiw_d <= {l_rd_data[54], l_rd_data[53], l_rd_data[52], l_rd_data[51],
						          l_rd_data[47:46], l_rd_data[45:38], l_rd_data[18:16],
						          l_rd_data[15:8], l_rd_data[7:0]};
					end
				end else begin
					lpw_e <= 1;
					lpw_a <= {h_idx[3:0] - 4'd14, lbeat};
					lpw_d <= l_rd_data;
				end
				if (lbeat == 3'd7) ld_rx <= ld_rx + 6'd1;
			end
			default: begin
				if (lq[lq_r][0]) imw1_e <= 1; else imw0_e <= 1;
				imw_a <= {h_idx[6:0], lbeat};
				imw_d <= l_rd_data;
			end
		endcase
	end
	if (l_pop) lq_r <= lq_r + 3'd1;
	lq_n <= lq_n + (l_issue ? 4'd1 : 4'd0) - (l_pop ? 4'd1 : 4'd0);
	img_out <= img_out + (is_img ? 7'd1 : 7'd0) - ((l_pop && h_type == TG_IMG) ? 7'd1 : 7'd0);

	if (if_push) begin
		ifq[ifq_w] <= if_d;
		ifq_w <= ifq_w + 4'd1;
	end
	ifq_n <= ifq_n + (if_push ? 5'd1 : 5'd0) - ((is_img && ifq_row) ? 5'd1 : 5'd0);

	if (reset) begin
		l_rd_req <= 0; lq_w <= 0; lq_r <= 0; lq_n <= 0; lbeat <= 0;
		pal_go <= 0; ld_go <= 0; ifq_w <= 0; ifq_r <= 0; ifq_n <= 0; ifq_row <= 0;
		img_out <= 0;
	end
end

// ==========================================================================================
// sprite-list pass (during LOAD): plist[x] from hd3_fmap's pixel map and sprite table and the
// line's lanes (8a). First the lanes of line y and of line 240 into registers (hd3_add's reader,
// one clock latency, presented one line ahead of the read); then pixels 0..255; on line 0 when
// the pack has additions, pixel 0 once more from line 239's pixel 0 and line 240's lanes.
// ==========================================================================================
reg         pp_kick = 0;             // pulse from the sequencer
reg         pp_on = 0;
reg   [2:0] pp_ph = 0;               // 1-3 lane reads, 4 pixels, 5-6 pixel (0,0) (vz)
reg   [8:0] pp_x = 0;                // pixel whose map pair is presented this clock
reg         pp_v1 = 0;
reg   [7:0] pp_x1;
reg   [7:0] pp_ay;

reg  [14:0] m_pa;                    // sequencer's fmap addresses
reg   [7:0] m_ly, m_sy;
reg   [7:0] m_ay;                    // sequencer's lane line
wire        pp_ex = pp_ph == 3'd5;   // present line 239, pixel 0
assign p_a = !pp_on ? m_pa : pp_ex ? {8'd239, 7'd0} : {y, pp_x[7:1]};
assign l_y = !pp_on ? m_ly : pp_ex ? 8'd239 : y;
assign s_y = !pp_on ? m_sy : pp_ex ? 8'd239 : y;
assign a_y = pp_on ? pp_ay : m_ay;

always @(posedge clk) begin
	plw_e <= 0;
	if (pp_kick) begin pp_on <= 1; pp_ph <= 3'd1; pp_ay <= y; end
	else if (pp_on) case (pp_ph)
		3'd1: begin pp_ay <= 8'd240; pp_ph <= 3'd2; end
		3'd2: begin ln_cur <= a_q; n_cur <= a_n; pp_ph <= 3'd3; end
		3'd3: begin ln_240 <= a_q; n_240 <= a_n; pp_x <= 0; pp_v1 <= 0; pp_ph <= 3'd4; end
		3'd4: begin
			pp_v1 <= !pp_x[8];
			pp_x1 <= pp_x[7:0];
			if (!pp_x[8]) pp_x <= pp_x + 9'd1;
			if (pp_v1) begin : pp_write
				// find4 into a blocking local first, then the non-blocking write: called inline
				// inside the ternary, Verilator dropped the entry's bgp bit (a behind-BG sprite
				// then drew in FRONT of the background); the other find4 sites use this shape too
				reg [29:0] fl;
				fl = find4(s_q, l_q[10:6], pp_x1);
				plw_e <= 1;
				plw_a <= pp_x1;
				plw_d <= mergep(fl, pp_x1[0] ? p_q[13] : p_q[6], ln_cur, n_cur, pp_x1, 5'd0, 2'd1);
			end
			if (pp_x[8] && !pp_v1) begin
				if (vz) pp_ph <= 3'd5;
				else pp_on <= 0;
			end
		end
		3'd5: pp_ph <= 3'd6;                  // line 239's map pair is read this clock
		default: begin : pp_00
			reg [29:0] fl;
			fl = find4(s_q, l_q[10:6], 8'd0);
			plw_e <= 1;
			plw_a <= 8'd0;
			plw_d <= mergep(fl, p_q[6], ln_240, n_240, 8'd0, 5'd16, 2'd2);
			pp_on <= 0;
		end
	endcase
	if (reset) begin pp_on <= 0; pp_v1 <= 0; pp_ph <= 0; end
end

// ==========================================================================================
// backgrounds (8j). Per frame (kicked at S_PAL, done while the memo sweep runs): each level's
// selected record (port B) - hdref.select_backgrounds ran in hd3_resolve. Per line (kicked at
// the end of LOAD, alongside the walks), level by level as hdref.render / bg_line / bg_pixel:
//   - sx = ratio(hr, scroll_x), sy = ratio(vr, scroll_y) from the line's first pixel (t,
//     finex); each lookup cached per level, since the scroll seldom changes between lines;
//   - visible iff y >= -sy and y + top + sy + 1 <= h; drawn pixels x in [lo, hi] = [-sx,
//     w - left - sx - 1] clipped to the window; px = base + x (base = left + sx), py = top +
//     y + sy, cell (px >> 3, py >> 3) of the record's map, row py & 7;
//   - the map words of the cells across [lo, hi] (at most 33, one cell row) into a buffer;
//   - per 8-pixel window group g its cells A = (base >> 3) + g and B = A + 1 (pixel j is in A
//     when ph + j < 8, ph = base & 7), an image slot per non-transparent cell (one per
//     (block, row) on the line: bsc), the mask byte of the group's drawn pixels and the group
//     entry {mode, row, ph, B, A} that compose reads.
// A level written on a line is "dirty": the next line rewrites or clears every window group of
// it, so the mask never holds a byte of an earlier line. An unselected or invisible clean
// level costs one clock.
// ==========================================================================================
localparam integer NLVL = 40;

// level records {mode 2, vr 16, hr 16, top 32, left 32, h 32, w 32, map 32} and line-to-line
// caches {xv, xs 9, xval 32, yv, ys 9, yval 32, rv, rcr 29, rval 28} (rval = rcr * cols mod
// 2**28: only address bits matter), both read at br_a
reg [193:0] bgr [NLVL];
reg [141:0] bgc [NLVL];
reg   [5:0] br_a, brw_a, bcw_a;
reg [193:0] br_q, brw_d;
reg [141:0] bc_q, bcw_d;
reg         brw_e = 0, bcw_e = 0;
always @(posedge clk) begin
	if (brw_e) bgr[brw_a] <= brw_d;
	br_q <= bgr[br_a];
end
always @(posedge clk) begin
	if (bcw_e) bgc[bcw_a] <= bcw_d;
	bc_q <= bgc[br_a];
end

// drawn-pixel masks per window group: byte L = level L's pixels 0..7 of the group (compose
// front stage reads bm_a)
reg [319:0] bgm [32];
reg   [4:0] bm_a, bmw_a;
reg   [5:0] bmw_l;
reg   [7:0] bmw_d;
reg [319:0] bm_q;
reg         bmw_e = 0;
always @(posedge clk) begin
	if (bmw_e) bgm[bmw_a][{bmw_l, 3'b000} +: 8] <= bmw_d;
	bm_q <= bgm[bm_a];
end

// group entries by {level, group}: {mode 2, row 3, ph 3, cell B 26, cell A 26}, a cell =
// {uncached, slot 7, block 18} (compose back stage reads be_ra)
reg  [59:0] bge [NLVL * 32];
reg  [10:0] bew_a;
wire [10:0] be_ra;
reg  [59:0] be_q, bew_d;
reg         bew_e = 0;
always @(posedge clk) begin
	if (bew_e) bge[bew_a] <= bew_d;
	be_q <= bge[be_ra];
end

// this line's (block, row) -> slot, direct mapped: {valid, y 8, block 18, row 3, slot 7}
reg  [36:0] bsc [64];
reg   [5:0] bs_a, bsw_a;
reg  [36:0] bs_q, bsw_d;
reg         bsw_e = 0;
always @(posedge clk) begin
	if (bsw_e) bsc[bsw_a] <= bsw_d;
	bs_q <= bsc[bs_a];
end

// map words of the level's cells on this line (three bursts at most)
reg  [63:0] bmb [32];
reg   [4:0] mb_a, mbw_a;
reg  [63:0] mb_q, mbw_d;
reg         mbw_e = 0;
always @(posedge clk) begin
	if (mbw_e) bmb[mbw_a] <= mbw_d;
	mb_q <= bmb[mb_a];
end

reg         bf_kick = 0, bl_kick = 0;   // pulses from the sequencer: frame load, line prep
reg         ba_req = 0;                 // a cell's (block, row) needs an image slot: the
reg  [17:0] ba_blk;                     //   sequencer allocates it (ns, image fetch queue)
reg   [2:0] ba_row;
reg         ba_done = 0, ba_unc = 0;    // from the sequencer
reg   [6:0] ba_slot;

typedef enum bit [4:0] {
	B_IDLE, B_F0, B_F1, B_F2, B_F3, B_F4,
	B_L1, B_L2, B_L3, B_L4, B_L5, B_RW, B_L6, B_L7, B_MUL, B_MF, B_MW,
	B_G, B_C0, B_C1, B_C2, B_C3, B_C4, B_C5, B_GW, B_Z
} bs_t;
bs_t bs = B_IDLE;

reg   [5:0] bl;                         // level
reg  [39:0] bsel = 0;                   // levels with a selected record this frame
reg  [39:0] bdirty = {40{1'b1}};        // levels whose mask bytes may be non-zero
reg   [5:0] bclr;
reg   [2:0] bbeat;
reg         bwhich, bhalf;              // ratio lookup: 0 x / 1 y; its word in the qword
// the level being prepared
reg  [31:0] bv_map, bv_w, bv_h, bv_left, bv_top;
reg  [15:0] bv_hr, bv_vr;
reg   [1:0] bv_mode;
reg         bv_xv, bv_yv, bv_rv;
reg   [8:0] bv_xs, bv_ys;
reg  [31:0] bv_xval, bv_yval;
reg  [28:0] bv_rcr;
reg  [27:0] bv_rval;
reg         bv_vis, bv_empty;
reg  [31:0] bv_py;
reg  [35:0] bv_base;
reg   [7:0] bv_lo, bv_hi;
reg  [28:0] bv_clo, bv_chi;
reg   [2:0] bv_row, bv_ph;
reg   [4:0] bv_mi;                      // rowbase multiply: bit of rcr
reg  [27:0] bv_acc;
reg  [27:0] bv_a0;                      // byte address of cell clo's map word
reg   [4:0] bv_nq;                      // map qwords to fetch (<= 17)
reg   [1:0] bv_k;                       // map burst
reg   [4:0] bg_g;                       // group
reg         ca_ok, ca_t, cb_t;          // cell A known; A / B transparent
reg  [25:0] ca_r, cb_r;                 // {uncached, slot 7, block 18}
reg         lk_b;                       // looking up B (else A)
reg  [32:0] lk_c;                       // the cell looked up
reg  [17:0] lk_blk;

wire  [8:0] bscr_x = {l_t[10], l_t[4:0], l_finex};
wire  [8:0] bscr_y = {1'b0, l_t[9:5], l_t[14:12]} + (l_t[11] ? 9'd240 : 9'd0);
wire  [4:0] bgl = ov_left[7:3];
wire  [4:0] bgr_g = xr[7:3];
wire [27:0] cols28 = 28'(({1'b0, bv_w} + 33'd7) >> 3);
wire [32:0] bcb = bv_base[35:3];        // cell of the group-0 pixel's A (signed)
wire [32:0] bca = bcb + {28'd0, bg_g};  // this group's cell A
// geometry of the level on this line (hdref.bg_line), 36-bit signed
wire signed [35:0] g_sx  = {{4{bv_xval[31]}}, bv_xval};
wire signed [35:0] g_sy  = {{4{bv_yval[31]}}, bv_yval};
wire signed [35:0] g_a   = {28'd0, y} + g_sy;                        // y + sy
wire signed [35:0] g_b   = {4'd0, bv_top} + g_a + 36'sd1;
wire signed [35:0] g_h   = {4'd0, bv_h};
wire signed [35:0] g_xl  = {28'd0, ov_left};   // signed operands on both sides of every compare
wire signed [35:0] g_xr  = {28'd0, xr};
wire signed [35:0] g_mn  = -g_sx;
wire signed [35:0] g_mx  = {4'd0, bv_w} - {4'd0, bv_left} - g_sx - 36'sd1;
wire signed [35:0] g_lo  = (g_mn > g_xl) ? g_mn : g_xl;
wire signed [35:0] g_hi  = (g_mx < g_xr) ? g_mx : g_xr;
wire        [35:0] g_c0  = bv_base + {28'd0, bv_lo};
wire        [35:0] g_c1  = bv_base + {28'd0, bv_hi};
// a cell's map word in the buffer: word (c - clo) + a0[2]
wire        [32:0] lk_o  = lk_c - {4'd0, bv_clo};
wire         [5:0] lk_w  = lk_o[5:0] + {5'd0, bv_a0[2]};
wire         [5:0] lk_si = lk_blk[5:0] ^ {bv_row, 3'b000};
wire        [31:0] lk_word = lk_w[0] ? mb_q[63:32] : mb_q[31:0];
// the group's mask byte: its pixels in [lo, hi], each in its cell, the cell not transparent
wire         [7:0] gp_lo = {bg_g, 3'b000};
wire         [7:0] gp_hi = {bg_g, 3'b111};
wire         [2:0] gp_lj = (gp_lo >= bv_lo) ? 3'd0 : 3'(bv_lo - gp_lo);
wire         [2:0] gp_hj = (gp_hi <= bv_hi) ? 3'd7 : 3'(bv_hi - gp_lo);
wire         [7:0] gp_rb = (8'hFF << gp_lj) & (8'hFF >> (3'd7 - gp_hj));
wire         [7:0] gp_am = 8'hFF >> bv_ph;
wire         [7:0] gp_m  = gp_rb & ((ca_t ? 8'd0 : gp_am) | (cb_t ? 8'd0 : ~gp_am));
wire         [5:0] bl_n  = bl + 6'd1;

task automatic bnext;                   // the next level, or the line is done
	if (bl == 6'd39) bs <= B_IDLE;
	else begin bl <= bl_n; bs <= B_L1; end
endtask

task automatic bcache_wb;               // the level's caches back
	bcw_e <= 1; bcw_a <= bl;
	bcw_d <= {bv_xv, bv_xs, bv_xval, bv_yv, bv_ys, bv_yval, bv_rv, bv_rcr, bv_rval};
endtask

// a looked-up cell: its result, then B (when the group straddles two cells) or the group
task automatic bcell(input t, input [25:0] r);
	if (!lk_b) begin
		ca_t <= t; ca_r <= r; ca_ok <= 1;
		if (bv_ph != 3'd0) begin lk_b <= 1; lk_c <= bca + 33'd1; bs <= B_C0; end
		else bs <= B_GW;
	end else begin
		cb_t <= t; cb_r <= r; bs <= B_GW;
	end
endtask

always @(posedge clk) begin
	brw_e <= 0; bcw_e <= 0; bmw_e <= 0; bew_e <= 0; bsw_e <= 0; mbw_e <= 0;
	if (b_rd_req && b_rd_ack) b_rd_req <= 0;
	if (ba_done) ba_req <= 0;

	case (bs)
	B_IDLE: begin
		if (bf_kick) begin bclr <= 0; bs <= B_F0; end
		else if (bl_kick) begin bl <= 0; bs <= B_L1; end
	end

	// ---- frame: clear the slot map, then each level's selection and record ----
	B_F0: begin
		bsw_e <= 1; bsw_a <= bclr; bsw_d <= 37'd0;
		bclr <= bclr + 6'd1;
		if (bclr == 6'd63) begin bl <= 0; bs <= B_F1; end
	end
	B_F1: begin gs_lvl <= bl; bs <= B_F2; end
	B_F2: bs <= B_F3;
	B_F3: begin
		bsel[bl] <= gs_q[20];
		if (gs_q[20]) begin
			b_rd_req <= 1;
			b_rd_addr <= bgrec_base + {3'd0, gs_q[19:0], 2'b00};
			bbeat <= 0;
			bs <= B_F4;
		end else if (bl == 6'd39) bs <= B_IDLE;
		else begin bl <= bl_n; bs <= B_F1; end
	end
	B_F4: if (b_rd_valid) begin
		bbeat <= bbeat + 3'd1;
		case (bbeat)
			3'd0: begin bv_map <= b_rd_data[31:0]; bv_w <= b_rd_data[63:32]; end
			3'd1: begin bv_h <= b_rd_data[31:0]; bv_left <= b_rd_data[63:32]; end
			3'd2: begin bv_top <= b_rd_data[31:0]; bv_hr <= b_rd_data[47:32]; bv_vr <= b_rd_data[63:48]; end
			3'd3: bv_mode <= (b_rd_data[7:0] == 8'd0) ? 2'd0 : (b_rd_data[7:0] == 8'd1) ? 2'd1 : 2'd2;
			default: ;
		endcase
		if (bbeat == 3'd7) begin
			brw_e <= 1; brw_a <= bl;
			brw_d <= {bv_mode, bv_vr, bv_hr, bv_top, bv_left, bv_h, bv_w, bv_map};
			bcw_e <= 1; bcw_a <= bl; bcw_d <= 142'd0;
			if (bl == 6'd39) bs <= B_IDLE;
			else begin bl <= bl_n; bs <= B_F1; end
		end
	end

	// ---- line: level by level ----
	B_L1: begin
		if (!bsel[bl]) begin
			if (bdirty[bl]) begin bg_g <= bgl; bs <= B_Z; end
			else bnext();
		end else begin br_a <= bl; bs <= B_L2; end
	end
	B_L2: bs <= B_L3;
	B_L3: begin
		{bv_mode, bv_vr, bv_hr, bv_top, bv_left, bv_h, bv_w, bv_map} <= br_q;
		{bv_xv, bv_xs, bv_xval, bv_yv, bv_ys, bv_yval, bv_rv, bv_rcr, bv_rval} <= bc_q;
		bs <= B_L4;
	end
	B_L4: begin                            // sx
		if (bv_xv && bv_xs == bscr_x) bs <= B_L5;
		else begin : rqx
			reg [27:0] a;
			a = ratio_base + {1'b0, bv_hr, 11'd0} + {17'd0, bscr_x, 2'b00};
			b_rd_req <= 1; b_rd_addr <= a[27:3]; bhalf <= a[2];
			bwhich <= 0; bbeat <= 0; bs <= B_RW;
		end
	end
	B_L5: begin                            // sy
		if (bv_yv && bv_ys == bscr_y) bs <= B_L6;
		else begin : rqy
			reg [27:0] a;
			a = ratio_base + {1'b0, bv_vr, 11'd0} + {17'd0, bscr_y, 2'b00};
			b_rd_req <= 1; b_rd_addr <= a[27:3]; bhalf <= a[2];
			bwhich <= 1; bbeat <= 0; bs <= B_RW;
		end
	end
	B_RW: if (b_rd_valid) begin
		bbeat <= bbeat + 3'd1;
		if (bbeat == 3'd0) begin
			if (!bwhich) begin
				bv_xv <= 1; bv_xs <= bscr_x; bv_xval <= bhalf ? b_rd_data[63:32] : b_rd_data[31:0];
			end else begin
				bv_yv <= 1; bv_ys <= bscr_y; bv_yval <= bhalf ? b_rd_data[63:32] : b_rd_data[31:0];
			end
		end
		if (bbeat == 3'd7) bs <= bwhich ? B_L6 : B_L5;
	end
	B_L6: begin                            // visibility and the pixel range
		bv_vis   <= !g_a[35] && g_b <= g_h;
		bv_py    <= bv_top + g_a[31:0];
		bv_empty <= g_lo > g_hi;
		bv_lo    <= g_lo[7:0];
		bv_hi    <= g_hi[7:0];
		bv_base  <= {4'd0, bv_left} + g_sx;
		bs <= B_L7;
	end
	B_L7: begin
		if (!bv_vis || bv_empty) begin
			bcache_wb();
			if (bdirty[bl]) begin bg_g <= bgl; bs <= B_Z; end
			else bnext();
		end else begin
			bv_clo <= g_c0[31:3];
			bv_chi <= g_c1[31:3];
			bv_row <= bv_py[2:0];
			bv_ph  <= bv_base[2:0];
			if (bv_rv && bv_rcr == bv_py[31:3]) bs <= B_MF;
			else if (bv_rv && bv_rcr + 29'd1 == bv_py[31:3]) begin
				bv_rcr <= bv_py[31:3]; bv_rval <= bv_rval + cols28; bs <= B_MF;
			end else begin
				bv_acc <= 28'd0; bv_mi <= 5'd28; bs <= B_MUL;
			end
		end
	end
	B_MUL: begin                           // rval = rcr * cols mod 2**28, a bit a clock
		bv_acc <= {bv_acc[26:0], 1'b0} + (bv_py[3 + bv_mi] ? cols28 : 28'd0);
		bv_mi <= bv_mi - 5'd1;
		if (bv_mi == 5'd0) begin
			bv_rv <= 1; bv_rcr <= bv_py[31:3];
			bv_rval <= {bv_acc[26:0], 1'b0} + (bv_py[3] ? cols28 : 28'd0);
			bs <= B_MF;
		end
	end
	B_MF: begin : mf                       // the cells' map words: first burst
		reg [27:0] a;
		reg  [5:0] nw;
		a  = bgmap_base + bv_map[27:0] + 28'({bv_rval + 28'(bv_clo), 2'b00});
		nw = 6'(bv_chi - bv_clo) + 6'd1;
		bv_a0 <= a;
		bv_nq <= 5'(({1'b0, nw} + {6'd0, a[2]} + 7'd1) >> 1);
		bv_k  <= 0;
		b_rd_req <= 1; b_rd_addr <= a[27:3]; bbeat <= 0;
		bs <= B_MW;
	end
	B_MW: if (b_rd_valid) begin
		mbw_e <= 1; mbw_a <= {bv_k, bbeat}; mbw_d <= b_rd_data;
		bbeat <= bbeat + 3'd1;
		if (bbeat == 3'd7) begin
			if ({bv_k + 2'd1, 3'b000} < bv_nq) begin
				bv_k <= bv_k + 2'd1;
				b_rd_req <= 1; b_rd_addr <= bv_a0[27:3] + {20'd0, bv_k + 2'd1, 3'b000};
			end else begin
				bg_g <= bgl; ca_ok <= 0; bs <= B_G;
			end
		end
	end

	// ---- the window groups of a visible level ----
	B_G: begin
		if (gp_hi < bv_lo || gp_lo > bv_hi) begin
			bmw_e <= 1; bmw_a <= bg_g; bmw_l <= bl; bmw_d <= 8'd0;
			ca_ok <= 0;
			if (bg_g == bgr_g) begin bcache_wb(); bdirty[bl] <= 1; bnext(); end
			else bg_g <= bg_g + 5'd1;
		end else if (!ca_ok) begin lk_b <= 0; lk_c <= bca; bs <= B_C0; end
		else if (bv_ph != 3'd0) begin lk_b <= 1; lk_c <= bca + 33'd1; bs <= B_C0; end
		else bs <= B_GW;
	end
	B_C0: begin                            // a cell outside [clo, chi] draws nothing
		if ($signed(lk_c) < $signed({4'd0, bv_clo}) || $signed(lk_c) > $signed({4'd0, bv_chi}))
			bcell(1'b1, 26'd0);
		else begin mb_a <= lk_w[5:1]; bs <= B_C1; end
	end
	B_C1: bs <= B_C2;
	B_C2: begin
		if (lk_word == 32'hFFFFFFFF) bcell(1'b1, 26'd0);
		else begin lk_blk <= lk_word[17:0]; bs <= B_C3; end
	end
	B_C3: begin bs_a <= lk_si; bs <= B_C4; end
	B_C4: bs <= B_C5;
	B_C5: begin
		if (bs_q[36] && bs_q[35:28] == y && bs_q[27:10] == lk_blk && bs_q[9:7] == bv_row)
			bcell(1'b0, {1'b0, bs_q[6:0], lk_blk});
		else if (!ba_req) begin ba_req <= 1; ba_blk <= lk_blk; ba_row <= bv_row; end
		else if (ba_done) begin
			if (!ba_unc) begin
				bsw_e <= 1; bsw_a <= lk_si; bsw_d <= {1'b1, y, lk_blk, bv_row, ba_slot};
			end
			bcell(1'b0, {ba_unc, ba_unc ? 7'd0 : ba_slot, lk_blk});
		end
	end
	B_GW: begin
		bmw_e <= 1; bmw_a <= bg_g; bmw_l <= bl; bmw_d <= gp_m;
		bew_e <= 1; bew_a <= {bl, bg_g};
		bew_d <= {bv_mode, bv_row, bv_ph, cb_r, ca_r};
		if (bv_ph != 3'd0) begin ca_t <= cb_t; ca_r <= cb_r; end
		else ca_ok <= 0;
		if (bg_g == bgr_g) begin bcache_wb(); bdirty[bl] <= 1; bnext(); end
		else begin bg_g <= bg_g + 5'd1; bs <= B_G; end
	end

	// ---- clear every window group of a level not drawn on this line ----
	B_Z: begin
		bmw_e <= 1; bmw_a <= bg_g; bmw_l <= bl; bmw_d <= 8'd0;
		if (bg_g == bgr_g) begin bdirty[bl] <= 0; bnext(); end
		else bg_g <= bg_g + 5'd1;
	end
	default: bs <= B_IDLE;
	endcase

	if (reset) begin
		bs <= B_IDLE; b_rd_req <= 0; ba_req <= 0; bsel <= 0; bdirty <= {40{1'b1}};
	end
end

// ==========================================================================================
// output stage: grayscale and emphasis per output pixel, then out_v. The draw pipe pushes a
// finished pixel {emph, gray, y, x, 4 sub-pixels} into an 8-entry FIFO; a pixel is taken every
// two clocks, its sub-pixels looked up two per clock through dual-read emphasis ROMs.
// ==========================================================================================
localparam integer OF_D = 1 << OF_LOG2;
reg  [115:0] of_m [OF_D];
reg  [OF_LOG2-1:0] of_wp = 0, of_rp = 0;
reg  [OF_LOG2:0]   of_n = 0;
reg          of_push = 0;            // from the draw pipe (sequencer)
reg  [115:0] of_d;
reg          os1 = 0, os2 = 0, os3 = 0;   // stage valid: A issued, B issued, A result
wire [115:0] of_h = of_m[of_rp];
wire         of_pop = !os1 && of_n != 0;
wire         out_idle = of_n == 0 && !of_push && !os1 && !os2 && !os3;

reg   [7:0] rom_er [2048];
reg   [7:0] rom_eg [2048];
reg   [7:0] rom_eb [2048];
initial begin
	$readmemh(`HD3_HEX_ER, rom_er);
	$readmemh(`HD3_HEX_EG, rom_eg);
	$readmemh(`HD3_HEX_EB, rom_eb);
end
reg  [10:0] er_a, eg_a, eb_a, er_b, eg_b, eb_b;
reg   [7:0] er_q, eg_q, eb_q, er_r, eg_r, eb_r;
always @(posedge clk) begin
	er_q <= rom_er[er_a]; er_r <= rom_er[er_b];
	eg_q <= rom_eg[eg_a]; eg_r <= rom_eg[eg_b];
	eb_q <= rom_eb[eb_a]; eb_r <= rom_eb[eb_b];
end

// grayscale: (r + g + b) / 3, exact for sums up to 765
function [7:0] avg3(input [23:0] c);
	reg  [9:0] s;
	reg [19:0] m;
	begin
		s = {2'b0, c[23:16]} + {2'b0, c[15:8]} + {2'b0, c[7:0]};
		m = s * 10'd683;
		avg3 = m[18:11];
	end
endfunction

reg  [95:0] o_in1;
reg  [15:0] o_yx1, o_yx2, o_yx3;
reg   [2:0] o_emph1;
reg         o_gray1;
reg  [47:0] o_res;

always @(posedge clk) begin
	out_v <= 0;
	os1 <= of_pop; os2 <= os1; os3 <= os2;
	if (of_push) begin of_m[of_wp] <= of_d; of_wp <= of_wp + 1'b1; end
	if (of_pop) of_rp <= of_rp + 1'b1;
	of_n <= of_n + {{OF_LOG2{1'b0}}, of_push} - {{OF_LOG2{1'b0}}, of_pop};
	if (of_pop) begin
		// phase A: sub-pixels 0 and 1 of the FIFO head, readable two clocks later
		er_a <= {of_h[115:113], of_h[112] ? avg3(of_h[23:0])  : of_h[23:16]};
		eg_a <= {of_h[115:113], of_h[112] ? avg3(of_h[23:0])  : of_h[15:8]};
		eb_a <= {of_h[115:113], of_h[112] ? avg3(of_h[23:0])  : of_h[7:0]};
		er_b <= {of_h[115:113], of_h[112] ? avg3(of_h[47:24]) : of_h[47:40]};
		eg_b <= {of_h[115:113], of_h[112] ? avg3(of_h[47:24]) : of_h[39:32]};
		eb_b <= {of_h[115:113], of_h[112] ? avg3(of_h[47:24]) : of_h[31:24]};
		o_in1 <= of_h[95:0]; o_yx1 <= of_h[111:96];
		o_emph1 <= of_h[115:113]; o_gray1 <= of_h[112];
	end else if (os1) begin
		// phase B: sub-pixels 2 and 3
		er_a <= {o_emph1, o_gray1 ? avg3(o_in1[71:48]) : o_in1[71:64]};
		eg_a <= {o_emph1, o_gray1 ? avg3(o_in1[71:48]) : o_in1[63:56]};
		eb_a <= {o_emph1, o_gray1 ? avg3(o_in1[71:48]) : o_in1[55:48]};
		er_b <= {o_emph1, o_gray1 ? avg3(o_in1[95:72]) : o_in1[95:88]};
		eg_b <= {o_emph1, o_gray1 ? avg3(o_in1[95:72]) : o_in1[87:80]};
		eb_b <= {o_emph1, o_gray1 ? avg3(o_in1[95:72]) : o_in1[79:72]};
	end
	if (os1) o_yx2 <= o_yx1;
	if (os2) begin o_res <= {er_r, eg_r, eb_r, er_q, eg_q, eb_q}; o_yx3 <= o_yx2; end
	if (os3) begin
		out_v  <= 1;
		out_y  <= o_yx3[15:8];
		out_x  <= o_yx3[7:0];
		out_px <= {er_r, eg_r, eb_r, er_q, eg_q, eb_q, o_res};
	end
	if (reset) begin os1 <= 0; os2 <= 0; os3 <= 0; of_wp <= 0; of_rp <= 0; of_n <= 0; end
end

// ==========================================================================================
// the sequencer
// ==========================================================================================
typedef enum bit [6:0] {
	S_IDLE, S_PAL, S_MCLR, S_PALW, S_LINE, S_LOADW,
	S_SCAN,
	S_BI0, S_BI1, S_BI2, S_BI3, S_BI4, S_BOX, S_BC0, S_BC1, S_BC2, S_BCP, S_BPIX,
	S_SF0, S_SF1, S_SF2, S_LF0, S_SC0, S_SC1, S_SC2,
	S_FW,
	S_CD, S_CSW, S_CSW2, S_CEND,
	S_FEND,
	W0, W1, W2, W_R0, W_R1, W_R2, W_S0, W_S0B, W_S1, W_S1B, W_LIST, W_MATCH, W_MW, W_NEXT,
	W_FIN, W_RET,
	E_LW0, E_C0, E_C1, E_M0, E_M1, E_EV, E_V0, E_V1, E_V2, E_V3, E_DIV, E_RES,
	T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, TP, SP0, SP1
} st_t;
st_t st = S_IDLE;

// walk return points
localparam [1:0] WR_BG = 2'd0, WR_BPIX = 2'd1, WR_SPR = 2'd2;
reg   [1:0] wret;

// ---- walk instance ----
reg         i_spr;
reg   [7:0] i_x0;
reg   [3:0] i_n;
reg         i_rv;
reg [RAW-1:0] i_rid;
reg   [2:0] i_offy;
reg         i_hm, i_vm, i_bgp;
reg   [2:0] ox [8];                  // per pixel: column inside the tile
reg   [3:0] ph [8];                  // per pixel: Mesen order at the pixel (0 = BG lookup)
reg   [1:0] kk [8];                  // per pixel: position in the pixel's sprite list
reg   [7:0] i_mask0;
reg         i_group;                 // a BG group walk: a sensitive condition restarts it
reg         restart;

// ---- walk state ----
reg   [1:0] w_kind;
reg  [10:0] w_left;
reg [RSW-1:0] w_ri;
reg  [24:0] w_offq;
reg  [31:0] w_n2, w_i2;
reg   [7:0] unres, mask;
reg  [31:0] r_pay;
reg  [19:0] r_wptr, r_fptr;
reg         r_pre;
reg         w_pre;
reg   [1:0] lph;                     // list phase: 0 frame list next, 1 walk list next, 2 done
reg   [3:0] mk;
reg  [41:0] m_res;                   // result written for the matched pixels

// ---- condition list ----
reg         lm_fl;                   // a frame list (else a walk list)
reg  [20:0] cp;
reg         c_last;
reg  [63:0] c0, c1, c2, c3;
wire  [7:0] c_type = c0[7:0];
wire        c_neg  = c0[8];
wire        c_ign  = c0[9];
wire        c_data = c0[10];
wire  [3:0] c_op   = c0[15:12];
wire  [7:0] c_msk  = c0[23:16];
wire        c_sens = c0[24];
// 8k: a tile condition given as a tile INDEX in a CHR-RAM game. Mesen compares the target's
// TileIndex (AbsoluteTileAddr/16, set for CHR-RAM tiles too) with no fallback; a CHR-RAM
// record's dictionary key holds its fingerprint, not its index (two slots with the same bytes
// share a record), so the index is read from the frame record (T6/T11). The nearby types are
// walked one pixel at a time and evaluated on the single-pixel path, like a rewrite-sensitive
// condition, so only that path needs the read.
wire        c_ixram = !c_data && game_ram && c_type >= 8'd3 && c_type <= 8'd6;
wire        c_ixnear = c_ixram && (c_type == 8'd5 || c_type == 8'd6);
wire        c_memo_ok = c0[63:43] == 21'd0;          // memo index < 2048
reg         c_memo_new;                               // memo missed: store the result
reg   [2:0] ek;
reg         e_res;
reg         e_uni;
reg   [7:0] e_a;
reg  [16:0] dv_rem;
reg   [4:0] dv_i;
reg         rc_pend = 0, cc_pend = 0;

// ---- condition prefetch: while a condition is evaluated, the condition most likely to be
// evaluated next is fetched (list word, then condition record) through the same two caches:
// the next entry of the list; at the end of a frame list, the rule's walk list; at the end of
// the rule's last list, the first list of the next rule, whose rule record is read first and
// kept for W_R0. The sequencer issues a cache or rule request of its own only while no
// prefetch is using that port. ----
localparam [2:0] PF_IDLE = 3'd0, PF_RC = 3'd1, PF_CC = 3'd2, PF_RR1 = 3'd3, PF_RR2 = 3'd4;
reg   [2:0] cf_st = PF_IDLE;
reg [RSW-1:0] pf_ri;                 // rule record read ahead
reg  [72:0] pf_rule;
reg         pf_rule_ok = 0;
reg  [20:0] pf_cp [2];               // per buffer: list position being / been prefetched
reg   [1:0] pf_ok = 0;               // per buffer: nc*[b] hold the condition at pf_cp[b]
reg  [63:0] nc0 [2], nc1 [2], nc2 [2], nc3 [2];
reg         nc_last [2];
reg         pf_b;                    // buffer of the fetch in flight
reg         pf_qv = 0;               // a second fetch queued behind the rule read
reg  [20:0] pf_qcp;
reg  [63:0] lw_q;                    // the last condition-list qword read
reg  [17:0] lw_a;                    // its qword index in the list area
reg         lw_v = 0;
wire        pf_busy = cf_st != PF_IDLE;

// ---- target ----
reg         t_spr;
reg   [7:0] t_y, t_x;
reg   [7:0] t_dy;                    // the line whose data the target reads (8a: 239 for (0,0))
reg   [2:0] t_n;
reg [4*TEW-1:0] t_f;                 // the target's list: {lane, fetch index or added id}
reg   [3:0] t_bgp, t_drawn4;
reg   [1:0] t_j;
reg         t_ln;                    // the entry being matched is an added sprite
reg         t_lk;
reg  [15:0] t_ri;                    // 8k: the target's TileIndex, read from the frame record
reg         t_rx = 0;                // ... and read for the current target

// ---- pipelined tileNearby (TP): one mask pixel issued per clock through the three dependent
// reads (frame map, line span, record), results at stage 6. Within one condition nothing is
// looked up, so each pixel's result is independent and the order does not matter. ----
reg   [7:0] tp_rem;                  // mask pixels not issued yet
reg   [7:0] tp_keep;                 // pixels whose result passed
reg   [6:1] tp_v;                    // stage valid
reg   [2:0] tp_k  [1:6];
reg   [7:0] tp_tx [1:6];
reg   [7:0] tp_ty [1:6];             // per pixel: x + X past the edge wraps into the next row
reg   [7:0] tp_dy [1:2];             // ... the line its data comes from (8a: 239 for (0,0))
reg         tp_dead [1:6];           // already false (outside the screen, no BG record)
reg         tp_lk [3:6];

// ---- pipelined spriteNearby: pass 1 (SP0) streams the mask pixels through the frame map /
// line header / sprite fetch reads and keeps each pixel's sprite list; pass 2 (SP1) streams
// every (pixel, list entry) pair through the sprite record lookups, one per clock. A pixel is
// a hit if any of its entries matches (the serial path stopped at the first match). ----
reg   [7:0] sp_rem;
reg   [2:1] sp_v;
reg   [2:0] sp_k [1:2];
reg   [7:0] sp_tx [1:2];
reg   [7:0] sp_ty [1:2];
reg   [7:0] sp_dy [1:2];             // the line its data comes from (8a: 239 for (0,0))
reg         sp_dead [1:2];
reg   [3:0] sp_np;                   // pixels with a sprite list
reg   [2:0] spl_k [8];
reg   [7:0] spl_tx [8];
reg   [7:0] spl_ty [8];
reg   [7:0] spl_dy [8];
reg [TLW-1:0] spl_fl [8];
reg   [3:0] sp_p;
reg   [1:0] sp_j;
reg   [7:0] sp_hit;
reg   [4:1] si_v;
reg   [2:0] si_k [1:4];
reg   [7:0] si_tx [1:4];
reg   [7:0] si_ty [1:4];
reg   [3:0] si_ph [1:4];
reg         si_dr [1:4];
reg         si_dead [3:4];
reg         si_ln [1:4];             // the entry is an added sprite (8a): its id goes to d_rid
reg [RAW-1:0] si_lid [1:2];

wire  [7:0] cur_x = i_x0 + {5'd0, ek};
wire  [3:0] cur_p = ph[ek];
function precedes(input [7:0] ty, input [7:0] tx, input [3:0] tp, input [7:0] cy,
                input [7:0] ccx, input [3:0] cpp);
	precedes = (ty < cy) || (ty == cy && (tx < ccx || (tx == ccx && tp < cpp)));
endfunction
// a rewrite applies to the target: it is the record being looked up itself (Mesen rewrites
// that record before walking its rules, so an offset-0 nearby condition sees it rewritten; a
// pixel not really looked up has its result discarded), or it was looked up (lk) before it
function seen(input lk, input [7:0] ty, input [7:0] tx, input [3:0] tp, input [7:0] cy,
              input [7:0] ccx, input [3:0] cpp);
	seen = (ty == cy && tx == ccx && tp == cpp) || (lk && precedes(ty, tx, tp, cy, ccx, cpp));
endfunction

// ---- per-line loops ----
reg   [8:0] si;                      // scan: next pixel to issue
reg         sv1 = 0, sv2 = 0;
reg   [7:0] sx1, sx2;
reg   [6:0] nbg;
reg   [5:0] bi;
reg         r_on;
reg   [7:0] r_x0;
reg   [2:0] r_nm1;
reg   [5:0] r_slot;
reg   [3:0] bk;
reg   [3:0] bxi;                     // offx pipeline: next pixel to issue
reg         bv1, bv2;
reg   [2:0] bk1, bk2;
reg         use_c;
reg  [41:0] cached;
reg   [3:0] bq;                      // S_BCP: next pixel to issue
reg         cv1, cv2;
reg   [2:0] ck1, ck2;
reg   [5:0] sf;                      // sprite fetch item (16..31: line 239's)
reg   [6:0] lf;                      // lane item
reg         lsel;                    // ... of line 240
reg   [3:0] sc;
// the sprite-run item: first pixel, pixels, column of the first, list entry code, a lane
reg   [7:0] it_x0;
reg   [3:0] it_nc;
reg   [2:0] it_c0;
reg   [7:0] it_code;
reg         it_lane;

// ---- image slots ----
reg   [7:0] ns;                      // next free slot (127 = full; 127 is the scratch slot)
localparam [7:0] NSLOT8 = NSLOT[7:0];
reg         la_v;
reg  [17:0] la_blk;
reg   [2:0] la_row;
reg   [6:0] la_slot;
reg         scr_v;                   // scratch slot holds (scr_blk, scr_row)
reg  [17:0] scr_blk;
reg   [2:0] scr_row;

// ---- compose: a FRONT stage plans pixel x+1 (reads its line data and sprite entries, works
// out which draw steps it needs) while the BACK stage (the sequencer, S_CD) issues the steps of
// pixel x, one per clock, into the draw pipe. The last step of a pixel carries an end flag; the
// pixel goes to the output FIFO when that step leaves the pipe. ----
localparam [3:0] F_OFF = 4'd0, F_A = 4'd1, F_B = 4'd2, F_C = 4'd3, F_W = 4'd4,
                 F_E0 = 4'd5, F_E1 = 4'd6, F_E2 = 4'd7, F_P = 4'd8;
reg   [3:0] fs = F_OFF;
reg   [7:0] cx;                      // front: pixel being planned
reg  [31:0] f_cpx;                   // front: captured line data of cx
reg [PLW-1:0] clst;
reg   [7:0] f_e;                     // front: the list entry being read
reg  [41:0] f_cbg;
reg   [2:0] cj;
reg         n_ready = 0;             // plan for the back stage
reg  [49:0] n_csteps;
reg  [39:0] f_bgm;                   // front: the pixel's background levels drawn there
reg  [31:0] n_cpx;
reg  [41:0] n_cbg;
reg   [7:0] n_x;
reg         n_last;
reg   [3:0] n_hm, n_vm;
reg   [2:0] n_ox [4];
reg   [1:0] n_sci [4];
reg   [5:0] n_col [4];
reg  [41:0] n_r [4];
reg  [31:0] cpx;                     // back: pixel being drawn
reg  [41:0] cbg;
reg   [7:0] bx;
reg         b_last;
reg   [3:0] e_hm, e_vm;
reg   [2:0] e_ox [4];
reg   [5:0] e_col [4];
reg  [41:0] e_r [4];
reg  [49:0] csteps = 0;
reg  [95:0] acc;
reg         dp_v = 0, dp2_v = 0;     // draw pipe: address in the RAM, data valid
reg         dp_col, dp_hm, dp_vm, dp_end;
reg         dp2_col, dp2_hm, dp2_vm, dp2_end;
reg   [1:0] dp_md = 0, dp2_md = 0;   // blend: 0 Alpha, 1 Add, 2 Subtract (backgrounds only)
reg   [7:0] dp_x, dp2_x;

// lowest index of a set bit (0..7), and of a set bit above position p
function [2:0] lsb8(input [7:0] m);
	casez (m)
		8'b???????1: lsb8 = 3'd0;
		8'b??????10: lsb8 = 3'd1;
		8'b?????100: lsb8 = 3'd2;
		8'b????1000: lsb8 = 3'd3;
		8'b???10000: lsb8 = 3'd4;
		8'b??100000: lsb8 = 3'd5;
		8'b?1000000: lsb8 = 3'd6;
		default:     lsb8 = 3'd7;
	endcase
endfunction

function cmp_op(input [3:0] op, input [31:0] a, input [31:0] b);
	case (op)
		4'd0: cmp_op = a == b;
		4'd1: cmp_op = a != b;
		4'd2: cmp_op = a > b;
		4'd3: cmp_op = a < b;
		4'd4: cmp_op = a <= b;
		4'd5: cmp_op = a >= b;
		default: cmp_op = 1'b0;
	endcase
endfunction

// tile match (hdref.tmatch) of the record in d_key / d_res, as written or REWRITTEN
// (TileIndex replaced by its fallback: test fallback(idx) == T or fallback2 == T)
wire [15:0] fb_i  = d_res[15:0];
wire        fb_v  = d_res[16];
wire [15:0] fb2_i = d_res[32:17];
wire        fb2_v = d_res[33];
wire  [1:0] t_via = d_res[62:61];
wire        t_palok = c_ign || d_key[95:64] == c3[31:0];
wire        t_hi0   = c2[63:16] == 48'd0;
// the record's own index: a captured tile's always fits 16 bits, an added key's body (8a) may
// not, so the whole body is compared (NOTILE, all ones, matches nothing)
wire        t_m_idx = c2 == d_key[63:0] && d_key[63:0] != {64{1'b1}};
wire        t_m_fb  = fb_v && t_hi0 && c2[15:0] == fb_i;
wire        t_m_fb2 = fb2_v && t_hi0 && c2[15:0] == fb2_i;
// ln: the record is an added sprite (8a). In a CHR-RAM game it has no index and no usable
// fingerprint (the converter keeps only additions of the game's kind; hdref gives them NOTILE),
// and an added NOTILE matches no tile condition.
function tmatch(input rw, input ln);
	if (ln && (game_ram || d_key[63:0] == {64{1'b1}}))
		tmatch = 1'b0;
	else if (c_data)
		tmatch = t_palok && (game_ram ? d_key[63:0] : 64'd0) == c2;
	else if (game_ram)                                   // 8k: index read into t_ri, no fallback
		tmatch = t_palok && t_hi0 && c2[15:0] == t_ri;
	else if (rw)
		tmatch = t_palok && (t_m_fb || t_m_fb2);
	else
		tmatch = t_palok && (t_m_idx || t_m_fb);
endfunction

function [3:0] phase_of(input bgp, input [2:0] n, input [1:0] j);
	phase_of = bgp ? {1'b0, n} - {2'b0, j}
	               : {1'b0, n} + {1'b0, n} - {2'b0, j};
endfunction

// a rewrite applies to this tile condition on this target
wire        rw_able = c_sens && !c_data && !game_ram && t_via == 2'd2;

// target index (hdref.at): tile/spriteAtPosition = absolute; nearby = relative, sprite
// offsets mirrored by the evaluated record's flips. 42-bit signed arithmetic.
wire signed [41:0] n_X  = {{10{c1[31]}}, c1[31:0]};
wire signed [41:0] n_Y  = {{10{c1[63]}}, c1[63:32]};
wire signed [41:0] n_Xs = (c_type == 8'd6 && i_hm) ? -n_X : n_X;
wire signed [41:0] n_Ys = (c_type == 8'd6 && i_vm) ? -n_Y : n_Y;
wire signed [41:0] n_at = (c_type == 8'd3 || c_type == 8'd4) ? {10'd0, c3[63:32]} :
                          ((($signed({34'd0, y}) + n_Ys) <<< 8) + $signed({34'd0, cur_x}) + n_Xs);
wire        n_in = n_at >= 0 && n_at < 42'sd61440;
// 8a: a target at pixel (0,0) reads line 239's pixel 0 and line 240's lanes (its looked-up
// state stays pixel (0,0)'s: lk_a, in_win and seen keep the original coordinates)
function [7:0] dline(input [15:0] at);
	dline = (add_en && at == 16'd0) ? 8'd239 : at[15:8];
endfunction
function [7:0] lline(input [15:0] at);
	lline = (add_en && at == 16'd0) ? 8'd240 : at[15:8];
endfunction

// positional checks: the value compared, as u32
wire  [9:0] v_x  = {2'b0, cur_x};
wire  [9:0] v_y  = {2'b0, y};
wire  [9:0] v_ox = i_hm ? {2'b0, cur_x} - 10'd7 + {7'd0, ox[ek]} : {2'b0, cur_x} - {7'd0, ox[ek]};
wire  [9:0] v_oy = i_vm ? {2'b0, y} - 10'd7 + {7'd0, i_offy} : {2'b0, y} - {7'd0, i_offy};
function [31:0] sx10(input [9:0] v);
	sx10 = {{22{v[9]}}, v};
endfunction

// blend one output pixel with a source pixel {A, B, G, R} (hdref.blend_sub, mode 0)
function [23:0] blend_px(input [23:0] o, input [31:0] s);
	reg [8:0] inv;
	reg [16:0] m0, m1, m2;
	begin
		inv = 9'd256 - {1'b0, s[31:24]};
		m0 = inv * o[23:16]; m1 = inv * o[15:8]; m2 = inv * o[7:0];
		if (s[31:24] == 8'd255)
			blend_px = {s[7:0], s[15:8], s[23:16]};
		else if (s[31:24] == 8'd0)
			blend_px = o;
		else
			blend_px = {s[7:0] + m0[15:8], s[15:8] + m1[15:8], s[23:16] + m2[15:8]};
	end
endfunction

// a background's blend (hdref.blend_sub): 0 Alpha as blend_px, 1 Add and 2 Subtract per
// channel, saturating (the source is premultiplied; alpha plays no part in Add / Subtract)
function [7:0] add_sat(input [7:0] a, input [7:0] b);
	reg [8:0] s;
	begin s = {1'b0, a} + {1'b0, b}; add_sat = s[8] ? 8'hFF : s[7:0]; end
endfunction
function [7:0] sub_sat(input [7:0] a, input [7:0] b);
	reg [8:0] s;
	begin s = {1'b0, a} - {1'b0, b}; sub_sat = s[8] ? 8'h00 : s[7:0]; end
endfunction
function [23:0] blend_m(input [23:0] o, input [31:0] s, input [1:0] md);
	case (md)
		2'd1:    blend_m = {add_sat(o[23:16], s[7:0]), add_sat(o[15:8], s[15:8]),
		                    add_sat(o[7:0], s[23:16])};
		2'd2:    blend_m = {sub_sat(o[23:16], s[7:0]), sub_sat(o[15:8], s[15:8]),
		                    sub_sat(o[7:0], s[23:16])};
		default: blend_m = blend_px(o, s);
	endcase
endfunction

// the draw pipe's result: the accumulator after the step leaving the pipe
// (sub-pixel (r, c) <- image row (vm ? 1-r : r), half (hm ? 1-c : c))
wire [95:0] acc_n = dp2_col ? {4{pal_col}} : {
	blend_m(acc[95:72], dp2_vm ? (dp2_hm ? im0_q[31:0] : im0_q[63:32])
	                           : (dp2_hm ? im1_q[31:0] : im1_q[63:32]), dp2_md),
	blend_m(acc[71:48], dp2_vm ? (dp2_hm ? im0_q[63:32] : im0_q[31:0])
	                           : (dp2_hm ? im1_q[63:32] : im1_q[31:0]), dp2_md),
	blend_m(acc[47:24], dp2_vm ? (dp2_hm ? im1_q[31:0] : im1_q[63:32])
	                           : (dp2_hm ? im0_q[31:0] : im0_q[63:32]), dp2_md),
	blend_m(acc[23:0],  dp2_vm ? (dp2_hm ? im1_q[63:32] : im1_q[31:0])
	                           : (dp2_hm ? im0_q[63:32] : im0_q[31:0]), dp2_md)};

// a pixel's background levels drawn: bit L of the group word's byte L, at the pixel's column
function [39:0] bg_bits(input [319:0] q, input [2:0] j);
	integer l;
	for (l = 0; l < 40; l = l + 1) bg_bits[l] = q[8 * l + {29'd0, j}];
endfunction

// the compose steps of a pixel from plan_steps' ten and its background levels: 0 backdrop,
// 1-10 levels 0-9 (layer 0), 11-14 sprites behind (list 3..0), 15-24 levels 10-19, 25 BG
// tile, 26-35 levels 20-29, 36-39 sprites in front (list 3..0), 40-49 levels 30-39
function [49:0] steps50(input [9:0] m, input [39:0] b);
	steps50 = {b[39:30], m[9:6], b[29:20], m[5], b[19:10], m[4:1], b[9:0], m[0]};
endfunction

// draw steps of a pixel (bit 0 backdrop, 1..4 behind sprites, 5 BG tile, 6..9 front sprites)
function [9:0] plan_steps(input [PLW-1:0] l, input [41:0] bg, input [31:0] px,
                          input [167:0] rr, input [7:0] scs, input ro);
	reg [9:0] m;
	integer k;
	begin
		m = 10'd1;
		for (k = 0; k < 4; k = k + 1)
			// drawn (looked-up) entries that paint something: a found tile that is not fully
			// transparent, else the sprite's own colour where it is opaque (an added sprite has
			// none). Behind-background ones are step 1 + (3 - j), front ones step 6 + (3 - j).
			if (l[PL_D + k] && (rr[42 * k + 41] ? !rr[42 * k + 40] : scs[2 * k +: 2] != 2'd0)) begin
				if (l[PL_B + k]) m[4 - k] = 1'b1;
				else m[9 - k] = 1'b1;
			end
		if (bg[41] ? !bg[40] : (ro && px[9:8] != 2'd0)) m[5] = 1'b1;
		plan_steps = m;
	end
endfunction

// compose: the step about to be issued (first set bit of csteps, numbered as steps50)
reg   [5:0] cst;
always @* begin
	cst = 6'd63;
	for (integer b = 49; b >= 0; b = b - 1) if (csteps[b]) cst = b[5:0];
end
wire        cst_bd   = cst == 6'd0;
wire        cst_tile = cst == 6'd25;
wire        cst_frt  = cst >= 6'd36 && cst <= 6'd39;
wire        cst_beh  = cst >= 6'd11 && cst <= 6'd14;
wire        cst_bg   = !cst_bd && !cst_tile && !cst_frt && !cst_beh && cst != 6'd63;
wire  [1:0] cst_j    = cst_frt ? 2'(6'd39 - cst) : 2'(6'd14 - cst);
function [5:0] step_lvl(input [5:0] s);   // background level of a compose step
	step_lvl = (s <= 6'd10) ? s - 6'd1 : (s <= 6'd24) ? s - 6'd5 :
	           (s <= 6'd35) ? s - 6'd6 : s - 6'd10;
endfunction
wire  [5:0] cst_l    = step_lvl(cst);
// the step after this one in the same pixel (the group entry it needs is read while this one
// issues, so a background step costs one clock unless it is a pixel's first step)
wire [49:0] cst_rest = csteps & ~(50'd1 << cst);
reg   [5:0] cst2;
always @* begin
	cst2 = 6'd63;
	for (integer b = 49; b >= 0; b = b - 1) if (cst_rest[b]) cst2 = b[5:0];
end
// a background step's group entry: be_q holds bge[be_tag]; the step may issue when be_tag is
// its own {level, group}. The pixel's cell is A or B, its column ph + j; a cell becomes a
// lookup result {found, not transparent, uncached, slot (the row when uncached), block}, as
// tiles draw them
reg  [10:0] be_tag = 0;
wire        be_hit = be_tag == {cst_l, bx[7:3]};
wire  [3:0] be_sum = {1'b0, be_q[54:52]} + {1'b0, bx[2:0]};
wire [25:0] be_ref = be_sum[3] ? be_q[51:26] : be_q[25:0];
wire [41:0] be_r   = {1'b1, 1'b0, be_ref[25], be_ref[25] ? {4'd0, be_q[57:55]} : be_ref[24:18],
                      14'd0, be_ref[17:0]};
wire [41:0] cst_r = cst_tile ? cbg : cst_bg ? be_r : e_r[cst_j];
// finished pixels already on their way to the output FIFO (a pixel's last step may only issue
// while one more still fits)
wire [OF_LOG2+1:0] of_ahead = {1'b0, of_n} + {{(OF_LOG2+1){1'b0}}, of_push}
                            + {{(OF_LOG2+1){1'b0}}, dp_v & dp_end}
                            + {{(OF_LOG2+1){1'b0}}, dp2_v & dp2_end};
// the back stage issues the step cst this clock (S_CD's conditions): a colour step, or an
// image step whose row is cached or already in the scratch slot, and (for a pixel's last step)
// room in the output FIFO
wire        cst_col     = cst_bd || (!cst_bg && !cst_r[41]);
wire        cst_scrmiss = !cst_col && cst_r[39] &&
                          !(scr_v && scr_blk == cst_r[17:0] && scr_row == cst_r[34:32]);
wire        cst_last    = (csteps & (csteps - 50'd1)) == 50'd0;
wire        cst_issue   = st == S_CD && csteps != 50'd0 && !(cst_bg && !be_hit) && !cst_scrmiss &&
                          (!cst_last || of_ahead < OF_D[OF_LOG2+1:0]);
assign be_ra = (cst_issue && !cst_last) ? {step_lvl(cst2), bx[7:3]} : {cst_l, bx[7:3]};
always @(posedge clk) be_tag <= be_ra;

integer zi;

// one pixel of the 8-pixel BG cache rule (hdref.render): wr_q holds pixel k's walk result
task automatic cache_step(input [2:0] k);
	reg [7:0] xx;
	reg       uce;
	xx  = i_x0 + {5'd0, k};
	uce = ((l_finex + xx[2:0]) == 3'd0) ? 1'b0 : use_c;
	bgw_e <= 1; bgw_a <= xx;
	lkw_a <= {y, xx[7:1]};
	if (xx[0]) lkw1_e <= 1; else lkw0_e <= 1;
	if (uce) begin
		bgw_d <= cached; lkw_d <= 1'b0;
	end else begin
		bgw_d <= wr_q[41:0]; lkw_d <= 1'b1;
		if (!wr_q[42] && cache_en) begin cached <= wr_q[41:0]; use_c <= 1; end
		else use_c <= 0;
	end
endtask

// compose front stage, one clock (see the compose registers). Reads of pixel cx+1's line data
// are issued while cx is captured, so with no sprite entries the front plans a pixel every two
// clocks; it waits only while the back stage has not taken the previous plan.
task automatic compose_front;
	case (fs)
		F_A: begin lp_a <= cx[7:1]; pl_a <= cx; bg_a <= cx; bm_a <= cx[7:3]; fs <= F_B; end
		F_B: fs <= F_C;
		F_C: begin : fcap
			reg [7:0] nx;
			nx = cx + 8'd1;
			f_cpx <= pword(lp_q, cx);
			clst  <= pl_q;
			f_cbg <= bg_q;
			f_bgm <= bg_bits(bm_q, cx[2:0]);
			cj    <= 0;
			if (cx != xr) begin lp_a <= nx[7:1]; pl_a <= nx; bg_a <= nx; bm_a <= nx[7:3]; end
			fs <= (pl_q[PL_N +: 3] == 3'd0) ? F_P : F_W;
		end
		F_W: if (!n_ready) fs <= F_E0;          // the entries go straight into the plan
		F_E0: begin
			if (cj == clst[PL_N +: 3]) fs <= F_P;
			else begin
				f_e  <= clst[8 * cj[1:0] +: 8];
				fi_a <= clst[8 * cj[1:0] +: 5];
				sr_a <= {cx, cj[1:0]};
				fs <= F_E1;
			end
		end
		F_E1: fs <= F_E2;
		F_E2: begin : fentry
			reg  [1:0] s;
			reg [LW-1:0] l;
			if (f_e[7:6] == 2'd0) begin
				// a sprite fetch (16..31: line 239's, with its palette RAM)
				s = sci_of(fi_q[15:8], fi_q[7:0], fi_q[26:19], cx);
				n_hm[cj[1:0]]  <= fi_q[30];
				n_vm[cj[1:0]]  <= fi_q[31];
				n_ox[cj[1:0]]  <= cx[2:0] - fi_q[21:19];
				n_sci[cj[1:0]] <= s;
				n_col[cj[1:0]] <= pal6(f_e[4] ? l_pal239 : l_pal, {1'b1, fi_q[28:27], s});
			end else begin
				// an added sprite (8a): its source's flips, column x - x0 + the source's, no colour
				l = (f_e[7:6] == 2'd1) ? ln_cur[LW * f_e[5:0] +: LW] : ln_240[LW * f_e[5:0] +: LW];
				n_hm[cj[1:0]]  <= l[LW-4];
				n_vm[cj[1:0]]  <= l[LW-3];
				n_ox[cj[1:0]]  <= l[RAW+11 +: 3] + 3'(cx - l[7:0]);
				n_sci[cj[1:0]] <= 2'd0;
				n_col[cj[1:0]] <= 6'd0;
			end
			n_r[cj[1:0]]   <= sr_q;
			cj <= cj + 3'd1;
			fs <= F_E0;
		end
		F_P: if (!n_ready) begin
			n_csteps <= steps50(plan_steps(clst, f_cbg, f_cpx, {n_r[3], n_r[2], n_r[1], n_r[0]},
			                               {n_sci[3], n_sci[2], n_sci[1], n_sci[0]}, render_orig),
			                    f_bgm);
			n_cpx  <= f_cpx;
			n_cbg  <= f_cbg;
			n_x    <= cx;
			n_last <= cx == xr;
			n_ready <= 1;
			if (cx == xr) fs <= F_OFF;
			else begin cx <= cx + 8'd1; fs <= F_C; end
		end
		default: ;
	endcase
endtask

// condition prefetch of list position nxt: its list word (unless it is the one already read),
// then its condition record
task automatic pf_fetch(input [20:0] nxt, input b);
	pf_cp[b] <= nxt; pf_ok[b] <= 1'b0; pf_b <= b;
	if (lw_v && lw_a == nxt[20:3]) begin
		cc_ci      <= nxt[2] ? lw_q[55:32] : lw_q[23:0];
		nc_last[b] <= nxt[2] ? lw_q[63] : lw_q[31];
		cc_req <= 1; cf_st <= PF_CC;
	end else begin
		rc_req <= 1; rc_addr <= clist_base + {7'd0, nxt[20:3]}; cf_st <= PF_RC;
	end
endtask

// compose back stage: take the front's plan
task automatic compose_load;
	csteps <= n_csteps; cpx <= n_cpx; cbg <= n_cbg; bx <= n_x; b_last <= n_last;
	e_hm <= n_hm; e_vm <= n_vm;
	for (integer k = 0; k < 4; k = k + 1) begin
		e_ox[k] <= n_ox[k]; e_col[k] <= n_col[k]; e_r[k] <= n_r[k];
	end
	n_ready <= 0;
endtask

always @(posedge clk) begin
	done <= 0;
	pal_kick <= 0; ld_kick <= 0; pp_kick <= 0; bf_kick <= 0; bl_kick <= 0;
	rc_req <= 0; rc_byp <= 0; cc_req <= 0;
	if_push <= 0;
	of_push <= 0;
	wrw_e <= 0; bgw_e <= 0; srw_e <= 0; blw_e <= 0; lkw0_e <= 0; lkw1_e <= 0; dmw_e <= 0;

	// a background cell's image slot (the background block asks, ba_req): from the line's slots
	// as the walks take theirs (W_MATCH, never on the same clock), uncached once they run out
	ba_done <= 0;
	if (ba_req && !ba_done && st != W_MATCH && st != S_CSW && !if_push) begin
		if (ns >= NSLOT8) begin
			ba_unc <= 1; ba_done <= 1;
		end else if (ifq_n < 5'd14) begin
			if_push <= 1; if_d <= {ns[6:0], ba_blk, ba_row};
			ba_unc <= 0; ba_slot <= ns[6:0]; ba_done <= 1;
			ns <= ns + 8'd1;
		end
	end

	// draw pipe (compose): blend the data that arrives; a pixel's last step hands it to the
	// output FIFO (the first step of every pixel is the backdrop, which overwrites acc)
	dp2_v <= dp_v; dp2_col <= dp_col; dp2_hm <= dp_hm; dp2_vm <= dp_vm; dp2_lo <= dp_lo;
	dp2_md <= dp_md;
	dp2_end <= dp_end; dp2_x <= dp_x;
	dp_v <= 0;
	if (dp2_v) begin
		acc <= acc_n;
		if (dp2_end) begin of_push <= 1; of_d <= {l_emph, l_gray, y, dp2_x, acc_n}; end
	end

	// compose front stage
	compose_front();

	// condition prefetch (started in E_C1)
	case (cf_st)
		PF_RC: if (rc_ack) begin
			lw_q <= rc_q; lw_a <= pf_cp[pf_b][20:3]; lw_v <= 1;
			cc_ci         <= pf_cp[pf_b][2] ? rc_q[55:32] : rc_q[23:0];
			nc_last[pf_b] <= pf_cp[pf_b][2] ? rc_q[63] : rc_q[31];
			cc_req <= 1; cf_st <= PF_CC;
		end
		PF_CC: if (cc_ack) begin
			nc0[pf_b] <= cc_q[63:0];    nc1[pf_b] <= cc_q[127:64];
			nc2[pf_b] <= cc_q[191:128]; nc3[pf_b] <= cc_q[255:192];
			pf_ok[pf_b] <= 1'b1;
			if (pf_qv) begin pf_qv <= 0; pf_fetch(pf_qcp, 1'b1); end
			else cf_st <= PF_IDLE;
		end
		PF_RR1: cf_st <= PF_RR2;
		PF_RR2: begin : pfrr
			reg [20:0] nxt;
			pf_rule <= q_rule; pf_rule_ok <= 1;
			nxt = (q_rule[51:32] != 20'd0) ? {1'b0, q_rule[51:32]} : {1'b0, q_rule[71:52]};
			if (nxt != 21'd0) pf_fetch(nxt, 1'b0);
			else if (pf_qv) begin pf_qv <= 0; pf_fetch(pf_qcp, 1'b1); end
			else cf_st <= PF_IDLE;                      // a rule with no condition
		end
		default: ;
	endcase

	case (st)
	// ======================================================================================
	// frame
	// ======================================================================================
	S_IDLE: if (start) begin
		busy  <= 1;
		fbank <= bank;
		f_ovf <= frame_ovf;
		y     <= ov_top;
		pf_ok <= 0; lw_v <= 0; pf_rule_ok <= 0;  // the lists and rules may be new this frame
		pf_qv <= 0;
		st    <= S_PAL;
	end
	S_PAL: begin
		frame <= d_frame;                    // d_bank is the new bank now
		if (has_pal) pal_kick <= 1;          // the palette and the backgrounds load while
		bf_kick <= 1;                        //   the memo is swept
		mc <= 0;
		st <= S_MCLR;
	end
	// 2048 entries: each memo entry takes resolve's seed (8i: the rewrite-sensitive conditions
	// background selection evaluated, else invalid). sd_a = mc, sd_q one clock later.
	S_MCLR: begin
		if (mc != 12'd0 && !mc[11]) begin dm_a <= 11'(mc - 12'd1); dmw_d <= sd_q; dmw_e <= 1; end
		if (mc == 12'd2048) begin
			dm_a <= 11'd2047; dmw_d <= sd_q; dmw_e <= 1;
			mc <= 12'd2049;
		end else if (mc != 12'd2049) mc <= mc + 12'd1;
		else if (bs == B_IDLE && !bf_kick) st <= has_pal ? S_PALW : S_LINE;
	end
	S_PALW: if (!pal_kick && !pal_go && pal_rx == 3'd4) st <= S_LINE;

	S_LINE: begin
		if (!win_ok || y > ybot) st <= S_FEND;
		else if (line_ok) begin
			ld_kick <= 1; pp_kick <= 1;
			ns <= 0; la_v <= 0; scr_v <= 0;
			st <= S_LOADW;
		end
	end
	S_LOADW: if (!ld_kick && !ld_go && ld_rx == ld_n && !pp_kick && !pp_on) begin
		bl_kick <= 1;                        // backgrounds: the line header (t, finex) is in
		si <= {1'b0, ov_left}; sv1 <= 0; sv2 <= 0;
		r_on <= 0; nbg <= 0; use_c <= 0;
		st <= S_SCAN;
	end

	// ---- BG instances: runs of up to 8 window pixels with one span ----------------------
	S_SCAN: begin
		if (si <= {1'b0, xr}) begin
			lp_a <= si[7:1];
			sv1 <= 1; sx1 <= si[7:0];
			si <= si + 9'd1;
		end else sv1 <= 0;
		sv2 <= sv1; sx2 <= sx1;
		if (sv2) begin : scan_px
			reg [31:0] w;
			reg  [7:0] b0;
			reg        vbg;
			w   = pword(lp_q, sx2[7:0]);
			b0  = w[7:0];
			// slot 62: line 239's span, pixel (0,0) on line 0 (8a)
			vbg = b0[5:0] == 6'd62 || (b0[5:0] != 6'd63 && b0[5:0] < l_nspan);
			// a pixel with no BG record draws no tile; a cached / unwalked one is not LOOKED UP
			bgw_e <= 1; bgw_a <= sx2; bgw_d <= 42'd0;
			lkw_a <= {y, sx2[7:1]}; lkw_d <= 1'b0;
			if (sx2[0]) lkw1_e <= 1; else lkw0_e <= 1;
			if (r_on && vbg && b0[5:0] == r_slot && r_nm1 != 3'd7)
				r_nm1 <= r_nm1 + 3'd1;
			else begin
				if (r_on) begin
					blw_e <= 1; blw_a <= nbg[5:0]; blw_d <= {r_x0, r_nm1, r_slot};
					nbg <= nbg + 7'd1;
				end
				r_on <= vbg; r_x0 <= sx2; r_nm1 <= 0; r_slot <= b0[5:0];
			end
		end
		if (!sv1 && !sv2 && si > {1'b0, xr}) begin
			if (r_on) begin
				blw_e <= 1; blw_a <= nbg[5:0]; blw_d <= {r_x0, r_nm1, r_slot};
				nbg <= nbg + 7'd1;
			end
			bi <= 0;
			st <= S_BI0;
		end
	end

	S_BI0: begin
		if ({1'b0, bi} == nbg) begin sf <= 0; st <= S_SF0; end
		else begin bl_a <= bi; st <= S_BI1; end
	end
	S_BI1: st <= S_BI2;
	S_BI2: begin
		i_x0 <= bl_q[16:9];
		i_n  <= {1'b0, bl_q[8:6]} + 4'd1;
		// slot 62 is line 239's span of its pixel 0 (8a); its tile row is in spoff[62]
		d_y <= (bl_q[5:0] == 6'd62) ? 8'd239 : y;
		d_slot <= (bl_q[5:0] == 6'd62) ? px239[5:0] : bl_q[5:0];
		so_a <= bl_q[5:0];
		st <= S_BI3;
	end
	S_BI3: st <= S_BI4;
	S_BI4: begin
		i_rv <= d_span[RAW]; i_rid <= d_span[RAW-1:0];
		i_offy <= so_q;
		i_spr <= 0; i_hm <= 0; i_vm <= 0; i_bgp <= 0;
		for (zi = 0; zi < 8; zi = zi + 1) begin ph[zi] <= 0; kk[zi] <= 0; end
		bxi <= 0; bv1 <= 0; bv2 <= 0;
		st <= S_BOX;
	end
	// offx of each pixel: b1[4:2] of the frame-record word
	S_BOX: begin
		if (bxi < i_n) begin
			lp_a <= 7'((i_x0 + {4'd0, bxi}) >> 1);
			bv1 <= 1; bk1 <= bxi[2:0];
			bxi <= bxi + 4'd1;
		end else bv1 <= 0;
		bv2 <= bv1; bk2 <= bk1;
		if (bv2) begin : box_px
			reg  [7:0] xx;
			reg [31:0] w;
			xx = i_x0 + {5'd0, bk2};
			w  = pword(lp_q, xx);
			ox[bk2] <= w[12:10];
		end
		if (!bv1 && !bv2 && bxi >= i_n) begin
			i_mask0 <= (8'd1 << i_n) - 8'd1;
			i_group <= i_n > 4'd1;
			wret <= WR_BG;
			st <= W0;
		end
	end
	// a BG group walk met a rewrite-sensitive condition: redo the instance one pixel at a
	// time, deciding the cache rule BEFORE each walk (a pixel served from the cache is not
	// looked up, so nothing may be evaluated for it - hdref.render)
	S_BPIX: begin : bpix
		reg [7:0] xx;
		xx = i_x0 + {4'd0, bk};
		if (bk == i_n) begin bi <= bi + 6'd1; st <= S_BI0; end
		else if (use_c && (l_finex + xx[2:0]) != 3'd0) begin
			bgw_e <= 1; bgw_a <= xx; bgw_d <= cached;
			bk <= bk + 4'd1;                     // the bitmap bit stays 0 (from the scan)
		end else begin
			i_mask0 <= 8'd1 << bk[2:0];
			i_group <= 0;
			wret <= WR_BPIX;
			st <= W0;
		end
	end
	// the 8-pixel BG cache rule (hdref.render). Group walk: S_BCP, one pixel per clock (wr is
	// read two clocks after its address). Per-pixel restart: one pixel through BC0..BC2.
	S_BCP: begin
		if (bq < i_n) begin
			wr_a <= bq[2:0];
			cv1 <= 1; ck1 <= bq[2:0];
			bq <= bq + 4'd1;
		end else cv1 <= 0;
		cv2 <= cv1; ck2 <= ck1;
		if (cv2) cache_step(ck2);
		if (!cv1 && !cv2 && bq >= i_n) begin bi <= bi + 6'd1; st <= S_BI0; end
	end
	S_BC0: begin wr_a <= bk[2:0]; st <= S_BC1; end
	S_BC1: st <= S_BC2;
	S_BC2: begin
		cache_step(bk[2:0]);
		bk <= bk + 4'd1;
		st <= S_BPIX;
	end

	// ---- sprite runs: per item (a sprite fetch - 16..31 line 239's, on line 0 when vz - or a
	// lane, 8a), its pixels whose list draws it, cut into runs at the gaps ------------------
	S_SF0: begin
		if (sf == (vz ? 6'd32 : 6'd16)) begin lf <= 0; lsel <= 0; st <= S_LF0; end
		else begin
			fi_a <= sf[4:0];
			d_y <= sf[4] ? 8'd239 : y; d_fetch <= sf[3:0];
			st <= S_SF1;
		end
	end
	S_SF1: st <= S_SF2;
	S_SF2: begin
		if (!fi_q[32] || {1'b0, sf[3:0]} >= (sf[4] ? v_nfet : l_nfet)) begin
			sf <= sf + 6'd1; st <= S_SF0;
		end else begin
			it_x0 <= fi_q[26:19]; it_nc <= 4'd8; it_c0 <= 3'd0; it_code <= {3'd0, sf[4:0]};
			it_lane <= 0;
			sc <= 0; i_n <= 0;
			i_spr <= 1; i_rv <= d_spr[RAW]; i_rid <= d_spr[RAW-1:0];
			i_vm <= fi_q[31]; i_hm <= fi_q[30]; i_bgp <= fi_q[29];
			i_offy <= fi_q[18:16];
			st <= S_SC0;
		end
	end
	// the lanes of line y, then (pixel (0,0) on line 0, vz) the lanes of line 240: an added
	// sprite is a copy of its source record (flips, background priority, tile row, column)
	S_LF0: begin : lane_item
		reg [LW-1:0] l;
		l = lsel ? ln_240[LW * lf[5:0] +: LW] : ln_cur[LW * lf[5:0] +: LW];
		if (lf >= (lsel ? n_240 : n_cur)) begin
			if (!lsel && vz) begin lsel <= 1; lf <= 0; end
			else st <= S_FW;
		end else begin
			it_x0 <= l[7:0]; it_nc <= {1'b0, l[10:8]} + 4'd1; it_c0 <= l[RAW+11 +: 3];
			it_code <= {lsel ? 2'd2 : 2'd1, lf[5:0]};
			it_lane <= 1;
			sc <= 0; i_n <= 0;
			i_spr <= 1; i_rv <= 1; i_rid <= l[11 +: RAW];
			i_vm <= l[LW-3]; i_hm <= l[LW-4]; i_bgp <= l[LW-2];
			i_offy <= l[RAW+14 +: 3];
			st <= S_SC0;
		end
	end
	S_SC0: begin : spr_col
		reg [8:0] xx;
		xx = {1'b0, it_x0} + {5'd0, sc};
		if (sc == it_nc || xx[8] || !in_win(y, xx[7:0])) begin
			// end of the item or a gap: walk the run collected so far
			if (i_n != 4'd0) begin
				i_mask0 <= (8'd1 << i_n) - 8'd1;
				i_group <= 0;
				wret <= WR_SPR;
				st <= W0;
			end else if (sc == it_nc || xx[8]) begin
				if (it_lane) begin lf <= lf + 7'd1; st <= S_LF0; end
				else begin sf <= sf + 6'd1; st <= S_SF0; end
			end else sc <= sc + 4'd1;
		end else begin
			pl_a <= xx[7:0];
			st <= S_SC1;
		end
	end
	S_SC1: st <= S_SC2;
	S_SC2: begin : spr_member
		reg       mem;
		reg [1:0] j;
		reg [8:0] xx;
		// a member is an entry of this pixel's list that hdref.render looks up (draws)
		mem = 0; j = 0;
		for (zi = 3; zi >= 0; zi = zi - 1)
			if (pl_q[PL_D + zi] && pl_q[8 * zi +: 8] == it_code) begin mem = 1; j = zi[1:0]; end
		xx = {1'b0, it_x0} + {5'd0, sc};
		if (mem) begin
			if (i_n == 4'd0) i_x0 <= xx[7:0];
			ox[i_n[2:0]] <= it_c0 + sc[2:0];
			kk[i_n[2:0]] <= j;
			ph[i_n[2:0]] <= 4'd1 + phase_of(i_bgp, pl_q[PL_N +: 3], j);
			i_n <= i_n + 4'd1;
			sc <= sc + 4'd1;
			st <= S_SC0;
		end else if (i_n != 4'd0) begin
			// a gap: walk the run, then continue at this column (sc is not advanced, so it is
			// tested again - and found absent again - after the walk)
			i_mask0 <= (8'd1 << i_n) - 8'd1;
			i_group <= 0;
			wret <= WR_SPR;
			st <= W0;
		end else begin
			sc <= sc + 4'd1;
			st <= S_SC0;
		end
	end

	// ---- wait for the image rows, then compose ------------------------------------------
	// the image rows are in and the line's backgrounds prepared (bl_kick was sent at LOAD)
	S_FW: if (!if_push && ifq_n == 5'd0 && img_out == 7'd0 && bs == B_IDLE && !ba_req) begin
		cx <= ov_left;
		fs <= F_A;
		st <= S_CD;
	end

	// back stage: one draw step per clock (a pixel's first background step two: its group entry
	// is read first); a pixel's last step loads the next plan
	S_CD: begin
		if (csteps == 50'd0) begin
			if (n_ready) compose_load();
		end else if (cst_bg && !be_hit) begin
			// a pixel's first background step: its group entry arrives next clock
		end else begin : cstep
			reg [41:0] r;
			reg        col;
			reg  [5:0] ci;
			reg  [2:0] o;
			reg        hm, vm;
			reg        last;
			r = cst_r;
			if (cst_bd) begin col = 1; ci = cpx[21:16]; end                 // backdrop: b2
			else if (cst_tile) begin col = !cbg[41]; ci = cpx[29:24]; end    // tile: b3
			else if (cst_bg) begin col = 0; ci = cpx[21:16]; end             // background
			else begin col = !r[41]; ci = e_col[cst_j]; end
			if (cst_tile) begin o = cpx[12:10]; hm = 0; vm = 0; end
			else if (cst_bg) begin o = be_sum[2:0]; hm = 0; vm = 0; end
			else begin o = e_ox[cst_j]; hm = e_hm[cst_j]; vm = e_vm[cst_j]; end
			last = (csteps & (csteps - 50'd1)) == 50'd0;
			if (!col && r[39] && !(scr_v && scr_blk == r[17:0] && scr_row == r[34:32])) begin
				// uncached tile (the line's slots ran out): fetch it into the scratch slot first
				st <= S_CSW;
			end else if (!last || of_ahead < OF_D[OF_LOG2+1:0]) begin
				csteps[cst] <= 1'b0;
				pc_a <= ci; dp_lo <= ci[0];
				im_a <= {(!col && r[39]) ? 7'd127 : r[38:32], hm ? 3'd7 - o : o};
				dp_v <= 1; dp_col <= col; dp_hm <= hm; dp_vm <= vm;
				dp_md <= cst_bg ? be_q[59:58] : 2'd0;
				dp_end <= last; dp_x <= bx;
				if (last) begin
					if (b_last) st <= S_CEND;
					else if (n_ready) compose_load();
				end
			end
		end
	end
	// an uncached tile: its two output rows into slot 127 (an uncached result keeps the tile
	// row in its slot field), once nothing in flight can still be reading or writing slot 127
	S_CSW: if (!if_push && ifq_n == 5'd0 && img_out == 7'd0 && !dp_v && !dp2_v) begin
		if_push <= 1;
		if_d <= {7'd127, cst_r[17:0], cst_r[34:32]};
		scr_v <= 1; scr_blk <= cst_r[17:0]; scr_row <= cst_r[34:32];
		st <= S_CSW2;
	end
	S_CSW2: if (!if_push && ifq_n == 5'd0 && img_out == 7'd0) st <= S_CD;
	// the line's last pixel is in the pipe: wait for it to reach the FIFO
	S_CEND: if (!dp_v && !dp2_v) begin
		y <= y + 8'd1;
		st <= S_LINE;
	end

	S_FEND: if (out_idle && !pf_busy) begin
		busy <= 0; done <= 1;
		st <= S_IDLE;
	end

	// ======================================================================================
	// walk (hwmodel.walk with the RESOLVE-filtered list, or the DDR3 list on the slow path)
	// ======================================================================================
	W0: begin q_rid <= i_rid; st <= W1; end
	W1: st <= W2;
	W2: begin
		unres <= i_mask0;
		restart <= 0;
		w_kind <= q_rec[48:47];
		w_left <= q_rec[45:35];
		w_ri   <= q_rec[34:25];
		w_offq <= q_rec[24:0];
		if (!i_rv || f_ovf || q_rec[48:47] == 2'd0 || q_rec[48:47] == 2'd3) begin
			w_pre <= 0; mk <= 0; st <= W_FIN;
		end else if (q_rec[48:47] == 2'd1) begin
			w_pre <= q_rec[46];
			if (q_rec[45:35] == 11'd0) begin mk <= 0; st <= W_FIN; end
			else st <= W_R0;
		end else st <= W_S0;
	end
	// (not while a prefetch is reading a rule record through the same port)
	W_R0: if (cf_st != PF_RR1 && cf_st != PF_RR2) begin
		if (pf_rule_ok && pf_ri == w_ri) begin
			// read ahead while the previous rule's last condition was evaluated
			r_pay  <= pf_rule[31:0];
			r_fptr <= pf_rule[51:32];
			r_wptr <= pf_rule[71:52];
			r_pre  <= pf_rule[72];
			mask   <= unres;
			lph    <= 0;
			st     <= W_LIST;
		end else begin
			q_ri <= w_ri; st <= W_R1;
		end
	end
	W_R1: st <= W_R2;
	W_R2: begin
		r_pay  <= q_rule[31:0];
		r_fptr <= q_rule[51:32];
		r_wptr <= q_rule[71:52];
		r_pre  <= q_rule[72];
		mask   <= unres;
		lph    <= 0;
		st     <= W_LIST;
	end
	// slow path: the record's whole list from DDR3 (n, then 16-byte rules)
	W_S0: begin
		if (!rc_pend) begin if (!pf_busy) begin rc_req <= 1; rc_pend <= 1; rc_addr <= rules_base + w_offq; end end
		else if (rc_ack) begin
			rc_pend <= 0;
			w_n2 <= rc_q[31:0];
			if (rc_q[31:0] == 32'd0) begin w_pre <= 0; mk <= 0; st <= W_FIN; end
			else st <= W_S0B;
		end
	end
	W_S0B: begin                                       // the last rule's prefix flag
		if (!rc_pend) begin
			if (!pf_busy) begin rc_req <= 1; rc_pend <= 1; end
			rc_addr <= rules_base + w_offq + 25'd2 + {w_n2[23:0] - 24'd1, 1'b0};
		end else if (rc_ack) begin
			rc_pend <= 0;
			w_pre <= rc_q[57];
			w_i2 <= 0;
			st <= W_S1;
		end
	end
	W_S1: begin
		if (!rc_pend) begin
			if (!pf_busy) begin rc_req <= 1; rc_pend <= 1; end
			rc_addr <= rules_base + w_offq + 25'd1 + {w_i2[23:0], 1'b0};
		end else if (rc_ack) begin
			rc_pend <= 0;
			r_pay <= rc_q[31:0]; r_fptr <= rc_q[51:32];
			st <= W_S1B;
		end
	end
	W_S1B: begin
		if (!rc_pend) begin
			if (!pf_busy) begin rc_req <= 1; rc_pend <= 1; end
			rc_addr <= rules_base + w_offq + 25'd2 + {w_i2[23:0], 1'b0};
		end else if (rc_ack) begin
			rc_pend <= 0;
			r_wptr <= rc_q[19:0]; r_pre <= rc_q[57];
			mask <= unres;
			lph <= 0;
			st <= W_LIST;
		end
	end
	W_LIST: begin
		case (lph)
			2'd0: begin
				lph <= 1;
				if (r_fptr != 20'd0) begin lm_fl <= 1; cp <= {1'b0, r_fptr}; st <= E_LW0; end
			end
			2'd1: begin
				lph <= 2;
				if (mask != 8'd0 && r_wptr != 20'd0) begin
					lm_fl <= 0; cp <= {1'b0, r_wptr}; st <= E_LW0;
				end
			end
			default: begin
				if (mask != 8'd0) st <= W_MATCH;
				else st <= W_NEXT;
			end
		endcase
	end
	// a rule took the pixels in `mask`: one image slot for (block, tile row), then results
	W_MATCH: begin : wmatch
		reg trans;
		trans = r_pay == 32'hFFFFFFFF;
		m_res[41] <= 1'b1;
		m_res[40] <= trans;
		m_res[31:0] <= r_pay;
		mk <= 0;
		if (trans) begin
			m_res[39] <= 0; m_res[38:32] <= 0;
			st <= W_MW;
		end else if (la_v && la_blk == r_pay[17:0] && la_row == i_offy) begin
			m_res[39] <= 0; m_res[38:32] <= la_slot;
			st <= W_MW;
		end else if (ns >= NSLOT8) begin
			m_res[39] <= 1; m_res[38:32] <= {4'd0, i_offy};          // uncached: compose fetches it
			st <= W_MW;
		end else if (!if_push && ifq_n < 5'd14) begin
			if_push <= 1; if_d <= {ns[6:0], r_pay[17:0], i_offy};
			la_v <= 1; la_blk <= r_pay[17:0]; la_row <= i_offy; la_slot <= ns[6:0];
			ns <= ns + 8'd1;
			m_res[39] <= 0; m_res[38:32] <= ns[6:0];
			st <= W_MW;
		end
	end
	W_MW: begin
		if (mask[mk[2:0]]) begin
			if (i_spr) begin
				srw_e <= 1; srw_a <= {i_x0 + {5'd0, mk[2:0]}, kk[mk[2:0]]}; srw_d <= m_res;
			end else begin
				wrw_e <= 1; wrw_a <= mk[2:0]; wrw_d <= {r_pre, m_res};
			end
		end
		mk <= mk + 4'd1;
		if (mk == 4'd7) begin
			unres <= unres & ~mask;
			if ((unres & ~mask) == 8'd0) st <= W_RET;
			else st <= W_NEXT;
		end
	end
	W_NEXT: begin
		mk <= 0;
		if (w_kind == 2'd1) begin
			w_ri <= w_ri + 1'd1;
			w_left <= w_left - 11'd1;
			if (w_left == 11'd1) st <= W_FIN;
			else st <= W_R0;
		end else begin
			w_i2 <= w_i2 + 32'd1;
			if (w_i2 + 32'd1 == w_n2) st <= W_FIN;
			else st <= W_S1;
		end
	end
	W_FIN: begin
		if (unres[mk[2:0]]) begin
			if (i_spr) begin
				srw_e <= 1; srw_a <= {i_x0 + {5'd0, mk[2:0]}, kk[mk[2:0]]}; srw_d <= 42'd0;
			end else begin
				wrw_e <= 1; wrw_a <= mk[2:0]; wrw_d <= {w_pre, 42'd0};
			end
		end
		mk <= mk + 4'd1;
		if (mk == 4'd7) st <= W_RET;
	end
	W_RET: begin
		case (wret)
			WR_BG: begin
				bk <= 0;
				if (restart) st <= S_BPIX;
				else begin bq <= 0; cv1 <= 0; cv2 <= 0; st <= S_BCP; end
			end
			WR_BPIX: st <= S_BC0;                              // bk unchanged: its cache step
			default: begin                                     // sprite run done
				i_n <= 0;
				st <= S_SC0;
			end
		endcase
	end

	// ======================================================================================
	// condition list (frame list: memoised, uniform; walk list: per pixel where it differs)
	// ======================================================================================
	E_LW0: begin
		if (pf_ok[0] && pf_cp[0] == cp && !rc_pend) begin
			// prefetched while the previous condition was evaluated
			c0 <= nc0[0]; c1 <= nc1[0]; c2 <= nc2[0]; c3 <= nc3[0]; c_last <= nc_last[0];
			st <= E_C1;
		end else if (pf_ok[1] && pf_cp[1] == cp && !rc_pend) begin
			c0 <= nc0[1]; c1 <= nc1[1]; c2 <= nc2[1]; c3 <= nc3[1]; c_last <= nc_last[1];
			st <= E_C1;
		end else if (!rc_pend && lw_v && lw_a == cp[20:3]) begin
			// the list qword is the one already read
			m_ci   <= cp[2] ? lw_q[55:32] : lw_q[23:0];
			c_last <= cp[2] ? lw_q[63] : lw_q[31];
			st <= E_C0;
		end else if (!rc_pend) begin
			if (!pf_busy) begin rc_req <= 1; rc_pend <= 1; rc_addr <= clist_base + {7'd0, cp[20:3]}; end
		end else if (rc_ack) begin
			rc_pend <= 0;
			lw_q   <= rc_q; lw_a <= cp[20:3]; lw_v <= 1;
			m_ci   <= cp[2] ? rc_q[55:32] : rc_q[23:0];
			c_last <= cp[2] ? rc_q[63] : rc_q[31];
			st <= E_C0;
		end
	end
	E_C0: begin
		// m_ci goes to the cache only when the request is sent: a prefetch in flight meanwhile
		// uses cc_ci for its own condition
		if (!cc_pend) begin if (!pf_busy) begin cc_req <= 1; cc_pend <= 1; cc_ci <= m_ci; end end
		else if (cc_ack) begin
			cc_pend <= 0;
			c0 <= cc_q[63:0]; c1 <= cc_q[127:64]; c2 <= cc_q[191:128]; c3 <= cc_q[255:192];
			st <= E_C1;
		end
	end
	E_C1: begin
		// prefetch the condition likely to be evaluated next, while this one is evaluated
		// (most conditions fail, ending the rule: the next rule's first condition comes first)
		if (!pf_busy) begin : pfk
			reg [20:0] q2;
			q2 = !c_last ? cp + 21'd4 : (lph == 2'd1 && r_wptr != 20'd0) ? {1'b0, r_wptr} : 21'd0;
			if (w_kind == 2'd1 && w_left > 11'd1) begin
				pf_ri <= w_ri + 1'd1; q_ri <= w_ri + 1'd1; pf_rule_ok <= 0; pf_ok[0] <= 1'b0;
				pf_qv <= q2 != 21'd0; pf_qcp <= q2;
				cf_st <= PF_RR1;
			end else if (q2 != 21'd0) pf_fetch(q2, 1'b1);
		end
		if (c_sens && i_group) begin
			// a rewrite can change this condition: walk the group one pixel at a time
			// (an 8k nearby index condition needs no restart - it is never uniform, so the
			// per-pixel loop below already reads each target's index, and an index read
			// is not rewrite-sensitive)
			restart <= 1;
			st <= W_RET;
		end else if (lm_fl && c_memo_ok) begin
			dm_a <= c0[42:32];
			st <= E_M0;
		end else begin
			// a condition that is the same for every pixel of the instance is evaluated once,
			// at the first pixel still unresolved - the pixel at which Mesen, walking pixels
			// in order, first reaches it
			c_memo_new <= 0;
			// (8k: a nearby index condition's target moves with the pixel, so it is never
			// uniform - a sprite run is not a group, it is walked pixel by pixel below)
			e_uni <= lm_fl || (c_ixram && !c_ixnear) ||
			         !(c_type == 8'd5 || c_type == 8'd6 || c_type == 8'd7 ||
			                    c_type == 8'd9 || c_type == 8'd10);
			ek <= lsb8(mask);
			st <= E_EV;
		end
	end
	E_M0: st <= E_M1;
	E_M1: begin
		if (dm_q[1]) begin e_res <= dm_q[0] ^ c_neg; c_memo_new <= 0; st <= E_RES; end
		else begin c_memo_new <= 1; e_uni <= 1; ek <= lsb8(mask); st <= E_EV; end
	end
	E_EV: begin
		case (c_type)
			8'd0, 8'd1: begin v_idx <= c1[7:0]; st <= E_V0; end
			8'd2: begin
				if (c1[15:0] == 16'd0) begin e_res <= 0; st <= E_RES; end
				else begin dv_rem <= 0; dv_i <= 5'd31; st <= E_DIV; end
			end
			8'd5: if (!e_uni && !c_ixram) begin     // tileNearby: all mask pixels pipelined
				tp_rem <= mask; tp_keep <= 0; tp_v <= 0;
				st <= TP;
			end else if (!n_in) begin e_res <= 0; st <= E_RES; end
			else begin
				t_spr <= 1'b0;
				t_y <= n_at[15:8]; t_x <= n_at[7:0];
				m_pa <= {dline(n_at[15:0]), n_at[7:1]}; t_dy <= dline(n_at[15:0]);
				m_ly <= dline(n_at[15:0]); m_sy <= dline(n_at[15:0]); m_ay <= lline(n_at[15:0]);
				lk_a <= {n_at[15:8], n_at[7:1]};
				st <= T1;
			end
			8'd6: if (!e_uni && !c_ixram) begin     // spriteNearby: all mask pixels pipelined
				sp_rem <= mask; sp_v <= 0; sp_np <= 0; sp_hit <= 0;
				st <= SP0;
			end else if (!n_in) begin e_res <= 0; st <= E_RES; end
			else begin
				t_spr <= 1'b1;
				t_y <= n_at[15:8]; t_x <= n_at[7:0];
				m_pa <= {dline(n_at[15:0]), n_at[7:1]}; t_dy <= dline(n_at[15:0]);
				m_ly <= dline(n_at[15:0]); m_sy <= dline(n_at[15:0]); m_ay <= lline(n_at[15:0]);
				lk_a <= {n_at[15:8], n_at[7:1]};
				st <= T1;
			end
			8'd3, 8'd4: begin
				if (!n_in) begin e_res <= 0; st <= E_RES; end
				else begin
					t_spr <= c_type == 8'd4 || c_type == 8'd6;
					t_y <= n_at[15:8]; t_x <= n_at[7:0];
					m_pa <= {dline(n_at[15:0]), n_at[7:1]}; t_dy <= dline(n_at[15:0]);
					m_ly <= dline(n_at[15:0]); m_sy <= dline(n_at[15:0]); m_ay <= lline(n_at[15:0]);
					lk_a <= {n_at[15:8], n_at[7:1]};
					st <= T1;
				end
			end
			8'd7:  begin e_res <= cmp_op(c_op, sx10(v_x),  c1[31:0]); st <= E_RES; end
			8'd8:  begin e_res <= cmp_op(c_op, sx10(v_y),  c1[31:0]); st <= E_RES; end
			8'd9:  begin e_res <= cmp_op(c_op, sx10(v_ox), c1[31:0]); st <= E_RES; end
			8'd10: begin e_res <= cmp_op(c_op, sx10(v_oy), c1[31:0]); st <= E_RES; end
			8'd11: begin e_res <= i_hm;  st <= E_RES; end
			8'd12: begin e_res <= i_vm;  st <= E_RES; end
			8'd13: begin e_res <= i_bgp; st <= E_RES; end
			default: begin e_res <= 0; unsup <= 1; st <= E_RES; end
		endcase
	end
	E_V0: st <= E_V1;
	E_V1: begin
		e_a <= v_q & c_msk;
		if (c_type == 8'd1) begin
			e_res <= cmp_op(c_op, {24'd0, v_q & c_msk}, {24'd0, c1[15:8]});
			st <= E_RES;
		end else begin v_idx <= c1[15:8]; st <= E_V2; end
	end
	E_V2: st <= E_V3;
	E_V3: begin
		e_res <= cmp_op(c_op, {24'd0, e_a}, {24'd0, v_q & c_msk});
		st <= E_RES;
	end
	E_DIV: begin : fdiv
		reg [16:0] r;
		r = {dv_rem[15:0], frame[dv_i]};
		if (r >= {1'b0, c1[15:0]}) r = r - {1'b0, c1[15:0]};
		dv_rem <= r;
		dv_i <= dv_i - 5'd1;
		if (dv_i == 5'd0) begin
			e_res <= r >= {1'b0, c1[31:16]};
			st <= E_RES;
		end
	end
	E_RES: begin : eres
		reg       r;
		reg [7:0] m;
		r = e_res ^ c_neg;
		if (lm_fl) begin
			if (c_memo_new) begin dmw_e <= 1; dmw_d <= {1'b1, r}; end   // dm_a still holds it
			if (!r) begin mask <= 0; st <= W_LIST; end
			else if (c_last) st <= W_LIST;
			else begin cp <= cp + 21'd4; st <= E_LW0; end
		end else if (e_uni) begin
			if (!r) begin mask <= 0; st <= W_LIST; end
			else if (c_last) st <= W_LIST;
			else begin cp <= cp + 21'd4; st <= E_LW0; end
		end else begin
			m = mask;
			if (!r) m[ek] = 1'b0;
			mask <= m;
			// next pixel still in the mask, above ek
			if ((m & ~((8'd2 << ek) - 8'd1)) != 8'd0) begin
				ek <= lsb8(m & ~((8'd2 << ek) - 8'd1));
				st <= E_EV;
			end else if (m == 8'd0) st <= W_LIST;
			else if (c_last) st <= W_LIST;
			else begin cp <= cp + 21'd4; st <= E_LW0; end
		end
	end

	// ======================================================================================
	// target pixel of a tile / sprite condition
	// ======================================================================================
	T1: st <= T2;
	T2: begin : tsel
		reg [6:0] px;
		px = t_x[0] ? p_q[13:7] : p_q[6:0];
		t_lk <= t_x[0] ? lk1_q : lk0_q;
		t_rx <= 0;
		if (!t_spr) begin
			if (px[5:0] == 6'd63 || px[5:0] >= l_q[5:0]) begin e_res <= 0; st <= E_RES; end
			else begin d_y <= t_dy; d_slot <= px[5:0]; st <= T3; end
		end else begin : tlist
			// the pixel's list: its captured sprites (where sprites are shown) and the lanes of
			// its line covering it (8a: additions are appended whether sprites are shown or not)
			reg [29:0] fl;
			reg [TLW-1:0] tl;
			fl = find4(s_q, l_q[10:6], t_x);
			tl = merget(fl, px[6], a_q, a_n, t_x);
			t_drawn4 <= tl[TLW-1 -: 4]; t_bgp <= tl[TLW-5 -: 4];
			t_n <= tl[4*TEW +: 3]; t_f <= tl[4*TEW-1:0];
			t_j <= 0;
			if (tl[4*TEW +: 3] == 3'd0) begin e_res <= 0; st <= E_RES; end
			else st <= T7;
		end
	end
	T3: st <= T4;
	T4: begin
		if (!d_span[RAW]) begin e_res <= 0; st <= E_RES; end
		else begin d_rid <= d_span[RAW-1:0]; st <= T5; end
	end
	T5: st <= T6;
	T6: if (c_ixram && !t_rx) begin
		// 8k: the target span's qword A (frame record, line t_y, slot d_slot): CHR linear [21:0]
		if (!rc_pend) begin
			if (!pf_busy) begin
				rc_req <= 1; rc_byp <= 1; rc_pend <= 1;
				rc_addr <= lbase + 25'h08000 + {9'd0, t_dy, 8'd0} + 25'd8 + {18'd0, d_slot, 1'b0};
			end
		end else if (rc_ack) begin
			rc_pend <= 0;
			t_ri <= rc_q[19:4];                       // (CHR linear & 0xFFFFF) >> 4
			t_rx <= 1;
		end
	end else begin
		e_res <= tmatch(rw_able && in_win(t_y, t_x) &&
		                seen(t_lk, t_y, t_x, 4'd0, y, cur_x, cur_p), 1'b0);
		st <= E_RES;
	end
	T7: begin : tent
		reg [TEW-1:0] e;
		e = t_f[TEW * t_j +: TEW];
		t_rx <= 0;
		t_ln <= e[TEW-1];
		if ({1'b0, t_j} >= t_n) begin e_res <= 0; st <= E_RES; end
		else if (e[TEW-1]) begin d_rid <= e[RAW-1:0]; st <= T10; end   // an added sprite
		else begin d_y <= t_dy; d_fetch <= e[3:0]; st <= T8; end
	end
	T8: st <= T9;
	T9: begin
		if (!d_spr[RAW]) begin
			if (t_j == 2'd3) begin e_res <= 0; st <= E_RES; end
			else begin t_j <= t_j + 2'd1; st <= T7; end
		end else begin d_rid <= d_spr[RAW-1:0]; st <= T10; end
	end
	T10: st <= T11;
	T11: if (c_ixram && !t_rx && !t_ln) begin
		// 8k: the target sprite fetch's qword A (line t_dy, fetch d_fetch): CHR linear [37:16]
		// (an added sprite has no index in a CHR-RAM game: tmatch fails it, nothing to read)
		if (!rc_pend) begin
			if (!pf_busy) begin
				rc_req <= 1; rc_byp <= 1; rc_pend <= 1;
				rc_addr <= lbase + 25'h08000 + {9'd0, t_dy, 8'd0} + 25'd80 + {20'd0, d_fetch, 1'b0};
			end
		end else if (rc_ack) begin
			rc_pend <= 0;
			t_ri <= rc_q[35:20];
			t_rx <= 1;
		end
	end else begin
		if (tmatch(rw_able && in_win(t_y, t_x) &&
		           seen(t_drawn4[t_j], t_y, t_x, 4'd1 + phase_of(t_bgp[t_j], t_n, t_j), y, cur_x,
		                cur_p), t_ln)) begin
			e_res <= 1; st <= E_RES;
		end else if (t_j == 2'd3) begin e_res <= 0; st <= E_RES; end
		else begin t_j <= t_j + 2'd1; st <= T7; end
	end

	// pipelined tileNearby: stage 2 has the frame map word, 4 the line span, 6 the record
	TP: begin : tpipe
		reg        [2:0] k;
		reg signed [41:0] at;
		reg        [6:0] px;
		reg              r;
		// issue
		if (tp_rem != 8'd0) begin
			k  = lsb8(tp_rem);
			at = (($signed({34'd0, y}) + n_Y) <<< 8) + $signed({34'd0, i_x0 + {5'd0, k}}) + n_X;
			tp_rem[k] <= 1'b0;
			tp_v[1] <= 1'b1;
			tp_k[1] <= k; tp_tx[1] <= at[7:0]; tp_ty[1] <= at[15:8]; tp_dy[1] <= dline(at[15:0]);
			tp_dead[1] <= !(at >= 0 && at < 42'sd61440);
			m_pa <= {dline(at[15:0]), at[7:1]}; m_ly <= dline(at[15:0]);
			lk_a <= {at[15:8], at[7:1]};
		end else tp_v[1] <= 1'b0;
		tp_v[6:2] <= tp_v[5:1];
		tp_dy[2] <= tp_dy[1];
		for (zi = 2; zi <= 6; zi = zi + 1) begin
			tp_k[zi] <= tp_k[zi - 1]; tp_tx[zi] <= tp_tx[zi - 1]; tp_ty[zi] <= tp_ty[zi - 1];
			tp_dead[zi] <= tp_dead[zi - 1];
		end
		for (zi = 4; zi <= 6; zi = zi + 1) tp_lk[zi] <= tp_lk[zi - 1];
		// stage 2: the BG span of the target pixel
		px = tp_tx[2][0] ? p_q[13:7] : p_q[6:0];
		tp_lk[3] <= tp_tx[2][0] ? lk1_q : lk0_q;
		if (px[5:0] == 6'd63 || px[5:0] >= l_q[5:0]) tp_dead[3] <= 1'b1;
		d_y <= tp_dy[2]; d_slot <= px[5:0];
		// stage 4: the record of that span
		if (!d_span[RAW]) tp_dead[5] <= 1'b1;
		d_rid <= d_span[RAW-1:0];
		// stage 6: match
		r = !tp_dead[6] && tmatch(rw_able && in_win(tp_ty[6], tp_tx[6]) &&
		                          seen(tp_lk[6], tp_ty[6], tp_tx[6], 4'd0, y,
		                               i_x0 + {5'd0, tp_k[6]}, ph[tp_k[6]]), 1'b0);
		if (tp_v[6] && (r ^ c_neg)) tp_keep[tp_k[6]] <= 1'b1;
		// done: every issued pixel has left stage 6
		if (tp_rem == 8'd0 && tp_v == 6'd0) begin
			mask <= mask & tp_keep;
			if ((mask & tp_keep) == 8'd0 || c_last) st <= W_LIST;
			else begin cp <= cp + 21'd4; st <= E_LW0; end
		end
	end

	// pipelined spriteNearby, pass 1: stage 2 has the frame map word, the target line's header
	// and its sprite fetch word, from which the pixel's sprite list is taken
	SP0: begin : sppix
		reg        [2:0] k;
		reg signed [41:0] at;
		reg        [6:0] px;
		reg       [29:0] fl;
		reg    [TLW-1:0] tl;
		if (sp_rem != 8'd0) begin
			k  = lsb8(sp_rem);
			// offsets mirrored by the evaluated record's flips (hdref.at)
			at = (($signed({34'd0, y}) + n_Ys) <<< 8) + $signed({34'd0, i_x0 + {5'd0, k}}) + n_Xs;
			sp_rem[k] <= 1'b0;
			sp_v[1] <= 1'b1;
			sp_k[1] <= k; sp_tx[1] <= at[7:0]; sp_ty[1] <= at[15:8]; sp_dy[1] <= dline(at[15:0]);
			sp_dead[1] <= !(at >= 0 && at < 42'sd61440);
			m_pa <= {dline(at[15:0]), at[7:1]}; m_ly <= dline(at[15:0]); m_sy <= dline(at[15:0]);
			m_ay <= lline(at[15:0]);
		end else sp_v[1] <= 1'b0;
		sp_v[2] <= sp_v[1];
		sp_k[2] <= sp_k[1]; sp_tx[2] <= sp_tx[1]; sp_ty[2] <= sp_ty[1]; sp_dy[2] <= sp_dy[1];
		sp_dead[2] <= sp_dead[1];
		px = sp_tx[2][0] ? p_q[13:7] : p_q[6:0];
		fl = find4(s_q, l_q[10:6], sp_tx[2]);
		tl = merget(fl, px[6], a_q, a_n, sp_tx[2]);
		if (sp_v[2] && !sp_dead[2] && tl[4*TEW +: 3] != 3'd0) begin
			spl_k[sp_np[2:0]] <= sp_k[2]; spl_tx[sp_np[2:0]] <= sp_tx[2];
			spl_ty[sp_np[2:0]] <= sp_ty[2]; spl_dy[sp_np[2:0]] <= sp_dy[2];
			spl_fl[sp_np[2:0]] <= tl;
			sp_np <= sp_np + 4'd1;
		end
		if (sp_rem == 8'd0 && sp_v == 2'd0) begin
			sp_p <= 0; sp_j <= 0; si_v <= 0;
			st <= SP1;
		end
	end
	// pass 2: stage 2 has the sprite's record id, stage 4 the record
	SP1: begin : spent
		reg [TLW-1:0] fl;
		reg [TEW-1:0] e;
		reg        r;
		reg  [7:0] m;
		if (sp_p != sp_np) begin
			fl = spl_fl[sp_p[2:0]];
			e  = fl[TEW * sp_j +: TEW];
			d_y <= spl_dy[sp_p[2:0]];
			d_fetch <= e[3:0];
			si_v[1] <= 1'b1;
			si_ln[1] <= e[TEW-1]; si_lid[1] <= e[RAW-1:0];
			si_k[1] <= spl_k[sp_p[2:0]]; si_tx[1] <= spl_tx[sp_p[2:0]]; si_ty[1] <= spl_ty[sp_p[2:0]];
			// bit sp_j of the 4-bit fields fl[TLW-5 -: 4] / fl[TLW-1 -: 4], written as one index:
			// indexing a part-select (x[a -: 4][j]) is accepted by Verilator but is not legal
			// SystemVerilog, and ModelSim 10.5 / Quartus 17 reject it
			si_ph[1] <= 4'd1 + phase_of(fl[TLW-8 + 32'(sp_j)], fl[4*TEW +: 3], sp_j);
			si_dr[1] <= fl[TLW-4 + 32'(sp_j)];
			if ({1'b0, sp_j} + 3'd1 >= fl[4*TEW +: 3]) begin sp_p <= sp_p + 4'd1; sp_j <= 0; end
			else sp_j <= sp_j + 2'd1;
		end else si_v[1] <= 1'b0;
		si_v[4:2] <= si_v[3:1];
		si_lid[2] <= si_lid[1];
		for (zi = 2; zi <= 4; zi = zi + 1) begin
			si_k[zi] <= si_k[zi - 1]; si_tx[zi] <= si_tx[zi - 1]; si_ty[zi] <= si_ty[zi - 1];
			si_ph[zi] <= si_ph[zi - 1]; si_dr[zi] <= si_dr[zi - 1]; si_ln[zi] <= si_ln[zi - 1];
		end
		si_dead[4] <= si_dead[3];
		// stage 2: a captured sprite's record id from its fetch; an added sprite's is its own
		si_dead[3] <= si_ln[2] ? 1'b0 : !d_spr[RAW];
		d_rid <= si_ln[2] ? si_lid[2] : d_spr[RAW-1:0];
		r = si_v[4] && !si_dead[4] &&
		    tmatch(rw_able && in_win(si_ty[4], si_tx[4]) &&
		           seen(si_dr[4], si_ty[4], si_tx[4], si_ph[4], y, i_x0 + {5'd0, si_k[4]},
		                ph[si_k[4]]), si_ln[4]);
		if (r) sp_hit[si_k[4]] <= 1'b1;
		if (sp_p == sp_np && si_v == 4'd0) begin
			m = mask & (c_neg ? ~sp_hit : sp_hit);
			mask <= m;
			if (m == 8'd0 || c_last) st <= W_LIST;
			else begin cp <= cp + 21'd4; st <= E_LW0; end
		end
	end

	default: st <= S_IDLE;
	endcase

	if (reset) begin
		st <= S_IDLE; busy <= 0; rc_pend <= 0; cc_pend <= 0; dp_v <= 0; dp2_v <= 0; fs <= F_OFF; n_ready <= 0; csteps <= 0;
		ba_done <= 0;
		cf_st <= PF_IDLE; pf_ok <= 0; lw_v <= 0; pf_rule_ok <= 0; pf_qv <= 0;
	end
end

`ifdef VERILATOR
// ==========================================================================================
// simulation-only path counters (tb_render.cpp prints them, so a test can show that a rare
// path was actually taken); not synthesised
// ==========================================================================================
reg [31:0] sim_walks    /*verilator public_flat_rd*/ = 0;
reg [31:0] sim_kind2    /*verilator public_flat_rd*/ = 0;   // walks of an unfiltered list
reg [31:0] sim_restart  /*verilator public_flat_rd*/ = 0;   // BG group walks redone per pixel
reg [31:0] sim_sens     /*verilator public_flat_rd*/ = 0;   // sensitive conditions evaluated
reg [31:0] sim_rw       /*verilator public_flat_rd*/ = 0;   // tile matches made on a rewrite
reg [31:0] sim_memo_hit /*verilator public_flat_rd*/ = 0;
reg [31:0] sim_memo_new /*verilator public_flat_rd*/ = 0;
reg [31:0] sim_scratch  /*verilator public_flat_rd*/ = 0;   // uncached tiles drawn
reg [31:0] sim_ofstall  /*verilator public_flat_rd*/ = 0;   // compose held by a full output FIFO
reg [31:0] sim_fovf     /*verilator public_flat_rd*/ = 0;   // frames drawn without lookups
reg [31:0] sim_ixr      /*verilator public_flat_rd*/ = 0;   // 8k frame-record index reads
reg [31:0] sim_bgstep   /*verilator public_flat_rd*/ = 0;   // background pixel-blends issued
reg [31:0] sim_bgunc    /*verilator public_flat_rd*/ = 0;   // background cells left uncached
reg [31:0] sim_bgadd    /*verilator public_flat_rd*/ = 0;   // background steps by mode: Add
reg [31:0] sim_bgsub    /*verilator public_flat_rd*/ = 0;   //   and Subtract
reg [31:0] sim_bgprep   /*verilator public_flat_rd*/ = 0;   // worst line's background prep (clk)
reg [31:0] sim_bgprep_c = 0;
wire sim_cst_col = cst_bd || (!cst_bg && !cst_r[41]);
wire sim_cst_scr = !sim_cst_col && cst_r[39] &&
                   !(scr_v && scr_blk == cst_r[17:0] && scr_row == cst_r[34:32]);
// cycle profile by phase, per frame and for the frame's slowest line (printed at frame end):
// 0 line load + scan, 1 sprite-target evaluation (spriteAt/Nearby), 2 BG instances + cache
// rule, 3 sprite runs, 4 fetch wait, 5 compose,
// 6 rule walk, 7 condition evaluation
function automatic [2:0] pf_cat(input st_t s);
	case (s)
		T7, T8, T9, T10, T11, SP0, SP1: pf_cat = 3'd1;
		S_BI0, S_BI1, S_BI2, S_BI3, S_BI4, S_BOX, S_BC0, S_BC1, S_BC2, S_BCP, S_BPIX: pf_cat = 3'd2;
		S_SF0, S_SF1, S_SF2, S_LF0, S_SC0, S_SC1, S_SC2: pf_cat = 3'd3;
		S_FW: pf_cat = 3'd4;
		S_CD, S_CSW, S_CSW2, S_CEND: pf_cat = 3'd5;
		W0, W1, W2, W_R0, W_R1, W_R2, W_S0, W_S0B, W_S1, W_S1B, W_LIST, W_MATCH, W_MW, W_NEXT,
		W_FIN, W_RET: pf_cat = 3'd6;
		E_LW0, E_C0, E_C1, E_M0, E_M1, E_EV, E_V0, E_V1, E_V2, E_V3, E_DIV, E_RES,
		T1, T2, T3, T4, T5, T6, TP: pf_cat = 3'd7;
		default: pf_cat = 3'd0;
	endcase
endfunction
reg [31:0] pf_f [8];
reg [31:0] pf_l [8];
reg [31:0] pf_w [8];
reg [31:0] pf_lt, pf_wt;
reg  [7:0] pf_ly, pf_wy;
reg        pf_lv;
reg        pf_pr = 0;
reg [31:0] pf_st [128];              // PROFILE >= 3: clocks per state
reg [31:0] pf_en [128];              //                entries into each state
st_t       pf_prev = S_IDLE;
always @(posedge clk) begin : prof_st
	integer k;
	st_t    s;
	if (PROFILE >= 3) begin
		pf_prev <= st;
		if (st == S_IDLE && start) for (k = 0; k < 128; k = k + 1) begin pf_st[k] <= 0; pf_en[k] <= 0; end
		else if (busy && (PLINE < 0 || {24'd0, y} == PLINE)) begin
			pf_st[st] <= pf_st[st] + 1;
			if (st != pf_prev) pf_en[st] <= pf_en[st] + 1;
		end
		if (pf_pr) for (k = 0; k < 128; k = k + 1)
			if (pf_st[k] != 0) begin
				s = st_t'(k[6:0]);
				$display("[render state] %s %0d %0d", s.name(), pf_st[k], pf_en[k]);
			end
	end
end
always @(posedge clk) begin : prof
	integer k;
	if (st == S_IDLE && start) begin
		for (k = 0; k < 8; k = k + 1) begin pf_f[k] <= 0; pf_l[k] <= 0; pf_w[k] <= 0; end
		pf_lt <= 0; pf_wt <= 0; pf_lv <= 0;
	end else if (busy) begin
		pf_f[pf_cat(st)] <= pf_f[pf_cat(st)] + 1;
		if ((st == S_LINE && line_ok) || st == S_FEND) begin   // clocks waiting on line_ok count to the line before
			if (PROFILE >= 2 && pf_lv)
				$display("[render line] %0d: %0d clk = load %0d speval %0d bgi %0d spr %0d fwait %0d compose %0d walk %0d eval %0d",
				         pf_ly, pf_lt, pf_l[0], pf_l[1], pf_l[2], pf_l[3], pf_l[4], pf_l[5], pf_l[6], pf_l[7]);
			if (pf_lv && pf_lt > pf_wt) begin
				pf_wt <= pf_lt; pf_wy <= pf_ly;
				for (k = 0; k < 8; k = k + 1) pf_w[k] <= pf_l[k];
			end
			for (k = 0; k < 8; k = k + 1) pf_l[k] <= 0;
			pf_lt <= 0; pf_lv <= st == S_LINE; pf_ly <= y;
		end else begin
			pf_l[pf_cat(st)] <= pf_l[pf_cat(st)] + 1;
			pf_lt <= pf_lt + 1;
		end
	end
	pf_pr <= st == S_FEND && out_idle && !pf_busy;     // print one clock later: last line folded
	if (PROFILE != 0 && pf_pr)
		$display("[render prof] frame load %0d speval %0d bgi %0d spr %0d fwait %0d compose %0d walk %0d eval %0d | worst line %0d: %0d clk = load %0d speval %0d bgi %0d spr %0d fwait %0d compose %0d walk %0d eval %0d",
		         pf_f[0], pf_f[1], pf_f[2], pf_f[3], pf_f[4], pf_f[5], pf_f[6], pf_f[7],
		         pf_wy, pf_wt, pf_w[0], pf_w[1], pf_w[2], pf_w[3], pf_w[4], pf_w[5], pf_w[6], pf_w[7]);
end
always @(posedge clk) begin
	if (st == W2) sim_walks <= sim_walks + 1;
	if (st == W2 && i_rv && !f_ovf && q_rec[48:47] == 2'd2) sim_kind2 <= sim_kind2 + 1;
	if (st == S_IDLE && start && frame_ovf) sim_fovf <= sim_fovf + 1;
	if (rc_req && rc_byp) sim_ixr <= sim_ixr + 1;
	if (st == E_C1 && c_sens && i_group) sim_restart <= sim_restart + 1;
	if (st == E_C1 && c_sens && !i_group) sim_sens <= sim_sens + 1;
	if (st == T6 && rw_able && in_win(t_y, t_x) && seen(t_lk, t_y, t_x, 4'd0, y, cur_x, cur_p))
		sim_rw <= sim_rw + 1;
	if (st == TP && tp_v[6] && !tp_dead[6] && rw_able && in_win(tp_ty[6], tp_tx[6]) &&
	    seen(tp_lk[6], tp_ty[6], tp_tx[6], 4'd0, y, i_x0 + {5'd0, tp_k[6]}, ph[tp_k[6]]))
		sim_rw <= sim_rw + 1;
	if (st == T11 && rw_able && in_win(t_y, t_x) &&
	    seen(t_drawn4[t_j], t_y, t_x, 4'd1 + phase_of(t_bgp[t_j], t_n, t_j), y, cur_x, cur_p))
		sim_rw <= sim_rw + 1;
	if (st == SP1 && si_v[4] && !si_dead[4] && rw_able && in_win(si_ty[4], si_tx[4]) &&
	    seen(si_dr[4], si_ty[4], si_tx[4], si_ph[4], y, i_x0 + {5'd0, si_k[4]}, ph[si_k[4]]))
		sim_rw <= sim_rw + 1;
	if (st == E_M1 && dm_q[1]) sim_memo_hit <= sim_memo_hit + 1;
	if (st == E_M1 && !dm_q[1]) sim_memo_new <= sim_memo_new + 1;
	if (st == S_CSW && !if_push && ifq_n == 5'd0 && img_out == 7'd0 && !dp_v && !dp2_v)
		sim_scratch <= sim_scratch + 1;
	if (st == S_CD && csteps != 50'd0 && !(cst_bg && !be_hit) && !sim_cst_scr &&
	    (csteps & (csteps - 50'd1)) == 50'd0 && of_ahead >= OF_D[OF_LOG2+1:0])
		sim_ofstall <= sim_ofstall + 1;
	if (st == S_CD && csteps != 50'd0 && cst_bg && be_hit && !sim_cst_scr &&
	    ((csteps & (csteps - 50'd1)) != 50'd0 || of_ahead < OF_D[OF_LOG2+1:0])) begin
		sim_bgstep <= sim_bgstep + 1;
		if (be_q[59:58] == 2'd1) sim_bgadd <= sim_bgadd + 1;
		if (be_q[59:58] == 2'd2) sim_bgsub <= sim_bgsub + 1;
	end
	if (bs == B_C5 && ba_done && ba_unc) sim_bgunc <= sim_bgunc + 1;
	if (bl_kick) sim_bgprep_c <= 0;
	else if (bs != B_IDLE && !(bs == B_F0 || bs == B_F1 || bs == B_F2 || bs == B_F3 || bs == B_F4)) begin
		sim_bgprep_c <= sim_bgprep_c + 1;
		if (sim_bgprep_c + 1 > sim_bgprep) sim_bgprep <= sim_bgprep_c + 1;
	end
end
`endif

endmodule
