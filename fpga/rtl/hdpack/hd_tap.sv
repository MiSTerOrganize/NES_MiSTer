// hd_tap.sv - HD pack observation tap for the NES PPU (test version).
//
// Copies what the HD engine needs from the PPU's own fetch pipeline. It only OBSERVES:
// nothing here feeds back into the PPU, so the NES picture is unchanged.
//
// Per NES line L the tap produces, in this order:
//   - sprite writes for line L        (PPU cycles 257-320 of line L-1, one per fetched sprite;
//                                      with the 16-sprite option also one extra-sprite write
//                                      per slot, 4 dots after that slot's main write; extra
//                                      entry 0 is deferred to cycle 326 and marked not valid
//                                      when it is the core's copy of main sprite 7 - see below)
//   - BG tile writes for line L       (cycles 321-336 of L-1 = slots 0,1; then 1-256 of L)
//   - one pixel write per pixel x     (cycles 1-256 of L, x = cycle-1)
//   - line_done with line_y = L       (cycle 256 of L, after pixel 255)
// so the engine can switch to a fresh buffer on every line_done.
//
// Timing mirrors BgPainter / SpriteAddressGen exactly: same ce, same cycle decodes.

module hd_tap
(
	input             clk,
	input             ce,
	input       [8:0] cycle,
	input       [8:0] scanline,
	input             bgp_en,          // BgPainter shift enable
	input             in_visible_frame,
	input       [2:0] fine_x_scroll,
	input      [14:0] vram_v,
	input       [7:0] vram_din,
	input      [21:0] chr_lin,         // linear CHR address of the current fetch (from the mapper)
	input       [3:0] bg_px,           // BG pixel {palette[1:0], colour index[1:0]} after clipping
	input             show_bg,         // BG visible on this pixel (clip + enable)
	input             show_obj,        // sprites visible on this pixel (clip + enable)
	input       [3:0] spr_load,        // SpriteAddressGen load strobes {pix1,pix2,x,attr}
	input      [26:0] spr_load_in,
	input             spr_dummy,       // current sprite fetch is a dummy (not in range)

	// the core's 16-sprite option: SpriteAddressGenEx runs in lock-step with SpriteAddressGen
	// (same count), reading its pattern bytes in the first half of each 8-dot fetch slot through
	// the cart's extra-sprite address port, so chr_lin names the extra sprite's tile there
	input             extra_sprites,
	input       [3:0] spr_load_ex,     // SpriteAddressGenEx load strobes {pix1,pix2,x,attr}
	input      [26:0] spr_load_in_ex,
	input             spr_ex_dummy,    // this extra sprite draws nothing (absent, masked, rendering off)
	output reg        spr_ex_we,       // one PPU dot after the main entry of the same slot
	output reg  [2:0] spr_ex_idx,
	output reg [54:0] spr_ex_data,     // same layout as spr_data

	output reg        bg_we,
	output reg  [5:0] bg_slot,
	output reg [21:0] bg_lin,
	output reg  [1:0] bg_pal,

	output reg        spr_we,
	output reg  [2:0] spr_idx,
	output reg [54:0] spr_data,        // {valid, attr[7:0], x[7:0], lin[21:0], pix1[7:0], pix2[7:0]}

	output reg        px_we,
	output reg  [7:0] px_x,
	output reg  [5:0] px_slot,
	output reg  [2:0] px_fx,
	output reg  [3:0] px_bgc,          // {palette, colour index}
	output reg        px_bg_on,
	output reg        px_spr_on,

	output reg        line_done,
	output reg  [8:0] line_y,

	// stage 2 (frame record): per-pixel state Mesen records at the same dot
	input      [14:0] vram_t,          // the "t" register
	input             grayscale,       // PPUMASK bit 0
	input       [2:0] emph,            // PPUMASK emphasis, bit 0 = red
	input             render_en,       // BG or sprites enabled (the PPU's effective value)
	input             pre_line,        // this is the pre-render line
	output reg        px_branch_b,     // Mesen branch B: rendering off and v in $3F00-$3FFF
	output reg [14:0] px_t,
	output reg  [2:0] px_finex,
	output reg        px_gray,
	output reg  [2:0] px_emph,
	output reg        line_pre,        // with line_done: the pre-render line finished
	output reg        snap             // one pulse at scanline 240, dot 0 (Mesen's SendFrame point:
	                                   // the watched-address snapshot and the frame number)
);

// ---------------- background ----------------
reg [21:0] lin_next;
reg  [1:0] attr_next;
reg [15:0] tsel;            // which of the two tiles in the pipe each pixel belongs to
reg  [2:0] off [16];        // pixel column within its tile
reg  [5:0] slot_of [2];
reg        tog;
reg  [5:0] slot_cnt;

wire [3:0] pi = {1'b0, fine_x_scroll};

// ---------------- sprites ----------------
reg  [7:0] s_attr, s_x, s_pix1;
reg [21:0] s_lin;
reg  [2:0] s_cnt;

// ---------------- extra sprites ----------------
reg  [7:0] x_attr, x_x, x_pix1;
reg [21:0] x_lin;
reg  [3:0] x_cnt;           // extra fetches seen this line (slot k fetches extra sprite k)
reg        x_pend = 0;      // extra entries 1-7: written 4 dots after their slot's main entry
reg  [2:0] x_wait;
reg  [2:0] x_idx;
reg [54:0] x_hold;
reg        x0_have = 0;     // extra entry 0: held until main entry 7 is known (see below)
reg        x0_pend = 0;
reg  [2:0] x0_wait;
reg [54:0] x0_hold;

integer i;

always @(posedge clk) begin
	bg_we     <= 0;
	spr_we    <= 0;
	spr_ex_we <= 0;
	px_we     <= 0;
	line_done <= 0;
	snap      <= 0;

	if (ce) begin
		if (cycle == 9'd0 && scanline == 9'd240) snap <= 1;

		// --- BG: mirror BgPainter's latches and shift ---
		if (bgp_en) begin
			if (cycle[2:0] == 4)
				attr_next <= (!vram_v[1] && !vram_v[6]) ? vram_din[1:0] :
				             ( vram_v[1] && !vram_v[6]) ? vram_din[3:2] :
				             (!vram_v[1] &&  vram_v[6]) ? vram_din[5:4] : vram_din[7:6];
			if (cycle[2:0] == 6)
				lin_next <= chr_lin;

			tsel <= {1'b0, tsel[15:1]};
			for (i = 0; i < 15; i = i + 1) off[i] <= off[i+1];

			if (cycle[2:0] == 0) begin
				tsel[15:8] <= {8{tog}};
				for (i = 0; i < 8; i = i + 1) off[8+i] <= i[2:0];
				slot_of[tog] <= slot_cnt;
				tog          <= ~tog;
				if (slot_cnt != 6'd63) slot_cnt <= slot_cnt + 1'd1;
				bg_we   <= 1;
				bg_slot <= slot_cnt;
				bg_lin  <= lin_next;
				bg_pal  <= attr_next;
			end
		end

		// --- pixels ---
		if (in_visible_frame) begin
			px_we     <= 1;
			px_x      <= cycle[7:0] - 8'd1;
			px_slot   <= slot_of[tsel[pi]];
			px_fx     <= off[pi];
			px_bgc    <= bg_px;
			px_bg_on  <= show_bg;
			px_spr_on <= show_obj;
			px_branch_b <= !render_en && (vram_v[13:8] == 6'h3F);
			px_t      <= vram_t;
			px_finex  <= fine_x_scroll;
			px_gray   <= grayscale;
			px_emph   <= emph;
		end

		// --- sprites: copy the fields as SpriteAddressGen hands them to SpriteSet ---
		if (spr_load[0]) s_attr <= spr_load_in[10:3];
		if (spr_load[1]) s_x    <= spr_load_in[10:3];
		if (spr_load[3]) begin s_pix1 <= spr_load_in[26:19]; s_lin <= chr_lin; end
		if (spr_load[2]) begin
			spr_we   <= 1;
			spr_idx  <= s_cnt;
			spr_data <= {~spr_dummy, s_attr, s_x, s_lin, s_pix1, spr_load_in[18:11]};
			s_cnt    <= s_cnt + 1'd1;
		end

		// --- extra sprites: the same fields from SpriteAddressGenEx. Its attr strobe is at slot
		// dot 2, where the fetch is the HIGH plane (or, if chr_lin lags one dot, the low plane);
		// the plane is address bit 3 inside a 16-byte tile, so clearing it names the low-plane
		// row exactly as the main entry's s_lin does.
		//
		// Both groups strobe pix2 on the same dot, and the capture has one sprite port, so an
		// extra entry is written 4 dots after its slot's main entry (main entries are 8 dots
		// apart, so writes stay 4 dots apart and the capture's entry slot never waits).
		//
		// Extra entry 0 is a copy of main entry 7 in this core: its start search (ppu.sv,
		// "hunt for the 8th sprite") stops after counting 7 in-range sprites with a 3-bit
		// counter, so the extra list begins at the 8th in-range sprite, which is already main
		// sprite 7, and the 16th is never loaded (upstream MiSTer NES does the same). In the SD
		// picture the copy never shows (main 7 wins wherever either is opaque), but an HD render
		// would draw the replacement tile twice and double-blend its translucent pixels, which
		// Mesen never does. So entry 0 is held until main entry 7 is fetched (slot 7) and marked
		// not valid when it is identical to it; written at dot 326, between extra entry 7 (324)
		// and the first BG span (328). Comparing rather than always dropping keeps a real 9th
		// sprite if the core's search is ever fixed.
		if (spr_load_ex[0]) begin x_attr <= spr_load_in_ex[10:3]; x_lin <= chr_lin & ~22'd8; end
		if (spr_load_ex[1]) x_x    <= spr_load_in_ex[10:3];
		if (spr_load_ex[3]) x_pix1 <= spr_load_in_ex[26:19];
		if (spr_load_ex[2] && extra_sprites) begin
			x_cnt <= x_cnt + 1'd1;
			if (x_cnt == 4'd0) begin
				x0_have <= 1;
				x0_hold <= {~spr_ex_dummy, x_attr, x_x, x_lin, x_pix1, spr_load_in_ex[18:11]};
			end else begin
				x_pend <= 1;
				x_wait <= 3'd4;
				x_idx  <= x_cnt[2:0];
				x_hold <= {~spr_ex_dummy, x_attr, x_x, x_lin, x_pix1, spr_load_in_ex[18:11]};
			end
		end
		// main entry 7 is being written this dot: decide extra entry 0
		if (spr_load[2] && s_cnt == 3'd7 && x0_have) begin
			x0_have <= 0;
			x0_pend <= 1;
			x0_wait <= 3'd6;
			if (!spr_dummy && x0_hold[53:0] == {s_attr, s_x, s_lin, s_pix1, spr_load_in[18:11]})
				x0_hold[54] <= 1'b0;
		end
		if (x_pend) begin
			if (x_wait == 3'd1) begin
				x_pend      <= 0;
				spr_ex_we   <= 1;
				spr_ex_idx  <= x_idx;
				spr_ex_data <= x_hold;
			end else
				x_wait <= x_wait - 1'd1;
		end
		if (x0_pend) begin
			if (x0_wait == 3'd1) begin
				x0_pend     <= 0;
				spr_ex_we   <= 1;
				spr_ex_idx  <= 3'd0;
				spr_ex_data <= x0_hold;
			end else
				x0_wait <= x0_wait - 1'd1;
		end

		// --- line boundary ---
		if (cycle == 9'd256) begin
			line_done <= 1;
			line_y    <= scanline;
			line_pre  <= pre_line;
			slot_cnt  <= 0;
			s_cnt     <= 0;
			x_cnt     <= 0;
			x0_have   <= 0;
		end
	end
end

endmodule
