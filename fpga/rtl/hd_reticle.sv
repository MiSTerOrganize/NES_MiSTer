// hd_reticle.sv - the Zapper crosshair drawn on the HD pack picture.
//
// video.sv draws the crosshair into the NES picture (palette $15, or $21 when aimed off-screen;
// zapper.sv decides when). The HD picture replaces the NES picture, so without this a Zapper
// game played with Zapper(Joy1/Joy2) or Zapper(Mouse) has nothing to aim with. This draws the
// same plus sign at the same screen position, coloured from the HD engine's own palette
// (rtl/hdpack/hd3_pal2c02.hex). Only the drawing is added: the gun senses light from the NES
// picture (zapper.sv reads the PPU colour index), exactly as on the console.
//
// The HD picture is one HD pixel per clk, 512 x 480 active, 2 x 2 HD pixels per NES pixel
// (hd3_scanout.sv), so the NES pixel is the active pixel / line count halved. HB/VB come with
// R/G/B in the same clock, so the counters index the pixel currently on R/G/B.
// Used by NES.sv and by tools/fullsim/sim_top.sv, so the simulation runs this exact code.

module hd_reticle
(
	input        clk,
	input        enable,     // a light gun is on and the Crosshairs option is On
	input  [7:0] aim_x,      // NES pixel the gun points at (zapper.sv)
	input  [7:0] aim_y,
	input        hb,
	input        vb,
	input  [7:0] r_in, g_in, b_in,
	output [7:0] r_out, g_out, b_out
);

reg [9:0] px = 0, ln = 0;
reg       hb_d = 1;
always @(posedge clk) begin
	hb_d <= hb;
	if (hb) px <= 0;
	else    px <= px + 1'd1;
	if (vb)              ln <= 0;
	else if (hb & ~hb_d) ln <= ln + 1'd1;   // an active line just ended
end

// zapper.sv tests the PPU dot counter (cycle), and screen column = cycle - 2 (the NES picture is two
// dots behind the counter - the same offset fullsim's --xoff 2 is calibrated to, and the HD picture is
// lined up column for column with the NES picture). So the gun senses, and video.sv draws its plus,
// at screen column aim_x - 2; the row is not offset. Signed, so an aim at x 0..1 lands off the left
// edge exactly as the NES picture's plus does.
wire signed [10:0] nx = {2'b00, px[9:1]};
wire signed [10:0] ny = {2'b00, ln[9:1]};
wire signed [10:0] zx = {3'b000, aim_x} - 11'sd2;
wire signed [10:0] zy = {3'b000, aim_y};

// zapper.sv: hit_x = |aim_x - cycle| <= 1 on the aim row, hit_y = |aim_y - scanline| <= 1 on the aim column
wire on_mark = enable && !hb && !vb &&
	(((nx + 11'sd1 >= zx) && (zx + 11'sd1 >= nx) && ny == zy) ||
	 ((ny + 11'sd1 >= zy) && (zy + 11'sd1 >= ny) && nx == zx));
// zapper.sv is_offscreen
wire off = (aim_x >= 8'd254 || aim_x <= 8'd1) || (aim_y >= 8'd224 || aim_y <= 8'd8);
wire [23:0] rgb = off ? 24'h64b0ff : 24'hb71e7b;   // hd3_pal2c02.hex entries $21 and $15

assign r_out = on_mark ? rgb[23:16] : r_in;
assign g_out = on_mark ? rgb[15:8]  : g_in;
assign b_out = on_mark ? rgb[7:0]   : b_in;

endmodule
