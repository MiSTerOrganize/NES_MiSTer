// hd3_fmap.sv - frame maps for RENDER's positional conditions (HDPACK_DESIGN.md "hd3_render",
// "Frame budget and the dictionary read port").
//
// A tileNearby / spriteNearby / tileAtPosition condition can name ANY pixel of the frame (the
// corpus reaches |Y| = 224, |X| = 176), so RENDER keeps a copy of what it needs about every
// pixel on chip:
//   pixel map    61,440 x {sprites shown, BG slot}   (byte 0 of each frame-record pixel)
//   line table   240 x {span count, fetch count}      (header q0, capped as hd3_dict caps)
//   sprite table 240 x 16 fetches x {valid, bgp, X, pattern bits}  (sprite qword A)
// The record ids, palettes and keys come from hd3_dict; these maps keep two banks too, one per
// frame-record buffer. They are written by snooping the capture writer's record stream (the
// same stream hd3_dict takes), so a map always holds exactly what the record buffer in DDR3
// holds for those fields: every word the writer stores lands in both, and a word it does not
// store (a sprite slot past the line's fetch count) is stale in both. RENDER reads only up to
// the line's fetch count. Filling as the frame is captured costs nothing in vblank and no DDR3
// reads (the earlier version re-read the finished record: 100,802 clocks a frame, which did not
// fit beside ADD and RESOLVE in the vblank window).
//
// Byte enables matter for pixels only: a pixel qword holds two pixels and the writer stores
// them with per-pixel enables (a missing even pixel, or a branch-B pixel that writes byte 0
// only). Byte 0 of each pixel is all the map keeps, so each half has its own write enable from
// byte enable 0 / 4. Header q0 bytes 3-7 and sprite qwords are always written whole.
//
// Memories: one write port, registered read (Quartus block-RAM shape). The sprite table is 16
// lanes, one per fetch, so a line's 16 entries read in one clock (RENDER finds a pixel's
// sprites with 16 comparators).

module hd3_fmap #(
	parameter [27:0] REC_BASE = 28'hC000000
)
(
	input             clk,

	// tap of the capture writer's record stream (one qword per clock when tap_v)
	input             tap_v,
	input      [27:3] tap_a,
	input      [63:0] tap_d,
	input       [7:0] tap_be,

	// read ports, one clock latency, from bank r_bank (RENDER's frame)
	input             r_bank,
	input      [14:0] p_a,              // y * 128 + x / 2
	output     [13:0] p_q,              // {pixel x|1, pixel x&~1}, each {shown, slot[5:0]}
	input       [7:0] l_y,
	output reg [10:0] l_q,              // {fetch count 5, span count 6}
	input       [7:0] s_y,
	output    [415:0] s_q               // 16 x {valid, bgp, X 8, pattern 1 8, pattern 0 8}
);

// ------------------------------------------------------------------------------------------
// tap decode (the record layout, HDPACK_DESIGN.md "Frame record")
// ------------------------------------------------------------------------------------------
// (hd3_dict's decode, the same bit numbering: rel is [27:3], so rel[20] is the 1 MB buffer)
wire [27:3] rel    = tap_a - REC_BASE[27:3];
wire        in_rec = rel[27:21] == 7'd0;              // the two 1 MB buffers
wire        t_bank = rel[20];
wire [16:0] t_q    = rel[19:3];                         // qword inside the buffer
wire [16:0] t_l    = t_q - 17'h08000;                   // 0x40000 / 8: line blocks
wire        t_pix  = in_rec && t_q < 17'h07800;          // 240 lines x 128 pixel qwords
wire        t_line = in_rec && t_q >= 17'h08000 && t_l[15:8] < 8'd240 && !t_l[16];
wire  [7:0] t_y    = t_l[15:8];
wire  [7:0] t_o    = t_l[7:0];
wire  [7:0] t_f    = t_o - 8'd80;                       // sprite area: fetch * 2 + {A, B}
wire        t_spr  = t_line && t_o >= 8'd80 && t_o < 8'd112 && !t_f[0];

// ------------------------------------------------------------------------------------------
// memories
// ------------------------------------------------------------------------------------------
// bank 1 starts at 32768 (bank in bit 15), so 65536 entries; 4096 per bank are unused (a
// 61,440-entry array would lose bank 1's lines 224..239)
reg [6:0] pm_e [65536];
reg [6:0] pm_o [65536];
reg [6:0] pq_e, pq_o;
wire [15:0] pw_a = {t_bank, t_q[14:0]};
always @(posedge clk) begin
	if (tap_v && t_pix && tap_be[0]) pm_e[pw_a] <= tap_d[6:0];
	pq_e <= pm_e[{r_bank, p_a}];
end
always @(posedge clk) begin
	if (tap_v && t_pix && tap_be[4]) pm_o[pw_a] <= tap_d[38:32];
	pq_o <= pm_o[{r_bank, p_a}];
end
assign p_q = {pq_o, pq_e};

reg [10:0] lmem [512];
always @(posedge clk) begin
	if (tap_v && t_line && t_o == 8'd0)
		lmem[{t_bank, t_y}] <= {(tap_d[55:48] > 8'd16) ? 5'd16 : tap_d[52:48],
		                        (tap_d[39:32] > 8'd34) ? 6'd34 : tap_d[37:32]};
	l_q <= lmem[{r_bank, l_y}];
end

// fetch i: qword A {valid 54, attr 53:46 (bgp = attr bit 5 = 51), X 45:38, ..., p1, p0}
wire [25:0] sw_d = {tap_d[54], tap_d[51], tap_d[45:38], tap_d[15:8], tap_d[7:0]};
genvar gi;
generate for (gi = 0; gi < 16; gi = gi + 1) begin : lane
	reg [25:0] m [512];
	reg [25:0] q;
	always @(posedge clk) begin
		if (tap_v && t_spr && t_f[4:1] == gi) m[{t_bank, t_y}] <= sw_d;
		q <= m[{r_bank, s_y}];
	end
	assign s_q[26 * gi +: 26] = q;
end endgenerate

endmodule
