// hd3_scanout.sv - the stage-3 HD picture: an 8-line ring written by hd3_render on the HD clock
// and read out on the core clock, two 512-pixel lines per NES line.
//
// Timing is the MVP's (hd_engine.sv): hcnt counts 0..1363 core clocks per NES line from the
// tap's line_done, the first 682 clocks are output line 2y and the rest output line 2y + 1,
// pixels 0..511 are active, HS is pixels 544..607, VB is line y >= 240 and VS lines 245..246.
// The line shown from line_done of NES line L + 1 is HD line L of the frame RENDER finished before - the
// picture is one frame behind the console, as Mesen's is (it renders a frame once it is
// complete).
//
// Frame identity: dcount counts the visible-to-vblank transitions of the scan-out. The frame
// shown in the visible lines after the k-th transition has id k (2 bits, modulo 4). RENDER
// tags the frame it draws with the id of the frame it will be shown as (hd3_engine), and
// publishes on the HD clock how far it has got: p_cur_t / p_cur_y (frame being drawn, every
// line < p_cur_y complete) and p_fin_t / p_fin_v (the last frame drawn to the end). A line is
// shown only if it is complete for the frame id being shown; otherwise it is black and
// `underflow` is set (sticky) - the ring would hold another frame's line or a half-written one.
// Pixels outside the pack's overscan window are black (RENDER draws only inside it).
//
// The ring is 2048 x 96 (the 2x2 block of each NES pixel, {row1 col1, row1 col0, row0 col1,
// row0 col0}, RRGGBB each): one write port on clk_f, one registered read on clk, the dual-
// clock shape Quartus maps to block RAM (20 M10K at 2048 x 5).

module hd3_scanout
(
	// HD clock: RENDER's output and progress
	input             clk_f,
	input             w_en,
	input       [7:0] w_y,
	input       [7:0] w_x,
	input      [95:0] w_px,
	input       [1:0] p_cur_t,
	input       [8:0] p_cur_y,
	input       [1:0] p_fin_t,
	input             p_fin_v,
	input             p_run,          // a frame has been started (p_cur_* mean something)

	// core clock
	input             clk,
	input             line_done,      // tap[114]
	input       [8:0] line_y,         // tap[123:115]
	input       [7:0] ov_top,
	input       [7:0] ov_bottom,
	input       [7:0] ov_left,
	input       [7:0] ov_right,

	output reg  [8:0] disp_y = 9'd511, // HD line being shown (>= 240: blanking)
	output reg  [1:0] dcount = 0,      // frame id (see above)
	output reg        underflow = 0,

	output reg  [7:0] R,
	output reg  [7:0] G,
	output reg  [7:0] B,
	output reg        HS,
	output reg        VS,
	output reg        HB,
	output reg        VB
);

reg [95:0] ring [2048];
always @(posedge clk_f) if (w_en) ring[{w_y[2:0], w_x}] <= w_px;

// disp_y lags line_y by one line_done, exactly as the MVP's out_y (job_y <= line_y,
// out_y <= job_y), so the output's sync geometry against the NES frame is unchanged
reg  [8:0] job_y = 9'd511;
reg [10:0] hcnt = 0;
always @(posedge clk) begin
	hcnt <= line_done ? 11'd0 : (hcnt == 11'd1363 ? 11'd0 : hcnt + 11'd1);
	if (line_done) begin
		job_y  <= line_y;
		disp_y <= job_y;
		if (disp_y < 9'd240 && job_y >= 9'd240) dcount <= dcount + 2'd1;
	end
end

wire        o_sub = (hcnt >= 11'd682);
wire [10:0] o_pxw = o_sub ? (hcnt - 11'd682) : hcnt;
wire  [9:0] o_px  = o_pxw[9:0];
wire  [7:0] o_x   = o_px[8:1];
wire  [8:0] ybot  = 9'd239 - {1'b0, ov_bottom};
wire  [8:0] xr    = 9'd255 - {1'b0, ov_right};
wire        o_act = (o_px < 10'd512) && (disp_y < 9'd240);
wire        o_win = (disp_y >= {1'b0, ov_top}) && (disp_y <= ybot) &&
                    ({1'b0, o_x} >= {1'b0, ov_left}) && ({1'b0, o_x} <= xr);
// a line is shown or blanked WHOLE: the decision is taken at its first clock (hcnt 0, disp_y
// already updated) and held, so a line RENDER completes while it is being scanned stays black
// instead of appearing part-way through
wire        o_ok0 = (p_run && p_cur_t == dcount && p_cur_y > disp_y) || (p_fin_v && p_fin_t == dcount);
reg         l_ok  = 0;
wire        o_ok  = (hcnt == 11'd0) ? o_ok0 : l_ok;
always @(posedge clk) l_ok <= o_ok;

// stage 1: ring read
reg [95:0] q;
reg  [1:0] s1_sub;
reg        s1_act, s1_show, s1_hs, s1_hb, s1_vs, s1_vb;
reg        armed = 0;
always @(posedge clk) begin
	q       <= ring[{disp_y[2:0], o_x}];
	s1_sub  <= {o_sub, o_px[0]};
	s1_act  <= o_act;
	s1_show <= o_act && o_win && o_ok;
	s1_hb   <= (o_px >= 10'd512);
	s1_hs   <= (o_px >= 10'd544) && (o_px < 10'd608);
	s1_vb   <= (disp_y >= 9'd240);
	s1_vs   <= (disp_y >= 9'd245) && (disp_y < 9'd247);
	// black before the scan-out reaches the first frame RENDER drew for it is the start-up
	// state, not a late line; from then on every shown frame must have been drawn in time
	if (p_run && p_cur_t == dcount) armed <= 1;
	if (o_act && o_win && !o_ok && armed) underflow <= 1;
	if (!p_run) armed <= 0;
end

// stage 2: the sub-pixel of the 2x2 block
wire [23:0] px = q[{s1_sub, 3'b000} * 3 +: 24];
always @(posedge clk) begin
	{R, G, B} <= s1_show ? px : 24'd0;
	HS <= s1_hs;
	VS <= s1_vs;
	HB <= s1_hb;
	VB <= s1_vb;
end

endmodule
