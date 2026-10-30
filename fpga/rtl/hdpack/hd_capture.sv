// hd_capture.sv - writes a record of every NES frame into DDR3 (stage 2, CAPTURE).
//
// Mesen renders a frame only after it is complete, from a per-pixel record of that frame
// (MESEN_HD_RUNTIME_SPEC.md section 0 and 1). This module builds the equivalent record from
// the PPU tap and writes it to DDR3, double-buffered: frame N goes to buffer N&1, so the
// renderer can read frame N while frame N+1 is captured, and a pixel that Mesen leaves
// untouched (branch B) keeps the value this buffer held two frames ago, exactly as Mesen's
// two ScreenTiles arrays do.
//
// Record layout, byte offsets inside one buffer (buffer = REC_BASE + 0x100000 * (N & 1)):
//
//   0x00000 + y*1024 + x*4     pixel (4 bytes)
//       b0 [5:0] BG span slot (63 = NoTile)   [6] sprites shown at this dot
//       b1 [1:0] BgColorIndex   [4:2] OffsetX (column within the tile)
//       b2 PpuBackgroundColor (palette 0, mirrors resolved)
//       b3 BgColor (backdrop if index 0, else palette[tile palette + index])
//       Branch B (rendering off and v in $3F00-$3FFF) writes b0 = 0x3F ONLY.
//   0x40000 + y*2048           line header
//       q0 [14:0] t  [17:15] fine X  [18] grayscale  [21:19] emphasis   (all as of x = 0;
//                  bytes 0-2 not written if x = 0 was branch B)
//          [24] x = 0 was branch A   [39:32] BG span count   [47:40] sprites (valid)
//          [55:48] sprite fetches   [63:56] y
//       q1-q3 raw palette RAM, 32 x 6 bits, entry n at bits [6n+5:6n] of {q3,q2,q1}
//   0x40000 + y*2048 + 64 + s*16      BG span s (fetch group, 0..33)
//       qA [21:0] CHR linear address of the pattern fetch   [33:32] attribute palette
//          [63] 1
//       qB fingerprint of the tile (CHR-RAM games), else 0
//   0x40000 + y*2048 + 640 + i*16     sprite fetch i (0..63)
//       qA [54:0] {valid, attr, x, CHR linear address, pattern low, pattern high}
//       qB fingerprint (CHR-RAM games), else 0
//   0xF0000                    frame trailer: q0 = frame number
//
// Frame number = 1 for the first frame after reset (spec 2.3). The trailer and frame_done are
// produced when line 239 finishes, the point where Mesen calls SendFrame (scanline 240 dot 0).
//
// Writes leave through a FIFO as {qword address, data, byte enables}. A producer that finds
// its slot still busy (the FIFO is backed up) sets `overflow`; the record is then incomplete
// and that is reported, never hidden.

module hd_capture #(
	parameter [27:0] REC_BASE = 28'hC000000,  // byte offset in the 0x30000000 window
	parameter        FP_AW    = 15             // hd_chrfp address width
)
(
	input             clk,
	input             reset,
	input             enable,

	input             chr_ram,                 // game uses CHR RAM: store fingerprints

	// PPU tap (one-clock pulses)
	input             bg_we,
	input       [5:0] bg_slot,
	input      [21:0] bg_lin,
	input       [1:0] bg_pal,

	input             spr_we,
	input       [5:0] spr_idx,
	input      [54:0] spr_data,

	input             px_we,
	input       [7:0] px_x,
	input       [5:0] px_slot,
	input       [2:0] px_fx,
	input       [3:0] px_bgc,                  // {tile palette, colour index}
	input             px_bg_on,
	input             px_spr_on,
	input             px_branch_b,
	input      [14:0] px_t,
	input       [2:0] px_finex,
	input             px_gray,
	input       [2:0] px_emph,

	input     [191:0] pal_all,                 // raw palette RAM, entry n at [6n+5:6n]

	input             line_done,
	input       [8:0] line_y,
	input             line_pre,                // line_done of the pre-render line

	// hd_chrfp read port (one clock latency)
	output reg [FP_AW-5:0] fp_tile = 0,
	input      [63:0] fp_q,

	// write FIFO output
	output            wq_valid,
	output     [27:3] wq_addr,
	output     [63:0] wq_data,
	output      [7:0] wq_be,
	input             wq_ready,

	output reg        frame_done = 0,          // pulse: frame frame_num is complete
	output reg [31:0] frame_num = 1,           // frame being captured (after the pulse: next)
	output reg        overflow = 0
);

localparam [27:0] HDR_OFF = 28'h40000;
localparam [27:0] TRL_OFF = 28'hF0000;

function [27:3] qw(input [27:0] b);                       // byte offset -> qword address
	qw = b[27:3];
endfunction

// ReadPaletteRam for the BG half only: entries 0-15 are never mirrored ($10/$14/$18/$1C ->
// $00/... applies to the sprite half, which the capture does not read; the renderer applies
// it when it resolves sprite colours from the line palette).
function [5:0] pal_at(input [191:0] p, input [3:0] a);
	pal_at = p[a*6 +: 6];
endfunction

// ------------------------------------------------------------------------------------------
// line bookkeeping
// ------------------------------------------------------------------------------------------
reg   [8:0] build_line = 9'h1FF;             // line the fetches/pixels belong to; >=240 = none
wire        build_ok = !build_line[8] && (build_line < 9'd240);
wire  [7:0] bl = build_line[7:0];
wire [27:0] buf_base = REC_BASE + (frame_num[0] ? 28'h100000 : 28'h0);

reg   [7:0] cnt_span, cnt_spr_valid, cnt_spr;
reg  [14:0] h_t;
reg   [2:0] h_finex, h_emph;
reg         h_gray, h_a;
reg [191:0] h_pal;

// ------------------------------------------------------------------------------------------
// producer slots
// ------------------------------------------------------------------------------------------
// pixel pair
reg         pix_v = 0;
reg  [27:3] pix_a;
reg  [63:0] pix_d;
reg   [7:0] pix_be;
reg  [31:0] even_d;
reg   [3:0] even_be;
reg         even_ok = 0;

// table entry (BG span or sprite): waits two clocks for the fingerprint, then 2 qwords
reg   [1:0] ent_st = 0;                      // 0 idle, 1/2 waiting for fp, 3 ready
reg         ent_q = 0;                       // next qword to send: 0 = qA, 1 = qB
reg  [27:3] ent_a;
reg  [63:0] ent_qa, ent_qb;

// line header: 4 qwords
reg         hdr_v = 0;
reg   [1:0] hdr_q = 0;
reg  [27:3] hdr_a;
reg  [63:0] hdr_d0;
reg   [7:0] hdr_be0;
reg [191:0] hdr_pal;

// trailer
reg         trl_v = 0;
reg  [27:3] trl_a;
reg  [63:0] trl_d;

// ------------------------------------------------------------------------------------------
// FIFO (first-word fall-through)
// ------------------------------------------------------------------------------------------
localparam FW = 25 + 64 + 8;
localparam FD = 8;                           // 256 entries

reg  [FW-1:0] fmem [1 << FD];
reg    [FD:0] fwp = 0, frp = 0;
// the count must be taken at the pointers' own width: compared directly against an integer,
// the subtraction is done in 32 bits and goes negative once the write pointer wraps, so the
// FIFO is never seen full and overwrites unread entries (found by the starvation test)
wire   [FD:0] fcount = fwp - frp;
wire          ffull  = fcount[FD];
reg           fpush;
reg  [FW-1:0] fin;

reg           o_v = 0;
reg  [FW-1:0] o_d;
wire          f_empty = (fwp == frp);
wire          o_take = o_v & wq_ready;

assign wq_valid = o_v;
assign {wq_addr, wq_data, wq_be} = o_d;

always @(posedge clk) begin
	if (fpush) begin
		fmem[fwp[FD-1:0]] <= fin;
		fwp <= fwp + 1'd1;
	end
	if ((!o_v || o_take) && !f_empty) begin
		o_d <= fmem[frp[FD-1:0]];
		o_v <= 1;
		frp <= frp + 1'd1;
	end else if (o_take) begin
		o_v <= 0;
	end
	if (reset) begin
		fwp <= 0;
		frp <= 0;
		o_v <= 0;
	end
end

// space must leave room for this clock's push even while the head is being refilled
wire f_space = !ffull;

// ------------------------------------------------------------------------------------------
// main
// ------------------------------------------------------------------------------------------
reg  [3:0] pidx;
reg  [5:0] pb2, pb3;
reg  [1:0] cidx;
reg  [31:0] pxd;
reg  [3:0]  pxbe;

// this clock's drain, decided first so a producer refilling its slot on the same clock is
// not mistaken for an overflow
wire d_pix = f_space && pix_v;
wire d_ent = f_space && !pix_v && ent_st == 2'd3;
wire d_hdr = f_space && !pix_v && ent_st != 2'd3 && hdr_v;
wire d_trl = f_space && !pix_v && ent_st != 2'd3 && !hdr_v && trl_v;
wire pix_busy = pix_v && !d_pix;
wire ent_busy = (ent_st != 0) && !(d_ent && ent_q);
wire hdr_busy = hdr_v && !(d_hdr && hdr_q == 2'd3);
wire trl_busy = trl_v && !d_trl;

// counts including an event on the same clock as line_done (slot 33 lands at dot 256)
wire       ev_ok   = enable && build_ok;
wire [7:0] span_n  = (ev_ok && bg_we && ({2'b0, bg_slot} + 8'd1 > cnt_span)) ? {2'b0, bg_slot} + 8'd1 : cnt_span;
wire [7:0] spr_n   = (ev_ok && spr_we) ? cnt_spr + 1'd1 : cnt_spr;
wire [7:0] sprv_n  = (ev_ok && spr_we && spr_data[54]) ? cnt_spr_valid + 1'd1 : cnt_spr_valid;

always @(posedge clk) begin
	frame_done <= 0;
	fpush <= 0;

	// ---- drain one qword per clock, fixed priority ----
	if (f_space) begin
		if (pix_v) begin
			fin   <= {pix_a, pix_d, pix_be};
			fpush <= 1;
			pix_v <= 0;
		end else if (ent_st == 2'd3) begin
			fin   <= {ent_a + (ent_q ? 25'd1 : 25'd0), ent_q ? ent_qb : ent_qa, 8'hFF};
			fpush <= 1;
			if (ent_q) ent_st <= 0;
			ent_q <= ~ent_q;
		end else if (hdr_v) begin
			case (hdr_q)
				2'd0: fin <= {hdr_a,          hdr_d0,           hdr_be0};
				2'd1: fin <= {hdr_a + 25'd1,  hdr_pal[63:0],    8'hFF};
				2'd2: fin <= {hdr_a + 25'd2,  hdr_pal[127:64],  8'hFF};
				2'd3: fin <= {hdr_a + 25'd3,  hdr_pal[191:128], 8'hFF};
			endcase
			fpush <= 1;
			hdr_q <= hdr_q + 1'd1;
			if (hdr_q == 2'd3) hdr_v <= 0;
		end else if (trl_v) begin
			fin   <= {trl_a, trl_d, 8'hFF};
			fpush <= 1;
			trl_v <= 0;
		end
	end

	// ---- fingerprint wait ----
	if (ent_st == 2'd1) ent_st <= 2'd2;
	if (ent_st == 2'd2) begin
		ent_qb <= chr_ram ? fp_q : 64'd0;
		ent_st <= 2'd3;
	end

	if (enable) begin
		// ---- pixels ----
		if (px_we && build_ok) begin
			cidx = px_bg_on ? px_bgc[1:0] : 2'd0;
			pidx = {px_bgc[3:2], cidx};
			pb2  = pal_at(pal_all, 4'd0);
			pb3  = (cidx == 2'd0) ? pb2 : pal_at(pal_all, pidx);
			if (px_branch_b) begin
				pxd  = 32'h3F;
				pxbe = 4'b0001;
			end else begin
				pxd  = {2'b0, pb3, 2'b0, pb2, 3'b0, px_fx, cidx,
				        1'b0, px_spr_on, px_bg_on ? px_slot : 6'd63};
				pxbe = 4'b1111;
			end

			if (!px_x[0]) begin
				even_d  <= pxd;
				even_be <= pxbe;
				even_ok <= 1;
			end else begin
				if (pix_busy) overflow <= 1;
				pix_v  <= 1;
				pix_a  <= qw(buf_base + {10'd0, bl, 10'd0} + {18'd0, px_x[7:1], 3'd0});
				pix_d  <= {pxd, even_ok ? even_d : 32'd0};
				pix_be <= {pxbe, even_ok ? even_be : 4'b0000};
				even_ok <= 0;
			end

			if (px_x == 8'd0) begin
				h_a <= !px_branch_b;
				if (!px_branch_b) begin
					h_t     <= px_t;
					h_finex <= px_finex;
					h_gray  <= px_gray;
					h_emph  <= px_emph;
				end
			end
			// palette cannot change while rendering; snapshot it on any rendered dot
			if (!px_branch_b && (px_bg_on || px_spr_on)) h_pal <= pal_all;
		end

		// ---- BG span / sprite entries ----
		if ((bg_we || spr_we) && build_ok) begin
			if (ent_busy) overflow <= 1;
			ent_st <= 2'd1;
			ent_q  <= 0;
			if (bg_we) begin
				ent_a   <= qw(buf_base + HDR_OFF + {9'd0, bl, 11'd0} + 28'd64 + {18'd0, bg_slot, 4'd0});
				ent_qa  <= {1'b1, 29'd0, bg_pal, 10'd0, bg_lin};
				fp_tile <= bg_lin[FP_AW-1:4];
			end else begin
				ent_a   <= qw(buf_base + HDR_OFF + {9'd0, bl, 11'd0} + 28'd640 + {18'd0, spr_idx, 4'd0});
				ent_qa  <= {9'd0, spr_data};
				fp_tile <= spr_data[16 + FP_AW - 1 : 16 + 4];
			end
		end
		cnt_span      <= span_n;
		cnt_spr       <= spr_n;
		cnt_spr_valid <= sprv_n;

		// ---- line end ----
		if (line_done) begin
			if (!line_y[8] && line_y < 9'd240) begin
				if (hdr_busy) overflow <= 1;
				hdr_v   <= 1;
				hdr_q   <= 0;
				hdr_a   <= qw(buf_base + HDR_OFF + {9'd0, line_y[7:0], 11'd0});
				hdr_d0  <= {line_y[7:0], spr_n, sprv_n, span_n,
				            7'd0, h_a, 2'd0, h_emph, h_gray, h_finex, h_t};
				hdr_be0 <= h_a ? 8'hFF : 8'hF8;
				hdr_pal <= h_pal;
			end
			cnt_span      <= 0;
			cnt_spr       <= 0;
			cnt_spr_valid <= 0;
			even_ok       <= 0;

			if (line_y == 9'd239) begin
				if (trl_busy) overflow <= 1;
				trl_v      <= 1;
				trl_a      <= qw(buf_base + TRL_OFF);
				trl_d      <= {32'd0, frame_num};
				frame_done <= 1;
				frame_num  <= frame_num + 1'd1;
			end

			if (line_pre)                               build_line <= 9'd0;
			else if (!line_y[8] && line_y < 9'd239)     build_line <= line_y + 1'd1;
			else                                        build_line <= 9'h1FF;
		end
	end

	if (reset) begin
		build_line <= 9'h1FF;
		frame_num  <= 1;
		pix_v      <= 0;
		ent_st     <= 0;
		hdr_v      <= 0;
		trl_v      <= 0;
		even_ok    <= 0;
		overflow   <= 0;
		cnt_span   <= 0;
		cnt_spr    <= 0;
		cnt_spr_valid <= 0;
	end
end

endmodule
