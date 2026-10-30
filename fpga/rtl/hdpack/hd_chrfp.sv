// hd_chrfp.sv - CHR-RAM shadow and per-tile fingerprints (stage 2).
//
// For CHR-RAM games Mesen keys a tile by its 16 data bytes, read from CHR RAM when the pixel
// is drawn (MESEN_HD_RUNTIME_SPEC.md 1.3/1.4). The pack stores those keys as a CRC-64 of the
// 16 bytes (the "fingerprint"), so the engine needs the fingerprint of any tile at any time.
//
// This module keeps:
//   - a shadow of CHR RAM (2^AW bytes), written whenever the core writes CHR RAM (PPU writes
//     through $2007, and savestate restores), so it equals the SDRAM copy;
//   - one 64-bit fingerprint per 16-byte tile, recomputed in the background whenever a byte
//     of the tile changes. Each tile has a dirty bit; the scanner clears it BEFORE it reads
//     the tile's bytes, so a write landing during the recompute dirties the tile again and it
//     is redone.
// The dirty bits live in block RAM (one bit per tile). Its single write port is shared: a core
// write (set) always wins; the scanner's clear and the reset sweep (set every bit) wait for a
// free clock. A clear that lands after a set to the same tile is harmless: the set's byte is
// already in the shadow, and the recompute that follows reads it. The scanner reads one bit
// every 2 clocks; a set that collides with that read is missed by THIS pass and found by the
// next, which is why `idle` needs a complete clean pass with no write since.
// A recompute takes 18 clocks. CPU writes to $2007 are at least 4 CPU cycles (48 core clocks)
// apart, so the background keeps up with any game; a savestate restore of 8 KB (512 tiles)
// is redone in about 10,000 clocks, well inside a frame. Reset marks every tile dirty (a
// 2^(AW-4)-clock sweep), so all fingerprints are valid about 2 ms after reset at 32 KB.
//
// Fingerprint = CRC-64 (poly 0x42F0E1EBA9EA3693, init 0, MSB first, no final XOR) over the 16
// bytes in address order; must match tools/hdp3_convert.py crc64.
//
// Difference from Mesen, by design: Mesen copies the bytes for every PIXEL; the engine uses
// the fingerprint when it looks the tile up. They differ only if CHR RAM is written while the
// PPU renders, which real hardware cannot do cleanly (a $2007 write during rendering corrupts
// the PPU address), so no game relies on it.

module hd_chrfp #(
	parameter AW = 15                     // shadow address bits: 2^15 = 32 KB of CHR RAM
)
(
	input             clk,
	input             reset,

	// writes into CHR RAM (byte offset within the CHR-RAM region)
	input             wr,
	input    [AW-1:0] wr_addr,
	input       [7:0] wr_data,

	// fingerprint read: one clock latency
	input    [AW-5:0] fp_tile,
	output reg [63:0] fp_q,

	output            idle                // no tile waiting for, or in, a recompute
);

localparam NT = 1 << (AW - 4);           // tiles
localparam TB = AW - 4;                  // tile index bits

function [63:0] crc64_byte(input [63:0] r, input [7:0] b);
	integer i;
	reg [63:0] x;
	begin
		x = r;
		for (i = 7; i >= 0; i = i - 1)
			x = {x[62:0], 1'b0} ^ ((x[63] ^ b[i]) ? 64'h42F0E1EBA9EA3693 : 64'd0);
		crc64_byte = x;
	end
endfunction

// shadow: written by the core, read by the recompute
reg    [7:0] shadow [1 << AW];
reg    [7:0] sh_q;
reg [AW-1:0] sh_ra = 0;
always @(posedge clk) if (wr) shadow[wr_addr] <= wr_data;
always @(posedge clk) sh_q <= shadow[sh_ra];

// fingerprints: written by the recompute, read by the engine
reg   [63:0] fp [NT];
reg          fp_we = 0;
reg [AW-5:0] fp_wa = 0;
reg   [63:0] fp_wd = 0;
always @(posedge clk) if (fp_we) fp[fp_wa] <= fp_wd;
always @(posedge clk) fp_q <= fp[fp_tile];

// dirty bits: block RAM, one bit per tile
typedef enum bit [2:0] { R_INIT, R_SCAN, R_CLEAR, R_READ, R_DONE } rstate_t;
rstate_t rs /*verilator public_flat_rd*/ = R_INIT;   // tb_chrfp aims writes at the scanner

reg    dbits [NT];
reg    d_q;
reg [TB-1:0] scan_p /*verilator public_flat_rd*/ = 0;   // tile the scanner is looking at
reg          ph     /*verilator public_flat_rd*/ = 0;   // 0: bit being read, 1: d_q holds it
reg [TB-1:0] init_p = 0;
// Consecutive clean tiles seen since the last write. idle needs NT+1: the first tile examined
// after a write may have been read in the same clock as that write (old data), so a full pass
// that also re-reads it is needed before nothing can be pending.
reg [TB+1:0] clean = 0;
localparam [TB+1:0] CLEAN_ALL = NT + 1;
reg [TB-1:0] r_tile = 0;
reg    [4:0] r_byte = 0;
reg   [63:0] r_crc = 0;

assign idle = (clean == CLEAN_ALL) && (rs == R_SCAN);

wire [TB-1:0] w_tile = wr_addr[AW-1:4];

// the one write port: a core write sets, else the scanner clears, else the reset sweep sets
reg          dw_en;
reg [TB-1:0] dw_a;
reg          dw_d;
always @* begin
	dw_en = 1'b0; dw_a = w_tile; dw_d = 1'b1;
	if (wr)                  begin dw_en = 1'b1; dw_a = w_tile; dw_d = 1'b1; end
	else if (rs == R_CLEAR)  begin dw_en = 1'b1; dw_a = r_tile; dw_d = 1'b0; end
	else if (rs == R_INIT)   begin dw_en = 1'b1; dw_a = init_p; dw_d = 1'b1; end
end
always @(posedge clk) if (dw_en) dbits[dw_a] <= dw_d;
always @(posedge clk) d_q <= dbits[scan_p];

integer k;
initial for (k = 0; k < NT; k = k + 1) dbits[k] = 1'b1;

always @(posedge clk) begin
	fp_we <= 0;

	// a write can make any tile dirty, including one already passed this round
	if (wr) clean <= 0;

	case (rs)
		R_INIT: if (!wr) begin
			init_p <= init_p + 1'd1;
			if (&init_p) begin rs <= R_SCAN; ph <= 0; end
		end

		R_SCAN: begin
			ph <= ~ph;
			if (ph) begin
				if (d_q) begin
					clean  <= 0;
					r_tile <= scan_p;
					rs     <= R_CLEAR;
				end else begin
					if (!wr && clean != CLEAN_ALL) clean <= clean + 1'd1;
					scan_p <= scan_p + 1'd1;
				end
			end
		end

		R_CLEAR: if (!wr) begin            // the clear takes the port this clock
			sh_ra  <= {r_tile, 4'd0};
			r_byte <= 0;
			r_crc  <= 0;
			rs     <= R_READ;
		end

		R_READ: begin
			// sh_q holds byte r_byte-1 (one clock read latency)
			if (r_byte != 0) r_crc <= crc64_byte(r_crc, sh_q);
			if (r_byte < 5'd15) sh_ra <= sh_ra + 1'd1;
			r_byte <= r_byte + 5'd1;
			if (r_byte == 5'd16) rs <= R_DONE;
		end

		R_DONE: begin
			fp_we  <= 1;
			fp_wa  <= r_tile;
			fp_wd  <= r_crc;
			scan_p <= r_tile + 1'd1;
			ph     <= 0;
			rs     <= R_SCAN;
		end

		default: rs <= R_INIT;
	endcase

	if (reset) begin
		rs     <= R_INIT;
		init_p <= 0;
		clean  <= 0;
	end
end

endmodule
