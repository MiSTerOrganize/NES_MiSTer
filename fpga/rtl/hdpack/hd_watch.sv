// hd_watch.sv - watched-address snapshot for HD pack memory conditions (stage 2).
//
// Mesen reads every watched address at scanline 240 dot 0, without side effects, and every
// memory condition of that frame sees those values (MESEN_HD_RUNTIME_SPEC.md 2.2):
//   CPU address        the CPU's view (PeekRam through the current mapping)
//   PPU < $3F00        pattern tables / nametables through the current mapping
//   PPU >= $3F00       palette RAM, $10/$14/$18/$1C mirrored to $00/$04/$08/$0C
//
// Reading SDRAM for all of them at that moment would race the CPU, which keeps running, so the
// engine keeps its own copies of the memory that changes, fed by the same writes SDRAM gets:
//   CPU RAM    2 KB, CPU writes to $0000-$1FFF and savestate writes to its SDRAM region
//   PRG-RAM    8 KB, CPU writes to $6000-$7FFF (unbanked PRG-RAM; banked is a known limit)
//              and savestate writes to the SDRAM page the CPU last used for $6000-$7FFF
//   nametable  4 x 1 KB pages keyed by their SDRAM page (so mirroring changes and four-screen
//              are exact); the PPU-to-page mapping is the one the last nametable access used.
//              The two CIRAM pages are mapped from the start so savestate restores land.
// At scanline 240 dot 0 (snap) the copies FREEZE: pass 1 reads every watched address from them
// at one entry per clock, while writes arriving in the meantime wait in a FIFO and are applied
// afterwards. So every value is the one at dot 0.
//
// ROM content does not change, only its mapping, so PRG-ROM ($8000+) and pattern-table entries
// are read from SDRAM in pass 2 through the mapping latched at dot 0 (the bank the CPU last used
// for each 2 KB PRG page, the bank the PPU last used for each 1 KB pattern page). CHR-RAM
// written between dot 0 and that read would be seen; real games do not write CHR RAM there.
//
// Known limits (none occurs in the 133-pack corpus): CPU $2000-$5FFF reads 0 (Mesen: register
// PeekRam; the HD audio device's own registers do read 0); active Game Genie cheats are not
// applied to the snapshot (Mesen applies them to CPU reads).
//
// Values are double-buffered by frame parity: the snapshot taken while frame N is captured goes
// to bank `bank`, so the renderer reading frame N-1's values is never disturbed.

module hd_watch
(
	input             clk,
	input             reset,          // state machine only: the copies follow SDRAM across resets
	input             enable,

	// watched-address list, from the pack loader (u32 each, bit 31 = PPU address)
	input             lw_we,
	input       [7:0] lw_idx,
	input      [31:0] lw_addr,
	input       [8:0] list_n,

	// CPU side (nes.v): an access is cpu_rd or cpu_wr; cpu_a = CPU address, cpu_lin = SDRAM
	input             cpu_rd,
	input             cpu_wr,
	input      [15:0] cpu_a,
	input      [24:0] cpu_lin,
	input       [7:0] cpu_d,

	// PPU side (nes.v): ppu_a = PPU address, ppu_lin = SDRAM address
	input             ppu_rd,
	input             ppu_wr,
	input      [13:0] ppu_a,
	input      [21:0] ppu_lin,
	input       [7:0] ppu_d,

	// savestate restore writes (25-bit SDRAM address)
	input             ss_wr,
	input      [24:0] ss_addr,
	input       [7:0] ss_d,

	input     [191:0] pal_all,        // palette RAM, 32 x 6 bits
	input             snap,           // scanline 240 dot 0
	input             bank,           // value bank for this snapshot (frame parity)

	// SDRAM read (pass 2): hold sr_req until sr_ack; sr_q valid with sr_ack
	output reg        sr_req = 0,
	output reg [24:0] sr_addr = 0,
	input             sr_ack,
	input       [7:0] sr_q,

	// values for the condition evaluators, two read ports on v_clk (the HD engine clock; it
	// is the 4x multiple of clk from the same PLL), one clock latency each: port 1 for
	// hd3_resolve, port 2 for hd3_render. Each port reads a copy of the values, so the two
	// never wait for each other; the bank a reader names is never the one being written
	// (the snapshot of frame N goes to bank N[0] while the readers use frame N-1's).
	input             v_clk,
	input             v_bank,
	input       [7:0] v_idx,
	output reg  [7:0] v_q,
	input             v2_bank,
	input       [7:0] v2_idx,
	output reg  [7:0] v2_q,

	output reg        done = 0,       // pulse: bank `bank` holds a complete snapshot
	output            overflow        // sticky: a write was lost or a snapshot was cut short
);

// one sticky flag per always block (a reg may only have one driver), OR-ed for the port
reg ov_fifo = 0, ov_nt = 0, ov_snap = 0;
assign overflow = ov_fifo | ov_nt | ov_snap;

localparam [24:0] CPURAM_LIN = 25'h0380000;   // nes cart.sv: $0000-$1FFF -> 0x380000 + a[10:0]
localparam [11:0] CIRAM_PG0  = 12'hE80;       // 0x3A0000 >> 10 (cart.sv vram_ce mapping)

// ------------------------------------------------------------------------------------------
// list and values
// ------------------------------------------------------------------------------------------
reg [31:0] list [256];
reg  [7:0] vals [512] /*verilator public_flat_rd*/;   // fullsim --watch reads the snapshot
reg  [7:0] l_ra;
reg [31:0] l_q;

always @(posedge clk) begin
	if (lw_we) list[lw_idx] <= lw_addr;
	l_q  <= list[l_ra];
end

// vals: one write port (vw_*, chosen by the snapshot state machine below) and a registered
// read on v_clk - the dual-clock shape Quartus maps to block RAM. vals2 is the same array for
// the second reader (one 512 x 8 block each).
reg  [7:0] vals2 [512];
reg        vw_en;
reg  [8:0] vw_a;
reg  [7:0] vw_d;
always @(posedge clk) if (vw_en) vals[vw_a] <= vw_d;
always @(posedge v_clk) v_q <= vals[{v_bank, v_idx}];
always @(posedge clk) if (vw_en) vals2[vw_a] <= vw_d;
always @(posedge v_clk) v2_q <= vals2[{v2_bank, v2_idx}];

// ------------------------------------------------------------------------------------------
// address bookkeeping (live)
// ------------------------------------------------------------------------------------------
reg [24:11] prg_base [16];      // $8000-$FFFF per 2 KB
reg  [15:0] prg_bv = 0;
reg [21:10] chr_base [8];       // $0000-$1FFF per 1 KB
reg   [7:0] chr_bv = 0;
reg [21:10] qmap [4];           // nametable quadrant -> SDRAM page
reg   [3:0] qmap_v = 0;
reg [24:13] ram_base /*verilator public_flat_rd*/;   // SDRAM page the CPU last used for $6000-$7FFF
reg         ram_bv = 0;

reg old_cr = 0, old_cw = 0, old_pr = 0, old_pw = 0, old_ss = 0;
wire c_acc = (cpu_rd & ~old_cr) | (cpu_wr & ~old_cw);
wire c_w   = cpu_wr & ~old_cw;
wire p_acc = (ppu_rd & ~old_pr) | (ppu_wr & ~old_pw);
wire p_w   = ppu_wr & ~old_pw;
wire ss_go = ss_wr & ~old_ss;

wire p_nt  = ppu_a[13] && (ppu_a[12:8] != 5'h1F);   // $2000-$3EFF

always @(posedge clk) begin
	old_cr <= cpu_rd; old_cw <= cpu_wr; old_pr <= ppu_rd; old_pw <= ppu_wr; old_ss <= ss_wr;
	if (c_acc && cpu_a[15]) begin
		prg_base[cpu_a[14:11]] <= cpu_lin[24:11];
		prg_bv[cpu_a[14:11]]   <= 1;
	end
	if (c_acc && cpu_a[15:13] == 3'b011) begin
		ram_base <= cpu_lin[24:13];
		ram_bv   <= 1;
	end
	if (p_acc && !ppu_a[13]) begin
		chr_base[ppu_a[12:10]] <= ppu_lin[21:10];
		chr_bv[ppu_a[12:10]]   <= 1;
	end
	if (p_acc && p_nt) begin
		qmap[ppu_a[11:10]]   <= ppu_lin[21:10];
		qmap_v[ppu_a[11:10]] <= 1;
	end
end

// ------------------------------------------------------------------------------------------
// write sources -> shadows (direct, or through the FIFOs while frozen / draining)
// kind: 0 CPU RAM, 1 PRG-RAM, 2 nametable page (by SDRAM page)
// ------------------------------------------------------------------------------------------
wire        c_ram  = c_w && cpu_a[15:13] == 3'b000;
wire        c_prg  = c_w && cpu_a[15:13] == 3'b011;
wire        ss_ram = ss_go && ss_addr[24:11] == CPURAM_LIN[24:11];
wire        ss_prg = ss_go && ram_bv && ss_addr[24:13] == ram_base;
wire        ss_nt  = ss_go && ss_addr[24:22] == 3'd0;           // page match decided at apply
wire        p_ntw  = p_w && p_nt;

// FIFO entry, 34 bits:
//   CPU RAM / PRG-RAM  {kind 0|1, 1'b0, offset[12:0] at [30:18], 10'd0, data}
//   nametable          {kind 2, keep_only, 1'b0, SDRAM page[21:10] at [29:18], offset[9:0] at
//                       [17:8], data}; keep_only = a savestate write, applied only to a page
//                       already known (a restore writes ALL of SDRAM, which must not claim slots)
// One source per FIFO per clock: the CPU and savestates never write at the same time (the
// core is paused during a restore), and the CPU wins if they ever did.
wire        cq_in  = c_ram | c_prg | ss_ram | ss_prg;
wire [33:0] cq_d   = (c_ram | c_prg) ? {c_prg ? 2'd1 : 2'd0, 1'b0, cpu_a[12:0], 10'd0, cpu_d}
                                     : {ss_prg ? 2'd1 : 2'd0, 1'b0, ss_addr[12:0], 10'd0, ss_d};
wire        pq_in  = p_ntw | (ss_nt & ~p_ntw);
wire [33:0] pq_d   = p_ntw ? {2'd2, 1'b0, 1'b0, ppu_lin[21:10], ppu_lin[9:0], ppu_d}
                           : {2'd2, 1'b1, 1'b0, ss_addr[21:10], ss_addr[9:0], ss_d};

reg  [33:0] cfifo [32];
reg  [33:0] pfifo [32];
reg   [5:0] cw_p = 0, cr_p = 0, pw_p = 0, pr_p = 0;
wire  [5:0] c_cnt = cw_p - cr_p;
wire  [5:0] p_cnt = pw_p - pr_p;
reg         frozen = 0;

wire        c_buf  = frozen || c_cnt != 0;
wire        p_buf  = frozen || p_cnt != 0;

// the shadow write chosen this clock
reg         cw_go;  reg [33:0] cw_e;
reg         pw_go;  reg [33:0] pw_e;
always @* begin
	cw_go = 0; cw_e = cq_d;
	if (!c_buf && cq_in)            begin cw_go = 1; cw_e = cq_d; end
	else if (!frozen && c_cnt != 0) begin cw_go = 1; cw_e = cfifo[cr_p[4:0]]; end
	pw_go = 0; pw_e = pq_d;
	if (!p_buf && pq_in)            begin pw_go = 1; pw_e = pq_d; end
	else if (!frozen && p_cnt != 0) begin pw_go = 1; pw_e = pfifo[pr_p[4:0]]; end
end

always @(posedge clk) begin
	if (c_buf && cq_in) begin
		if (c_cnt[5]) ov_fifo <= 1;
		else begin cfifo[cw_p[4:0]] <= cq_d; cw_p <= cw_p + 1'd1; end
	end
	if (!frozen && c_cnt != 0) cr_p <= cr_p + 1'd1;
	if (p_buf && pq_in) begin
		if (p_cnt[5]) ov_fifo <= 1;
		else begin pfifo[pw_p[4:0]] <= pq_d; pw_p <= pw_p + 1'd1; end
	end
	if (!frozen && p_cnt != 0) pr_p <= pr_p + 1'd1;
end

// --- CPU RAM / PRG-RAM ---
reg  [7:0] cpuram [2048];
reg  [7:0] prgram [8192];
wire [12:0] sh_ra;          // pass-1 stage A address (combinational from the list entry)
reg  [7:0] cpuram_q, prgram_q;
always @(posedge clk) begin
	if (cw_go && cw_e[33:32] == 2'd0) cpuram[cw_e[28:18]] <= cw_e[7:0];
	if (cw_go && cw_e[33:32] == 2'd1) prgram[cw_e[30:18]] <= cw_e[7:0];
	cpuram_q <= cpuram[sh_ra[10:0]];
	prgram_q <= prgram[sh_ra];
end

// --- nametable pages: 4 slots tagged with their SDRAM page ---
reg  [7:0] ntram [4096];
reg [21:10] ntag [4];
reg   [3:0] ntag_v = 4'b0011;
reg   [1:0] nt_alloc = 2'd2;
initial begin ntag[0] = CIRAM_PG0; ntag[1] = CIRAM_PG0 + 12'd1; ntag[2] = 0; ntag[3] = 0; end

function [2:0] nt_find(input [21:10] pg);   // {hit, slot}
	integer j;
	begin
		nt_find = 3'b000;
		for (j = 3; j >= 0; j = j - 1)
			if (ntag_v[j] && ntag[j] == pg) nt_find = {1'b1, j[1:0]};
	end
endfunction

wire [2:0] w_find = nt_find(pw_e[29:18]);
wire [11:0] nt_ra;          // pass-1 stage A address
reg   [7:0] nt_q;
always @(posedge clk) begin
	if (pw_go) begin
		if (w_find[2]) ;
		else if (!pw_e[31]) begin
			// a page not seen yet (four-screen RAM): take the next slot after the CIRAM pair;
			// a third extra page would evict one, which no mapper here does
			if (ntag_v[nt_alloc]) ov_nt <= 1;
			ntag[nt_alloc]   <= pw_e[29:18];
			ntag_v[nt_alloc] <= 1;
			nt_alloc <= (nt_alloc == 2'd3) ? 2'd2 : nt_alloc + 1'd1;
		end
	end
end
// ntram: ONE write statement and a registered read, the shape Quartus maps to block RAM (two
// write statements in branches can leave it in registers: 32 kbit of them). The write goes to
// the hit slot, or to the slot being claimed above (a live PPU write, pw_e[31] clear; a
// savestate write only updates a page already held).
wire        nt_we = pw_go && (w_find[2] || pw_e[31] == 1'b0);
wire [11:0] nt_wa = w_find[2] ? {w_find[1:0], pw_e[17:8]} : {nt_alloc, pw_e[17:8]};
always @(posedge clk) begin
	if (nt_we) ntram[nt_wa] <= pw_e[7:0];
	nt_q <= ntram[nt_ra];
end

// ------------------------------------------------------------------------------------------
// snapshot
// ------------------------------------------------------------------------------------------
reg [191:0] s_pal;
reg [24:11] s_prg  [16];
reg  [15:0] s_prgv;
reg [21:10] s_chr  [8];
reg   [7:0] s_chrv;
reg [21:10] s_qmap [4];
reg   [3:0] s_qmapv;
reg   [8:0] s_n;
reg         s_bank;

localparam ST_IDLE = 3'd0, ST_P1 = 3'd1, ST_P2 = 3'd2, ST_P2R = 3'd3, ST_P2W = 3'd4;
reg  [2:0] st = ST_IDLE;
reg  [8:0] i;              // pass-1 issue index
reg  [8:0] p2;             // pass-2 index

// pass-1 pipeline: issue list[i] -> (A) l_q, choose shadow -> (B) shadow data, write value
reg        a_v;  reg [7:0] a_idx;
reg        b_v;  reg [7:0] b_idx;  reg [2:0] b_src;  reg [7:0] b_imm;

// classify an entry: 0 CPU RAM, 1 PRG-RAM, 2 nametable, 3 immediate (b_imm), 4 SDRAM later
function [2:0] kind(input [31:0] e);
	begin
		if (e[31]) begin
			if (e[13:8] == 6'h3F)  kind = 3'd3;          // palette
			else if (!e[13])       kind = 3'd4;          // pattern table
			else                   kind = 3'd2;          // nametable
		end else begin
			case (e[15:13])
				3'b000:  kind = 3'd0;
				3'b011:  kind = 3'd1;
				3'b001, 3'b010: kind = 3'd3;             // registers / expansion: 0
				default: kind = 3'd4;                    // PRG-ROM
			endcase
		end
	end
endfunction

wire [31:0] e = l_q;
wire  [4:0] pidx = ((e[4:0] & 5'h13) == 5'h10) ? {1'b0, e[3:0]} : e[4:0];
reg   [2:0] a_find;
always @* begin
	a_find = 3'b000;
	if (s_qmapv[e[11:10]]) a_find = nt_find(s_qmap[e[11:10]]);
end
assign sh_ra = e[12:0];
assign nt_ra = {a_find[1:0], e[9:0]};

// pass 2: SDRAM address of an entry
wire [31:0] e2 = l_q;
wire        e2_ppu = e2[31];
wire        e2_ok  = e2_ppu ? s_chrv[e2[12:10]] : s_prgv[e2[14:11]];
wire [24:0] e2_lin = e2_ppu ? {3'b000, s_chr[e2[12:10]], e2[9:0]}
                            : {s_prg[e2[14:11]], e2[10:0]};

// the snapshot value written this clock: the state machine below runs its case only when it
// is not resetting and not starting a snapshot, and so does this
always @* begin
	vw_en = 0;
	vw_a  = {s_bank, b_idx};
	vw_d  = b_imm;
	if (!reset && !(snap && enable && list_n != 0)) case (st)
		ST_P1: if (b_v && b_src <= 3'd3) begin
			vw_en = 1;
			vw_d  = (b_src == 3'd0) ? cpuram_q : (b_src == 3'd1) ? prgram_q :
			        (b_src == 3'd2) ? nt_q : b_imm;
		end
		ST_P2R: if (p2 < s_n && kind(e2) == 3'd4 && !e2_ok) begin
			vw_en = 1;
			vw_a  = {s_bank, p2[7:0]};
			vw_d  = 8'd0;
		end
		ST_P2W: if (sr_ack) begin
			vw_en = 1;
			vw_a  = {s_bank, p2[7:0]};
			vw_d  = sr_q;
		end
		default: ;
	endcase
end

integer q;
always @(posedge clk) begin
	done <= 0;

	if (reset) begin
		st     <= ST_IDLE;
		frozen <= 0;
		sr_req <= 0;
		a_v    <= 0;
		b_v    <= 0;
	end else begin
		if (snap && enable && list_n != 0) begin
			if (st != ST_IDLE) ov_snap <= 1;           // previous one cut short
			st      <= ST_P1;
			frozen  <= 1;
			sr_req  <= 0;
			s_pal   <= pal_all;
			for (q = 0; q < 16; q = q + 1) s_prg[q]  <= prg_base[q];
			for (q = 0; q < 8;  q = q + 1) s_chr[q]  <= chr_base[q];
			for (q = 0; q < 4;  q = q + 1) s_qmap[q] <= qmap[q];
			s_prgv  <= prg_bv;
			s_chrv  <= chr_bv;
			s_qmapv <= qmap_v;
			s_n     <= (list_n > 9'd256) ? 9'd256 : list_n;
			s_bank  <= bank;
			i       <= 0;
			l_ra    <= 0;
			a_v     <= 0;
			b_v     <= 0;
		end else case (st)
			ST_P1: begin
				// issue
				a_v   <= (i < s_n);
				a_idx <= i[7:0];
				if (i < s_n) begin
					i    <= i + 1'd1;
					l_ra <= i[7:0] + 1'd1;
				end
				// stage A: l_q holds list[a_idx]
				b_v   <= a_v;
				b_idx <= a_idx;
				b_src <= kind(e);
				b_imm <= (kind(e) == 3'd3 && e[31]) ? {2'b00, s_pal[pidx * 6 +: 6]} : 8'd0;
				if (a_v && kind(e) == 3'd2 && !a_find[2]) b_src <= 3'd3;   // page never written: 0
				// stage B: shadow data valid - the value is written by vw_* (above)
				if (!a_v && !b_v && i >= s_n) begin
					frozen <= 0;                             // FIFOs drain from here
					st     <= ST_P2;
					p2     <= 0;
					l_ra   <= 0;
				end
			end
			ST_P2: begin
				// l_ra = p2 was set last clock; one clock for l_q
				st <= ST_P2R;
			end
			ST_P2R: begin
				if (p2 >= s_n) begin
					st   <= ST_IDLE;
					done <= 1;
				end else if (kind(e2) == 3'd4) begin
					if (e2_ok) begin
						sr_addr <= e2_lin;
						sr_req  <= 1;
						st      <= ST_P2W;
					end else begin
						// page never accessed: unknown, 0 (written by vw_*)
						p2   <= p2 + 1'd1;
						l_ra <= p2[7:0] + 1'd1;
						st   <= ST_P2;
					end
				end else begin
					p2   <= p2 + 1'd1;
					l_ra <= p2[7:0] + 1'd1;
					st   <= ST_P2;
				end
			end
			ST_P2W: if (sr_ack) begin
				sr_req <= 0;                                 // value written by vw_*
				p2   <= p2 + 1'd1;
				l_ra <= p2[7:0] + 1'd1;
				st   <= ST_P2;
			end
			default: ;
		endcase
	end
end

endmodule
