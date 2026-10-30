// hd3_dict.sv - stage 3 record dictionary, CAPTURE side (HDPACK_DESIGN.md "hd3_dict").
//
// Listens to hd_capture's write stream (every qword hd_ddr accepts) and, for every BG span and
// sprite fetch of every line, forms the record key Mesen looks up - {palette, body}, body = the
// tile fingerprint in a CHR-RAM game, else the tile index - gives each distinct key of the
// frame an id, and stores the id per line and slot. A background resolver then resolves each
// new key once, in allocation order: exact key, then (CHR-ROM only) the fallback tile under the
// same palette and under the default palette, then the default palette. RENDER reads keys,
// resolutions and ids through the r_* ports, from the bank of the frame it draws.
//
// Banks: frame N's records live in bank (frame-record buffer of N) = frame number & 1, so
// RENDER reads frame N while frame N+1 is captured into the other bank.
//
// Every memory has one write port and registered reads (the shape Quartus maps to block RAM);
// the key store has two read ports (dictionary/resolver, and RENDER) and therefore uses its
// read address for its write too.

module hd3_dict #(
	parameter [27:0] REC_BASE = 28'hC000000,   // frame-record buffer 0 (byte offset in the window)
	parameter        RAW      = 11              // record id width: 2^RAW records per frame
)
(
	input             clk,
	input             reset,

	// pack and game configuration (stable while a pack is loaded)
	input             game_ram,         // CHR-RAM game: keys are fingerprints, no fallbacks
	input             legacy,           // pack version < 100: no backdrop/0xFF palette byte
	input      [27:3] keys_base,
	input      [27:3] rules_base,
	input      [31:0] bucket_mask,
	input      [15:0] max_probe,
	input      [27:3] fb_base,          // fallback map (section 10), i32 per CHR tile
	input      [31:0] fb_len,           // its entry count (0 = no fallbacks)
	// additions (8a): section 16 = addition-key table (the key table's format), section 17 =
	// the lists it points into. add_en = the pack has the table.
	input             add_en,
	input      [27:3] akeys_base,
	input      [31:0] abucket_mask,
	input      [15:0] amax_probe,
	input      [27:3] alists_base,

	// tap of hd_capture's write stream: one qword per clock when tap_v
	input             tap_v,
	input      [27:3] tap_a,
	input      [63:0] tap_d,

	// DDR3 read port (8-qword bursts, data in order)
	output            rd_req,
	output     [27:3] rd_addr,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid,

	// read ports (RENDER, and the testbench), one clock latency
	input             r_bank,
	input   [RAW-1:0] r_rid,
	output reg [95:0] r_key,            // {palette, body}
	output reg [63:0] r_res,            // see "result" below
	input       [7:0] r_y,
	input       [5:0] r_slot,           // BG span slot 0..33
	output reg [RAW:0] r_span,          // {valid, id}
	input       [3:0] r_fetch,          // sprite fetch 0..15
	output reg [RAW:0] r_spr,           // {valid, id}
	output    [RAW:0] r_nrec,           // records of bank r_bank
	output     [31:0] r_frame,          // frame number of bank r_bank (its trailer, spec 2.3)

	// addition read ports (hd3_add), one clock latency: a record's addition list (merged in
	// file order) and the pool of list entries
	input             a_bank,
	input   [RAW-1:0] a_rid,
	output reg [16:0] a_m,              // {count 7, start 10}; count 0 = no additions
	input       [9:0] a_pi,
	output reg [47+RAW:0] a_pe,         // {file index 16, oy 16, ox 16, added id}

	output reg        frame_done = 0,   // pulse: every record of frame done_bank is resolved
	output reg        done_bank = 0,
	output            overflow,         // sticky: see the causes where ovf_d / ovf_a are set
	// per bank: the frame in it hit one of those causes, so its records are incomplete. RENDER
	// draws such a frame with no HD lookups at all (HDPACK_DESIGN.md, "8o"); cleared when the
	// bank starts its next frame
	output      [1:0] frame_ovf
);

// overflow, from the dictionary (_d) and from the resolver's addition pass (_a)
reg         ovf_d = 0, ovf_a = 0;
reg   [1:0] fovf_d = 0, fovf_a = 0;
assign overflow  = ovf_d | ovf_a;
assign frame_ovf = fovf_d | fovf_a;

localparam NREC = 1 << RAW;

// result word: {done, via[1:0], list offset / 8 [24:0], 2'b00, fb2 {valid, idx[15:0]},
//               fb {valid, idx[15:0]}}; via 0 none, 1 exact, 2 fallback, 3 default palette
function [63:0] res_word(input [1:0] via, input [24:0] offq, input [16:0] fb, input [16:0] fb2);
	res_word = {1'b1, via, offq, 2'b00, fb2, fb};
endfunction

// ------------------------------------------------------------------------------------------
// tap decode
// ------------------------------------------------------------------------------------------
wire [27:3] rel     = tap_a - REC_BASE[27:3];
wire        in_rec  = rel[27:21] == 7'd0;              // the two 1 MB buffers
wire        t_bank  = rel[20];
wire [16:0] t_q     = rel[19:3];                       // qword inside the buffer
wire [16:0] t_l     = t_q - 17'h08000;                 // 0x40000 / 8: line blocks
wire        t_line  = in_rec && t_q >= 17'h08000 && t_l[15:8] < 8'd240 && !t_l[16];
wire  [7:0] t_y     = t_l[15:8];
wire  [7:0] t_o     = t_l[7:0];                        // qword inside the 2 KB line block
wire        t_trl   = in_rec && t_q == 17'h1E000;      // 0xF0000: frame trailer
// kept in the line store: header q0-q3, spans (8..75), sprite fetches 0..15 (80..111)
wire        t_keep  = t_line && (t_o < 8'd4 || (t_o >= 8'd8 && t_o < 8'd76) ||
                                 (t_o >= 8'd80 && t_o < 8'd112));

// ------------------------------------------------------------------------------------------
// line store: 4 line slots x 128 qwords
// ------------------------------------------------------------------------------------------
reg  [63:0] lmem [512];
reg   [8:0] ls_a;                   // read address (combinational, below)
reg  [63:0] ls_q;
always @(posedge clk) begin
	if (tap_v && t_keep) lmem[{t_y[1:0], t_o[6:0]}] <= tap_d;
	ls_q <= lmem[ls_a];
end

// ------------------------------------------------------------------------------------------
// hash index: 4096 x {valid, id}
// ------------------------------------------------------------------------------------------
reg  [RAW:0] imem [4096];
reg   [11:0] ix_ra, ix_wa;
reg  [RAW:0] ix_wd;
reg          ix_we;
reg  [RAW:0] ix_q;
always @(posedge clk) begin
	if (ix_we) imem[ix_wa] <= ix_wd;
	ix_q <= imem[ix_ra];
end

// ------------------------------------------------------------------------------------------
// key store: {bank, id} -> {palette, body}; port A = dictionary (read, write) and resolver
// (read), port B = RENDER
// ------------------------------------------------------------------------------------------
reg  [95:0] kmem [2 * NREC];
reg  [RAW:0] ka_a;
reg  [95:0] ka_d;
reg         ka_we;
reg  [95:0] ka_q;
always @(posedge clk) begin
	if (ka_we) kmem[ka_a] <= ka_d;
	ka_q <= kmem[ka_a];
end
always @(posedge clk) r_key <= kmem[{r_bank, r_rid}];

// result store: written by the resolver, read by RENDER
reg  [63:0] rmem [2 * NREC];
reg  [RAW:0] rs_wa;
reg  [63:0] rs_wd;
reg         rs_we;
always @(posedge clk) begin
	if (rs_we) rmem[rs_wa] <= rs_wd;
	r_res <= rmem[{r_bank, r_rid}];
end

// span id store: {bank, y * 34 + slot}; sprite id store: {bank, y, fetch}
reg [RAW:0] smem [16384];
reg [RAW:0] pmem [8192];
reg  [13:0] sp_wa;
reg  [12:0] pp_wa;
reg [RAW:0] id_wd;
reg         sp_we, pp_we;
wire [12:0] r_sa = {r_y, 5'd0} + {4'd0, r_y, 1'b0} + {7'd0, r_slot};
always @(posedge clk) begin
	if (sp_we) smem[sp_wa] <= id_wd;
	r_span <= smem[{r_bank, r_sa}];
end
always @(posedge clk) begin
	if (pp_we) pmem[pp_wa] <= id_wd;
	r_spr <= pmem[{r_bank, r_y, r_fetch}];
end

// ------------------------------------------------------------------------------------------
// additions (8a), per bank: each record's merged addition list {count, start} and the pool
// of list entries, both written by the resolver; and a bit per record "allocated for an added
// key" (written by the dictionary when it allocates), which the resolver reads so it does not
// look up additions for added keys (the converter refuses a pack where an added key would
// itself match an addition)
// ------------------------------------------------------------------------------------------
reg  [16:0] amem [2 * NREC];
reg  [RAW:0] am_wa;
reg  [16:0] am_wd;
reg         am_we;
always @(posedge clk) begin
	if (am_we) amem[am_wa] <= am_wd;
	a_m <= amem[{a_bank, a_rid}];
end
reg [47+RAW:0] apool [2048];
reg  [10:0] ap_wa;
reg [47+RAW:0] ap_wd;
reg         ap_we;
always @(posedge clk) begin
	if (ap_we) apool[ap_wa] <= ap_wd;
	a_pe <= apool[{a_bank, a_pi}];
end
reg         injm [2 * NREC];
reg  [RAW:0] ij_wa;
reg         ij_wd, ij_we;
reg  [RAW:0] ij_ra;
reg         injm_q;
always @(posedge clk) begin
	if (ij_we) injm[ij_wa] <= ij_wd;
	injm_q <= injm[ij_ra];
end

// injection: the resolver asks the dictionary for the id of an added key (deduplicated
// against the frame's keys through the same hash index), between line items
reg         inj_req = 0;            // resolver: a key is waiting (held until inj_done)
reg  [95:0] inj_key;
reg         inj_bank;
reg         inj_done = 0;           // dictionary: one-clock answer
reg [RAW:0] inj_id;                 // {valid, id}; invalid = no id (see inj_done's causes)

// ------------------------------------------------------------------------------------------
// frame / line bookkeeping
// ------------------------------------------------------------------------------------------
reg   [8:0] ready_n = 0;            // lines 0..ready_n-1 are complete in the line store
reg   [8:0] proc_y  = 9'd240;       // next line to process (240 = none)
reg   [3:0] slot_ok = 0;            // header of the line in this slot arrived, not processed
reg         nf_pend = 0;            // a new frame started (header of line 0)
reg         nf_bank = 0;
reg         fr_bank = 0;            // bank of the frame being deduplicated
// per bank: a frame has begun and is not yet fully resolved. Two toggles, one per always
// block (the dictionary flips tog_d when a frame starts, the resolver flips tog_r when it
// finishes one), so neither block writes the other's register.
reg   [1:0] tog_d = 0, tog_r = 0;
wire  [1:0] started = tog_d ^ tog_r;
// per bank: flips at every new frame, like tog_d, but never touched by the resolver - its
// "this bank began a new frame". A frame that restarts a bank whose previous frame was never
// resolved flips tog_d back to tog_r (started cancels for a clock); the resolver sees the new
// epoch with the old frame unfinished and retires that frame (flips tog_r: abandoned, no
// frame_done), so the new one - flagged overflowed - is resolved from its first record and
// reported done like any other. Without the retirement it would never be resolved at all.
reg   [1:0] ep_d = 0;
reg   [1:0] dd_done = 0;            // per bank: every line deduplicated
reg [RAW:0] nrec [2];               // per bank: ids allocated
initial begin nrec[0] = 0; nrec[1] = 0; end
assign r_nrec = nrec[r_bank];

// frame number per bank, from the trailer write (RESOLVE's frameRange conditions read it)
reg  [31:0] frame_no [2];
initial begin frame_no[0] = 0; frame_no[1] = 0; end
always @(posedge clk) if (tap_v && t_trl) frame_no[t_bank] <= tap_d[31:0];
assign r_frame = frame_no[r_bank];

// ------------------------------------------------------------------------------------------
// line processor / deduplicator
// ------------------------------------------------------------------------------------------
typedef enum bit [3:0] {
	L_IDLE, L_Q0, L_Q1, L_Q2, L_Q3, I_A, I_B, I_C, K_HASH, K_I0, K_I1, K_K, K_ALLOC, K_WR, L_CLR
} lst_t;
lst_t lst = L_IDLE;

reg   [7:0] y;                      // line being processed
reg   [1:0] slot;
reg  [12:0] y34;                    // y * 34
reg   [5:0] nspan;
reg   [4:0] nfet;
reg [191:0] pal;
reg   [5:0] item;                   // 0..nspan-1 spans, then nspan.. fetches
reg         is_spr;
reg   [5:0] sidx;                   // span slot or fetch index
reg  [63:0] qa;
reg  [95:0] key;
reg  [11:0] h;
reg [RAW-1:0] rid_c;
reg  [11:0] clr;
reg [RAW:0] id_r;
reg         inj = 0;                // the key being looked up is an injection, not a line item
reg         clr_pend = 0;           // the index must be cleared before the next frame

function [5:0] c6(input [191:0] p, input [4:0] n);
	c6 = p[6 * n +: 6];
endfunction

// key of the item, from qa (latched), qB (ls_q in I_C), the palette and the config
wire  [2:0] grp     = is_spr ? {1'b1, qa[47:46]} : {1'b0, qa[33:32]};
wire [21:0] lin     = is_spr ? qa[37:16] : qa[21:0];
wire  [7:0] top     = legacy ? 8'h00 : (is_spr ? 8'hFF : {2'b00, c6(pal, 5'd0)});
wire [31:0] k_pal   = {top, 2'b00, c6(pal, {grp, 2'd1}), 2'b00, c6(pal, {grp, 2'd2}),
                       2'b00, c6(pal, {grp, 2'd3})};
wire [63:0] k_body  = game_ram ? ls_q : {48'd0, lin[19:4]};
wire [11:0] k_hash  = key[11:0] ^ key[23:12] ^ key[35:24] ^ key[47:36] ^
                      key[59:48] ^ key[71:60] ^ key[83:72] ^ key[95:84];

wire        last_item = (item + 6'd1) == ({1'b0, nspan} + {1'b0, nfet});
wire        cap_full  = nrec[fr_bank][RAW];

// port A of the key store is the dictionary's in K_I1 (reading the probed record) and K_ALLOC
wire        dd_ka     = (lst == K_I1 && ix_q[RAW]) || (lst == K_ALLOC && !cap_full);

// resolver request on port A (granted when the dictionary does not use it)
reg         rs_kreq;
reg [RAW:0] rs_ka;

always @* begin
	// line store read address: the data shows up in the NEXT state
	case (lst)
		L_IDLE:  ls_a = {proc_y[1:0], 7'd0};
		L_Q0:    ls_a = {slot, 7'd1};
		L_Q1:    ls_a = {slot, 7'd2};
		L_Q2:    ls_a = {slot, 7'd3};
		I_A:     ls_a = {slot, is_spr ? 7'd80 + {sidx, 1'b0} : 7'd8 + {sidx, 1'b0}};
		I_B:     ls_a = {slot, is_spr ? 7'd81 + {sidx, 1'b0} : 7'd9 + {sidx, 1'b0}};
		default: ls_a = {slot, 7'd0};
	endcase

	ix_ra = h;
	ix_we = 0; ix_wa = h; ix_wd = {1'b1, nrec[fr_bank][RAW-1:0]};
	if (lst == K_ALLOC && !cap_full) ix_we = 1;
	if (lst == L_CLR) begin ix_we = 1; ix_wa = clr; ix_wd = 0; end

	ka_we = 0; ka_d = key;
	if (lst == K_I1 && ix_q[RAW])        ka_a = {fr_bank, ix_q[RAW-1:0]};
	else if (lst == K_ALLOC && !cap_full) begin ka_a = {fr_bank, nrec[fr_bank][RAW-1:0]}; ka_we = 1; end
	else                                  ka_a = rs_ka;
end

integer b;
always @(posedge clk) begin
	sp_we <= 0;
	pp_we <= 0;

	// ---- tap: line readiness, frame boundaries ----
	if (tap_v && t_line && t_o == 8'd3) begin
		if (slot_ok[t_y[1:0]]) begin                     // the line 4 back was never processed
			ovf_d <= 1;
			fovf_d[fr_bank] <= 1;
		end
		slot_ok[t_y[1:0]] <= 1;
		if (t_y == 8'd0) begin
			nf_pend <= 1;
			nf_bank <= t_bank;
			ready_n <= 0;
		end else
			ready_n <= {1'b0, t_y};
	end
	if (tap_v && t_trl) ready_n <= 9'd240;

	inj_done <= 0;
	ij_we    <= 0;
	case (lst)
		L_IDLE: begin
			// the index is cleared once the resolver has finished the frame (it may still
			// inject added keys until then) - or when the next frame starts, which closes the
			// old frame to injections (the resolver then marks it overflowed)
			if (clr_pend && (nf_pend || !started[fr_bank])) begin
				clr_pend <= 0;
				clr      <= 0;
				lst      <= L_CLR;
			end else if (nf_pend && proc_y < 9'd240) begin
				// a frame abandoned before all its lines were processed is incomplete; its keys
				// are in the index, which is cleared (next clock) before the new frame starts
				ovf_d <= 1;
				fovf_d[fr_bank] <= 1;
				proc_y   <= 9'd240;
				clr_pend <= 1;
			end else if (nf_pend) begin
				nf_pend <= 0;
				if (started[nf_bank]) ovf_d <= 1;      // that bank was still being resolved
				// the new frame starts clean unless it restarted a bank still being resolved
				fovf_d[nf_bank] <= started[nf_bank];
				fr_bank          <= nf_bank;
				tog_d[nf_bank]   <= ~tog_d[nf_bank];
				ep_d[nf_bank]    <= ~ep_d[nf_bank];
				dd_done[nf_bank] <= 0;
				nrec[nf_bank]    <= 0;
				proc_y           <= 0;
			end else if (proc_y < 9'd240 && proc_y < ready_n) begin
				y    <= proc_y[7:0];
				slot <= proc_y[1:0];
				y34  <= {proc_y[7:0], 5'd0} + {4'd0, proc_y[7:0], 1'b0};
				lst  <= L_Q0;
			end else if (proc_y == 9'd240 && started[fr_bank] && !dd_done[fr_bank] &&
			             ready_n == 9'd240) begin
				dd_done[fr_bank] <= 1;
				if (add_en) clr_pend <= 1;              // the resolver may still inject keys
				else begin
					clr <= 0;
					lst <= L_CLR;
				end
			end else if (inj_req && !inj_done) begin
				// an injection is for the frame being captured only: a key for an older frame
				// (its bank already handed to the next frame) gets no id. The index never holds
				// a cleared frame's keys while this bank test passes - it is cleared only once
				// the resolver has finished the frame, or as the next frame starts, which changes
				// fr_bank (or, restarting the same bank, overflows that frame anyway)
				if (inj_bank == fr_bank) begin
					inj <= 1;
					key <= inj_key;
					lst <= K_HASH;
				end else begin
					inj_done <= 1;                          // the frame is closed: no id
					inj_id   <= 0;
				end
			end
		end

		L_Q0: begin                                     // ls_q = q0
			if (slot_ok[slot]) begin
				nspan <= (ls_q[39:32] > 8'd34) ? 6'd34 : ls_q[37:32];
				nfet  <= (ls_q[55:48] > 8'd16) ? 5'd16 : ls_q[52:48];
				if (ls_q[55:48] > 8'd16) begin            // this PPU fetches at most 16
					ovf_d <= 1;
					fovf_d[fr_bank] <= 1;
				end
			end else begin
				nspan <= 0;                              // header never written: no records
				nfet  <= 0;
			end
			lst <= L_Q1;
		end
		L_Q1: begin pal[63:0]    <= ls_q; lst <= L_Q2; end
		L_Q2: begin pal[127:64]  <= ls_q; lst <= L_Q3; end
		L_Q3: begin
			pal[191:128] <= ls_q;
			item   <= 0;
			is_spr <= (nspan == 0);
			sidx   <= 0;
			if (nspan == 0 && nfet == 0) begin
				slot_ok[slot] <= 0;
				proc_y <= proc_y + 9'd1;
				lst    <= L_IDLE;
			end else
				lst <= I_A;
		end

		I_A: lst <= I_B;
		I_B: begin qa <= ls_q; lst <= I_C; end           // ls_q = qA
		I_C: begin                                       // ls_q = qB
			key <= {k_pal, k_body};
			if (is_spr && !qa[54]) begin
				id_r <= 0;                                // not a valid sprite: no record
				lst  <= K_WR;
			end else
				lst <= K_HASH;
		end

		K_HASH: begin h <= k_hash; lst <= K_I0; end
		K_I0:   lst <= K_I1;                             // index read of h
		K_I1: begin                                      // ix_q = index[h]
			rid_c <= ix_q[RAW-1:0];
			lst   <= ix_q[RAW] ? K_K : K_ALLOC;
		end
		K_K: begin                                       // ka_q = key of the probed record
			if (ka_q == key) begin
				id_r <= {1'b1, rid_c};
				lst  <= K_WR;
			end else begin
				h   <= h + 12'd1;
				lst <= K_I0;
			end
		end
		K_ALLOC: begin
			if (cap_full) begin
				ovf_d <= 1;                            // more distinct records than ids
				fovf_d[fr_bank] <= 1;
				id_r <= 0;
			end else begin
				id_r <= {1'b1, nrec[fr_bank][RAW-1:0]};
				nrec[fr_bank] <= nrec[fr_bank] + 1'd1;
				ij_we <= 1; ij_wa <= {fr_bank, nrec[fr_bank][RAW-1:0]}; ij_wd <= inj;
			end
			lst <= K_WR;
		end

		K_WR: if (inj) begin
			inj      <= 0;
			inj_done <= 1;
			inj_id   <= id_r;
			lst      <= L_IDLE;
		end else begin
			if (is_spr) begin pp_we <= 1; pp_wa <= {fr_bank, y, sidx[3:0]}; end
			else        begin sp_we <= 1; sp_wa <= {fr_bank, y34 + {7'd0, sidx}}; end
			if (last_item) begin
				slot_ok[slot] <= 0;
				proc_y <= proc_y + 9'd1;
				lst    <= L_IDLE;
			end else begin
				item <= item + 6'd1;
				if (!is_spr && item + 6'd1 == nspan) begin
					is_spr <= 1;
					sidx   <= 0;
				end else
					sidx <= sidx + 6'd1;
				lst <= I_A;
			end
		end

		L_CLR: begin
			clr <= clr + 12'd1;
			if (clr == 12'hFFF) lst <= L_IDLE;
		end

		default: lst <= L_IDLE;
	endcase
	id_wd <= id_r;

	if (reset) begin
		lst     <= L_IDLE;
		ready_n <= 0;
		proc_y  <= 9'd240;
		slot_ok <= 0;
		nf_pend <= 0;
		tog_d   <= tog_r;               // no frame begun
		dd_done <= 0;
		for (b = 0; b < 2; b = b + 1) nrec[b] <= 0;
		inj      <= 0;
		clr_pend <= 0;
	end
end

// ------------------------------------------------------------------------------------------
// resolver
// ------------------------------------------------------------------------------------------
reg         lk_req = 0;
reg  [31:0] lq_pal, lq_lo, lq_hi;
wire        lk_done, lk_found;
wire [31:0] lk_off, lk_count, lk_pp, lk_pi;
wire        lk_pv;
wire  [3:0] lk_probes;
wire        lk_rd_req;
wire [27:3] lk_rd_addr;

hd3_lookup lookup
(
	.clk(clk), .reset(reset),
	.keys_base(keys_base), .rules_base(rules_base), .bucket_mask(bucket_mask),
	.max_probe(max_probe),
	.req(lk_req), .key_only(1'b1),
	.q_pal(lq_pal), .q_kind(game_ram), .q_lo(lq_lo), .q_hi(lq_hi),
	.done(lk_done), .found(lk_found), .list_off(lk_off), .list_count(lk_count),
	.pick_valid(lk_pv), .pick_payload(lk_pp), .pick_index(lk_pi), .probes(lk_probes),
	.rd_req(lk_rd_req), .rd_addr(lk_rd_addr), .rd_ack(rd_ack), .rd_data(rd_data),
	.rd_valid(rd_valid)
);


// additions (8a): a second lookup unit for the addition-key table (section 16); its list
// pointer is relative to section 17
reg         alk_req = 0;
reg  [31:0] aq_pal, aq_lo, aq_hi;
wire        alk_done, alk_found;
wire [31:0] alk_off, alk_count, alk_pp, alk_pi;
wire        alk_pv;
wire  [3:0] alk_probes;
wire        alk_rd_req;
wire [27:3] alk_rd_addr;

hd3_lookup alookup
(
	.clk(clk), .reset(reset),
	.keys_base(akeys_base), .rules_base(alists_base), .bucket_mask(abucket_mask),
	.max_probe(amax_probe),
	.req(alk_req), .key_only(1'b1),
	.q_pal(aq_pal), .q_kind(game_ram), .q_lo(aq_lo), .q_hi(aq_hi),
	.done(alk_done), .found(alk_found), .list_off(alk_off), .list_count(alk_count),
	.pick_valid(alk_pv), .pick_payload(alk_pp), .pick_index(alk_pi), .probes(alk_probes),
	.rd_req(alk_rd_req), .rd_addr(alk_rd_addr), .rd_ack(rd_ack), .rd_data(rd_data),
	.rd_valid(rd_valid)
);

// fallback map and addition list reads use the same DDR3 port (one requester at a time: the
// resolver is sequential)
reg         fm_req = 0;
reg  [27:3] fm_addr;
reg         al_req = 0;
reg  [27:3] al_addr;
assign rd_req  = lk_rd_req | fm_req | alk_rd_req | al_req;
assign rd_addr = fm_req ? fm_addr : al_req ? al_addr : alk_rd_req ? alk_rd_addr : lk_rd_addr;

typedef enum bit [4:0] {
	R_IDLE, R_KEY, R_EXW, R_FB0, R_FB1, R_FB2, R_FBP, R_FBD, R_DEFW, R_F2,
	R_A0, R_AK, R_AKW, R_AL0, R_AL1, R_MH, R_MH1, R_MH2, R_MS, R_ME0, R_ME1, R_MI, R_MR, R_MR1,
	R_WR
} rst_t;
rst_t rst = R_IDLE;

reg         res_bank = 0;
reg [RAW:0] res_ptr = 0;
reg  [31:0] r_pal;
reg  [63:0] r_body;
reg         ex_found;
reg  [24:0] ex_offq;
reg   [1:0] via;
reg  [24:0] offq;
reg  [16:0] fb, fb2;                // {valid, tile}
reg   [2:0] beat;
reg         got;
reg  [15:0] fm_idx;                 // tile whose map entry is being read
reg         fm_second;              // reading fallback(fallback(idx))

wire        rs_grant = rs_kreq && !dd_ka;
wire [31:0] fm_word  = fm_idx[0] ? rd_data[63:32] : rd_data[31:0];

task automatic start_lookup(input [31:0] p, input [63:0] body);
	lq_pal <= p;
	lq_lo  <= body[31:0];
	lq_hi  <= body[63:32];
	lk_req <= 1;
endtask

// ---- additions: the up-to-four lists a record matches - (palette, body), (any, body) and,
// CHR-ROM with a fallback F = fallback(body) != body, (palette, F) and (any, F) - read into a
// 64-entry buffer, then merged in file order (the lists are disjoint, each in file order).
// Each merged entry's added key gets an id from the dictionary and goes to the pool. ----
localparam ABUF = 64;
reg  [47:0] ab0 [ABUF];             // {oy 16, ox 16, file index 16}
reg  [31:0] ab1 [ABUF];             // added palette
reg  [63:0] ab2 [ABUF];             // added body
reg   [5:0] ab_ra;
reg  [47:0] ab0_q;
reg  [31:0] ab1_q;
reg  [63:0] ab2_q;
reg         ab_we;
reg   [5:0] ab_wa;
reg   [1:0] ab_ww;                  // which word
reg  [63:0] ab_wd;
always @(posedge clk) begin
	if (ab_we && ab_ww == 2'd0) ab0[ab_wa] <= ab_wd[47:0];
	if (ab_we && ab_ww == 2'd1) ab1[ab_wa] <= ab_wd[31:0];
	if (ab_we && ab_ww == 2'd2) ab2[ab_wa] <= ab_wd;
	ab0_q <= ab0[ab_ra];
	ab1_q <= ab1[ab_ra];
	ab2_q <= ab2[ab_ra];
end

reg   [2:0] ad_j;                   // list being looked up (0..3)
reg   [6:0] ad_n;                   // entries buffered
reg         ad_ovf;                 // more than ABUF entries, or no id / pool space
reg   [6:0] ls_start [4];
reg   [6:0] ls_len [4];
reg  [27:3] al_q;                   // next burst of the list being read
reg  [31:0] al_pos;                 // qword within the list (0 = {count, 0})
reg  [31:0] al_cnt;
reg   [1:0] al_w;                   // word of the entry (0..2)
reg   [6:0] al_e;                   // entry within the list
reg   [6:0] hp [4];                 // merge: next entry of each list
reg   [6:0] rem [4];                //   entries left
reg  [15:0] hidx [4];               //   file index of the next entry
reg   [1:0] mh_j, ms_sel;
reg  [15:0] e_idx, e_ox, e_oy;
reg   [6:0] add_cnt;
reg   [9:0] add_start;
reg  [10:0] ppool [2];              // pool entries used, per bank (1,024 = full)
reg   [1:0] seen_tog = 0;           // ep_d as last seen: a change = the bank's new frame
reg   [1:0] rp_ep = 0;              // ep_d as last seen at R_IDLE: a change = restart the walk
reg   [1:0] rdone = 2'b11;          // the bank's latest frame is reported done (or none begun)
initial begin ppool[0] = 0; ppool[1] = 0; end

wire        fbv     = !game_ram && fb[16] && fb[15:0] != r_body[15:0];
wire [31:0] aj_pal  = ad_j[0] ? 32'hFFFFFFFF : r_pal;
wire [63:0] aj_body = ad_j[1] ? {48'd0, fb[15:0]} : r_body;

task automatic start_alookup(input [31:0] p, input [63:0] body);
	aq_pal  <= p;
	aq_lo   <= body[31:0];
	aq_hi   <= body[63:32];
	alk_req <= 1;
endtask

// smallest file index among the lists with entries left
reg   [1:0] msel;
reg         many;
always @* begin
	msel = 0; many = 0;
	for (integer j = 0; j < 4; j = j + 1)
		if (rem[j] != 7'd0 && (!many || hidx[j] < hidx[msel])) begin msel = 2'(j); many = 1; end
end

always @(posedge clk) begin
	lk_req     <= 0;
	alk_req    <= 0;
	frame_done <= 0;
	rs_we      <= 0;
	am_we      <= 0;
	ap_we      <= 0;
	ab_we      <= 0;
	if (rd_ack) begin fm_req <= 0; al_req <= 0; end
	if (inj_done) inj_req <= 0;

	// a bank's new frame: its addition state starts over
	for (integer bb = 0; bb < 2; bb = bb + 1)
		if (seen_tog[bb] != ep_d[bb]) begin
			seen_tog[bb] <= ep_d[bb];
			fovf_a[bb]   <= 0;
			ppool[bb]    <= 0;
			// the previous frame of this bank was never finished: retire it (abandoned, no
			// frame_done) so the new frame counts as begun
			if (!rdone[bb]) tog_r[bb] <= ~tog_r[bb];
			rdone[bb]    <= 0;
		end

	case (rst)
		R_IDLE: begin
			rs_kreq <= 0;
			// a new frame began on this bank since the walk started: walk it from record 0
			// (normally res_ptr is 0 already; after a restart it was part-way through the
			// abandoned frame)
			if (rp_ep[res_bank] != ep_d[res_bank]) begin
				rp_ep[res_bank] <= ep_d[res_bank];
				res_ptr         <= 0;
			// only a STARTED bank: right after finishing a frame the resolver moves to the
			// other bank, whose nrec still counts the records of two frames ago until its
			// next frame begins (and resets it to 0, with res_ptr already 0)
			end else if (started[res_bank] && res_ptr != nrec[res_bank]) begin
				rs_ka   <= {res_bank, res_ptr[RAW-1:0]};
				rs_kreq <= 1;
				if (rs_kreq && rs_grant) begin       // the address was on port A this clock
					rs_kreq <= 0;
					rst     <= R_KEY;
				end
			end else if (started[res_bank] && dd_done[res_bank]) begin
				frame_done      <= 1;
				done_bank       <= res_bank;
				tog_r[res_bank] <= ~tog_r[res_bank];
				rdone[res_bank] <= 1;
				res_bank        <= ~res_bank;
				res_ptr         <= 0;
			end else if (!started[res_bank] && started[~res_bank]) begin
				res_bank <= ~res_bank;
				res_ptr  <= 0;
			end
		end

		R_KEY: begin                                 // ka_q = the record's key
			r_pal  <= ka_q[95:64];
			r_body <= ka_q[63:0];
			fb     <= 0;
			fb2    <= 0;
			start_lookup(ka_q[95:64], ka_q[63:0]);
			rst    <= R_EXW;
		end

		R_EXW: if (lk_done) begin
			ex_found <= lk_found;
			ex_offq  <= lk_off[27:3];
			if (game_ram) begin
				if (lk_found) begin via <= 2'd1; offq <= lk_off[27:3]; rst <= R_A0; end
				else begin start_lookup(32'hFFFFFFFF, r_body); rst <= R_DEFW; end
			end else
				rst <= R_FB0;
		end

		// ---- fallback(idx) ----
		R_FB0: begin
			fm_second <= 0;
			fm_idx    <= r_body[15:0];
			// hdref.fallback: an entry only for 0 <= idx < len. A captured tile's index always
			// fits 16 bits; an added key's body (8a) may not, or may be negative
			if (r_body[63:16] == 48'd0 && {16'd0, r_body[15:0]} < fb_len) begin
				fm_req  <= 1;
				fm_addr <= fb_base + {10'd0, r_body[15:1]};
				beat    <= 0;
				got     <= 0;
				rst     <= R_FB1;
			end else
				rst <= R_FB2;                           // no entry: fb stays invalid
		end
		R_FB1: if (rd_valid) begin                       // first beat holds the entry
			if (!got) begin
				got <= 1;
				if (!fm_word[31]) begin
					if (fm_second) fb2 <= {1'b1, fm_word[15:0]};
					else           fb  <= {1'b1, fm_word[15:0]};
				end
			end
			beat <= beat + 3'd1;
			if (beat == 3'd7) rst <= fm_second ? R_A0 : R_FB2;
		end
		R_FB2: begin
			if (ex_found) begin
				via  <= 2'd1;
				offq <= ex_offq;
				rst  <= R_F2;
			end else if (fb[16]) begin
				start_lookup(r_pal, {48'd0, fb[15:0]});
				rst <= R_FBP;
			end else begin
				start_lookup(32'hFFFFFFFF, r_body);
				rst <= R_DEFW;
			end
		end
		R_FBP: if (lk_done) begin
			if (lk_found) begin via <= 2'd2; offq <= lk_off[27:3]; rst <= R_F2; end
			else begin start_lookup(32'hFFFFFFFF, {48'd0, fb[15:0]}); rst <= R_FBD; end
		end
		R_FBD: if (lk_done) begin
			if (lk_found) begin via <= 2'd2; offq <= lk_off[27:3]; rst <= R_F2; end
			else begin start_lookup(32'hFFFFFFFF, r_body); rst <= R_DEFW; end
		end
		R_DEFW: if (lk_done) begin
			via  <= lk_found ? 2'd3 : 2'd0;
			offq <= lk_found ? lk_off[27:3] : 25'd0;
			rst  <= game_ram ? R_A0 : R_F2;
		end

		// ---- fallback(fallback(idx)) ----
		R_F2: begin
			if (fb[16] && {16'd0, fb[15:0]} < fb_len) begin
				fm_second <= 1;
				fm_idx    <= fb[15:0];
				fm_req    <= 1;
				fm_addr   <= fb_base + {10'd0, fb[15:1]};
				beat      <= 0;
				got       <= 0;
				rst       <= R_FB1;
			end else
				rst <= R_A0;
		end

		// ---- additions ----
		R_A0: begin
			ad_j      <= 0;
			ad_n      <= 0;
			ad_ovf    <= 0;
			add_cnt   <= 0;
			add_start <= ppool[res_bank][9:0];
			for (integer j = 0; j < 4; j = j + 1) begin ls_start[j] <= 0; ls_len[j] <= 0; end
			// injm_q: this record was allocated for an added key (no additions of its own)
			rst <= (add_en && !injm_q) ? R_AK : R_WR;
		end
		R_AK: begin
			if (ad_j == 3'd4 || (ad_j == 3'd2 && !fbv) || ad_ovf) begin
				for (integer j = 0; j < 4; j = j + 1) begin hp[j] <= ls_start[j]; rem[j] <= ls_len[j]; end
				mh_j <= 0;
				rst  <= ad_ovf ? R_WR : R_MH;
			end else begin
				start_alookup(aj_pal, aj_body);
				rst <= R_AKW;
			end
		end
		R_AKW: if (alk_done) begin
			ls_start[ad_j[1:0]] <= ad_n;
			if (alk_found) begin
				al_q   <= alists_base + alk_off[27:3];
				al_pos <= 0;
				al_w   <= 0;
				al_e   <= 0;
				rst    <= R_AL0;
			end else begin
				ad_j <= ad_j + 3'd1;
				rst  <= R_AK;
			end
		end
		R_AL0: begin
			al_req  <= 1;
			al_addr <= al_q;
			beat    <= 0;
			rst     <= R_AL1;
		end
		R_AL1: if (rd_valid) begin : albeat
			reg [31:0] cnt;
			reg [31:0] q3;
			// the count arrives as qword 0 and is latched; it is first USED at beat 7 (lists are
			// 8-byte aligned and every burst starts at al_q, so beat 7 is never qword 0)
			cnt = al_cnt;
			q3  = {cnt[29:0], 1'b0} + cnt;               // 3 * count
			if (al_pos == 32'd0) begin
				al_cnt <= rd_data[31:0];
				if ({25'd0, ad_n} + rd_data[31:0] > ABUF) ad_ovf <= 1;
			end else if (al_pos <= q3 && !ad_ovf) begin
				ab_we <= 1;
				ab_wa <= 6'(ad_n + al_e);
				ab_ww <= al_w;
				ab_wd <= rd_data;
				if (al_w == 2'd2) begin al_w <= 0; al_e <= al_e + 7'd1; end
				else al_w <= al_w + 2'd1;
			end
			al_pos <= al_pos + 32'd1;
			beat   <= beat + 3'd1;
			if (beat == 3'd7) begin
				// the burst is drained; next burst, or the list is complete
				if (al_pos >= q3 || ad_ovf || ({25'd0, ad_n} + cnt > ABUF)) begin
					if (!ad_ovf && {25'd0, ad_n} + cnt <= ABUF) begin
						ls_len[ad_j[1:0]] <= cnt[6:0];
						ad_n <= ad_n + cnt[6:0];
					end
					ad_j <= ad_j + 3'd1;
					rst  <= R_AK;
				end else begin
					al_q <= al_q + 25'd8;
					rst  <= R_AL0;
				end
			end
		end
		// merge: first the head index of each list (buffer read: address, then a clock for
		// the read, then the data)
		R_MH: begin ab_ra <= 6'(hp[mh_j]); rst <= R_MH1; end
		R_MH1: rst <= R_MH2;
		R_MH2: begin
			hidx[mh_j] <= ab0_q[15:0];
			if (mh_j == 2'd3) rst <= R_MS;
			else begin mh_j <= mh_j + 2'd1; rst <= R_MH; end
		end
		R_MS: begin
			if (!many) rst <= R_WR;                      // every list merged
			else begin
				ms_sel <= msel;
				ab_ra  <= 6'(hp[msel]);
				rst    <= R_ME0;
			end
		end
		R_ME0: rst <= R_ME1;
		R_ME1: begin
			e_idx    <= ab0_q[15:0];
			e_ox     <= ab0_q[31:16];
			e_oy     <= ab0_q[47:32];
			inj_key  <= {ab1_q, ab2_q};
			inj_bank <= res_bank;
			inj_req  <= 1;
			hp[ms_sel]  <= hp[ms_sel] + 7'd1;
			rem[ms_sel] <= rem[ms_sel] - 7'd1;
			rst <= R_MI;
		end
		R_MI: if (inj_done) begin
			if (!inj_id[RAW] || ppool[res_bank][10]) begin
				// no id (the frame closed, or out of ids) or no pool space: the frame is
				// incomplete (8o)
				ovf_a <= 1;
				fovf_a[res_bank] <= 1;
				rst <= R_WR;
			end else begin
				ap_we <= 1;
				ap_wa <= {res_bank, ppool[res_bank][9:0]};
				ap_wd <= {e_idx, e_oy, e_ox, inj_id[RAW-1:0]};
				ppool[res_bank] <= ppool[res_bank] + 11'd1;
				add_cnt <= add_cnt + 7'd1;
				if (rem[ms_sel] != 7'd0) begin ab_ra <= 6'(hp[ms_sel]); rst <= R_MR; end
				else rst <= R_MS;
			end
		end
		R_MR: rst <= R_MR1;                          // the list's next head: read
		R_MR1: begin hidx[ms_sel] <= ab0_q[15:0]; rst <= R_MS; end

		R_WR: begin
			rs_we   <= 1;
			rs_wa   <= {res_bank, res_ptr[RAW-1:0]};
			rs_wd   <= res_word(via, offq, fb, fb2) | {29'd0, add_cnt != 7'd0, 34'd0};
			am_we   <= 1;
			am_wa   <= {res_bank, res_ptr[RAW-1:0]};
			am_wd   <= {ad_ovf ? 7'd0 : add_cnt, add_start};
			if (ad_ovf) begin ovf_a <= 1; fovf_a[res_bank] <= 1; end
			res_ptr <= res_ptr + 1'd1;
			rst     <= R_IDLE;
		end

		default: rst <= R_IDLE;
	endcase

	if (reset) begin
		rst      <= R_IDLE;
		rs_kreq  <= 0;
		fm_req   <= 0;
		al_req   <= 0;
		inj_req  <= 0;
		res_bank <= 0;
		res_ptr  <= 0;
		rdone    <= 2'b11;
		seen_tog <= ep_d;
		rp_ep    <= ep_d;
	end
end

// injm read address: the record the resolver is on
always @* ij_ra = {res_bank, res_ptr[RAW-1:0]};

endmodule
