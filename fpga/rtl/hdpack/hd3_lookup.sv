// hd3_lookup.sv - format-3 key lookup in DDR3 (stage 2).
//
// Finds a tile key in the pack's key table and walks its rule list.
//
//   key    = {palette[31:0], kind, body[63:0]}   (kind 1 = CHR-RAM: body is the tile
//            fingerprint; kind 0 = CHR-ROM: body = tile index, high word 0)
//   bucket = CRC-32 (poly 0x04C11DB7, init 0, MSB first, no final XOR) over the 12 bytes
//            palette(LE) + body(LE), masked to the bucket count. Linear probing over buckets,
//            at most max_probe+1 buckets; an empty entry ends the search.
//
// Layout (tools/hdp3_convert.py, HDPACK_DESIGN.md "Format 3"):
//   key entry, 16 bytes, 4 per 64-byte bucket:
//     q0 [31:0] palette  [63:32] body low
//     q1 [31:0] body high  [61:32] rule list pointer / 8  [62] kind  [63] valid
//   rule list: qword {u32 0, u32 count}, then count rules of 2 qwords:
//     qA = {w1 frame-condition list, w0 payload}   qB = {w3 flags, w2 walk-condition list}
//
// Stage 2 has no condition evaluator, so after the key is found the walk reports the FIRST
// rule whose condition lists are both empty (an unconditional rule). Stage 3 replaces the
// pick with the real condition walk; the key search does not change. With key_only set the
// lookup ends as soon as the key is decided (found, list_off; list_count and the pick are not
// read and stay 0): stage 3's record dictionary needs only the list offset, and its filter
// reads the list itself.
//
// One request at a time. Every DDR3 read is an 8-qword burst, data returned in order.

module hd3_lookup
(
	input             clk,
	input             reset,

	// configuration (from the pack header)
	input      [27:3] keys_base,       // qword address of the key table
	input      [27:3] rules_base,      // qword address of the rules section
	input      [31:0] bucket_mask,     // bucket count - 1 (power of 2)
	input      [15:0] max_probe,       // extra buckets searched beyond the first

	// request: inputs held stable from req until done
	input             req,
	input             key_only,        // stop after the key search (no rule-list read)
	input      [31:0] q_pal,
	input             q_kind,
	input      [31:0] q_lo,
	input      [31:0] q_hi,

	output reg        done = 0,        // one-clock pulse
	output reg        found = 0,       // key present
	output reg [31:0] list_off = 0,    // rule list byte offset in the rules section
	output reg [31:0] list_count = 0,  // rules in the list
	output reg        pick_valid = 0,  // an unconditional rule exists
	output reg [31:0] pick_payload = 0,// its w0 (block index or 0xFFFFFFFF)
	output reg [31:0] pick_index = 0,  // its position in the list
	output reg  [3:0] probes = 0,      // buckets read (diagnostic)

	// DDR3 read port (8-qword bursts)
	output reg        rd_req = 0,
	output reg [27:3] rd_addr = 0,
	input             rd_ack,
	input      [63:0] rd_data,
	input             rd_valid
);

// CRC-32 MSB-first over the 12 bytes in order, each byte MSB first. Unrolled by synthesis
// into an XOR network; must match tools/hdp3_convert.py crc32.
function [31:0] crc96(input [95:0] v);
	reg [31:0] r;
	integer    i;
	begin
		r = 32'd0;
		for (i = 95; i >= 0; i = i - 1)
			r = {r[30:0], 1'b0} ^ ((r[31] ^ v[i]) ? 32'h04C11DB7 : 32'd0);
		crc96 = r;
	end
endfunction

// the 12 bytes in stream order: palette LE, body low LE, body high LE
function [95:0] key_bytes(input [31:0] p, input [31:0] lo, input [31:0] hi);
	key_bytes = {p[7:0],  p[15:8],  p[23:16],  p[31:24],
	             lo[7:0], lo[15:8], lo[23:16], lo[31:24],
	             hi[7:0], hi[15:8], hi[23:16], hi[31:24]};
endfunction

typedef enum bit [2:0] { S_IDLE, S_HASH, S_KREQ, S_KDATA, S_RREQ, S_RDATA } state_t;
state_t st = S_IDLE;

reg  [31:0] h;            // bucket of probe 0
reg  [15:0] probe;
reg   [2:0] beat;
reg  [63:0] q0_hold;
reg         decided;      // this bucket ended the search
reg         hit;
reg  [29:0] hit_ptr;

reg  [27:3] list_q;       // qword address of the next burst of the rule list
reg  [31:0] qpos;         // qword position within the list (0 = header)
reg  [31:0] count;
reg  [63:0] qa_hold;
reg         got;

wire [31:0] bucket = (h + {16'd0, probe}) & bucket_mask;

// per-beat next values (blocking temporaries)
reg         dec_n, hit_n, got_n;
reg  [29:0] ptr_n;
reg  [31:0] cnt_n;

always @(posedge clk) begin
	done <= 0;
	if (rd_ack) rd_req <= 0;

	case (st)
		S_IDLE: if (req) begin
			st     <= S_HASH;
			probe  <= 0;
			probes <= 0;
		end

		S_HASH: begin
			h  <= crc96(key_bytes(q_pal, q_lo, q_hi));
			st <= S_KREQ;
		end

		S_KREQ: begin
			rd_req  <= 1;
			rd_addr <= keys_base + {bucket[21:0], 3'd0};   // bucket = 8 qwords
			beat    <= 0;
			decided <= 0;
			hit     <= 0;
			if (probes != 4'hF) probes <= probes + 4'd1;
			st      <= S_KDATA;
		end

		S_KDATA: if (rd_valid) begin
			dec_n = decided;
			hit_n = hit;
			ptr_n = hit_ptr;
			if (!beat[0]) q0_hold <= rd_data;
			else if (!decided) begin
				if (!rd_data[63]) begin
					dec_n = 1; hit_n = 0;                       // empty entry: absent
				end else if (q0_hold[31:0] == q_pal && q0_hold[63:32] == q_lo &&
				             rd_data[31:0] == q_hi && rd_data[62] == q_kind) begin
					dec_n = 1; hit_n = 1; ptr_n = rd_data[61:32];
				end
			end
			decided <= dec_n;
			hit     <= hit_n;
			hit_ptr <= ptr_n;
			beat    <= beat + 3'd1;

			if (beat == 3'd7) begin
				if (dec_n && hit_n) begin
					found    <= 1;
					list_off <= {ptr_n[28:0], 3'd0};
					list_q   <= rules_base + ptr_n[24:0];
					qpos     <= 0;
					got      <= 0;
					pick_valid <= 0;
					if (key_only) begin
						list_count <= 0;
						done       <= 1;
						st         <= S_IDLE;
					end else
						st <= S_RREQ;
				end else if (dec_n || probe == max_probe) begin
					found      <= 0;
					list_off   <= 0;
					list_count <= 0;
					pick_valid <= 0;
					done       <= 1;
					st         <= S_IDLE;
				end else begin
					probe <= probe + 16'd1;
					st    <= S_KREQ;
				end
			end
		end

		S_RREQ: begin
			rd_req  <= 1;
			rd_addr <= list_q;
			beat    <= 0;
			st      <= S_RDATA;
		end

		S_RDATA: if (rd_valid) begin
			got_n = got;
			cnt_n = count;
			if (qpos == 0) begin
				cnt_n = rd_data[31:0];
				list_count <= rd_data[31:0];
			end else if (!got && qpos <= {cnt_n[30:0], 1'b0}) begin
				// qwords 1,2 = rule 0; 3,4 = rule 1; ...  qA at odd positions
				if (qpos[0]) qa_hold <= rd_data;
				else if (qa_hold[63:32] == 32'd0 && rd_data[31:0] == 32'd0) begin
					got_n        = 1;
					pick_valid   <= 1;
					pick_payload <= qa_hold[31:0];
					pick_index   <= (qpos >> 1) - 32'd1;
				end
			end
			got   <= got_n;
			count <= cnt_n;
			qpos  <= qpos + 32'd1;
			beat  <= beat + 3'd1;

			if (beat == 3'd7) begin
				// done once the pick is known or every rule qword has been seen
				if (got_n || qpos >= {cnt_n[30:0], 1'b0}) begin
					done <= 1;
					st   <= S_IDLE;
				end else begin
					list_q <= list_q + 25'd8;
					st     <= S_RREQ;
				end
			end
		end

		default: st <= S_IDLE;
	endcase

	if (reset) begin
		st     <= S_IDLE;
		rd_req <= 0;
	end
end

endmodule
