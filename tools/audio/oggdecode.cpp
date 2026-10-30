// oggdecode - decode an OGG Vorbis file exactly as Mesen2's HD pack audio does
// (Mesen2's own stb_vorbis + HermiteResampler, tools/audio/mesen/), for hdp3_convert.
//
//   oggdecode info FILE
//       prints "rate=R channels=C frames=N" (N = stb_vorbis_stream_length_in_samples)
//   oggdecode render FILE OUT RATE LOOPPOS
//       decodes to 2-channel interleaved s16 (stb_vorbis_get_samples_short_interleaved, as
//       OggReader::ApplySamples requests) and resamples to RATE with Mesen's Hermite resampler,
//       the whole stream once through one continuous resampler. Writes OUT (little-endian s16
//       stereo) and prints "frames=F loopframe=K ...":
//         K = the output frame a loop restarts at. Hermite path: the number of frames emitted
//             before source frame LOOPPOS + 4 is fed (each output reads four source samples,
//             so that is the first output made wholly from frames >= LOOPPOS). Equal rates:
//             Mesen copies samples through with no history, so K = LOOPPOS. 0 when LOOPPOS is 0.
//       LOOPPOS >= stream length means 0, as OggReader::Init does. The resampler is fed one
//       source frame at a time, which gives the same output as Mesen's per-mix-call feeding:
//       its state (history, fraction) is carried across calls, and the pending-sample queue
//       only defers output, never alters it. (HDPACK_DESIGN.md "Stage 6 - audio".)
//   oggdecode adpcm FILE OUT RATE LOOPPOS
//       the same PCM, encoded as the pack stores it: IMA ADPCM, one byte per stereo frame,
//       low nibble left, high nibble right. Prints "frames=F loopframe=K state0=S0 statel=SL"
//       with the states as 16 hex digits, [15:0] pred L, [22:16] index L, [47:32] pred R,
//       [54:48] index R: state0 = the state the decoder starts from (pred = first sample,
//       index 0), statel = the encoder's state after frames [0, K), i.e. what a decoder has
//       there, so a loop that restores it replays the first pass exactly. Also writes OUT.ck:
//       the state before every 16384th frame (little-endian u64 each, state0 first), the
//       section's restore checkpoints (hdaudio.checkpoints).
//       The encoder looks ahead 3 samples (ima_encode_la); depth 0 is the original greedy one.
//   oggdecode ckscan IN OUT STATE0
//       decodes a stored stream from STATE0 (16 hex digits) and writes its checkpoints to OUT,
//       the file encode_and_write writes beside its stream (for hdp3_read.check_audio)
//   oggdecode pcm2adpcm IN OUT LOOPFRAME [DEPTH]
//       encodes an s16 stereo file (for tests: tools/audio/test_ima.py); DEPTH 0-4, default 3
//   oggdecode snr FILE RATE [DEPTH]
//       signal-to-noise of the stored ADPCM against the PCM Mesen mixes (render from the
//       start): overall per channel, peak error, and the worst 100 ms window that is not
//       near-silent (HDPACK_DESIGN.md "Stage 6 - ADPCM against Mesen")
//
// Exit 0 ok, 1 file cannot be opened/decoded, 2 usage.
#include "pch.h"
#include <cmath>
#include "mesen/stb_vorbis.h"
#include "mesen/HermiteResampler.h"

static int usage()
{
	fprintf(stderr, "usage: oggdecode info FILE | render|adpcm FILE OUT RATE LOOPPOS |"
		" pcm2adpcm IN OUT LOOPFRAME [DEPTH] | snr FILE RATE [DEPTH] | ckscan IN OUT STATE0\n");
	return 2;
}

// ---- IMA ADPCM (the decoder rules are the hardware's; tools/audio/hdaudio.py has a copy) ----
static const int16_t kStep[89] = {
	7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60,
	66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371,
	408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878,
	2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845,
	8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086,
	29794, 32767 };
static const int8_t kIdx[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

struct Ima { int pred; int idx; };

static void ima_decode(Ima& s, int n)
{
	int step = kStep[s.idx];
	int diff = step >> 3;
	if(n & 4) diff += step;
	if(n & 2) diff += step >> 1;
	if(n & 1) diff += step >> 2;
	s.pred += (n & 8) ? -diff : diff;
	if(s.pred > 32767) s.pred = 32767;
	if(s.pred < -32768) s.pred = -32768;
	s.idx += kIdx[n & 7];
	if(s.idx < 0) s.idx = 0;
	if(s.idx > 88) s.idx = 88;
}

static int ima_encode(Ima& s, int sample)
{
	int step = kStep[s.idx];
	int delta = sample - s.pred;
	int n = 0;
	if(delta < 0) { n = 8; delta = -delta; }
	if(delta >= step) { n |= 4; delta -= step; }
	if(delta >= (step >> 1)) { n |= 2; delta -= step >> 1; }
	if(delta >= (step >> 2)) { n |= 1; }
	ima_decode(s, n);   // keep the encoder in step with the decoder
	return n;
}

// ---- lookahead encoder (OPEN_ITEMS 13f; user decision 2026-10-25: depth 3) ----
// Picks each nibble by the smallest total squared error over the next `depth` samples (this one
// and depth-1 after it), then commits only this sample's nibble. The decoder, and so the stored
// format and the hardware, are unchanged: only the choice of nibble is better. The gain comes
// from raising the step BEFORE an edge, which the greedy encoder cannot do; it matters most on
// chiptune square waves (Metal 7.4 -> 14.3 dB), a little on music.
// Levels before the last try all 16 nibbles. The last level only needs the smallest error, and
// that is exact without trying all 16: for a given sign the reconstruction is a non-decreasing
// function of the magnitude (the bit weights step, step/2, step/4 each exceed the sum below
// them, and the clamp keeps it monotone), so the error is smallest at one of the two
// magnitudes either side of the target. The other sign moves away from the target, so its best
// is magnitude 0, which can only tie or win through the clamp (pred on a rail, target on it);
// it is the third candidate. test_ima.py checks this against all 16 nibbles.
// Ties go to the lowest nibble, which the Python model (hdaudio.encode_stereo) matches.
static const int kDepth = 3;
static const size_t kCkSpacing = 16384;   // hdaudio.CK_SPACING, aud_arm.h AUD_CK

static int64_t sq(int e) { return (int64_t)e * e; }

// smallest squared error any nibble gives for `sample` from state s (the last level)
static int64_t best_last(const Ima& s, int sample)
{
	int step = kStep[s.idx];
	int delta = sample - s.pred;
	int neg = delta < 0;
	int64_t best = INT64_MAX;
	int lo = 0;
	for(int m = 0; m < 8; m++) {                 // largest m whose reconstruction is <= target
		Ima t = s;
		ima_decode(t, m | (neg ? 8 : 0));
		if(neg ? t.pred >= sample : t.pred <= sample) lo = m;
		else break;
	}
	for(int m = lo; m <= lo + 1 && m < 8; m++) {
		Ima t = s;
		ima_decode(t, m | (neg ? 8 : 0));
		int64_t e = sq(sample - t.pred);
		if(e < best) best = e;
	}
	Ima t = s;                                   // the other sign, magnitude 0: only the clamp
	ima_decode(t, neg ? 0 : 8);                  // can make it win (pred at the rail, target
	int64_t e = sq(sample - t.pred);             // on it), and its error grows with magnitude
	if(e < best) best = e;
	return best;
}

// smallest total error over samples x[0..k) from state s
static int64_t best_cost(const Ima& s, const int16_t* x, size_t stride, int k)
{
	if(k == 1) return best_last(s, x[0]);
	int64_t best = INT64_MAX;
	for(int n = 0; n < 16; n++) {
		Ima t = s;
		ima_decode(t, n);
		int64_t c = sq(x[0] - t.pred) + best_cost(t, x + stride, stride, k - 1);
		if(c < best) best = c;
	}
	return best;
}

// encode one sample: x points at it in the interleaved buffer, `left` samples remain in this
// channel including it. depth 0 = the original greedy encoder.
static int ima_encode_la(Ima& s, const int16_t* x, size_t stride, size_t left, int depth)
{
	if(depth == 0) return ima_encode(s, x[0]);
	int k = (int)(left < (size_t)depth ? left : (size_t)depth);
	int bestN = 0;
	int64_t best = INT64_MAX;
	for(int n = 0; n < 16; n++) {
		Ima t = s;
		ima_decode(t, n);
		int64_t c = sq(x[0] - t.pred) + (k > 1 ? best_cost(t, x + stride, stride, k - 1) : 0);
		if(c < best) { best = c; bestN = n; }
	}
	ima_decode(s, bestN);
	return bestN;
}

static uint64_t pack_state(const Ima& l, const Ima& r)
{
	return (uint64_t)(uint16_t)l.pred | ((uint64_t)l.idx << 16)
		| ((uint64_t)(uint16_t)r.pred << 32) | ((uint64_t)r.idx << 48);
}

static int encode_and_write(const vector<int16_t>& pcm, size_t loopFrame, const char* outPath,
	int depth)
{
	size_t frames = pcm.size() / 2;
	Ima l = { frames ? pcm[0] : 0, 0 }, r = { frames ? pcm[1] : 0, 0 };
	uint64_t state0 = pack_state(l, r), statel = state0;
	vector<uint8_t> out(frames);
	// restore checkpoints (hdaudio.checkpoints): the decoder's state before every
	// kCkSpacing-th frame; the encoder's state IS the decoder's (it decodes each nibble it picks)
	vector<uint64_t> ck(1, state0);
	for(size_t i = 0; i < frames; i++) {
		if(i == loopFrame) statel = pack_state(l, r);
		int nl = ima_encode_la(l, &pcm[2 * i], 2, frames - i, depth);
		int nr = ima_encode_la(r, &pcm[2 * i + 1], 2, frames - i, depth);
		out[i] = (uint8_t)(nl | (nr << 4));
		if((i + 1) % kCkSpacing == 0) ck.push_back(pack_state(l, r));
	}
	FILE* o = fopen(outPath, "wb");
	if(!o) { fprintf(stderr, "oggdecode: cannot write %s\n", outPath); return 1; }
	fwrite(out.data(), 1, out.size(), o);
	fclose(o);
	string ckPath = string(outPath) + ".ck";     // little-endian u64 each (x86, ARM)
	FILE* c = fopen(ckPath.c_str(), "wb");
	if(!c) { fprintf(stderr, "oggdecode: cannot write %s\n", ckPath.c_str()); return 1; }
	fwrite(ck.data(), 8, ck.size(), c);
	fclose(c);
	printf("frames=%zu loopframe=%zu state0=%016llx statel=%016llx\n", frames, loopFrame,
		(unsigned long long)state0, (unsigned long long)statel);
	return 0;
}

// ---- OGG -> PCM at the output rate ----
static stb_vorbis* open_file(const char* path, vector<uint8_t>& data)
{
	FILE* f = fopen(path, "rb");
	if(!f) return nullptr;
	fseek(f, 0, SEEK_END);
	long n = ftell(f);
	fseek(f, 0, SEEK_SET);
	data.resize(n > 0 ? (size_t)n : 0);
	size_t got = n > 0 ? fread(data.data(), 1, data.size(), f) : 0;
	fclose(f);
	if(got != data.size() || data.empty()) return nullptr;
	int err = 0;
	return stb_vorbis_open_memory(data.data(), (int)data.size(), &err, nullptr);
}

// feed src frames [a, b) of the decoded stream one at a time, appending output frames
static void feed(HermiteResampler& r, const vector<int16_t>& src, size_t a, size_t b, vector<int16_t>& out)
{
	int16_t buf[64];
	for(size_t i = a; i < b; i++) {
		int16_t in[2] = { src[2 * i], src[2 * i + 1] };
		uint32_t n = r.Resample<false>(in, 1, buf, 16);
		out.insert(out.end(), buf, buf + 2 * n);
	}
}

// decode + resample; returns 0 and fills out/loopFrame, or 1
static int render(const char* path, unsigned rate, unsigned loopPos, vector<int16_t>& out, size_t& loopFrame,
	unsigned& srcRate, size_t& srcFrames)
{
	vector<uint8_t> data;
	stb_vorbis* v = open_file(path, data);
	if(!v) { fprintf(stderr, "oggdecode: cannot open %s\n", path); return 1; }
	stb_vorbis_info info = stb_vorbis_get_info(v);
	unsigned int len = stb_vorbis_stream_length_in_samples(v);
	if(loopPos >= len) loopPos = 0;
	vector<int16_t> src;
	int16_t chunk[8192];
	for(;;) {
		int got = stb_vorbis_get_samples_short_interleaved(v, 2, chunk, 8192);
		if(got <= 0) break;
		src.insert(src.end(), chunk, chunk + 2 * got);
	}
	stb_vorbis_close(v);
	size_t frames = src.size() / 2;
	if(frames == 0) { fprintf(stderr, "oggdecode: no samples in %s\n", path); return 1; }
	if(loopPos >= frames) loopPos = 0;

	HermiteResampler r;
	r.SetSampleRates((double)info.sample_rate, (double)rate);
	bool same = info.sample_rate == rate;
	size_t mark = loopPos ? (size_t)loopPos + (same ? 0 : 4) : 0;
	if(mark > frames) mark = frames;
	feed(r, src, 0, mark, out);
	loopFrame = loopPos ? out.size() / 2 : 0;
	feed(r, src, mark, frames, out);
	if(loopFrame >= out.size() / 2) loopFrame = 0;
	srcRate = info.sample_rate;
	srcFrames = frames;
	return 0;
}

int main(int argc, char** argv)
{
	if(argc < 3) return usage();
	string mode = argv[1];

	if(mode == "info" && argc == 3) {
		vector<uint8_t> data;
		stb_vorbis* v = open_file(argv[2], data);
		if(!v) { fprintf(stderr, "oggdecode: cannot open %s\n", argv[2]); return 1; }
		stb_vorbis_info info = stb_vorbis_get_info(v);
		printf("rate=%u channels=%d frames=%u\n", info.sample_rate, info.channels,
			stb_vorbis_stream_length_in_samples(v));
		stb_vorbis_close(v);
		return 0;
	}
	if(mode == "ckscan" && argc == 5) {
		// decode a stored ADPCM stream (one byte a frame) from STATE0 (16 hex digits) and write
		// OUT: its restore checkpoints, as encode_and_write writes them (hdp3_read.check_audio
		// compares a section's table against this over the whole stream)
		FILE* f = fopen(argv[2], "rb");
		if(!f) { fprintf(stderr, "oggdecode: cannot open %s\n", argv[2]); return 1; }
		vector<uint8_t> s;
		uint8_t b[65536];
		size_t n;
		while((n = fread(b, 1, sizeof b, f)) > 0) s.insert(s.end(), b, b + n);
		fclose(f);
		uint64_t st = strtoull(argv[4], nullptr, 16);
		Ima l = { (int16_t)(st & 0xFFFF), (int)((st >> 16) & 0x7F) };
		Ima r = { (int16_t)((st >> 32) & 0xFFFF), (int)((st >> 48) & 0x7F) };
		vector<uint64_t> ck(1, st);
		for(size_t i = 0; i < s.size(); i++) {
			ima_decode(l, s[i] & 15);
			ima_decode(r, s[i] >> 4);
			if((i + 1) % kCkSpacing == 0) ck.push_back(pack_state(l, r));
		}
		FILE* o = fopen(argv[3], "wb");
		if(!o) { fprintf(stderr, "oggdecode: cannot write %s\n", argv[3]); return 1; }
		fwrite(ck.data(), 8, ck.size(), o);
		fclose(o);
		return 0;
	}
	if(mode == "pcm2adpcm" && (argc == 5 || argc == 6)) {
		int depth = argc == 6 ? atoi(argv[5]) : kDepth;
		if(depth < 0 || depth > 4) return usage();
		FILE* f = fopen(argv[2], "rb");
		if(!f) { fprintf(stderr, "oggdecode: cannot open %s\n", argv[2]); return 1; }
		vector<int16_t> pcm;
		int16_t b[4096];
		size_t n;
		while((n = fread(b, 2, 4096, f)) > 0) pcm.insert(pcm.end(), b, b + n);
		fclose(f);
		pcm.resize(pcm.size() & ~(size_t)1);
		size_t lf = (size_t)strtoull(argv[4], nullptr, 10);
		if(lf >= pcm.size() / 2) lf = 0;
		return encode_and_write(pcm, lf, argv[3], depth);
	}
	if(mode == "snr" && (argc == 4 || argc == 5)) {
		int depth = argc == 5 ? atoi(argv[4]) : kDepth;
		if(depth < 0 || depth > 4) return usage();
		// the PCM Mesen mixes (render, from the start) against what the hardware plays: the
		// encoder's reconstruction, which is the decoder's output (ima_encode keeps the two in
		// step; test_ima.py pins this encoder to the model, tb_audio the RTL to the model).
		unsigned rate = (unsigned)strtoul(argv[3], nullptr, 10);
		if(rate == 0) return usage();
		vector<int16_t> pcm;
		size_t lf = 0, srcFrames = 0;
		unsigned srcRate = 0;
		if(render(argv[2], rate, 0, pcm, lf, srcRate, srcFrames)) return 1;
		size_t frames = pcm.size() / 2;
		Ima ch[2] = { { pcm[0], 0 }, { pcm[1], 0 } };
		double sig[2] = { 0, 0 }, err[2] = { 0, 0 }, wsig = 0, werr = 0, worst = 1e9;
		int peak = 0;
		size_t loud = 0, w = 0;
		for(size_t i = 0; i < frames; i++) {
			for(int c = 0; c < 2; c++) {
				int x = pcm[2 * i + c];
				ima_encode_la(ch[c], &pcm[2 * i + c], 2, frames - i, depth);
				int e = x - ch[c].pred;
				sig[c] += (double)x * x; err[c] += (double)e * e;
				wsig += (double)x * x; werr += (double)e * e;
				if(abs(e) > peak) peak = abs(e);
			}
			// worst 100 ms window that is not near-silent (rms >= 256, -42 dBFS)
			if(++w == rate / 10 || i + 1 == frames) {
				if(wsig / (2.0 * w) >= 256.0 * 256.0) {
					loud++;
					double s = werr > 0 ? 10 * log10(wsig / werr) : 200;
					if(s < worst) worst = s;
				}
				w = 0; wsig = 0; werr = 0;
			}
		}
		auto db = [](double s, double e) { return e > 0 ? 10 * log10(s / e) : 200.0; };
		printf("src_rate=%u frames=%zu snr_l=%.2f snr_r=%.2f snr=%.2f peak_err=%d "
			"loud_windows=%zu worst_window_snr=%.2f\n", srcRate, frames, db(sig[0], err[0]),
			db(sig[1], err[1]), db(sig[0] + sig[1], err[0] + err[1]), peak, loud,
			loud ? worst : 0.0);
		return 0;
	}
	if((mode != "render" && mode != "adpcm") || argc != 6) return usage();
	unsigned rate = (unsigned)strtoul(argv[4], nullptr, 10);
	unsigned loopPos = (unsigned)strtoul(argv[5], nullptr, 10);
	if(rate == 0) return usage();

	vector<int16_t> out;
	size_t loopFrame = 0, srcFrames = 0;
	unsigned srcRate = 0;
	if(render(argv[2], rate, loopPos, out, loopFrame, srcRate, srcFrames)) return 1;

	if(mode == "adpcm") return encode_and_write(out, loopFrame, argv[3], kDepth);

	FILE* o = fopen(argv[3], "wb");
	if(!o) { fprintf(stderr, "oggdecode: cannot write %s\n", argv[3]); return 1; }
	fwrite(out.data(), 2, out.size(), o);
	fclose(o);
	printf("frames=%zu loopframe=%zu rate=%u src_rate=%u src_frames=%zu looppos=%u\n",
		out.size() / 2, loopFrame, rate, srcRate, srcFrames, loopPos);
	return 0;
}
