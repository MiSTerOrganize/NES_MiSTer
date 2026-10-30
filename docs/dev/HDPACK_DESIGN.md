# NES HD Pack - test version design

Fork of MiSTer-devel/NES_MiSTer at github.com/MiSTerOrganize/NES_MiSTer (work on master,
baseline tag `pre-hdpack` = upstream 49a0a66), run the way odelot/NES_MiSTer is: upstream is
merged before each release and releases carry the RBF. Nothing is pushed until the user asks.
The goal is a working, measured core that runs every real Mesen HD pack, which can also back a
proposal to the MiSTer-devel maintainers.

## Scope of the test version (MVP)

In:
- Mesen HD pack tile replacement at 2x, CHR-ROM games only.
- Up to 8 sprites per line (the normal NES limit), layered the way Mesen layers them.
- One-bit transparency from the PNG alpha (alpha >= 128 = opaque).
- SD fallback for tiles the pack does not replace, using the pack's palette (or Mesen's default).
- New OSD entry "Load HD Pack" (file slot 4, `.HDP`), and an "HD Pack" on/off option, default Off.

Out of the MVP; all of these are now planned in "Full compatibility" below: CHR-RAM games,
conditions, background images, partial alpha / blending, 4x, audio replacement,
"extra sprites" > 8. (Brightness is already baked into the atlas by the converter.)

## How Mesen keys a tile (read from Mesen2 Core/NES/HdPacks, pack version >= 100)

- TileIndex = (absolute CHR ROM byte address) / 16.
- PaletteColors (32-bit):
  - background tile: `pal[0]<<24 | pal[p+1]<<16 | pal[p+2]<<8 | pal[p+3]`
  - sprite tile:     `0xFF<<24  | pal[p+1]<<16 | pal[p+2]<<8 | pal[p+3]`
- Lookup: exact (TileIndex, PaletteColors); on miss, (TileIndex, 0xFFFFFFFF) = "default tile".
- OffsetX/OffsetY = pixel within the 8x8 tile (8x16 sprites reduced mod 8); H/V flip mirror the HD tile.
- Draw order per pixel: backdrop colour -> behind-BG sprites (last to first) -> BG tile ->
  front sprites (last to first, only those in front of the lowest opaque behind-BG sprite).
  An HD sprite tile is drawn across all 8 columns it covers, its own alpha deciding
  coverage; a sprite without an HD tile draws its NES colour where its NES pixel is opaque.

## Mapping onto this core

- The core's mapper output `chr_linaddr` (rtl/nes.v) is the linear SDRAM address of every CHR
  fetch after banking. CHR ROM is loaded at 0x200000, so
  `TileIndex = (chr_linaddr - 0x200000) >> 4` and `OffsetY = chr_linaddr[2:0]`, valid for every
  mapper with no per-mapper work.
- PPU tap (rtl/ppu.sv), new outputs only, no change to existing behaviour:
  - BG: capture chr_linaddr on the pattern-low read (cycle[2:0]==6), shift it through a 2-tile
    pipe in step with BgPainter's pattern pipe; per pixel emit tile slot, fine x, palette select,
    NES colour index.
  - Sprites: capture chr_linaddr + attributes + X + pattern bytes for each of the 8 sprites at
    their fetch (cycles 257-320); publish the 8-entry sprite table for the next line.
  - Palette RAM exported read-only (32 x 6 bits).

## HD engine (new rtl/hdpack/*)

Three-stage line pipeline (triple-buffered), output delayed two NES lines (the scaler does not care):

1. BUILD (NES line N, core clock): write BG tile table (<= 34 entries), sprite table (8 entries),
   per-pixel BG descriptor (slot, fine x, colour index), palette snapshot.
2. FETCH (during NES line N+1, 85.9 MHz clock): for each table entry compute the key, probe the
   on-chip hash table (exact, then default), and on a hit burst-read the two needed HD rows
   (2 x 16 px x 32 bit) from the DDR3 atlas into the line cache.
3. OUTPUT (during NES line N+2): emit two output lines of 512 px at 21.48 MHz pixel rate
   (31 kHz line rate, like the scandoubler), compositing per output pixel in Mesen's order.

The HD stream replaces the scandoubler input to video_mixer when HD Pack is on and a pack is
loaded; otherwise the existing video path is untouched.

## Memory

- Hash table: on-chip M10K, 8192 entries x 72 bits (key 18+32, atlas index 16, valid) ~ 58 blocks.
- SD palette: 64 x 24 bits on-chip.
- Atlas: DDR3, one 16x16 RGBA8888 tile = 1 KB per entry, packed by the PC converter.
- DDR3 port moves to the 85.9 MHz clock; the existing savestate channel gets a clock crossing.
- Budget: ~42 hash probes + ~84 64-byte bursts per NES line, estimated ~30 us of the 63.5 us line.

## Pack file (.hdp), built on the PC by tools/hdp_convert.py

Little-endian. Header (magic `NHDP`, version, scale, counts) | 64-entry RGB palette |
hash table image | atlas tiles. The core streams it in through the normal OSD file load.

Format 2 (current): the table hash is CRC-13 (poly 0x1CF5) over the 44-bit key, with linear
probing capped at 16 steps. Format 1 used an XOR fold that clustered badly on real packs,
because they use few palettes. Worst-case probes on real packs, format 1 vs format 2:
Duck Hunt HD 69 vs 2, Ice Climber 38 vs 3, Pac-Man 129 vs 1. The core refuses any
version other than 2, so old .hdp files must be reconverted.

Header byte 7 holds flags. Bit 0 = legacy key: for packs older than version 100, Mesen keys a
tile on colours 1-3 only (no colour 0, no sprite marker), so the core clears colour 0 and
uses kind 0 for sprites too. Unknown flag bits make the core refuse the pack.

Converter rules taken from Mesen2's source, not guessed:
- Parsing is literal: lines split on '\n' with only trailing CR/LF removed (a line starting
  with a space is ignored), fields split on ',' with no trimming. Decimal fields use C++
  stoi (leading digits, so "565F" reads 565, a real case in Ninja Gaiden II); hex fields
  use HexUtilities::FromHex (non-hex characters count as 0); "default" is true only for
  exactly "Y". A field with no digits makes Mesen refuse the whole pack, and so does the
  converter. Pack versions above 109 and scales outside 1-10 are refused, as in Mesen.
- Brightness is Mesen's integer B = int(float32(value) * 255), wrapped through a byte for
  versions 1-104, 255 for version 0; each channel becomes min(255, (B * (c + 1)) >> 8),
  applied to every source pixel before resampling.
- Any scale is area-resampled to 2x: each output pixel averages the source pixels under it,
  weighted by overlap; it is opaque when half its area is (alpha >= 128), and its colour
  averages only the opaque pixels. 1x and 2x are exact; 4x is a 2x2 box.
- Duplicate keys: Mesen keeps every definition and draws the first whose conditions pass,
  so for unconditional tiles the first one wins, as here.

Known gap until conditions are supported (stage 3): a key defined only by conditional tiles
draws the ORIGINAL NES tile in Mesen when its conditions fail; it does not fall back to the
default tile. The converter currently skips conditional tiles, so such a key falls back to
the default tile when one exists. No test pack has been checked for this case yet.

## Full compatibility (goal set 2026-10-13: every real pack we can find must work)

Reference: Mesen2 Core/NES/HdPacks (HdPackLoader.cpp, HdData.h, HdNesPpu.h, HdNesPack.cpp).
Survey tool: tools/hdpack_survey.py. Corpus: 28 packs (~24 distinct games) from the 19-pack
"Mesen roms & packs" archive.org bundle, LiQuiDz's GitHub, retrogamingbanter.com's list and the
romhacking.net threads it links (Ninja Gaiden 1+2, Metroid HD, Metroid Brazilian v108, Zelda
Remastered, Zelda II Revamped, Mega Man HD/Super, Castlevania, Shatterhand, Twin Bee, Battle
City, Bomberman, Little Nemo, DK, DK Jr, Ice Climber, Pac-Man, SMB HD, Paper Mario, Duck Hunt,
Kung Fu, Nuts & Milk, Rogue Dawn, Bowser's Castle).

Feature use across the 28: scale 1 x12, 2 x9, 4 x7; CHR-RAM x6; conditional tiles x14
(tileAtPosition 17, frameRange 12, memoryCheckConstant 8, ppuMemoryCheckConstant 7,
spriteAtPosition 9, spriteNearby 9, tileNearby 9, memoryCheck 2, originPositionCheckY 1);
backgrounds x16; patch x9; bgm x8; sfx x4; addition x2; fallback x2; overscan x2; options
disableSpriteLimit 5, disableOriginalTiles 3, automaticFallbackTiles 2, disableContours 6
(a no-op since v106). Pack versions 0, 2, 100-108.

Decisions (user, 2026-10-13):
- Pure FPGA. No ARM helper.
- 4x packs are box-filtered to 2x by the converter. 1x packs are stored at 1x and doubled
  by nearest neighbour (exact).
- Conditions are evaluated exactly as Mesen does, on the whole frame, so HD output is one
  frame behind the NES (only while an HD pack is on).

Memory: everything fits in the FPGA's ~512 MB DDR3 window once images are stored as unique
16x16 blocks plus a block map (lossless). Measured, raw RGBA at 2x -> blocks + map:
Zelda Remastered 2137 MB -> 50 MB, Metroid Brazilian 3923 MB -> 150 MB, Zelda II Revamped
8293 MB -> 70 MB, Mega Man Super 378 MB -> 20 MB. Audio is stored decoded (ADPCM).

Stages (each one: converter + RTL + sim against the pack PNGs, then the seed rule and
hardware; the major ones get the iterative audit, capped at round 5):
1. Converter-only: pack versions 0/2/100-108 parsing, scale normalisation, more lookup room.
2. Engine rework: hash table and block-mapped image store in DDR3, engine registers moved to
   M10K, legacy 24-bit palette keys (pre-100 packs), CHR-RAM shadow + per-tile fingerprint.
3. Frame-delayed renderer + all conditions (global per frame, positional, nearby, mirror).
4. Backgrounds (40 priority levels, scroll ratios, offsets, alpha, brightness), additions,
   fallbacks, options, overscan.
5. ROM patches (IPS applied in SDRAM at pack load; BPS resolved by the converter).
6. Audio: $4100-$4106 register API, BGM + SFX mixing from decoded tracks.

## Target architecture for stages 2-6 (written 2026-10-15)

Source for every behaviour below: MESEN_HD_RUNTIME_SPEC.md (Mesen2 Core/NES/HdPacks, with
file:line citations). The MVP line pipeline cannot be extended to full compatibility, because
Mesen renders a frame only after it is complete, from a record of that whole frame:
tileNearby/spriteNearby read pixels the PPU has not drawn yet, tileAtPosition/spriteAtPosition
can name any pixel, and memory conditions read a snapshot taken at scanline 240 dot 0. So the
engine is rebuilt around a frame record, and every stage below targets this one design rather
than patching the line pipeline.

### Clocks and bandwidth

The HD engine moves to its own clock, 4 x the core clock (85.9 MHz), which also clocks the
DDR3 port. The tap crosses into it through a FIFO, the savestate channel through the existing
request/ack handshake (synchronised), and the picture leaves on the core clock through the
line buffer, exactly as the MVP does. Budget per NES line at 85.9 MHz: 5456 cycles.

### 1. CAPTURE (frame N, while the NES draws it)

The tap writes a compact record of frame N into DDR3 (double-buffered, frames A/B):
- per line: line header {t register and fine X at dot 1, grayscale, emphasis, palette RAM
  snapshot (32 x 6 bits), sprite count}
- per line: BG span table, one entry per 8-pixel fetch group (up to 34): {TileIndex 22 bits,
  OffsetY 3, palette select 2}, plus, for CHR-RAM games, a 32-bit fingerprint of the 16 tile
  bytes (see stage 2)
- per line: sprite table, 8 entries (up to 64 with disableSpriteLimit): {TileIndex, OffsetY,
  attributes, X, the two pattern bytes}
- per pixel (256 per line, 2 bytes): {BG span slot 6, BG colour index 2, fine column 3,
  backdrop-forced flag, sprites-enabled flag}
About 700 bytes per line, 170 KB per frame, about 10 MB/s of writes at 60 Hz.
Palette per line instead of per pixel is exact on real hardware: palette RAM cannot be
written while rendering is on, and during forced blanking no tile is drawn (Mesen branch B).
At scanline 240 dot 0 the watched-address snapshot (up to 256 CPU/PPU bytes, list from the
pack) and the frame counter are written into the same record.

### 2. RESOLVE (start of frame N+1's display, i.e. vblank + early lines)

Per-frame work, done once, in the same order as Mesen's OnBeforeApplyFilter:
- per-frame conditions (memoryCheck*, frameRange, tileAtPosition, spriteAtPosition)
  evaluated into a condition-result bitmask (up to 1024 conditions)
- background selection: first passing background per priority level (40 levels)
- additions inserted into the frame record
Results live in M10K for the renderer.

### 3. RENDER (frame N, line by line, during frame N+1's display)

For each NES line L of frame N, while the NES draws line L of frame N+1:
- LOOKUP: for each BG span and each sprite: hash the key, read the table bucket from DDR3,
  walk the definition list, evaluate each definition's conditions (bitmask for per-frame
  ones; tileNearby/spriteNearby/position checks from the frame record, fetching the
  neighbour lines' records), take the first that passes. Exact key missing -> fallback ->
  default key, exactly as GetMatchingTile.
- FETCH: burst the needed rows of each chosen tile image and each active background's
  two source rows into on-chip row caches.
- COMPOSITE: build each of the two 512-pixel output lines in a line buffer by painting the
  layers in Mesen's order (backdrop, backgrounds 0-9, behind sprites, backgrounds 10-19,
  BG tile, backgrounds 20-29, front sprites, backgrounds 30-39), with full alpha blending
  (Alpha/Add/Subtract) and brightness, then grayscale and emphasis per line.
The picture is one frame behind the NES while an HD pack is on.

### Pack store in DDR3 (format 3) - see "Format 3" below for the byte layout

- key table: 64-byte buckets of 4 entries, CRC-32 of the key picks the bucket, linear probing
  over buckets; entry = {key, rule list pointer}
- rule list: every `<tile>` defined for that key, in file order: {image block, frame-condition
  list, walk-condition list, flags}
- condition table: compiled conditions, deduplicated, 32 bytes each
- image store: unique 16x16 premultiplied RGBA blocks (a 2x tile is exactly one block), plus
  block maps for backgrounds; brightness is baked in
- backgrounds: 40 priority levels, each a list of {block map, size, offsets, scroll-ratio
  tables, blend mode} in file order, with a value index over their memory conditions
- watched-address list, fallback map, additions, palette, options, overscan; patch and audio
  sections are reserved until stages 5 and 6

## Format 3 (written 2026-10-15)

Built by `tools/hdp3_convert.py`, which parses with `tools/hdpack_model.py` (the line-for-line
port of Mesen2's HdPackLoader, checked against all 120 corpus archives). The MVP format 2 and
`hdp_convert.py` stay for the existing sims until the stage-2 RTL replaces them.

### Corpus measurements that set the design

Measured over the 110 loadable packs (`scratchpad pack_stats.py / rule_shape.py /
pixel_conds.py`):

    keys                                          44,626   Ninja Gaiden 2
    <tile> lines                                 150,801   Metroid HD 2.1.2
    rules for one key                              5,495   Metroid HD (JUD6MENT)
    distinct conditions used                      60,697   Castlevania Rondo (2026)
    watched addresses                                130
    backgrounds in one priority level             20,489   Zelda II (ZII-mesen)
    memory signatures in one background level         30   Zelda II
    backgrounds left after indexing on them          256   Castlevania Rondo
    per-pixel rules before an unconditional one      360   Metroid HD (one key, one memory state)
    tileNearby/spriteNearby offsets       x -176..176, y -224..224   1943, Donkey Kong

Two consequences:
- A linear scan of a background level does not fit the vblank (20k entries), but indexing a
  level on the values its `memoryCheckConstant ==` conditions test leaves at most 256
  candidates. Format 3 stores that index for backgrounds.
- Nearby offsets span the whole screen, so the capture keeps the whole frame's tile and sprite
  records, not a window of lines.

### Rule selection at run time (what the format is shaped for)

Mesen picks, for a record's key, the FIRST rule in file order whose conditions all pass
(spec 3.16). Conditions fall in two classes, and each rule stores them as two lists:

- frame class: memoryCheck, memoryCheckConstant, frameRange, tileAtPosition,
  spriteAtPosition (and their '!' twins). Same result everywhere in the frame; evaluated at
  most once per frame and memoised (2 bits per condition: done, value), as Mesen caches them.
- walk class: hmirror, vmirror, bgpriority, tileNearby, spriteNearby, positionCheckX/Y,
  originPositionCheckX/Y. Depend on the record or the pixel.

Lookup of a key in a frame: on its first use, the engine filters its rule list by the frame
class, keeping the rules that pass, in order, and stopping after the first one whose walk
list is empty (it always matches, so nothing after it can be reached). That short list is
kept for the rest of the frame; each record then walks it with the walk class. Since all
conditions are pure, evaluating them in a different order or once per frame gives Mesen's
result exactly.

The BG 8-pixel cache needs one more fact: Mesen disables the cache if any rule it EXAMINED up
to the match carries ForceDisableCache (spec 3.17), including rules that failed frame
conditions. Each rule therefore stores `prefix_fdc` = OR of ForceDisableCache over rules 0..i
of its list; the engine reads it at the matched rule (or the last rule if none matched).

Backgrounds (spec 4.2) use the same rule shape with only frame conditions. A level with more
than 32 entries carries a value index: each distinct set of (address, mask) pairs tested by
`memoryCheckConstant ==` conditions (a "signature") has a hash table from the snapshot values
to the ascending list of entries with that signature and those values. The first passing
entry of the level is the smallest entry index that passes among all signatures' candidates;
an entry's frame list then holds only its remaining conditions. Entries with no such
conditions form the empty signature, whose list is always a candidate.

### Byte layout

Little-endian. Every section starts on a 64-byte boundary. A pointer is a byte offset inside
the section it points into (the header gives each section's offset in the file); offset 0 of
the condition-list, map and index sections is never used, so 0 can mean "none".

Header, 512 bytes:

    0x00  char[4]  "NHDP"
    0x04  u16      format version = 3
    0x06  u8       pack scale (1-10)
    0x07  u8       flags: bit 0 legacy key (pack version < 100), 1 palette present,
                   2 has CHR-RAM keys, 3 has CHR-ROM keys, 4 game is CHR-RAM,
                   5 game CHR type known (from the ROM, or every key is one kind)
    0x08  u32      pack version
    0x0C  u32      Mesen option bits (spec 6.3)
    0x10  i32[4]   overscan top, right, bottom, left; -1 = not set
    0x20  u32      file size
    0x24  u32      key table bucket count (power of 2)
    0x28  u32      key table max probe (buckets)
    0x2C  u32      condition count
    0x30  u32      frame-class condition count (memo size)
    0x34  u32      watched address count (<= 256)
    0x38  u32      longest rule list
    0x3C  u32      block count
    0x40  u32      addition count
    0x44  u32      fallback map length (CHR tiles)
    0x48  u32      ratio table count
    0x80  16 x {u32 offset, u32 size}  sections: 0 keys, 1 rules, 2 conditions,
                   3 condition lists, 4 blocks, 5 bg levels, 6 bg records, 7 bg maps,
                   8 ratio tables, 9 watched, 10 fallback map, 11 additions, 12 palette,
                   13 bg index, 14 patches (reserved), 15 audio (reserved)

Key table entry, 16 bytes (4 per 64-byte bucket, empty entry = all zero):

    q0  [31:0] palette (PaletteColors; 0xFFFFFFFF = default key)
        [63:32] body low:  CHR-ROM: tile index (i32)  CHR-RAM: fingerprint[31:0]
    q1  [31:0] body high: CHR-ROM: 0                  CHR-RAM: fingerprint[63:32]
        [61:32] rule list pointer / 8   [62] kind (1 = CHR-RAM)   [63] valid

Bucket = CRC-32 (poly 0x04C11DB7, init 0, MSB first, no final XOR) over the 12 bytes
palette(LE) + body(8, LE), masked to the bucket count; linear probing over buckets. Buckets
>= 2 x keys / 4 (at most half full).

CHR-RAM fingerprint: CRC-64 (poly 0x42F0E1EBA9EA3693, init 0, MSB first, no final XOR) over
the 16 tile bytes in order. The converter refuses a pack in which two different tile data
(in keys, conditions or additions) share a fingerprint. For CHR-ROM games Mesen never fills
TileData (spec 1.3), so the engine uses the fingerprint of 16 zero bytes for their records.

Rule list: u32 count, u32 0, then `count` rules of 16 bytes:

    w0 payload: tiles - block index (0xFFFFFFFF = every byte zero, nothing to draw; a block
                with zero alpha but non-zero colour is kept, because brightness above 255
                lifts premultiplied colour off 0 and Mesen adds it in);
                backgrounds - background record index
    w1 frame condition list pointer (0 = none)
    w2 walk condition list pointer (0 = none)
    w3 [23:0] source index (tile line / background number, for debugging)
       [24] force_disable_cache  [25] prefix_fdc

Condition list: u32 words `[23:0] condition index, [31] last`.

Condition, 32 bytes:

    q0  [7:0] type  [8] negate  [9] ignore palette  [10] tile given as data
        [11] frame class  [15:12] operator (0 ==, 1 !=, 2 >, 3 <, 4 <=, 5 >=)
        [23:16] mask   [24] rewrite-sensitive (tile conditions: a fallback rewrite of a
        looked-up record can change the result, so keys using it are walked in lookup
        order; set by the converter, re-derived by brute force in hdp3_read)
        [31:25] 0   [63:32] memo index (frame class)
    q1  memoryCheck: [7:0] slot A, [15:8] slot B
        memoryCheckConstant: [7:0] slot A, [15:8] value
        frameRange: [15:0] A, [31:16] B
        position checks: [31:0] operand
        tile conditions: [31:0] X (i32), [63:32] Y (i32)
    q2  tile conditions: tile index (i32, sign-extended) or fingerprint
    q3  tile conditions: [31:0] palette, [63:32] Y*256+X (tileAt/spriteAt)

Types: 0 memoryCheck, 1 memoryCheckConstant, 2 frameRange, 3 tileAtPosition, 4
spriteAtPosition, 5 tileNearby, 6 spriteNearby, 7 positionCheckX, 8 positionCheckY, 9
originPositionCheckX, 10 originPositionCheckY, 11 hmirror, 12 vmirror, 13 bgpriority.
Identical conditions (same type, negation and operands) share one entry. Slots index the
watched-address list (u32 each, bit 31 = PPU address, as Mesen stores them).

Blocks: 16x16 pixels, row-major, 4 bytes each R, G, B, A, colour premultiplied by alpha
exactly as Mesen's PNG loader does (`c = ((a+1)*c) >> 8` when a < 255) and with the rule's
brightness applied (`min(255, (B*(c+1)) >> 8)`, alpha unchanged). Scale 1: each pixel
doubled. Scale 2: copied. Other scales: area-resampled premultiplied RGBA, each output
channel the overlap-weighted mean rounded half up (approximate, only for scales other than 1
and 2). Identical blocks are stored once.

Backgrounds: level table = 40 x {u32 rule list pointer (0 = none), u32 index pointer
(0 = none)}. Background record, 32 bytes: u32 map pointer, u32 width, u32 height (NES
pixels, `floor(png size / scale)`; 0 when the PNG does not decode, so it is never drawn),
u32 left, u32 top, u16 h-ratio table, u16 v-ratio table, u8 blend (0 Alpha, 1 Add,
2 Subtract), pad. Map: u32 block index per 8x8 NES-pixel cell, row-major,
`ceil(width/8)` x `ceil(height/8)`, 0xFFFFFFFF = transparent cell; pixels past width/height in
the last cells are transparent. A map exists per distinct (file, brightness). Ratio tables:
512 x i32, entry s = `(int)((float)s * ratio)` in float32 as Mesen computes it, so scroll is
exact.

Background index (one per indexed level): u32 signature count, then 48 bytes per signature
{u8 term count, u8 0, 16 x (u8 watched slot, u8 mask), u32 table pointer, u32 bucket count,
u32 max probe}. Table entries, 32 bytes: {u8[16] values (zero past the term count), u32
entry list pointer, u32 count, u32 valid, u32 0}, bucket = CRC-32 of the 16 value bytes,
linear probing, an invalid entry ends the probe. Each table starts on a 32-byte boundary. Entry lists: ascending u32 entry indices into the level's rule list.

Both the fallback map and the additions depend on whether the GAME has CHR-RAM or CHR-ROM, as
in Mesen (HdNesPack.cpp): the converter takes it from the ROM (`--rom`), else from the pack
when every tile and addition key is one kind, and refuses a pack that needs it but mixes kinds.

Additions, 64 bytes each, file order, only those whose added tile kind equals Mesen's
`HasChrRam()` for the patched ROM (CleanupInvalidRules deletes the rest; this is NOT always the
game's CHR kind, see "FOUND AND FIXED" under the stage-3 reference renderer notes): original key {palette, kind, body}, added key
{palette, kind, body}, offset X, offset Y (i32), flags (ignore palette). Fallback map: i32 per
CHR tile (-1 = none), explicit `<fallback>` lines over the automatic ones (spec 6.2), empty for
a CHR-RAM game (InitializeFallbackTiles builds nothing then, explicit lines included). The
automatic part of a CHR-ROM game needs the ROM; without it the converter refuses. Palette: 64 x u32 0x00RRGGBB when palette.dat is valid.

### Stage plan

2. Format 3 converter (bucketed DDR3 table, definition lists, block store, compiled
   conditions, CHR-RAM fingerprint keys); CAPTURE writer; 85.9 MHz engine clock; DDR3 lookup
   path. Test: the current 13-pack sim set plus Ninja Gaiden 1/2, Shatterhand, Metroid HD.
3. RESOLVE + RENDER with all conditions; 1-frame delay. Test: sim against Mesen-rendered
   reference frames for every conditional pack in the corpus.
4. Backgrounds, full alpha blending, brightness, additions, fallbacks, options, overscan,
   grayscale/emphasis.
5. ROM patches applied to the ROM in SDRAM at pack load (BPS resolved by the converter).
   Measured 2026-10-15: a patch can change the cart TYPE, not just bytes. Metroid's
   `mmm.ips` makes a 128 KB PRG CHR-RAM cart into 256 KB PRG + 128 KB CHR-ROM, and the pack is
   keyed for the patched cart. So the patch must be applied before the core picks its mapper
   configuration, and the pack's header bits 4-5 (game CHR kind) describe the PATCHED ROM. The
   SHA1 match is on the whole file including the 16 header bytes, so authors list one
   `<patch>` line per header variant (Metroid lists five, including the No-Intro NES 2.0 one).
   For a pack listing only an iNES 1.0 hash, the converter also tries the header with bytes
   7-15 zeroed and says so. Decided 2026-10-22 (user: full compatibility for all 136): the
   core does the same, a step beyond Mesen - see "Beyond Mesen" in the stage-5 section.
   `tools/rompatch.py` (IPS/BPS/UPS,
   CRC-checked; `tools/test_rompatch.py` 13 checks, negative-controlled) is the reference for
   whatever applies the patch on the MiSTer.

   Variant packs: a third of the corpus's refusals were add-on folders holding only a
   `hires.txt` (music options, alternate maps, hack add-ons) that are installed by copying them
   over a base pack. The converter and checker take `--base DIR` (repeatable) and lay the
   variant over its base, variant files winning. DECIDED (user, 2026-10-15): each variant
   stays its own `.hdp` file; there are no in-pack options. Recipes for every corpus pack
   that needs `--base` or `--rom` are in `NES_HDPack_testdata/pack_recipes.tsv`. SquidPac is
   one of them: a reskin of the 125scratch Pac-Man pack whose one missing image is
   byte-identical in its base, so it is built over that base (133 of 133 corpus packs pass).
6. Audio: $4100-$4106 (and the alternate range), BGM/SFX playback from decoded tracks in
   DDR3, mixed with the APU.

Reference frames for sim: Mesen2 itself (a headless build that dumps frames for a given ROM,
pack and input), so every stage is checked against the real renderer, not against our own
reading of it.

## Stage 2 RTL (started 2026-10-15)

### DDR3 memory map

The core addresses DDR3 through hd_ddr as qword offsets [27:3] inside the window at
0x30000000 (256 MB). Everything stage 2 adds lives in that window:

    0x30000000 - 0x3BFFFFFF   pack store, 192 MB: the .hdp file verbatim, byte 0 at the base,
                              so every header section offset is also a DDR3 offset
    0x3C000000 - 0x3DFFFFFF   frame records A/B and engine scratch, 32 MB
    0x3E000000 - 0x3E7FFFFF   savestates (statemanager: 4 slots x 2 MB at dword 0x3800000),
                              unchanged
    0x20000000 - 0x21FFFFFF   framework scaler (ascal, RAMBASE 0x20000000, 8 MB x 3 when
                              triple-buffered; Main_MiSTer reserves 32 MB for it, video.cpp
                              "512mb + 32mb(Core's fb)"), never touched
    0x22000000 - 0x237BAFFF   Main_MiSTer's Linux framebuffer (video.cpp FB_ADDR, 1920x1080
                              x 4 bytes x 3), never touched
    0x24000000 - 0x2FFFFFFF   pack store overflow, 192 MB (stage 6, see "Stage 6 - audio")

CORRECTED 2026-10-21: this table used to say the scaler takes only 0x20000000-0x207FFFFF and
that 0x20800000-0x2FFFFFFF was free. Read from Main_MiSTer's source (commit 0c98d24): the
scaler region is 32 MB and the ARM framebuffer sits at 0x22000000, so the free part of the
lower window starts at 0x24000000.

The largest corpus pack is 157 MB (Contra80s), so 192 MB has room for images. Audio is the
last section of the file and may run past 192 MB; those bytes go to the overflow region.

Measured 2026-10-21 with audio included (scratchpad pack_fit.py: image-only size from the
last full conversion, out/convert_all2.tsv, plus each pack's distinct audio files at 48 kHz
IMA ADPCM, one byte per stereo frame): all 133 converted packs fit both loader limits, 40 of
them carry audio. Largest total p071 (Dragon Quest 3 Delocalized, source a064) 284.8 MB of
384 (20.8 images + 263.9 audio), then a034 265.3 MB; largest image section p069 149.8 MB of
192. Headroom at the largest: 99 MB. The estimate was checked against the real a034 file
with audio (279,324,224 bytes): 0.4 % under, 1.1 MB, small against the headroom.
None of this is NES cartridge space: the game's ROM is unchanged and the HD content lives in
DDR3, so the limits are these windows, not the NES's.

### hd3_loader (rtl/hdpack/hd3_loader.sv)

Streams the OSD file load into the pack store, one qword write per 8 bytes, holding the
transfer (load_wait) while a write is pending. It counts bytes itself: hps_io's ioctl_addr
is 27 bits (128 MB), smaller than the largest pack. Only the 13 header words the engine
uses are kept in registers, and each section-table entry is checked as its 8 bytes arrive
(the file-size word at 0x20 is complete before the table at 0x80). At the end of the load
the pack is accepted only if the magic, version 3, scale 1-10, flag bits, header file size =
bytes received, size <= 192 MB, power-of-two bucket count, the whole 256-byte table received,
and every section (inside the file, qword aligned) check out; otherwise `reject` holds a
reason code (1-10) and the engine stays off. A byte arriving while a write is still pending
(a broken handshake) is refused too (code 9) rather than accepted with a lost qword.

Size (yosys synth_intel_alm estimate, not Quartus): the first version kept all 64 header
words and checked the 16 sections in one combinational block, about 2,270 LUTs and 1,519
registers; the streaming version is about 774 LUTs and 610 registers (2026-10-15). It matters
because the last Quartus fit (18:24, before the block-RAM hd_chrfp change) failed at 102% of
ALMs, and the loader is not yet instantiated in NES.sv. Estimator: scratchpad yosys_area.sh.

Test: tools/sim3/tb_loader.cpp streams real packs with random gaps and a randomly busy write
port and checks the DDR3 image byte for byte, the header outputs, and 18 corrupted variants
(each refused with its own code; two added for the streaming check: the LAST table entry past
the end, and a file that ends inside the table with its size field agreeing), then that an
intact reload is accepted again. Passed: SquidPac, Bomberman, Mega Man Super add-on (17.6 MB),
Contra80s (157 MB, intact load); after the streaming rewrite, 109 checks 0 failures on p000,
p001, p053 and a057 (Zelda II, 125 watched addresses). tools/sim3/neg_loader.py: 7 mutants,
two of them for the streaming check; 7 of 7 caught on p013 + p080 (Kung Fu Guy, Road
Fighter: 3 and 23 watched addresses). On p001 + p053, which have NO watched list, only 4 of
7 were caught: the list mutants cannot show without a list. neg_loader.py now refuses a
fixture with no watched list rather than report those as passes.

Exports for hd3_dict / hd3_resolve (2026-10-16): `fb_base` (section 10, header 0xD0) and
`fb_len` (header 0x44) - two more header words kept; `legacy` = flags bit 0; and `game_ram` =
flags bit 4 when bit 5 says the converter knew the game's CHR kind, else the loaded cart's
(new input `cart_no_chr_rom`, NES.sv's `chrrom == 0`). That fallback is Mesen's own rule,
!HasChrRom, and must NOT be `mapper_flags[15]` (has_chr_ram): an NES 2.0 cart can have both
CHR-ROM and CHR-RAM (Metroid + mmm.ips) and keys by tile index. 39 of the 137 corpus files
leave bit 5 clear (the 32 audio/patch-only packs below, p072, p100 and the 4 test packs), so
the cart decides for them; every pack with tiles and a known kind carries bit 5.
tb_loader checks all four outputs on every file, game_ram with the cart input both ways:
all 137 corpus files, 1,781 checks, 0 failures (98 decided by the pack, 39 by the cart).
neg_loader.py gained 5 mutants (cart ignored, pack ignored, legacy from bit 1, fallback base
from section 9, fallback length from header 0x40): 12 of 12 caught on two small fixtures,
tools/sim3/make_loader_fixture.py's copy of p072 with a 37-entry fallback map appended and
flags 0x29 (legacy, kind known) plus p072 itself (kind not known; 4 watched addresses). Only
60-75 MB packs carry a real fallback map, and the control streams every file once per mutant
and variant. Two tb_loader variants assumed a larger file and were fixed on the way: the
power-of-two mutation added 1 to the bucket count, which turns 1 (the smallest packs) into a
power of two; and the broken-handshake byte sat at index 4104, past the end of a 1,216-byte
file, so it was never sent. Both now pass on every size. The loader is not instantiated in
NES.sv yet (the MVP loader is), so the outputs are wired only in the testbench until stage 3
replaces the MVP.

32 corpus files are 1,152 bytes: no keys, no rules. Not a converter defect: none of their
hires.txt has a `<tile>` line. 31 are audio packs (`<bgm>`/`<sfx>`, stage 6), some with a
`<patch>` (stage 5); p099 (Life Force US) has only options and a `<patch>` whose hash is an
MD5 (32 hex digits) where Mesen requires a SHA1, so Mesen rejects that line too. In 19 of them
some or all OGG files are not in the archive (folders recursed, scratchpad empty_packs.py):
PeV's "Nintendo Enhanced Audio" packs ship hires.txt and an IPS only, and their PleaseRead
says the player supplies "your custom OGG music files"; p085 lacks 1 of its 16 (scratchpad
ogg_missing.py counts from hires.txt). Stage 6 has to take music the user adds to the pack
folder.

### hd3_lookup (rtl/hdpack/hd3_lookup.sv)

One key per request: CRC-32 of the key picks the bucket; each bucket is one 8-qword DDR3
burst; the search stops at the key, at an empty entry, or after max_probe+1 buckets. Then it
streams the rule list in 8-qword bursts. Until stage 3's condition walk exists it reports
the first rule whose condition lists are both empty.

Test: tools/sim3/tb_lookup.cpp queries every key in a pack's table, the default key of each
tile, and mutated keys that must miss; tools/sim3/check_lookup.py checks each answer
(presence, list offset, rule count, picked rule, and the number of buckets read) against
tools/hdp3_read.py. 0 mismatches on Ninja Gaiden II (111,827 queries), Contra80s (deepest
probes, CHR-RAM), Mega Man Super (CHR-RAM), Metroid HD a034 (5,495-rule list), Castlevania
and SquidPac, each with DDR3 latency 12/no busy and latency 40/30% busy. Negative control
(tools/sim3/neg_lookup.py): 6 of 6 RTL mutants caught (kind not compared, probe limit one
short, search not stopping at an empty entry, hash byte order reversed, pick ignoring the
walk list, rule count off by one). The empty-entry mutant gives the same answers on a
well-formed table; only the bucket count exposes it, which is why the check counts buckets.

Measured cost, one request at a time: about 33 clocks mean at latency 12, about 80 at
latency 40 with 30% busy. A line needs about 42 lookups, so the stage-3 renderer must keep
several in flight; the long-list case (Metroid's 5,495 rules, ~1,200 clocks mean when no rule
is unconditional) is what the per-frame filtered list in "Rule selection at run time" is for.

### hd_chrfp (rtl/hdpack/hd_chrfp.sv)

CHR-RAM games key tiles by their 16 data bytes, so the engine needs the fingerprint (CRC-64)
of any tile at any time. hd_chrfp keeps a 32 KB shadow of CHR RAM (fed by every SDRAM write
into the CHR region: PPU writes and savestate restores) and one fingerprint per tile,
recomputed in the background (18 clocks per tile) whenever a byte of the tile changes. A
tile's dirty bit is cleared before its recompute reads any byte, so a write during the
recompute causes another. Reset marks every tile dirty with a 2,048-clock sweep.

Dirty bits are in BLOCK RAM, one bit per tile (2026-10-15). The first version kept them in
registers (64 x 32-bit words, set and clear merged in one clock); a Quartus fit showed that
cost ~3,900 ALUTs and put the whole core at 102% of the chip. The RAM version shares one write
port: a core write (set) always wins; the scanner's clear and the reset sweep wait for a free
clock. A clear that lands after a set to the same tile is harmless (the byte is already in the
shadow and the recompute reads it afterwards); a clear that loses the port just costs a repeat
recompute. The scanner reads one bit every 2 clocks; a write on the exact clock a bit is read
is missed by that pass (the RAM returns the old bit), so `idle` requires NT+1 consecutive
clean tiles with no write since - one full pass that also re-reads the possibly-stale tile.

Test: tools/sim3/tb_chrfp.cpp checks every tile against a software CRC-64 after a 32 KB
restore written one byte per clock, 400 paced uploads, 3,000 bursts aimed at tiles being
recomputed, back-to-back writes across tile-group boundaries, and 60 writes aimed at the
exact clock the scanner reads that tile's dirty bit (phase 4, using public_flat_rd on rs /
scan_p / ph); and that `idle`, once reported, never drops without a write. Negative control
(tools/sim3/neg_chrfp.py): 6 of 6 mutants caught, including "idle after NT tiles instead of
NT+1", which the random phases alone never hit (5/6 before phase 4 was added). The first
version of the test missed an "idle reported too early" mutant: reading 2,048 fingerprints
takes long enough for the scanner to finish behind it, and the false idle lasted one clock.
The check now fails on any idle that drops without a write.

### Frame record (CAPTURE) - layout

Derived from MESEN_HD_RUNTIME_SPEC.md sections 1, 2, 4.4, 5.2 and 7. Everything the renderer
reads is captured; nothing it reads is reconstructed later from live state.

Per pixel, 4 bytes (256 per line):

    byte 0  [5:0] BG span slot of this pixel's tile (63 = NoTile: BG disabled or masked)
            [6]   sprites shown at this dot (PPUMASK sprites on and past the left mask); which
                  sprites cover the dot is derived by the renderer from the line's sprite table
    byte 1  [1:0] BgColorIndex   [4:2] OffsetX (column within the tile)
    byte 2  [5:0] PpuBackgroundColor (palette RAM entry 0 at this dot)
    byte 3  [5:0] BgColor (backdrop if index 0, else palette RAM[tile palette + index])

Mesen's "branch B" (forced blanking with v in $3F00-$3FFF, spec 1.2) writes only NoTile and
SpriteCount = 0; every other field keeps what that buffer held two frames ago. The record is
double-buffered the same way, so writing only byte 0 (DDR3 byte enables) for such a pixel
reproduces the stale fields exactly, including the (BUG?) stale backdrop.

Per line, a header written with the line:

    t register and fine X as of x = 0          (scroll, spec 4.4)
    grayscale and emphasis as of x = 0         (spec 5.2: applied per line from x = 0)
    the four fields above are not written when x = 0 is branch B (same stale rule)
    palette RAM, 32 x 6 bits                   (see below)
    BG span count, sprite count

Per line, the BG span table: one entry per tile fetch (up to 34):
{TileIndex 28 bits (offset in its memory region / 16; 0x0FFFFFFF when unmapped), OffsetY 3
bits (fine Y at fetch), attribute palette 2 bits, fingerprint 64 bits (CHR-RAM games)}.
Identity is taken from the pattern fetch address. Mesen uses the CHR mapping at dot 8n+1
(before the nametable fetch) rather than at the pattern fetches (8n+5, 8n+7); these differ only
if the mapping changes inside a fetch group, which MMC2/MMC4 never do (their latch switch is
triggered by the previous tile's fetch, before 8n+1 of the next).

Per line, the sprite table (8 entries; up to 64 with disableSpriteLimit): {TileIndex, OffsetY
within the 8x8 half, OAM attributes, X, the two pattern bytes, fingerprint}. The 4 sprites a
pixel records (spec 1.4: covering sprites in slot order, transparent ones included, at most 4)
are derived from this table and the pixel's byte-0 bit 6 by the renderer.

Palette per line is exact: palette RAM cannot be written while rendering is on, and when
rendering is off every pixel is NoTile with no sprites, so no key reads the palette. The line
snapshot is taken at the last dot of the line where rendering was enabled.

Per frame, after line 239: the frame number (spec 2.3: 1 for the first frame after reset) and
the watched-address values.

### hd_capture (rtl/hdpack/hd_capture.sv)

Builds the record from the PPU tap and writes it through a 256-entry FIFO as {qword address,
data, byte enables}. Frame N goes to buffer N & 1 (1 MB each, from 0x3C000000):

    0x00000 + y*1024 + x*4          pixels (4 bytes, as above)
    0x40000 + y*2048                line header: q0 = t, fine X, grayscale, emphasis (bytes 0-2,
                                    not written when x = 0 was branch B), x = 0 branch flag,
                                    span count, valid sprites, sprite fetches, y;
                                    q1-q3 = raw palette RAM, 32 x 6 bits
    0x40000 + y*2048 + 64 + s*16    BG span s: qA = CHR linear address of the pattern fetch +
                                    attribute palette, qB = fingerprint (CHR-RAM) or 0
    0x40000 + y*2048 + 640 + i*16   sprite fetch i (up to 64): qA = {valid, attr, x, CHR address,
                                    pattern bytes}, qB = fingerprint or 0
    0xF0000                         trailer: frame number

Fetches for line L happen at the end of line L-1 (sprites 257-320, BG slots 0-1 at 321-336),
so the module tracks the line a fetch belongs to; the pre-render line's fetches go to line 0.
The span fetched at dot 256 lands on the same clock as line_done and is counted in that
line's header. A producer finding its slot still occupied sets a sticky `overflow`.

Test: tools/sim3/tb_capture.cpp drives a PPU-shaped stream (262 lines, normal / rendering-off /
branch-B / mixed lines, palette writes, CHR-RAM changes in vblank, random DDR3 backpressure
with long busy windows) into hd_capture + a real hd_chrfp, and compares the DDR3 image
against a software model of this layout after every frame: 8 frames x 4 seeds, plus readiness
0.3, all byte-identical, no overflow. A starvation phase (the FIFO never drains) must set
`overflow`. Negative control tools/sim3/neg_capture.py: 8 mutants.

The starvation phase found a real defect: the FIFO's full test compared `fwp - frp` against an
integer, so the subtraction ran in 32 bits, went negative once the write pointer wrapped, and
the FIFO was never seen full, overwriting unread entries under sustained backpressure. Normal
readiness never reached the condition, so only the forced-starvation control exposed it. The
count is now taken at the pointers' own width. (The other FIFOs in rtl/hdpack compare 3-bit
pointers against 3-bit constants and are not affected.)

Extra sprites (the core's 16-per-line OSD option) - DONE (2026-10-16). Mesen's equivalent is
RemoveSpriteLimit; spec 1.4 lets a pixel name sprites past the 8th, so the record must carry
them. SpriteAddressGenEx (rtl/ppu.sv) fetches the extra list in step with the main one, and
the tap now takes its strobes too (hd_tap ports spr_load_ex / spr_load_in_ex / spr_ex_dummy,
`is_dummy` added to SpriteAddressGenEx). hd_tap_bus grew to 416 bits: [352] extra-entry write,
[355:353] its index, [410:356] its data, in the same 55-bit format as the main entry; bits
351:0 are unchanged. hd_stage2 muxes the two onto the capture's one sprite port, extras at
entries 8-15, so the record layout did not change (it always had room for 64).

The extra entry is written 4 dots after its slot's main entry: both groups strobe their high
pattern byte on the same dot, main entries are 8 dots apart, so writes stay 4 apart and the
capture's entry slot never waits.

Core defect found and handled in the tap: the extra list repeats main sprite 7, and the
16th sprite is never loaded. ppu.sv's start search ("hunt for the 8th sprite") counts 7 in-range
sprites with a 3-bit counter and the extra list begins at the 8th - which is already main
sprite 7. (Upstream MiSTer NES has the same code; checked 2026-10-16.) In the SD picture the
copy never shows: main 7 wins wherever either is opaque. An HD render would draw that sprite's
replacement twice, double-blend its translucent pixels and use one of the 4 per-pixel sprite
slots, none of which Mesen does. So extra entry 0 is held until main entry 7 is fetched and is
recorded NOT VALID when it is identical to it (written at dot 326, between extra entry 7 at 324
and the first BG span at 328). Comparing instead of always dropping keeps a real 9th sprite if
the search is ever fixed. The core's evaluation is left as it is: changing ppu.sv changes which
sprites the SD picture shows with the option on.
Remaining difference from Mesen: the 16th sprite on a line is not drawn (Mesen draws 16). That
is the core's evaluation, seen by both the SD and the HD picture alike.

8h - a pack's disableSpriteLimit (2026-10-17). Mesen forces RemoveSpriteLimit and
AdaptiveSpriteLimit for such a pack (MESEN_HD_RUNTIME_SPEC.md 7.1), in the SD picture too; 37 of
the 137 corpus packs set it, Metroid, Zelda II and Castlevania among them. The PPU gained
hd_nolimit (nes.v -> ppu.sv): extras on whatever the OSD option says, and the masking decision
taken by Mesen's rule instead of the core's heuristic. The two rules disagree: the core masks
when 7 consecutive main sprites repeat (y, tile, attributes); Mesen walks all 64 OAM entries,
keeps the ones in range of the line, and masks when 8 consecutive ones share (y, x). OAMEval
runs Mesen's walk over the idle dots 0-63 (one entry a dot, before evaluation; the result
mx_block is read by the extra fetches at 256-319, saved in savestate bit 61). The pre-render
line needs no case of its own: the core copies extras only while `evaluating` (visible lines),
so the pre-render's extra slots stay cleared - matching Mesen's scanline -1, in range of nothing.
An explicit pre-render block was written first, then removed when the negative control showed
it unobservable (row E sits in range of the pre-render line; line 0 records no valid extras
either way). With hd_nolimit low the core
is unchanged. Up to 15 sprites on a line the core loads exactly Mesen's set (main 1-8, extras
9-15; the copy of main 7 is recorded not valid); above 15 Mesen loads up to 64 and the core 15,
the documented limit above. Not modelled: OAM written between dot 63 and Mesen's dot 320 of the
same line, and a first visible sprite after entry 0 (Mesen's wrap-around scan), both only
possible with OAMADDR games outside the corpus. The loader already exports the header options
word (bit 0; set by 37 of the 137 converted packs, measured); NES.sv ties hd_nolimit low until
the integration (OPEN_ITEMS 10), which replaces the MVP hd_engine, drives it with options[0]
while the HD picture is enabled.
Test: test_exspr.py adds row D (line 80: 8 sprites at one position with different tiles, 2
more elsewhere) and two --nolimit runs (fullsim's flag for hd_nolimit, the OSD option off):
line 80 masked (8 + 0), line 180 (row C: one tile at 10 x positions) not masked (8 + 2), lines
40 / 120 unchanged; with the OSD option on, the opposite on both lines.

Test: tools/fullsim/test_exspr.py writes two NROM carts (tools/fullsim/make_exspr_rom.py:
line 40 = 16 sprites with every flip/priority, line 120 = 12, line 180 = 10 identical sprites,
the core's masked case; one cart 8x8, one 8x16), runs fullsim --rec with the option on (and the
8x8 cart with it off), and checks every frame with check_record.py and hdref.py --sd-check, plus
the per-line counts: line 40 = 16 fetches, 8 main + 7 extra valid; 120 = 8 + 4; 180 = 8 + 0;
option off = 8 fetches. Result: all frames pass (sd-check 61,440 / 0 bad each frame, sprite
identity 280 and 560 ok). check_record.py gained two checks for this: 7 sprite identity (lin
bit 3 clear, CHR bytes at lin and lin+8 equal the recorded pattern bytes, bit-reversed unless
attr bit 6) and 8 the copy rule (entry 8 must not be a valid copy of entry 7). The record of the
pre-fix build fails check 8 on every line with 16 sprites.
Negative control tools/fullsim/neg_exspr.py (rebuilds fullsim per mutant, restores the source
from a byte snapshot): extras dropped by hd_stage2 -> SD composite 495/990 bad; high-plane CHR
address kept -> check 7, 104/208 bad (invisible to the SD composite); copy rule removed ->
check 8, 16/32 bad (invisible to the SD composite). 3/3 caught.
Regression: rec_smb3 re-recorded with the new build (the option off) - rec, col and chr files
of all 5 frames byte-identical to the stored ones, so widening the bus changed nothing else.

The watched-address snapshot is built by hd_watch (see "hd_watch" below).

Emphasis bit order on PAL/Dendy - CHECKED (2026-10-16), no region handling needed anywhere
downstream. Mesen2 NesPpu::SetMaskRegister: NTSC takes IntensifyRed from 0x20 and
IntensifyGreen from 0x40; PAL and Dendy swap them ("the green and red bits swap meaning"),
and EmphasisBits is then bit0 red, bit1 green, bit2 blue (spec PPUH:94). The core already
normalises the same way at the $2001 write: rtl/ppu.sv `emph_reg <= pal_or_dendy ?
{ppu_dbus[7], ppu_dbus[5], ppu_dbus[6]} : ppu_dbus[7:5]`, so emph_reg is {B, G, R} on every
region, and the tap carries emph_reg itself (hd_tap_bus[342:340], via `.emph(emph_reg)`).
The renderer applies these three bits as R/G/B directly. Vs. System (sys_type 3) is NTSC in
both (pal_or_dendy false; Mesen's Vs region is NTSC).

### Integration (hd_stage2.sv)

One wrapper, rtl/hdpack/hd_stage2.sv, holds hd_chrfp + hd_capture and is instantiated the same
way in NES.sv and tools/fullsim/sim_top.sv, so the tap-to-module glue exists once.

- Clock: the core clock for now. Moving the record writer and the renderer to the 85.9 MHz
  HD clock happens in stage 3, when the engine itself is rebuilt; doing it here would add a
  clock crossing that stage 3 replaces anyway.
- Tap: hd_tap_bus grew to 352 bits. Bits 319:0 are unchanged (hd_engine still takes only
  those); [320] branch-B flag, [335:321] t, [338:336] fine X, [339] grayscale, [342:340]
  emphasis, [343] pre-render-line flag. (The wrapper first read these at 339:316 - lint cannot
  see a wrong slice; caught by comparing against ppu.sv's assign before the first run.)
  Later 416 bits: [352] extra-sprite entry write, [355:353] its index, [410:356] its data
  (see "Extra sprites" above); [415:411] unused, 0.
- CHR snoop: nes.v exports hd_chr_wr = a PPU write with the mapper's CHR write enable and not
  a nametable access. A region test is not enough: most mappers put CHR at 0x200000 in the
  22-bit PPU address, but CPROM uses 0x100000 and one MMC3 variant 0x3FF000. Savestate restores
  are snooped by region (0x0200000-0x02FFFFF in their 25-bit address), which covers every
  mapper except those two.
- DDR3: hd_ddr gained a fourth client, cw_valid/addr/data/be with cw_ready, single qwords with
  byte enables, lowest priority (savestates > pack load > MVP reads > record writes).
- Enable: the "HD Pack" OSD option (status[88]). Reset: reset_nes, so frame numbers restart.
- fullsim: --rec turns capture on and, for each saved frame, writes col_FFFF.bin (the PPU's
  colour index per cycle) and rec_FFFF.bin (the record buffer for that frame).
  tools/fullsim/check_record.py checks every visible pixel: BgColor against the PPU's output
  where no sprite is fetched, BgColorIndex against the CHR tile decoded from the span's address
  (CHR-ROM games), the span palette, and the line header.

### Watched-address snapshot - how the hardware gets it exactly

Mesen reads every watched address at scanline 240 dot 0, instantly and without side effects
(spec 2.2). Measured over the corpus: up to 130 watched addresses in one pack; CPU RAM in 64
packs, palette RAM in 41, nametables in 12, PRG-RAM in 5 (Metroid), PRG-ROM in 2, pattern
memory in 3 (Zelda II). Reading SDRAM at line 240 would race the CPU (the main loop keeps
running) and cost SDRAM slots the CPU and PPU need, so the engine keeps its own copies:

- CPU RAM: a 2 KB shadow fed by every SDRAM write to the CPU-RAM region (CPU writes,
  savestate restores). Always on from power-up, independent of any pack, so it equals SDRAM.
- PRG-RAM: an 8 KB shadow of the first 8 KB of the cart-RAM region, same feeding. Exact for
  unbanked PRG-RAM (Metroid's MMC1); banked PRG-RAM (MMC5, SXROM) is a known limit.
- Nametable RAM: four 1 KB shadow pages keyed by their SDRAM page (a 4-entry page CAM), fed
  by PPU writes and by savestate writes to a page already in the CAM (savestate writes never
  claim a slot). The two CIRAM pages are in the CAM from the start. The
  PPU-address-to-SDRAM-page mapping at line 240 is the one the last access to each 1 KB
  quadrant used, so mirroring changes and four-screen are exact.
- Palette RAM: read directly from the PPU at line 240 (mirrors $10/$14/$18/$1C -> $00/...).
- Pattern memory and PRG-ROM: read from SDRAM after the freeze (pass 2) through the mapping
  latched at dot 0: the bank the PPU last used for that 1 KB pattern page, the bank the CPU
  last used for that 2 KB PRG page. CHR-ROM and PRG-ROM do not change, so only the mapping
  matters. A CHR-RAM byte written between dot 0 and its pass-2 read would be seen (real games
  do not write CHR RAM there).
At line 240 dot 0 the shadows FREEZE: pass 1 reads every shadowed address at one entry per
clock while writes arriving meanwhile wait in 32-deep FIFOs (CPU-side and PPU-side) and are
applied afterwards, so every value is the one at dot 0, not a mixture. Values are
double-buffered: the snapshot that closes frame N goes to bank N[0], so the renderer reading
frame N-1's values is never disturbed.

Implementation: rtl/hdpack/hd_watch.sv, inside hd_stage2. Inputs from nes.v: cpumem_read/
write/addr/dout plus hd_cpu_a (the CPU address), ppumem_read/write/addr/dout plus hd_ppu_a (the
PPU address, the extra-sprite address when that fetch is active - the mappers use the same
mux). The list comes from hd3_loader, which copies section 9 out while streaming (lw_* port;
reject 10 for more than 256 entries or a list longer than the section). Until stage 3 replaces
the MVP loader with hd3_loader, NES.sv ties the list off and the snapshot does no work.

Pass 2 reads SDRAM through ch2 of rtl/sdram.sv (rtl/hdpack/hd_sdram_port.sv). ch2 belongs to
battery saves and savestates, so the port only takes it when no savestate is sleeping the
core, no battery copy is running and no save read/write is pending; while it owns the channel
the save path sees busy. Savestates wait for the core to pause plus 100 clocks before touching
SDRAM, and a port read takes a few clocks, so it always finishes first. The controller caches
the last word each channel read and answers a repeat read from that cache; that is only right
when one client owns a region, and these reads re-read words the CPU/PPU write, so the port
raises sdram.ch2_nocache (new input) and its reads neither use nor fill the cache.

Known limits (none occurs in the 133-pack corpus): banked PRG-RAM (MMC5, SXROM); CPU
$2000-$5FFF read 0 (Mesen PeekRam of registers; the HD audio registers do read 0); active Game
Genie cheats are not applied (Mesen applies them to CPU reads); nametables a mapper serves
itself instead of from SDRAM (MMC5 ExRAM/fill mode) read the SDRAM page.

Tests:
- tools/sim3/tb_watch.cpp: a software model of SDRAM, mappings, mirroring, four-screen pages,
  palette and savestate restores; 25 snapshots per seed with writes arriving during the freeze,
  both value banks; every value compared with the model at the snapshot clock. 3 seeds pass,
  no overflow. Negative control tools/sim3/neg_watch.py: 9 mutants, 9 caught (no freeze,
  writes bypassing the FIFO, FIFO drained while frozen, palette mirrors, nametable mapping read
  live instead of latched, savestate writes claiming slots, value bank ignored, PRG-RAM page
  never learned, registers read as RAM).
- tools/sim3/tb_loader.cpp checks the lw_* copy against each pack's section 9.
- tools/fullsim --watch LIST runs the whole core: at the snap pulse it dumps the model's
  CPU RAM, CIRAM, PRG-RAM page and palette, and tools/fullsim/check_watch.py compares every
  value hd_watch took (expected values are computed from those dumps and the ROM file, not from
  hd_watch's state). sdram_sim models ch2_busy and holds a decoy in ch2_dout until the read is
  done, so an arbiter sampling too early fails. Results (2026-10-15): SMB 300 frames (title +
  scrolling gameplay), 5 snapshots x 120 entries, every class incl. PRG-ROM and pattern reads
  through ch2: all match. Metroid (MMC1, PRG-RAM, CHR-RAM) 450 frames, 4 snapshots: 96/96 match
  per frame incl. PRG-RAM; its 18 nametable entries are skipped because MMC1 sets mirroring at
  run time. Negative control tools/fullsim/neg_check_watch.py: 11/11. The same runs also
  pass check_record.py (Metroid: the first CHR-RAM run, fingerprints checked against the CHR
  dump). tb_loader: all 137 corpus .hdp files, lists copied entry for entry; neg_loader.py 5/5.

## Verification

- PC reference first: the same pack in Mesen on PC.
- tools/make_test_pack.py builds a synthetic Mesen-format pack from the ROM's own CHR (2x
  upscaled, recoloured) so every replaced pixel is checkable.
- Converter round-trip check against Mesen's key rules; Quartus fit + timing after each stage;
  hardware test on the dev MiSTer.
- tools/sim/hd_sim.py checks every output pixel against the pack's PNGs. Scene "synthetic"
  is the fixed test scene; scene "pack" builds the scene from a real pack's keys (deepest
  probes first) and runs 64 lines. Duck Hunt HD passes with lookups at probe 0, 1 and 2.
  Negative controls: a version-1 header is refused, and repointing one probe-2 entry gives
  512 mismatches.
- The expected picture's resampling and brightness are written separately from the
  converter's. 13 real packs pass in the pack scene (SMB test, Duck Hunt, Pac-Man 4x,
  Kung Fu v0, Nuts & Milk v2, SMB HD v2, Ice Climber, Donkey Kong 4x, Bomberman 1x, Little
  Nemo 1x with a probe-7 lookup, DK Jr, Twin Bee, Paper Mario). Negative controls: nearest
  instead of box for 4x (7239 mismatches); legacy packs on a core without the flag (~38500);
  brightness rounded c*B/255 (6623); no byte wrap for version 104 (1881).
- Not yet runnable: Ninja Gaiden, Ninja Gaiden II, Shatterhand and Metroid HD have more keys
  than the 8192-entry on-chip table holds (stage 2).

### Stage-3 reference renderer vs Mesen 2.1.1 (tools/hdref.py, tools/mesen_cmp.py)

- Frame alignment: fullsim frame f = Mesen frame f+2 with MESEN_INPUT_OFFSET=-2 (SMB; recheck
  per game with `mesen_cmp.py align` against a no-pack Mesen shot, which must be 0 pixels off).
- SMB + p053 (a041 pack, 2x, 1729 conditions): frames 240 and 270 bit-exact; frame 300 differs
  in 12 of 245,760 pixels, all at NES x 93..97, y 31 (the coin icon at the status-bar split).
- Cause, measured, NOT a capture defect: SMB writes $2005 mid-line 31 after polling sprite 0.
  The record moves the fetch position by 2 at x=93; moving it to x=98 (scratchpad
  shift_test.py) gives 0 pixels off. Mesen fills HdPpuPixelInfo inside DrawPixel from the same
  _xScroll as the SD pixel (HdNesPpu.h, OffsetX = (_xScroll + ((_cycle-1)&7)) & 7), so in Mesen
  the SD and HD change together - the gap is WHEN the write lands. The BIT $2002 poll loop is
  7 CPU cycles = 21 dots, so the core's and Mesen's power-on CPU/PPU phase decide the dot, as
  two real consoles would. The SD picture cannot distinguish it here: every background pixel
  on that stretch is index 0 (checked; and the check's control was re-done after the first
  version cancelled the offset under test and passed by construction).
- Rule for comparisons: a difference confined to the dots right after a mid-line scroll write
  is phase, not a bug. Anything else is a bug.
- INPUT offset is per game, the frame offset is not: Mesen's endFrame event fires BEFORE the
  frame is sent to the decoder (NesPpu::SendFrame), so a screenshot at Mesen frame n shows an
  earlier decode; fullsim frame f = Mesen shot f+2 for both games tested (NOT a rule - see
  Castlevania below: the offset varies per frame, so align every frame). The input offset
  (MESEN_INPUT_OFFSET) depends on when the game polls the pad: SMB -2, Metroid -1. Find it
  with `mesen_cmp.py align` (no-pack SD must be 0 pixels off on every compared frame); with
  the wrong one Metroid diverged 8-10k pixels after the first held direction.
- Metroid HD (a034, 2x, CHR-ROM after mmm.ips, 66 watched addresses) with rec_mmm (input
  NES_HDPack_testdata/metroid_mmm_play.txt): frames 200, 400, 600, 800, 880, 920, 960, 1020,
  1100 all IDENTICAL to Mesen - backgrounds chosen by memory conditions, HD sprites, scrolling,
  ~40-72k condition evaluations per frame (mostly spriteNearby/hmirror).
- Castlevania p002 (CHR-RAM, 2026-10-15) DISPROVES a fixed frame offset: with input offset 0
  (cv_play.txt) the no-pack shot matching fullsim frame f is f+1 for 60/250/450/1000/1300 but
  f+2 for 700. So pair every frame separately: `mesen_cmp.py align` picks the SD shot (0 pixels
  off), and the HD shot with the SAME number is the target. Do not reuse one game's offset.
- FrameNumber phase: for the same game state Mesen's FrameNumber is one LOWER than ours (both
  number the first frame after reset 1 - NesPpu.cpp _frameCount = 1, stamped by
  HdNesPpu::OnBeforeSendFrame before the scanline-240 increment - so the game reaches that state
  one frame later on the core: power-on phase, as above). It moves every frameRange result by a
  frame. Render with hdref.py --frame-delta -1 (Castlevania) when comparing; the hardware is not
  changed, its counter follows the spec. With that, all six frames (60/250/450/700/1000/1300)
  are 0 pixels off outside the partial-alpha mask, and the hardware lookup model (hwmodel.py,
  plain and --dict) differs in 0 lookups on each. What is left is inside the mask: blend
  rounding of at most 6 levels (60/250/450) and the ~1018-pixel partial-alpha artifact
  (700/1000/1300). Script: scratchpad cv_frames.sh.
- Frame 250 took the longest because the earlier pairing (frame f vs shot f+2, recorded
  FrameNumber) happened to fit four frames while Simon changed pose between shots 251 and 252:
  every condition hypothesis (0530, spriteNearby, CHR-RAM keys) was a wrong turn. The tell was
  that the whole screen matched shot 252 except Simon, and Simon matched shot 251 exactly.
  Diagnose a region mismatch by checking NEIGHBOURING shots before the rule conditions.
- The earlier m899 input workaround (a hand-edited Castlevania input script) is not needed.
- FOUND AND FIXED in the converter: Mesen keeps an <addition> only when its added tile's kind
  equals mapper->HasChrRam() (HdNesPack::CleanupInvalidRules), NOT !HasChrRom(), which is what
  tile keys and fallbacks use. A cart can have both: mmm.ips gives Metroid an NES 2.0 header
  with 128 KB CHR-ROM and 8 KB CHR-RAM, so all 1987 index-keyed additions are dropped while the
  tiles stay index-keyed. HasChrRam (BaseMapper.cpp:524, iNesLoader.cpp:42, NesHeader.cpp:161):
  NES 2.0 -> byte 11 low nibble != 0 (no database lookup for NES 2.0); no CHR-ROM -> CHR-RAM;
  iNES 1.0 with CHR-ROM -> game database or mapper default, which the converter cannot see and
  assumes absent (it prints a note). hdp3_convert.read_rom() returns it; the checker
  (hdp3_read.rom_chr_ram) derives it separately; negative control: the old a034.hdp fails the
  checker ("file has 1987, expected 0"). Affected corpus packs: the five Metroid ones (a033 x4,
  a034); Zelda II Revamp (a057-a059) is unchanged (its patched header declares no CHR-RAM).
- REBUILD VERIFIED, every difference accounted for (2026-10-16): the whole corpus was
  re-converted (Z:/NES_HDPack_work/out3, now swapped in as out; the previous build kept as
  out_prev_20261016). Of 133 packs, 39 are byte-identical and 94 differ, and all 94 are
  explained: 29 only in the header flags byte (bits 5 "CHR type known" and 4 "game is
  CHR-RAM" newly set: 08->28, 09->29, 0b->2b, 04->34); 60 also in block order only
  (build_backgrounds now groups maps by file name, so sections 4/6/7 are permuted while every
  tile rule, background record and map cell resolves to the same pixels); 5 (a033 x4, a034)
  only in section 2, where a few walk-class tileNearby/spriteNearby conditions gained the
  rewrite-sensitive bit - a converter fix: hdp3_read's brute-force flag check (with the
  Metroid ROM) reports 3 wrong flags on the old a034 and 0 on the new. Checked by
  scratchpad sem_equal2.py (block digests; every tile-rule payload and map cell resolved
  through them), validated by a positive control that swaps two used blocks, edits one, and
  redirects one map cell - each caught by its own check. A first version counted
  background-level rule payloads as block indices (they are background-RECORD indices) and
  reported 26,263 false mismatches on p002, whose old and new renders are pixel-identical on
  Castlevania frames 250/700/1000: a checker bug, fixed.
- The visible symptom before the fix was small: 25 px at line 0 (the additions pass copies
  each line's first pixel record over pixel (0,0), so line 0's BG cache started from line 239's
  tile) and 20 px from drawn additions. Mesen never ran that pass because its list was empty.
- Lookup cost measured on real frames (hdref counters: rules_walked, max_rules_walked,
  conds_evaluated, walk_hist): Metroid gameplay walks at most 61 rules in one lookup, SMB 19,
  against a static worst case of 5495 rules for one a034 key. Size the stage-3 lookup on the
  measured numbers plus margin, not on the static maximum.
- Test ROM trap: Z:/NES_HDPack_work/roms/metroid_mmm.nes was stale (4886 PRG bytes and the
  header differed from what Mesen builds; the game never left the title). Rebuilt with
  tools/rompatch.py from roms/metroid.nes + a034/mmm.ips; PRG and CHR now equal Mesen's
  in-memory patched ROM (dumped with NES_HDPack_testdata/tools/mesen2/dumprom.lua). The old
  file is kept as metroid_mmm.nes.stale.

### Stage-3 lookup: one walk per tile instance (tools/hwmodel.py, 2026-10-15)

Mesen walks a key's rule list per PIXEL. The hardware cannot (256 pixels x up to 5 records
per line, 5456 cycles), so the lookup is done once per tile INSTANCE: a BG fetch group (up to
8 pixels of one line sharing a capture span), the visible slice of one sprite on one line, or
the pixels an `<addition>` copied from one source instance. Each condition is evaluated for
the instance's pixels at once as an 8-bit mask:

    unresolved = all pixels of the instance
    for each rule of the key, in file order:
        mask = unresolved AND (its frame conditions pass ? all : none)
        mask &= each walk condition's per-pixel result
        the pixels in mask take this rule; unresolved &= ~mask; stop when empty
    a pixel's disableCache = OR of ForceDisableCache over rules 0..its match

Per pixel this is the same first match as Mesen's walk, provided conditions are pure.
hmirror/vmirror/bgpriority/positionCheckY are the same for every pixel of an instance;
positionCheckX, originPositionCheckX/Y, tileNearby, spriteNearby are not and are evaluated per
pixel. The BG 8-pixel cache then picks, from the 8 per-pixel answers, the ones Mesen would
have looked up (first pixel, and following pixels while disableCache is set) - a few gates.

`tools/hwmodel.py` is that algorithm, run against hdref's Mesen-order lookup on the same
frame record: it checks EVERY lookup Mesen performs (pixel, payload, disableCache) and counts
the work. Result: 0 differences over 12 frames, 109,000 lookups (Metroid HD a034 frames
200-1100, SMB + p053 frames 240-300; both packs have walk conditions, SMB's are all
tileNearby).

One Mesen side effect is NOT modelled yet: a fallback hit rewrites the looked-up record's
TileIndex in raster order (spec 3.16), and a later tile condition reading that pixel sees the
rewritten value. The model evaluates on the unrewritten capture. (Written when none of the
checked frames had a fallback hit; the tracking below is now modelled, and Zelda II frames
300 and 420 have real ones.)

Whether the rewrite can ever matter is decidable per condition from the pack alone
(scratchpad fallback_rewrite_scan.py). A record (p, i) is rewritten only if its exact key is
missing and the key (p, f) or (FFFFFFFF, f) of f = fallback(i) exists. A tile condition naming
T sees `i == T or fallback(i) == T` on the original record and `f == T or fallback(f) == T` on
the rewritten one, so it can differ only when (a) T itself falls back and (T's key under the
condition's palette, or any palette for IGNOREPALETTE) is rewritable, or (b) a two-hop chain
fallback(fallback(i)) == T exists. Corpus result:

    Zelda II a057/a058/a059      0 conditions can be affected
    Metroid HD a034 / a033 (4)   7 / 9: spriteNearby tile 921 under palettes ff272a3c and
                                 ff261a31 (exact key missing, fallback 665 found), and 3-5
                                 IGNOREPALETTE conditions (tiles 976, 977, 2801, 5873)

So the hardware needs it, but only for conditions the converter can flag. Plan: the
converter marks each tile condition `rewrite-sensitive` and stores, for it, whether a
rewritten target would still match (`f == T or fallback(f) == T`, a constant) and, for
IGNOREPALETTE ones, the small set of palettes whose (p, T) key exists (those are never
rewritten). At evaluation the hardware then needs one more fact about the target record:
was it LOOKED UP before this evaluation, in Mesen's order (raster order; at one pixel the BG
lookup first, then the BG-priority sprites, then the front sprites, each from the last
record to the first). Earlier lines and earlier pixels of this line are settled by the time
an instance is evaluated if instances are processed left to right; the target being in the
SAME instance (a nearby offset of -7..7) needs the 8 pixels taken in order. The same applies
to tileAtPosition/spriteAtPosition, which Mesen evaluates at their FIRST use during drawing
(so possibly after the target was rewritten): a rewrite-sensitive one would have to be
evaluated at first use, not in RESOLVE. The first scan (a033/a034 only, and over-counting:
it flagged any condition naming a falling-back tile) found 9, all Nearby. The converter now
sets the exact flag (condition bit 24: the rewrite must be possible for that palette AND
change the test); on a034 that is 3 conditions, 2 spriteNearby + 1 tileNearby. Its log line
counts the frame-class ones separately; the corpus count comes from the 133-pack rebuild.

DONE in hwmodel (2026-10-15), with the order rule above. A key whose list holds a sensitive
condition is walked one pixel at a time. No recorded frame has a fallback hit, so
tools/test_hwmodel_rewrite.py builds one from a real a034 frame: a sprite of tile 921 under
palette ff272a3c (exact key missing, fallback 665 found) next to a sprite whose rule reads it
with spriteNearby X=8. Two scenes: T to the right of Q (Q first, Mesen sees the original) and
Q mirrored so T is to its left (T first, Mesen sees 665). Result: 0 differences in both with
tracking; 8 differences without it, so the scene does reach the path. The 12 recorded frames
still match (0 differences). The converter's rewrite-sensitive flag is DONE (bit 24 above;
hdp3_read re-derives it by brute force, negative control: clearing a set flag and setting a
clear one are both reported), and hwmodel now takes the flag from the file rather than
recomputing it, so the model runs on what the hardware will read; the a034 rewrite test
still passes 4/4 with only the 3 flagged conditions walked in order.

RECORDED fallback hits (2026-10-15): Zelda II frames 300 and 420 (re-recorded with the fixed
fullsim; check_record and check_watch pass) have 352 and 496 fallback hits. hdref is
IDENTICAL to Mesen on both (512x416), and hwmodel gives 0 differences on 7,936 and 7,168
lookups, plain and --dict (DICT: 50 records / 44 keys and 30 / 23). So fallback resolution
is proven against Mesen on real frames. The REWRITE-tracking path is still proven only by
test_hwmodel_rewrite.py: a057 has no rewrite-sensitive condition (scan above), and both
frames report "rewritten targets read by the model 0". A recorded Metroid frame where tile
921 meets a spriteNearby reader would close that; the a034 recordings so far have none.

Measured cost (max over lines / mean, per line; 5456 engine cycles per line):

    instances           BG 32-44, sprites up to 6 (8 at the sprite limit)
    rule records        full lists: 309 (Metroid line 34)
                        after RESOLVE filtering: 42 Metroid, 213 SMB (no frame conditions)
    walk conditions     212 (SMB line 152), all tileNearby, every one grid-aligned (X, Y
                        multiples of 8), so each reads ONE target tile, not 8 records
    frame conditions    41 distinct per frame (Metroid); memoised, a bit lookup at render
    Zelda II a057       frames 700, 760 (rec_z2, input offset -1): IDENTICAL to Mesen, and the
                        model 0 differences over 19,664 lookups. Sprite instances up to 36 on one
                        line (additions copy sprite records: 2,048 added pixels a frame), rule
                        records 504 / 144 after filtering, nearby reads 72, all grid-aligned
                        Frames 910 and 1300 (overworld): IDENTICAL, model 0 differences.
                        Frame 1000: 1,232 HD pixels differ, ALL inside Link's sprite box; the
                        plain NES frames already differ there (218 px at the best alignment;
                        Mesen 996-1006 all checked, 1002 is the only close one, so the frame
                        mapping is right). Link is at the same position in a different stride
                        pose (unshifted is the best fit, 178 of 864 px off): NES game state one
                        animation step apart after the 940 "LEFT" input, back in step by 1300.
                        An NES-side difference (core or input timing), not an HD one.

Per frame, RESOLVE filtering every key used (50 or so) reads at most 1,233 rule records and
evaluates 2,387 frame-condition references (Metroid title), well inside a vblank. The
filtered lists are short (mean 1-4) but not always: one Metroid title key keeps 495 rules
(walk-conditioned) before its unconditional one; the 8-pixel mask stops the walk early, so the
most any line examined was 73.

The hard part is not rule count but the nearby reads: up to 212 per line, each a random read
of another line's BG span (key) at the target column. That shapes stage-3 memory: the span
keys of the whole frame (240 lines x 34 spans) must be readable at a few cycles each, not at
DDR3 latency.

### Stage-3 memory plan: a per-frame record dictionary (2026-10-15, modelled in hwmodel --dict)

Per-frame sizes over the first 16 recorded frames with lookups (hwmodel RESOLVE/DICTS lines;
Metroid a034 x8, SMB p053 x3, Zelda II a057 x5). A CHR-RAM game raises the record count:
Castlevania's title (p002) has 276 distinct records for only 21 keys, because most tiles on
screen have no HD replacement. Records without a key still need an id, since a nearby
condition can name any tile, keyed or not.

    distinct keys used                  max 53
    distinct (palette, tile) records    max 79      distinct tile indices   max 63
    rule records kept after filtering   max 529 (Metroid title; next 212)
    walk conditions reachable           max 163 (SMB)

Hardware bounds, which the design must still accept: the visible BG is at most 33 x 31 = 1,023
tile slots and OAM holds 64 entries (128 records in 8x16 mode), so a frame cannot hold more
than about 1,150 distinct records before additions. The plan sizes for 2,048 and treats an
overflow as a slow path (walk the DDR3 rule list for that instance), never as wrong output.

RESOLVE (vblank, after the capture of frame N is complete) reads frame N's record once and:
1. gives every distinct key record (palette, tile index, CHR-RAM fingerprint) an 11-bit record
   id, through an on-chip hash table (M10K, linear probing);
2. rewrites the frame's BG spans as record ids into an on-chip SPAN STORE, 240 x 34 x 11 bits
   = 90 kbit (9 M10K), and the sprite records likewise into a SPRITE STORE (8 per line with
   the sprite limit; X, record id, attributes);
3. for each record id: one DDR3 key lookup (exact, fallback, default key, as GetMatchingTile),
   the rule list filtered by the frame-condition bitmask, kept rules appended to an on-chip
   RULE STORE (1,024 x 64 bits: payload, walk-condition list pointer, flags), and a RECORD
   TABLE entry {list start, length, tile index, fallback index, SECOND-hop fallback index,
   palette, rewritable flag}; for a CHR-RAM game the indexes are replaced by the 64-bit
   fingerprint (no fallbacks there). The second hop is needed because a nearby read of a
   target that has been rewritten tests `f == T or fallback(f) == T` with f = fallback(idx).
   A kept rule also carries its PREFIX flag (OR of ForceDisableCache over the whole list up
   to it, dropped rules included), and the list its LAST rule's prefix, used when no kept
   rule matches: dropping a rule must not drop its effect on the BG cache. Real in 5 corpus
   packs (Castlevania p002: 431 kept rules without the flag after a droppable rule with it).
   Frame conditions flagged rewrite-sensitive are not evaluated in RESOLVE but kept with the
   rule and tested at render, since Mesen memoises them at first use.
Then RENDER needs DDR3 only for image rows: an instance reads its record id from the span or
sprite store (1 cycle), its list from the rule store, and every tileNearby/spriteNearby read is
an on-chip span/sprite-store read plus a record-table read (a few cycles, against up to 212
per line). Estimated on-chip total about 40 M10K of 553.

RESOLVE timing (item b below, worked out 2026-10-15). Vblank is 21 lines (241-261), about
114,000 engine cycles. Key resolution is per RECORD and costs up to three key lookups
(exact, fallback, default key; a record with no HD tile at all, the common case in a CHR-RAM
game, misses twice). At the worst case of about 1,150 records and the slow figure of 80
cycles per lookup that is 276,000 cycles, more than two vblanks, so key resolution cannot
wait for vblank. It does not have to: a record's key depends only on its own palette, tile
and fingerprint, never on a condition, so the CAPTURE writer resolves each NEW record while
frame N is being drawn (240 lines, 1.3 M cycles; a line adds at most about 40 new records).
Only the filtering needs the finished frame (memory conditions read the scanline-240
snapshot, tileAtPosition reads any pixel), and it reads each used key's list once: at most
1,233 rule records on the frames measured, 16 bytes each, contiguous, plus the condition
records on first use (memoised). That is a few thousand cycles, inside the vblank with room
for the worst case. Records discovered after the snapshot cannot exist (capture ends at
line 240), with one exception: <addition> inserts sprite records in RESOLVE. But the tile and
palette an addition inserts are fixed in the pack, so their keys are known at conversion;
the converter can store each addition's resolved list offset (or "no key") with it, and
RESOLVE needs no lookup for them. So: resolve during capture, filter in vblank, render
during frame N+1.

Open, to settle before writing RTL: (a) model the dictionary in hwmodel (record ids instead
of keys) and require 0 differences on all recorded frames - DONE as `hwmodel.py --dict`
(per-frame filtered lists, prefix flags, deferred sensitive frame conditions), 0 differences
on all 19 recorded frames (Metroid a034 x8, SMB p053 x4, Zelda II a057 x7). Per frame at most
79 records over 53 keys (Zelda II has more records than keys: records sharing a fallback or
default key share its list), 1,233 rule records read, 2,387 frame-condition references,
529 rules kept (longest kept list 495, the Metroid title); negative control
tools/sim3/neg_dict_model.py, run by tools/sim3/run_dict_model.sh (three mutants: keep every
rule, stop at the first kept rule, ignore the prefix flag). Castlevania p002 catches the
first two on frames 250-1300, but NO recorded frame can show the third, and it was missed on
all six Castlevania frames. The 8-pixel BG cache never changes a lookup RESULT; the prefix
only decides which pixels are looked up, which shows only through a fallback rewrite read by a
rewrite-sensitive tile condition, or through the disableCache flag itself. A corpus scan
(scratchpad prefix_reach.py) found 17 packs with a prefix inherited from a frame-filtered
rule, 6 of those with a fallback map, 5 with a rewrite-sensitive condition, all five Metroid
(CHR-RAM, where fallback rewrites never happen). So tools/sim3/make_prefix_frame.py BUILDS
the case on Zelda II a057 (CHR-ROM): point a keyless tile's fallback entry at a new key (added
in an empty probe slot within max_probe, so no existing lookup moves) whose list matches the
tile's group at a later rule, give an earlier non-matching rule ForceDisableCache and
recompute the prefix bits as the converter would, and plant a rewrite-sensitive tileNearby on
a later group. Kept only if the unmutated model has 0 differences and the mutant differs.
Result on frame 700: unmutated 0, the three mutants 2304 / 2304 / 5376 differences - 3/3.
The fixture is also the ONLY input that exercises hwmodel's fallback-rewrite tracking: all 25
recorded frames (hw_all --dict, 2026-10-16, every one 0 differences) make 0 fallback
rewrites, while on the fixture the reference makes 2,304 and the model reads 288 rewritten
targets through 1 rewrite-sensitive condition - 0 differences plain and --dict.
Two traps found on the way: the generator must search inside the pack's overscan (a057's
top overscan is 32, so lines above 32 are never looked up and every trap there is invisible),
and render() compares the BG disableCache flag directly, so a group's own pixels already
show the mutant. Stage 3's hd3_render test must include this fixture; (b) RESOLVE
time - SETTLED above (resolve keys during capture, filter in vblank); (c) the logic cost,
measured with the yosys estimate as each block is written, since ALMs (not memory) are the
limit.

### Stage-3 RTL blocks (plan, 2026-10-15)

Three blocks, each with a golden reference from hwmodel (`--dict` gives the per-frame state
the RTL must reproduce) and its own C++ testbench, in the stage-2 style:

1. hd3_dict - runs during CAPTURE. Takes each BG span and sprite fetch as hd_capture emits
   it, forms the record (palette from the line's palette RAM, tile index from the CHR
   address, fingerprint), finds or allocates its 11-bit id in an on-chip hash, writes the id
   into the span / sprite store, and for a NEW record queues the key resolution (hd3_lookup:
   exact, fallback, default key; fallback and second-hop fallback read from section 10).
   Output: RECORD TABLE {palette, tile, fallback, fallback2, fingerprint, list offset or none}.
   Test: the record table and stores after a frame against hwmodel's dictionary.
2. hd3_resolve - runs in vblank. Frame conditions (memory from the watch snapshot,
   frameRange from the frame number, tileAt/spriteAt from the stores and record table) into
   a memo, background selection, additions inserted (resolved at conversion), each used
   list filtered into the RULE STORE with prefix flags. Test: rule store against hwmodel's
   filtered lists, frame by frame.
3. hd3_render - runs line by line in frame N+1. Instances, 8-pixel masks, walk conditions,
   nearby reads from the stores, rewrite tracking for flagged keys, block fetch, compositing
   with backgrounds, alpha and brightness, grayscale and emphasis, the 2x line buffer. Test:
   the output picture against hdref (and so Mesen), per recorded frame.
Each block's yosys estimate is taken when it is written; the MVP engine is removed when
stage 3 replaces it, which is the room these blocks have to fit in.

ALM budget, from the last real fit (output_files/NES.fit.rpt, 2026-10-15 18:24, failed at
42,539 of 41,910 ALMs; per-entity table, scratchpad fit_entities.py):

    NES core          19,008   (mappers 9,303, PPU 5,691)
    framework etc.     ~7,200  (ascal 2,158, audio_out 954, hps_io 992, OSDs 939, ...)
    hd_engine (MVP)    7,569   ALUTs 5,732, REGISTERS 15,624
    hd_stage2          3,037   hd_chrfp 2,569 (before its tile store moved to block RAM)
    composite_decoder  1,190   (+27 DSP)

M10K budget (8l, measured 2026-10-21 without a compile). The chip has 553 M10K blocks, and
it is BLOCKS that run out, not bits: the 2026-10-15 fit placed 238 blocks while using only 30 %
of the block-memory bits (the 30 % figure quoted under "Frame budget and the dictionary read
port" measured the wrong unit). From that fit's RAM summary (scratchpad fit_m10k.py):
hd_engine (the MVP) 78, ascal and other framework RAMs ~75, hd_stage2 48, video_mixer 15, NES
core 12, composite_decoder 8, eeprom/miracle/misc ~2 each. Without the MVP: 160 blocks, which
leaves 393 for the stage-3 engine.
The stage-3 engine's memories, elaborated by Verilator with every parameter resolved
(--json-only on the tb_engine file list; scratchpad m10k_est.py; each array priced at the
cheapest M10K shape for its depth and width: 8192x1, 4096x2, 2048x5, 1024x10, 512x20, 256x40):
about 496 blocks. Largest: hd3_fmap pixel maps pm_e + pm_o 56 + 56 (65,536 x 7 each, the two
snooping banks), hd3_dict kmem 40 (4,096 x 96) / rmem 26 / smem 24 / pmem 12 / apool 12,
hd3_scanout ring 20 (2,048 x 96), hd3_resolve rt 20 / rs 15 / lt 10, hd3_rcache 13, hd3_fmap
sprite lanes 16 x 2, hd3_add lanes 16 x 1, hd3_render bge 9 / img0+img1 14 / bgm 8.
Sizes checked against the source (hd3_fmap.sv:71-72, hd3_dict.sv:143). Arrays with depth <= 64
(bgm, bgr, bgc, of_m, lb, palm, bmb, ab0-2, lbuf, ...) come to about 39 blocks if priced as
M10K; Quartus may build them as MLAB or registers instead (ALMs, not blocks). So: 457-496
blocks against 393. THE STAGE-3 ENGINE DOES NOT FIT IN M10K AS WRITTEN, by roughly 64-103
blocks, before ALMs are considered. Only a compile is exact (inference shapes, MLAB choices,
true-dual-port duplication), but the deep arrays alone (pixel maps, keys, ring) cost what
they cost on any packing. Candidate levers, largest first, none applied:
- one fmap bank instead of two (-56): the 2026-10-17 decision for two banks was made on the
  bits figure; one bank puts the 100,802-clock fill back in vblank (the window problem above).
- narrower or shallower dictionary keys/records (kmem 96 bits, 2 x NREC entries).
- scan-out ring 2,048 x 96 (20): 4 lines x 512 qwords is what it holds.
- the 2-entry output FIFO (OF_LOG2=1, already sim-verified to lose nothing): of_m 8 x 116.
- composite_decoder (8 blocks, 1,190 ALMs, 27 DSP): a user decision.
OPEN_ITEMS 11 carries the decision.

The yosys estimate is NOT a stand-in for ALMs: it gave the MVP about 1,900 LUTs and 2,457
flip-flops, Quartus placed 15,624 registers. Arrays yosys mapped to block RAM, Quartus built
from registers (an inference rule mismatch: a reset, an extra read port or a combinational
read stops Quartus inferring M10K). So every stage-3 memory is written in the plain Quartus
inference shape (one write port, registered read, no reset, no initial clear loop) and its
register count is checked against the array size before it is trusted.

### hd3_dict (rtl/hdpack/hd3_dict.sv) - the per-frame record dictionary

Input: a tap of hd_capture's write stream (each qword hd_ddr accepts: address, data, byte
enables). The frame record already carries everything a record needs, so hd3_dict listens
rather than taking a second set of PPU taps. Per line block (record layout above) it keeps the
raw header qwords q0-q3 (span count, sprite fetch count, palette RAM), the span qwords and the
sprite-fetch qwords in a line store of 4 line slots (y & 3) x 128 qwords, M10K.

A line is processed when the NEXT line's header q3 arrives (line 239 at the trailer). hd_capture
writes a line's header at its line_done, with the span fetched at dot 256 landing on the same
clock, so a line is only certainly complete once the next header is out; the next line's
fetches (its sprites from dot 257, BG slots 0-1 from dot 321) go to other slots. A slot is used
only if its header was seen this frame (a capture enabled mid-frame must not read stale data).

Record and key, matching hdref.build_records and find exactly:

    BG span s < span count:   pal = c[pb+3] | c[pb+2] << 8 | c[pb+1] << 16 | c[0] << 24
                              (pb = 4 x attribute palette; no c[0] byte in a legacy pack)
    sprite fetch i, valid:    pal = c[q+3] | c[q+2] << 8 | c[q+1] << 16 | 0xFF << 24
                              (q = 0x10 + 4 x (attr & 3); top byte 0 in a legacy pack)
    body                      CHR-RAM game: the fingerprint (qB); else the tile index
                              (CHR linear address & 0xFFFFF) >> 4

(c = the line's palette RAM.) The dictionary key is (pal, kind, body), the pack's own key: two
records with the same key resolve the same way by construction, so hwmodel's (pal, idx, fp)
id - which in a CHR-RAM game also splits by tile index - is finer than needed. Addition
records are not seen here (they are inserted before drawing, from fixed keys the converter
resolves); note that in a CHR-RAM game an addition names its target by fingerprint and has no
tile index, so "has a key" must be tested as find() does it (fixed in hwmodel.build_dict on
the same day: it tested the tile index and dropped Castlevania's additions).

Dedup: an on-chip hash index, 4096 slots of {valid, record id}, linear probing, hash = an XOR
fold of the 96-bit key (cheap; quality only costs probes). A miss allocates the next id
(at most 2048 per frame; beyond that `overflow`, the record gets no id) and writes its key to
the key store. The index is cleared after each frame's last line (4096 clocks, inside vblank).

Stores, two banks (frame N's are read by RENDER while frame N+1's are written; bank = the
frame-record buffer's address bit 20, i.e. frame number & 1):

    key store       2048 x 96   {pal, body}            written at allocation
    result store    2048 x 64   {done, via, list offset/8, fallback, fallback2}
    span ids        240 x 34    {valid, id}            per line and BG slot
    sprite ids      240 x 16    {valid, id}            per line and fetch (this PPU fetches
                                                        at most 16 sprites per line)

RESOLVE of a record (key resolution only; list filtering is hd3_resolve's job) runs in the
background in allocation order, sharing one DDR3 read port: hd3_lookup in key_only mode for
the exact key; in a CHR-ROM game, on a miss, the fallback map entry (section 10) and then
(pal, f) and (FFFFFFFF, f); then (FFFFFFFF, body). `via` records which one answered (none /
exact / fallback / default); `via == fallback` is exactly hwmodel.rewritable. fallback2 =
fallback(fallback(idx)), needed by the rewrite rule for tile conditions. A CHR-RAM game has no
fallbacks. Budget: up to 4 lookups of about 55 clocks each plus 2 map reads, so ~300 clocks
per record worst case; 2,048 records fit in a frame (1.43 M clocks), and the frames measured
so far need 30-276 records.

Test plan (tools/sim3/tb_dict.cpp + dict_golden.py): replay recorded frame records as the
write stream in hd_capture's real order (per line: BG slots 2-33, header, the next line's
sprites, the next line's slots 0-1), with random gaps, over a DDR3 model holding the .hdp;
the golden is computed from the same rec_FFFF.bin with hdref's own functions. Checked per
line and slot: the record's key and resolution against the golden, plus id consistency
(same key -> same id, different keys -> different ids). Two different frames alternate banks,
then the first is replayed again (index cleared, ids rebuilt). Negative control: wrong
palette byte order, legacy flag ignored, fallback not tried, default key not tried,
second-hop fallback missing, index not cleared between frames, a line processed before the
next header.

Status (2026-10-15): written, Verilator lint 0 warnings, about 126 M10K by estimate (every
memory in the Quartus inference shape). 0 mismatches against dict_golden.py on SMB p053,
Zelda II a057 (the fallback path), Castlevania p002 (CHR-RAM keys, default-palette results),
a chain fixture (make_fb_chain.py: a057 with second-hop fallbacks, fb2 set on 576 of 608
records) and the legacy packs p125/p126 on SMB3 frames, serial and overlapped, with line 100's
header dropped. Notes from the test:
- The testbench paces each line's writes at the real rate, 341 dots x 16 = 5,456 clocks. The
  first version streamed as fast as it could and overran the 4-slot line store, which no
  hardware can do; the failure was the testbench's.
- The planned `late` flag is folded into the sticky `overflow`, which has four causes: a line
  slot reused before it was processed, a bank restarting before its frame_done, more than 16
  fetches on a line, more than 2,048 records.
- A tile is 8 lines tall, so dropping one line's header never changes the record count; the
  observable is that line's own id slots (all 50 must read invalid).
- r_frame (the frame number per bank, from the trailer) was added for hd3_resolve's frameRange.
Negative control tools/sim3/neg_dict_rtl.py: the real RTL passes three suites (A Zelda II
chain pack; B Castlevania frames 250/450/700/1000/1300, overlapped, line 100 header dropped -
50,190 items, 0 mismatches with the two later frames added; C the legacy pack p125), and
16 of 16 mutants are caught, the legacy-flag one only by suite C.

### hd3_resolve (rtl/hdpack/hd3_resolve.sv) - filtering the frame's rule lists

Runs once per frame in vblank, started after hd3_dict's frame_done for the bank (and the
watch snapshot of that frame). For each record id in order: if hd3_dict resolved it to a list
(via != 0), the list is filtered ONCE per frame (a list table, 2,048 entries, linear probing
on the list offset, shares the result between records); otherwise the record gets kind 0.
Filtering is hwmodel.filter_list exactly: walk the rules in order; a rule's frame-condition
list is evaluated condition by condition, a condition flagged rewrite-sensitive (q0 bit 24) is
skipped and marks the rule "deferred", any other is evaluated through a per-frame memo
(indexed by the condition's memo index) and the first false drops the rule; a kept rule is
appended to the RULE STORE; the list stops after a kept rule with no walk list and nothing
deferred. The list's LAST rule's prefix flag is read first (render's flag when no kept rule
matches, dropped rules included).

Condition evaluation (hdref.eval_cond), on the frame as captured:
    memoryCheck / memoryCheckConstant   the watch snapshot through hd_watch's value port
                                        (1-clock latency), (a & mask) OP (b & mask | value)
    frameRange                          A != 0 and frame % A >= B, 32-step restoring divide,
                                        frame = hd3_dict's copy of the trailer
    tileAtPosition / spriteAtPosition   the record at Y*256+X rebuilt from the frame record in
                                        DDR3 as hdref.build_records does: pixel byte 0 gives
                                        the BG slot (63 or >= span count: none) and bit 6
                                        "sprites shown"; spans and fetches from the line block;
                                        sprites are the valid fetches covering X, at most 4,
                                        in fetch order; then hdref.tmatch, with the fallback
                                        map read for idx == T or fallback(idx) == T
Outputs, two banks, read by render with 1-clock latency:
    record table    [bank, rid] {kind 2, last prefix, count 11, start 10, list offset / 8}
                    kind 0 no list, 1 filtered, 2 unfiltered (the rule store filled: render
                    walks that list from DDR3, the planned slow path; sticky `overflow`)
    rule store      [bank, 1,024] {prefix, walk list ptr 20, frame list ptr 20 or 0, payload}
                    the frame list pointer is kept only when the rule has deferred conditions

Field widths, measured over all 133 packs: section 3 (condition lists) is at most 45,468 bytes
(p074), so 20-bit list pointers have room; the largest memo index is 1,181 (p002) against a
2,048-entry memo (a larger one is evaluated unmemoised, never wrong); the largest rules section
is 3.6 MB, so a list offset / 8 fits 25 bits.

Timing. The first version took 140,506 clocks on the Metroid title (frame 200), over the
114,000-clock vblank: one line buffer served both the rule stream and the condition-list
words, so every rule evicted the other. Two changes: four 8-qword line buffers, one per stream
(rules, condition-list words, condition records, random reads); and a condition-LIST result
cache (1,024 entries, direct-mapped on the list pointer, full tag), since within a frame a
list's pass/fail depends only on its pointer (every condition memoised or deferred) and the
Metroid title's 1,190-rule list uses two condition lists. Now 25,580 clocks on the same frame
with 12-clock DDR3 latency, and 47,180 with 60-clock latency, a 40% busy port and the next
frame's capture overlapping.

Test (tools/sim3: hd3_resolve_tb.sv joins hd3_dict and hd3_resolve as stage 3 will, each with
its own DDR3 port; tb_resolve.cpp; resolve_golden.py from hwmodel.filter_list; check_resolve.py;
run_resolve.sh runs the suite). Per record: kind, list offset (against hd3_dict's), every kept
rule and the last prefix against the golden; every golden list must be reached. 0 mismatches
on Metroid a034 frames 200/400/880/1020 (frame types 1-4, the 495-rule list), SMB p053
240/270/300, Castlevania p002 250/450/700/1000/1300 (CHR-RAM, fingerprint data conditions,
1,182 frame conditions) and Zelda II a057 300/420/700/1000 (fallbacks in tileAt), serial and
overlapped - plus the fixtures below, 13 runs, all 0 mismatches (2026-10-15). Fixtures reach
paths no recorded frame does:
- No corpus pack flags a FRAME condition rewrite-sensitive (the 23 flagged conditions are all
  walk-class), so make_sens_frame.py flags 199 of Metroid's 593 frame conditions; 983 kept
  rules then carry deferred lists, and the deferral path matches the golden.
- No fixture pack has a frame-class two-slot memoryCheck, and their masked conditions never
  change a result on the recorded frames. make_mem_frame.py turns every 4th frame
  memoryCheckConstant of p002 and a034 into a memoryCheck against the next watched slot (51 and
  48) and gives every 4th one mask 0x0F (55 and 42).
- No recorded frame puts a frameRange or ordered memory comparison exactly on its boundary where
  it matters, so >= vs > and <= vs < were invisible. make_edge_frame.py sets every frameRange's
  B to frame % A and every ordered memoryCheckConstant's constant to the watched value, each for
  one of the run's frames in rotation. Memo indexes are unique per condition in both packs
  (checked), so retuning one cannot leave a stale shared memo entry.
- No recorded positional condition sits where a palette mismatch, a tile-row boundary, a
  sprite's right edge, a fallback tile or the sprite palette half decides its result.
  make_pos_frame.py turns frame conditions of a034, a057 and p002 into traps of each kind:
  wrong palette with ignore clear; the exact tile on the last line of its tile row (the next
  line's span at the same slot differs); a sprite's tile one pixel right of it; T =
  fallback(idx); a sprite on a dot where sprites are not shown; an exact sprite match whose
  BG-half palette group would differ.
- WHY THE FIRST VERSIONS OF THE LAST TWO FIXTURES CAUGHT ONLY 20 OF 29 MUTANTS (2026-10-15):
  they tuned condition i for frame FRAMES[i % n] whether or not that frame evaluates it, and a
  later trap could make an earlier one unreachable (a rule now passes first, so the list stops
  before the trapped rule). Measured with sim3/trap_sense.py: on Metroid NO positional condition
  was evaluated at all. Both generators now use sim3/fixture_pick.py: a condition is a candidate
  only if its frame evaluates it, a trap is kept only if inverting it changes the golden
  (`visible`), and it must stay reachable as later traps are placed (`all_reach`, re-checked at
  the end). Result: two traps per role per pack (edge: range/gt/lt/le/ge on p002 and a034;
  position: every role that the frames can host), and the RTL negative control
  (neg_resolve_rtl.py) catches 29 of 29 mutants. Roles a pack cannot host are reported with the
  reason: Metroid's frames make 0 fallback rewrites (so no tile_fb trap; Zelda II has two),
  and only rec_mmmhid has a sprite on a dot where sprites are hidden.
- No legacy pack (version < 100) in the corpus has a condition (all 7 have 0), so the legacy
  palette path (record palette top byte 0) was reached by no fixture. make_legacy_cond.py writes
  a variant of Super Mario Bros p125 with 24 positional conditions on its tile rules (only
  tileAtPosition/spriteAtPosition: Mesen rejects frameRange and memoryCheck below version 101),
  converted with hdp3_convert --base; make_pos_frame.py then traps it on rec_smb3 240/270/300.
  sim3/make_fixture_goldens.sh regenerates every rewritten fixture's goldens.
- No recording, in any game, has a valid sprite fetch on a dot where sprites are not shown
  (all frames scanned). make_hidden_rec.py copies Metroid frames 0880/1020 with the
  sprites-shown bit cleared on the left half of every line - a legal record, what the PPU
  writes with sprite rendering off there - for the hidden-sprite traps.
- A build with RS_MAX=64 (VFLAGS=-GRS_MAX=64) fills the rule store on the Metroid title: 16
  records go unfiltered (kind 2) and every list filtered before that is still exact.
The golden inserts additions before evaluating (Mesen's order); the RTL does not (stage 4).
Additions can change spriteAtPosition and, through Mesen's lineFirstPixel copy, tileAt(0,0),
so on a pack with additions RESOLVE is exact only where no frame condition reads them. The
recorded frames of p002 (528 additions) and a057 (2,267) match nonetheless.

Room for stage 3: removing the MVP gives back 7,569; if hd_chrfp's change lands as expected
(2,569 down to a few hundred) the design without stage 3 is about 32,700 ALMs. At a 92% fill
(38,550, where fits still close reliably) stage 3 has about 5,900 ALMs for hd3_dict,
hd3_lookup, hd3_resolve and hd3_render together. That is tight; composite_decoder (1,190) is
the next candidate to compile out of an HD build if it is needed (user decision).

### hd3_render - plan and measurements (2026-10-16)

Measured before writing it (scratchpad add_census.py, bg_census.py, add_static.py,
add_keys.py, cond_types.py, chrram_idxcond.py; hwmodel --dict census):
- Per line, worst over the recorded frames: 42 BG instances, 36 sprite instances, 150 rules
  walked, 171 walk-condition evaluations (each up to 8 pixel reads for nearby conditions).
- Backgrounds: Zelda II a057 frame 420 selects 28 backgrounds (7 per layer); per line at most
  99 non-transparent cells are drawn and only 14-35 distinct (block, row) pairs, so a per-line
  row cache cuts the image reads to about 70 bursts.
- Additions: 6 corpus packs (8 files) use them, two of them test packs (Castlevania p002, Zelda
  II a057); frames fire up to 2,048 added sprites. No pack has an addition whose ADDED key can
  match another addition's ORIGINAL key (0 chain pairs), largest offset 64 pixels. p002 and a057
  also use spriteNearby, so nearby reads must see added sprites.
- Every CHR-RAM pack uses data tile conditions, every CHR-ROM pack index ones; 4 packs whose
  CHR kind the cart decides use one or the other.

Data flow per frame N (rendered while the NES draws frame N+1):
1. CAPTURE (frame N): hd_capture, hd3_dict (keys), and a pre-pass trailing the capture that
   emits each addition's candidate sprites into per-pixel slots in DDR3 (stage 4).
2. VBLANK: hd3_resolve (background selection, then list filtering, stage 4 adds both), and a
   frame FILLER that copies the record's per-pixel slot bytes and per-line sprite X/ids into
   on-chip frame maps (single bank: frame N's maps are only read by frame N's render).
3. RENDER, per line: load the line (pixels, header, spans, sprites), build each pixel's sprite
   list (covering valid sprites in fetch order, at most 4), walk instances exactly as
   hwmodel.walk (8-pixel masks; a rewrite-sensitive condition restarts the instance one pixel
   at a time), apply the BG 8-pixel cache rule left to right (marking looked-up pixels in a
   frame-wide bitmap for rewrite tracking), then composite each pixel's layers in Mesen's order
   with the 2x2 sub-block of each image (one qword per output row), grayscale and emphasis, into
   an output ring of 4 NES lines. Only pixels inside the overscan are looked up (as hdref).
Built in steps, each tested against hdref: v1 tiles, sprites, backdrop, colours, overscan,
effects; v2 backgrounds; v3 additions (and the line-0/pixel-(0,0) copy rule they bring).

### hd3_render v1 - tested, and measured too slow (2026-10-16)

Test: tools/sim3/hd3_render_tb.sv + tb_render.cpp wire hd3_dict, hd3_resolve, hd3_fmap and
hd3_render with one DDR3 port each (latency 12, no contention) and collect out_* into the 2x
image; check_render.py compares it pixel for pixel with hdref.render (backgrounds and additions
left out of the reference until v2/v3). run_render.sh: 4 recorded-pack sets (SMB p053, Metroid
a034, Castlevania p002, Zelda II a057; 16 frames) and the 11 resolve fixture packs (42 frames),
plus two builds with lowered limits that must TAKE their rare path: NSLOT=4 (uncached tiles
through the scratch slot, 4,842 times) and RS_MAX=64 (unfiltered lists walked from DDR3, 33,240
times). All pixel-exact.

Bug the fixtures found: the deferred frame-condition memo was tagged with a 1-bit frame parity,
so on every second frame an entry never written in the previous frame read as a hit with value
0 (a034_sens frame 400, 578 pixels). Fixed: {valid, value} entries cleared by a 2,048-clock
sweep at frame start, overlapping the palette load. Also: a sized literal in Verilator -G
(-GNSLOT=8'd4) is silently ignored; use a plain integer.

Cycle profile (PROFILE=2 build; clocks per line, budget 5,456):

    frame            mean    max     lines over   peak backlog
    SMB p053 300     6,411   23,938   74           48 lines
    CV p002 250      7,563   16,225  193           93 lines
    mean per line    load 603, scan 259, BG instances + cache rule ~1,285, sprites 56-88,
                     compose ~2,640, walk 530-698, condition eval 1,049-1,981

Every line pays ~4,800 before any rule is walked, and the phases run one after another. The
planned 4-line output ring cannot absorb a 48-93 line backlog, so v1 does NOT meet the budget.
Speed-up plan, measured after each step: (1) BG cache rule one pixel per clock (now 3);
(2) load line L+1 while line L is composed (double-buffered line data); (3) compose as its own
engine running line L while the lookup engine works on line L+1, with the per-line results and
the image slots double-banked, so a line costs max(lookup, compose) instead of their sum;
(4) compose fewer clocks per pixel; (5) cheaper nearby-condition evaluation. If the bursts still
exceed what an on-chip ring can hold, the fallback is an HD frame buffer in DDR3 (whole-frame
slack, one more frame of latency) - a user decision.

Speed-up results so far (all suites pixel-exact after every step; worst line / peak backlog):

    step                                      SMB p053 300         CV p002 250
    v1                                        23,938  48 lines     16,225  93 lines
    BG cache rule 1 px/clock, tileNearby      13,349  14.8         15,809  37.9
      pipelined (TP)
    compose split into a front stage          11,275   6.8         13,848  11.1
      (fetch + plan of pixel N+1) and a back
      stage (one draw step per clock), 8-entry
      output FIFO, dual-read emphasis ROMs
    spriteNearby pipelined (SP0/SP1)          11,275   6.8          7,997   2.0
    rcache/ccache 2-clock hits, condition      8,954   3.4          6,799   0.9
      prefetch (two buffers: next rule's
      first condition read ahead with its
      rule record; next list entry / walk list)

tools/sim3/prof_all.sh profiles all 16 base frames: Metroid and Zelda II never exceed 3,300
clocks a line (no backlog); SMB 1.3-3.4 lines, CV 0-0.9 lines. So every measured frame now
fits the planned 4-line output ring, SMB 300 with the least margin. Remaining levers, in order of
measured size on SMB line 152 (8,954 clocks after the cc_ci fix below): the tileNearby drain per condition (TP, 3,180 for
212 conditions), condition cache misses (E_C0/E_LW0), the line load (S_LOADW 603 + scan 259,
which could overlap the previous line's compose with double-buffered line data), and BG image
instances (S_BOX/S_BCP 355 each). The negative control (neg_render_rtl.py) gained mutants for
the pipelined nearby conditions, the compose plan / FIFO guard and the three prefetch tags.

Two bugs found while closing this out (2026-10-16), both by tests written for the purpose:

- The condition prefetch shared the cache's index register (cc_ci) with the main path, which
  loads it in E_LW0 but sends the request later from E_C0; a prefetch in flight in between
  overwrote it and the main path read the wrong condition record. The four recorded sets and
  every earlier fixture passed - the memo negative control showed why: the deferred-condition
  memo ran in only one fixture (Metroid sens, 5,009 hits) whose memoised values never change
  between its frames and are never negated, so neither memo mutant was observable. Two new
  fixtures, cvsens and z2sens (make_sens_frame.py on p002 and a057, built and goldened by
  make_fixture_goldens.sh, run by run_render.sh and run_resolve.sh), cover a memoised value that
  changes between frames and a negated one; cvsens frame 700 showed 7 wrong pixels, bisected to
  one flagged condition (c547, a frameRange before hmirror + spriteNearby walk conditions) and to
  the prefetch by disabling it. Fixed: the main path keeps its index in m_ci and loads cc_ci only
  on the clock it sends the request. The neg control has a two-edit mutant reproducing the bug.
- hd3_ccache acknowledged a miss on the last fill beat, one clock before that beat's lane and
  tag writes land; a request captured on the very next edge (a requester reacting
  combinationally) read the old lane or tag. The renderer is a registered requester (its request
  lands two edges after the ack) and was never exposed, but the cache now answers one clock later
  (S_HIT, as hd3_rcache already did). Found by the new cache unit test, tb_cache.cpp +
  hd3_cache_tb.sv: a random requester (registered and combinational) against a DDR3 model whose
  contents change at every flush, both caches built with 4 lines so sets conflict; it runs
  first in test_render_all.sh. Its negative control neg_cache.py catches 9 of 9 mutants; a tenth
  (valid bit not cleared at the start of a refill) was unobservable, because one request is in
  flight at a time, and the clear was removed from both caches as dead logic.
- The render negative control (20 mutants, first full run) missed six. None was a fluke of
  timing; each named a scene no fixture contained, and building those scenes found a real bug.
  Four new fixtures, each made from a recorded frame by a generator in tools/sim3 and each
  checked to reach its path (the unmutated RTL passes, the mutant fails, hwmodel agrees with
  the reference):
  - z2rw (make_rewrite_frame.py): a BG group whose lookups disable the cache, with a sensitive
    tileNearby(-1,0) reading the group's previous pixel, which that pixel's own fallback lookup
    has just rewritten. Catches "sensitive condition in a group not walked per pixel" and
    "tileNearby ignores rewrites". The first version of this fixture set ForceDisableCache
    (rule word 3 bit 24) but not the prefix bit beside it (bit 25, the running OR the hardware
    reads), and RTL and hwmodel both disagreed with the reference - the pack was inconsistent,
    not the renderer; hdp3_convert always writes both.
  - z2slot (make_slotrow_rec.py): a second sprite of one tile at another tile row beside the
    first on one line, both drawn with the same block. Catches "image slot reused for a
    different tile row". Recorded lines always hold 8 fetches (unused OAM slots are invalid
    dummies), so the copy replaces a dummy; a candidate is kept only if the reference draws both
    with one block - SMB's pack picks the block from spriteNearby context, so most do not.
  - z2ph (make_phase_frame.py, edited frame + pack): a sensitive spriteNearby(0,0) whose target
    sprite (transparent, rewritable key) Mesen looks up AFTER the evaluated one. Catches
    "spriteNearby ignores the sprite's phase".
  - z2rw0 (make_rewrite_frame.py --dx 0): tileNearby(0,0) - the condition reads the record
    being looked up itself. THE BUG: Mesen rewrites that record before walking its rules, so it
    sees its own rewritten index; hwmodel.looked_up and the RTL's precedes() used a strict
    order, so the record was "not yet looked up" by its own conditions, and the per-pixel
    looked-up bitmap is written only after the result. Both now count the record itself as
    seen (hwmodel.looked_up; hd3_render seen(), used at all four rewrite sites and their
    counters). z2rw0 failed on both frames before the fix and passes after; the phase fixture
    first exposed it as an 8-lookup hwmodel mismatch that the image hid (both sprites drew one
    block). Real packs carry sensitive offset-0 nearby conditions: 9 in Metroid (a033 x4, a034;
    685 offset-0 nearby conditions in the 137-pack corpus, 616 of them spriteNearby). The neg
    control has a mutant dropping the self term.
  The second full run (20 mutants, after the four fixtures above) caught 16. Three of the
  misses were closed by making the path decisive; the fourth turned out unobservable:
  - grayscale / emphasis ignored: no recording sets PPUMASK bits 0/5-7, so neither effect ever
    decided a pixel. mmmfx (make_effects_rec.py) sets grayscale, emphasis R, emphasis G+B and
    grayscale+all emphasis on bands of lines of two Metroid frames.
  - compose overruns the output FIFO: UNOBSERVABLE, mutant removed. Measured with the stall
    taken out, the FIFO never holds more than one pixel on any fixture - depth 8 or 2, with
    or without the NSLOT=4 scratch stalls (4,842 of them). The output stage pops one pixel
    every 2 clocks, on the clock the next lands, and compose can issue at most two pixels
    back to back (the front stage refills its one-plan buffer no faster than every 2 clocks).
    The stall is kept as a guard for a faster front stage (an open speed lever). The depth
    is now a parameter (OF_LOG2, default 3 = 8 entries, logic unchanged) and an OF_LOG2=1
    build (2 entries) runs the four base sets: the stall fires there (sim_ofstall, printed
    as `ofstall` on the paths line, required non-zero) and every frame must still match,
    which checks that stalling loses nothing. ALM candidate for 8g: since the peak is one
    pixel, a 2-entry FIFO would do; it was not changed (the stall then fires often, and its
    effect on line timing has not been measured).
  - scratch slot reused without comparing the tile row: in the NSLOT=4 build Metroid and SMB3
    lines run out of slots constantly, but two uncached steps of one block at different rows
    are never consecutive - a BG tile's step sits between neighbouring sprites. smbscr
    (make_slotrow_rec.py --dx 0) puts a copy of a sprite exactly over the original at another
    tile row, so the two are consecutive steps of every pixel they cover; it runs in the
    NSLOT=4 section of run_render.sh only.
  The two "no record" mutants (tp_dead[5], si_dead[3]) were removed: a pixel's sprite list holds
  valid fetches only and every BG span gets a record, so a target without one exists only after
  hd3_dict overflowed (more distinct records than ids). Since 8o (below) such a frame is drawn
  with no lookups at all, so no condition ever reads the missing record and both guards stay
  unreachable.

### 8o - a frame whose dictionary overflowed is drawn without HD lookups (decided 2026-10-17)

hd3_dict has 2**RAW record ids per frame (2,048 at RAW=11). It can also fail to finish a frame
cleanly in three other ways: a slot reused while still allocated, a frame started before the
previous one's lines were all processed, and more than 16 sprite fetches on a line. Any of the
four sets `frame_ovf[bank]` (cleared when that bank starts its next frame); hd3_render latches
the bit at `start`, and for that frame every BG and sprite walk ends at W2 with "no rule": each
pixel keeps its original colour, and no condition is evaluated. Backgrounds (8i/8j) do not
depend on the dictionary and are still selected and drawn.

Rejected: falling back per record (draw the records that got an id, skip the rest). Resolve's
tileAt rebuilds keys itself while render's positional conditions use dict ids, so on an
overflowed frame the two would disagree about which records exist and a condition's answer
would depend on which side asked. A whole frame without HD is a visible, explainable fallback;
a frame where some tiles are HD and some are not, decided inconsistently, is neither.

Measured headroom: the recorded games use 29-111 distinct keys per frame, so 2,048 is ~18x the
worst case seen. The test therefore lowers the limit: run_render.sh's RAW=6 build (64 ids)
makes every Castlevania frame overflow (73-111 keys) while SMB3, Metroid and Zelda II stay
under (Zelda II peaks at 63), so one run checks both sides. check_render.py --raw 6 counts the
same keys (dict_keys.frame_overflows) and renders the reference with every lookup answering
"no rule"; the RTL counts overflowed frames (`fovf` on the paths line, required non-zero).
Negative control: neg_render_rtl.py "overflowed frame still looked up" (W2 without f_ovf).

Found by this test, and not by any earlier fixture: the sprite-list pass lost a sprite's
behind-background bit, so a behind-BG sprite drew in FRONT of the background. Cause:
find4() called inline inside the ternary of a non-blocking assignment (`plw_d <= c ? find4(...)
: 0`) - Verilator 5.036 evaluated it differently from the same expression through a blocking
temp (verified both ways at one pixel). find4 is the only function here that writes a local
through a variable bit index (`bgp[n[1:0]] = ...`) and calls another function; the other two
find4 call sites already assigned to a blocking local, and the fix makes the third do the same.
The normal fixtures could not see it: wherever such a sprite overlaps the background, the HD
tile drawn there covers both orderings. Only a frame drawn WITHOUT HD exposes the priority.
Control: neg_render_rtl.py "sprite list loses the behind-background bit".

### 8k - index tile conditions in a CHR-RAM game (2026-10-17)

A tile condition (tileAtPosition / spriteAtPosition / tileNearby / spriteNearby) normally names
its tile by the 16-byte data fingerprint in a CHR-RAM game. Given as an INDEX instead, Mesen
compares the target's TileIndex = (CHR linear address & 0xFFFFF) / 16 - CHR-RAM has a linear
address too - with no default-tile fallback (spec 3.5; hdref.tmatch: the target must have a
tile, and its palette must match unless the condition's ignore flag is set). hd3_resolve
already did this. hd3_render now does it too, instead of raising `unsup`:

- the target's TileIndex is read from the frame record (span qword A [19:4] for a BG target,
  sprite fetch qword A [35:20] for a sprite target), one burst through hd3_rcache with the new
  `byp` input: an uncached read that writes no data, tag or valid bit. The frame record is
  rewritten every frame while the condition cache is only flushed on pack load, so a cached
  read could return the previous frame's record; bypassing also keeps the record out of the
  rule lines. tb_cache.cpp covers byp with an exact cache model (expected hit/miss per read,
  one burst per byp read, no fill) and neg_cache.py has 5 byp mutants.
- tmatch's CHR-RAM branch compares that index (palette unless ignored) instead of the
  fingerprint.
- a nearby index condition is never "uniform": its target moves with the pixel, so it is
  evaluated pixel by pixel through the single-pixel target path (T1..T11), never the
  pipelined TP/SP passes (those carry no index). No group restart is needed: the per-pixel
  loop already reads each target, and an index read is not rewrite-sensitive.

No corpus pack has such a condition (every CHR-RAM pack uses data conditions), so the test makes
them: tools/sim3/make_ixram_cond.py converts Castlevania p002's 1,341 data tile conditions to
index form, giving each the TileIndex its fingerprint was seen at in the recorded frames (with
the condition's own palette), which hdref accepts as the same pack. Fixtures:
  p002_ix   plain; flipping every index changes 591-3,368 reference pixels per frame
  p002_ixh  --shift 3 --palmix: nearby offsets moved off the 8-pixel grid, some palettes mixed
  p002_ixf  --palflip: each matching index given a palette never seen with it (false
            everywhere unless the palette check is skipped - the only decisive palette test)
plus p002_ix with three frames in the SAME record bank back to back. test_render_all.sh runs all
four; neg_ix.py has 11 mutants, all caught.

Found by p002_ixh and not by p002_ix: a sprite run is not an i_group, so a nearby index
condition was evaluated once at the run's first pixel and applied to the whole run. On the
8-pixel grid every pixel of a sprite row reached the same target tile, so p002_ix could not see
it. Isolated by shifting one condition at a time (bisect_shift: condition 52, a spriteNearby
on an h-flipped sprite). Control: neg_ix.py "nearby index condition uniform over a sprite run".
A forced build that sends EVERY nearby condition (data form too) down the single-pixel path
also matches hdref (p002 with --shift 3 --keep-data), so that path is checked independently.

The one mutant that is not outcome-visible: "index read through the cache". The cache is
direct-mapped (256 lines) and a frame's rule traffic evicts the record lines before the next
frame, so no stale read reaches a pixel in these fixtures. REQUIRE_IXR=1 in run_ix.sh therefore
pins the design STRUCTURALLY (every frame must make uncached index reads; the tb's `ixr` count)
- a check on the mechanism, not the picture; the bypass behaviour itself is proven by neg_cache.

Cost (PROFILE build, p002_ix): frame 250 mean 9,474 clocks per line, worst 32,760, a backlog of
183 lines against the 5,456-clock line; frame 1000 mean 4,692, worst 28,198. Every target read
is one uncached DDR3 burst, and those frames make 44,544-93,200 of them for at most ~12,000
distinct (line, span/fetch) targets. No corpus pack has an index condition in a CHR-RAM game,
so this is correctness-first; the lever, if a pack ever needs it, is an exact 1,024-entry
target-index cache keyed by (kind, line, slot/fetch), cleared in the frame-start memo sweep
(OPEN_ITEMS item 7).

Closed 2026-10-17: the full render suite passes with the 8k changes in rtl/ and tools/, and every
negative control catches every mutant - neg_ix.py 11/11, neg_cache.py 14/14, neg_render_rtl.py
21/21 (all run against the promoted files, not the scratch copies).

### 8i/8j - backgrounds (planned and built 2026-10-17)

Corpus (scratchpad bg_survey.py, 110 packs): 74 packs have backgrounds; worst 35 levels, 20,489
entries in one level (30 of a pack's levels indexed), 51,556 background records, images up to
11,264 x 7,680 NES pixels. Background frame conditions: memoryCheckConstant 1.05M, tileAtPosition
199k, frameRange 195k, spriteAtPosition 11.7k. Blend modes: Alpha 667,511 records, Add 1,827,
Subtract 1 - all three are used. No pack has a rewrite-sensitive background condition. Per line
(a057 frame 420, the worst recorded): 28 selected backgrounds, at most 99 non-transparent cells
drawn, 14-35 distinct (block, row) pairs.

Reference order (hdref.render): backgrounds are selected BEFORE additions (PACK:300-313); per
level the first entry whose frame conditions all pass. Per line, scroll comes from the line's
first pixel (t, finex). Per pixel: backdrop, layer 0 (levels 0-9), sprites behind, layer 1, BG
tile, layer 2, sprites in front, layer 3; each background blended in level order with its mode
(blend_sub). Alpha is Mesen's integer form `pr + ((256-a)*o >> 8 & 0xFF) & 0xFF` (it WRAPS) and
Add/Subtract saturate, so a layer group cannot be pre-composited over an unknown base: each
background is blended in order, per pixel, inside compose.

8i - selection, in hd3_resolve, before list filtering (as built):
- Per level: read the level table entry {rule list, index}. Unindexed: entries in order, the
  first whose frame conditions all pass. Indexed: per signature, the masked snapshot values ->
  CRC-32 -> probe the table -> its candidate list. The plan merged the lists (k-way, ascending);
  the build instead sets each candidate's bit in a per-level BITMAP (1,024 words of {level + 1,
  32 entry bits}, so entries up to 32,767 against a corpus maximum of 20,488; a word whose tag is
  another level's reads as empty, so the bitmap is never cleared between levels) and then scans
  the touched words in ascending order. Same answer - the smallest passing index - with no
  per-list cursors, one RAM, and no limit on the number of lists. A candidate past 32,767 makes
  that level fall back to a linear scan of its entries (bg_lin, never seen in the corpus).
  Candidates use the frame-condition evaluator resolve already has; the payload is the
  background record index. Output per bank: 40 x {valid, record index}.
- The index CRC is the pack format's own (poly 0x04C11DB7, init 0, MSB first, no reflection,
  no final XOR - hdp3_convert.crc32, the same form as hd3_lookup's crc96), NOT zlib's CRC-32.
- Read port: render reads the selection of ITS frame's bank (g_bank), not the bank resolve is
  working on - in the stage-3 pipeline resolve already filters frame N+1 while render draws N.
  tb_resolve.cpp reads it with the other bank selected on purpose, and neg_resolve_rtl.py has a mutant
  that reads resolve's own bank.
- Mesen memoises a frame condition at first use and selection is that first use for any
  condition a background tests. Only rewrite-sensitive conditions can differ by time, and
  before any lookup nothing has been rewritten, so the value resolve computes IS the memo.
  Resolve writes each sensitive condition it evaluates during selection into a per-bank seed
  table (2,048 x {valid, value}); render's frame-start memo sweep (S_MCLR, already 2,048
  clocks) loads the seed instead of clearing. The seed stores the value BEFORE negation,
  because render's memo does and negates on read: the first build stored it negated and the
  sens fixtures caught it. (No corpus pack has a sensitive background condition, so
  make_sens_frame.py fixtures make them, as 8k did.)

8j - drawing, in hd3_render (as built):
- FRAME LOAD (at S_PAL, overlapping the 2,048-clock memo sweep, which waits for it): each
  level's selection through the read port above and, for a selected level, its 32-byte record
  over a new DDR3 read port B. Records go to an on-chip table (40 x 194 bits) with a per-level
  line-to-line cache beside it (sx, sy and the map row offset, each tagged with what it was
  computed for).
- LINE PREP (kicked at the end of the line load, running alongside the walks; compose waits
  for it): level by level, sx = ratio(hr, scroll_x) and sy = ratio(vr, scroll_y) from the
  line's first pixel (one burst each, skipped when the scroll is unchanged), the visibility
  test, the drawn pixel range clipped to the window, the map row offset (cached; +cols for the
  next row, else a 29-step shift-add), then the map words of the cells across the range (at
  most 33, up to 3 bursts). Per 8-pixel window group: cells A and B (pixel j is in A when
  phase + j < 8), an image slot per non-transparent cell from the line's slots - the same pool
  and fetch queue the tiles use, via a request the sequencer serves between walk steps; past
  NSLOT the cell is drawn uncached through the scratch slot exactly as an uncached tile - a
  per-line (block, row) -> slot map so each pair is fetched once, the group's mask byte
  (drawn pixels in range, each in a non-transparent cell) and the entry {mode, row, phase, B,
  A}. A level drawn on a line is marked dirty and its mask bytes are rewritten or cleared on
  the next line, so no byte from an earlier line survives; an unselected or invisible clean
  level costs one clock.
- COMPOSE: the plan grows from 10 to 50 steps per pixel: backdrop, levels 0-9, sprites behind,
  levels 10-19, BG tile, levels 20-29, sprites in front, levels 30-39. The front stage reads
  the pixel's group of mask bytes with the rest of its data. A background step reads the
  group entry, picks the cell, and draws the texel through the existing draw pipe with a mode:
  Alpha as blend_px, Add / Subtract per channel saturating. The entry of the NEXT step in the
  same pixel is read while the current one issues, so a background step costs one clock (two
  for a pixel's first; the first build took two for every step - Zelda II 420 backlog 5.6 lines
  - and this cut it to 0.7).

Test: check_render.py WITH backgrounds (run_render.sh's default is now --no-add only) over the
four recorded sets and every resolve fixture, plus make_bg_frame.py fixtures (a034_bg, p002_bg,
a057_bg: every fifth record Add, every fifth Subtract, every fifth moved 3 right and 5 down in
its map, and every ratio row rescaled to (old * 5) // 4 - 20 - 3r, so small scrolls are
negative and large ones run off the record). The NSLOT=4 build must take the uncached path for
background cells too. Negative control: neg_render_rtl.py's background mutants (blend mode
ignored, Add/Subtract swapped, cell B / cell phase / mask phase ignored, a dirty level not
cleared, each visibility and clip test, ratio and row-offset cache tags, map word half, slot map
line tag, uncached cell from slot 0) and the memo seed mutants; neg_resolve_rtl.py's selection
mutants.

Correction (2026-10-20): the paragraph above was written before those mutants had been run
against the final RTL, and three of them were NOT caught by the fixtures listed: "pixels past
the record's right edge drawn", "lines past the record's bottom drawn" and "vertical ratio
reused after the scroll changed". No recorded frame scrolls far enough to reach a full-size
record's right edge, and every recording keeps one vertical scroll for the whole picture. Two
fixture sources close them:
- make_bg_frame.py --edges (a057_bge, p002_bge; run_render.sh z2bge / cvbge): record i % 5 == 4
  is 56 px narrower and 48 lines shorter, so its right and bottom edges fall on screen. Measured
  with the hdref model: 5,568-6,224 px past a right edge on six frames and 32-2,501 lines past
  a bottom edge. a034's selected records are never in that class, so it has no edge pack.
- make_vscroll_rec.py (rec_mmmvs, rec_z2vs; run_render.sh mmmbgv / z2bgv): the line header's
  scroll Y moves by +24 on lines with y % 32 in 16..31. Only t changes (the tiles are recorded
  per pixel), so hdref and the RTL both see a legal frame; 36-42 vertical-ratio re-reads per
  frame on five of the eight frames.
Why the bottom mutant passed even though a057_bg already draws lines past full-size records'
bottoms (1,224 / 2,233 level-lines on frames 300 / 420): every cell those lines read past the
record's last row is empty (0 non-empty cells per frame, hdref model), so drawing them changes
nothing. On the shortened records the same rows hold real cells (5,600 per frame) - which is
what makes the edge pack decisive.
With them the unmutated suite passes and all three mutants are caught. The fourth earlier miss,
"prefetch tag without the level", is output-equivalent and is listed in neg_render_rtl.py's
EQUIVALENT table with the proof: every pixel's first compose step is the backdrop, which never
tests be_hit, and every issuing step writes the next step's {level, group} into be_tag, so the
level bits already match whenever a background step tests the tag. The full suite also passes
with that mutant, on frames with 49,088-55,900 two-level pixels. neg_render_rtl.py over the
final RTL: 54 of 54 run mutants caught, 1 equivalent. neg_resolve_rtl.py over the same RTL:
baseline passes, 50 of 50 caught (21 background-selection and memo-seed mutants among them).

### 8a - additions: plan (2026-10-17)

Reference (hdref.process_additions, spec 6.1). Before any pixel is drawn, line by line from 0:
pixel (0,0)'s record is replaced by a copy of line y's first pixel (Mesen's lineFirstPixel is
a reference to ScreenTiles[0]; the copy made for line 239 is what pixel (0,0) finally holds,
so pixel (0,0) is drawn and read by conditions with line 239's first-pixel data); BG tiles
are scanned only on lines where the tile row starts or the scroll changed, and on every line
after a line whose scan found a BG tile with additions; every pixel's sprites are scanned on
every line, including sprites appended earlier (a growing list). A record matches an addition
on its exact key, on its body when the addition ignores the palette, or (CHR-ROM) on its
fallback tile under its palette or ignoring it - each addition at most once, in file order.
Each match appends, at (x +/- ox, y +/- oy) (mirrored with the record), a copy of the record
with the added key and sci = 0, if that pixel has fewer than 4 sprites. Additions are inserted
after background selection and before every other condition is evaluated.

Measured on the recorded frames with additions (scratchpad add_hw_census.py; Castlevania p002
x5, Zelda II a057 x4): up to 2,048 sprites inserted a frame, 64 on one line, from 512 source
records; every source is a sprite (no BG tile source fires); no inserted sprite is itself a
source (no chain; the converter census found 0 chain pairs in the corpus); targets on earlier
pixels and lines are as common as later ones (offsets -32..+64 lines); the 4-sprite cap never
refuses an insertion on these frames; the (0,0) copy changes pixel (0,0) in 7 of 9 frames.

Model first (scratchpad add/addmodel.py, the 8a equivalent of bg_golden.py): it computes every
pixel's final sprite list WITHOUT Mesen's mutable walk - from an ordered set of contributions -
with its OWN table-based matcher (asserted equal to hdref.additions_for on every record), and
compares the result with hdref.process_additions field by field. 0 differences on all 9
recorded frames. Those frames cannot tell a right model from a wrong one, though: mutants of
the order rule and the cap also give 0 differences, matching the census (no competing
contributions, no refusals, no BG source). So make_add_frame.py rewrites a pack's addition
records in place for ONE frame to force every rule - two BG-tile sources 16 px apart (order),
four self-targeted additions on a sprite (cap), an ignore-palette and two fallback-keyed
additions, a source on line 239 aimed at pixel (0,0) - and requires the model to equal hdref
AND every mutant to fail: a057 frame 700 (7,296 sprites inserted; order, cap, copy, nobg,
nochain, noign, nofb all caught) and p002 frame 250 (25,024 inserted; the same minus nofb,
which a CHR-RAM game cannot have).

Two facts the model settled:
- Exact per-pixel order needs no per-pixel scan. Along two overlapping runs the source x
  differs by a constant, so a run's order key (source line, -dx, source slot k, addition
  index) is the same at every pixel it covers. Sort a line's runs by it and fill each pixel's
  list in that order, stopping at 4 including the captured sprites, and the result is Mesen's.
- Line 0 is affected too: Mesen copies the whole pixel record (lineFirstPixel), so for a pack
  with additions line 0 is drawn with line 239's line fields (scroll t and fine x, grayscale,
  emphasis) and pixel (0,0) is line 239's pixel 0 record. (Whenever the pack has additions,
  even on a frame where none fire.)

Sizes (model, accepted sprite-pixels after the cap / runs; an order-blind generator must store
every emitted run): recorded worst 2,048 sprites, 256 runs a frame, 8 runs on a line, 16
distinct added keys; dense synthetic 25,024 sprites, 3,128 runs, 52 emitted runs on a line.

Hardware (to build):
- Pack: section 16 = an addition-key table in the key table's exact format (64-byte buckets of
  four 16-byte entries, CRC-32 of {palette, body lo, body hi}), so hd3_lookup is reused as is
  (key_only). The palette 0xFFFFFFFF means "any palette" (ignore-palette additions); no real
  palette can be it (its low three bytes are colour indices <= 0x3F). Section 17 = the lists:
  {count, then per addition 3 qwords: {index 16, ox 16, oy 16, added kind}, added palette,
  added body}. (14 and 15 stay reserved for stages 5 and 6; the section table grows to 18
  entries, 0x80..0x10F of the 512-byte header.) Header 0x4C/0x50 = section 16's bucket count /
  max probe (0 = no table). The
  converter refuses a pack where an added key would itself match an addition under the exact,
  ignore-palette or fallback rules (a chain: Mesen would grow the list it is walking; the
  corpus has none).
- hd3_dict: a "has additions" bit per record, probed by its resolver when the pack has the
  table ((palette, body), (ANY, body), and in a CHR-ROM game the same two for the fallback
  index unless it is the index itself); an injection port that allocates and resolves an added
  key like a captured one; frame_done waits for the addition pass.
- hd3_add (new, vblank, after hd3_fmap's fill and the dictionary): lines 0..239 in order with
  Mesen's process_bg rule (scroll t from the line table, chained by a BG source that had
  additions). Per line it visits only the spans and fetches whose record has additions; their
  pixels (sprite rank k from the same find4 render uses; the BG slot from the pixel map) form
  source runs. For each run's additions (lists re-probed once per record, memoised), the
  target run is clipped to the screen, its added key injected for an id, and the run held in a
  per-source-line buffer; at the end of the line the buffer is sorted by (-dx, k, index) and
  appended to each target line's list in an on-chip pool (single bank like hd3_fmap: frame N's
  pool is read only by frame N's render). Run: {x0, length, offx0, offy, hm, vm, bgp, added
  id, source line, -dx, k, index}. Limits 4,096 runs a frame and 64 on a line; past either the
  frame is drawn without lookups (8o).
- hd3_render / hd3_resolve: a pixel's sprite list is its captured sprites (find4), then the
  runs on its line covering it in list order up to 4 in all; an added sprite has sci 0 (no SD
  colour, never the lowest behind-background sprite). The same merge serves spriteAtPosition
  (resolve) and spriteNearby (render) on any line. Pixel (0,0) and line 0's line fields read
  line 239's when the pack has additions; pixel (0,0)'s added list is line 239's runs at x 0
  from source lines < 239, then line 0's runs at x 0 from source line 239.
- Tests: check_render without --no-add on the recorded frames and both synthetic fixtures; a
  negative control with the model's seven mutants.

Storage scheme checked before any RTL (scratchpad add/addhw.py): it builds the per-line run
lists exactly as above (runs sorted per source line by (-dx, k, index), appended to the target
line, virtual line 240 for pixel (0,0)) and reconstructs every pixel's tile, sprite list and line
fields the way a reader would. Equal to hdref.process_additions on every pixel of p002 frames
250/450/700/1000/1300, a057 frames 300/420/700/1000 and both synthetic fixtures. Runs on a line:
recorded at most 8, synthetic at most 52 (p002_add line 16: 32 BG-tile runs of one fallback-keyed
addition plus 20 sprite runs), so the 64 limit holds every fixture. Pixel (0,0) gets 1-2 runs.
The section-16/17 tables (scratchpad add/add_tables.py) return the same matches as hdref's
matcher on every record of every recorded frame (check_add_tables.py).

### 8a - as built: pack sections 16/17, hd3_add (2026-10-17)

Pack. tools/hdp3_addtab.py builds sections 16/17 (format above) and tools/hdp3_convert.py calls
it (N_SECTIONS 18, header 0x4C/0x50); the converter refuses a chained pack. A fresh conversion
of Zelda II (a057) equals hdp3_addtab.patch of the old file in all 18 sections and both header
words, byte for byte. tools/hdp3_read.py checks the tables independently: for every key some
addition is filed under (from a plain scan of section 11), the hardware-style probe must return
exactly those additions, in file order, with their offsets and added keys; 0 problems on the
new a057, and tools/neg_read_addtab.py (6 damaged copies: an offset, an added body, a dropped
table entry, a short list count, a bad bucket count, a flipped ignore-palette flag) catches all
6. The 8 corpus packs with additions (a057, a058, a059, p002, p077, p079, test_cv, test_cv2) were
given their tables by hdp3_addtab.patch (sections 0-15 checked unchanged, the table check 0
problems each); the originals are kept in Z:/NES_HDPack_work/out/pre8a/. The other 129 have no
additions and are unchanged (zero at 0x100-0x10F and 0x4C/0x50 is what the converter writes).

hd3_loader. The section table grows to 18 entries (0x80-0x10F, checked as they stream past);
new exports add_en / akeys_base / abucket_mask / amax_probe / alists_base; new refusal 11 for an
inconsistent addition table: additions (0x40) only with section 16, section 16 = 64 bytes per
bucket with a power-of-two bucket count and section 17 present; no additions = no table and
0x4C/0x50 zero. A pack with additions but no table is a stale conversion and is refused rather
than drawn without its additions. tb_loader: the exports on every file, 5 new corrupted variants
(each refused with 11) and the last-entry check moved to section 17; make_loader_fixture.py now
also appends a 4-bucket table (its decoy 0x40 word is the addition count).

hd3_add (rtl/hdpack/hd3_add.sv), vblank, after hd3_dict's frame_done, per bank:
- clears every line list (0..240), then sweeps the bank's records (hd3_dict r_nrec, one a
  clock): if none has additions the pass ends there - most frames of most packs cost ~300
  clocks;
- per line it reads only the line header burst; the fetch area (sprite sources), the pixel
  pairs and the span area (BG sources) are read only when the line has a source;
- sources, runs, clipping, the (-dx, k, index) sort and the target-line append are as planned;
  a BG run never continues from column 7 to 0 (a run is at most one tile: the model splits
  where offx does not step by +1 - the first RTL merged runs across tiles through the 3-bit
  column wrap);
- limits: NL lanes on a target line (parameter; 64 holds every recorded and synthetic frame,
  the most is 52), NB = 64 from one source line, and time: `abort` (from the stage-3
  controller when RENDER needs the bank) stops issuing reads, waits for every outstanding burst
  to return (a late beat must not land in the next frame's plan) and ends. Past any limit `ovf`
  is set: the frame is drawn without lookups (8o).

Cost (tools/sim3/prof_add.sh, clocks a frame): recorded frames with additions 26-38 k at DDR3
latency 12, 82-85 k with 40% busy and latency 60 if each burst waited for the previous one, and
46-49 k with the same busy/latency when bursts overlap (--pipelined, what hd3_ddr's queue
does); frames without additions ~300. The synthetic dense fixtures (25,024 sprites inserted)
take 160-410 k and cannot fit vblank (~114 k clocks, shared with hd3_resolve's 25-47 k): there
the abort draws the frame without lookups. The recorded worst (49 k + 47 k) leaves ~18 k.

Tests (tools/sim3/run_add.sh, tb_add.cpp + check_add.py, every pixel of every frame against the
lane model, which equals hdref): the converted packs' own additions (a057, p002), the synthetic
fixtures, heavy merged lists, overlapped capture/ADD, a slow serial DDR3, aborts early and late
in a pass (the next frame of the same bank exact), the NL=16 build's required overflows, and
three new fixtures: make_add_edge.py (an edited a057 frame: a fifth covering sprite, hidden
sprite pixels, one tile in two capture slots under different palettes, a mid-row scroll change,
a vertically mirrored source, a clipped lane), make_add_srcline.py (one source line emitting 70
lanes spread over 70 target lines, so only the source-line limit can flag it) and a same-bank
sequence where each frame with additions is followed by one with none (that frame skips the
line-240 pass, so only the clear empties line 240). 16 suites, 0 differences. Negative control
tools/sim3/neg_add_rtl.py: 29 of 29 mutants caught; the first run missed 10, each a fixture
gap now covered by the fixtures above. One further mutant (the addition index left out of the
sort key) is output-equivalent and documented there instead of run: equal (-dx, k) keys from
different sources cover disjoint pixels, and one source's lanes come out in its merged-list
order, which hd3_dict builds in file order.

Reading the lanes (hd3_resolve, hd3_render; 2026-10-17). Both take a lane reader (a_y in, a_n /
a_q one clock later, NL lanes of RAW+21 bits) and an add_en input; with add_en low nothing
changes. hd3_render:
- Pixel lists. A line's lanes (and, for pixel (0,0) of line 0, line 240's) are appended to each
  pixel's sprite list after the captured sprites, up to 4 entries, whether or not sprites are
  shown, exactly hdref's order. A list entry is {kind, idx}: kind 0 a fetch (idx 16..31 = line
  239's fetches), 1 a lane of this line, 2 a lane of line 240. An added entry never sets
  "lowest" and copies its source's flips, palette and background priority.
- Pixel (0,0) on line 0 is line 239's pixel 0 (its pixel word, span, sprite fetches and palette
  RAM) plus line 240's lanes; line 0's header t / fine x / grey / emphasis come from line 239
  (span and fetch counts stay line 0's). The line load reads a second burst set for this
  (bursts 30..44: line 239's pixel burst 0 and its header block); its span entry goes to slot
  62. Lookup state (the window, "seen", rewrites) keeps the original coordinates.
- Added sprites are drawn by the same sprite-run machinery: after the fetch runs, S_LF0 walks
  the lanes of the line (then line 240's) as items {x0, length, column of x0, key id, lane};
  an item's column is its source column plus (x - x0).
- Conditions: spriteAtPosition and spriteNearby build their target lists with the same merge,
  so a condition sees added sprites; a target at (0,0) reads line 239 (dline), a lane lookup
  uses the line-240 lanes when its line is 0 (lline). An added NOTILE / CHR-RAM key never
  matches (tmatch with the lane flag).
- Overflow (hd3_add ovf, NL lanes on a line or 64 from one source line) is ORed into the
  frame-overflow input: the frame is drawn without lookups, like a dictionary overflow.

Test (tools/sim3/run_render.sh, adds()): hd3_render_tb runs dict -> add -> resolve -> render,
and check_render.py includes additions by default, deciding lane overflow itself with the
addhw model (--nl gives the build's NL), never from the testbench. The recorded frames' own
additions do not change any picture (the added keys find no HD payload, or a transparent one),
so make_add_draw.sh writes two fixtures (a057 frame 700, p002 frame 250, the make_add_frame.py
rules with --draw: added keys whose own record finds an opaque payload). Results: the 16 base
frames, z2d / cvd and the five correctness fixtures pixel-exact on NL=16 and NL=64 builds;
z2d / cvd decisive (the no-addition reference differs by 312 to 37,216 pixels a frame), z2mix /
cvmix decisive by overflow (over 64 lanes on a line). The dictionary overflow the reference
models (dict_keys.frame_overflows_add) counts the added keys too: hd3_dict gives each one an
id, deduplicated on the full {palette, body}, and also overflows a frame when one record's
merged additions exceed its 64-entry buffer or the bank's pool exceeds 1,024 entries. Found by
the RAW=6 build: Zelda II frames 700 / 1000 hold 63 record keys, under 64, but their added keys
push the ids over, and the testbench rightly drew them without lookups while a record-only
count expected HD. check_render caches reference renders
(obj/refcache, keyed on the inputs and the reference's source).

Fixtures added because the first negative-control run missed mutants (each is a gap the
recorded frames cannot reach, not a guess):
- z2dh / cvdh (make_hidden_rec.py: sprites hidden on the left half of every line). The recorded
  frames always show sprites where an added sprite lands, so "lanes appended only where sprites
  are shown" passed them; with the left half hidden it fails 3 of 4 frames.
- cvl0 (make_line0_rec.py: line 0 greyed, pixel (0,0) given its own value, p002_adddnb.hdp with
  no backgrounds). Line 0's header and pixel (0,0) come from line 239 in hdref; on the recorded
  frames lines 0 and 239 agree, so nothing could tell them apart.
- lane / lane_nz (make_lane_pack.py on rec_mmml0, Metroid frame 880 with the left half hidden
  and line 0 greyed): a pack written for the lanes. 12 keys and 6 conditions: an added sprite
  behind the background, placed by a flip-aware offset search so 64 of its pixels land on opaque
  background in the hidden half (the "background priority dropped" mutant changes 256 pixels in
  the model and fails on the RTL); spriteAtPosition naming an added sprite, and the same with a
  palette it does not have (sapx); spriteNearby over an added sprite (nzd); a NOTILE added
  sprite; an addition onto line 240 and so onto pixel (0,0). lane_nz is the same pack without
  the line-240 lanes, which is what decides "pixel (0,0) not taken from line 239" (on lane.hdp
  line 240's lanes cover the pixel either way).
- RS1 (NL=64, RS_MAX=1): every rule list unfiltered, so render's own condition walk, not
  resolve's filter, decides the frame conditions. The spriteAtPosition and (0,0)-target mutants
  live in that walk and are decided only there; run_render.sh runs lane / lane_nz on it
  (RS1_EXE) and checks that the unfiltered path was taken.
All pass on the real NL=16, NL=64 and RS1 builds. Negative control on the lane mutants: 12 of 15
on lane.hdp, the remaining 3 on lane_nz (pixel (0,0) from line 239) and on RS1
(spriteAtPosition targets, a target at (0,0)): 15 of 15. neg_render_rtl.py now builds RS1 for
every "add" mutant.

Still owed for 8a: the stage-3 controller's abort (integration, item 10).

### hd3_fmap: snooping the capture stream, and its unit test (2026-10-18)

hd3_fmap no longer reads the finished record (100,802 clocks a frame, see 8m below). It takes
the capture writer's record stream - the same tap hd3_dict takes, plus the byte enables - and
keeps two banks, one per record buffer; RENDER reads the bank of the frame it draws (r_bank).
What it stores per word:
- pixel map: byte 0 of each pixel ({sprites shown, BG slot}); a pixel qword holds two pixels and
  the writer stores them with per-pixel byte enables (1111, 0001 for a branch-B pixel, 0000 for
  a missing even pixel), so each half has its own write enable, be[0] / be[4];
- line table: from header q0 (written whole or with bytes 3-7 only - bytes 4 and 6, the span
  and fetch counts, are always written), capped at 34 spans / 16 fetches as hd3_dict caps;
- sprite table: qword A of fetches 0..15 (line-block qword 80 + 2i), 16 lanes so one line reads
  in one clock.
A word the writer does not store (a sprite slot past the line's fetch count) is stale in the map
exactly as it is stale in DDR3, and RENDER reads only up to the fetch count. Writes outside the
two buffers, in the gap between the pixel area and the line blocks, to line blocks 240-255 or
past the record are ignored. Bank 1's pixel map starts at entry 32,768, so each half is 65,536
entries (a 61,440-entry array would lose bank 1's lines 224-239 - a mutant sized that way is
caught).

Unit test (tools/sim3/run_fmap.sh, gen_fmap_stream.py + tb_fmap.cpp): gen_fmap_stream.py writes
a byte model of the two 1 MB buffers and a write stream: a full write of both banks (a
Castlevania and an SMB3 record), then partial rewrites of bank 0, bank 1, bank 0 with random
pixel byte enables (0000 / 0001 / 1111 / random), header q0 with enables F8 or FF, the boundary
lines (fetch / span counts 16/34, 17/35, 0/0, 255/255, 15/33), random counts that leave stale
slots, rewrites of the same word, the trailer, junk writes (below the buffers, 2 MB and more
above, the pixel/line gap, line blocks 240-255, past the record) and idle clocks. tb_fmap.cpp
drives the stream, moves the read ports at random during the writes, and at each dump point
reads every entry of both banks; the check compares each against the byte model. Two seeds,
about 335,000 writes and 4 dumps each: pass. Negative control neg_fmap.py: 20 mutants (tap_v
ignored, each byte-enable error, halves swapped, the shown bit lost, the pixel bank ignored, the
61,440 sizing, the pixel area cut at line 224, the record range or the line-block bound not
checked, either cap missing, the header from the wrong qword, the line bank, qword B stored, the
sprite area offset, the lane from the wrong fetch bits, bgp from the wrong attribute bit, the
sprite bank, the read bank) - 20 of 20.

### Frame budget and the dictionary read port (8m, measured 2026-10-17)

When this was measured the testbench ran ADD, RESOLVE, fmap and RENDER one after the other (it
no longer has an fmap step - see the decision below); in the engine, ADD and
RESOLVE prepare frame N while RENDER is still drawing frame N-1, and all three read hd3_dict
through its single read port (each dictionary memory has one write port, the capture/injection
side, and one read port). Measured per frame with a copy of tb_render.cpp that times each step
(PROFILE=2 build, design NL=16, scratchpad budget/run_budget.sh), at 85.9 MHz / 5,456 clocks a
NES line:

  set            add      resolve   fmap     render tail at frame end
  SMB 240-300    243      ≤ 8,095   100,802  0
  Metroid        243      ≤ 52,355  100,802  0
  Castlevania    ≤ 35,050 ≤ 26,384  100,802  0
  Zelda II       ≤ 38,335 ≤ 53,302  100,802  0
  bg fixtures    243      ≤ 52,355  100,802  0
  z2d / cvd      up to 409,459 (synthetic lane stress) ...

The window: capture of frame N ends at NES line 240; RENDER needs frame N's line 0 when the
display reaches it, about 22 NES lines later - roughly 120,000 clocks. RENDER N-1 has caught up
by then (its backlog at the end of every measured frame is 0; the worst mid-frame backlog is 3.5
lines, inside the 4-line output ring). fmap holds ONE bank, so it can only fill once RENDER N-1
is done. Serially, ADD + RESOLVE + fmap reaches 192,439 on Zelda II 700 - over the window. fmap
alone is 100,802 on every frame (a fixed 5,040-burst copy of the finished record).

Decision: fmap leaves the vblank chain. It snoops the same record writes hd3_dict takes during
CAPTURE and keeps two banks (render reads its own frame's bank), so its fill costs nothing in
vblank and no DDR3 bursts. The vblank chain is then ADD + RESOLVE: 91,637 at worst on the
recorded frames, about 76 % of the window, with the dictionary read port owned by one client
at a time - ADD, then RESOLVE, and RENDER otherwise - exactly the ownership the testbench
already uses, so no second read port is needed. A frame whose ADD would not finish in time is
the stage-3 controller's abort (hd3_add ovf: drawn without lookups), which the synthetic lane
fixtures exercise. Cost: the second fmap bank. Counted by M10K shape (a block is 8192 x 1 for
the pixel maps, 512 x 20 for the 26-bit sprite lanes): pixel maps 2 x 65,536 x 7 = 112 blocks
(one bank was 56), sprite table 16 lanes x 512 x 26 = 32 blocks (one bank: 16, 256 x 40), line
table 1 - about 72 M10K more than the single-bank fmap, of 553 (the last full compile used 30 %
of the block-memory bits). The compile is what settles it. Built and tested as described under
"hd3_fmap: snooping the capture stream". Adopted 2026-10-18: rtl/hdpack/hd3_fmap.sv is the
snooping version (no start / bank / read port any more); hd3_render_tb feeds it the record tap
with byte enables and RENDER's bank, and tb_render.cpp streams each line's 128 pixel qwords
with the headers, spans, sprites and trailer instead of filling the maps in a separate step.
The full render suite and test_render_all.sh pass on it.

Replaces hd_ddr.sv when the MVP goes. Clients: savestates (ddram.sv's interface, unchanged,
highest), pack-load writes, frame-record writes (byte enables, valid/ready) and NR read clients
(hd3_dict, hd3_resolve, hd3_render), each with hd3_lookup's handshake: req held until a
one-clock ack, then 8 beats in order, several bursts outstanding. Returns are routed through
an 8-entry queue of {savestate, client, beats}; one entry is kept free for a savestate read.
Order: savestates > pack-load writes > the capture writer and the readers, alternating when
both wait; the readers take turns among themselves through their own pointer.

The unit test found a starvation bug in the first version, which had ONE round-robin pointer
over {cw, readers}: a cw grant while the return queue was full (readers not eligible) moved the
pointer back to reader 0, so reader 0 was served after every such grant and readers 1-2 waited
for up to 12 other grants. With the separate pointer a reader waits for at most NR - 1 other
readers and the capture writer for at most one reader.

Test: tools/sim3/tb_ddr3.cpp, a DDR3 model with random DDRAM_BUSY, read latency and gaps, all
clients at once with random traffic plus a phase where the three readers request without
pause. Checks: every beat to the right client, in order, with the data at its address; every
write lands with exactly its byte enables; savestate data; every access completes; a reader is
passed over by at most NR - 1 readers and the capture writer by at most one; the writer keeps
moving during the saturated phase. Five seeds (busy 10-50%, latency 12-60): about 1 M checks
each, 0 failures. Negative control tools/sim3/neg_ddr3.py: 8 mutants (pointer never advances,
cw never gets its turn, beat to the wrong reader, savestate beats misrouted, byte enables
ignored, queue room ignored, 16-beat bursts, write acknowledged but not issued) - 8 of 8.
The "cw never gets its turn" mutant was first MISSED: the readers often stall on a full queue,
which lets the writer through anyway, so the clock-count limit never tripped; the test now
counts reader grants while the writer waits.
Not done yet (stage-3 integration): it is not instantiated; the engine's move to clk85 needs
the savestate channel crossed into that clock.


## Stage 5 - ROM patches as built (2026-10-18)

### What the hardware does

The ROM is patched the way Mesen does it: the pack lists `<patch>file,SHA1`; when the SHA1 of
the WHOLE ROM file (16-byte header included) matches, the patched ROM replaces the original.
Since 2026-10-22 the core also tries the file's iNES 1.0 form when the file itself names no
entry - see "Beyond Mesen" below. A patch can change the cart type (Metroid's `mmm.ips` adds CHR-ROM),
so the patched ROM has to reach the core before it picks its mapper. `rtl/hdpack/hd3_romfix.sv`
does that by REPLAYING the patched file into the core as a second download:

1. Every NES ROM download passes to the core unchanged and, alongside, is copied into DDR3
   (staging area, byte 0xC800000, up to 4 MB) and hashed (`hd3_sha1.sv`, byte-serial SHA-1,
   one round a clock). hps_io is held off (`ioctl_wait`) only while the 16-byte FIFO in front
   of the hash is half full.
2. When the download ends the core is held in reset, the hash is finished and section 14 is
   searched. No entry -> released; the core already holds the unpatched ROM.
3. On a match the patched ROM is built in DDR3 (byte 0xCC00000) from the entry's run list, its
   CRC-32 checked for BPS/UPS, and replayed: downloading rises, every byte goes to the core's
   loader port (waiting on the core's own `ioctl_wait`), downloading falls. The core cannot tell
   it from the OSD loading the patched file.
4. A pack loaded or unloaded AFTER the ROM is acted on once no download is running: a match is
   built and replayed; if the core holds a patched ROM and the new pack has no usable entry,
   the staged original is replayed. A pack without an entry for the running game never resets
   it (the core is held only while a replay runs).
5. A target that cannot be built (size or bounds wrong, unknown run mode, a TargetCopy reading
   ahead of itself, CRC mismatch) is not used: the ROM stays (or goes back to) unpatched, as
   Mesen leaves it when its patcher refuses. `rom_result` says why: 1 patched, 2 no match,
   3 refused, 4 CRC, 5 ROM too big.

DDR3 is reached through the savestate crossing (single access), shared with the savestate
engine through a clock-domain arbiter in `hd3_engine` (savestates first).

The converter compiles every `<patch>` to section 14 (`tools/hdp3_patch.py`): IPS records are
resolved into non-overlapping literal/fill runs (later records win), BPS into
source/literal/target-copy runs with its CRCs, UPS into XOR runs over the source. The hardware
only follows runs; it never parses a patch format.

### Beyond Mesen: which ROM a `<patch>` applies to (2026-10-22)

Decided by the user ("full compatibility for all 136"). Mesen patches only when the SHA1 of the
whole file is on the line. Pack authors hashed iNES 1.0 dumps; the library (No-Intro) carries
NES 2.0 headers, so with the ROM most people have, Mesen never applies the author's patch.
Measured over every `<patch>`/`<supportedRom>` hash in the corpus against every library ROM:
113 match only the file with header bytes 7-15 zero, 38 match the file as it is, and none
needs byte 7's upper nibble kept - zeroing 7-15 is the whole rule. (Correction 2026-10-29: that
count covered only hashes some library ROM already matched. Re-measured over all 71 distinct
pack hashes against NES + NES_Private: exactly ONE needs the true iNES 1.0 form with byte 7's
mapper nibble kept - MD5 caa299d1, the Gryzor packs, VRC2 mapper 23. Handled pack-side in
`tools/corpus/make_audio_patch_overlays.py`: the line names the library file's own SHA1, which is
safe because that patch is an IPS writing only past the header. The core rule stays as is.) `tools/rommatch.py` holds
it, and every tool uses it (converter, `hdp3_read.py --rom`, `sim3/rom_expect.py`, the corpus
planner):

1. The file's own SHA1 first, then (only if the file has `NES\x1a` and a non-zero byte in
   7-15) the SHA1 of its iNES 1.0 form - the same file with 7-15 zero.
2. The patch is built from the form that matched. A BPS checks its source's CRC, so building
   an iNES-1.0-keyed BPS from the NES 2.0 file fails; built from the matched form it succeeds.
3. MD5 lines (`<patch>file,<32 hex>`; a037 Ninja Gaiden, p099 Life Force), which Mesen rejects
   as an invalid SHA1, are resolved through `tools/corpus/romindex.tsv` to the SHA1 of the
   library form they name, and keyed by that. A SHA1 line wins a clash.
4. `tools/pack_fixes.py`: a `<patch>` line naming a file the pack does not ship is mapped to
   the file it does ship, checked by MD5 so it applies to that one pack only (a030 names
   `Mega Man.ips`, ships `Megaman - Super.ips`; a037 names `Ninja Gaiden.ips`, ships
   `Ninja Gaiden HD (U) [Hacks].ips`). The pack model still logs Mesen's own error for the
   line, and reports the fix in `Pack.compat`.

The core (`hd3_romfix.sv`): when the file's hash names no entry and the staged file has an iNES
header with something in 7-15, it streams the staged copy back from DDR3 through the same
SHA-1 unit with those nine bytes read as 0 (`S_N0`..`S_NW`, one qword read per 8 bytes; no
second hash unit, the FPGA is at 102% ALMs) and searches section 14 again. On a match the build
runs with `p2` set: a source byte 7-15 reads as 0 (`f_zero`) and the base fill writes byte 7 of
qword 0 and all of qword 1 as 0. The staged original is never changed, so a later pack without
an entry still replays the file exactly as it was loaded. The second hash is computed once per
ROM and kept (`sha_n`), so a later pack swap does not stream it again.

Tests: `tools/test_rommatch.py` (forms, order, MD5 resolution, a BPS needing the iNES 1.0
form, the a030/a037 fixes on the real packs - ALL PASS); `test_hdp3_patch.py` now covers 104
corpus lines (71 through the iNES 1.0 form or an MD5), all identical to `rompatch.apply`;
`romfix_cases.py` adds seven cases from `sim3/make_ines1.py` (IPS, BPS, UPS XOR over the zeroed
header, the file's own line beating its iNES 1.0 line, a CRC refusal, a file without the magic,
and a pack swap that brings the ORIGINAL back) - 14 of 14 runs pass; the whole list, 27 cases /
54 runs, passes on the final RTL. `neg_romfix.py` gains 8 mutants of the second pass (no second
pass, header hashed raw, byte 15 not zeroed, magic unchecked, last partial qword overrun, build
reading the raw header, base fill writing the raw header, byte 7 kept) and two re-anchored
ones: 10 of 10 caught, each by an ines1 case except past_source_not_zero (synth6) - its new
`--first ines1` option tries those cases first. test_sha1 416 digests 0 wrong (hd3_sha1.sv
unchanged; its build is newer than the source).

### Tests

- `tools/test_hdp3_patch.py`: the section-14 compiler and its hardware model against
  `rompatch.apply`, byte for byte - 21 corpus `<patch>` lines with their library ROM (all 21
  applied and identical), 300 synthetic IPS/BPS/UPS, the refusals, multi-entry sections.
  Negative control (`tools/neg_patch.py`): 13 of 13 mutants caught.
- `hdp3_read.py --rom` decodes section 14 independently and checks the hardware build against
  `rompatch.apply` (Urban Champion: 0 problems; one flipped data byte: 2 problems).
- `hd3_sha1.sv`: 416 digests (lengths 0..262160, two real ROMs) against hashlib, 0 wrong;
  9 of 9 mutants caught.
- `hd3_romfix.sv` inside the full engine (`tb_engine --rom`, `tools/sim3/romfix_cases.py`): 20
  cases (19 until 2026-10-20, see below), each with the ROM loaded after the pack and before it
  (40 runs), all with random
  savestate traffic on the shared crossing, DDR3 busy 20%, and a core loader that is busy 2-20
  clocks after every byte (longer than the replay's own gap). The testbench records every download the core sees and checks:
  the first is the OSD file unchanged, the last equals the expected ROM (`rompatch.apply`, or
  the original), the number of loads, `rom_result`, that the core stayed in reset between the
  download and the replay, and that no replayed byte was written while the core was busy. The
  staging area starts with garbage, as real DDR3 does. Cases: Urban Champion's corpus BPS
  (patched / another ROM / a damaged pack -> CRC refusal); six synthetic ROMs (IPS with RLE,
  overlaps and growth; IPS growing then truncated; UPS growing; UPS shrinking; BPS with
  overlapping target copies; a ROM size not a multiple of 8, patched and later replayed as
  the original; UPS writing into the grown part, i.e. XOR against the source past its end);
  a ROM with no entry among decoy entries; six malformed sections (unknown run mode, a target
  copy reading ahead, a run count past the section, valid runs placed past the section, literal
  data past the entry's data, a recorded source size that disagrees) -> refused; a pack swap
  to a pack without an entry (the original is replayed) and to a damaged one (CRC -> original
  back); and since 2026-10-20 `uc_ok_backpressure` (below). 40 of 40 pass. (Before that
  case this read "20 cases, 40 of 40", which overcounted: the result file of that run has 19
  cases, 38 runs, all passing.)
- Negative control (`tools/sim3/neg_romfix.py`): 32 mutants of hd3_romfix.sv, each built into
  the engine and run until a case fails. First pass 26 of 32; the six misses each named a hole
  in the cases (no wrong-size source, a run list refused for another reason, no XOR past the
  source, a core never busy longer than the replay's gap, no odd-size ROM whose tail is used),
  closed by the cases above: 31 of 32. The one left, dropping the target-cache invalidation
  on a combine flush, cannot change a section the converter writes: runs are in target order
  and BPS writes the target sequentially, so a target qword is in the combine register from
  its first byte until flushed and is never read from DDR3 before that. (A hand-made section
  could exploit it only to produce a wrong ROM, and BPS's CRC refuses that.)
  Re-run from the repo 2026-10-20: 30 of 32. `no_stall` (the download never held back) had been
  caught only by chance: the 16-entry staging FIFO filled on `uc_ok` when the random DDR3-busy
  sequence happened to line up, and the repo testbench draws extra random numbers (the replay's
  core-wait), which moved the sequence so the FIFO never filled. Fixed by a case that forces it:
  `uc_ok_backpressure` (`--busy-burst 200`, DDR3 busy 200 clk_f at a time) - the good build
  passes it in both orders, `no_stall` fails it with result 5 (the FIFO overflow), so the case
  list is now 20 cases / 40 runs (40 of 40 pass) and this catch no longer depends on the
  random sequence.
- `hd3_loader`'s section-14 outputs (`patch_base`, `patch_len`) were not declared in the loader
  unit testbench, so the loader's own negative control could not even build against the current
  RTL. `tb_loader` now checks both against header 0xF0/0xF4, with two new mutants (base not
  offset by the pack's DDR3 base; length dropped); `neg_loader.py` refuses a pack set without a
  patched pack (the romfix fixture `uc.hdp`). Result 2026-10-19: 34 of 34 caught. Four mutants need particular
  header values - the two section-10 mutants reuse an existing case label, so the register keeps
  its reset value 0 and only a non-zero fallback map shows them (smallest: a058, 62 MB, ~11
  minutes a run); `legacy` needs flag bits 0/1 different (p132); `render_orig` needs option bits
  3/4 different (a058). On the small pack set they are reported NOT OBSERVABLE (exit 3) instead of
  missed; they were caught on p132 / a058. The unmutated testbench passes all seven packs (295
  checks, 0 failures).
- Found by these tests: an IPS patch that both extends past the ROM and truncates made the
  size step apply its two rules on alternate clocks forever; the size is now computed in one
  step.

Promoted 2026-10-20: hd3_romfix / hd3_sha1 and the NES.sv glue are in rtl/hdpack and NES.sv;
the tests run from tools/sim3 (romfix_cases.py 38 of 38 at promotion, 40 of 40 with the
back-pressure case added the same day, test_sha1.py 416 digests 0 wrong -
the two library ROMs are now part of its default set, they had been passed by hand). Fixtures
in tools/sim3/fixtures/romfix. Still to run: the whole-core check (fullsim: Metroid with
mmm.ips, which changes the cart type, against Mesen) - the fullsim build list now includes
the stage-3 engine.

### Since 2026-10-28: the decision runs on the ARM (OPEN_ITEMS item 11)

Owner decision (item 11, the FPGA was at 102% of its ALMs): the hashing, the section-14
search and the build moved to the ARM (`tools/armdec/romfix_arm.c`, linked into
`hdarm_daemon.c`'s service thread on core 1, and into `tb_engine` and `fullsim` so the tests run
the code the device runs). `hd3_romfix.sv` keeps what the ARM cannot do: stage every NES ROM
download into DDR3 (window byte 0xC800000, 4 MB) while it passes to the core, hold the core in
reset from the end of a download until the ARM answers (only with a section-14 pack active), and
replay the ROM the ARM asks for (the patched one it built at 0xCC00000, or the staged
original). `hd3_sha1.sv` is no longer in the core.

Handshake, two qwords beside the mailbox (field layout at the top of hd3_romfix.sv and in
romfix_arm.h):

    status  0xC400040  written by the FPGA: {0xA6, rseq, aseq, last command id, active,
                       section 14 present, content, staged, ROM length}
    command 0xC400048  written by the ARM after the target: {0xC7, rseq, aseq, id, action
                       (0 none, 1 replay target, 2 replay original), result, target length}

A command is carried out once and only while its rseq/aseq are the status's, so an answer to an
old question is ignored. A download with no answer within 2^25 core clocks (~1.56 s NTSC,
~1.26 s PAL; the count pauses while a DDR3 access is in flight) runs unpatched (result 6) and a
later command can still replay the patched ROM. A "replay the target" command whose length is
0 or past the 4 MB target area is refused (result 3) - romfix_arm.c never writes one. The rules
of "Beyond Mesen" (the iNES 1.0 form, MD5 lines) are the ARM's now; the run lists and their
refusals are unchanged.

A core reload keeps DDR3, so the last session's status and command are still there when the
new core starts, and both sessions count rseq from the same place. At power-up the FPGA reads
the old status, starts its rseq 0x40 past the old one, and only then writes both qwords 0. So
neither a command left in DDR3 nor an answer the daemon was still working out for the old
status (it may finish after the new session has asked) can be taken for a new question. The
cleared command also tells the daemon a new session began (`rf_session_check`, which forgets
the old session), and the daemon re-reads the status after writing a command and withdraws the
command if the status has been cleared meanwhile. The FPGA also never mixes up two memory
answers: a download that starts while an access is in flight swallows that access's answer.

Tests: `test_romfix_arm.py` (rf_build against rompatch.apply on the corpus lines and the
synthetic set, the bounds refusals); `romfix_cases.py` on tb_engine with romfix_arm linked in
(every case of the list above, plus `arm_silent`: the ARM never answers, the core runs the
original after the timeout - tb_engine is built with `HD3_SIM_TMO` = 2^20 clocks, since 2^25
takes over half an hour of simulation; the RBF keeps 2^25; `uc_ok_stale` / `uc_ok_stale2`: DDR3 holds a previous session's
command at power-up, after a first and after a second reload; `uc_badlen` / `uc_badlen0`: a
target length past the area or 0; `swap_midaccess`: a pack download starts while the patcher's
poll read is in flight and that answer is held back past the download); `neg_romfix_arm.py`
(mutants of romfix_arm.c, including the record budget and the session check) and
`neg_romfix.py` (the FPGA half's mutants, including the power-up rseq jump, the old-status read
and the length refusal). The abandoned-access `drop` has no killing mutant: removing it shifts
the answers by one access, and the shift heals at the next idle answer before any read whose
data matters (measured with swap_midaccess; recorded in neg_romfix.py). Results in OPEN_ITEMS
item 12.

## Stage 6 - audio (design, 2026-10-21)

### What Mesen does (read from Mesen2 source, commit b9fa69d)

The spec (section 8) marked the decoder, resampler and mix point INFERRED because Mesen's
`Utilities/` and `Shared/Audio/` were not in the extracted tree. They are now read
(`tools/audio/mesen/`):
- OggReader decodes with stb_vorbis v1.22, always 2-channel interleaved s16, and resamples
  with `Utilities/Audio/HermiteResampler.cpp` (4-point Hermite in double, output clamped to
  s16) from the file's rate to the host rate. Volume is `sample * vol / 255` in int
  (truncating toward zero), added with a clamp to s16 after each source.
- SoundMixer::PlayAudioBuffer resamples the NES mix to the host rate first, then calls every
  audio provider's MixAudio on that buffer, then the equalizer / reverb / crossfeed / master
  volume. So HD audio is added to the NES signal after Mesen's own NES filtering (blip_buf,
  which has a built-in high-pass, so the NES signal is DC-free).
- Mixing order inside OggMixer::MixAudio: BGM (if present and not paused), then each SFX in the
  order they were started; finished SFX are erased after the call.
- Looping: when the stream ends inside a mix call, stb_vorbis_seek to LOOPPOSITION and keep
  feeding the SAME resampler (no reset), so the history carries over the seam.

Corpus (85 hires.txt with `<bgm>`/`<sfx>` lines, scratchpad audio_survey2.py, each file
counted once): 49,124 s of audio in all. The largest pack is Dragon Quest 3 Delocalized
(a064, 56 BGM, 5,766 s, no tiles); next Metroid JUD6MENT (a034, 4,243 s), Zelda 2,787 s.
Sample rates 22,050 / 32,000 / 44,100 / 48,000, mono and stereo.

### Storage: 48 kHz IMA ADPCM, stereo, one byte per frame

Decided on 2026-10-13 ("audio is stored decoded (ADPCM)"); the rate and layout are fixed here.
The converter decodes each OGG with Mesen's own stb_vorbis and resamples to 48,000 Hz with
Mesen's own Hermite resampler (`tools/audio/oggdecode.exe`, built from those files), then
encodes IMA ADPCM, 4 bits per channel, low nibble left, high nibble right. 48 kB per second:
a064 is 277 MB, a034 204 MB. The hardware then needs no resampler and no multiplier for
decoding, only a step table.

48 kHz is the MiSTer audio rate (audio_out's 48 kHz mode); storing at the file's own rate
would save 8 % on 44.1 kHz files but needs a resampler in the FPGA.

### Memory: the pack window plus an overflow region

The `.hdp` file is loaded verbatim as before. Byte offset f < 192 MB goes to 0x30000000 + f;
f >= 192 MB goes to 0x24000000 + (f - 192 MB). Audio is the last section of the file, so it
is the only section that can cross 192 MB; the converter refuses a pack whose other sections
end past 192 MB, or whose total is over 384 MB. a064 (277 MB of audio, no images) fits, as
does a034 (75 MB of images + 204 MB of audio).

### Section 15 layout

    0x00  char[4]  "HDAU"
    0x04  u32      entry count N (<= 512)
    0x08  u32      data offset (inside the section, multiple of 64)
    0x0C  u32      checkpoint-table offset inside the section (0 before 2026-10-29 = none)
    0x10  N x 32-byte entries, sorted by key:
          u32 key       [15:0] id = album*256 + track, [16] 1 = SFX, 0 = BGM
          u32 start     byte offset of the stream inside the section (multiple of 8)
          u32 frames    48 kHz frames in the stream (one byte each)
          u32 loop      48 kHz frame where a loop restarts (0 = from the start)
          u64 state0    ADPCM state at frame 0: [15:0] pred L, [22:16] index L,
                        [47:32] pred R, [54:48] index R
          u64 stateL    ADPCM state at frame `loop`, same layout
    data: the streams, each starting on an 8-byte boundary
    checkpoint table (8-byte aligned, after the streams):
          u32 spacing   16384 (hdaudio.CK_SPACING = aud_arm.h AUD_CK)
          u32 count     sum over the entries of frames / spacing + 1
          count x u64   for each entry in table order, the ADPCM state before frame
                        k * spacing, k = 0 .. frames / spacing (k = 0 is state0), same layout

The checkpoint table (added 2026-10-29) lets a savestate load restore the BGM at any frame by
decoding at most 16,383 frames (2.3 ms on the A9), from the moment the pack loads. Before it,
the ARM built the states itself in the background after each pack load, and a load that came
before the builder reached that stream decoded from frame 0 (562 ms for a frame 87 s in): the
HD music dropped out for that long. oggdecode writes the states as it encodes (OUT.ck; the
encoder's state is the decoder's, test_ima.py checks the two agree), hdp3_convert stores them,
and hdp3_read.check_audio re-derives every one by decoding the whole stream (oggdecode ckscan).
The ARM takes the table only if it fits the section exactly (spacing, count, each entry's first
state = its state0, every index <= 88); otherwise, and for a pack converted earlier (offset 0),
it builds the states in the background as before. The FPGA reads only the first header qword,
so it is unaffected; a pack converted earlier still plays, but needs converting again to gain
this.

Each `<bgm>`/`<sfx>` line is one entry; lines naming the same file share one stream. A line
whose file is missing or does not decode is KEPT as an entry with `frames` 0 (corrected
2026-10-21; this paragraph said "left out"): Mesen keeps the line too and fails when it opens
the file, which is what tells "defined but undecodable" (SFX status error bit 2 set) apart
from an undefined id (error 0). hdp3_convert.py has always written it this way; the RTL marks
such keys in its key table and the model (hdaudio.py) does the same.

The loop frame is the 48 kHz frame of the first output Mesen's resampler produces wholly from
source frames >= LOOPPOSITION: the frame count emitted before source frame LOOPPOSITION + 4 is
fed, because each Hermite output reads four source samples. At equal rates Mesen copies the
samples through with no history, so it is LOOPPOSITION itself. On a loop the hardware jumps
there and restores `stateL`, which is the encoder's own state at that frame, so the replayed samples are exactly the first pass's. Difference from Mesen: at
the seam Mesen interpolates across the last source samples and the first looped ones (about
three 48 kHz samples); we jump. Not audible; recorded so a comparison does not mistake it.

### Hardware (hd3_audio)

- Registers (core clock, on the CPU bus): $4100-$4106 read and write, or with the pack's
  `alternateRegisterRange` option writes at $3002/$3012/.../$3062 and reads at $4018/$4019,
  exactly as spec 8.1, including the inverted BGM error bit. While the device is active the
  mapper does not see those writes and their reads come from the device (Mesen's
  SetAllowOverride). Active = a pack with a section 15 is loaded AND the "HD Pack" OSD option is
  on (the same gate as disableSpriteLimit).
- The play commands (reg 5/6) look the key up in an on-chip copy of the entry keys, by
  binary search, so the status read that follows a play write sees the right error bit, as
  in Mesen. As built (2026-10-21) hd3_audio reads the table from DDR3 itself once the pack
  is accepted (512 x 18: the 17-bit key plus "did not decode"), rather than the loader
  copying it while the pack streams in; the search ends long before the CPU can read the
  status back (>= 3 CPU cycles), and the 4-entry write queue covers a burst of writes.
- Voices: 1 BGM + 7 SFX, on the HD clock. Mesen allows any number of SFX; with all 7 busy a
  new SFX replaces the oldest one. (Corpus measurement of the most SFX at once: to do.)
- Every 48 kHz tick each live voice decodes one frame (ADPCM state per voice), scaled by its
  volume with Mesen's exact `x * v / 255`, and the mix is accumulated with a clamp after
  each source, BGM first then SFX in start order, starting from the NES sample.
- NES path while the device is active: the core's unsigned sample is made signed and run
  through a first-order DC blocker (Mesen's NES signal is DC-free; the MiSTer one sits near
  full scale at silence, which would clip any added audio), and the output goes out signed
  (AUDIO_S = 1) and stereo. Without a section 15 the NES audio path is bit-for-bit the old one.

### As built (rtl/hdpack/hd3_audio.sv, 2026-10-21)

- Clock: clk_f (4 x the core clock). The 48 kHz tick is a modulus accumulator on clk_f
  (step 48,000, modulus 85,909,091 NTSC / 106,406,848 PAL; `pal` is NES.sv's PLL condition,
  `effective_sys_type == 1`), so 48 kHz is exact over time on both clocks.
- Bus (core clock): `hit_wr` / `hit_rd` / `rdata` are combinational from the address. nes.v
  exports the CPU/DMA bus (`addr`, `dbus`, and `cpu_ce & mw_int` as the write strobe, one
  per write cycle, so a read-modify-write's two writes are two events, as in Mesen) and takes
  the hits back: `prg_read` / `prg_write` and the PPU write are gated by them, and
  `ext_bus_driven` takes `rdata` first. Writes go through a 4-entry queue into clk_f.
- Memory: the device reads section 15 through hd3_ddr reader 7 with its own file-offset
  mapping; file bytes >= 192 MB use the overflow window (`rd_hi`), which hd3_ddr now carries
  per reader (and the loader per write, `wr_hi`). DDRAM_ADDR = {3'b001, ~hi, qaddr}.
- Streaming: each voice keeps two 64-byte blocks tagged by absolute file block. Fetches are
  issued in two passes: first any voice MISSING THE BLOCK IT IS PLAYING NOW, then
  prefetches. One pass in voice order (the first version) let a new voice's first block
  wait behind other voices' prefetches under DDR3 contention and start a frame late; the
  busy-DDR3 runs found it, and mutant 35 pins the two-pass order.
  2026-10-27 (item 11, ALM budget): the scan is SERIALISED - one candidate {pass, voice} =
  {ps_k, ps_i} is examined per clk_f while the reader is idle, through constant-index muxes
  (sc_*), instead of 2 x 8 copies of the block arithmetic in one clock. After a read is
  issued the scan restarts at pass 0, voice 0, so the order is the old one, spread over at
  most 16 clocks (a voice plays one byte per 48 kHz frame, ~1,790 clk_f). The voice arrays
  are written per constant index: a variable-index write (and a function reading an array
  indexed by the register ps_i) crashed Quartus 17's elaboration ("read to RAM wasn't
  mapped"), the hd3_add rank() crash again. Quartus 17 synthesis of hd3_audio alone: 3,699
  -> 3,447 ALMs (est.), registers 3,033 -> 3,037, RAM bits unchanged. Tests: tb_audio_all.sh
  26/26 runs identical (U 0 in every run, busy-DDR3 runs included); neg_audio.py mutant 35
  re-anchored to the new code plus three new mutants (scan never reaches the prefetch pass,
  scan skips the last voice, scan does not restart after a read) - all 61 caught; ModelSim
  front-end check PASS. In the whole NES_PAINT synthesis (quartus_map only): hd3_audio 4,777
  -> 4,161 ALUTs, design 61,761 -> 61,380 ALUTs, map estimate 42,887 -> 42,837 ALMs. A real
  saving, but far short of the gap to fitting (OPEN_ITEMS item 11).
- Output: NES.sv sets AUDIO_S = 1 and puts out_l / out_r on AUDIO_L / AUDIO_R while the
  device is active; otherwise the console path is bit-for-bit the old one.

Known limits (not differences from Mesen's audible output, recorded so a comparison does not
mistake them):
- A play written so close to a 48 kHz tick that its first block cannot be fetched in time
  starts one 48 kHz frame (20.8 us) later than the model's "next tick". Mesen applies a play
  when it mixes the frame's audio buffer (OggMixer::MixAudio, once per NES frame), so its own
  start point is quantized to ~16.7 ms; the bit-exact comparison with the model holds when
  writes land early in the frame, and the testbench's timed writes (`Q`) use an undefined id
  (no fetch) so they test the tick race without this.
- Under extreme DDR3 contention a looping BGM whose last block holds about one 48 kHz frame
  has a one-frame prefetch margin (BGM is voice 0, so it has fetch priority). No underrun
  in any test run.
- SFX voices: 7 (Mesen: unlimited; with all busy the oldest is replaced).

### NES level (2026-10-21)

Mesen adds HD audio at full s16 scale (`sample * vol / 255`) to the NES signal, and its NES
signal is `NesSoundMixer::GetOutputVolume() * 4` through blip_buf (which keeps the units):
4 x (479400 / (8128 / sq + 100) + 798950 / (22638 / tnd + 100)) with sq = pulse1 + pulse2 and
tnd = dmc + 2.7517 tri + 1.8494 noise, i.e. 20,000 x the standard normalised APU output (about
20,000 at full scale). Master volume is applied after both, so it does not change the balance.

The core's NES sample is rtl/apu.sv's APUMixer (pulse_lut + mix_lut, unsigned, full scale
63,344), inverted in nes.v and, for 52 of the mapper modules (MMC1, UxROM, MMC3, ...), halved
by the mapper (`{1'b0, audio_in[15:1]}`). Measured over every channel and level (scratchpad
nes_level.py, which parses the LUTs out of apu.sv and evaluates Mesen's formula): core/Mesen =
1.753-1.761 for the pulses, 1.53 for the triangle, 1.43-1.73 for noise, 1.52-1.58 for DMC,
1.584 at everything-max. The two APU mixers weight pulse against TND differently, so no single
gain matches every channel. The device now scales the DC-blocked NES sample by **5/8**
(`x/2 + x/8`, arithmetic shifts, `nes_level` in hd3_audio.sv / Device.nes_level in the model):
every channel within -1.0..+0.8 dB of Mesen, everything-max -0.1 dB. (NES_SHIFT 0, the previous
value, was +3.1..+4.9 dB; a shift of 1 would be -2.9..-1.1 dB.) The parameter NES_SHIFT is gone.

Mappers with expansion audio halve the 2A03 too, one level up: cart.sv feeds them a premix
from mmc5_mixed / SS5b_mixed / vrc6_mixed / vrc7_mixed / fds_mixed / namco163_mixed, each of
which outputs (2A03 + expansion) / 2 (scratchpad mapper_audio_scale.py lists every
`assign audio_b`; the mapper module passing `audio_in` through unhalved is passing that
premix). SS5b and FDS also invert it, which the DC blocker does not care about. So 5/8 holds for
every mapper of the 85 audio packs, Castlevania III (MMC5) and Journey to Silius (Sunsoft 5B)
included.
Known limit, outside the audio corpus: Mapper 111 (GTROM), Mapper 83 and the NSF player with no
expansion chip pass the raw 2A03 (6 dB louder than Mesen against HD audio, as they already are
against every other mapper on this core), and Namco 163 submappers 4/5 attenuate it a further
6 / 12 dB in namco163_mixed. Both are the core's own mapper mixing.

### ADPCM against Mesen (2026-10-21)

The pack stores exactly the PCM Mesen mixes (Mesen's own stb_vorbis + Hermite resampler, at
48 kHz), so the only difference between what the core plays and what Mesen plays is the IMA
ADPCM quantisation. `oggdecode snr FILE 48000` measures it: it renders as the converter does,
runs the same encoder and compares the encoder's reconstruction - which is the decoder's output
(test_ima.py pins the C encoder to the model, tb_audio the RTL to the model) - against the PCM.
Cross-checked on three a034 files by decoding the stored stream with the model's decoder in
Python: the two figures agree to 0.01 dB.

Over every distinct file the 85 audio packs name (the work tree walked as audio_survey2.py
walks it: 2,401 `<bgm>`/`<sfx>` lines, 980 naming a file the pack does not have; 681 files
after deduplication by content, 33,959 s; scratchpad snr_corpus.py, results snr_corpus.tsv):
SNR duration-weighted mean 34.2 dB, median 33.2 dB; worst loud (>= -42 dBFS rms) 100 ms window
over the corpus 7.4 dB, 1st percentile 9.1 dB. 58 files are under 20 dB, 216 under 30 dB.
(A first pass parsed audio_survey2.txt's own listing and saw only 74 packs: that listing cuts
long paths at its column width.)
The low end is chiptune: 8-bit SFX remakes (a032 8BitTroid: Metal.ogg 7.4 dB) and Mappy's
square-wave BGM/SFX (a098, 11.9-13.6 dB) - IMA's known weakness on full-scale square edges, the
step index has to climb at every edge (test_ima.py floors a full-scale square at 5 dB).
Orchestral BGM is 33-38 dB (a034 Kraid 37.4, Brinstar 34.7).

The figures above are for the first, greedy encoder (each nibble chosen for the sample in
hand). Since 2026-10-26 the converter uses a **depth-3 lookahead encoder** (OPEN_ITEMS 13f):
each nibble is chosen by the least total squared error over that sample and the next two. The
first two levels try all 16 nibbles; the last level tries only the two magnitudes either side of
the target plus the other sign's magnitude 0 (which wins when the predictor is clamped at full
scale), and test_ima.py checks that shortcut against all 16 over 20,000 random states. Ties go
to the lowest nibble, so the output is deterministic, and C and Python agree byte for byte.
The decoder, the stream format and the hardware are unchanged; `pcm2adpcm` and `snr` take an
optional depth (0 = the greedy encoder).

Measured with `oggdecode snr` on the files that were worst under the greedy encoder:

    a032 Metal.ogg              7.4 -> 14.3 dB
    BirdOutOfHole.ogg          10.2 -> 18.1 dB
    PowerBeamShot.ogg          11.5 -> 18.9 dB
    a098 0_sfx-wave            11.9 -> 22.0 dB
    a098 3_sfx-get             12.0 -> 23.1 dB
    a098 bgm-flydisc           11.5 -> 20.9 dB
    orchestral music           +0.2 .. +1.2 dB

Synthetic floors in test_ima.py at depth 3: sine 38.5 dB, noise 19.0, full-scale square 9.9,
burst 8.6, each no worse than greedy. Searching only where the greedy pick misses gains nothing:
the gain comes from raising the step BEFORE an edge. The cost is converter time, about 0.55 s per
second of audio (a033 25 min instead of 3; the whole corpus about 5.5 h instead of 1.5 h).
The corpus-wide figures in the paragraph above are to be re-measured on the depth-3 corpus.

### Savestates (2026-10-21)

Mesen's HdAudioDevice::Serialize saves the album, the last BGM track, its position, both
volumes and the playback options - not the SFX and not the pause flag - and on load restarts
that BGM at the saved position and re-applies volumes and options. The MiSTer core keeps the
same set in two 64-bit savestate registers (rtl/regs_savestates.sv indices 29 and 30, in
NES.sv beside SS_Ext; their read-back is ORed into the external savestate bus):

    A: [63:56] album  [55:48] BGM volume  [47:40] SFX volume  [39] loop option
       [38] BGM on  [37:29] its entry (key-table index)  [28:0] its next 48 kHz frame
    B: the BGM's ADPCM state, in the file's state layout ([15:0] predictor L, [22:16] index L,
       [47:32] predictor R, [54:48] index R); [63] = entry not fetched yet, use its state 0

A's reset value is the device's power-on state (volumes 128). With no BGM, its fields are 0.

Two deliberate differences from Mesen, both in the "resume exactly" direction:
- Position. Mesen saves `stb_vorbis_get_file_offset` (a byte offset in the .ogg) and restores
  it with `stb_vorbis_seek`, which takes a SAMPLE number, so its BGM resumes at the wrong place.
  The core saves the exact frame and ADPCM decoder state, so the BGM continues sample-exact.
- A save with no BGM playing. Mesen's load then leaves whatever BGM is playing; the core stops
  it, so the audio after a load matches the moment of the save.

How it works (hd3_audio.sv):
- Save: clk_f takes a snapshot of the state whenever the previous one has been taken over and
  hands it to the core clock with a toggle handshake (the snapshot holds still until the ack).
  The core clock copies it into `ss_out_a/b` (the registers' read-back) except while nes.v's
  `saving_savestate` (new port `SaveStateExt_save`) is high, so the two registers, read at
  different times during a save, come from one snapshot. Any clk_f clock can be snapshotted:
  every field pair it uses is written in the same clock, and a save starts reading 100 core
  clocks after the CPU stops (savestates.vhd SETTLECOUNT).
- Load: the falling edge of `loading_savestate` (the registers hold the loaded values by then)
  requests a restore on clk_f, applied in S_IDLE once the key table is in: the registers, and
  the BGM on its entry with the saved frame and ADPCM state, used when the entry has been
  fetched (`rs_pend`). A play before that fetch wins. An entry index past the table (a state
  from another pack) means no BGM. A load before the pack is ready is dropped by the pack's
  arrival, which resets the device.
- A BGM saved before its entry was fetched (just played) is saved as "frame 0, state 0"; a
  restored BGM saved before its entry was fetched is saved as restored.
- fullsim ties the ports off (its savestate machine never runs).

### Tests (2026-10-21)

- `tools/audio/hdaudio.py`: the reference model (device + mixer + the shared ADPCM codec).
- `tools/sim3/tb_audio.cpp` + `tools/audio/tb_audio_run.py`: the RTL against the model,
  every 48 kHz frame and every register read compared exactly. A synthetic section 15 (built
  with the model's own encoder) holds BGM with loops inside and at block boundaries, a stream
  with no loop point, 11 SFX, undecodable BGM and SFX entries, one stream shared by a BGM and
  five SFX (six voices on one stream crossing blocks together), and a full-scale square that
  drives the IMA index to 88. The script covers play / loop / pause / resume / stop, both
  volumes from power-on (128), albums, undefined ids, the error and status bits, a register
  sweep, the alternate range, voice stealing, a read-modify-write's double write, and writes
  timed 9-14 clk_f before a tick. The DDR3 model has random latency and gaps, a "busy" mode
  (300-450 clk_f per read), and returns junk for a wrong-window read; the tb also counts
  ticks against the modulus (rate check) and underruns.
  `tools/audio/tb_audio_all.sh`: 13 runs per build (seeds, alt, random ops, PAL, busy) on
  the 192 MB window build and on a build whose window ends at 9,216 bytes (`--need-hi`: the
  run fails unless reads really use the overflow window). Result: 26 of 26 runs identical.
- Savestates: the model has `Device.save()` / `load()` (the same two words), and the script
  has save (`V`), load (`L`), a save with a register write between its two reads (`X`), and a
  DDR3 stall (`H`) so a save can land while an entry fetch is still out. Scripted cases: a
  load back to an earlier BGM position, a BGM resumed after a stop with its volume restored,
  a load that stops a BGM (saved with none), saves taken before the entry fetch (after a play,
  and after a load), a play while a restored BGM waits for its entry, a load of an entry one
  past the table, a load before the pack is ready, and the `X` save; the random runs add ~2%
  saves and loads per frame. Saved words are compared with the model's (a not-yet-fetched BGM
  normalised to its entry's state 0). The load in the testbench changes the register values
  part-way through, as the bus does, so a restore taken at the load's start is caught.
  Result: the 26 runs of tb_audio_all.sh identical, 994 saves and 1,044 loads compared.
- `tools/audio/neg_audio.py`: 58 mutants (35 + 21 savestate + 2 NES level), each anchor
  matching exactly once; ALL 58 CAUGHT, all re-run after the NES-level change. The savestate set: restore ignoring the frame / the ADPCM state / the
  volumes / the album / the loop option / the BGM's loop flag, a restore keeping a BGM the save
  had not, never requested, taken at the load's start, the entry bound, a pre-pack load
  applied, a pending restore surviving a play or kept after its fetch, and on the save side the
  frame off by one, a pending restore saved from the voice (frame, state) or as not fetched,
  the not-fetched flag never set, BGM fields kept with no BGM, and the read-back not held
  during a save. Moving the stalled-fetch cases first made the IMA-clamp mutant escape (they
  had cut the full-scale square to 10 frames); they were moved to frames 940-990 and all 56
  re-run on the final scenario.
- Before the savestate work: 35 mutants, ALL 35 CAUGHT.
  Four were first missed and each named a gap now closed: the IMA index clamp (the square
  stream), no prefetch (busy mode + the shared stream), a lost tick (timed writes + the rate
  check), the PAL modulus (PAL runs); "end flagged late" needed the per-frame status read. One
  mutant (the declaration initial value of `active`) was shown to change nothing - `active`
  is set only by the pack-load edge, which resets it - and was replaced by the reset-value
  mutant.
- Loader overflow window (`tools/sim3/run_loader_window.sh`): `make_audio_pack.py` appends
  the synthetic section 15 to obj/loader_fix.hdp (ending mid-qword), and hd3_loader is built
  with MAX_BYTES inside the stream data and MAX_TOTAL just above the file. tb_loader maps
  every DDR3 write back to its file offset and fails on a write in the wrong window: 45
  checks, 0 failures - image byte-exact, the overflow window used iff the file crosses
  MAX_BYTES, a non-audio section past the window refused (8), a file over MAX_TOTAL refused
  (6). `neg_loader_window.py`: 8 of 8 mutants caught (boundary, offset, audio exemption,
  window check dropped, wrong section exempted, size limit, both wr_hi assignments). The
  ordinary loader tests are unchanged: neg_loader.py 33 of 33 caught (render_orig needs a058
  as before); two of its anchors were updated for the 29-bit counter.
- Integration: engine lint 0 errors 0 warnings (lint.sh), ModelSim front-end 22 files PASS
  (msim_check.sh, hd3_audio included), run_engine.sh quick ALL PASS.

### Since 2026-10-28: decoding and volumes on the ARM (OPEN_ITEMS item 11)

Owner decision (item 11): the ADPCM decoding, the volumes and the streaming left the FPGA. The
split, at the top of hd3_audio.sv and aud_arm.h:

- FPGA (`hd3_audio.sv`): the registers and the status the CPU reads; on-chip copies of the entry
  table's keys (512 x 18) and of each entry's frame count and loop frame (512 x 58), so a play is
  resolved by binary search before the CPU can read the status back; a voice TRACKER (which
  voices play, their frame, ends, loops, the SFX start order, the pause) for the status bits and
  the savestate; the 48 kHz tick; and the mix.
- An EVENT LOG in DDR3 (window byte 0xD000000, 2,048 events of two qwords): every write the
  device takes (kind 1), every savestate restore (kind 2, word A), every pack reset (kind 3,
  the section's offset and length) and, every 256 ticks, a state event (kind 4: word A of now,
  header flags bit 0 = BGM paused) from which an ARM program started late resumes the BGM at
  its frame (one started with the session ignores them), each stamped with the tick it applies before, through a
  256-entry FIFO into the engine's write port (lowest priority, after the records, the watch
  copier and the pack word). Every 16 ticks a status qword (0xC400050) gives the events logged
  and the ticks mixed.
- ARM (`tools/armdec/aud_arm.c`, its own thread on core 1 in hdarm_daemon.c): hdaudio.Device on
  that log, line for line, and for every tick the volume-scaled sample of each voice in mix
  order (BGM, then the SFX in start order) into a PCM slot (0xD010000, 4,096 slots of 8 qwords:
  a tag {0xD5, t} written last, then 8 x {R, L}). The section is NOT copied into ARM memory (one
  pack's is over 100 MB): a pack reset reads the header and entry table, and each voice reads
  its stream 256 bytes at a time from the pack in DDR3.
- The mix (FPGA) reads slot t at tick t + AUD_D, AUD_D = 512 ticks (10.7 ms; 1024 = 21.3 ms
  until 2026-10-29, see "Sizing AUD_D" below): the NES sample is
  delayed by the same AUD_D through a 2,048 x 16 RAM, so NES and HD stay aligned, and the output
  is the model's sample for sample, AUD_D ticks late. A slot whose tag is not t (the ARM late or
  not running) mixes no HD voice and counts an underrun; the first AUD_D ticks after a pack load
  never count.

What this changes for the user (owner-level, not differences from Mesen's sound):
- All audio while the device is active, the NES channels included, comes out 10.7 ms later than
  before (~0.64 of a frame behind the picture).
- With the ARM program not running there is no HD music or SFX; the NES audio still plays.
- A program started after the game has begun plays only voices started after it (it begins at
  the oldest event still in the log when more than 2,048 have been written).

Savestates: register B (the BGM's ADPCM state) is now always saved as 0, and a load uses word A
only. The ARM derives the state at the saved frame by decoding from the nearest checkpoint (the
state is kept every AUD_CK = 16,384 frames of an entry; frame 0's is the entry's state 0, and a
background builder fills in the rest after a pack load), at most 16,383 frames: 2.31 ms on the
A9 (it was 65,536 / 9.0 ms until 2026-10-29). The model's `save()` returns
(A, 0) and `load(a, b=0)` derives the state the same way (`state_at`), so the saved words still
compare exactly. A savestate made before this change carries a B the core now ignores.

Sizing AUD_D (measured 2026-10-29 on the A9, the audio thread's own settings: core 1,
SCHED_FIFO 10; scratch aud_bench.c). AUD_D must cover the longest gap between two render
passes. Decode: 132-135 ns a frame. One pass can hold a restore (at most AUD_CK - 1 frames:
2.31 ms max) and a checkpoint-builder step (8,192 frames: 1.6 ms max), then sleeps 2 ms and
wakes at most 0.15 ms late (60 s, 29,506 sleeps); 96 ticks of rendering take 0.04 ms. Worst
gap ~6.1 ms = ~292 ticks, so 512 leaves 1.75x. At the old AUD_CK the restore alone was 9.0 ms,
which is what had needed the 21.3 ms. Two traps met measuring it: a builder run NONSTOP for
over 950 ms hits Linux RT throttling (sched_rt_runtime_us 950000 of 1000000) and stalls 52 ms;
the daemon sleeps every pass and never does (5 runs with the sleep: no pass over 2 ms). And a
restore BEFORE the builder had reached that stream decoded from frame 0 (562 ms for a frame
4.19 M into a stream): no AUD_D covers that. Fixed the same day by the section's checkpoint
table ("Section 15 layout"); only a pack converted before it still has that window.

Found while testing the split: the log FIFO wrote each entry one slot late (the write index was
advanced in the same clock as the registered write), so the ARM read an unwritten entry first -
in hardware that would have written zeros to window qword 0, the pack header. The slow-ARM run
(`--arm 30000`) found it; the testbench now fails on any write outside the status qword and the
log (`nstray`).

Tests (2026-10-28):
- `tb_audio` links aud_arm.c and runs it against a DDR3 store at a chosen cadence ("A n"
  clk_f, 0 = never), with the write port's ready random and held low during DDR stalls; the
  model (`tb_audio_run.py --d`, `--arm`) mixes the NES sample and the voices AUD_D ticks late.
  `tb_audio_all.sh`: the previous matrix on the 192 MB window build and on a build whose window
  starts inside the entry table (WIN 8448, `--need-hi`: its read must cross into the overflow
  window), plus an ARM running about every 17 ticks, an ARM that never runs (U = frames + 1 - D,
  NES only), an ARM event queue of 2 (the queue-full path) and the shipped AUD_D (512 since
  2026-10-29, with an ARM pass only every ~274 ticks - the measured worst gap - and a control
  at ~600 ticks a pass that must mismatch). Also
  `test_aud_arm.c`: a second pack load at the same file offset (play and restore must decode the
  new pack, not a block cached from the old one) and an SFX ending under a full voice list.
  Result: ALL IDENTICAL, test_aud_arm ALL PASS.
- `neg_audio.py`: mutants of BOTH halves (hd3_audio.sv: mix, delay, slot tag, underrun gate, NES
  level, registers, search, entry tables, overflow window, tracker, savestates, the log, its
  FIFO and the write port; aud_arm.c: IMA, volumes, loops, pause, stealing, mix order, restore,
  the log reader, the stream blocks), each built with an event queue of 2 and the window inside
  the entry table. Results in OPEN_ITEMS item 13.
- `fullsim` (both routes) links aud_arm.c and serves it from the DDR3 model, so a whole-core run
  hears the HD audio; its final line gives the ticks rendered, the events and the underruns.

Still to do (OPEN_ITEMS 13): Mesen's register writes for a recorded input against the writes
fullsim sees for the same input, with a pack whose ROM writes the device; the ADPCM output
against Mesen's decode + Hermite chain as a signal-to-noise figure. The savestate path through NES.sv / nes.v (savestates.vhd's bus) is only exercised by
the hardware: tb_audio drives hd3_audio's ports directly, and Questa cannot compile NES.sv /
nes.v (they use names before declaring them, which Quartus accepts) - vlog reports no error on
any new line or signal.

## Stage-3 engine (8f) - pacing and scan-out tests (2026-10-19)

`tb_engine` runs the whole engine (pack load over the OSD port, records at the NES line rate on
the core clock, the engine on the HD clock, its own scan-out captured); `run_engine.sh` compares
every shown frame with hdref. quick: SMB p053, the lane fixtures (incl. long DDR3 busy bursts and
an asymmetric overscan window), the light frames, slow (latency 300) and pace (latency 250).
full: adds Metroid, Castlevania (incl. a watch snapshot 14 lines late), Zelda II and the two
addition fixtures. Full suite ALL PASS, 0 pixels differing on every compared frame; savestate
traffic 946,503 accesses, 0 failures.

The engine negative control (`tools/sim3/neg_engine.py`) first missed five mutants of the scan-out
pacing. Each named a case the tests never reached, not a harmless change:

- **Drawing into the ring while the previous frame is still shown** (`line_ok` ignoring the
  frame id) and **forgetting the finished frame** (`p_fin_v` never set). On an ordinary frame
  RESOLVE ends after the last line's show/blank decision, so neither path is reached. A light
  frame (one tile row, no sprites) finishes so early that RENDER starts the next frame at
  disp_y 238: the frame on screen must still show its line 239, from the finished-frame record.
  Case `light`: SMB frames alternating with two light frames (`make_light.py`).
- **Claiming a line complete one line early** (`p_cur_y = out_y + 1`). Only visible when
  RENDER is drawing the very line being decided. The `slow` case falls behind for good after
  ~20 lines, so that moment never came. Case `pace`: DDR3 latency 250 (150 keeps up, 300 falls
  behind), so RENDER is caught and passed repeatedly through the frame.
- **The frame id advancing one line late**: caught once the testbench checks the pacing rule
  directly (below).
- **Ignoring the window's left edge**: the ring columns left of the window are never written,
  and the model's ring started as zeros - exactly the colour the mask produces. On hardware the
  ring holds whatever the previous pack drew there. The testbench now fills the ring with
  random data before the run.

Two checks were added to `tb_engine` on every ring write: no write while the scan-out shows the
frame before the one being drawn, no line beyond disp_y + 7 (lines 0-7 only in the blanking);
and no write to a line the scan-out has decided to show while it is on screen. They catch a
pacing error even when the overwritten pixels happen to look the same.

Result: 34 of 34 engine mutants caught (the full re-run against the final RTL with the new
checks), and the full suite passes with them.

Promoted 2026-10-20 to rtl/hdpack (hd3_engine, hd3_cross, hd3_scanout and the newer hd3_loader,
hd_stage2, hd_watch) with NES.sv and files.qip; the tests run from tools/sim3: `lint.sh` 0
errors 0 warnings, `build_engine.sh`, `run_engine.sh quick` ALL PASS (0 pixels differing on
every frame, slow/pace 0 wrong lines), `neg_engine.py`. Build output goes to tools/sim3/obj.

### Regression list (what to run after an RTL change; 2026-10-21)

`test_render_all.sh` covers the renderer side only (cache unit, fmap, every render group on
the normal / NSLOT=4 / RS_MAX=64 / OF_LOG2=1 / RAW=6 / NL=64 builds, the 8k index fixtures).
It does NOT run the hd3_add unit test, the engine, ROM patching or SHA-1, so after a change
also run, from the folder named:

    tools/sim3   run_engine.sh full          the whole stage-3 engine (needs build_engine.sh)
    tools/sim3   run_add.sh                  hd3_add, 16 suites; rebuild BOTH tb_add builds
                                             (obj/add64 NL=64, obj/add NL=16 as SMALL_EXE) from
                                             rtl/ first - run_add.sh uses whatever is in obj/
    tools/sim3   romfix_cases.py             hd3_romfix in the engine, 20 cases x 2 load orders
    tools/sim3   test_sha1.py                hd3_sha1, 416 digests
    tools        test_hdp3_patch.py          converter section 14 against rompatch
    tools        msim_check.sh               ModelSim front-end: every files.qip
                                             rtl/hdpack file with vlog (0 errors, 0 warnings),
                                             then hd3_render loaded from the project folder so
                                             the four $readmemh ROMs must read their .hex files
    .            tools/fullsim/build.sh      whole core under Verilator, then a pack run with
                 + fullsim.exe --rec         --rec and tools/sim3/check_render.py on its hdv_
                                             frames (0 px against hdref); slow (~5 s/frame
                                             with a pack), so for changes to the integration,
                                             the engine or the scan-out rather than every edit

plus the negative control of every module touched (neg_*.py in tools/sim3; neg_patch.py and
neg_read_addtab.py in tools). Each neg_*.py builds from rtl/ itself; run_add.sh,
romfix_cases.py and test_sha1.py do not, so a stale obj/ build passes them silently - check
the exe date against the .sv it tests.

msim_check.sh was checked against planted faults on a scratch copy: a missing .hex file fails
(that ROM reads x) and the 10-20 part-select fix undone fails (vlog-13069 in hd3_render). It
checks that each ROM loaded its file, not that the file is right. ModelSim is not Quartus: the
final compile still needs its map report read for $readmemh / 10850 warnings.

hd3_render's .hex paths (2026-10-21). The four $readmemh names are relative, and a relative
name resolves against the folder the tool runs in. Quartus and ModelSim run from the project
folder and need "rtl/hdpack/..."; the Verilator unit testbenches run from rtl/hdpack and need
the bare name. The file picks the bare names only under `VERILATOR` and only when
`HD3_HEX_PROJECT` is not defined; tools/fullsim/build.sh defines it because fullsim runs from
the project folder. Before this define the first Metroid fullsim run loaded all four ROMs as
zeros (Verilator only warns on a missing $readmemh file), so fullsim.cpp now refuses to start,
rc 2, when the files are not where the build will look. Checked by preprocessing hd3_render
both ways with verilator -E (bare names by default, rtl/hdpack/... with the define) and by
re-running msim_check (PASS).

fullsim result with the integrated engine (2026-10-21): Metroid (USA) with the a034 pack and
its mmm.ips, 400 frames - 0 frames dropped, 0 ADD aborts, 0 underflow, 0 overrun, 0 dictionary
overflow; hdv frames 0/100/200/300 equal hdref (0 px); hd frames 100/300/400 equal Mesen
102/302/402 in exact RGB, and hd 200 equals Mesen 201 (the frame-offset between the two runs
lands on a title-animation step there). Details in tools/fullsim/README.txt.

After the 2026-10-20 front-end fixes (hd3_add, hd3_ccache, hd3_rcache, hd3_render,
hd3_resolve) everything above was re-run on the result: run_engine full ALL PASS,
test_render_all rc 0, run_add.sh 16/16 (both tb_add builds were stale, dated 10-17, and were
rebuilt), romfix_cases 40/40, test_sha1 416/0 wrong (its build is newer than hd3_sha1.sv,
which the fixes did not touch), test_hdp3_patch ALL PASS, msim_check PASS; neg_render 54/54 (+1 equivalent), neg_resolve
50/50, neg_engine 34/34, neg_add_rtl 29/29 (+1 equivalent), neg_cache 14/14, neg_ix 11/11.
