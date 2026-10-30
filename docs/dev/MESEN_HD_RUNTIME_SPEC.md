# Mesen2 NES HD Pack Runtime Specification

Source read: Mesen2 `Core/NES/HdPacks/` plus the parts of `Core/NES/` it calls
(NesPpu.cpp, BaseNesPpu.cpp, BaseMapper.cpp, NesMemoryManager.cpp, NesConsole.cpp,
INesMemoryHandler.h). Read-only; nothing was modified. `Utilities/` (PNG, VirtualFile, patching,
stb_vorbis, resampler, SoundMixer) is NOT in the extracted tree, so anything about those is
marked INFERRED.

Citation abbreviations (all paths under `Core/NES/`):

    LDR   HdPacks/HdPackLoader.cpp        DATA  HdPacks/HdData.h
    PACK  HdPacks/HdNesPack.cpp           PACKH HdPacks/HdNesPack.h
    PPUH  HdPacks/HdNesPpu.h              PPUC  HdPacks/HdNesPpu.cpp
    COND  HdPacks/HdPackConditions.h      AUD   HdPacks/HdAudioDevice.cpp
    MIX   HdPacks/OggMixer.cpp            OGG   HdPacks/OggReader.cpp
    VF    HdPacks/HdVideoFilter.cpp       NPPU  NesPpu.cpp
    BPPU  BaseNesPpu.cpp                  MAP   BaseMapper.cpp
    MEM   NesMemoryManager.cpp            CON   NesConsole.cpp

Files present but NOT part of the runtime: HdBuilderPpu.h, HdPackBuilder.cpp/.h (pack recording
tool, only used while "recording an HD pack").

Notation: "(BUG?)" marks behaviour in the source that looks unintended. A bit-exact
reproduction would copy it; a "what the author meant" reproduction would not. Each is
called out so the choice can be made deliberately.


## 0. Architecture overview (what happens when)

1. Load time (CON:216-236): `hires.txt` is parsed into `HdPackData`. If the pack contains a
   `<patch>` whose SHA1 equals the ROM file's SHA1, the patch is applied to the ROM file before
   the ROM is used (CON:224-228). PNGs are decoded asynchronously in a detached thread
   (CON:230-234, DATA:507-521); any PNG not yet decoded is decoded on first use
   (DATA:224-244, DATA:338-355).
2. Emulation time, per PPU dot: the PPU subclass `HdNesPpu` records, for every visible pixel,
   an `HdPpuPixelInfo` record (BG tile identity + up to 4 sprite identities + scroll register
   snapshot + colour/emphasis state) into a 256x240 array (PPUH:73-182). Two such arrays are
   double-buffered (PPUC:14-16, PPUC:43).
3. End of frame (scanline 240, dot 0, before the frame counter increments and before VBlank/NMI
   at scanline 241): `OnBeforeSendFrame` stores the frame number and snapshots every "watched"
   CPU/PPU memory address into the array that was just filled, then swaps arrays
   (NPPU:1412-1417, NPPU:1178-1184, PPUC:26-46).
4. Video filter (possibly another thread, one frame later): `HdNesPack::Process` renders the
   completed frame entirely from that snapshot: per-frame setup (condition cache reset, palette,
   background selection, `<addition>` insertion), then per line, then per pixel
   (PACK:569-587). Nothing reads live emulator state during rendering except
   `GetFallbackTile` (static table) and the mapper CHR in `InitializeFallbackTiles` (load-time
   only).

So: every condition and every background decision is evaluated from (a) the completed frame's
per-pixel record and (b) a memory snapshot taken at scanline 240 dot 0 of that same frame.
Nothing reads "the previous frame", nothing reads "the state at the moment the pixel was
drawn" except what the per-pixel record itself captured at that moment.

Hardware implication: a hardware implementation needs, per frame, (1) a 256x240 per-pixel
record (or a cheaper equivalent derived per tile/line, see section 7.4), (2) a snapshot of the
watched addresses taken at the end of the last visible line, and (3) a frame counter.


## 1. Per-pixel capture (what the PPU records)

### 1.1 Record layout

`HdPpuPixelInfo` (DATA:102-112): `Tile` (one HdPpuTileInfo), `Sprite[4]`, `TmpVideoRamAddr`
(the PPU "t" register, 15 bits), `XScroll` (fine X, 3 bits), `EmphasisBits` (3 bits),
`Grayscale` (bool), `SpriteCount` (0..4).

`HdPpuTileInfo` (DATA:86-100, inherits HdTileKey DATA:12-74): `PaletteColors` (u32),
`TileData[16]`, `TileIndex` (i32, -1 = NoTile), `IsChrRamTile`, `OffsetX`, `OffsetY`,
`HorizontalMirroring`, `VerticalMirroring`, `BackgroundPriority`, `BgColorIndex` (0-3),
`SpriteColorIndex` (0-3), `BgColor` (6-bit palette value), `SpriteColor` (6-bit),
`PpuBackgroundColor` (6-bit backdrop value), `PaletteOffset`.

`IsChrRamTile` is fixed per game: true iff the mapper has no CHR ROM
(PPUC:13-15, DATA:122-136).

### 1.2 When DrawPixel records a pixel

`DrawPixel` runs once per visible dot (scanlines 0-239, dots 1-256), from
`ProcessScanlineImpl` (NPPU:881-883). Buffer offset = scanline*256 + (dot-1) (PPUH:75).

Branch A, taken if rendering is enabled OR v (`_videoRamAddr`) is not in $3F00-$3FFF
(PPUH:79):

- `Grayscale` = (PPUMASK grayscale bit) (PPUH:93; `_paletteRamMask` is 0x30 when grayscale,
  BPPU:158).
- `EmphasisBits` = PPUMASK emphasis, bit0 = red, bit1 = green, bit2 = blue (PPUH:94 with
  BPPU:159: `_intensifyColorBits` = R 0x40, G 0x80, B 0x100, shifted right 6).
- `Tile.PpuBackgroundColor` = palette RAM entry 0 at this dot (PPUH:95; ReadPaletteRam
  BPPU:74-81).
- `Tile.BgColorIndex` = the 2-bit BG pixel from the shifters, or 0 if BG disabled or dot is in
  the masked left column (PPUH:86-89). Note: uses PPUMASK only, not Mesen's "hide BG layer"
  debug option.
- `Tile.BgColor` = backdrop if index 0, else palette RAM[tilePalette + index] (PPUH:98-102).
- `XScroll` = fine X, `TmpVideoRamAddr` = t, both as of THIS dot (PPUH:104-105).
- Sprites: see 1.4.
- BG tile identity: see 1.3. If BG disabled or in the masked left column,
  `Tile.TileIndex = NoTile` (-1) (PPUH:161-175).

Branch B (forced blanking with v in $3F00-$3FFF, PPUH:176-181): only `Tile.TileIndex = NoTile`
and `SpriteCount = 0` are written. (BUG?) `PpuBackgroundColor`, `Grayscale`, `EmphasisBits`,
`TmpVideoRamAddr`, `XScroll` are NOT written, so they keep whatever that buffer held two frames
ago (the buffer is double-buffered). The HD renderer then paints the stale backdrop colour,
not the $3Fxx colour real hardware shows.

### 1.3 Background tile identity

The pipeline mirrors the PPU's tile/palette pipeline. At every fetch-group dot where
`(cycle & 7) == 1` (dots 1,9,...,249 and 321,329) and rendering is enabled,
`StoreTileInformation` runs BEFORE the nametable byte fetch (NPPU:667-683, PPUH:56-66):

    previousTileEx = currentTileEx
    currentTileEx  = nextTileEx
    tileIndex      = DebugReadVram(NametableAddr(v))         (PPUH:61)
    tileAddr       = (tileIndex << 4) | (v >> 12) | PPUCTRL.bgPatternBase
    nextTileEx.OffsetY          = v >> 12                    (fine Y at fetch time)
    nextTileEx.AbsoluteTileAddr = mapper absolute address of tileAddr

This matches the PPU's own `_previousTilePalette / _currentTilePalette / _tile` pipeline
(NPPU:674-675). At each pixel the record uses the previous or current tile according to fine X
(PPUH:83-85): `usePrev = (fineX + ((dot-1) & 7)) < 8`.

Recorded (PPUH:161-173):

- `Tile.TileIndex = AbsoluteTileAddr / 16`.
- If CHR-RAM game: `Tile.TileData` = 16 bytes copied from CHR RAM at
  `AbsoluteTileAddr & ~0xF` AT THE MOMENT THIS PIXEL IS OUTPUT (PPUH:163-165, MAP:998-1005),
  i.e. the current CHR-RAM contents, not the bytes the PPU actually fetched 4-12 dots earlier.
  Layout = standard NES 2bpp tile: bytes 0-7 plane 0 rows 0-7, bytes 8-15 plane 1 rows 0-7.
- `Tile.PaletteColors` (pack version >= 100) =
  `pal[p+3] | pal[p+2]<<8 | pal[p+1]<<16 | pal[0]<<24`, where `p` is the tile's attribute
  palette base (0,4,8,12) and `pal` is RAW palette RAM (`_paletteRam[]`, not the
  mirror-resolving ReadPaletteRam) (PPUH:166-170). So the BG key includes the CURRENT
  BACKDROP colour in its top byte. Versions < 100: top byte 0.
- `Tile.OffsetY` = fine Y captured at fetch (row of the tile, 0-7).
- `Tile.OffsetX = (fineX + ((dot-1) & 7)) & 7` (column within the tile, 0-7).
- `Tile.HorizontalMirroring/VerticalMirroring/BackgroundPriority` are never set for BG
  (constructed false, DATA:127-130).
- (BUG?) `Tile.PaletteOffset = Tile.PaletteOffset` (PPUH:97) is a self-assignment; the BG
  PaletteOffset is never set and stays 0.

Absolute address (MAP:1035-1059): `_chrPages[addr>>8] + (addr & 0xFF)` is located inside CHR
ROM, CHR RAM, mapper RAM or nametable RAM and the OFFSET WITHIN THAT REGION is returned; the
region TYPE is discarded by the HD code. So `TileIndex` = (byte offset into whichever memory the
pattern currently maps to) / 16. For an unmapped page the address is -1, giving
`0xFFFFFFFF/16 = 0x0FFFFFFF`. For CHR-ROM games this is "tile number within the whole CHR ROM",
independent of bank switching. (INFERRED consequence: a mapper that can map both CHR ROM and
CHR RAM/ExRAM into pattern space produces colliding TileIndex values.)

Timing caveats, both visible in the source:
- The CHR bank used for the identity is the mapping at dot (8n+1), when the nametable byte is
  fetched; the PPU's real pattern fetches happen at dots 8n+5 and 8n+7 (NPPU:691-697). A mapper
  that switches CHR between those dots (MMC2/MMC4 latches, MMC5) can make the identity differ from
  the pixels actually shown.
- `DebugReadVram(addr)` is called with `disableSideEffects=false` (PPUH:61, MAP:944-951), so it
  calls `NotifyVramAddressChange` - a side effect on mappers with VRAM-address hooks.

### 1.4 Sprite identity

Per sprite loaded for the next line, `StoreSpriteInformation` is called from `LoadSprite`
(NPPU:738, PPUH:47-54) with the sprite slot index:

    TileAddr         = pattern address of the row to be shown (includes row and, for 8x16,
                       the +16 to the bottom tile) (NPPU:710-720)
    AbsoluteTileAddr = mapper absolute address of TileAddr (CHR mapping at sprite fetch time,
                       dots 257-320, NPPU:920)
    VerticalMirror   = OAM attr bit 7
    OffsetY          = lineOffset = row within the sprite, ALREADY FLIPPED if V-mirrored
                       (0-7, or 0-15 for 8x16) (NPPU:710-714)

At each visible dot `GetPixelColor` sets `_lastSprite` to any sprite whose 8-pixel span covers
the dot (NPPU:831-836), only when `_hasSprite[dot]` and `dot > _minimumDrawSpriteCycle` (left
column sprite masking). `DrawPixel` resets `_lastSprite` to null first (PPUH:77). If
`_lastSprite != null && PPUMASK.spritesEnabled` (PPUH:107), the record collects, in sprite-slot
order (= OAM priority order, slot 0 first), every sprite whose span covers the dot, INCLUDING
sprites whose pixel here is transparent, stopping after 4 (PPUH:108-156):

- `TileIndex = AbsoluteTileAddr / 16` (for 8x16 this is the 8x8 half actually showing).
- `TileData` from CHR RAM at pixel time (CHR-RAM games) (PPUH:115-117).
- `PaletteColors` (v>=100) = `0xFF000000 | pal[q+3] | pal[q+2]<<8 | pal[q+1]<<16` with
  `q = 0x10 + 4*(attr & 3)`, raw palette RAM (PPUH:118-122). v<100: no 0xFF top byte. The 0xFF
  top byte is what `IsSpriteTile()` tests (DATA:70-73).
- `OffsetY` = lineOffset, minus 8 if >= 8 (row within the 8x8 half) (PPUH:123-127).
- `OffsetX = dot - spriteX - 1` (screen column within the sprite, 0-7, NOT flipped)
  (PPUH:110, 129).
- `HorizontalMirroring`, `VerticalMirroring`, `BackgroundPriority` (attr bit 5),
  `PaletteOffset = 0x10 + 4*(attr&3)` (PPUH:130-133, NPPU:727).
- `SpriteColorIndex` = the sprite's 2-bit pixel at this dot (PPUH:135-139);
  `SpriteColor` = backdrop if 0 else palette RAM[q + index] (PPUH:141-145).
- `PpuBackgroundColor` and `BgColorIndex` copied from the BG record (PPUH:147-148).

`SpriteCount` = number collected (0-4). If no sprite covers the dot, `SpriteCount = 0`
(PPUH:157-159). Records beyond 4 overlapping sprites are dropped.

### 1.5 Fields not refreshed

Only the fields listed above are written each frame. When `SpriteCount = 0`, the `Sprite[]`
contents are stale; when `TileIndex = NoTile`, the other BG fields are stale. Code only reads
`Sprite[0..SpriteCount-1]`, and BG fields only after testing NoTile, EXCEPT `BgColorIndex`/
`PpuBackgroundColor` which are always written in branch A.


## 2. Watched-memory snapshot and frame number

### 2.1 Which addresses

Every `memoryCheck`/`ppuMemoryCheck` adds both operands; every `memoryCheckConstant`/
`ppuMemoryCheckConstant` adds operand A (LDR:530, LDR:538). PPU addresses are stored with bit 31
set (`PpuMemoryMarker = 0x80000000`, COND:66; LDR:505-508, 525-526).

### 2.2 When and how

`OnBeforeSendFrame` (PPUC:26-46) is called from `SendFrame` (NPPU:1184), which runs at
scanline 240, dot 0 (NPPU:1412-1416), immediately after the last visible line and before the
VBlank flag/NMI at scanline 241. For each watched address:

- CPU address: `NesMemoryManager::DebugRead(addr)` = the registered handler's `PeekRam(addr)`
  (side-effect free), then active cheats applied (MEM:111-118). Note the HD audio device's own
  registers use the default `PeekRam` which returns 0 (INesMemoryHandler.h), so a memoryCheck on
  $4100-$4106 always sees 0.
- PPU address < $3F00: `mapper->DebugReadVram(addr & 0x3FFF, true)` - pattern tables or
  nametables through the CURRENT (end-of-frame) mapping, no side effects (PPUC:36, MAP:944-951).
- PPU address >= $3F00: `ReadPaletteRam(addr)` = palette RAM [addr & 0x1F] with $10/$14/$18/$1C
  mirrored to $00/$04/$08/$0C (PPUC:33-34, BPPU:74-81). Value is the raw 6-bit entry.

Values are stored as 8-bit in `WatchedAddressValues` (DATA:117).

Consequence: every memory condition in a frame sees ONE value per address - the value at the end
of that frame's visible area. A game that changes a variable mid-frame (e.g. in a sprite-0 or
IRQ split handler) is seen only in its end-of-frame state. A game updating RAM in NMI sees,
during frame N's rendering, the value its NMI wrote at the start of frame N's VBlank... no: the
snapshot is taken at line 240 of frame N, i.e. AFTER frame N's rendering but BEFORE frame N's
own VBlank NMI, so it reflects the NMI of the previous VBlank plus anything the main loop wrote
during frame N's visible lines.

### 2.3 Frame number

`FrameNumber = _frameCount` at the same moment (PPUC:29). `_frameCount` is set to 1 on PPU reset
(NPPU:143) and incremented right after `SendFrame` (NPPU:1417). So the first rendered frame
after reset has FrameNumber 1, incrementing by 1 per frame (u32).


## 3. Conditions

### 3.1 Declaration syntax

    <condition>name,type,arg1,arg2,...

Tokens are split on ',' (LDR:158-161). Name: whitespace trimmed from the right, must be
non-empty, must not contain '!' (LDR:418-420, LDR:451). Every `<condition>` line creates TWO
objects: `name` and `!name` (LDR:160-161, LDR:454-456); `!name` returns the negation of
`name`'s internal result (DATA:193-195). Errors are therefore logged twice.

A condition must be declared BEFORE any tile/background line that uses it - names are resolved
at parse time (LDR:797-802). An unknown name logs an error and is SILENTLY DROPPED from that
line's condition list, which makes the tile LESS restricted (LDR:800-802).

`<ver>` and `<scale>` take effect at the point they appear in the file: version checks and the
tile size (8*scale) use the value current at parse time (LDR:179-191, LDR:389-390). In practice
`<ver>` and `<scale>` must come first.

Pack versions: max supported 109 (PACKH:15, LDR:181-184). With version >= 109,
`checkConstraintEx` violations abort the tag; below 109 they are only logged (LDR:19).

### 3.2 Using conditions

A tile or background line may start with `[c1&c2&...]` (LDR:141-149). Inside the brackets,
spaces/tabs/CR/LF are ignored and '&' separates names (LDR:806-817). Combination is pure AND,
evaluated in list order with short-circuit at the first false (DATA:284-292 for tiles,
PACK:271-277 for backgrounds). There is no OR syntax; OR is expressed by multiple tile lines
with the same key.

### 3.3 Built-in conditions (no `<condition>` line needed)

Created before the file is parsed (LDR:114, LDR:249-261), each with a '!' twin:

    hmirror     tile->HorizontalMirroring                  (COND:107-118)
    vmirror     tile->VerticalMirroring                    (COND:120-131)
    bgpriority  tile->BackgroundPriority (sprite attr 5)   (COND:133-144)

Each returns false if `tile` is null. For BG tiles all three are always false.

`sppalette0..3` (COND:414-426): true iff `tile->PaletteOffset == 0x10 + 4*n` (sprite palette n).
(BUG?) They are only registered `if(_data->Version >= 107)` (LDR:255-260), but
`InitializeGlobalConditions()` runs BEFORE any line of the file is read (LDR:114), when
`Version` is still 0 (DATA:498). So in this source the sppalette conditions are NEVER
registered: any use logs "Condition not found" and the condition is dropped (tile becomes
unconditional for that term). Also note BG tiles have PaletteOffset 0 (see 1.3), so sppalette is
sprite-only by design.

### 3.4 Condition caching

`CheckCondition` (DATA:186-202): if `_resultCache >= 0` return it; else evaluate, apply '!'
negation, and if `_useCache` store the result. `Initialize` resets `_resultCache = -1`
(DATA:179-184); it is called for every condition at the start of each frame's processing
(PACK:289-291). So `_useCache = true` means "evaluated at most once per frame, at the first use,
whatever pixel triggered it".

    cached once per frame:  memoryCheck, memoryCheckConstant (both CPU and PPU variants),
                            frameRange, tileAtPosition, spriteAtPosition
                            (COND:233, 256, 281, 310, 331)
    evaluated on every call: tileNearby, spriteNearby, positionCheckX/Y,
                            originPositionCheckX/Y (COND:152 sets false explicitly),
                            hmirror, vmirror, bgpriority, sppaletteN

The cached ones are position-independent, so caching does not change results.

### 3.5 Tile/sprite comparison (shared by the four tile-identity conditions)

Declaration (LDR:460-494):

    <condition>name,tileAtPosition|tileNearby|spriteAtPosition|spriteNearby,X,Y,TILE,PALETTE[,IGNOREPALETTE]

- X, Y: decimal (`stoi`), may be negative for the Nearby types (LDR:467-468).
- TILE: if exactly 32 characters, 16 hex bytes of CHR data (CHR-RAM style), TileIndex = -1;
  otherwise a tile index, hex for version >= 104, decimal below (LDR:469-481, COND:26-31).
- PALETTE: hex u32 (LDR:482). Must be written the way the key is built: for BG tiles
  `BBP1P2P3` (backdrop in the top byte), for sprites `FFP1P2P3`.
- IGNOREPALETTE: `Y`/`N`, version >= 108 only (LDR:484-490).
- 6-8 tokens accepted (LDR:464-465).
- `PixelOffset = Y*256 + X` is precomputed (COND:22).

Matching a target record (COND:316-325, the same block repeated in each type):

    if TILE is an index (>= 0):
        (target.PaletteColors == PALETTE  or IGNOREPALETTE)
        and (target.TileIndex == TILE  or  FallbackTile(target.TileIndex) == TILE)
    else (CHR data):
        IGNOREPALETTE: target.TileData[0..15] == DATA
        else:          target.PaletteColors (4 bytes) + target.TileData (16) == PALETTE + DATA

The fallback relation is only one hop: the TARGET's fallback is compared to TILE
(see section 6.2 for the fallback table).

Note: a CHR-index condition compares `TileIndex`, which is set for CHR-RAM games too (it is
`AbsoluteTileAddr/16`, the CHR-RAM tile slot). The data form compares the CHR-RAM bytes
captured at pixel time.

### 3.6 tileAtPosition (COND:308-327)

Looks at the BG record of the single pixel `ScreenTiles[Y*256 + X]` of the current frame. X/Y
are NES screen pixel coordinates (0-255, 0-239), independent of overscan cropping and of
scroll. Cached per frame. No bounds check (an out-of-range X/Y reads out of the array).

### 3.7 spriteAtPosition (COND:329-353)

True if ANY of the up-to-4 sprite records at pixel (X,Y) matches (3.5). Records include
sprites whose pixel there is transparent, and `<addition>` sprites inserted earlier the same
frame (section 6.1). Cached per frame.

### 3.8 tileNearby (COND:355-378)

Evaluated for the pixel currently being drawn at screen (x, y):

    pixelIndex = PixelOffset + y*256 + x  =  (y + Y)*256 + (x + X)

then compares the BG record there. Not mirroring-aware. No per-axis bounds check: an X offset
that leaves the row wraps to the adjacent row. Bounds test is
`pixelIndex < 0 || pixelIndex > 61440` (COND:363): (BUG?) index 61440 itself passes and reads
one past the array. Not cached.

### 3.9 spriteNearby (COND:380-412)

    xSign = (tile && tile->HorizontalMirroring) ? -1 : 1
    ySign = (tile && tile->VerticalMirroring)   ? -1 : 1
    pixelIndex = (y + Y*ySign)*256 + x + X*xSign

where `tile` is the record being matched (sprite or BG tile; BG is never mirrored). True if any
sprite record at that pixel matches (3.5). Same bounds issue as 3.8. Not cached.

### 3.10 memoryCheck / ppuMemoryCheck (COND:230-251, LDR:496-540)

    <condition>name,memoryCheck,ADDR_A,OP,ADDR_B[,MASK]
    <condition>name,ppuMemoryCheck,ADDR_A,OP,ADDR_B[,MASK]

- Requires version >= 101 (LDR:498). 5-6 tokens (LDR:499-500).
- ADDR_A, ADDR_B: hex; CPU range <= $FFFF, PPU range <= $3FFF (LDR:503-509, 524-528).
- OP: one of `==` `!=` `>` `<` `<=` `>=` (LDR:591-607). Anything else rejects the tag.
- MASK: hex, only parsed if a 6th token exists AND version >= 103; default $FF (LDR:516-520).
- Evaluation: `a = snap[A] & MASK; b = snap[B] & MASK; return a OP b`, unsigned 8-bit compare
  (COND:236-250). Snapshot = section 2.
- (BUG?) The mask branch runs `checkConstraint(operandB <= 0xFF, "Out of range memoryCheck
  mask")` BEFORE operand B is validated as an address (LDR:518). So a two-address memoryCheck
  WITH a mask is rejected entirely unless ADDR_B <= $00FF.

### 3.11 memoryCheckConstant / ppuMemoryCheckConstant (COND:253-274)

    <condition>name,memoryCheckConstant,ADDR,OP,VALUE[,MASK]

- VALUE: hex, must be <= $FF (LDR:534).
- Evaluation: `(snap[ADDR] & MASK) OP VALUE`. VALUE itself is NOT masked (COND:261-262).
- Same token/mask rules as 3.10.

### 3.12 frameRange (COND:276-306, LDR:561-581)

    <condition>name,frameRange,A,B

- Version 101: A, B hex. Version >= 102: decimal. Both 0..65535.
- Evaluation: `FrameNumber % A >= B`. True for the last (A-B) frames of every A-frame cycle.
- (BUG?) A = 0 is accepted by the parser and causes a division by zero.
- Cached per frame.

### 3.13 positionCheckX / positionCheckY (COND:146-189, LDR:543-558)

    <condition>name,positionCheckX,OP,VALUE

- Version >= 108. VALUE decimal (`stoi`), <= 65535.
- `val = x` (or `y`) of the pixel currently being drawn, NES screen coordinates, as u32; then
  `val OP VALUE`.

### 3.14 originPositionCheckX / originPositionCheckY (COND:176-177)

    X: val = hmirror ? x - (7 - OffsetX) : x - OffsetX
    Y: val = vmirror ? y - (7 - OffsetY) : y - OffsetY

computed as u32 (negative wraps to a huge number, so `<`/`<=` against small values fail at the
left/top edges). This yields the screen coordinate of the 8x8 tile's top-left corner:
- BG tile: OffsetX is the column under the pixel and OffsetY the fine Y at fetch, so `x-OffsetX`
  = the screen x of the tile column start, `y-OffsetY` = the screen y of the tile's row 0
  (assuming no mid-tile vertical scroll change).
- Sprite: OffsetX = dot-spriteX-1 so (with mirroring compensation) the result is spriteX+... the
  screen X of the sprite's left edge; OffsetY is the DATA row (already flipped), the `7-OffsetY`
  term converts back to the screen row. For 8x16 sprites the origin is that of the 8x8 HALF
  being shown, not of the whole sprite.
Requires a non-null tile (crashes with null; backgrounds cannot use it, see 4.2).

### 3.15 Effect on the BG-tile 8-pixel cache

Tiles whose condition list contains spriteNearby, any position check, or a tileNearby whose X
or Y is not a multiple of 8 get `ForceDisableCache` (LDR:334-352). See 3.17.

### 3.16 Choosing among several definitions for the same key

`InitializeHdPack` (LDR:846-863) builds `TileByKey`: for every `<tile>` in FILE ORDER, it is
appended to `TileByKey[exact key]`, and, if its DefaultTile flag is `Y`, also to
`TileByKey[default key]` where the default key is the same key with `PaletteColors =
0xFFFFFFFF` (DATA:23-32).

`GetMatchingTile` (PACK:442-480):

    1. list = TileByKey[exact key of the record]
    2. if no such key:
         f = FallbackTile(record.TileIndex)         (only non-empty for CHR-ROM games)
         if f >= 0:
             record.TileIndex = f                   (MUTATES the frame record)
             list = TileByKey[exact key]  else  TileByKey[default key]
             if still none: record.TileIndex = original
         if still none: list = TileByKey[default key of the record]
    3. if a list was found: return the FIRST entry (file order) whose conditions all pass;
       else return null

Crucial: the default key and the fallback are tried ONLY when the exact key does not exist at
all. If the exact key exists but none of its entries' conditions pass, the result is null - it
does NOT fall through to the default tile. Likewise, once the default-key list is chosen, the
first passing entry wins, and if none pass the result is null.

(BUG?/side effect) When the fallback hit is used, the record's TileIndex stays rewritten to the
fallback index. Conditions that later read that pixel's record (tileAtPosition etc.) or later
lookups for the same record see the fallback index.

Null result means "no HD replacement": the original NES pixel is drawn (section 5.3), except
that for BG tiles `disableOriginalTiles` suppresses it.

### 3.17 BG tile lookup cache ("disableCache")

Only BG tile lookups are cached; sprite lookups are done fresh for every sprite record of every
pixel (PACK:523, PACK:553).

`GetCachedMatchingTile` (PACK:418-439):

    at each line start: _useCachedTile = false       (PACK:240)
    _scrollX (per line) = coarseX(t)*8 | fineX | (t.ntX ? 256 : 0), taken from the line's
                          first pixel record (PACK:239)
    per pixel x: if ((_scrollX + x) & 7) == 0: _useCachedTile = false
                 if _useCachedTile: reuse previous result (may be null)
                 else: result = GetMatchingTile(...), and if no ForceDisableCache tile was
                       ENCOUNTERED in the key's list before the match and the cache is enabled,
                       cache it for following pixels

So, with caching on (default), conditions on a BG tile are evaluated once per 8-pixel tile span
per scanline, at the tile's leftmost visible pixel on that line (or at the line's first
rendered pixel, i.e. overscan.Left). `x`, `y` passed to conditions are that first pixel's
coordinates. The tile boundary is computed from the line's starting scroll, so mid-line fine-X
changes misalign it (option `disableCache` turns caching off, LDR:402-403, PACK:298).

Note the "encountered before the match" detail: the `ForceDisableCache` flag of entries after
the matching entry are never looked at (PACK:465-475).


## 4. Backgrounds

### 4.1 Parsing (LDR:609-700)

    [conds]<background>FILE,BRIGHTNESS[,HSCROLLRATIO[,VSCROLLRATIO[,PRIORITY[,LEFT,TOP[,BLENDMODE]]]]]

- 2-8 tokens (LDR:611-612).
- FILE: PNG; the same file name reuses one decoded bitmap (LDR:615-629).
- BRIGHTNESS: float; stored as `(int)(f*255)` for version >= 105, `(uint8)(f*255)` (wraps)
  below (LDR:634-638).
- HSCROLLRATIO, VSCROLLRATIO: float, default 0 (version >= 101) (LDR:663-669).
- PRIORITY: version >= 106: integer 0-39 (LDR:672-674). Version 102-105: `Y` -> 0, `N` -> 10
  (LDR:675-677). Default 10.
- LEFT, TOP: both required together (token count > 6), integers clamped to >= 0, version >= 105
  (LDR:679-683). Units: NES pixels (multiplied by scale when used).
- BLENDMODE: `Alpha` (default), `Add`, `Subtract` - exact enum names, case-sensitive,
  version >= 107 (LDR:685-693, DATA:401-406).
- Allowed conditions: tileAtPosition, spriteAtPosition, memoryCheck(+ppu),
  memoryCheckConstant(+ppu), frameRange and their '!' twins. Any other condition (including
  hmirror, tileNearby, positionCheck...) logs an error and is dropped (LDR:647-661).
- Stored in `BackgroundsByPriority[PRIORITY]`, in file order (LDR:696).

### 4.2 Per-frame selection (one background per priority level)

`OnBeforeApplyFilter` (PACK:300-311) for each of the 40 priority levels calls `GetLayerIndex`
(PACK:268-284): the FIRST background (file order) at that priority whose conditions all pass,
evaluated as `CheckCondition(0, 0, nullptr)`. At most one background per priority level is
drawn per frame. The selected ones are packed per layer (priority/10) in ascending priority.

This is evaluated once per frame, before any pixel is drawn, against the completed frame's
record and memory snapshot (section 0).

### 4.3 Layer order (PACK:497-566, PACKH:49-53)

For each output pixel, in this order (each step paints over the previous):

    1. backdrop: palette[this pixel's PpuBackgroundColor]                    (PACK:510)
    2. backgrounds with priority  0- 9 (ascending)                           (PACK:512-514)
    3. behind-BG sprites (attr bit 5 = 1), back to front                     (PACK:516-531)
    4. backgrounds with priority 10-19                                       (PACK:533-535)
    5. BG tile (HD tile, or the SD BG pixel)                                 (PACK:537-544)
    6. backgrounds with priority 20-29                                       (PACK:546-548)
    7. front sprites (attr bit 5 = 0), back to front, with the NES masking   (PACK:550-561)
    8. backgrounds with priority 30-39                                       (PACK:563-565)

The default priority 10 is therefore "behind the BG tiles, in front of behind-BG sprites".
Priority 0 = behind everything except the backdrop. Priority 30+ = in front of everything.

Sprite sub-order (both passes): records are visited from `SpriteCount-1` down to 0, i.e.
lowest-priority OAM sprite first, highest (slot 0) last, so slot 0 ends on top. Step 3 also
computes `lowestBgSprite` = the smallest index k of a behind-BG sprite whose SD pixel is opaque
(SpriteColorIndex != 0). Step 7 only draws front sprites with index `< lowestBgSprite`
(PACK:552). This reproduces the NES rule that an opaque higher-priority behind-BG sprite hides
lower-priority front sprites, decided from SD pixels, not from HD alpha.

### 4.4 Scroll math (OnLineStart, PACK:237-265)

Computed once per scanline from that line's FIRST pixel record (x = 0, ScreenTiles[y*256]),
which holds the PPU `t` register and fine X as they were when x=0 was output:

    scrollX = (t & 0x1F)*8 | fineX | (t & 0x400 ? 256 : 0)                    0..511
    scrollY = ((t & 0x3E0) >> 2 | (t & 0x7000) >> 12) + (t & 0x800 ? 240 : 0)
              = coarseY*8 + fineY + (ntY ? 240 : 0)                            0..495

Note this uses `t` (TmpVideoRamAddr), not `v`. (INFERRED consequence: for split-screen games
the background follows the t value at the start of each line, which is the last value written
via $2005/$2006, not necessarily what the PPU is rendering with.)

For each active background:

    BgScrollX = (int)(scrollX * HSCROLLRATIO)     (C++ float->int truncation toward zero)
    BgScrollY = (int)(scrollY * VSCROLLRATIO)

    if  y >= -BgScrollY  and  (y + TOP + BgScrollY + 1)*scale <= PNG.height:
        BgMinX = -BgScrollX
        BgMaxX = PNG.width/scale - LEFT - BgScrollX - 1          (integer division)
    else:
        BgMinX = BgMaxX = -1                                     (line not drawn)

Per pixel x, the background is drawn only if `BgMinX <= x <= BgMaxX` (PACK:486), and the source
HD pixel block is at (DrawCustomBackground, PACK:137-138):

    src = PNG[ ((TOP + y + BgScrollY)*scale) row, ((LEFT + x + BgScrollX)*scale) column ]

reading a scale x scale block. No wrapping or tiling: outside the image is transparent.

### 4.5 Brightness, alpha and blending

PNG decoding premultiplies alpha for every pixel with alpha < 255 (DATA:246-257):
`c = ((alpha+1) * c) >> 8` per channel. (Pixel format is 0xAARRGGBB as u32, INFERRED from
`PixelData[i] < 0xFF000000` and byte indexing [3] = alpha on little-endian.)

AdjustBrightness (PACK:105-113), applied per pixel to the premultiplied colour when
brightness != 255:

    c' = min(255, (brightness * (c + 1)) >> 8)     per RGB channel; alpha unchanged

BlendColors (PACK:83-102), `in` premultiplied source, `out` current framebuffer:

    Alpha:    out = in + (((256 - a) * out) >> 8)    with (256-a) stored in a uint8
    Add:      out = min(255, in + out)
    Subtract: out = max(0, out - in)
    result alpha forced to 0xFF

Background drawing (PACK:133-175):
- Alpha mode: alpha 255 -> overwrite; alpha 1..254 -> blend; alpha 0 -> skip.
- Add/Subtract modes: always apply (alpha 0 pixels are black after premultiply, so they add or
  subtract nothing).

HD tile drawing (DrawTile, PACK:177-234):
- A tile whose image is fully transparent (all alpha 0) draws nothing at all - and the caller
  then also does not draw the SD pixel, so it effectively ERASES the original tile/sprite
  (PACK:180-182, PACK:537-544).
- Otherwise per pixel: if the tile has no transparent pixels, or this pixel's alpha is 255:
  overwrite (with brightness applied); else if alpha > 0: Alpha blend; alpha 0: skip
  (PACK:210-216). Tiles always use Alpha blending; there is no blend mode for tiles.
- Brightness uses the same formula as backgrounds.

Interaction with the NES backdrop: backgrounds have no special relationship with the backdrop
colour. Step 1 always paints the backdrop first; a background replaces it only where the
background's alpha covers it. The BG tile step paints over backgrounds of priority < 20 with:
the HD tile (its transparent areas reveal whatever is below), or, if no HD tile, the SD BG
colour ONLY where BgColorIndex != 0 (SD colour 0 is "transparent", so the background shows
through BG colour-0 pixels).

### 4.6 HD tile pixel addressing (DrawTile)

For the record's OffsetX/OffsetY the HD tile's scale x scale sub-block is selected:

    col = hmirror ? 7 - OffsetX : OffsetX
    start = (OffsetY*scale)*(8*scale) + col*scale
    hmirror: sub-block read right-to-left
    vmirror: sub-block rows read bottom-to-top (OffsetY is already the flipped data row)

(PACK:184-198.) Tile image size is 8*scale square, source position X,Y in HD pixels inside the
`<img>` PNG (LDR:330-331, LDR:388-390).


## 5. Per-pixel rendering details not covered above

### 5.1 Palette

If `palette.dat` exists in the pack and is exactly 192 bytes (64 RGB triplets) it is used,
`0xFF000000 | R<<16 | G<<8 | B` (LDR:830-844, PACK:293-294); otherwise Mesen's configured 2C02
palette (PACK:296). Lookup is always `palette[6-bit value]`.

### 5.2 Grayscale and emphasis (PACK:589-640)

Applied per SCANLINE after the line is drawn, using the flags of pixel x=0 of that line
(ScreenTiles[i*256], PACK:585), regardless of overscan:
- Grayscale: every RGB pixel -> (r+g+b)/3 in all channels (integer). This is NOT the hardware
  `& 0x30` palette masking.
- Emphasis: per set bit multiply its channel by 1.1 and the other two by 0.9 (compounding when
  several bits are set), clamp to 255, alpha 0xFF.

### 5.3 SD fallbacks

- BG: HD tile found -> DrawTile; else if `disableOriginalTiles` not set and BgColorIndex != 0
  -> palette[BgColor] (PACK:537-544). With `disableOriginalTiles`, BG tiles without an HD tile
  show only backdrop/backgrounds.
- Sprites: HD tile found -> DrawTile; else if SpriteColorIndex != 0 -> palette[SpriteColor]
  (PACK:523-528, 553-558). `disableOriginalTiles` does NOT affect sprites.
- A BG pixel with `TileIndex == NoTile` (BG disabled/masked) skips the lookup (PACK:504-506).

### 5.4 Overscan and output size

`<overscan>TOP,RIGHT,BOTTOM,LEFT` (note the order) - decimal NES pixels (LDR:284-293). If
present it overrides the emulator's overscan setting (VF:38-45). Output is
`(256-L-R)*scale x (240-T-B)*scale` (VF:27-36). Only lines T..239-B and columns L..255-R are
processed (PACK:576-583); conditions still read the full 256x240 record.

### 5.5 Scale

`<scale>N`, 1..10 (LDR:185-191). Each NES pixel becomes an N x N block (PACK:116-131).


## 6. Additions, fallbacks, options, patches, other directives

### 6.0 All directives accepted by the loader (LDR:151-201)

`<tile>`, `<background>`, `<condition>`, `<img>`, `<addition>`, `<fallback>`, `<bgm>`,
`<sfx>`, `<ver>`, `<scale>`, `<overscan>`, `<patch>`, `<options>`. Any other line is silently
ignored. A line may be prefixed by `[conditions]`, but only `<tile>` and `<background>` use
them (the parsed list is ignored for other tags). `<img>FILE` appends a PNG; its index is its
0-based position among `<img>` lines (LDR:221-233). A `<img>` that fails to load aborts the
whole pack (LDR:164-166).

`<tile>` (version >= 100, LDR:314-393):

    <tile>IMGINDEX,TILE,PALETTE,X,Y,BRIGHTNESS,DEFAULT[,CHRBANKID,TILEINDEX]

TILE as in 3.5 (32 hex chars = CHR-RAM data, else index: decimal for v <= 102, hex for >= 103,
LDR:263-282). CHRBANKID and TILEINDEX only for CHR-RAM tiles and only informational (bank id is
used by the pack builder; TileIndex is not part of a CHR-RAM key, DATA:34-54). DEFAULT `Y`/`N`.

### 6.1 `<addition>` (LDR:702-721, PACK:316-415)

    <addition>TILE,PALETTE,OFFSETX,OFFSETY,ADDTILE,ADDPALETTE[,IGNOREPALETTE]

Version >= 107; IGNOREPALETTE version >= 108. TILE/ADDTILE parsed as in 6.0. At startup
(PACK:30-39, CleanupInvalidRules) an addition is kept only if ADDTILE's kind (CHR-RAM data vs
index) equals `mapper->HasChrRam()`. That is NOT the rule tile keys use (`!HasChrRom()`, 3.x):
a cart can have both CHR-ROM and CHR-RAM, and then every index-keyed addition is removed while
index-keyed tiles still work. Measured case: Metroid patched by the a034 pack's mmm.ips has an
NES 2.0 header with 128 KB CHR-ROM and 8 KB CHR-RAM, so all 1987 additions are dropped.
HasChrRam is `_chrRamSize > 0` (BaseMapper.cpp:524): NES 2.0 header byte 11 low nibble (size
64 << n, 0 = none; no database lookup for NES 2.0); with no CHR-ROM, CHR-RAM is assumed (mapper
default or 8 KB); an iNES 1.0 CHR-ROM cart gets CHR-RAM only from Mesen's game database or the
mapper's own default, which a converter cannot see.

Runtime (once per frame, inside `OnBeforeApplyFilter`, BEFORE any pixel is drawn and after the
condition caches are reset, PACK:313): the frame record is MODIFIED by inserting extra sprite
records.

For every pixel of every line, for every existing sprite record at that pixel, and (on selected
lines) for the BG record, find the additions for that record's key (cached forever in
`_additionalTilesByKey`, PACK:360-395). An addition matches the record if (PACK:379-391):

    not IGNOREPALETTE and record key == TILE key             (key equality, 3.16 rules)
    or IGNOREPALETTE and record default key == TILE default key
    or (CHR-ROM game with fallbacks) and (IGNOREPALETTE or record.Palette == PALETTE)
       and FallbackTile(record.TileIndex) == TILE.index

For each match, at target = (x + (hmirror ? -OFFSETX : OFFSETX), y + (vmirror ? -OFFSETY :
OFFSETY)), if inside 256x240 and the target has fewer than 4 sprite records, append a record
that is a copy of the SOURCE record (same OffsetX/OffsetY, mirroring, BackgroundPriority,
PaletteOffset, BG fields) with `SpriteColorIndex = 0` and TileIndex/PaletteColors/TileData
replaced by ADDTILE/ADDPALETTE (PACK:397-415).

Consequences:
- Because the copy is done pixel by pixel, an 8x8 source produces an 8x8 shifted image of the
  added tile at the offset.
- Appended records have the lowest priority at their pixel (drawn first, behind existing
  sprites) and, with SpriteColorIndex 0, never draw an SD fallback and never set
  `lowestBgSprite`. If no HD tile matches the added key, nothing is drawn.
- An addition from a BG tile becomes a "sprite" record with BackgroundPriority false, i.e. it
  is drawn in the front-sprite pass (step 7). Its PALETTE must be whatever ADDPALETTE says.
- Records are visible to spriteAtPosition/spriteNearby of the same frame (conditions are
  evaluated later, during drawing).
- Insertion proceeds in raster order and reads `SpriteCount` live, so an added record at a pixel
  not yet visited can itself trigger additions (chaining).
- Records beyond 4 per pixel are silently dropped.
- BG records are only scanned on "processed" lines (PACK:329-357): a line is processed if
  `(yScroll + y) & 7 == 0` (first line of a tile row) or `t` changed since the last processed
  line, or the previous processed line found a BG addition. An optimisation; for normal scrolling
  it scans every line of any tile row containing a BG addition.
- (BUG?) `HdPpuPixelInfo& lineFirstPixel = ScreenTiles[0]` is a REFERENCE, and each line does
  `lineFirstPixel = ScreenTiles[y << 8]` (PACK:324, 330), which COPIES line y's first-pixel
  record over pixel (0,0). After the loop pixel (0,0) holds line 239's x=0 record. This only
  happens when at least one `<addition>` SURVIVES the startup cleanup above
  (`ProcessAdditionalSprites` returns at once on an empty list). It corrupts pixel (0,0) and line 0's
  scroll/grayscale/emphasis (all read from pixel (0,0)), and any condition targeting (0,0).

### 6.2 `<fallback>` and `automaticFallbackTiles` (LDR:723-729, PACK:41-79)

    <fallback>TILEINDEX,FALLBACKINDEX     (both hex, version >= 107)

The fallback table maps a CHR-ROM tile index to another. It is built only for CHR-ROM games
(PACK:46); for CHR-RAM games it is always empty.

`automaticFallbackTiles` option (PACK:47-73): for every `<tile>` line, its tile's 16 CHR-ROM
bytes (read at `TileIndex*16`) are put in a set keyed by the 16 data bytes (first definition
wins on identical data). Then every CHR-ROM tile i whose 16 bytes are not all zero and equal the
data of some defined tile with a different index gets `fallback[i] = that index`. Explicit
`<fallback>` lines are applied afterwards and override (PACK:75-77).

Fallback is used in: the main tile lookup (3.16), the four tile-identity conditions (3.5), and
addition matching (6.1). Single hop only.

### 6.3 `<options>` (LDR:395-414)

Comma-separated tokens:

    disableSpriteLimit      NoSpriteLimit=1          see section 7.1
    alternateRegisterRange  AlternateRegisterRange=2 audio registers move, see section 8.1
    disableCache            DisableCache=8           turns off the BG 8-pixel cache (3.17)
    disableOriginalTiles    DontRenderOriginalTiles=16  BG tiles without HD match not drawn
    automaticFallbackTiles  AutomaticFallbackTiles=32   see 6.2
    disableContours         accepted, does nothing (removed feature)

Any other token logs "Invalid option".

### 6.4 `<patch>` (LDR:295-312, CON:224-228)

    <patch>FILE,SHA1

SHA1 must be 40 hex chars, upper-cased on load. At ROM load, if the ROM FILE's SHA1 equals one
of the listed hashes, `romFile.ApplyPatch(patchFile)` is applied before the ROM is parsed.
INFERRED (VirtualFile is not in the extracted tree): the hash is of the whole ROM file as loaded
(including the iNES header), and ApplyPatch auto-detects IPS / UPS / BPS by header, patching the
whole file (so it can change header, PRG and CHR). This is a load-time ROM substitution; there
is no runtime component.

MEASURED 2026-10-15 (supports the whole-file reading): the Metroid packs list the same
`mmm.ips` under FIVE hashes. `ECF39EC5…` is the SHA1 of the No-Intro `Metroid (USA).nes` body
under a plain iNES 1.0 header (`4E 45 53 1A 08 00 10 00 00…`) and of neither the body alone;
`2080059B…` is the same body with its No-Intro NES 2.0 header. So the hash covers the 16 header
bytes, which is exactly why the author lists one entry per known dump/header variant. A pack
that lists only one header variant will not match a ROM carrying another, in Mesen either.
Patching
that ROM turns Metroid into a 256 KB PRG / 128 KB CHR-ROM cart, which is why those packs are
keyed by CHR-ROM tile index even though stock Metroid has CHR-RAM. The Zelda II packs'
`Revamp.ips,08FA60F2…` matches the No-Intro `Zelda II - The Adventure of Link (USA).nes` file
as it is.


## 7. Sprite limit and the per-pixel 4-sprite cap

### 7.1 disableSpriteLimit

Sets `_forceRemoveSpriteLimit`, which makes BOTH `RemoveSpriteLimit()` and
`UseAdaptiveSpriteLimit()` true (PPUC:17, PPUH:44-45). Effect, in `LoadExtraSprites`, called at
dot 320 of every rendering scanline (NPPU:930-932, NPPU:765-803):

- Only if the normal evaluation found exactly 8 sprites for the next line.
- Adaptive check: walk all 64 OAM entries in OAM order; consider those in range of the current
  scanline (`scanline >= Y && scanline < Y + 8/16`); count runs of CONSECUTIVE in-range entries
  with identical (Y,X). If the final run, or the longest earlier run, is >= 8, extra sprites are
  NOT loaded (a game stacking 8+ identical sprites deliberately to mask). Note the final run is
  compared without being folded into the max (NPPU:790), harmless.
- Otherwise: Mesen re-scans OAM itself, starting at the entry after the last sprite the normal
  evaluation accepted (`_lastVisibleSpriteAddr + 4`) and wrapping round until the first
  (`_firstVisibleSpriteAddr`), and loads every in-range sprite as an extra slot
  (`LoadSprite(..., extraSprite=true)`, pattern bytes read with DebugReadVram so there are no
  mapper side effects), incrementing `_spriteCount` (up to 64 slots, `_spriteTiles[64]`).

The source data is primary OAM (`_spriteRam`) as it is at dot 320 of the evaluating scanline.
Extra sprites participate in normal (SD) pixel output too, but never in sprite-0 hit or the
overflow flag (those were already decided). They also get `StoreSpriteInformation`, so they
appear in HD records.

### 7.2 Per-pixel cap

Regardless of the limit, the HD record holds at most 4 overlapping sprites per pixel (1.4) and
additions also respect 4 (6.1). With disableSpriteLimit, more than 8 sprites per LINE can be
shown, but at most 4 can stack on one PIXEL in the HD output.


## 8. Audio (`<bgm>`, `<sfx>`, HdAudioDevice)

### 8.1 Registers

The device is registered as a CPU I/O handler for every HD pack (CON:195-198) with
`SetAllowOverride` (AUD:91), so it takes these addresses away from any mapper.

Default (AUD:99-101): read AND write handler on $4100-$4106.

Alternate (`alternateRegisterRange`, AUD:93-98): WRITE handlers at $3002, $3012, $3022, $3032,
$3042, $3052, $3062 (mirrors of PPU $2002, where writes are harmless on real hardware); READ
handlers at $4018 and $4019 only.

Write register number (AUD:113): `addr > $4100 ? addr & $F : (addr & $F0) >> 4`, so
$4100->0, $4101..$4106->1..6, $3002->0 ... $3062->6.

    reg 0  Playback options. Bit 0 = loop BGM. Takes effect immediately on the current BGM's
           loop flag and on later BGM plays (AUD:119-122, MIX:27-35, MIX:86).
    reg 1  Playback control (AUD:129, AUD:77-86):
             bit 0: paused flag := bit 0 (a LEVEL, not a toggle, despite the comment; writing
                    any value with bit 0 = 0 un-pauses). Pause affects BGM only.
             bit 1: stop BGM (discards it)
             bit 2: stop all SFX
    reg 2  BGM volume 0-255, immediate (AUD:133-136)
    reg 3  SFX volume 0-255, immediate, all SFX share it (AUD:140-143)
    reg 4  album number 0-255, no immediate effect (AUD:147)
    reg 5  play BGM: track id = album*256 + value (AUD:151)
    reg 6  play SFX: id = album*256 + value (AUD:155, AUD:68)

Read register (AUD:159-182), selected by `addr & 7` ($4100-$4106 -> 0..6; alternate $4018 -> 0,
$4019 -> 1):

    0  status: bit 0 = BGM playing (exists and not paused, MIX:62-65)
               bit 1 = at least one SFX playing (MIX:67-70)
               bit 2 = `_trackError`
    1  revision = 1
    2  'N' ($4E)
    3  'E' ($45)
    4  'A' ($41)
    5,6  0

(BUG?) `_trackError` is assigned the RETURN VALUE of the last play command (AUD:151, 155):
- BGM: `PlayBgmTrack` returns TRUE ON SUCCESS (AUD:52-64), so bit 2 = 1 after a successful BGM
  start and 0 if the track is undefined or fails to open - inverted relative to its name.
- SFX: `PlaySfx` returns `!Play(...)` when the id exists (true = failed to open) and false when
  the id is undefined (AUD:66-75). So bit 2 = 1 only for a defined SFX whose file fails.
The value is overwritten by each reg 5/6 write.

Other behaviour:
- Playing an undefined BGM id does NOT stop the current BGM (only logs) (AUD:60-62); a defined
  one replaces it (MIX:91). Starting a BGM does not clear the paused flag.
- SFX are unlimited and simultaneous (MIX:88-89); they never loop (MIX:86).
- Reads of the registers via DebugRead/PeekRam return 0 (2.2).
- Power-on values: album 0, options 0, volumes 128 (AUD:15-23). OggMixer::Reset (called by the
  sound system, INFERRED on audio reset) clears BGM/SFX, sets volumes 128, options 0, unpaused
  (MIX:16-25). There is no NES reset hook in HdAudioDevice.

### 8.2 Pack lines (LDR:731-788)

    <bgm>ALBUM,TRACK,FILE[,LOOPPOSITION]
    <sfx>ALBUM,TRACK,FILE

ALBUM, TRACK decimal 0-255; id = ALBUM*256 + TRACK (LDR:731-751). FILE must exist (checked at
load). LOOPPOSITION decimal u32, in sample frames at the OGG file's own sample rate (used with
stb_vorbis_seek, OGG:31-36, 73). Only OGG Vorbis is supported (decoded by stb_vorbis, OGG:3).

### 8.3 Decoding, looping, mixing

- Each play opens the OGG from memory (OGG:22-45). Loop position is ignored (treated as 0) if
  >= the stream length in samples (OGG:31-36).
- Decoding always requests 2-channel interleaved 16-bit (OGG:70). INFERRED from stb_vorbis
  behaviour: mono files are duplicated to both channels.
- Per mix call (OGG:64-87): enough OGG samples are decoded to produce `sampleCount` output
  frames (`needed * oggRate / outRate + 2`); if the stream ends inside that chunk: looping ->
  seek to LOOPPOSITION and decode the remainder; not looping -> mark done (removed after this
  call). Then resampled from the file's rate to the output rate (Mesen's resampler, INFERRED
  Hermite/linear; not in the tree).
- Volume and mix (OGG:83-86): `out[i] = clamp(out[i] + (sample * volume / 255), -32768, 32767)`
  (integer, truncating). Volume 255 = unity, default 128 = ~0.502.
- The mixer is an "audio provider" registered with the emulator's SoundMixer (AUD:24). It adds
  into the final stereo output buffer at the HOST output sample rate (`MixAudio(out,
  sampleCount, sampleRate)`, MIX:107-121), i.e. after/alongside the NES APU mix, not through the
  APU or its filters (INFERRED from the provider interface; SoundMixer not in the tree).
- Pause: BGM is not advanced and not mixed while paused (MIX:109).
- Order: BGM first, then each SFX, each summed with clamping after each source (so clamping is
  per-source cumulative, not at the end).
- Savestates store a stb_vorbis FILE byte offset and on load pass it to `stb_vorbis_seek`, which
  takes a SAMPLE index (AUD:36, AUD:43-45, OGG:38-40). (BUG?) Irrelevant for hardware.


## 9. Tile keys (recap of the hardware-relevant parts only)

- CHR-ROM key: (TileIndex, PaletteColors). Hash `TileIndex ^ PaletteColors` (DATA:34-41) is
  only for the host hash table; equality is exact on both fields (DATA:47-54).
- CHR-RAM key: (PaletteColors, TileData[16]) - 20 bytes compared with memcmp; TileIndex ignored
  (DATA:49-50). Mesen's hash: over the 20 bytes as five little-endian u32 chunks,
  `h += chunk; h = rotl(h, 2)` starting from 0 (DATA:56-68). Any hash works in hardware; only
  equality matters.
- PaletteColors for BG includes the backdrop colour in bits 31-24 (v>=100); for sprites bits
  31-24 = 0xFF. Bits 23-16 / 15-8 / 7-0 = palette entries 1 / 2 / 3.
- Default key: same with PaletteColors = 0xFFFFFFFF; only tiles flagged DEFAULT=Y are reachable
  through it, and only when the exact key is absent (3.16).


## 10. Summary of evaluation granularity (for hardware planning)

    Memory snapshot (memoryCheck*)   once per frame, at scanline 240 dot 0
    Frame counter (frameRange)       once per frame, value = frames since reset (starts 1)
    tileAtPosition/spriteAtPosition  once per frame, read the finished frame's pixel record
    Background selection             once per frame (first passing per priority level)
    Background scroll                once per scanline, from t + fine X recorded at x = 0
    <addition> insertion             once per frame, before drawing, rewrites pixel records
    BG tile HD lookup + conditions   once per 8-pixel tile span per scanline (cache on), else
                                     per pixel; conditions spriteNearby / position / off-grid
                                     tileNearby force per-pixel for keys that use them
    Sprite HD lookup + conditions    per pixel, per sprite record (max 4 per pixel)
    tileNearby/spriteNearby          relative to the pixel being evaluated, read the finished
                                     frame's records (so they may look at pixels BELOW/RIGHT of
                                     the current one - future pixels in raster order)
    Grayscale/emphasis               per scanline, from x = 0's flags

The last point under tileNearby/spriteNearby/tileAtPosition is the main structural difficulty
for a raster-order FPGA implementation: Mesen renders a frame AFTER it is complete, so a
condition may reference any pixel of the same frame, including ones not yet output by the PPU.
A hardware implementation that renders in lockstep with the PPU must either delay output by up
to a frame, or restrict/approximate conditions that look forward (positive Y offsets, or
positions later in raster order), e.g. by using the previous frame's record for those.
