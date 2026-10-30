# Lua support for HD packs - scope (2026-10-29)

Question asked: can the core run the Lua scripts some HD packs carry, so Dragon Warrior III works?

## Who needs it

One game. Across all 96 downloads in the corpus, the only Lua files are the two Dragon Warrior III
packs: a064 (released v0.5.1) and a065 (the git repo of the same mod). Every other pack is plain
hires.txt plus a patch.

## What the DQ3 script does (Dragon Warrior III Delocalized v1.12.lua, 665 lines)

1. At load, overwrites the game's own music pointer table in PRG ROM (0x78010-0x7815F) with a
   silence pattern, so the original music never plays.
2. Watches the game READ that pointer table (a memory callback on PRG ROM). Each pointer read
   means "the game is starting track X"; the script maps it to an HD track and writes the HD
   audio registers $4100-$4105 (loop, stop, volume, album, track). Jingles (level up, inn,
   victory) play once; the field tracks loop.
3. Every frame, reads game RAM (map id, battle flag, position) and writes five spare RAM bytes,
   $7C5-$7C9: minimap on/off, battle backdrop, current tile, underworld flag, night shadow /
   current track. The pack's hires.txt conditions test those bytes, so all of the HD graphics
   choices (backdrops, night, minimap image) depend on them.
4. Select (short press) toggles the minimap; it draws a blinking 4x4 player marker on top of the
   HD picture at a position computed from the player's coordinates.
5. Controller 2 buttons run menu macros (Search, Magic) by feeding fake controller-1 input.
6. Select (long press) shows a debug text overlay.

Without 2 and 3 the pack shows original graphics in most places and plays no HD music, which is
why it reads as not complete.

## Option A - a Lua runtime in the core (general)

What it would take, each a new piece of work:
- A Lua VM on the ARM side (the HD pack daemon already runs there), with a frame clock tied to
  the NES frame, and the event types inputPolled, startFrame, endFrame.
- emu.read: from the CPU RAM / PRG-RAM shadows the condition engine already keeps.
- emu.write to NES RAM: a new ARM-to-FPGA write path into live CPU RAM (none exists).
- emu.write to $4100-$4105: a local call into the ARM audio engine (easy).
- emu.write to PRG ROM at load: the staged ROM is in DDR3 before the core runs, so the ARM can
  patch it (easy, same place the pack's <patch> is applied).
- addMemoryCallback on PRG ROM reads: an FPGA trap that watches the CPU's PRG fetches for an
  address range, maps bank + address to a PRG offset, and signals the ARM. New hardware on a
  chip that was at 102% of its ALMs before the romfix move to ARM.
- getInput / setInput: read and override the controller words between hps_io and the core.
- drawRectangle / drawString on a script layer: a new overlay plane composited over the HD
  picture.
- Only the subset DQ3 uses, sandboxed (no file or OS access), since the script comes with a pack.
Size: a multi-week project with FPGA, ARM and simulation work, plus hardware testing. It pays
off only if more Lua packs appear; today there is one game.

## Option B - turn the DQ3 script into a ROM patch (this game only)

The same way the TwinBee sound patch was written from scratch (tbsound_a048): a 6502 patch the
pack's <patch> line applies at load, so it runs through the machinery that already works.
- Item 1 is a plain byte patch.
- Item 2: hook the routine that reads the music pointer table and write $4100-$4105 from there.
  The audio engine already takes those writes from the game.
- Item 3: a per-frame hook (in NMI) that computes the five $7C5-$7C9 bytes from the same RAM the
  script reads. The condition engine's RAM shadow sees the writes, so the graphics conditions
  work unchanged.
- Item 4: the minimap toggle is easy in 6502 (read Select). The blinking marker cannot be drawn
  over the HD picture by the game; it could be an extra NES sprite the patch places, given an HD
  replacement image, or it is left out.
- Item 5: macros fit in the controller-read routine, or are left out (they are a convenience,
  not part of the pack's look or sound).
- Item 6: debug text, left out.
Needs: free space in the ROM for the hooks (to be measured), the Delocalized v1.12 ROM the
script was written for (the script's addresses are for that build, not the library's
Dragon Warrior III (USA)), and checking against Mesen running the original pack.
Size: days, not weeks; no FPGA change.

## Recommendation

Option B for Dragon Warrior III, after the collection ships. Option A only if more Lua packs turn
up. Open item for either option: the pack is written for the "Dragon Warrior III Delocalized
v1.12" hack, which is not in the library; it would have to be obtained, or the patch rewritten
for Dragon Warrior III (USA).

## Outcome - Option B built (2026-10-30)

- v1.12 is no longer published; v1.13 and v1.14 were taken instead (Z:/NES_HDPack_work/dl/
  dq3_deloc_114, built from the hack's own IPS files on the library ROM). Every byte the script
  reads ($2A, $2B, $92, $581, $2C0, $6DF and the music pointer table) is in the same place in the
  USA ROM, v1.13 and v1.14, and the patch only touches PRG bank 30, which is byte-identical in all
  three. So one IPS (dq3hd.ips, 1005 bytes, 31 records) fits all three.
- Shipped ROM: Delocalized v1.14 (user's call), with the USA ROM's NES 2.0 header, named as the
  T-En Collection DAT names its releases. The DAT still lists v1.13 and will catch up; its v1.13
  [n] entry is exactly our v1.13 build in that header form, which checks the naming.
- Patch: tools/corpus/make_dq3_hd_patch.py (617 bytes of 6502 at $BB60 in bank 30). Items 1, 2 and
  3 above are done: the music table is silenced, song requests are hooked at the two places the
  driver stores them, and an NMI hook writes $7C5-$7C9 every frame. Not done: the blinking minimap
  marker (item 4 - the toggle itself works), the controller-2 macros (5) and the debug text (6).
- One intended difference: the Lua's read callback stops one entry short of the last song (42,
  "Into the Legend"), so the Lua silences it but never starts the HD track. The patch plays it.
- Checks: tools/corpus/test_dq3_hd_patch.py runs the patch (py65) and the pack's own Lua (lupa)
  side by side on random input - 4 seeds, 200,000 frames, all 19 backdrops, no difference; 9 of 9
  planted bugs caught. Mesen A/B on the real game (3,600 and 7,000 frames): the patched ROM
  writes the same $7C5-$7C9 bytes as the Lua on every frame.
- Pack: zz_var/dq3hd_a064 (tools/corpus/make_dq3_hd_overlay.py) = a064 with the script line
  dropped and three <patch> lines (shipped ROM, v1.14 iNES 1.0 form, DAT v1.13 [n]). It replaces
  p071 in the ship plan.
