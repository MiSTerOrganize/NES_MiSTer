# NES HD for MiSTer

<a href="https://github.com/sponsors/MiSTerOrganize"><img src="https://img.shields.io/badge/Sponsor-%E2%9D%A4-db61a2?logo=GitHub&style=for-the-badge" alt="Sponsor" width="190"></a>

NES HD is a hybrid FPGA + ARM core for MiSTer that plays NES games with **Mesen HD packs**: high-resolution
graphics, replacement music and sound effects, and the small ROM patches some packs need. The NES itself
runs in the FPGA (based on the [MiSTer-devel NES core](https://github.com/MiSTer-devel/NES_MiSTer)); the
HD pack engine runs on the MiSTer's ARM CPU and draws each tile and sprite the way the
[Mesen](https://github.com/SourMesen/Mesen2) emulator does.

**Status: testing build.** It is not in `update_all` yet. Things to know before installing:

- The FPGA build closes timing with every slack positive (worst +0.145 ns), so the build is valid.
- The core's MiSTer framework (`fpga/sys`) is one Template_MiSTer update behind and will be refreshed.
- All 124 packs in the collection have been converted and checked on a PC, and are installed on the
  developer's MiSTer for hardware testing. Report anything that looks or sounds wrong.

## What you need

1. A MiSTer with **MiSTer Frontier** installed. Its `Master_Daemon` starts the NES HD engine when the core
   loads. Install it from [MiSTer_Frontier](https://github.com/MiSTerOrganize/MiSTer_Frontier). If you
   already play PICO-8 or OpenBOR on your MiSTer, you have it.
2. The core files from this repo.
3. The HD packs you want (from this repo's Releases page).
4. **The exact ROM each pack was made for.** Every ROM is listed below with its size, MD5 and SHA-1.

## Install

Copy these to the SD card:

```
_Other/NES_HD_20261029.rbf          ->  /media/fat/_Other/
games/NES_HD/NES_HD                 ->  /media/fat/games/NES_HD/
games/NES_HD/_handler.sh            ->  /media/fat/games/NES_HD/
games/NES_HD/Palette/               ->  /media/fat/games/NES_HD/Palette/   (optional palettes)
```

Then put your ROMs in `/media/fat/games/NES_HD/` (any subfolder works, for example
`/media/fat/games/NES_HD/1 NES_HD/`) and the packs in `/media/fat/games/NES_HD/HD_Packs/`.

The `HD_Packs` folder and the log folder `/media/fat/logs/NES_HD/` are created automatically the first
time the core loads.

## Getting the HD packs

The packs are too large for the repo itself (the full collection is 5.19 GB, and several single packs are
over GitHub's 100 MB file limit), so they are attached to the
**[HD Packs release](https://github.com/MiSTerOrganize/NES_HD_MiSTer/releases/tag/hd-packs)**:

- **One zip per pack.** Download only the games you want. Each zip holds a single `.hdp` file.
- **The whole collection** as `NES_HD Packs (part 1 of 2).zip` and `(part 2 of 2).zip`. Each part already
  has an `HD_Packs` folder inside, so unzip both into `/media/fat/games/NES_HD/`.

GitHub rewrites file names on the release page (spaces, brackets and some other characters turn into dots
or disappear). That does not matter: the name of the `.hdp` file **inside** each zip is the correct one,
so always unzip rather than renaming the download.

## How to use a pack

1. Load the NES HD core and load a ROM from the file browser, as with the normal NES core.
2. Open the OSD (menu button), go to **Audio & Video**, choose **Load HD Pack** and pick the pack.
3. Set **HD Pack** to **On**.

Loading a pack restarts the game, so load the pack before you start playing.

**Matching a pack to a ROM.** A pack's file name is the name of its ROM, plus an optional label in square
brackets when there is more than one pack for the same ROM:

```
Metroid (USA).nes                   the ROM
Metroid (USA).hdp                   the main pack for it
Metroid (USA) [Megatroid].hdp       another pack for the same ROM
```

So for any pack, the ROM it needs is the pack name with the `[...]` label removed and `.nes` at the end.
The core does not pick a pack for you; choose it with **Load HD Pack**.

**Use the exact ROM.** Packs identify graphics by the ROM's own tile data, and many packs also patch the
ROM as it loads (music hooks, fixes, sound packs). Those patches are applied only when the SHA-1 of the
whole ROM file, header included, matches the one listed below. With a different dump or revision you may
get missing graphics, wrong graphics or no music. Check your file against the list before reporting a
problem.

## ROMs needed

49 ROMs cover all 124 packs. Sizes are the full file including the 16-byte header. 44 of them match the
No-Intro NES DAT (headered) exactly; a note on each of the other five says where it comes from.

**(F1140) Dig Dug.nes**

- Size: 40,976 bytes
- MD5: `98AD65086B2CE260D620D744004E9A81`
- SHA-1: `7725FE244178115F39B9D7C3B0D192DBC0C3E90B`
- Note: From the NES 2.0 collection (Front Fareast Magic Card single).
- Packs (1): (F1140) Dig Dug

**1942 (Japan, USA) (En).nes**

- Size: 40,976 bytes
- MD5: `073F7E6BD3DA86BD1E1D5E3ABF525D7E`
- SHA-1: `1FC8410C271441B313AD4B382FBE9DCD9EEFB6CB`
- Packs (2): 1942 (Japan, USA) (En) [No Music, Evgeny Fixes], 1942 (Japan, USA) (En)

**1943 - The Battle of Midway (USA).nes**

- Size: 131,088 bytes
- MD5: `3948924CEFE485A5F52FE7AF4E3ABD25`
- SHA-1: `395746B85B734CC926F0E126ECDF8B41091E3694`
- Packs (1): 1943 - The Battle of Midway (USA)

**Akumajou Dracula (Japan) (En).nes**

- Size: 131,088 bytes
- MD5: `0CA641E3973EDD1ACA5F93FCC9C87E0D`
- SHA-1: `5D86E8144587903DF90669DEE1791917215719B9`
- Packs (6): Akumajou Dracula (Japan) (En) [Aclectico 30th], Akumajou Dracula (Japan) (En) [SD Pack v1.2], Akumajou Dracula (Japan) (En) [Vampire Killer v1.1], Akumajou Dracula (Japan) (En) [Yone2008 VRC6 Audio, batty], Akumajou Dracula (Japan) (En) [Yone2008 VRC6 Audio], Akumajou Dracula (Japan) (En)

**BattleCity (Japan) (En).nes**

- Size: 24,592 bytes
- MD5: `AB8EE82F19D5266C5868405D6ED6CB7B`
- SHA-1: `941AD7CA825E3F86407472113AAD00520CB45783`
- Packs (4): BattleCity (Japan) (En) [HD], BattleCity (Japan) (En) [Kiwi], BattleCity (Japan) (En) [Yuri Egorov 2023], BattleCity (Japan) (En)

**Bio Miracle Bokutte Upa (Japan) (En).nes**

- Size: 262,160 bytes
- MD5: `265B565B598EF79E047326132A3E6930`
- SHA-1: `E1C7D975E6964F9860B58D5D9530ED7E2A24B376`
- Packs (1): Bio Miracle Bokutte Upa (Japan) (En)

**Bomb Sweeper (World) (Proto) (Aftermarket) (Unl).nes**

- Size: 24,592 bytes
- MD5: `5524E8EF0312EC634AE38D83DDA023A1`
- SHA-1: `7503A7F87997B7332B234DFF963F4B0F17651A86`
- Packs (1): Bomb Sweeper (World) (Proto) (Aftermarket) (Unl)

**Bomberman (USA).nes**

- Size: 24,592 bytes
- MD5: `775F664B177E561B34615614D160DFB7`
- SHA-1: `1A38860F7583E4619DACD293E80D80E7BD4EE021`
- Packs (1): Bomberman (USA)

**Castlevania (USA) (Rev 1).nes**

- Size: 131,088 bytes
- MD5: `A1B849F232781D0E929435DF447822B5`
- SHA-1: `4E3EF47DE86A941C37359BB3ACFA5542A0FA7876`
- Packs (3): Castlevania (USA) (Rev 1) [Sour 2018], Castlevania (USA) (Rev 1) [Templates johnnyoak85], Castlevania (USA) (Rev 1)

**Castlevania II - Simon's Quest (Europe).nes**

- Size: 262,160 bytes
- MD5: `4A9F5D922238AA7CD8BCF5FFF0CE33B7`
- SHA-1: `14A8B4CE15C681ECE714E80CE0565FE3DD66D7DC`
- Packs (3): Castlevania II - Simon's Quest (Europe) [Yone2008 VRC6 Audio, v2], Castlevania II - Simon's Quest (Europe) [Yone2008 VRC6 Audio], Castlevania II - Simon's Quest (Europe)

**Castlevania III - Dracula's Curse (USA).nes**

- Size: 393,232 bytes
- MD5: `561999DAA97A1C30C81CAB5142A1D672`
- SHA-1: `0BB28DD9C2F8DFABB3349D1F65407B941EE78435`
- Packs (2): Castlevania III - Dracula's Curse (USA) [bogaa Test 2017], Castlevania III - Dracula's Curse (USA)

**Circus Charlie (Japan) (En).nes**

- Size: 24,592 bytes
- MD5: `0D5260519C3DA652A980C8F2581FDADA`
- SHA-1: `C2458A3B54B1B0A913F76F4D81A3DBBA48B59F9A`
- Packs (1): Circus Charlie (Japan) (En)

**Contra (Japan) [T-En by Stardust Crusaders v1.01] [n].nes**

- Size: 262,160 bytes
- MD5: `8F5200F6979149F4F6E99FFB2163C249`
- SHA-1: `89E0798D908EE0DDB44C4C21A499E8BB1B6F8640`
- Note: Contra (Japan) with the Stardust Crusaders English translation v1.01.
- Packs (2): Contra (Japan) [T-En by Stardust Crusaders v1.01] [n] [Vomitron MET Hybrid], Contra (Japan) [T-En by Stardust Crusaders v1.01] [n]

**Contra (USA).nes**

- Size: 131,088 bytes
- MD5: `CDF73714FF3EF47F4EEB2B71707BE2A0`
- SHA-1: `4B9FAD65727042595C5820CA03BC32BFD72DE5E7`
- Packs (2): Contra (USA) [Remastered Aclectico Demo], Contra (USA)

**Donkey Kong (World) (Rev 1).nes**

- Size: 24,592 bytes
- MD5: `DECCA1D26ECE603A0F69A5834FB5DDDA`
- SHA-1: `6F0AE51888043F04D8AAA8AAB005B9B6CE1DFE81`
- Packs (2): Donkey Kong (World) (Rev 1) [Bowser's Castle], Donkey Kong (World) (Rev 1)

**Donkey Kong Jr. (Japan) (En).nes**

- Size: 24,592 bytes
- MD5: `74F929DB0DD04377A00F91FF6175A404`
- SHA-1: `6C25EA2D62CCCFD0DE63D453AA7C51E8DD3FD3F7`
- Packs (1): Donkey Kong Jr. (Japan) (En)

**Dr. Mario (Japan, USA) (En) (Rev 1).nes**

- Size: 65,552 bytes
- MD5: `A4E02BAF4284348C045655B91CB84BDE`
- SHA-1: `0F18C2A5760BE0647339B0E3CAB9183AF86FDD8C`
- Packs (1): Dr. Mario (Japan, USA) (En) (Rev 1)

**Dragon Quest III (USA) [T-En by Translation Quest v1.14] [Delocalized] [n].nes**

- Size: 524,304 bytes
- MD5: `0FC159D4B59A2C43262FB826697E3807`
- SHA-1: `02F259B882F4909F40C19B07FE0862F128EE7495`
- Note: Dragon Warrior III (USA) patched with the Dragon Quest III Delocalized v1.14 hack by Translation Quest, then given a NES 2.0 header. New enough that no DAT lists it yet. The pack also accepts the same hack with its own iNES 1.0 header (SHA-1 B49CE1DA...) and v1.13 (SHA-1 B6346425...).
- Packs (1): Dragon Quest III (USA) [T-En by Translation Quest v1.14] [Delocalized] [n]

**Duck Hunt (World).nes**

- Size: 24,592 bytes
- MD5: `1C2B01E982667D7BA723988C771D88FA`
- SHA-1: `F5B310E71129315192C28EF457DA16D5FB1BC91E`
- Packs (4): Duck Hunt (World) [Audio], Duck Hunt (World) [HD v1.1], Duck Hunt (World) [NEA Audio], Duck Hunt (World)

**Faxanadu (USA).nes**

- Size: 262,160 bytes
- MD5: `2A8A8344C47CF13D94C220DCC7E1ED1A`
- SHA-1: `2E2B95DB2BE615CF588F0D758E0F7B1CCD81590A`
- Packs (1): Faxanadu (USA)

**Galaga (Japan) (En).nes**

- Size: 24,592 bytes
- MD5: `C3B2A6D269F2FF9D014CC976F0336E07`
- SHA-1: `3345115B8BD5B8BB28A29D535BBA0D7BD90613D9`
- Packs (1): Galaga (Japan) (En)

**Galaga - Demons of Death (USA).nes**

- Size: 40,976 bytes
- MD5: `6E4C6F22429CEA272C6EDC58290347ED`
- SHA-1: `B32A3B3FEC91A9481174C5300533B2917DB43A9A`
- Packs (1): Galaga - Demons of Death (USA)

**Galaxian (rev0)[PRG doubled to 16 KiB].nes**

- Size: 24,592 bytes
- MD5: `DD983EA60A1643FB795582F5C1885B23`
- SHA-1: `C02673E8A9A7BFBC1732912AA29291903406E698`
- Note: From the NES 2.0 collection (compatibility hack).
- Packs (1): Galaxian (rev0)[PRG doubled to 16 KiB]

**Ice Climber (USA, Europe, Asia) (En).nes**

- Size: 24,592 bytes
- MD5: `4A140974854A221E1D3A93D9E871E79A`
- SHA-1: `7309BA29C5BDFE088302E107EB4F7C9E67C8CD32`
- Packs (5): Ice Climber (USA, Europe, Asia) (En) [NEA VS v1.0], Ice Climber (USA, Europe, Asia) (En) [NEA VS v1.1], Ice Climber (USA, Europe, Asia) (En) [NEA, 2A03 Music], Ice Climber (USA, Europe, Asia) (En) [NEA, SFC Music], Ice Climber (USA, Europe, Asia) (En)

**Jackal (USA).nes**

- Size: 131,088 bytes
- MD5: `7C54D27E8B5A100EF8E6429E6C81959B`
- SHA-1: `C0067EB00E9F1F82561E9A96A5C2B7E22CEE806C`
- Packs (2): Jackal (USA) [NEA], Jackal (USA)

**Journey to Silius (USA).nes**

- Size: 262,160 bytes
- MD5: `03779FC352A51B1594445E587F99AFAE`
- SHA-1: `90B34471E1CF8547060E926ED6E56A704769C23F`
- Packs (1): Journey to Silius (USA)

**Kung Fu (Japan, USA) (En).nes**

- Size: 40,976 bytes
- MD5: `80F3E8672C5642249FC070E335A93180`
- SHA-1: `B36ECE8FC8330C36716A7EB19874C31FC5F14287`
- Packs (2): Kung Fu (Japan, USA) (En) [MkWong98], Kung Fu (Japan, USA) (En)

**Legend of Zelda, The (USA).nes**

- Size: 131,088 bytes
- MD5: `4156F66C0FAC36222B5A1DA05DB85D68`
- SHA-1: `799459548F9636DD263200DA494F04058F5540E2`
- Packs (5): Legend of Zelda, The (USA) [GodModeKidd 2026 Zip Build], Legend of Zelda, The (USA) [GodModeKidd 2026], Legend of Zelda, The (USA) [Remastered v1.3 Low CPU], Legend of Zelda, The (USA) [Remastered v1.3], Legend of Zelda, The (USA)

**Life Force (USA) (Rev 1) (Virtual Console).nes**

- Size: 131,088 bytes
- MD5: `D5EA3575AD3051BC2404ECE12FF84CD8`
- SHA-1: `00CC60E8207B7B8A1F870AE63D3A8F44AF688586`
- Packs (1): Life Force (USA) (Rev 1) (Virtual Console)

**Little Nemo - The Dream Master (USA).nes**

- Size: 262,160 bytes
- MD5: `B13469FA546240B1645E39DC0F9DC369`
- SHA-1: `01456A52E6F8B217A3975BA970404BF04BF51FCB`
- Packs (1): Little Nemo - The Dream Master (USA)

**Mappy (Japan) (En).nes**

- Size: 24,592 bytes
- MD5: `9B805EC23480A676DC6538B5A927D7CD`
- SHA-1: `B808A6A7AEE80097406C5FBA7867018B7748D36E`
- Packs (2): Mappy (Japan) (En) [30th Anniversary], Mappy (Japan) (En)

**Mega Man (USA).nes**

- Size: 131,088 bytes
- MD5: `8CFAA33B1EC522CE89AB82AF2DD17DA7`
- SHA-1: `51A9FE7B5CB82833FC416E0F29B965495D9CE49A`
- Packs (18): Mega Man (USA) [Super 2.1.1, NES Music, Pause], Mega Man (USA) [Super 2.1.1, NES Music], Mega Man (USA) [Super 2.1.1, Pause], Mega Man (USA) [Super 2.1.5, NES Music, Pause], Mega Man (USA) [Super 2.1.5, Remixed Music, Pause], Mega Man (USA) [Super 2.1.5, Remixed Music], Mega Man (USA) [Super 2.1.5], Mega Man (USA) [Super GitHub, NES Music, Pause], Mega Man (USA) [Super GitHub, NES Music], Mega Man (USA) [Super GitHub, Remixed Music, Pause], Mega Man (USA) [Super GitHub, Remixed Music], Mega Man (USA) [Super GitHub], Mega Man (USA) [Super NEA, Vomitron Music], Mega Man (USA) [Super NEA, Woodfrog Music], Mega Man (USA) [Super Vomitron], Mega Man (USA) [Super Woodfrog YM2612], Mega Man (USA) [Super Woodfrog, batty], Mega Man (USA)

**Metroid (USA).nes**

- Size: 131,088 bytes
- MD5: `397D10E475266AD28144A5FA6EC3C466`
- SHA-1: `2080059BEA4563FFFF7535E6C0A0F3BC2F34B194`
- Packs (13): Metroid (USA) [8BitTroid], Metroid (USA) [Fast, Extra Wave Beam Room], Metroid (USA) [Fusiontroid], Metroid (USA) [JUD6MENT Orchestral], Metroid (USA) [Legacytroid], Metroid (USA) [Megatroid], Metroid (USA) [v1.5 8BitTroid], Metroid (USA) [v1.5 Fusiontroid], Metroid (USA) [v1.5 Megatroid], Metroid (USA) [v1.5], Metroid (USA) [Vanilla Speed, Vanilla Map], Metroid (USA) [Vanilla Speed], Metroid (USA)

**Ms. Pac-Man (USA) (Namco).nes**

- Size: 40,976 bytes
- MD5: `67E0ACB6D58FF6ED027E6B0E3A059074`
- SHA-1: `E6083431010ECA48F541F68DDE960B342F9043C1`
- Packs (1): Ms. Pac-Man (USA) (Namco)

**Ninja Gaiden (USA).nes**

- Size: 262,160 bytes
- MD5: `15C46B30C216086D1F215977A1019691`
- SHA-1: `282C846DDBE968CC5E4D1ADC56DC9B8E3BEF4753`
- Packs (3): Ninja Gaiden (USA) [HDNes Beta], Ninja Gaiden (USA) [NEA], Ninja Gaiden (USA)

**Ninja Gaiden II - The Dark Sword of Chaos (USA).nes**

- Size: 262,160 bytes
- MD5: `ED8DE08E511D6F6A95E1CF58A57DAAB2`
- SHA-1: `269478947A5BC518551AB5D7B4687653006E243C`
- Packs (1): Ninja Gaiden II - The Dark Sword of Chaos (USA)

**Nuts & Milk (Japan) (En).nes**

- Size: 24,592 bytes
- MD5: `32870A9DAA6FD7F9348D5A48FAE4B1F7`
- SHA-1: `12655B634F79F387404A7AB319124BA2FFCE8829`
- Packs (2): Nuts & Milk (Japan) (En) [HDNes 2014], Nuts & Milk (Japan) (En)

**Pac-Man (USA) (Namco).nes**

- Size: 24,592 bytes
- MD5: `45335AC1FAB5AEDAF25C0388E7E2B53B`
- SHA-1: `EF76CEBDDC57B7C96CFC95B55DCC712FD5934B2C`
- Packs (3): Pac-Man (USA) (Namco) [Arcade], Pac-Man (USA) (Namco) [HDNes], Pac-Man (USA) (Namco)

**Popeye (World) (Rev 1).nes**

- Size: 24,592 bytes
- MD5: `1A64FBC39FC787DB51BD838F1A8F4A81`
- SHA-1: `1EF97E56922D6741D6F0833C503CEB4FD170240E`
- Packs (1): Popeye (World) (Rev 1)

**Road Fighter (Europe).nes**

- Size: 24,592 bytes
- MD5: `34C6581B5DBB44AF2D441EC45AA9091B`
- SHA-1: `950D0E45DFCAA5252ECFDDC8EB0E0F237CE39053`
- Packs (1): Road Fighter (Europe)

**Road Fighter (Japan) (En).nes**

- Size: 24,592 bytes
- MD5: `278942ECD4606FDE731E85E93E8E8EDD`
- SHA-1: `0B345DEBB4E18F4434EF5C20C7A513135366FBA3`
- Packs (1): Road Fighter (Japan) (En)

**Shatterhand (USA).nes**

- Size: 262,160 bytes
- MD5: `D13E4A1106F24CB2FC8124E064FF9D2A`
- SHA-1: `8BADCA09DA49523E95C6F635A551D515833468B7`
- Packs (1): Shatterhand (USA)

**Super Mario Bros. (JU) [!] [T+Rus Guyver (24.11.2016)].nes**

- Size: 40,976 bytes
- MD5: `0B24529DCEB4CD1DA0CA6E1575681E76`
- SHA-1: `4567780A20E94EC3DB5D840586DF4F235C7F2F8C`
- Note: Super Mario Bros. with the Russian translation by Guyver (24.11.2016), romhacking.net translation 3852. Not in any DAT.
- Packs (1): Super Mario Bros. (JU) [!] [T+Rus Guyver (24.11.2016)]

**Super Mario Bros. (World).nes**

- Size: 40,976 bytes
- MD5: `F94BB9BB55F325D9AF8A0FFF80B9376D`
- SHA-1: `33D23C2F2CFA4C9EFEC87F7BC1321CE3CE6C89BD`
- Packs (6): Super Mario Bros. (World) [LyonHrt], Super Mario Bros. (World) [MkWong98], Super Mario Bros. (World) [NEA Music], Super Mario Bros. (World) [Paper Mario], Super Mario Bros. (World) [roney33 Alpha], Super Mario Bros. (World)

**Super Mario Bros. 2 (USA) (Rev 1).nes**

- Size: 262,160 bytes
- MD5: `89102AA7CC7B5927CD7F2D6711333224`
- SHA-1: `0DDE824764DABA70217E6167D01F202FB1666CB0`
- Packs (3): Super Mario Bros. 2 (USA) (Rev 1) [NEA, Doki Doki Music], Super Mario Bros. 2 (USA) (Rev 1) [NEA, Remix Music], Super Mario Bros. 2 (USA) (Rev 1)

**TwinBee (Japan) (En).nes**

- Size: 49,168 bytes
- MD5: `E1390A0C7DCC9B3E1BE4FD8FA2A1DF31`
- SHA-1: `632B2FCC7D4D0D0373C5A15B2CB26C8144945A17`
- Packs (1): TwinBee (Japan) (En)

**Urban Champion (World).nes**

- Size: 24,592 bytes
- MD5: `7596031514FAD25169F834CD803C7E71`
- SHA-1: `D130CB806B16094EDD0EA93D3C418F5AC77EBA6E`
- Packs (2): Urban Champion (World) [Gangster], Urban Champion (World)

**Yie Ar Kung-Fu (Japan) (En) (Rev 1.2).nes**

- Size: 24,592 bytes
- MD5: `302B5615B9D48F1DB818E492114262BF`
- SHA-1: `C24D3DB34F7EC1373DF0617ABDFAF9BA85172455`
- Packs (1): Yie Ar Kung-Fu (Japan) (En) (Rev 1.2)

**Zelda II - The Adventure of Link (USA).nes**

- Size: 262,160 bytes
- MD5: `E6DD4104FA46EB4BEF2CB52DC341DD38`
- SHA-1: `08FA60F23E477752FDD850B1FD4171C6CA025127`
- Packs (3): Zelda II - The Adventure of Link (USA) [Main], Zelda II - The Adventure of Link (USA) [Pre-release], Zelda II - The Adventure of Link (USA)

## Credits

Every pack in the collection is the work of its original authors, converted to the `.hdp` format this core
loads. **All rights stay with the authors.** If you made one of these packs and want it changed or taken
down, open an issue and it will be removed.

Where a pack was missing a file (a ROM patch its `hires.txt` names, or audio that another release of the
same pack carries), the missing file was taken from that other release and nothing else was changed. The
TwinBee sound patch and the Dragon Quest III ROM patch were written for this collection by MiSTer Organize.

**1942** - music by bogaa games (2022). The `No Music, Evgeny Fixes` pack uses the pack's own patch with
evgeny's improvements and no music.

**1943 - The Battle of Midway** - mkwong98 (2024).

**Akumajou Dracula** (Castlevania, Japan)
- main pack: kya, HD graphics and music, with JUD6MENT's orchestral soundtrack (2023)
- `Aclectico 30th`: Aclectico, Castlevania 30th Anniversary
- `SD Pack v1.2`, `Vampire Killer v1.1`: kya
- `Yone2008 VRC6 Audio`: VRC6 soundtrack by Yone2008
- `Yone2008 VRC6 Audio, batty`: the same soundtrack in batty's set

**Battle City**
- main pack: SUPERTANK BROS, Yuri Egorov
- `HD`: Battle City Remastered, imkrut
- `Kiwi`: Froser (Kiwi Machine)
- `Yuri Egorov 2023`: Yuri Egorov (2023)

**Bio Miracle Bokutte Upa** - LiQuiDz and Cubear (HDnes repository).

**Bomb Sweeper** - mkwong98 (HDNes era).

**Bomberman** - Bomberman Remastered, imkrut.

**Castlevania** (USA)
- main pack: Rondo of Blood style, mkwong98 (2026)
- `Sour 2018`: Sour's conversion (2018)
- `Templates johnnyoak85`: johnnyoak85's recolour templates

**Castlevania II - Simon's Quest**
- main pack: Cubear's HD patch with JUD6MENT's orchestral soundtrack
- `Yone2008 VRC6 Audio`, `Yone2008 VRC6 Audio, v2`: VRC6 soundtrack by Yone2008 (v2 is batty's set)

**Castlevania III - Dracula's Curse**
- main pack: HD graphics with Kurrono's patch and JUD6MENT's orchestral soundtrack
- `bogaa Test 2017`: bogaabogaa's character test pack (2017)

**Circus Charlie** - 8x texture pack (2021), author unknown.

**Contra** (USA)
- main pack: Contra 80s 1.4.2, Tastic
- `Remastered Aclectico Demo`: Contra Remastered demo, Aclectico

**Contra** (Japan, English translation) - pev's NEA music kit (Gryzor MET). The `Vomitron MET Hybrid` pack
uses pev's Vomitron hybrid set.

**Dig Dug** - Dig Dug Deluxe, MyDude Studios.

**Donkey Kong**
- main pack: Donkey Kong HD, mkwong98
- `Bowser's Castle`: Crash Gaming

**Donkey Kong Jr.** - Donkey Kong Jr. Remastered, imkrut.

**Dr. Mario** - NEA audio, LiQuiDz and Cubear (HDnes repository).

**Dragon Quest III** (Delocalized translation) - HD pack v0.5.1 by KCN2. The game is the Dragon Quest III
Delocalized translation by the Translation Quest team. The pack's Lua script was turned into a ROM patch
for MiSTer by MiSTer Organize.

**Duck Hunt**
- main pack: Duck Hunt HD v1.1 with NEA audio
- `HD v1.1`: Duck Hunt HD v1.1, Aclectico and GBAGuy007
- `Audio`, `NEA Audio`: LiQuiDz and Cubear (HDnes repository)

**Faxanadu** - Faxanadu: Revisioned v1.26, Altwolf.

**Galaga** (Japan) - 4x pack by LiQuiDz, sprites by KESTREL (2025).

**Galaga - Demons of Death** - arcade-style pack, 125scratch.

**Galaxian** - arcade-style pack, 125scratch.

**Ice Climber**
- main pack: Ice Climber Remastered, imkrut
- `NEA, 2A03 Music`, `NEA, SFC Music`, `NEA VS v1.0`, `NEA VS v1.1`: LiQuiDz and Cubear (HDnes
  repository)

**Jackal**
- main pack: Arasoi Rock soundtrack
- `NEA`: pev's NEA music

**Journey to Silius** - pev's NEA music.

**Kung Fu**
- main pack: Kung Fu Guy, mkwong98 (2024)
- `MkWong98`: Kung Fu HD, mkwong98

**The Legend of Zelda**
- main pack: Zelda HD with JUD6MENT's orchestral soundtrack
- `Remastered v1.3`, `Remastered v1.3 Low CPU`: Zelda Remastered, Aclectico
- `GodModeKidd 2026`, `GodModeKidd 2026 Zip Build`: GodModeKidd's 2026 edit

**Life Force** - Vomitron music set (pev).

**Little Nemo - The Dream Master** - imkrut.

**Mappy**
- main pack: audio by LiQuiDz
- `30th Anniversary`: arcade-style pack, 125scratch

**Mega Man** - Megaman: Super, AxlRocks.
- main pack and `Super 2.1.1` packs: version 2.1.1; `Super 2.1.5` packs: the 2.1.5 graphics release;
  `Super GitHub` packs: the newest GitHub version
- `NES Music`, `Remixed Music`, `Pause` labels: the music and pause options that ship with Megaman: Super
- `Super NEA, Vomitron Music`, `Super NEA, Woodfrog Music`: pev's NEA music
- `Super Vomitron`: Vomitron music set
- `Super Woodfrog YM2612`: Woodfrog's YM2612 soundtrack
- `Super Woodfrog, batty`: Woodfrog's soundtrack in batty's set

**Metroid** - Metroid HD, Aclectico, kya and others.
- main pack and the `8BitTroid`, `Fusiontroid`, `Legacytroid`, `Megatroid`, `Fast, Extra Wave Beam Room`,
  `Vanilla Speed`, `Vanilla Speed, Vanilla Map` options: version 2.1.2
- `v1.5` packs: version 1.5
- `JUD6MENT Orchestral`: JUD6MENT's orchestral soundtrack

**Ms. Pac-Man** - arcade-style pack, 125scratch.

**Ninja Gaiden**
- main pack: RichterSnipes' SNES-Trilogy graphics, merged by darthvaderx (2023), with files from
  Conspiracy83's music set
- `HDNes Beta`: RichterSnipes, Beta 1.0
- `NEA`: pev's NEA music

**Ninja Gaiden II - The Dark Sword of Chaos** - RichterSnipes.

**Nuts & Milk** - evgeny. The `HDNes 2014` pack is evgeny's original 2014 release.

**Pac-Man**
- main pack: SquidPac, gamethich2020, over 125scratch's arcade pack
- `Arcade`: arcade-style pack, 125scratch
- `HDNes`: HDNes-era pack (HDnes repository)

**Popeye** - Arcade Restore, LiQuiDz.

**Road Fighter** - Europe: AllanUr4. Japan: LiQuiDz, based on AllanUr4's pack.

**Shatterhand** - Shatter Man, Mugi (2023 repost by darthvaderx).

**Super Mario Bros.**
- main pack: Super Mario Bros. Arcade, ScrubSandwich (final version, 2025)
- `LyonHrt`: lyonhrt
- `MkWong98`: Super Mario Bros. HD, mkwong98
- `NEA Music`: Cubear's NEA audio patch
- `Paper Mario`: Paper Mario Bros., mkwong98 (2025 update)
- `roney33 Alpha`: roney33

**Super Mario Bros.** (Russian translation) - HQ pack, author unknown (2016).

**Super Mario Bros. 2**
- main pack: asshat818 (HDNes, 2014)
- `NEA, Doki Doki Music`, `NEA, Remix Music`: LiQuiDz and Cubear (HDnes repository)

**TwinBee** - TwinBee Remastered, imkrut. Sound patch by MiSTer Organize.

**Urban Champion** - NEA audio (jazz and gangster versions), LiQuiDz and Cubear (HDnes repository).

**Yie Ar Kung-Fu** - NEA audio, LiQuiDz and Cubear (HDnes repository).

**Zelda II - The Adventure of Link** - ZII, Giric (code), HaddianKey (art) and others, with music by
Niskala. The main pack is the newest 2x version; `Main` and `Pre-release` are the other two published
builds.

The core is based on the [MiSTer-devel NES core](https://github.com/MiSTer-devel/NES_MiSTer) by Ludvig
Strigeus and the MiSTer developers, and the HD pack format and behaviour follow
[Mesen](https://github.com/SourMesen/Mesen2) by Sour.

## For developers

```
fpga/              the core (Quartus 17.0.2 project, rtl/, sys/ framework)
src/               the ARM HD pack engine (built by .github/workflows/build.yml)
tools/             the PC converter that turns a Mesen HD pack folder into a .hdp file
docs/dev/          design notes
games/NES_HD/      what goes on the SD card
_Other/            the core (.rbf)
```

To convert your own pack: `python tools/hdp3_convert.py "<Mesen pack folder>" out.hdp --rom game.nes`
(needs Python 3 with numpy and Pillow; audio needs the decoder built by `tools/audio/build.sh`).

## Support

<p align="center">
  <a href="https://www.patreon.com/join/MiSTer_Organize">
    <img src="assets/patreon_banner.png" alt="Support MiSTer Organize on Patreon" width="500">
  </a>
</p>
