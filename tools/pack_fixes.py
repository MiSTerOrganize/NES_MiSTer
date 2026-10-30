"""pack_fixes.py - explicit corrections for pack-author mistakes the core works around (beyond
Mesen, user decision 2026-10-22 "full compatibility for all 136"). Each entry names exactly one
line and the file that must be there, checked by MD5, so a fix can never apply to a pack it was
not written for. The pack model (hdpack_model.py) records every use in Pack.compat.

PATCH_FILE_FIXES: a <patch> line naming a file the pack does not ship.
    named        the file on the <patch> line
    hash         the hash on that line
    actual       the file the pack does ship
    actual_md5   its MD5 (the fix is ignored if it differs)
"""

PATCH_FILE_FIXES = [
    # Mega Man Super, YM2612 audio release (collection 11_Megaman_Super_audio_Woodfrog_YM2612.zip,
    # converted as a030). hires.txt line 5 names "Mega Man.ips"; the folder ships
    # "Megaman - Super.ips", byte-identical to the patch the sibling releases a028/a029/a031
    # name on the same line with the same hash.
    dict(named="Mega Man.ips", hash="2F88381557339A14C20428455F6991C1EB902C99",
         actual="Megaman - Super.ips", actual_md5="cc404135d018fae5b9895ea2dc4d0987"),
    # Ninja Gaiden HD Mesen + Hacks (collection 13_NinjaGaiden_darthvaderx_merged_2023_Ninja
    # Gaiden HD Mesen + Hacks.zip, converted as a037 over a036). hires.txt line 20995 names
    # "Ninja Gaiden.ips" with an MD5 (Mesen rejects both); the pack's only patch file is
    # "Ninja Gaiden HD (U) [Hacks].ips", and it applies cleanly to the ROM that MD5 names
    # (Ninja Gaiden (USA), iNES 1.0 form; 602 PRG bytes changed). The release title says it
    # bundles the hacks, so this is the file the line means.
    dict(named="Ninja Gaiden.ips", hash="0C2CCCDA6BA6CAB7BEDE0FF05E7F6852",
         actual="Ninja Gaiden HD (U) [Hacks].ips", actual_md5="7f8b3d43b73b194471630501671eb102"),
    # Bio Miracle Bokutte Upa NEA soundtrack (BioMiracle-sndtrk.zip from Google Drive, linked by
    # the HDnes readme; corpus folder zz_dl_biomiracle_sndtrk). hires.txt names
    # "NEA-BioMiracle.ips"; the zip ships "NEA-BioMicacle.ips" (typo), byte-identical to the
    # collection's "Bio Miracle Bokutte Upa v2.ips" (sha1 1e389100...), keyed to the same ROM.
    dict(named="NEA-BioMiracle.ips", hash="E1C7D975E6964F9860B58D5D9530ED7E2A24B376",
         actual="NEA-BioMicacle.ips", actual_md5="085321792e66082773f4f238ebec76e7"),
]

# SOUND_FILE_FIXES / BACKGROUND_FILE_FIXES (2026-10-29): a <bgm>/<sfx>/<background> line naming
# a file the pack does not ship. `named` is the file field exactly as written; for a sound line
# whose file name holds a comma it is the whole rest of the line (Mesen splits it, so it can
# never load there). The line is kept for the core under `actual`, checked by `actual_md5`.
SOUND_FILE_FIXES = [
    # 1942 bogaa music (a082) and every build over it (1942 HDnes Drive pack, v1942b variants):
    # hires.txt names "Stage 6.ogg"; the release ships "Stage 6-A.ogg" (no other stage-6 file).
    dict(named="Stage 6.ogg", actual="Stage 6-A.ogg",
         actual_md5="bde3678f27d28213c70ad6d0dd7b3885"),
    # Same releases: the file "Boss 1, Mid Boss A.ogg" exists, but its comma splits the line.
    dict(named="Boss 1, Mid Boss A.ogg", actual="Boss 1, Mid Boss A.ogg",
         actual_md5="14983679eecf974679eaa86e837c4558"),
]

BACKGROUND_FILE_FIXES = [
    # Super Mario Bros. (JU) (PRG0) pack (a041): hires.txt names "water_bg.png"; the folder
    # ships "water bg.png" (space for underscore), the only water background in the pack.
    dict(named="water_bg.png", actual="water bg.png",
         actual_md5="4b946b4982809191b04e90a0b689b98c"),
    # Castlevania Mesen SD Pack V12 (MEGA folder "Mesen me up.", corpus zz_dl_cv_sd_v12): the
    # title-screen blink frame c names "titlecb.png", which the zip does not ship. Every one of
    # the 15 shipped blink frames (title0b..titlefb) is pixel-identical to its plain frame, so
    # frame c's blink image is titlec.png.
    dict(named="titlecb.png", actual="titlec.png",
         actual_md5="5f95c6f85879c23d942baa9b262fbc18"),
]
