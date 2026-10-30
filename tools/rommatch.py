"""rommatch.py - which of a pack's <patch> lines applies to a ROM file. One rule, shared by the
converter (patch_rom, build_patches), the corpus planner and the hardware model.

Mesen applies a <patch> only when the SHA1 of the WHOLE ROM file, header included, is the one on
the line. Pack authors hashed their own dumps, which almost all carry an iNES 1.0 header (bytes
7-15 zero), while No-Intro ships the same games with an NES 2.0 header. So with the ROM most
people have, Mesen never applies the patch and the pack runs on the wrong program (Mega Man
Super's readme: white squares on the title). The core goes one step further than Mesen, by user
decision 2026-10-22 ("full compatibility for all 136"):

  1. the whole file, as Mesen does;
  2. the same file with an iNES 1.0 header - bytes 7-15 set to 0 - when it has an iNES header.
     The patch is then applied to THAT form, which is the file the author patched (a BPS checks
     the CRC of its source, so it has to be).

Measured over every <patch> and <supportedRom> hash in the collection against the No-Intro
library (tools/corpus/header_forms.py, 2026-10-22): 38 match the raw file, 113 match only the
zeroed form, 0 need byte 7's mapper nibble kept. Zeroing 7-15 is therefore the whole rule.

A <patch> line may also carry an MD5 (32 hex digits) where Mesen requires a SHA1, and Mesen
rejects it. The core has no MD5, so the converter turns such a line into SHA1 entries using the
ROM library (resolve_md5); tools that hold the ROM itself match the MD5 directly (find). Both
give the same answer for a library ROM.

    ines1(d)            -> the iNES 1.0 form of d, or None (no iNES header, or already that form)
    forms(d)            -> [(label, bytes)] in match order: the file, then its iNES 1.0 form
    find(d, sha1s, md5s)-> (source bytes, patch file, label) or None
"""
import hashlib

INES_MAGIC = b"NES\x1a"
FORM_RAW = ""
FORM_INES1 = " (iNES 1.0 header)"


def ines1(d):
    """d with header bytes 7-15 zeroed, or None when d has no iNES header or already is that."""
    d = bytes(d)
    if len(d) < 16 or d[:4] != INES_MAGIC:
        return None
    f = d[:7] + bytes(9) + d[16:]
    return None if f == d else f


def forms(d):
    d = bytes(d)
    out = [(FORM_RAW, d)]
    f = ines1(d)
    if f is not None:
        out.append((FORM_INES1, f))
    return out


def find(d, sha1s, md5s=None):
    """The <patch> to apply to ROM file d. sha1s / md5s: {hash upper: patch file}. The file is
    tried before its iNES 1.0 form, and in each form SHA1 before MD5 - the order the core's two
    hash passes give (MD5 lines reach the core as SHA1 entries of the same form)."""
    md5s = md5s or {}
    for label, c in forms(d):
        name = sha1s.get(hashlib.sha1(c).hexdigest().upper())
        if name is None:
            name = md5s.get(hashlib.md5(c).hexdigest().upper())
        if name is not None:
            return c, name, label
    return None


def core_lookup(d, entry_shas):
    """What the core does with section 14 (hd3_romfix.sv): hash the file, look it up; if no
    entry names it and the file has an iNES header, hash its iNES 1.0 form and look again.
    entry_shas: SHA1s (upper) the section's entries carry. -> (source form, sha, label) or None.
    The patch is built from the form that matched."""
    for label, c in forms(d):
        sha = hashlib.sha1(c).hexdigest().upper()
        if sha in entry_shas:
            return c, sha, label
    return None


def section_keys(patches, patches_md5, index_rows):
    """{SHA1 upper: patch file} the converter writes to section 14: every SHA1 line, plus each
    MD5 line resolved through the ROM index (a SHA1 line for the same key wins)."""
    keyed = dict(patches)
    for md5, name in sorted(patches_md5.items()):
        for sha in resolve_md5(md5, index_rows or []):
            keyed.setdefault(sha, name)
    return keyed


def resolve_md5(md5, index_rows):
    """SHA1s (upper) of the library ROM forms whose MD5 is md5. index_rows: romindex.py rows as
    dicts (md5, sha1, ines1_md5, ines1_sha1). A raw match gives the raw SHA1; an iNES 1.0 match
    gives the SHA1 of that form, which the core's second pass computes."""
    md5 = md5.lower()
    out = []
    for r in index_rows:
        if r["md5"] == md5:
            out.append(r["sha1"].upper())
        if r["ines1_md5"] == md5 and r["ines1_sha1"] != r["sha1"]:
            out.append(r["ines1_sha1"].upper())
    seen = set()
    return [s for s in out if not (s in seen or seen.add(s))]


def load_index(path):
    """romindex.py output -> [dict]."""
    lines = open(path, encoding="utf-8").read().splitlines()
    head = lines[0].split("\t")
    return [dict(zip(head, l.split("\t"))) for l in lines[1:] if l]
