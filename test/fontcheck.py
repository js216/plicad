# SPDX-License-Identifier: GPL-3.0-only
# fontcheck.py -- Check the widths of src/font.c against osifont
# Copyright (c) 2026 Jakob Kastelic

# Run with python3 -I test/fontcheck.py FONT.ttf src/font.c: the widths
# of the characters from the space to the tilde in font.c, and its
# height of a capital, must be those of the font file, as fontTools
# reads it: its advance widths and the cap height of its OS/2 table.
# Prints each that differs, and exits 1 if any does.

import re
import sys

from fontTools.ttLib import TTFont


def main(font_path, src_path):
    font = TTFont(font_path)
    cmap = font.getBestCmap()
    hmtx = font["hmtx"]
    src = open(src_path, encoding="ascii").read()
    start = src.index("widths[] = {")
    table = src[start : src.index("};", start)]
    widths = [int(x) for x in re.findall(r"\d+", table)]
    cap = float(re.search(r"#define CAP ([0-9.]+)", src).group(1))
    bad = 0
    if len(widths) != 0x7F - 0x20:
        print(f"font.c has {len(widths)} widths, not {0x7F - 0x20}")
        return 1
    for i, w in enumerate(widths):
        c = 0x20 + i
        want = hmtx[cmap[c]][0]
        if w != want:
            print(f"{chr(c)!r}: font.c {w}, the font {want}")
            bad = 1
    if cap != font["OS/2"].sCapHeight:
        print(f"cap height: font.c {cap}, the font {font['OS/2'].sCapHeight}")
        bad = 1
    return bad


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
