#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# inbox.sh -- Check that the texts of a drawing stay in their rooms
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/inbox.sh WORDS.box
# The word boxes are those of pdftotext -bbox of the drawing of
# test/inside.draw, on a B sheet, 17 x 11 in, in points.  Each word of
# the notes ends by the right side of the region of the notes of
# lib/templates/plain_b.ps, 6.4 in, and the SIZE label of the title block before
# the line at 11.6 in.
notes=$(sed -n 's/^note //p' test/inside.draw)
awk -v notes="$notes" '
BEGIN {
	n = split(notes, w, /[ \n]+/)
	for (i = 1; i <= n; i++) note[w[i]] = 1
}
/<word / {
	match($0, /xMin="[^"]*"/); x0 = substr($0, RSTART + 6, RLENGTH - 7)
	match($0, /xMax="[^"]*"/); x1 = substr($0, RSTART + 6, RLENGTH - 7)
	t = $0; sub(/.*">/, "", t); sub(/<\/word>.*/, "", t)
	if (t in note && x0 < 6.4 * 72) {
		seen++
		if (x1 > 6.4 * 72 + 0.5) bad++
	}
	if (t == "SIZE" && x1 > 11.6 * 72) bad++
}
END { exit !(seen > 10 && !bad) }' "$1"
