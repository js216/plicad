#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# dimcheck.sh -- Check the dimensions of the views of a part by measure
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/dimcheck.sh DIR part.fold VIEW...
# The programs come from DIR.  For each VIEW of top, front, right and
# flat, every dimension that smdim prints must say what measure says
# of the part, in inches to three decimals without the zero:
# - the text of each ordinate is its own place from the datum;
# - the last ordinate on each axis is the size of the box of the part,
#   or of its blank in the flat view;
# - each hole that measure lists along the line of sight has an
#   ordinate at its place on both axes, but for the holes inside a row,
#   whose callout must give the pitch of the row;
# - each size of those holes has one callout, with the diameter and the
#   count of measure;
# - the text of each radius and linear dimension is its own size.
# - no leader is longer than smdim means it to be.
# It prints the first dimension that fails, and exits with 1.

S=$1
fold=$2
shift 2
out=${TMPDIR:-/tmp}/dimcheck.$$
trap 'rm -f "$out".*' EXIT
"$S/measure" "$fold" > "$out.measure" || exit 1
for view in "$@"; do
	"$S/smview" "$view" "$fold" | "$S/smdim" > "$out.dim" || exit 1
	awk -v view="$view" '
	function inch(mm,    s) {
		s = sprintf("%.3f", mm / 25.4)
		sub(/^0\./, ".", s)
		return s
	}
	function near(a, b) { return a - b < 0.001 && b - a < 0.001 }
	function bad(what) {
		print FILENAME ": view " view ": " what
		failed = 1
	}
	# Whether an ordinate of the axis stands at mm from the datum.
	function has(axis, mm,    k) {
		for (k = 1; k <= nord[axis]; k++)
			if (near(ord[axis, k], mm))
				return 1
		return 0
	}
	FNR == NR && $1 == "bbox" {
		for (k = 1; k <= 6; k++)
			box[k] = $(k + 1)
		next
	}
	FNR == NR && $1 == "flat" { flat["x"] = $2; flat["y"] = $3; next }
	FNR == NR && $1 == "hole" {
		# The axis the eye looks along, and the two of the paper.
		w = view == "top" ? 10 : (view == "front" ? 9 : 8)
		u = view == "right" ? 3 : 2
		v = view == "top" ? 3 : 4
		if (view != "flat" && $w == 1) {
			nh++
			hu[nh] = $u - box[u - 1]
			hv[nh] = $v - box[v - 1]
			hd[nh] = $6
			count[$6]++
		}
		size["x"] = box[u + 2] - box[u - 1]
		size["y"] = box[v + 2] - box[v - 1]
		next
	}
	FNR == NR { next }
	$1 == "dim" && $2 == "ord" {
		ord[$3, ++nord[$3]] = $4
		if ($5 != inch($4))
			bad("ordinate " $3 " at " $4 " mm says " $5)
		if ($4 > last[$3])
			last[$3] = $4
	}
	$1 == "dim" && $2 == "hole" {
		called[$3 + 0] = $4
		want = "%%c" inch($3) ($4 > 1 && $5 !~ /THRU/ ? " " $4 "X" : "")
		got = $5 ($6 ~ /X$/ ? " " $6 : "")
		if (got != want && $5 != "%%c" inch($3))
			bad("hole of " $3 " mm says " got)
	}
	$1 == "dim" && $2 == "pitch" {
		if (($3 > 0 && !index($0, " " inch($3))) ||
			($4 > 0 && !index($0, " " inch($4))))
			bad("pitch of " $3 " and " $4 " mm says " $6)
		rows = 1
	}
	$1 == "dim" && $2 == "radius" && $5 != "R" inch($3) {
		bad("radius of " $3 " mm says " $5)
	}
	$1 == "dim" && $2 == "lin" && $5 != inch($4) {
		bad("length of " $4 " mm says " $5)
	}
	$1 == "far" && $2 > 0 { bad($2 " leaders longer than their reach") }
	END {
		if (view == "flat") {
			if (last["x"] < last["y"]) {
				t = flat["x"]
				flat["x"] = flat["y"]
				flat["y"] = t
			}
			size["x"] = flat["x"]
			size["y"] = flat["y"]
		}
		if (size["x"] && !near(last["x"], size["x"]))
			bad("the x size is " size["x"] " mm, not " last["x"])
		if (size["y"] && !near(last["y"], size["y"]))
			bad("the y size is " size["y"] " mm, not " last["y"])
		for (k = 1; k <= nh; k++)
			if (!rows && !(has("x", hu[k]) && has("y", hv[k])))
				bad("no ordinates for the hole at " hu[k] \
					" " hv[k] " mm")
		for (d in count)
			if (called[d + 0] != count[d])
				bad(count[d] " holes of " d " mm, but the " \
					"callout counts " called[d + 0])
		exit failed
	}' "$out.measure" "$out.dim" || exit 1
done
