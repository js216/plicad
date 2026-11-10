#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# occt.sh -- Check the STEP files of the test parts with OCCT
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/occt.sh BIN PARTS
# BIN holds the programs, PARTS names the test parts.  OCCT must read
# each as valid solids, as many as the STEP file writes, with the volume
# and area of measure.

bin=$1
parts=$2
steps=
for t in $parts; do
	"$bin/smfold" test/$t.sm > build/$t.fold &&
		"$bin/tostep" build/$t.fold > build/$t.step || exit 1
	steps="$steps build/$t.step"
done
test/freecad.sh test/occtcheck.py build/occt.txt $steps \
	> build/occt.log 2>&1
fail=0
for t in $parts; do
	"$bin/measure" build/$t.fold | awk -v t=$t '
		$1 == "volume" { v = $2 }
		$1 == "area" { a = $2 }
		END { print "solid", t, "valid volume", v, "area", a }' \
		> build/occt.want
	grep "^solid $t " build/occt.txt | cut -d' ' -f1-7 > build/occt.one
	k=$(grep -c MANIFOLD_SOLID_BREP build/$t.step)
	if "$bin/numdiff" -t volume=rel:1e-6 -t area=rel:1e-6 \
		build/occt.want build/occt.one &&
		grep -q "^solid $t .* solids $k\$" build/occt.txt; then
		echo "ok occt $t"
	else
		echo "FAIL occt $t"
		fail=1
	fi
done
exit $fail
