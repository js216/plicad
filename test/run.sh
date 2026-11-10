#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# run.sh -- Run the tests of PliCad
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/run.sh [GROUP...]
# The groups are arith, parts, place, bad, tools, docs, args and occt;
# with none it runs all of them.  It builds the programs with make
# first, and the tests run the sanitized ones in build/san.  It prints
# ok or FAIL for each group and exits non-zero if any fails.  The occt
# group needs FreeCAD (FREECAD, see test/freecad.sh).

cd "$(dirname "$0")/.." || exit 1
S=build/san
TOL="-t volume=rel:1e-3 -t area=rel:1e-3 -t bbox=abs:0.01 \
-t hole=abs:0.01 -t d=abs:0.001 -t axis=abs:0.001 -t flat=abs:0.01"
PROGS=$(printf 'progs:\n\t@echo $(PROGS)\n' | make -s -f Makefile -f - progs)
TESTS=$(for f in test/*.sm; do
	b=$(basename "$f" .sm)
	case $b in bad_*) ;; *) echo "$b" ;; esac
done)

build() {
	targets="all $S/test_arith"
	for p in $PROGS; do
		targets="$targets $S/$p"
	done
	make -s -j2 $targets
}

# The arithmetic of the expressions and of the bends.
arith() {
	$S/test_arith
}

# Each test/NAME.sm folds, its measure output matches test/NAME.expected,
# and every writer runs on it.  The two pieces of twopiece.sm make two
# solids in STEP.
parts() {
	for t in $TESTS; do
		$S/smfold test/$t.sm > build/$t.fold &&
			$S/measure build/$t.fold > build/$t.measure &&
			$S/numdiff $TOL test/$t.expected build/$t.measure &&
			$S/tostep build/$t.fold > build/$t.step &&
			$S/smflat -f dxf build/$t.fold > build/$t.dxf ||
			{ echo "FAIL part $t"; return 1; }
	done
	test "$(grep -c MANIFOLD_SOLID_BREP build/twopiece.step)" -eq 2 ||
		{ echo "FAIL twopiece solids"; return 1; }
}

# The place and interfere programs find the one overlapping pair, the
# move in a placement comes after its rotations, colours from a table
# reach the fold and STEP files, and three copies of a part in one place
# collide fast (fuzzer finding 0008).
place() {
	parts > /dev/null || return 1
	$S/place test/pair.txt > build/pair.fold &&
		$S/interfere build/pair.fold > build/pair.out &&
		grep -q '^a b points' build/pair.out &&
		test "$(wc -l < build/pair.out)" -eq 1 || return 1
	$S/place test/order.txt > build/order.fold &&
		test "$(grep -c '^place 0 -1 0 1 0 0 0 0 1 10 0 0$' \
			build/order.fold)" -eq 2 || return 1
	$S/tostep test/quote.fold | grep -q "BREP('it''s'" || return 1
	$S/place test/colour.txt > build/colour.fold &&
		grep -q '^colour red 0.8 0.1 0.1 0.5$' build/colour.fold &&
		test "$($S/tostep build/colour.fold | grep -c STYLED_ITEM)" \
			-eq 2 || return 1
	$S/place test/coincident.txt > build/coincident.fold &&
		timeout 10 $S/interfere build/coincident.fold \
			> build/coincident.out &&
		printf '%s\n' 'a0 a0 points 10000' 'a0 a1 points 10000' \
			'a0 a1 points 10000' | cmp -s - build/coincident.out
}

# Each malformed input makes its program fail with a message.
bad() {
	parts > /dev/null || return 1
	for f in test/bad_*.sm test/bad_*.fold test/bad_*.txt; do
		case $f in
		*.sm) p=smfold ;;
		*.fold) p=measure ;;
		*.txt) p=place ;;
		esac
		if $S/$p "$f" > /dev/null 2> build/bad.err ||
			! test -s build/bad.err ||
			grep -q 'Sanitizer' build/bad.err; then
			echo "FAIL $f"
			cat build/bad.err
			return 1
		fi
	done
}

# The programs numdiff and lint accept good input and reject bad, lint
# takes the Makefile and the dot files without a header, and smfold -M
# lists the parameters a part uses through others too.
tools() {
	$S/smfold -M test/deps.sm | sort | tr '\n' ' ' | grep -qx 'A B C ' ||
		{ echo "FAIL smfold -M"; return 1; }
	printf 'volume 100\nbbox 0 0\n' > build/nd.a
	printf 'volume 100.05\nbbox 0.005 0\n' > build/nd.b
	printf 'volume 100.2\nbbox 0 0\n' > build/nd.c
	$S/numdiff $TOL build/nd.a build/nd.b &&
		! $S/numdiff $TOL build/nd.a build/nd.c > /dev/null &&
		! $S/numdiff build/nd.a build/nd.b > /dev/null &&
		$S/numdiff -t volume=abs:0.3 build/nd.a build/nd.c || return 1
	printf '%s\n' '// SPDX-License-Identifier: GPL-3.0-only' \
		'// ok.c -- x' '// Copyright (c) 2026 Jakob Kastelic' '' \
		'int x;' > build/ok.c
	sed 's/ok.c/no.c/' build/ok.c > build/bad.c
	sed 's|int x;|int x; /''* y *''/|; s/ok.c/bad2.c/' build/ok.c \
		> build/bad2.c
	printf '%s\n' '#!/bin/sh' '# SPDX-License-Identifier: GPL-3.0-only' \
		'# ok.sh -- x' '# Copyright (c) 2026 Jakob Kastelic' '' \
		'true' > build/ok.sh
	printf '%s\n' '# SPDX-License-Identifier: GPL-3.0-only' \
		'# no.txt -- x' '# Copyright (c) 2026 Jakob Kastelic' '' \
		'a # plicad is a word - not a path -- here' > build/no.txt
	mkdir -p build/bare
	printf '%s\n' '# Build it.' 'all:' '	true' > build/bare/Makefile
	printf '%s\n' '# Leave the build out.' 'build/' > build/bare/.ignore
	sed 's|^#!/bin/sh|#!/bin/sh|; 2,4d' build/ok.sh > build/bare/no.sh
	$S/lint build/bare/Makefile build/bare/.ignore &&
		! $S/lint build/bare/no.sh > /dev/null || return 1
	$S/lint build/ok.c build/ok.sh &&
		! $S/lint build/bad.c > /dev/null &&
		! $S/lint build/bad2.c > /dev/null &&
		test "$($S/lint build/no.txt | wc -l)" -eq 3
}

# README.md shows the cube of test/cube.sm, statement for statement, and
# lists every program; the checked-in cube STEP file is up to date.
docs() {
	awk '/^```$/ { if (inb) exit; if (seen) inb = 1; next }
		/cube can be described/ { seen = 1 } inb' README.md \
		> build/readme.cube
	grep -v '^#' test/cube.sm | grep -v '^$' > build/cube.body
	cmp -s build/readme.cube build/cube.body || {
		echo "README.md: the cube differs from test/cube.sm"
		return 1
	}
	awk -F'|' '/^\| Program/ { t = 1; next } t && /^\|-/ { next }
		t && /^\|/ { gsub(/ /, "", $2); print $2; next } t { exit }' \
		README.md | sort > build/readme.progs
	printf '%s\n' $PROGS | sort > build/progs
	cmp -s build/readme.progs build/progs || {
		echo "README.md: the program table differs from PROGS:"
		diff build/readme.progs build/progs
		return 1
	}
	build/smfold test/cube.sm > build/cube.fold &&
		build/tostep build/cube.fold | cmp -s - test/cube.step
}

# Each program, with stdin closed, fails with a message on stderr and
# nothing on stdout: with no arguments, with an unknown option and with
# a file that is missing.
args() {
	for p in $PROGS; do
		for a in "" "-Q build/nofile" "build/nofile build/nofile"; do
			timeout 5 build/$p $a > build/args.out \
				2> build/args.err <&-
			st=$?
			if test $st -eq 0 || test $st -ge 124; then
				echo "FAIL $p $a: exit $st"
				return 1
			fi
			if test -s build/args.out ||
				! test -s build/args.err; then
				echo "FAIL $p $a: output"
				return 1
			fi
		done
	done
}

# OCCT reads the STEP files of the test parts as valid solids, as many
# as each file writes, with the volume and area of measure.
occt() {
	test/occt.sh $S "$(echo $TESTS)"
}

build || { echo "FAIL build"; exit 1; }
groups=${*:-arith parts place bad tools docs args occt}
fail=0
for g in $groups; do
	case $g in
	arith | parts | place | bad | tools | docs | args | occt) ;;
	*)
		echo "unknown group $g"
		exit 2
		;;
	esac
	if $g > "build/test.$g.log" 2>&1; then
		echo "ok $g"
	else
		echo "FAIL $g"
		cat "build/test.$g.log"
		fail=1
	fi
done
exit $fail
