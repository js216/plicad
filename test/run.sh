#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# run.sh -- Run the tests of PliCad
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/run.sh [GROUP...]
# The groups are arith, parts, solids, fillet, place, bad, tools, docs,
# args, draw, mesh, viewer, occt, eda and sizes;
# with none it runs all of them.  It builds the programs with make
# first, and the tests run the sanitized ones in build/san.  It prints
# ok or FAIL for each group and exits non-zero if any fails.  The occt
# group needs FreeCAD (FREECAD, see test/freecad.sh).
# Each run of a program is a case that build/budget (test/budget.c)
# runs, named by its group, the program and its arguments: its peak
# resident set and its time on the processor must keep within their
# budgets in test/run.budget; and the sizes group holds the source
# lines and the bytes of each program to theirs in test/size.budget.

cd "$(dirname "$0")/.." || exit 1
# PLICAD_BIN names other programs to test, made elsewhere.  The tests
# run each through a runner of the same name in build/run, S, which
# holds that run to its budget (runners).
R=${PLICAD_BIN:-build/san}
S=build/run
TOL="-t volume=rel:1e-3 -t area=rel:1e-3 -t bbox=abs:0.01 \
-t hole=abs:0.01 -t d=abs:0.001 -t axis=abs:0.001 -t flat=abs:0.01"
PROGS=$(printf 'progs:\n\t@echo $(PROGS)\n' | make -s -f Makefile -f - progs)
TESTS=$(for f in test/*.sm; do
	b=$(basename "$f" .sm)
	case $b in bad_*) ;; *) echo "$b" ;; esac
done)
SOLIDS=$(for f in test/*.part; do
	b=$(basename "$f" .part)
	case $b in bad_* | bootsharp | roundfirst | include | draft[whps]* | \
		block | groovedraft | tapped | coatdisc | twotex | roundsect) ;;
	*) echo "$b" ;;
	esac
done)
STOL="-t volume=rel:1e-9 -t area=rel:1e-9 -t center=abs:2e-4 \
-t bbox=abs:2e-4"

build() {
	targets="all build/budget $R/test_arith $R/smdraw"
	for p in $PROGS; do
		targets="$targets $R/$p"
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
		$S/smfold test/$t.sm > build/$t.st &&
			$S/measure build/$t.st > build/$t.measure &&
			$S/numdiff $TOL test/$t.expected build/$t.measure &&
			$S/tostep build/$t.st > build/$t.step &&
			$S/smflat -f dxf build/$t.st > build/$t.dxf ||
			{ echo "FAIL part $t"; return 1; }
	done
	test "$(grep -c MANIFOLD_SOLID_BREP build/twopiece.step)" -eq 2 ||
		{ echo "FAIL twopiece solids"; return 1; }
	inserts
}

# The inserts of test/localins/plate.sm: its table next to it adds its
# types to those of the library, which gives the inch types of PEM.  The
# stream holds all of each insert, its body and its material, so that
# read from elsewhere, with no library, it gives the bodies: the flush
# nut fills its hole, flush with both faces, and the stud of a blind
# hole fills the hole and stands on the face, its hole 1.5 deep, which
# leaves the far face whole and is no cut of the flat pattern, as a
# concealed-head stud of the library does too.  An insert named by as
# names its body, and its hole keeps the name of the statement.
inserts() {
	mkdir -p build/far
	st=build/far/ins.st
	$S/smfold test/localins/plate.sm > $st &&
		grep -q '^insert nut_c s-632-1 far 2 ' $st &&
		grep -q '^insert d f-632-2 near 3 .* fills material' $st &&
		grep -q '^hole 0 50 10 .* by c$' $st &&
		grep -q '^hole 0 30 10 2 blind 1.5 near by b$' $st &&
		grep -q '^hole 0 50 40 .* blind 1.0922 near by i$' $st &&
		grep -q '^material brass ' $st &&
		grep -q '^material stainless ' $st &&
		test "$($S/smflat -f dxf $st | grep -c '^CIRCLE$')" -eq 8 ||
		{ echo "FAIL insert stream"; return 1; }
	m="$(cd "$S" && pwd)/measure"
	(cd build/far && PLICAD=/nonexistent "$m" ins.st) \
		> build/far/ins.measure ||
		{ echo "FAIL inserts read without a library"; return 1; }
	awk 'function r2(d) { return d * d / 4 }
		function near(a, b, e) { return a - b < e && b - a < e }
		/^part / { p = $2 }
		/^volume / { v[p] = $2 }
		END {
			pi = 3.14159265358979; inch = 25.4
			t = 2.5
			through = r2(5) + 2 * r2(0.1875 * inch) + \
				r2(0.213 * inch) + r2(0.137 * inch) + \
				3 * r2(0.166 * inch)
			blind = pi * 4 * 1.5 + pi * r2(0.213 * inch) * 1.0922
			plate = 120 * 60 * t - pi * t * through - blind
			d = pi * r2(0.213 * inch) * t
			b = pi * 4 * 1.5 + pi * 2.25 * 8
			e = pi * r2(0.137 * inch) * t + \
				pi * r2(0.138 * inch) * 0.25 * inch
			f = pi * r2(0.187 * inch) * 0.25 * inch
			ok = near(v["plate"], plate, 1e-3)
			ok = ok && near(v["plate/d"], d, 1e-4)
			ok = ok && near(v["plate/b"], b, 1e-4)
			ok = ok && near(v["plate/e"], e, 1e-4)
			ok = ok && near(v["plate/f"], f, 1e-4)
			ok = ok && ("plate/nut_c" in v) && !("plate/c" in v)
			ok = ok && ("plate/g" in v) && ("plate/h" in v)
			exit !ok
		}' build/far/ins.measure ||
		{ echo "FAIL insert bodies"; return 1; }
	$S/interfere $st > build/far/ins.clash &&
		test ! -s build/far/ins.clash ||
		{ echo "FAIL inserts touch their sheet"; return 1; }
}


# The stream of the part source $1 through every program that makes
# solids, into $2.
solid() {
	$S/extrude "$1" > "$2.1" && $S/bool "$2.1" > "$2.2" &&
		$S/fillet "$2.2" > "$2"
}

# Each test/NAME.part makes its solids, their measure output matches
# test/NAME.expected, and tostep takes the stream.  A part source reads
# the parameters of test/channel.params by include (test/include.part).
# The mold check finds the walls and the drafts of the boot as
# test/boot.mold has them: the faces of its set pins, the screw holes
# and the cavity, are pins, and no face is an undercut.
solids() {
	for t in $SOLIDS; do
		solid test/$t.part build/part_$t.st &&
			$S/measure build/part_$t.st > build/part_$t.measure &&
			$S/numdiff $STOL test/$t.expected \
				build/part_$t.measure &&
			$S/tostep build/part_$t.st > build/part_$t.step ||
			{ echo "FAIL solid $t"; return 1; }
	done
	solid test/include.part build/part_include.st &&
		$S/measure build/part_include.st > build/part_include.measure &&
		$S/numdiff $STOL test/include.expected \
			build/part_include.measure ||
		{ echo "FAIL solid include"; return 1; }
	$S/mold y build/part_boot.st pins |
		cmp -s - test/boot.mold ||
		{ echo "FAIL mold boot"; return 1; }
	! grep -q '^undercut' test/boot.mold || return 1
	# A face whose texture needs more draft than it has fails the mold
	# check, with its draft and the least of its texture.
	$S/mold z build/part_textured.st > /dev/null &&
		! $S/mold z build/part_shallow.st > /dev/null \
			2> build/shallow.err &&
		grep -q 'blk.side has a draft of 1.000 degrees, under the 3' \
			build/shallow.err ||
		{ echo "FAIL mold texture"; return 1; }
}

# The fillet program lists the edges of a part, and bool refuses a solid
# with round faces.  A rule whose edges a later rule all takes is
# refused with the line of that rule (test/bad_override.part).  A round
# larger than a drafted corner where it meets it is refused with the
# face of the corner and its radius there (test/bad_bigcorner.part).
fillet() {
	$S/extrude test/oneround.part | $S/fillet -l - |
		cmp -s - test/oneround.edges || return 1
	$S/extrude test/bad_override.part | $S/fillet - 2>&1 > /dev/null |
		grep -q ':7: this rule matches no edge: the rule of line 8' ||
		{ echo "FAIL fillet override"; return 1; }
	$S/extrude test/bad_bigcorner.part | $S/fillet - 2>&1 > /dev/null |
		grep -q ':10: the round of 5.08 of the edge of box.end and '\
'box.side is larger than the corner of box.side there, of radius '\
'2.69406 at 3.09501 -14 3.09501' ||
		{ echo "FAIL fillet round larger than a corner"; return 1; }
	$S/extrude test/roundfirst.part | $S/fillet - > build/roundfirst.st &&
		! $S/bool build/roundfirst.st > /dev/null 2> build/bad.err &&
		grep -q 'bool takes faces on planes' build/bad.err || return 1
	pipes || return 1
	# A drafted block that a plate crosses, its front stepping back
	# sharply (test/draftstep.part), rounds where the two meet: at the
	# ends of a round the strip of another one ends, and holds none of
	# it.
	solid test/draftstep.part build/part_draftstep.st ||
		{ echo "FAIL rounds at a drafted step"; return 1; }
	# A groove that ends against a wall in a drafted floor rounds where
	# it meets the floor (test/groovedraft.part), and tostep takes it.
	solid test/groovedraft.part build/part_groovedraft.st &&
		$S/tostep build/part_groovedraft.st > build/groovedraft.step ||
		{ echo "FAIL groove in a drafted floor"; return 1; }
}

# Rounds along ellipses: where the drafted faces of a box meet the
# cylinders of a wave across its front (test/draftwave.part), eight,
# and where a counterbored hole meets them (test/drafthole.part), four,
# the rounds are pipes, which tostep writes as rational B-spline
# surfaces, one for each.  A round takes from the solid less than a
# square band of its radius along its edges, as fillet -l gives their
# lengths, and more than a tenth of it.
pipes() {
	for t in draftwave:8:box:wave drafthole:4:b:hole/bore \
		draftplate:2:box:plate; do
		n=${t#*:}
		k=${n%%:*}
		pat=${n#*:}
		a=${pat%%:*}
		b=$(echo "${pat#*:}" | tr / '|')
		t=${t%%:*}
		solid test/$t.part build/part_$t.st &&
			$S/measure build/part_$t.st > build/part_$t.measure &&
			grep -q "^faces .* pipe $k\$" build/part_$t.measure &&
			$S/tostep build/part_$t.st > build/part_$t.step &&
			test "$(grep -c RATIONAL_B_SPLINE_SURFACE \
				build/part_$t.step)" -eq "$k" ||
			{ echo "FAIL pipes $t"; return 1; }
		grep -v '^round' test/$t.part > build/sharp_$t.part
		$S/extrude build/sharp_$t.part | $S/bool - | $S/measure - \
			> build/sharp_$t.measure &&
			$S/extrude test/$t.part | $S/bool - | $S/fillet -l - \
				> build/$t.edges || return 1
		awk -v a="^$a[.]" -v b="^($b)[.]" -v r=5.08 -v t=$t '
			FILENAME ~ /edges$/ && (($2 ~ a && $3 ~ b) ||
				($2 ~ b && $3 ~ a)) { l += $6 }
			FILENAME ~ /sharp/ && /^volume/ { v0 = $2 }
			FILENAME ~ /part_/ && /^volume/ { v1 = $2 }
			END {
				if (t != "draftwave") r = 1
				d = v0 - v1
				exit !(d > 0.1 * r * r * l && d < r * r * l)
			}' build/$t.edges build/sharp_$t.measure \
			build/part_$t.measure ||
			{ echo "FAIL pipe volume $t"; return 1; }
	done
}

# The place and interfere programs find the one overlapping pair, the
# steps of a place apply in their order, materials from a table reach
# the stream and the STEP file and give measure the masses and their
# centers, and three copies of a part in one place collide fast (fuzzer
# finding 0008).  An assembly of a folded part and solid parts
# (test/mixed.txt) measures as test/mixed.expected has it, writes four
# STEP solids, and collides only where a solid goes through the plate,
# not where one stands on it.  Solids with rounds, spheres and tori,
# collide where one reaches into another and not where they touch
# (test/rounded.txt), and so does the boot with a plate through its wall
# (test/booted.txt).  assemble brings lids down onto a plate to touch,
# each path sampled as the file and its line say (test/lids.txt), and
# says a lid that passes nearer a plate than the clearance on its way,
# refuses a path without a step (test/nostep.txt), and finds the clash
# of a path that runs through a part (test/through.txt).  Subassemblies
# nest (test/nested.txt): their parts
# take full names, collide by them, and measure gives each unit its mass
# and center; a file that holds itself is refused with the chain of
# files (test/bad_cycle.txt).  Flat heads in countersinks collide only when
# sunk into them, not when a cone lies on a cone or touches it along a
# circle (test/seated.txt).  With -s, place writes the scene of an assembly,
# a picture on a part in the frame of that part (test/pictured.txt).
place() {
	parts > /dev/null || return 1
	$S/place test/pair.txt > build/pair.st &&
		$S/interfere build/pair.st > build/pair.out &&
		grep -q '^a b points' build/pair.out &&
		test "$(wc -l < build/pair.out)" -eq 1 || return 1
	$S/place test/order.txt > build/order.st &&
		grep -q '^place 0 -1 0 1 0 0 0 0 1 10 0 0$' build/order.st &&
		grep -q '^place 0 -1 0 1 0 0 0 0 1 0 10 0$' build/order.st ||
		return 1
	$S/tostep test/quote.st | grep -q "BREP('it''s'" || return 1
	$S/place -s m test/pictured.txt | grep -v '^#' > build/pictured &&
		grep -qx 'mesh a m/a.obj turn z 90 about 0 0 0 move 50 0 0' \
			build/pictured &&
		grep -qx 'image label test/pic.ppm on a 0 0 1 10 0 1 0 5 1' \
			build/pictured &&
		test "$(wc -l < build/pictured)" -eq 2 || return 1
	$S/place test/color.txt > build/color.st &&
		grep -q '^material red 0.8 0.1 0.1 0.5 2.68$' build/color.st &&
		grep -q '^base red$' build/color.st &&
		test "$($S/tostep build/color.st | grep -c STYLED_ITEM)" \
			-eq 2 &&
		$S/measure build/color.st > build/color.measure &&
		$S/numdiff -t mass=rel:1e-6 -t center=abs:1e-4 \
			test/color.expected build/color.measure || return 1
	solid test/point.part build/part_point.st &&
		$S/place test/mixed.txt > build/mixed.st &&
		$S/interfere build/mixed.st > build/mixed.out &&
		grep -q '^plate through points' build/mixed.out &&
		test "$(wc -l < build/mixed.out)" -eq 1 &&
		$S/measure build/mixed.st > build/mixed.measure &&
		$S/numdiff $STOL -t mass=rel:1e-6 test/mixed.expected \
			build/mixed.measure &&
		test "$($S/tostep build/mixed.st |
			grep -c MANIFOLD_SOLID_BREP)" -eq 4 || return 1
	solid test/rounded.part build/part_rounded.st &&
		$S/place test/rounded.txt > build/rounded.st &&
		$S/interfere build/rounded.st > build/rounded.out &&
		grep -q '^base into points' build/rounded.out &&
		test "$(wc -l < build/rounded.out)" -eq 1 || return 1
	$S/place test/nested.txt > build/nested.st &&
		test "$(grep '^part' build/nested.st | tr '\n' ' ')" = \
			"part p1/plate part p1/knob/base part p1/knob/cap \
part p2/plate part p2/knob/base part p2/knob/cap " &&
		$S/interfere build/nested.st > build/nested.out &&
		grep -q '^p1/knob/cap p2/plate points' build/nested.out &&
		test "$(wc -l < build/nested.out)" -eq 1 &&
		$S/measure build/nested.st | grep '^unit' > build/units &&
		printf 'unit %s mass %s center 48.8774 25.0000 %s\n' \
			p1 73.6816 -4.3333 p1/knob 49.1211 4.0000 \
			p2 73.6816 24.6667 p2/knob 49.1211 33.0000 |
		cmp -s - build/units &&
		! $S/place test/bad_cycle.txt 2> build/cycle.err &&
		grep -q 'a cycle of subassemblies' build/cycle.err &&
		c=test/bad_cycle.txt && grep -q "$c, test/cycle.txt, $c" \
			build/cycle.err ||
		{ echo "FAIL place nested"; return 1; }
	$S/assemble -v test/lids.txt > build/lids.out 2>&1 &&
		printf 'path %s steps %s samples %s\n' base 0 1 lid 1 121 \
			top 1 31 | cmp -s - build/lids.out &&
		sed 's/^clearance 0.2/clearance 0.3/' test/lids.txt \
			> build/near.txt &&
		$S/assemble build/near.txt 2> build/near.err &&
		grep -q 'near.txt:11: lid passes 0.25 mm from base' \
			build/near.err &&
		! $S/assemble test/nostep.txt 2> build/nostep.err &&
		grep -q 'nostep.txt:6: a path needs a step' build/nostep.err &&
		! $S/assemble test/through.txt 2> build/through.err &&
		grep -q 'path of fin, step 1 at .* mm: fin clashes with base' \
			build/through.err ||
		{ echo "FAIL assemble"; return 1; }
	solid test/flathead.part build/part_flathead.st &&
		solid test/steephead.part build/part_steephead.st &&
		$S/smfold test/countersunk.sm > build/countersunk.st &&
		$S/place test/seated.txt > build/seated.st &&
		$S/interfere build/seated.st > build/seated.out &&
		grep -q '^plate deep points' build/seated.out &&
		test "$(wc -l < build/seated.out)" -eq 1 || return 1
	solid test/boot.part build/part_boot.st &&
		$S/smfold test/plate.sm > build/plate.st &&
		$S/place test/booted.txt > build/booted.st &&
		$S/interfere build/booted.st > build/booted.out &&
		grep -q '^boot through points' build/booted.out &&
		test "$(wc -l < build/booted.out)" -eq 1 || return 1
	$S/place test/coincident.txt > build/coincident.st &&
		timeout 10 $S/interfere build/coincident.st \
			> build/coincident.out &&
		printf '%s\n' 'a0 a1 points 10000' 'a0 a2 points 10000' \
			'a1 a2 points 10000' | cmp -s - build/coincident.out
}

# Each malformed input makes its program fail with a message.
bad() {
	parts > /dev/null || return 1
	for f in test/bad_*.part; do
		if solid "$f" build/bad.st 2> build/bad.err &&
			$S/tostep build/bad.st > /dev/null 2>> build/bad.err ||
			! test -s build/bad.err ||
			grep -q 'Sanitizer' build/bad.err; then
			echo "FAIL $f"
			cat build/bad.err
			return 1
		fi
	done
	for f in test/bad_*.sm test/bad_*.st test/bad_*.txt; do
		case $f in
		*.sm) p=smfold ;;
		*.st) p=measure ;;
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

# family writes the source of a pan head screw M3 by 10 of the table
# screws of the library, a head and a shank of their volume.
# The programs numdiff and lint accept good input and reject bad, lint
# takes the Makefile and the dot files without a header, smfold -M lists
# the parameters a part uses through others too, param prints the values
# of names and expressions of a source file and refuses a name it lacks,
# and a second fuzzer in one WORK directory fails at once.
tools() {
	$S/smfold -M test/deps.sm | sort | tr '\n' ' ' | grep -qx 'A B C ' ||
		{ echo "FAIL smfold -M"; return 1; }
	$S/family screws M3 pan 10 | $S/extrude - | $S/bool - | $S/measure - |
		grep -qx 'volume 129.7980' || { echo "FAIL family"; return 1; }
	# The bought parts of the library make their solids: the fan, and
	# the mount and the strap of the handle, whose volumes are those of
	# their plates less the corners of 1 mm.
	solid lib/parts/sunon/EEC0252B3-000U-A99.part build/fan.st &&
		$S/tostep build/fan.st > build/fan.step || return 1
	for k in mount:19.05:25.4:6.5 strap:190.5:22:3; do
		$S/smfold lib/parts/keystone/8353_${k%%:*}.sm |
			$S/measure - | awk -v k="$k" '/^volume/ {
				split(k, d, ":"); pi = 3.14159265358979
				v = (d[2] * d[3] - 4 * (1 - pi / 4)) * d[4]
				ok = $2 - v < 1e-3 && v - $2 < 1e-3 }
				END { exit !ok }' ||
			{ echo "FAIL handle $k"; return 1; }
	done
	# A table screws where family runs adds its rows to those of the
	# library: the row of its own and the 4-40 low head of the library
	# both make screws.
	mkdir -p build/fam
	printf '%s\n' 'X1 odd 5 1 2 0.4' > build/fam/screws
	f="$(cd "$S" && pwd)/family"
	(cd build/fam && "$f" screws X1 odd 4) |
		grep -qx 'extrude s from -4 to 0 shank' &&
		(cd build/fam && "$f" screws 4-40 lowhead 0.25in) |
		grep -qx 'extrude h from 0 to 1.4986 head' ||
		{ echo "FAIL family adds to the library"; return 1; }
	test "$($S/param test/channel.sm L W)" = "$(printf '101.6\n100')" &&
		test "$($S/param -e 'L/2 - 1in' test/channel.sm)" = 25.4 &&
		! $S/param test/channel.sm nothing 2> /dev/null ||
		{ echo "FAIL param"; return 1; }
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
		'# The language is that of plicad(7).' 'true' > build/ok.sh
	printf '%s\n' '# SPDX-License-Identifier: GPL-3.0-only' \
		'# no.txt -- x' '# Copyright (c) 2026 Jakob Kastelic' '' \
		'a # plicad is a word - not a path -- here' > build/no.txt
	mkdir -p build/bare
	printf '%s\n' '# Build it.' 'all:' '	true' > build/bare/Makefile
	printf '%s\n' '# Leave the build out.' 'build/' > build/bare/.ignore
	sed 's|^#!/bin/sh|#!/bin/sh|; 2,4d' build/ok.sh > build/bare/no.sh
	$S/lint build/bare/Makefile build/bare/.ignore &&
		! $S/lint build/bare/no.sh > /dev/null || return 1
	fz=$(mktemp -d) || return 1
	flock "$fz/fuzz.lock" sleep 5 &
	sleep 1
	if test/fuzzrun.sh "$fz" > /dev/null 2> "$fz/err" ||
		! grep -q 'another fuzzer' "$fz/err"; then
		echo "FAIL a second fuzzer started"
		return 1
	fi
	wait
	rm -rf "$fz"
	$S/lint build/ok.c build/ok.sh &&
		! $S/lint build/bad.c > /dev/null &&
		! $S/lint build/bad2.c > /dev/null &&
		test "$($S/lint build/no.txt | wc -l)" -eq 3
}

# README.md shows the cube of test/cube.sm, statement for statement, and
# lists every program; the checked-in cube STEP file is up to date.
# Each program has a manual page in man/man1 whose SYNOPSIS names it
# and the options of its usage message, and every page of man/man1,
# man/man5 and man/man7 renders under groff -ww without a warning
# (mancheck.sh).
docs() {
	# The widths of the lettering of drawings are those of osifont.
	python3 -I test/fontcheck.py lib/fonts/osifont-lgpl3fe.ttf src/font.c ||
		{ echo "FAIL font widths"; return 1; }
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
	build/smfold test/cube.sm > build/cube.st &&
		build/tostep build/cube.st | cmp -s - test/cube.step ||
		return 1
	test/mancheck.sh $S $PROGS smdraw ||
		{ echo "FAIL manual pages"; return 1; }
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

# The drawing programs.  Each dimension in the top, front and flat views
# of the test parts says what measure says of the part (dimcheck.sh);
# not so for angled, whose hole lies under its flange, nor for profile,
# whose two holes of one size differ in their countersinks, nor for
# rowcorner and shortline, whose front views see holes at an angle.  The views
# of the bracket with their hidden lines, and the drawings of the cube
# and the bracket, come out as checked in.  Bad input is refused
# (drawbad.sh), and the faults once found stay away (drawfix.sh).
draw() {
	parts > /dev/null && place > /dev/null || return 1
	for t in $TESTS; do
		case $t in angled | profile | rowcorner | shortline) continue ;;
		esac
		test/dimcheck.sh $S build/$t.st top front flat ||
			{ echo "FAIL dimensions $t"; return 1; }
	done
	for v in top front right flat; do
		$S/smview -h $v build/bracket.st |
			cmp -s - test/bracket.$v.expected ||
			{ echo "FAIL view bracket $v"; return 1; }
	done
	for t in cube bracket; do
		PLICAD_BIN=$S build/smdraw test/$t.draw build/$t.st \
			> build/$t.ps &&
			cmp -s build/$t.ps test/$t.ps.expected ||
			{ echo "FAIL drawing $t"; return 1; }
	done
	test/drawbad.sh $S && test/drawfix.sh $S || return 1
	# The coats that a view sees face on are hatched, their sides drawn
	# in phantom lines, and a linear dimension spans the black band of
	# test/coat.sm from side to side (test/coat.draw).
	$S/smfold test/coat.sm > build/coat.st &&
		PLICAD_BIN=$S build/smdraw test/coat.draw build/coat.st \
			> build/coatdraw.ps &&
		$S/smview front build/coat.st | $S/smdim > build/coat.dim &&
		test "$(grep -c '^line p v' build/coat.dim)" -eq 8 &&
		test "$(grep -c '^hatch' build/coat.dim)" -eq 2 ||
		{ echo "FAIL drawing coats"; return 1; }
	# The coats of a material that an unhatched record names get no
	# hatch, their sides drawn as before; smsheet passes the record to
	# smdim under each view.
	{ $S/smview front build/coat.st; echo 'unhatched clear black'; } |
		$S/smdim > build/coat.unhatched.dim &&
		test "$(grep -c '^hatch' build/coat.unhatched.dim)" -eq 0 &&
		test "$(grep -c '^line p v' build/coat.unhatched.dim)" -eq 8 &&
		{ cat test/coat.draw; echo 'unhatched black'; } \
			> build/coatu.draw &&
		$S/smsheet -v build/coatu.draw | grep -qx ' unhatched black' ||
		{ echo "FAIL drawing coats unhatched"; return 1; }
	# A textured coat bears its mark, and its callout goes into the
	# notes (test/patch.draw).
	PLICAD_BIN=$S build/smdraw test/patch.draw build/patch.st \
		> build/patch.ps &&
		grep -q 'TEXTURE: VDI 3400 REF 30 ON SURFACES MARKED' \
			build/patch.ps &&
		grep -q '^\[(T)\]' build/patch.ps ||
		{ echo "FAIL drawing texture"; return 1; }
	# The inserts of one type on one side share a callout of the format
	# of the drawing, their holes no callout of their own; the stream
	# gives each its body, an exact solid of its own (test/inserts.draw).
	# Drawings of solid parts: the machined block (test/block.part)
	# seen from the top shows its two holes, which smdim calls out, and
	# from the front its holes hidden, and a section across them filled,
	# a region of the faces cut of its one part;
	# the drafted block of a texture (test/textured.part) shows the
	# texture on the face that looks at the eye, marked T, and the
	# drawing notes it.
	solid test/block.part build/part_block.st &&
		solid test/textured.part build/part_textured.st &&
		$S/smview top build/part_block.st > build/block.top &&
		test "$(grep -c '^hole ' build/block.top)" -eq 2 &&
		$S/smdim < build/block.top | grep -q '^dim hole' &&
		$S/smview -h front build/part_block.st | grep -q '^line o h ' &&
		$S/smview -c -15 front build/part_block.st |
		grep -qx 'fill cut 0' &&
		PLICAD_BIN=$S build/smdraw test/block.draw build/part_block.st \
			> build/block.ps &&
		$S/smview front build/part_textured.st |
		grep -q '^texture vdi30 T VDI 3400 REF 30$' &&
		PLICAD_BIN=$S build/smdraw test/molded.draw \
			build/part_textured.st | grep -q 'VDI 3400 REF 30' ||
		{ echo "FAIL drawings of solids"; return 1; }
	# The threads and the countersinks of the holes of a solid
	# (test/tapped.part): seen from the top, the M3 holes tapped through
	# and countersunk at the far side, the 4-40 hole tapped as deep as its
	# drill, 6 from its mouth, and countersunk there, and the plain hole
	# countersunk; their callouts give the threads, not the drills, and
	# the countersinks as those of a sheet (test/tapped.draw).
	t='csink 3.500000 90.000 far thread thru M3X0.5-6H by taps'
	b='hole 20.000000 10.000000 2.260000 deep 6.000000'
	b="$b csink 2.794000 82.000 near thread 6.000000 4-40 UNC-2B by blind"
	c='hole 20.000000 15.000000 3.400000 csink 6.400000 90.000 near'
	solid test/tapped.part build/part_tapped.st &&
		$S/smview top build/part_tapped.st > build/tapped.top &&
		test "$(grep -c "^hole .* $t\$" build/tapped.top)" -eq 2 &&
		grep -qx "$b" build/tapped.top &&
		grep -qx "$c by plain" build/tapped.top &&
		$S/smdim -u mm < build/tapped.top > build/tapped.dim &&
		grep -qx 'dim hole 2.5000 2 M3X0.5-6H THRU' build/tapped.dim &&
		grep -q ' %%v%%c3.50 X 90%%d FARSIDE, 2X$' build/tapped.dim &&
		grep -qx 'dim hole 2.2600 1 4-40 UNC-2B X 6.00 DEEP' \
			build/tapped.dim &&
		grep -q ' %%v%%c2.79 X 82%%d$' build/tapped.dim &&
		grep -qx 'dim hole 3.4000 1 %%c3.40 THRU' build/tapped.dim &&
		PLICAD_BIN=$S build/smdraw test/tapped.draw build/part_tapped.st \
			> build/tapped.ps &&
		grep -q '^\[(M3X0.5-6H THRU)\]' build/tapped.ps ||
		{ echo "FAIL drawing threads"; return 1; }
	# A section of two of the plates, one on the other (test/stack.draw):
	# the faces cut of each part are a region of their own, whose section
	# lines run at 45 degrees and at 135, 1.5 mm apart as the drawing says;
	# the faces cut hide the plain hole behind them.
	printf '%s\n' 'part lower part_tapped.st' \
		'part upper part_tapped.st move 0 0 8' > build/stack.txt &&
		$S/place build/stack.txt > build/stack.st &&
		$S/smview -c -10 front build/stack.st > build/stack.view &&
		grep -qx 'fill cut 0' build/stack.view &&
		grep -qx 'fill cut 1' build/stack.view &&
		! grep -q '^line o v 18.300000 ' build/stack.view &&
		$S/smdim -u mm < build/stack.view > build/stack.dim &&
		grep -qx 'fill hatch 45 0.0800' build/stack.dim &&
		grep -qx 'fill hatch 135 0.0800' build/stack.dim &&
		PLICAD_BIN=$S build/smdraw test/stack.draw build/stack.st \
			> build/stack.ps &&
		grep -qx '45.0000 0.0591 SECT grestore newpath' build/stack.ps &&
		grep -qx '135.0000 0.0591 SECT grestore newpath' build/stack.ps ||
		{ echo "FAIL drawing sections of assemblies"; return 1; }
	# A curve of a solid seen at an angle, of a disc turned about z, is a
	# poly record, of fewer points for a looser chord; none is of the kind
	# u.  The coat of the face of the disc seen face on, a polygon of more
	# points than a record once had room for, goes through smdim and gets
	# its mark, and the circle of the face, two arcs, no radius
	# (test/coatdisc.part).
	solid test/coatdisc.part build/part_coatdisc.st &&
		printf '%s\n' 'part disc part_coatdisc.st turn z 30' \
			> build/tilt.txt &&
		$S/place build/tilt.txt > build/tilt.st &&
		$S/smview -t 0.1 front build/tilt.st > build/tilt.coarse &&
		$S/smview -t 0.001 front build/tilt.st > build/tilt.fine &&
		test "$(grep -c '^poly e v ' build/tilt.fine)" -ge 2 &&
		! grep -q '^line u ' build/tilt.fine &&
		test "$(awk '$1 == "poly" { n += $4 } END { print n }' \
			build/tilt.coarse)" -lt \
			"$(awk '$1 == "poly" { n += $4 } END { print n }' \
				build/tilt.fine)" &&
		$S/smdim < build/tilt.fine | grep -q '^poly e v ' &&
		$S/smview front build/part_coatdisc.st > build/coatdisc.view &&
		test "$(awk '$1 == "coat" { print $4 }' build/coatdisc.view)" \
			-gt 12 &&
		$S/smdim < build/coatdisc.view > build/coatdisc.dim &&
		grep -q '^text .* T$' build/coatdisc.dim &&
		! grep -q '^dim radius' build/coatdisc.dim ||
		{ echo "FAIL drawing curves of solids"; return 1; }
	# The three textures of test/twotex.part: Ref 30, of the most area,
	# is that of all the surfaces not marked, first in each view and with
	# no coat; the others take T and U, the same in every view, and each
	# stands once in a view, on its largest face with room, though the
	# top of the block and those of its bosses are four faces of Ref 18.
	# The drawing notes the marked ones, and last ALL OTHER SURFACES; with
	# -u, Ref 18 is the one not marked instead.
	solid test/twotex.part build/part_twotex.st &&
		$S/smview top build/part_twotex.st > build/twotex.top &&
		$S/smview front build/part_twotex.st > build/twotex.front &&
		test "$(grep -m 1 '^texture' build/twotex.top)" = \
			'texture vdi30 - VDI 3400 REF 30' &&
		! grep -q '^coat vdi30 ' build/twotex.top build/twotex.front &&
		test "$(grep -c '^coat vdi18 ' build/twotex.top)" -eq 4 &&
		r18=$(awk '$1 == "texture" && $2 == "vdi18" { print $3 }' \
			build/twotex.top) &&
		r24=$(awk '$1 == "texture" && $2 == "vdi24" { print $3 }' \
			build/twotex.front) &&
		{ test "$r18$r24" = UT || test "$r18$r24" = TU; } &&
		$S/smdim < build/twotex.top > build/twotex.top.dim &&
		test "$(grep -c "^text .* $r18\$" build/twotex.top.dim)" -eq 1 &&
		$S/smdim < build/twotex.front > build/twotex.front.dim &&
		test "$(grep -c "^text .* $r24\$" build/twotex.front.dim)" -eq 1 &&
		PLICAD_BIN=$S build/smdraw test/twotex.draw build/part_twotex.st \
			> build/twotex.ps &&
		grep -o 'TEXTURE: [^)]*' build/twotex.ps | tail -1 |
		grep -qx 'TEXTURE: ALL OTHER SURFACES VDI 3400 REF 30' &&
		grep -q "REF 18 ON SURFACES MARKED \"$r18\"" build/twotex.ps &&
		grep -q "REF 24 ON SURFACES MARKED \"$r24\"" build/twotex.ps &&
		$S/smview -u vdi18 top build/part_twotex.st |
		grep -qx 'texture vdi18 - VDI 3400 REF 18' ||
		{ echo "FAIL drawing marks of textures"; return 1; }
	# A section keeps each flat side that runs into rounds as one line,
	# upright, however loose the chord that thins the rounds, so that a
	# lin finds its ends (test/roundsect.part): thinned as one loop, the
	# short sides once leaned into the rounds and no lin found them.
	solid test/roundsect.part build/part_roundsect.st || return 1
	for t in 0.01 0.1; do
		$S/smview -t $t -c -10 front build/part_roundsect.st \
			> build/roundsect.view &&
			awk '$1 == "line" && $2 == "e" && $4 == $6 &&
				($4 == "0.000000" || $4 == "30.000000") &&
				$5 + $7 == 10 && ($5 == "4.600000" ||
				$7 == "4.600000") { n++ }
				END { exit n != 2 }' build/roundsect.view &&
			{ cat build/roundsect.view; echo 'lin x 0 1.181102'; } |
			$S/smdim | grep -qx 'dim lin x 30.0000 1.181' ||
			{ echo "FAIL drawing lin on a section, chord $t"; return 1; }
	done
	PLICAD_BIN=$S build/smdraw test/inserts.draw build/inserts.st \
		> build/inserts.ps &&
		grep -q '3 X NUT-M3, INSTALL FAR SIDE' build/inserts.ps &&
		grep -q '1 X STANDOFF-M3, INSTALL NEAR SIDE' build/inserts.ps &&
		$S/smview top build/inserts.st | $S/smdim > build/inserts.dim &&
		! grep -q '^dim hole' build/inserts.dim &&
		test "$($S/tostep build/inserts.st | grep -c MANIFOLD)" -eq 5 ||
		{ echo "FAIL drawing inserts"; return 1; }
	# A drawing in mm gives each size in mm and in inches after it in
	# brackets, and tells the template so (test/metric.draw).
	PLICAD_BIN=$S build/smdraw test/metric.draw build/plate.st \
		> build/metric.ps &&
		grep -q '(100.00 \[3.937\])' build/metric.ps &&
		grep -q '(50.00 \[1.969\])' build/metric.ps &&
		grep -q '/Units (MILLIMETERS \[INCHES\]) def' build/metric.ps ||
		{ echo "FAIL drawing in mm"; return 1; }
	# A drawing whose notes run longer than a line keeps every line in
	# the region of the notes of plain_b.ps, and the label SIZE of its
	# title block in its cell, whatever font Ghostscript draws them in;
	# a linear dimension spans the inside of a notch, from edges that the
	# view sees on end; and the Filename field keeps the extension.
	PLICAD_BIN=$S build/smdraw test/inside.draw build/profile.st \
		> build/inside.ps &&
		grep -q '/Filename (inside.draw) def' build/inside.ps ||
		{ echo "FAIL drawing inside"; return 1; }
	command -v ps2pdf > /dev/null && command -v pdftotext > /dev/null ||
		return 0
	ps2pdf build/inside.ps build/inside.pdf &&
		pdftotext -bbox build/inside.pdf build/inside.box &&
		test/inbox.sh build/inside.box ||
		{ echo "FAIL drawing inside: text out of room"; return 1; }
}

# OCCT reads the STEP files of the test parts as valid solids, as many
# as each file writes, with the volume and area of measure, and the
# solid of nearhem.sm equals the reference solid that test/occtworker.py
# builds from its source (fuzzer finding 0011).  Each solid of a
# test/NAME.part is one valid solid to OCCT, with the volume and area of
# measure, and equals the reference solid of test/occtworker.py, or for
# bnc.part, the Creo solid in test/bnc.brep; the empty solid of
# test/apart.part is none.  A part with rounds has no reference
# solid.  The visible lines of the bottom view of shortline,
# some of them .2 mm long, are those of the hidden-line removal of OCCT
# (fuzzer finding 0023).  The answers of the worker come on a
# descriptor of their own, apart from the trace of OCCT.
occt() {
	test/occt.sh $S "$(echo $TESTS)" || return 1
	solids > /dev/null && pipes > /dev/null || return 1
	for t in $SOLIDS draftwave drafthole draftplate; do
		if test $t = bnc; then
			ref='"brep": "test/bnc.brep"'
		elif grep -q '^round' test/$t.part; then
			ref='"spec": null'
		else
			ref="\"spec\": \"test/$t.part\""
		fi
		printf '{"step": "build/part_%s.step", %s}\n' $t "$ref"
	done | OCCTWORKER_FD=3 test/freecad.sh test/occtworker.py \
		3> build/part_occt.raw > /dev/null 2>&1
	sed 's/^R //' build/part_occt.raw > build/part_occt.json
	for t in $SOLIDS draftwave drafthole draftplate; do
		ref=
		if grep -q '^round' test/$t.part; then
			ref=-n
		fi
		m=build/part_$t.measure
		grep "\"build/part_$t.step\"" build/part_occt.json |
			python3 -I test/occtpart.py $ref $m ||
			{ echo "FAIL occt solid $t"; return 1; }
	done
	printf '{"step": "build/nearhem.step", "spec": "test/nearhem.sm"}\n' |
		OCCTWORKER_FD=3 test/freecad.sh test/occtworker.py \
			3> build/nearhem.ref > /dev/null 2>&1
	grep -q '"symdiff": [0-9.e-]*[0-9]}' build/nearhem.ref &&
		awk -F'"symdiff": ' '{ split($2, v, "}"); d = v[1] + 0
			exit !(d < 1e-6 && d > -1e-6) }' build/nearhem.ref ||
		return 1
	$S/smview bottom build/shortline.st > build/shortline.view &&
		printf '{"step": "%s", "view": [0, 0, -1], "lines": "%s"}\n' \
			build/shortline.step build/shortline.view |
		OCCTWORKER_FD=3 test/freecad.sh test/occtworker.py \
			3> build/shortline.hlr > /dev/null 2>&1
	grep -q '"extra": 0, "lost": 0}' build/shortline.hlr ||
		{ echo "FAIL occt hlr shortline"; return 1; }
}

# A textured face shows the color of the body, 0.85 of it, in a group
# of its own.  tomesh writes the cube of test/cube.sm and, with normals,
# the pit of test/point.part, whose cone ends in a point, as
# test/cube.obj and test/point.obj have them.  The mesh of every test
# part is closed, and its volume and area match those of measure within
# the chord
# (objcheck.py), and so are those of test/smallhole.sm and
# test/wrapround.part at the chords and angles of fuzzer findings 0032
# and 0033.  The coats of test/coat.sm cut the mesh of the box, which
# stays closed; seen from the front, the outside of its front wall shows
# the black band between its red, and the band reaches the back wall
# too unless from and to hold it to the front.
mesh() {
	parts > /dev/null && solids > /dev/null || return 1
	printf '%s\n' 'materials materials' \
		'part t ../build/part_textured.st material red' \
		> build/textured.txt &&
		$S/place build/textured.txt > build/textured.st &&
		$S/tomesh build/textured.st > build/textured.obj &&
		grep -q '^usemtl vdi30$' build/textured.obj &&
		grep -A1 '^usemtl vdi30$' build/textured.obj |
		grep -qx '# texture 30' &&
		grep -q '^# color 0.6800 0.0850 0.0850$' build/textured.obj ||
		{ echo "FAIL mesh texture"; return 1; }
	$S/tomesh build/cube.st > build/cube.obj &&
		$S/numdiff -t v=abs:2e-5 test/cube.obj build/cube.obj &&
		$S/tomesh -n build/part_point.st > build/point.obj &&
		$S/numdiff -t v=abs:2e-5 -t vn=abs:2e-5 test/point.obj \
			build/point.obj || return 1
	$S/tomesh -c 0.2007 -a 4.4786 build/smallhole.st \
		> build/smallhole.chord.obj &&
		python3 -I test/objcheck.py build/smallhole.chord.obj \
			build/smallhole.measure 0.2007 > build/smallhole.mesh ||
		{ echo "FAIL mesh smallhole at its chord"; return 1; }
	$S/tomesh -c 0.2228 -a 13.784 build/part_wrapround.st \
		> build/wrapround.chord.obj &&
		python3 -I test/objcheck.py build/wrapround.chord.obj \
			build/part_wrapround.measure 0.2228 \
			> build/wrapround.mesh ||
		{ echo "FAIL mesh wrapround at its chord"; return 1; }
	$S/place test/coat.txt > build/coatbox.st &&
		$S/tomesh build/coatbox.st > build/coatbox.obj &&
		$S/measure build/coatbox.st > build/coatbox.measure &&
		python3 -I test/objcheck.py build/coatbox.obj \
			build/coatbox.measure 0.02 > build/coatbox.mesh &&
		printf 'mesh box coatbox.obj\nview front\n' \
			> build/coat.scene &&
		echo 'shot build/coat.ppm' |
		$S/view -b -s 80x60 build/coat.scene &&
		test "$(pixel build/coat.ppm 40 30)" = ' 11 11 11' &&
		test "$(pixel build/coat.ppm 40 25)" = ' 177 22 22' &&
		test "$(grep -c '^coat black far -' build/coat.st)" -eq 1 ||
		{ echo "FAIL mesh coats"; return 1; }
	for t in $TESTS; do
		$S/tomesh build/$t.st > build/$t.obj &&
			python3 -I test/objcheck.py build/$t.obj \
				build/$t.measure 0.02 > build/$t.mesh ||
			{ echo "FAIL mesh $t"; return 1; }
	done
	for t in $SOLIDS; do
		m=build/part_$t
		$S/tomesh -c 0.05 -a 15 $m.st > $m.obj &&
			python3 -I test/objcheck.py $m.obj $m.measure 0.05 \
				> $m.mesh ||
			{ echo "FAIL mesh solid $t"; return 1; }
	done
}

# The pixel X, Y of the PPM file $1 of 80 by 60, as ' R G B'.
pixel() {
	od -An -tu1 -j $((13 + 3 * ($3 * 80 + $2))) -N3 "$1" | tr -s ' '
}

# view checks test/view.scene, with the cube of FreeCAD (test/freecad.obj,
# by Mesh.export of test/cube.step), closed and of the volume of the cube
# within its chord of 0.1 mm; runs the commands of a session with no
# display, its picks, its lists, its refusals and its shot as
# test/view.expected has them; and refuses broken scenes, meshes and
# pictures.  Through the clear hole of the picture of test/hole.scene
# (PAM, RGB_ALPHA) a pick finds the cube, and a shot shows it as with the
# picture hidden; elsewhere the picture hides it.  The parts move, and
# undo and redo walk back and forth through what shows and the moves,
# as the picks, lists and shot of test/moves.expected see.  The
# assembly sequence of test/play.scene plays: a mesh not yet in stands
# at the start of its path, one partway along it, one with another
# moves with it, and a picture with its mesh (test/play.expected);
# help prints the help of the key ?; the keys h, o and s hide the part
# under the pointer, show it alone, or turn it see-through, and see and
# solid do so by name.  In the scene of the subassemblies of
# test/nested.txt, the label brackets the unit of a part at each level,
# the keys H, O and S act on that unit, a move moves it, and a name
# hides a unit, as test/units.expected has them.
# A background line sets the color round the parts, FreeCAD's gray by
# default.  The picks of a grid of pixels in three views find the parts
# that a ray against every triangle finds (test/pick.expected), through
# the tree of boxes of each part.  With a display, view draws a shot by
# OpenGL.
viewer() {
	parts > /dev/null || return 1
	printf 'P6\n2 2\n255\n\377\0\0\0\377\0\0\0\377\377\377\377' \
		> build/view.ppm
	{ echo 'o cube'; cat test/freecad.obj; } > build/freecad.obj
	python3 -I test/objcheck.py build/freecad.obj build/cube.measure 0.1 \
		> build/freecad.mesh || return 1
	{
		$S/view -c test/view.scene &&
			printf '%s\n' 'list' 'view top' \
				'pick 35 30' 'pick 46 30' 'pick 35 11' \
				'pick 35 52' 'pick 2 2' 'hide cube' \
				'pick 35 30' 'only fc' 'list' 'all' \
				'view iso' 'zoom 0.5' 'turn 10 5' \
				'shot build/view.shot.ppm' 'bogus' \
				'hide nomatch' 'turn x 1' 'quit' 'list' |
			$S/view -b -s 80x60 test/view.scene 2>&1 &&
			cksum < build/view.shot.ppm
	} > build/view.out && cmp -s build/view.out test/view.expected ||
		{ echo "FAIL view session"; return 1; }
	for bad in 'mesh a' 'mesh a nofile.obj' 'view sideways' 'thing' \
		'mesh a ../test/cube.obj move 1 2' \
		'image a ../test/cube.obj 0' 'mesh a bad.obj' \
		'image a ../test/cube.obj on a 0 0 0 1 0 0 0 1 0' \
		'mesh c ../test/cube.obj
image a ../test/cube.obj on c 0 0 0 1 0 0 0 1 0'; do
		printf '%s\n' 'f 1 2 3' > build/bad.obj
		printf '%s\n' "$bad" > build/bad.scene
		if $S/view -c build/bad.scene > /dev/null 2> build/bad.err ||
			! grep -q '^view: ' build/bad.err ||
			grep -q Sanitizer build/bad.err; then
			echo "FAIL view refuses $bad"
			return 1
		fi
	done
	{
		printf '%s\n' 'help' 'view top' 'play 0' 'fit' \
			'pick 34 30' 'pick 46 30' 'pick 34 6' 'pick 34 54' \
			'move cube 0 30 0' 'pick 34 30' 'list' 'undo' \
			'pick 34 30' 'redo' 'list' 'home' 'pick 34 30' \
			'play 1' 'hide cube' 'only fc' 'pick 34 6' 'undo' \
			'undo' 'pick 34 30' 'redo' 'pick 34 30' \
			'move fc 0 0 10' \
			'redo' 'list' 'reset' 'list' 'undo' 'list' \
			'shot build/moves.shot.ppm' 'reset' 'fit' \
			'key h 35 30' 'key h 2 2' 'key o 35 11' 'list' \
			'key o 2 2' 'list' 'undo' 'undo' 'list' 'reset' \
			'see cube' 'key s 35 11' \
			'list' 'undo' 'list' 'see cube' 'see point' 'see fc' \
			'solid point' 'list' \
			'shot build/see.shot.ppm' |
			$S/view -b -s 80x60 test/view.scene 2>&1 &&
			cksum < build/moves.shot.ppm &&
			cksum < build/see.shot.ppm
	} > build/moves.out &&
		cmp -s build/moves.out test/moves.expected ||
		{ echo "FAIL view moves"; return 1; }
	{
		printf '%s\n' 'view front' 'play 0' 'list' \
			'play 1' 'list' 'play 1.5' 'list' 'play 2' 'list' \
			'play 3' 'play 0' 'undo' 'list' 'redo' 'list' \
			'move cube 0 10 0' 'play 1' 'list' \
			'shot build/play.shot.ppm' 'home' 'list' |
			$S/view -b -s 80x60 test/play.scene 2>&1 &&
			cksum < build/play.shot.ppm
	} > build/play.out && cmp -s build/play.out test/play.expected ||
		{ echo "FAIL view play"; return 1; }
	# select picks a part and center turns the view onto it, and again
	# zooms in two times, as zoom 0.5 does; a bare select picks none,
	# so center does nothing.
	{
		v="$S/view -b -s 80x60 test/view.scene"
		printf '%s\n' 'view top' 'fit' 'select point' 'center' \
			'pick 40 30' 'center' 'shot build/center2.ppm' |
			$v > build/center.out &&
			grep -qx point build/center.out &&
			printf '%s\n' 'view top' 'fit' 'select point' 'center' \
				'zoom 0.5' 'shot build/zoom2.ppm' | $v &&
			cmp -s build/center2.ppm build/zoom2.ppm &&
			printf '%s\n' 'view top' 'fit' 'select' 'center' \
				'shot build/center0.ppm' | $v &&
			printf '%s\n' 'view top' 'fit' 'shot build/fit0.ppm' |
				$v &&
			cmp -s build/center0.ppm build/fit0.ppm &&
			! cmp -s build/center2.ppm build/fit0.ppm
	} > /dev/null 2>&1 ||
		{ echo "FAIL view select and center"; return 1; }
	$S/place test/nested.txt > build/nested.st &&
		$S/place -s nm test/nested.txt > build/nested.scene || return 1
	for f in p1/plate p1/knob/base p1/knob/cap p2/plate p2/knob/base \
		p2/knob/cap; do
		mkdir -p build/nm/$(dirname $f) &&
			$S/tomesh build/plate.st > build/nm/$f.obj || return 1
	done
	printf '%s\n' 'view front' 'fit' 'zoom 0.3' \
		'label 40 39' 'level 2' 'label 40 39' 'label 40 43' 'level 1' \
		'label 40 43' 'key H 40 39' 'list' 'reset' 'level 2' \
		'key O 40 63' 'list' 'key O 1 1' 'list' 'reset' 'level 1' \
		'key S 40 43' 'list' 'reset' 'move p2 0 0 50' 'list' 'undo' \
		'pick 40 43' 'hide p1/knob' 'list' 'level 0' |
		$S/view -b -s 80x80 build/nested.scene > build/units.out 2>&1 &&
		cmp -s build/units.out test/units.expected ||
		{ echo "FAIL view units"; return 1; }
	{
		for v in iso top front; do
			echo "view $v"
			for y in $(seq 1 2 59); do
				for x in $(seq 1 2 79); do
					echo "pick $x $y"
				done
			done
		done
	} | $S/view -b -s 80x60 test/view.scene > build/pick.out &&
		cmp -s build/pick.out test/pick.expected ||
		{ echo "FAIL view picks"; return 1; }
	{
		printf 'P7\nWIDTH 3\nHEIGHT 3\nDEPTH 4\nMAXVAL 255\n'
		printf 'TUPLTYPE RGB_ALPHA\nENDHDR\n'
		w='\377\377\377\377'
		printf "$w$w$w$w"'\0\0\0\0'"$w$w$w$w"
	} > build/hole.pam
	printf '%s\n' 'pick 40 30' 'pick 30 20' \
		'shot build/hole.a.ppm' 'hide window' 'shot build/hole.b.ppm' |
		$S/view -b -s 80x60 test/hole.scene > build/hole.out &&
		printf '%s\n' cube window | cmp -s - build/hole.out &&
		test "$(pixel build/hole.a.ppm 40 30)" = \
			"$(pixel build/hole.b.ppm 40 30)" &&
		test "$(pixel build/hole.a.ppm 30 20)" = ' 255 255 255' &&
		test "$(pixel build/hole.b.ppm 30 20)" != ' 255 255 255' ||
		{ echo "FAIL view hole"; return 1; }
	printf '%s\n' 'mesh cube ../test/cube.obj' 'background 1 0.5 0' \
		> build/back.scene
	echo 'shot build/back.ppm' |
		$S/view -b -s 80x60 build/back.scene &&
		test "$(pixel build/back.ppm 2 2)" = ' 255 128 0' &&
		test "$(pixel build/hole.a.ppm 2 2)" = ' 81 81 81' ||
		{ echo "FAIL view background"; return 1; }
	test -z "$DISPLAY" && return 0
	printf '%s\n' 'shot build/view.gl.ppm' 'quit' |
		ASAN_OPTIONS=detect_leaks=0 timeout 30 \
			$S/view test/view.scene &&
		test "$(head -c 2 build/view.gl.ppm)" = P6 ||
		{ echo "FAIL view by OpenGL"; return 1; }
	# The grain of a texture shows over its color: the textured block
	# of the mesh group drawn by OpenGL differs from it without the
	# texture.
	mesh > /dev/null || return 1
	sed '/^# texture /d' build/textured.obj > build/plain.obj
	for m in textured plain; do
		printf '%s\n' "mesh t $m.obj" 'view left' > build/grain_$m.scene
		printf '%s\n' "shot build/grain_$m.ppm" 'quit' |
			ASAN_OPTIONS=detect_leaks=0 timeout 30 \
				$S/view build/grain_$m.scene || return 1
	done
	! cmp -s build/grain_textured.ppm build/grain_plain.ppm ||
		{ echo "FAIL view grain"; return 1; }
	command -v xprop > /dev/null || return 0
	view_names && view_fit && view_play
}

# Each key moves its part to the end of its motion at once, in the next
# frame, however slow a frame is to draw: with a mesh of 180000
# triangles beside them, the three cubes that three quick minus keys
# take out stand at their starts, 40 up, 0.2 s later; three plus keys
# put them back, and A takes all of them out at once.
view_play() {
	cc -o build/xkey test/xkey.c -lXtst -lXdamage -lXext -lX11 ||
		return 1
	awk 'BEGIN { n = 300
		for (j = 0; j <= n; j++)
			for (i = 0; i <= n; i++)
				printf "v %d %d %g\n", i, j, \
					sin(i / 9) * cos(j / 7)
		for (j = 0; j < n; j++)
			for (i = 0; i < n; i++) {
				a = j * (n + 1) + i + 1
				printf "f %d %d %d\n", a, a + 1, a + n + 2
				printf "f %d %d %d\n", a, a + n + 2, a + n + 1
			}
	}' > build/play_big.obj
	printf '%s\n' 'mesh big play_big.obj' \
		'mesh c1 ../test/cube.obj path move 0 0 -40' \
		'mesh c2 ../test/cube.obj move 60 0 0 path move 0 0 -40' \
		'mesh c3 ../test/cube.obj move 120 0 0 path move 0 0 -40' \
		> build/play_key.scene
	rm -f build/play.fifo
	mkfifo build/play.fifo || return 1
	ASAN_OPTIONS=detect_leaks=0 timeout 60 $S/view build/play_key.scene \
		< build/play.fifo > build/play_key.out &
	runner=$!
	exec 3> build/play.fifo
	k=0
	while test $k -lt 100 && ! build/xkey view space 2> /dev/null; do
		sleep 0.1
		k=$((k + 1))
	done
	echo list >&3
	sleep 1
	build/xkey view minus minus minus
	sleep 0.2
	echo list >&3
	build/xkey view plus plus plus
	sleep 0.2
	echo list >&3
	build/xkey view A
	sleep 0.2
	printf '%s\n' list quit >&3
	exec 3>&-
	wait $runner
	rm -f build/play.fifo
	awk '$1 ~ /^c[123]$/ { z[$1, n[$1]++] = $NF }
		END { ok = 1
			for (c = 1; c <= 3; c++)
				for (k = 1; k <= 3; k++) {
					d = z["c" c, k] - z["c" c, 0]
					w = k == 2 ? 0 : 40
					ok = ok && n["c" c] == 4 &&
						d > w - 0.01 && d < w + 0.01
				}
			exit !ok }' build/play_key.out ||
		{ echo "FAIL view steps keys at once"; return 1; }
}

# view opens fitted to all at the size the window manager gives its
# window: resized from outside before a key, a button or a command
# sets the view, it draws what fit draws.
view_fit() {
	make -s build/xresize || return 1
	rm -f build/fit.fifo
	mkfifo build/fit.fifo || return 1
	ASAN_OPTIONS=detect_leaks=0 timeout 30 $S/view test/view.scene \
		< build/fit.fifo > /dev/null &
	runner=$!
	exec 3> build/fit.fifo
	k=0
	while test $k -lt 100 && ! build/xresize view 200 50; do
		sleep 0.1
		k=$((k + 1))
	done
	sleep 1
	printf '%s\n' 'shot build/fit1.ppm' 'fit' 'shot build/fit2.ppm' \
		'quit' >&3
	exec 3>&-
	wait $runner
	rm -f build/fit.fifo
	test -s build/fit1.ppm && cmp -s build/fit1.ppm build/fit2.ppm ||
		{ echo "FAIL view opens fitted"; return 1; }
}

# The window of view has the class and the process by which window
# placers find it: WM_CLASS and _NET_WM_PID, the pid of view itself,
# under the job that started it.
view_names() {
	rm -f build/view.fifo
	mkfifo build/view.fifo || return 1
	ASAN_OPTIONS=detect_leaks=0 timeout 30 $S/view test/view.scene \
		< build/view.fifo > /dev/null &
	runner=$!
	exec 3> build/view.fifo
	props=
	k=0
	while test $k -lt 100; do
		props=$(xprop -name 'view view' WM_CLASS _NET_WM_PID \
			2> /dev/null) && case $props in
			*_NET_WM_PID*=*) break ;;
			esac
		sleep 0.1
		k=$((k + 1))
	done
	wpid=$(printf '%s\n' "$props" | awk '/_NET_WM_PID/ { print $3 }')
	# view runs under timeout, by its runner (build/budget): the job
	# is an ancestor of view, at most two up.
	parent=$(ps -o ppid= -p "${wpid:-0}" 2> /dev/null | tr -d ' ')
	up=$(ps -o ppid= -p "${parent:-0}" 2> /dev/null | tr -d ' ')
	test "$up" = "$runner" && parent=$up
	echo quit >&3
	exec 3>&-
	wait $runner
	rm -f build/view.fifo
	printf '%s\n' "$props" |
		grep -qx 'WM_CLASS(STRING) = "view", "View"' &&
		test "$parent" = "$runner" ||
		{ echo "FAIL view window names: $props"; return 1; }
}

# The programs of schematics and boards (eda.mk), by their own tests:
# test/schtest.sh, test/pcbtest.sh and test/viewtest.sh, on build/eda.
# Each case of them runs through build/budget with its own file of
# budgets, test/SCRIPT.budget; sh -a hands it the name of the case.
eda() {
	make -s -f eda.mk eda san || return 1
	for t in schtest pcbtest viewtest; do
		RUN="$PWD/build/budget $PWD/test/$t.budget" \
			sh -a test/$t.sh build/eda || return 1
	done
}

# The lines of the source of each program, and the bytes of it as built,
# stripped, held to their budgets, those of schematics and boards too;
# the library of the programs counts as one, lib.
sizes() {
	for p in $PROGS smdraw lib; do
		case $p in
		lib) lines=$(for l in $(printf 'l:\n\t@echo $(LIB)\n' |
			make -s -f Makefile -f - l); do
				cat src/$l.c src/$l.h; done | wc -l)
			bytes=0 ;;
		smdraw) lines=$(wc -l < src/smdraw.sh)
			bytes=$(wc -c < build/smdraw) ;;
		*) lines=$(wc -l < src/$p.c)
			bytes=$(wc -c < build/$p) ;;
		esac
		build/budget -v test/size.budget "$p" lines "$lines" \
			bytes "$bytes" || return 1
	done
	for p in $(printf 'e:\n\t@echo $(EDA)\n' |
		make -s -f eda.mk -f - e); do
		build/budget -v test/size.budget "$p" \
			lines "$(wc -l < src/$p.c)" \
			bytes "$(wc -c < build/eda/$p)" || return 1
	done
}

# Writes the runners: for each program of R, a script of its name in S
# that runs it by build/budget, its case named by the group under way,
# the program and its arguments.
runners() {
	root=$(pwd)
	real=$(cd "$R" && pwd)
	mkdir -p "$S"
	for p in $PROGS test_arith smdraw; do
		printf '%s\n' '#!/bin/sh' "name=\"\$group $p \$*\"" \
			'export name' \
			"exec \"$root/build/budget\" \\" \
			"  \"$root/test/run.budget\" \"$real/$p\" \"\$@\"" \
			> "$S/$p"
		chmod +x "$S/$p"
	done
}

all_groups="arith parts solids fillet place bad tools docs args draw mesh \
viewer occt eda sizes"
test -n "$PLICAD_BIN" || build || { echo "FAIL build"; exit 1; }
runners || { echo "FAIL runners"; exit 1; }
groups=${*:-$all_groups}
fail=0
for g in $groups; do
	case " $all_groups " in
	*" $g "*) ;;
	*)
		echo "unknown group $g"
		exit 2
		;;
	esac
	group=$g
	export group
	if $g > "build/test.$g.log" 2>&1; then
		echo "ok $g"
	else
		echo "FAIL $g"
		cat "build/test.$g.log"
		fail=1
	fi
done
exit $fail
