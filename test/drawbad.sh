#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# drawbad.sh -- Check that the drawing programs refuse bad input
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/drawbad.sh DIR
# The programs come from DIR, and the folds of the test parts from
# build.  Each case must fail with a message that holds the words given,
# and without a report of a sanitizer.

S=$1
B=build/drawbad
mkdir -p $B

# draw part.draw part.st: the drawing of the part, by the pipe of the
# programs.
draw() {
	$S/smsheet -v "$1" | $S/smview "$2" | $S/smdim | $S/smsheet "$1" -
}

# sv RECORD part.st: the view of the part that the view record RECORD,
# VIEW -h|- SCALE DEPTH|-, asks smview for.
sv() {
	printf 'view %s\n' "$1" | $S/smview "$2"
}

# refuse WORDS COMMAND...: the command, with $B/in as its input, fails
# and says WORDS.
refuse() {
	words=$1
	shift
	if "$@" < $B/in > $B/out 2> $B/err || ! grep -q "$words" $B/err ||
		grep -q Sanitizer $B/err; then
		echo "FAIL: $* does not say: $words"
		cat $B/err
		exit 1
	fi
}

echo 'view above - 1 -' > $B/in
refuse 'stdin:1: above: the views are' $S/smview build/cube.st
echo 'view flat - 1 -' > $B/in
refuse 'stdin:1: the flat view is of one folded part' $S/smview build/pair.st
: > $B/in
refuse 'stdin: no view' $S/smview build/cube.st
refuse 'usage' $S/smview top build/cube.st
sv 'top - 1 -' build/pair.st | grep -q '^line e v ' || exit 1
printf '%s\n' 'sheet 1 1 0.44' 'plate p 0 0 60 40 at 0 0 0 axes +x +y' \
	'flange f p +x 45 30' 'round 5 f' > $B/oblique.sm
$S/smfold $B/oblique.sm > $B/oblique.st || exit 1
echo 'view top - 1 -' > $B/in
refuse 'stdin:1: view top: the circle about (.*) is seen at an angle' \
	$S/smview $B/oblique.st
sv 'front - 1 -' $B/oblique.st > /dev/null || exit 1

echo 'hello there' > $B/in
refuse 'stdin:1: unknown record hello' $S/smdim
echo 'view top cube' > $B/in
refuse 'stdin:1: view top: no box' $S/smdim
printf '%s\n' 'view top cube' 'line e v 0 0 x 1' > $B/in
refuse 'stdin:2: x is not a number' $S/smdim
printf '%s\n' 'view top cube' 'hole 1 2' > $B/in
refuse 'stdin:2: hole: too few fields' $S/smdim
printf '%s\n' 'view top cube' 'poly e v 3 0 0 1 1' > $B/in
refuse 'stdin:2: a poly has N of 2 or more, and N points' $S/smdim
printf '%s\n' 'view top cube' 'fill hatch 45' > $B/in
refuse 'stdin:2: expected fill \[cut I\]' $S/smdim
sv 'top - 1 -' build/plate.st > $B/plate.view
{ cat $B/plate.view; echo 'lin x 0 9.999'; } > $B/in
refuse 'stdin:[0-9]*: lin x: no edge or hole at 9.999' $S/smdim
{ cat $B/plate.view; echo 'leader 1 1'; } > $B/in
refuse 'stdin:[0-9]*: a leader without its text' $S/smdim
sed '1s/ 1$/ 0/' $B/plate.view > $B/in
refuse 'stdin:1: a scale is above 0' $S/smdim
refuse 'usage' $S/smdim -s 1

$S/smdim < $B/plate.view > $B/plate.dim
head -5 test/cube.draw > $B/bare
{ cat $B/bare; echo 'view top scale 1'; } > $B/nosize.draw
refuse 'nosize.draw: no template record; name the template of the paper' \
	$S/smsheet $B/nosize.draw $B/plate.dim
refuse 'nosize.draw: no template record' draw $B/nosize.draw build/plate.st
cp lib/templates/plain_b.ps lib/templates/plain_c.ps $B/
{ cat $B/bare; echo 'template plain_c.ps'; } > $B/head
{ cat $B/head; echo 'view top scale 1'; echo 'color red'; } > $B/a.draw
refuse 'a.draw:8: unknown record color' $S/smsheet $B/a.draw $B/plate.dim
cat $B/head > $B/b.draw
refuse 'b.draw: no view' $S/smsheet $B/b.draw $B/plate.dim
{ cat $B/head; echo 'view front scale 1'; } > $B/c.draw
refuse 'the view top is not the next' $S/smsheet $B/c.draw $B/plate.dim
{ cat $B/head; echo 'view top scale 1'; echo 'view front scale 1'; } > $B/d.draw
refuse 'no view front' $S/smsheet $B/d.draw $B/plate.dim
{ cat $B/head; echo 'view top scale 1'; echo 'note RADIUS %%r'; } > $B/e.draw
refuse 'a bend radius or a thickness that the part has not' \
	$S/smsheet $B/e.draw $B/plate.dim
{ cat $B/bare; echo 'template plain_b.ps'; echo 'view top scale 5'; } \
	> $B/f.draw
sv 'top - 5 -' build/plate.st | $S/smdim > $B/plate5.dim
refuse 'f.draw:7: sheet 1: view top at scale 5 takes .* more than the' \
	$S/smsheet $B/f.draw $B/plate5.dim
# A drawing file is ASCII: a byte of Latin-1 and a character of UTF-8
# are refused with their line (fuzzer finding).
{ cat $B/head; echo 'view top scale 1'; printf 'note CAF\351\n'; } > $B/l1.draw
refuse 'l1.draw:8: the byte 0xe9 is no printable ASCII' \
	draw $B/l1.draw build/plate.st
{ cat $B/head; echo 'view top scale 1'; printf 'note 90\302\260 BEND\n'; } \
	> $B/u8.draw
refuse 'u8.draw:8: the byte 0xc2 is no printable ASCII' \
	draw $B/u8.draw build/plate.st
{ cat $B/head; echo 'view top scale 1 title'; } > $B/t0.draw
refuse 't0.draw:7: view top: title without its text' \
	$S/smsheet $B/t0.draw $B/plate.dim
# Each view gives its scale; there is no scale of the whole drawing.
{ cat $B/bare; echo 'template plain_b.ps'; echo 'view top'; } > $B/f2.draw
refuse 'f2.draw:7: view top: no scale' draw $B/f2.draw build/plate.st
{ cat $B/bare; echo 'template plain_b.ps'; echo 'scale 1'
	echo 'view top scale 1'; } > $B/f3.draw
refuse 'f3.draw:7: unknown record scale' draw $B/f3.draw build/plate.st
{ cat $B/head; echo 'view top scale 1 sheet 0'; } > $B/g.draw
refuse 'g.draw:7: a sheet is above 0' \
	$S/smsheet $B/g.draw $B/plate.dim
{ cat $B/head; echo 'boxed MARK WHERE SHOWN.'; echo 'view top scale 1'; } \
	> $B/h.draw
refuse 'h.draw: note 1 has a box, but no view has a flag for it' \
	$S/smsheet $B/h.draw $B/plate.dim
{ cat $B/plate.view; echo 'flag 1 1 2'; } | $S/smdim > $B/flag.dim
{ cat $B/head; echo 'note NO BOX.'; echo 'view top scale 1'; } > $B/i.draw
refuse 'i.draw: view top: the flag 2 names no note with a box' \
	$S/smsheet $B/i.draw $B/flag.dim
refuse 'usage' $S/smsheet $B/a.draw
{ cat $B/head; echo 'view above'; } > $B/j.draw
refuse 'j.draw:7: the views are front, back' $S/smsheet $B/j.draw $B/plate.dim
{ cat $B/head; echo 'view top scale 1'; echo ' lin x 0 9.999in'; } > $B/k.draw
refuse 'k.draw:8: lin x: no edge or hole at 9.999' draw $B/k.draw \
	build/plate.st
{ echo 'view top plate 1'; echo 'line e v 1 1 1 1'; echo 'box 0 0 2 2'; } \
	> $B/zero.dim
{ cat $B/head; echo 'view top scale 1'; } > $B/zero.draw
refuse 'zero.dim:2: a line of no length' $S/smsheet $B/zero.draw $B/zero.dim

# The template: missing, without its paper or its room, with a room off
# the paper; and the fields it lists.
{ cat $B/bare; echo 'template none.ps'; echo 'view top scale 1'; } > $B/t1.draw
refuse 'cannot open .*none.ps' $S/smsheet $B/t1.draw $B/plate.dim
grep -v PliCadPaper lib/templates/plain_c.ps > $B/nopaper.ps
{ cat $B/bare; echo 'template nopaper.ps'; echo 'view top scale 1'; } \
	> $B/t2.draw
refuse 't2.draw:6: the template has no %%PliCadPaper' \
	$S/smsheet $B/t2.draw $B/plate.dim
grep -v PliCadRoom lib/templates/plain_c.ps > $B/noroom.ps
{ cat $B/bare; echo 'template noroom.ps'; echo 'view top scale 1'; } \
	> $B/t3.draw
refuse 't3.draw:6: the template has no %%PliCadRoom' \
	$S/smsheet $B/t3.draw $B/plate.dim
T=lib/templates
sed 's/^%%PliCadRoom: .*/%%PliCadRoom: 0.9 0.9 30 16/' $T/plain_c.ps \
	> $B/offroom.ps
{ cat $B/bare; echo 'template offroom.ps'; echo 'view top scale 1'; } \
	> $B/t4.draw
refuse '%%PliCadRoom 0.9 0.9 30 16 lies off the paper of 22 x 17 in' \
	$S/smsheet $B/t4.draw $B/plate.dim
{ cat $B/head; echo 'field Titel X'; echo 'view top scale 1'; } > $B/t5.draw
refuse 't5.draw:7: the template has no field Titel' \
	$S/smsheet $B/t5.draw $B/plate.dim
{ cat $B/head; echo 'field Sheet 9'; echo 'view top scale 1'; } > $B/t6.draw
refuse 't6.draw:7: smsheet gives the field Sheet itself' \
	$S/smsheet $B/t6.draw $B/plate.dim
{ cat $B/head; echo 'field 2x X'; echo 'view top scale 1'; } > $B/t7.draw
refuse 't7.draw:7: 2x is no name for a field' \
	$S/smsheet $B/t7.draw $B/plate.dim
