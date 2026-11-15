#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# drawfix.sh -- Check the drawing programs against the faults once found
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/drawfix.sh DIR
# The programs come from DIR, and the folds of the test parts from
# build.  It prints the first check that fails, and exits with 1.

S=$1
B=build/drawfix
mkdir -p $B
cp test/plain_b.ps test/plain_c.ps $B/

fail() {
	echo "FAIL: $*"
	exit 1
}

# No view of a test part, with its hidden lines, has a line or an arc
# shorter than a micrometre: such a piece printed with no length, and
# Ghostscript stopped on its dashes (channel, nearhem).  Nor does one
# that rounding to six places makes so: the edge of the cut of endon
# seen nearly end on (fuzzer finding 0031).
for f in build/*.fold; do
	case $f in */bad_* | */pair.fold | */order.fold | */colour.fold | \
		*/coincident.fold) continue ;; esac
	grep -q '^map' "$f" || continue
	for v in top bottom front back left right flat; do
		$S/smview -h $v "$f" > $B/view 2> /dev/null || continue
		awk '$1 == "line" && ($4 - $6) ^ 2 + ($5 - $7) ^ 2 < 1e-6 ||
			$1 == "arc" && $6 * ($8 - $7) * 0.01745 < 1e-3 {
			bad = 1 } END { exit bad }' $B/view ||
			fail "$f $v: a line or an arc of no length"
	done
done

# A hatch over the whole view moves no callout: a text may lie on it,
# and its lines stop round the text.
$S/smview top build/bracket.fold > $B/bracket.view
$S/smdim < $B/bracket.view | grep '^text' > $B/plain
{ cat $B/bracket.view; echo 'hatch 0 0 4 2.2'; } | $S/smdim > $B/hatch.dim
grep '^text' $B/hatch.dim | cmp -s - $B/plain ||
	fail "a hatch moves the callouts of the bracket"
grep -q '^clear' $B/hatch.dim || fail "no clear record for a hatch"

# The datum ordinate on the far side starts at the datum edge, which
# stops at x = 60 mm, not at the far side of the view.
$S/smview top build/notch.fold | $S/smdim > $B/notch.dim
awk '$1 == "thin" && $3 == 0 && $5 == 0 && $2 < 2.4 && $4 > 4 { ok = 1 }
	END { exit !ok }' $B/notch.dim ||
	fail "notch: the datum ordinate does not start at the datum edge"

# A leader whose point lies where a hole callout would stand: the
# callout keeps off the point, and the leader finds its room.
$S/smview top build/plate.fold > $B/plate.view
at=$($S/smdim < $B/plate.view | awk '$1 == "text" && $7 ~ /%%c/ {
	print $2 + 0.1, $3; exit }')
{ cat $B/plate.view; echo "leader $at MASK HERE"; } |
	$S/smdim > /dev/null || fail "plate: no room for a leader at $at"

# A drawing file without a newline at its end, whose last line is its
# one view, is drawn (fuzzer finding); one with a scale and no view is
# refused with a message (fuzzer finding).
printf 'template plain_b.ps\nview front scale 1' > $B/nonl.draw
PLICAD=$S build/smdraw $B/nonl.draw build/cube.fold > $B/nonl.ps &&
	grep -q '^%%Page: 1 1' $B/nonl.ps ||
	fail "a drawing file without a final newline"
printf 'scale 1\n' > $B/noview.draw
if PLICAD=$S build/smdraw $B/noview.draw build/cube.fold \
	> /dev/null 2> $B/err ||
	! grep -q 'noview.draw: no view' $B/err; then
	fail "a drawing file with no view: $(cat $B/err)"
fi

# A note with a word longer than a line of the notes is broken inside
# the word; once the search for its end ran past the start of the text
# for ever (fuzzer finding).
{ cat test/cube.draw; printf 'note A %0200d\n' 9; } > $B/word.draw
PLICAD=$S timeout 60 build/smdraw $B/word.draw build/cube.fold > $B/word.ps &&
	grep -q '^%%Page: 1 1' $B/word.ps || fail "a note with a long word"
# So too a long word after words that fill part of a line (fuzzer
# finding 0022, a hang of smdraw).
{ cat test/cube.draw
	printf 'note REMOVE ALL BURRS AND SHARP EDGES. %0400d\n' 9; } \
	> $B/word2.draw
PLICAD=$S timeout 60 build/smdraw $B/word2.draw build/cube.fold \
	> $B/word2.ps && grep -q '^%%Page: 1 1' $B/word2.ps ||
	fail "a long word after words in a note"

# A field of the drawing file reaches the template as its text, with
# parentheses and backslashes as they are.
{ cat test/plain_c.ps; echo '(<) print Title print (>) print'; } > $B/echo.ps
{ head -4 test/cube.draw; echo 'template echo.ps'; echo 'view top'
	printf '%s\n' 'field Title A (B) \C \\ )( ('; } > $B/echo.draw
$S/smview top build/plate.fold | $S/smdim > $B/echo.dim
$S/smsheet $B/echo.draw $B/echo.dim > $B/echo.ps.out
gs -q -dNOPAUSE -dBATCH -dSAFER -sDEVICE=nullpage $B/echo.ps.out \
	2> /dev/null | grep -F -q '<A (B) \C \\ )( (>' ||
	fail "a field with parentheses and backslashes"

# The notes take only the height and the width their lines need, and
# views stand MARGIN off them and off the title block: the top and the
# front view of a wide plate fit a B sheet at scale 1 beside the notes
# and under the revision table (as a wide guard and a wide inlet well need).
cat > $B/wide.draw << 'D'
template plain_b.ps
revision A 11/13/26 JK FIRST
material POLYPROPYLENE SHEET, %%t THICK.
finish NONE.
boxed PART MARKING: VENDOR ID, MONTH/YEAR OF MANUFACTURE, PART NUMBER,
+ AND CURRENT REVISION LEVEL USING METAL AND/OR RUBBER STAMP,
+ APPROXIMATELY WHERE SHOWN.
note REMOVE ALL BURRS AND SHARP EDGES.
view top
 flag 5 1.5 3
view front
D
PLICAD=$S build/smdraw $B/wide.draw build/wide.fold > $B/wide.ps &&
	grep -q '^/Scale (1.00) def' $B/wide.ps ||
	fail "wide: the views do not fit a B sheet at scale 1"

# A note wraps to the width of the region of the notes of the template.
sed 's/^%%PliCadNotes: 0.9 0.9 6.4/%%PliCadNotes: 0.9 0.9 3.4/' \
	test/plain_b.ps > $B/narrow.ps
sed 's/plain_b/narrow/' $B/wide.draw > $B/narrow.draw
PLICAD=$S build/smdraw $B/narrow.draw build/wide.fold > $B/narrow.ps.out ||
	fail "narrow notes: no drawing"
awk '/^\[\(/ && $NF == "TX" && $(NF - 5) + 0 == 1.2 {
	s = $0; sub(/^\[\(/, "", s); sub(/\)\].*/, "", s)
	if (length(s) > 30) bad = 1; n++ } END { exit bad || n < 8 }' \
	$B/narrow.ps.out || fail "narrow notes: a line wider than the region"

# A leader to a point off the part: the box of the view covers the
# leader and its dot, so that smsheet keeps them on the paper and off
# the other views (fuzzer finding).
{ $S/smview top build/plate.fold; echo 'leader 2 3.5 OFF'; } |
	$S/smdim > $B/off.dim
awk '$1 == "box" && $5 >= 3.52 { ok = 1 } END { exit !ok }' $B/off.dim ||
	fail "a leader off the part lies outside the box of its view"

# A view whose own scale differs from that of the sheet, with a title,
# has room under it for both lines: at the foot of a tight room the
# scale stays on the paper (fuzzer finding).
printf '%s\n' '%%PliCadPaper: 30 10.45' '%%PliCadRoom: 0.5 0.5 29.5 9.95' \
	> $B/tight.ps
printf '%s\n' 'template tight.ps' 'view front scale 1' \
	'view bottom scale 2 title BOTTOM' > $B/tight.draw
PLICAD=$S build/smdraw $B/tight.draw build/bracket.fold > $B/tight.out &&
	awk '/translate$/ { ty = $3 } /SCALE/ && / TX$/ {
		if (ty + $(NF - 4) - 0.06 < 0.5) bad = 1; n++ }
		END { exit bad || !n }' $B/tight.out ||
	fail "the scale of a view with a title lies under the room"

# A view as tall as the room, to the hundredth, fits it; one a little
# taller is refused with an excess of a hundredth, never of none
# (fuzzer finding).
printf '%s\n' '%%PliCadPaper: 30 3' '%%PliCadRoom: 0.5 0.5 29.5 1' \
	> $B/low.ps
printf '%s\n' 'template low.ps' 'view top' > $B/low.draw
$S/smview top build/plate.fold | $S/smdim > $B/low.dim
h=$($S/smsheet $B/low.draw $B/low.dim 2>&1 > /dev/null |
	sed -n 's/.* takes [0-9.]* x \([0-9.]*\) in .*/\1/p')
: > $B/low.res
for d in 0.005 -0.006; do
	y=$(echo "0.5 + $h + $d" | bc)
	printf '%s\n' '%%PliCadPaper: 30 5' "%%PliCadRoom: 0.5 0.5 29.5 $y" \
		> $B/low.ps
	$S/smsheet $B/low.draw $B/low.dim > /dev/null 2> $B/low.err
	echo "$d $?" >> $B/low.res
done
grep -q '^0.005 0$' $B/low.res && grep -q '^-0.006 1$' $B/low.res &&
	grep -q ' 0.00 x 0.01 in more than the room' $B/low.err ||
	fail "a view as tall as the room: $(cat $B/low.res $B/low.err)"

# Two rows of ordinates do not both spread into one corner of a view:
# the texts of the row over the flat view of rowcorner at scale 1/4
# stand past the column right of it, whose texts stay under that row
# (fuzzer finding).
$S/smview flat build/rowcorner.fold | $S/smdim -s 0.25 > $B/rowcorner.dim
awk '$1 == "text" && $4 == 90 && $5 == "l" { if (!rx || $2 > rx) rx = $2
		if (!ry || $3 < ry) ry = $3 }
	$1 == "text" && $4 == 0 && $5 == "l" && $6 == 0.120 &&
		$7 ~ /^[0-9]*\.[0-9][0-9][0-9]$/ { if (!cx || $2 < cx) cx = $2
		if ($3 > cy) cy = $3 }
	END { exit !(rx && cx) || (rx + 0.1 > cx && cy + 0.1 > ry) }' \
	$B/rowcorner.dim ||
	fail "rowcorner: the ordinates of two rows meet in a corner"

# A drawing file with CR LF line ends is drawn; once the CR stayed on
# the name of the view, and smview refused it with no place in the file
# (fuzzer finding).  Each refusal of a program in the pipe names the
# line of the view in the drawing file.
printf 'template plain_b.ps\r\nview top\r\n' > $B/crlf.draw
PLICAD=$S build/smdraw $B/crlf.draw build/cube.fold > $B/crlf.ps &&
	grep -q '^%%Page: 1 1' $B/crlf.ps || fail "a drawing file with CR LF"
printf 'template plain_b.ps\nview front\n' > $B/angle.draw
if PLICAD=$S build/smdraw $B/angle.draw build/rowcorner.fold \
	> /dev/null 2> $B/err ||
	! grep -q '^build/drawfix/angle.draw:2: view front: ' $B/err; then
	fail "a refusal of smview without its line: $(cat $B/err)"
fi

# The layout check of the fuzzer takes an arc for the part of its
# circle it sweeps: a quarter arc low left of its centre reaches no
# higher than the centre (fuzzer finding 0028, a view taken as off the
# room of a D sheet).
python3 -I -c 'import sys; sys.path.insert(0, "test"); import fuzz
b = fuzz.arc_box(0, 0, 1, -180, -90)
sys.exit(not (abs(b[2]) < 1e-9 and abs(b[3]) < 1e-9 and b[0] == -1))' ||
	fail "the fuzzer takes a quarter arc for its whole circle"

# No fixed buffer bounds a text of the drawing file: a field of 400
# characters, a note of 80 words and a leader of 300 characters are
# drawn; a leader of 1000 characters is refused as too wide for the
# paper, with the line of its view; a view name of 400 characters is
# refused with its line (fuzzer finding).
cp test/plain_d.ps $B/
{ echo 'template plain_d.ps'
	printf 'field Title %0400d\n' 0
	printf 'note'; printf ' WORD%.0s' $(seq 80); echo
	echo 'view top'
	printf ' leader 1 0.5 %0300d\n' 0; } > $B/long.draw
PLICAD=$S build/smdraw $B/long.draw build/plate.fold > $B/long.ps &&
	grep -q '^%%Page: 1 1' $B/long.ps || fail "long texts"
{ echo 'template plain_d.ps'; echo 'view top'
	printf ' leader 1 0.5 %01000d\n' 0; } > $B/long2.draw
if PLICAD=$S build/smdraw $B/long2.draw build/plate.fold \
	> /dev/null 2> $B/err ||
	! grep -q '^build/drawfix/long2.draw:2: sheet 1: view top at' $B/err
then
	fail "a leader wider than the paper: $(cat $B/err)"
fi
printf 'template plain_d.ps\nview %0400d\n' 9 > $B/long3.draw
if PLICAD=$S build/smdraw $B/long3.draw build/plate.fold \
	> /dev/null 2> $B/err ||
	! grep -q '^build/drawfix/long3.draw:2: the views are' $B/err; then
	fail "a long view name: $(cat $B/err)"
fi

# A view whose title is wider than the view takes the width of its
# title: the right view keeps off the title of the front view, which
# once ran under it (fuzzer finding 0030).
printf 'template plain_b.ps\nview front title %s\nview right\n' \
	"$(printf 'WIDE%.0s ' $(seq 16))" > $B/wt.draw
PLICAD=$S build/smdraw $B/wt.draw build/cube.fold > $B/wt.ps &&
	python3 -I -c 'import sys; sys.path.insert(0, "test"); import fuzz
sys.exit(fuzz.sheet_properties(open(sys.argv[1], encoding="latin-1").read(),
                               open("test/plain_b.ps").read()) is not None)' \
		$B/wt.ps || fail "the title of a view runs under the next view"
