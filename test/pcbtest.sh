#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# pcbtest.sh -- Run the tests of the layout tools
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/pcbtest.sh DIR
# The programs are those in DIR.  The board test/pcb_filter.pcb must be
# placed and routed the same, byte for byte, twice; given its own lock it
# must give it back unchanged; it must check clean; a part's statement
# edited must change that part and its nets in the lock and nothing
# else; the lock must say in words what it was made from, and pcbcheck
# tell from them a statement changed, a pad moved and a pad's net
# changed; a via tangent to its pad, one inside it, a mask sliver, silk
# on a pad, a small part's pads heated unevenly, two courtyards on one
# side overlapping and a part over the keep-out of a hole must be told,
# and the
# fanout must neck a small part's plane pads, on the plane's own layer
# too; a track must end at a pad's center, pads in line be joined by
# one straight track, and a connection take its own width, an edit of
# it making that connection alone anew, copper narrower than its width
# told; its Gerbers must turn into vectors; pcbvec must name its
# parts; and every layer file, of pcbvec or of gbrvec from the Gerbers,
# must name its layer.  It prints ok or FAIL for each and exits non-zero
# if any fails.  A group laid out alone must have its parts alone, its
# root at the origin; as a block of the board, its parts and copper
# must stand as its lock has them, moved whole.

cd "$(dirname "$0")/.." || exit 1
P=${1:?usage: test/pcbtest.sh DIR}
B=test/pcb_filter.pcb
tmp=$(mktemp -d) || exit 1
trap 'rm -rf "$tmp"' EXIT
fail=0

# Each case runs through $RUN, if set: a runner that measures it.
check() {
	if ${RUN:+$RUN} "$@"; then
		echo "ok $name"
	else
		echo "FAIL $name"
		fail=1
	fi
}

build() {
	"$P/pcbplace" "$1" < "$2" | "$P/pcbroute" "$1"
}

name="pcb deterministic"
build "$B" /dev/null > "$tmp/a" && build "$B" /dev/null > "$tmp/b"
check cmp -s "$tmp/a" "$tmp/b"
name="pcb idempotent"
build "$B" "$tmp/a" > "$tmp/c"
check cmp -s "$tmp/a" "$tmp/c"
name="pcb check"
check sh -c '"$1/pcbcheck" "$2" < "$3" > "$4/out" && ! test -s "$4/out"' \
	- "$P" "$B" "$tmp/a" "$tmp"
name="pcb small edit"
mkdir "$tmp/e"
cp -r test/footprints test/sch_filter.expected "$tmp/e"
sed 's/^place filter.lpf.c2 right of filter.lpf.c$/place filter.lpf.c2 left of filter.lpf.c/' "$B" \
	> "$tmp/e/pcb_filter.pcb"
build "$tmp/e/pcb_filter.pcb" "$tmp/a" > "$tmp/d" 2> /dev/null
check sh -c 'diff "$1" "$2" | grep "^[<>] part" | grep -v "^[<>] part filter.lpf.c2 " |
	grep -q . && exit 1; exit 0' - "$tmp/a" "$tmp/d"
# The lock in plain words: a part's designator and the nets of its pads,
# a net's pads where they stood; and pcbcheck tells, from them, what is
# stale and why.
name="the lock tells in words"
check sh -c 'grep -q "^part filter.buf.amp U[0-9]* [-0-9.]* [-0-9.]* [0-9]* top nets 1 - 2 " "$1" &&
	grep -q "^net OUT by$" "$1" && grep -q "^link OUT .* 0.4000$" "$1" &&
	grep -q "^pad OUT filter.lpf.c.1 C[0-9]* [-0-9.]* [-0-9.]* [0-9.]* [0-9.]* top$" "$1"' \
	- "$tmp/a"
name="pcbcheck tells a statement since changed"
check sh -c '! "$1/pcbcheck" "$2/e/pcb_filter.pcb" < "$2/a" > "$2/out" &&
	grep -q "filter.lpf.c2 (C[0-9]*) is placed by a statement since changed" "$2/out"' \
	- "$P" "$tmp"
name="pcbcheck tells a pad moved under its copper"
awk '$1 == "part" && $2 == "filter.lpf.r" { $5 = $5 + 0.5 } { print }' \
	"$tmp/a" > "$tmp/moved"
check sh -c '! "$1/pcbcheck" "$2" < "$3/moved" > "$3/out" &&
	grep -q "net OUT: its copper was made for filter.lpf.r.2 at .*, now at " "$3/out"' \
	- "$P" "$B" "$tmp"
name="pcbcheck tells a pad on another net"
sed 's/^\(net OUT .*\) \(R[0-9]*\.2\)\(.*\)$/\1\3/' test/sch_filter.expected \
	> "$tmp/e/sch_filter.expected"
cp "$B" "$tmp/e/pcb_filter.pcb"
check sh -c '! "$1/pcbcheck" "$2/e/pcb_filter.pcb" < "$2/a" > "$2/out" &&
	grep -q "was placed with its pad 2 on OUT, now on -" "$2/out"' \
	- "$P" "$tmp"
# Vias off pads: the board, laid out, keeps the via gap and the mask web
# (pcb check above); a via tangent to its pad, one inside it, a mask
# sliver, silk on a pad and a pad heated unevenly must each be told.
via_to() {
	awk -v how="$1" '
	$1 == "pad" && $2 == "GND" && !px { px = $5; py = $6; pw = $7 }
	$1 == "via" && $2 == "GND" && !done {
		$3 = how == "in" ? px : px + pw / 2 + $5 / 2; $4 = py; done = 1
	} { print }' "$tmp/a"
}
name="pcbcheck tells a via tangent to its pad"
via_to tangent > "$tmp/tangent"
check sh -c '! "$1/pcbcheck" "$2" < "$3/tangent" > "$3/out" &&
	grep -q "a via of GND is 0.000 mm from pad .* under the via gap 0.300" "$3/out"' \
	- "$P" "$B" "$tmp"
name="pcbcheck tells a via inside its pad"
via_to in > "$tmp/inside"
check sh -c '! "$1/pcbcheck" "$2" < "$3/inside" > "$3/out" &&
	grep -q "a via of GND is 0.000 mm from pad .* under the via gap 0.300" "$3/out"' \
	- "$P" "$B" "$tmp"
name="pcbcheck tells a pad of a small part heated unevenly"
check sh -c '! "$1/pcbcheck" "$2" < "$3/inside" > "$3/out" &&
	grep -q "thermal imbalance: pad .* joins the plane GND with a via by it" "$3/out"' \
	- "$P" "$B" "$tmp"
# A contact pad (no paste) is never soldered: a via may stand in it.
mkdir "$tmp/k"
cp test/sch_filter.expected "$B" "$tmp/k"
cp -r test/footprints "$tmp/k"
sed 's/^pad 2 .*/& nopaste/' test/footprints/c0603 > "$tmp/k/footprints/c0603"
name="a via may stand in a contact pad"
check sh -c 'awk "\$1 == \"pad\" && \$2 == \"GND\" { print \$3; exit }" "$1/inside" |
	grep -q "^[a-z.]*c\.2$" &&
	"$2/pcbcheck" "$1/k/pcb_filter.pcb" < "$1/inside" > "$1/out";
	! grep -q "a via of GND is 0.000 mm" "$1/out"' - "$tmp" "$P"
mkdir "$tmp/s"
cp test/sch_filter.expected "$B" "$tmp/s"
cp -r test/footprints "$tmp/s"
printf "pad 1 rrect -0.525 0 0.9 0.95\npad 2 rrect 0.525 0 0.9 0.95\n" \
	> "$tmp/s/footprints/c0603"
printf "court -1.48 -0.73 1.48 0.73\nsilk 0.12 -1 0 1 0\n" \
	>> "$tmp/s/footprints/c0603"
name="pcbcheck tells a mask sliver"
check sh -c '! "$1/pcbcheck" "$2/s/pcb_filter.pcb" < "$2/a" > "$2/out" &&
	grep -q "the mask web between pad filter.lpf.c.1 (C[0-9]*) and pad filter.lpf.c.2 (C[0-9]*) is 0.050 mm, under 0.100" "$2/out"' \
	- "$P" "$tmp"
name="pcbcheck tells silk on a pad"
check sh -c '! "$1/pcbcheck" "$2/s/pcb_filter.pcb" < "$2/a" > "$2/out" &&
	grep -q "the silkscreen of filter.lpf.c (C[0-9]*) runs over pad filter.lpf.c.[12] (C[0-9]*)" "$2/out"' \
	- "$P" "$tmp"
# Parts against each other, whatever their nets: two courtyards on one
# side must not overlap, and a hole a screw goes through keeps the parts
# of both sides off its keep-out (pcbfp(5)): a ferrite of a footprint
# with such a hole, moved to the bottom, and a capacitor on the top over
# it, must be told, and so must the capacitor moved onto another.
mkdir "$tmp/h"
cp test/sch_filter.expected "$B" "$tmp/h"
cp -r test/footprints "$tmp/h"
echo "hole 1 0 0 1 0.5 keep 4" >> "$tmp/h/footprints/l0603"
awk '$1 == "part" && $2 == "filter.vneg.fb" { $4 = 10; $5 = 5; $7 = "bottom" }
	$1 == "part" && $2 == "filter.lpf.c2" { $4 = 11; $5 = 5; $7 = "top" }
	{ print }' "$tmp/a" > "$tmp/over"
name="pcbcheck tells a part over the keep-out of a hole"
check sh -c '! "$1/pcbcheck" "$2/h/pcb_filter.pcb" < "$2/over" > "$2/out" &&
	grep -q "the courtyard of filter.lpf.c2 (C[0-9]*) in the keep-out of hole filter.vneg.fb.1/[0-9] (FB[0-9]*)" "$2/out"' \
	- "$P" "$tmp"
awk -v lock="$tmp/a" 'BEGIN { while ((getline l < lock) > 0) {
		split(l, w, " "); if (w[1] == "part" && w[2] == "filter.lpf.c") {
			x = w[4]; y = w[5] } } }
	$1 == "part" && $2 == "filter.lpf.c2" { $4 = x; $5 = y }
	{ print }' "$tmp/a" > "$tmp/onto"
name="pcbcheck tells two courtyards on one side overlapping"
check sh -c '! "$1/pcbcheck" "$2" < "$3/onto" > "$3/out" &&
	grep -q "the courtyards of filter.lpf.c (C[0-9]*) and filter.lpf.c2 (C[0-9]*) overlap" "$3/out"' \
	- "$P" "$B" "$tmp"
# The fanout keeps a small part's pads alike however wide its plane's
# net: its necks no wider than a track; and its pad on the plane's own
# layer (the bottom) is relieved, joined by a neck.
mkdir "$tmp/t"
cp -r test/footprints test/sch_filter.expected "$tmp/t"
{ cat "$B"; echo "width GND 0.5"; } |
	sed 's/^place filter.vneg.fb left of filter.vneg.c$/fix filter.vneg.fb at 14 24 bottom/' \
	> "$tmp/t/pcb_filter.pcb"
build "$tmp/t/pcb_filter.pcb" /dev/null > "$tmp/t/lock" 2> /dev/null
name="the fanout necks a small part's pad on a wide plane net"
check sh -c '! grep -q "^track GND top 0.5000" "$1/t/lock" &&
	grep -q "^track GND top 0.2000" "$1/t/lock" &&
	! "$2/pcbcheck" "$1/t/pcb_filter.pcb" < "$1/t/lock" |
	grep -q "thermal imbalance"' - "$tmp" "$P"
name="a small part's pad on its plane's own layer joins it by a neck"
check grep -q "^track -12V bottom 0.2000 " "$tmp/t/lock"
# Routes end at pad centers: a track end moved off its pad's center is
# told; pads in line are joined by one straight track through their
# centers.
name="pcbcheck tells a track ending off a pad's center"
awk '$1 == "track" && $3 == "top" && !done { $5 += 0.3; done = 1 } { print }' \
	"$tmp/a" > "$tmp/offcenter"
check sh -c '! "$1/pcbcheck" "$2" < "$3/offcenter" > "$3/out" &&
	grep -q "ends off a pad.s center or end and off a via" "$3/out"' \
	- "$P" "$B" "$tmp"
mkdir "$tmp/i"
cp -r test/footprints test/sch_filter.expected "$tmp/i"
sed 's/^place filter.lpf.c down from filter.lpf.r.2$/& rot 90/;
	s/^place filter.lpf.c2 right of filter.lpf.c$/& rot 90/' "$B" > "$tmp/i/pcb_filter.pcb"
build "$tmp/i/pcb_filter.pcb" /dev/null > "$tmp/i/lock" 2> /dev/null
name="pads in line are joined by one straight track through their centers"
check awk '
	$1 == "pad" && $3 == "filter.lpf.c.1" { ax = $5; ay = $6 }
	$1 == "pad" && $3 == "filter.lpf.c2.1" { bx = $5; by = $6 }
	$1 == "link" && $2 == "OUT" { on = ($3 ~ /^filter\.lpf\.c2?\.1$/ && $4 ~ /^filter\.lpf\.c2?\.1$/) }
	$1 == "link" && $2 != "OUT" { on = 0 }
	$1 == "track" && on { n++; t = $0; np = (NF - 4) / 2; x0 = $5; y0 = $6
		x1 = $(NF - 1); y1 = $NF }
	END {
		if (n != 1 || np != 2 || ay != by) exit 1
		if (!((x0 == ax && y0 == ay && x1 == bx && y1 == by) ||
		      (x0 == bx && y0 == by && x1 == ax && y1 == ay))) exit 1
	}' "$tmp/i/lock"
# Widths by connection: the copper from the output pin to the filter's
# resistor is wide, the branch to the feedback resistor the net's; an
# edit of that width makes anew that connection alone; and copper
# narrower than its width is told.
mkdir "$tmp/w"
cp -r test/footprints test/sch_filter.expected "$tmp/w"
{ cat "$B"; echo "width filter.buf.amp.out filter.lpf.r.1 0.5"; } > "$tmp/w/pcb_filter.pcb"
build "$tmp/w/pcb_filter.pcb" /dev/null > "$tmp/w/lock" 2> /dev/null
name="a connection takes its own width, the rest the net's"
check sh -c 'grep -q "^link filter.buf.out filter.buf.amp.6 filter.lpf.r.1 0.5000$" "$1" &&
	grep -q "^link filter.buf.out .*filter.buf.rf.1.* 0.2000$" "$1" &&
	grep -A1 "^link filter.buf.out filter.buf.amp.6 filter.lpf.r.1" "$1" |
	grep -q "^track filter.buf.out top 0.5000 "' - "$tmp/w/lock"
sed 's/^width filter.buf.amp.out filter.lpf.r.1 0.5$/width filter.buf.amp.out filter.lpf.r.1 0.45/' \
	"$tmp/w/pcb_filter.pcb" > "$tmp/w/edit.pcb"
cp "$tmp/w/pcb_filter.pcb" "$tmp/w/keep.pcb"
cp "$tmp/w/edit.pcb" "$tmp/w/pcb_filter.pcb"
build "$tmp/w/pcb_filter.pcb" "$tmp/w/lock" > "$tmp/w/lock2" 2> /dev/null
name="an edit of a width makes anew that connection alone"
check sh -c 'diff "$1" "$2" | grep "^[<>]" | grep -v \
	-e "^[<>] link filter.buf.out filter.buf.amp.6 filter.lpf.r.1 " \
	-e "^[<>] track filter.buf.out top 0.[45]" | grep -q . && exit 1
	diff "$1" "$2" | grep -q "^> link filter.buf.out filter.buf.amp.6 filter.lpf.r.1 0.4500$"' \
	- "$tmp/w/lock" "$tmp/w/lock2"
name="pcbcheck tells copper narrower than its width"
awk '/^link filter.buf.out filter.buf.amp.6 filter.lpf.r.1/ { on = 1; print; next }
	$1 == "link" || $1 == "net" { on = 0 }
	$1 == "track" && on { $4 = "0.2000" } { print }' "$tmp/w/lock" > "$tmp/w/narrow"
cp "$tmp/w/keep.pcb" "$tmp/w/pcb_filter.pcb"
check sh -c '! "$1/pcbcheck" "$2/w/pcb_filter.pcb" < "$2/w/narrow" > "$2/out" &&
	grep -q "the copper from pad filter.buf.amp.6 (U701) to pad filter.lpf.r.1 (R703) is 0.200 mm wide, narrower than its width 0.500" "$2/out"' \
	- "$P" "$tmp"
# A layout by groups: the filter laid out alone (its root at the
# origin, its section of the lock its own), then the board with the
# filter a block: its parts as its section has them, moved whole, and
# its copper with them; one lock file, a section a group.
mkdir "$tmp/b"
cp -r test/footprints test/sch_filter.expected "$tmp/b"
{ cat "$B"; echo "block filter.lpf"; } > "$tmp/b/b.pcb"
"$P/pcbplace" -g filter.lpf "$tmp/b/b.pcb" < /dev/null |
	"$P/pcbroute" -g filter.lpf "$tmp/b/b.pcb" > "$tmp/b/lpf.lock" 2> /dev/null
build "$tmp/b/b.pcb" /dev/null > "$tmp/b/board.lock" 2> /dev/null
"$P/pcbplace" "$tmp/b/b.pcb" "$tmp/b/missing.lock" 2> /dev/null |
	"$P/pcbroute" "$tmp/b/b.pcb" > "$tmp/b/fromnone.lock" 2> /dev/null
name="a run lays out the groups and the board; a lock not there is none"
check cmp -s "$tmp/b/board.lock" "$tmp/b/fromnone.lock"
section() {
	awk -v g="$1" '$1 == "group" { on = $2 == g; next } on' "$2"
}
board() {
	awk '$1 == "group" { exit } { print }' "$1"
}
section filter.lpf "$tmp/b/lpf.lock" > "$tmp/b/lpf.sec"
section filter.lpf "$tmp/b/board.lock" > "$tmp/b/lpf.kept"
board "$tmp/b/board.lock" > "$tmp/b/board.sec"
name="a group laid out alone: its parts alone, its root at the origin"
check awk '$1 == "part" { n++; if ($2 !~ /^filter\.lpf\./) bad = 1
		if ($2 == "filter.lpf.r" && ($4 != 0 || $5 != 0)) bad = 1 }
	$1 == "track" && $2 == "OUT" { t++ }
	END { exit !(n == 3 && !bad && t > 0) }' "$tmp/b/lpf.sec"
name="one lock: the board's section, and the group's as -g lays it out"
check sh -c 'grep -q "^part filter.buf.amp " "$1/b/board.sec" &&
	cmp -s "$1/b/lpf.sec" "$1/b/lpf.kept" &&
	test "$(grep -c "^group " "$1/b/board.lock")" = 1' - "$tmp"
rel() {
	awk -v g="filter.lpf." '$1 == "part" && index($2, g) == 1 { p[$2] = $4 " " $5 " " $6 " " $7 }
	$1 == "part" && $2 == "filter.lpf.r" { x0 = $4; y0 = $5 }
	$1 == "track" && $2 == "OUT" { t[++n] = $0 }
	END { for (k in p) { split(p[k], v, " ")
		printf "part %s %.4f %.4f %s %s\n", k, v[1] - x0, v[2] - y0, v[3], v[4] }
		for (i = 1; i <= n; i++) { m = split(t[i], w, " "); s = w[4]
			for (j = 5; j < m; j += 2)
				s = s sprintf(" %.4f %.4f", w[j] - x0, w[j + 1] - y0)
			print "track " s } }' "$1" | sort
}
rel "$tmp/b/lpf.sec" > "$tmp/b/alone"
rel "$tmp/b/board.sec" > "$tmp/b/placed"
name="a block keeps its layout and its copper on the board"
check sh -c 'test -s "$1/b/alone" && cmp -s "$1/b/alone" "$1/b/placed"' \
	- "$tmp"
# The loops of current: the decoupling capacitor's, from its supply pin
# to its ground via, and the feedback resistor's, through the amplifier.
name="pcbcheck -l measures the loop through each part of two pins"
check sh -c '"$1/pcbcheck" -l "$2" < "$3/a" > "$3/l"
	awk "\$1 == \"loop\" && \$6 > 0 { n[\$2] = 1 }
	END { exit !(n[\"filter.vpos.c\"] && n[\"filter.vneg.c\"] && n[\"filter.buf.rf\"]) }" "$3/l"' \
	- "$P" "$B" "$tmp"
# Alike groups, by alike statements: each laid out alone, the board of
# the two; the second's copper is the first's, moved.
T=test/pcb_twin.pcb
build "$T" /dev/null > "$tmp/twin.board" 2> /dev/null
copper() {
	awk -v c="$1." '$1 == "group" { exit }
	$1 == "part" && $2 == c "r" { x0 = $4; y0 = $5 }
	$1 == "track" && index($2, c) == 1 { t[++n] = $0 }
	END { for (i = 1; i <= n; i++) { m = split(t[i], w, " ")
		s = substr(w[2], length(c) + 1) " " w[3] " " w[4]
		for (j = 5; j < m; j += 2)
			s = s sprintf(" %.4f %.4f", w[j] - x0, w[j + 1] - y0)
		print s } }' "$tmp/twin.board" | sort
}
copper a > "$tmp/twin.a"
copper b > "$tmp/twin.b"
name="alike groups by alike statements have alike copper"
check sh -c 'test -s "$1/twin.a" && cmp -s "$1/twin.a" "$1/twin.b"' - "$tmp"
name="gerber vectors"
mkdir "$tmp/g"
check sh -c '"$1/pcbgerber" -o "$2/g" "$3" < "$2/a" &&
	"$1/gbrvec" "$2/g/top.gbr" | grep -q "^name filter.buf.amp "' \
	- "$P" "$tmp" "$B"
mkdir "$tmp/v"
name="pcbvec names the parts"
check sh -c '"$1/pcbvec" -o "$2/v" "$3" < "$2/a" &&
	grep -q "^name filter.buf.amp " "$2/v/top.vec" &&
	grep -q "^part filter.buf.amp" "$2/v/topsilk.vec"' - "$P" "$tmp" "$B"
name="the layout names its parts by designator too"
check sh -c 'grep -qx "part filter.buf.amp U701" "$2/v/top.vec" &&
	"$1/pcbplace" -v "$3" < /dev/null 2>&1 > /dev/null |
	grep -q ": filter.buf.rf (R701) at "' - "$P" "$tmp" "$B"
# Each layer says what it is, before it draws, by KiCad's name; so do
# the Gerbers through gbrvec.
name="the vectors name their layers"
check sh -c 'cd "$1/v" && for f in top:F.Cu in1:In1.Cu in2:In2.Cu \
	bottom:B.Cu topsilk:F.Silkscreen bottomsilk:B.Silkscreen \
	topcourt:F.Courtyard topmask:F.Mask toppaste:F.Paste edge:Edge.Cuts \
	drill:Drill; do
	grep -v "^#" "${f%%:*}.vec" | head -1 | grep -qx "layer ${f#*:}" || exit 1
	done' - "$tmp"
# A region render: the window of a part, named by its designator, with
# the designators and the nets' names in it; and the window of a file
# is of its drawing alone (a net named -500 is no point).
name="vecps renders the region of a part with its names"
check sh -c '"$1/vecps" -p U701 "$2/v/top.vec" "$2/v/topsilk.vec" > "$2/r.ps" &&
	grep -q "(U701) dup stringwidth" "$2/r.ps" &&
	grep -q "(filter.buf.out) dup stringwidth" "$2/r.ps" &&
	"$1/vecps" -r 0 0 40 30 "$2/v/top.vec" | grep -q "(U701) dup"' \
	- "$P" "$tmp"
printf "net -500\nline 0.1 0 0 10 10\nnet\n" > "$tmp/neg.vec"
name="vecps windows a file by its drawing alone"
check sh -c '"$1/vecps" "$2/neg.vec" | grep -q " -5.0000 -5.0000 translate"' \
	- "$P" "$tmp"
name="gbrvec names the layers of Gerbers"
check sh -c '"$1/gbrvec" "$2/g/top.gbr" | grep -qx "layer F.Cu" &&
	"$1/gbrvec" "$2/g/in2.gbr" | grep -qx "layer In2.Cu" &&
	"$1/gbrvec" "$2/g/bottomsilk.gbr" | grep -qx "layer B.Silkscreen" &&
	"$1/gbrvec" "$2/g/edge.gbr" | grep -qx "layer Edge.Cuts" &&
	"$1/gbrvec" "$2/g/drill.drl" | grep -qx "layer Drill"' - "$P" "$tmp"
exit $fail
