#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# viewtest.sh -- Run the tests of schview and pcbview
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/viewtest.sh DIR
# The programs are those in DIR, with xkey (test/xkey.c), built there by
# make.  The viewers run on a display of their own (Xvfb), driven as a
# user drives them: the keys and the mouse by XTest, through xkey, and
# the commands of their stdin (select, layer, shot, ...).  The holes of
# the letters and digits must show the ground; a click on a part's
# label, between the strokes of its body, or on its body selects it, and
# a click on nothing prints nothing; in pcbview a click on a part
# selects it; and the median time of a frame while zooming, dragging and
# clicking, measured from outside (from each input to the window's next
# picture), must keep within its budget in test/view.budget.  It prints
# ok or FAIL for each, or skip without Xvfb, and exits non-zero if any
# fails.

cd "$(dirname "$0")/.." || exit 1
P=${1:?usage: test/viewtest.sh DIR}
tmp=$(mktemp -d) || exit 1
xvfb=
trap 'test -n "$xvfb" && kill "$xvfb" && wait "$xvfb"; rm -rf "$tmp"' EXIT
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

if ! command -v Xvfb > /dev/null; then
	echo "skip viewers: no Xvfb"
	exit 0
fi
X=$P/xkey
if ! test -x "$X"; then
	echo "skip viewers: no $X (make build/xkey)"
	exit 0
fi
n=90
while [ -e "/tmp/.X11-unix/X$n" ] && [ $n -lt 99 ]; do
	n=$((n + 1))
done
Xvfb ":$n" -screen 0 1000x700x24 -nolisten tcp > /dev/null 2>&1 &
xvfb=$!
DISPLAY=":$n"
export DISPLAY
i=0
while [ ! -e "/tmp/.X11-unix/X$n" ] && [ $i -lt 50 ]; do
	sleep 0.1
	i=$((i + 1))
done

# view PROGRAM ARGS... < COMMANDS: runs a viewer on the commands, one a
# twentieth of a second (a line of ! runs as a command of the test: xkey,
# the input of a user), and waits for the picture the last one asks
# for; its stdout goes to $tmp/out and its stderr to $tmp/err.
view() {
	rm -f "$tmp/in" "$tmp/shot.ppm"
	mkfifo "$tmp/in"
	"$@" < "$tmp/in" > "$tmp/out" 2> "$tmp/err" &
	v=$!
	exec 3> "$tmp/in"
	sleep 0.5 # Its window mapped, for the keys.
	while read -r line; do
		case $line in
		!*) sh -c "${line#!}" ;; # A command of the test: input.
		*) echo "$line" >&3 ;;
		esac
		sleep 0.05
	done
	echo "shot $tmp/shot.ppm" >&3
	i=0
	while [ ! -s "$tmp/shot.ppm" ] && [ $i -lt 100 ]; do
		sleep 0.1
		i=$((i + 1))
	done
	sleep 0.2
	exec 3>&-
	kill "$v" 2> /dev/null
	wait "$v" 2> /dev/null
}

# The holes of the glyphs: the middle of the points of each hole of the
# text of view_glyphs.vec, by the glyph table, in pixels of the window
# as the viewer fits the sheet (its box 0 to 1000 by 0 to 700, as two
# disks give it, grown by their radius).
holes() {
	awk -v text="0 4 6 8 9 B O R p" '
	$1 == "font" { font = $2 }
	font == "R" && $1 == "glyph" { code = $2; adv[code] = $3; n[code] = 0 }
	font == "R" && $1 == "-" {
		k = ++n[code]; sx = 0; sy = 0; m = 0
		for (i = 2; i <= NF; i++) {
			if ($i ~ /^[0-9.-]+$/ && $(i + 1) ~ /^[0-9.-]+$/) {
				sx += $i; sy += $(i + 1); m++; i++
			}
		}
		hx[code, k] = sx / m; hy[code, k] = sy / m
	}
	END {
		z = 1000 / 1001 < 700 / 701 ? 1000 / 1001 : 700 / 701
		z *= 0.95; x = 40
		for (i = 1; i <= length(text); i++) {
			c = substr(text, i, 1)
			d = index("0123456789", c)
			code = d ? 47 + d : \
			    c == " " ? 32 : c == "B" ? 66 : c == "O" ? 79 : \
			    c == "R" ? 82 : 112
			for (k = 1; k <= n[code]; k++) {
				wx = x + hx[code, k] * 0.08
				wy = 300 + hy[code, k] * 0.08
				printf "%s %d %d\n", c, 500 + (wx - 500) * z, \
				    350 - (wy - 350) * z
			}
			x += adv[code] * 0.08
		}
	}' lib/fonts/nimbussans.txt
}

name="schview holes of letters"
echo "" | view "$P/schview" test/view_glyphs.vec
holes > "$tmp/holes"
check sh -c 'test -s "$1/holes" &&
	while read -r c x y; do
		at=$((16 + (y * 1000 + x) * 3))
		set -- $(od -An -tu1 -v -j $at -N 3 "$0/shot.ppm")
		test "$1" -lt 40 || { echo "$c at $x $y is lit"; exit 1; }
	done < "$1/holes"' "$tmp" "$tmp"

# The sheets of the tests, drawn.
mkdir "$tmp/sheets"
for f in test/sch_*.sch; do
	"$P/schdraw" -L lib "$f" > "$tmp/sheets/$(basename "$f" .sch).vec"
done

# Clicks on view_pick.vec, by the same fit: its box 0 to 400 by 0 to 300.
px() {
	awk -v x="$1" -v y="$2" 'BEGIN {
		z = 1000 / 401 < 700 / 301 ? 1000 / 401 : 700 / 301; z *= 0.95
		printf "%d %d\n", 500 + (x - 200) * z, 350 - (y - 150) * z }'
}
name="schview picks a part by its elements"
echo "!$X schview click $(px 105 130) click $(px 155 143) \
	click $(px 320 120) click $(px 200 200) click $(px 2 150) \
	click $(px 50 260)" | view "$P/schview" test/view_pick.vec
check sh -c 'printf "select a\nselect a\nselect b\n" |
	diff -u - "$1/out"' - "$tmp"
# The last clicks, on a wire, the frame and nothing, select no part, and
# the last, on nothing, clears the net the wire selected: no pixel is the
# cyan of a selection.
name="schview lights nothing of no part"
check sh -c '! od -An -tu1 -v -w3 "$1/shot.ppm" |
	awk "\$1 < 200 && \$2 > 250 && \$3 > 250 { f = 1 }
	     END { exit !f }"' - "$tmp"

# Clicks everywhere, on every part's body, label and pin number: the
# viewer lives through them all and draws its picture.
grid() {
	printf "!%s %s" "$X" "$1"
	for y in $(seq 5 20 695); do
		for x in $(seq 5 20 995); do
			printf " click %d %d" "$x" "$y"
		done
	done
	echo
}
for f in test/view_pick.vec test/view_glyphs.vec $tmp/sheets/*.vec; do
	name="schview lives through clicks all over $(basename "$f")"
	grid schview | view "$P/schview" "$f"
	check test -s "$tmp/shot.ppm"
done

# The time of a frame, from outside: xkey -t prints, for each input, the
# time to the window's next picture; the median of those after the
# first two.
median() {
	awk '$1 == "frame" { t[++n] = $2 } END {
		m = 0; for (i = 3; i <= n; i++) s[++m] = t[i]
		for (i = 1; i <= m; i++) for (j = i + 1; j <= m; j++)
			if (s[j] < s[i]) { x = s[i]; s[i] = s[j]; s[j] = x }
		print m ? s[int((m + 1) / 2)] : 1e9 }' "$1"
}
budget() {
	awk -v p="$1" '$1 == p { print $2 }' test/view.budget
}
# drive CLASS: zooming in, dragging, and clicking and zooming out, ten
# times each, timed.
drive() {
	printf "!%s -t %s move 500 350" "$X" "$1"
	for k in 1 2 3 4 5 6 7 8 9 10; do
		printf " wheel 1 500 350"
	done
	for k in 1 2 3 4 5 6 7 8 9 10; do
		printf " drag 7 3"
	done
	for k in 1 2 3 4 5 6 7 8 9 10; do
		printf " click 500 350 wheel -1 500 350"
	done
	printf " > %s/times\n" "$tmp"
}

rm -f "$tmp/times"
drive schview | view "$P/schview" test/view_pick.vec
t=$(median "$tmp/times")
name="schview frame $t ms, budget $(budget schview) ms"
check awk -v t="$t" -v b="$(budget schview)" 'BEGIN { exit !(t <= b) }'

# A board: test/pcb_filter.pcb laid out, its vectors from pcbvec.
mkdir "$tmp/v"
"$P/pcbplace" test/pcb_filter.pcb < /dev/null |
	"$P/pcbroute" test/pcb_filter.pcb > "$tmp/lock"
"$P/pcbvec" -o "$tmp/v" test/pcb_filter.pcb < "$tmp/lock"
L="$tmp/v/bottom.vec $tmp/v/top.vec $tmp/v/topmask.vec $tmp/v/topsilk.vec"
L="$L $tmp/v/topcourt.vec $tmp/v/edge.vec $tmp/v/drill.vec"
name="pcbview picks a part, not the edge"
printf "select buf.amp\ncenter\n!%s pcbview click 500 350 wheel -30 500 350 click 1 1\n" \
	"$X" | view "$P/pcbview" $L
check sh -c 'test "$(cat "$1/out")" = "select buf.amp"' - "$tmp"

# Real keys, by XTest: ? shows the help, and the keypad's 1 hides the
# first inner layer, as the bare 1 does.
name="pcbview keys: ? and the keypad"
{
	echo "shot $tmp/a.ppm"
	echo "!$X pcbview question KP_1"
} | view "$P/pcbview" $tmp/v/in1.vec $tmp/v/top.vec $tmp/v/edge.vec
	check sh -c 'test -s "$1/a.ppm" &&
		test "$(cmp -l "$1/a.ppm" "$1/shot.ppm" | wc -l)" -gt 1000' \
		- "$tmp"
	# Every toggle, twice off and on, by its key, by its key in the bar
	# and by its command: each off shows a change, each on the same as
	# before, each off the same as the first.  The bar's keys here are
	# t b s y e d z, a cell of 12 pixels each.  Each picture is kept as
	# its checksum.
	toggle() {
		k=$1
		how=$2
		cmd=$(printf "%s\n" "t layer top" "b layer bottom" \
			"s layer silk" "y courtyard" "e layer edge" \
			"d layer drill" "z zones" "l bar" |
			awk -v k="$k" '$1 == k { print substr($0, 3) }')
		at=$(expr index "tbsyedz" "$k")
		for n in base off on off2 on2; do
			echo "shot $tmp/$n.ppm"
			test $n = on2 && break
			case $how in
			key) echo "!$X pcbview $k" ;;
			bar) echo "!$X pcbview click $((at * 12 - 6)) 4" ;;
			pipe) if test "$k" = z; then
				case $n in
				base | on) echo "zones fill" ;;
				*) echo "zones outline" ;;
				esac
			else
				echo "$cmd"
			fi ;;
			esac
		done
	}
	bad=
	for k in t b s y e d z l; do
		for how in key bar pipe; do
			test $how = bar && test "$k" = l && continue
			toggle $k $how | view "$P/pcbview" $L
			for n in base off on off2 on2; do
				eval "$n=\$(cksum < \"\$tmp/\$n.ppm\")"
				rm -f "$tmp/$n.ppm"
			done
			test "$base" != "$off" && test "$base" = "$on" &&
				test "$off" = "$off2" &&
				test "$base" = "$on2" || bad="$bad $k-$how"
		done
	done
	name="pcbview toggles each layer off and on, twice, three ways$bad"
	check test -z "$bad"
	name="pcbview: A, layer none hide all; a, layer all show all"
	{
		echo "shot $tmp/all0.ppm"
		echo "!$X pcbview A"
		echo "shot $tmp/all1.ppm"
		echo "!$X pcbview a"
		echo "shot $tmp/all2.ppm"
		echo "layer none"
		echo "shot $tmp/all3.ppm"
		echo "layer all"
	} | view "$P/pcbview" $L
	check sh -c '! cmp -s "$1/all0.ppm" "$1/all1.ppm" &&
		cmp -s "$1/all0.ppm" "$1/all2.ppm" &&
		cmp -s "$1/all1.ppm" "$1/all3.ppm" &&
		cmp -s "$1/all0.ppm" "$1/shot.ppm"' - "$tmp"
# The bar: its z plain while the zones are outlines (at first), on
# gray once clicked; the pointer over a key names it on a dark box under the bar.
# Its keys here: t b s y e d z, a cell each, 12 pixels wide (the font
# fixed, 6 wide, and 6 more); the names drawn on 0.1 gray, its red 26.
cell() {
	od -An -tu1 -v -j $((16 + ($2 * 1000 + $1) * 3)) -N 1 "$tmp/shot.ppm"
}
name="pcbview bar: zones"
echo "!$X pcbview move 1 1" | view "$P/pcbview" $L
z1=$(cell $((6 * 12 + 1)) 2)
h1=$(cell 4 30)
printf "!%s pcbview click %d 4\n" "$X" $((6 * 12 + 6)) | view "$P/pcbview" $L
z2=$(cell $((6 * 12 + 1)) 2)
check test "$z1" -lt 60 -a "$z2" -gt 100
name="pcbview bar: names"
check test "$h1" -gt 15 -a "$h1" -lt 40
# The names of nets and pins, as KiCad shows them: none at the fit, and
# on the pads and tracks zoomed in on the amplifier (their white).
whites() {
	od -An -tu1 -v -w3 "$1" |
		awk '$1 > 240 && $2 > 225 && $3 > 225 { n++ } END { print n + 0 }'
}
{
	echo "shot $tmp/n0.ppm"
	echo "select buf.amp"
	echo "center"
	echo "select"
	echo "!$X pcbview wheel 14 500 350"
} | view "$P/pcbview" $L
n0=$(whites "$tmp/n0.ppm")
n1=$(whites "$tmp/shot.ppm")
name="pcbview names pads and tracks once they can be read: $n0 at the fit, $n1 zoomed"
check test "$n0" -lt 20 -a "$n1" -gt 200
# A click on a track selects its connected track, brightened as KiCad
# brightens it (the copper's 200 52 52 to 217 114 114), with its net's
# name in the upper right corner: the longest segment of a track of
# buf.out, at the fit (the board 0 0 40 30 about 20 15, 22.17 pixels a
# mm).
near() {
	od -An -tu1 -v -w3 "$1" | awk -v r="$2" -v g="$3" -v b="$4" '
	function d(x, y) { return x > y ? x - y : y - x }
	d($1, r) <= 2 && d($2, g) <= 2 && d($3, b) <= 2 { n++ }
	END { print n + 0 }'
}
at=$(awk '$1 == "link" { on = $2 == "buf.out" }
	$1 == "track" && on && !done {
		for (i = 5; i + 3 <= NF; i += 2) {
			l = ($(i + 2) - $i) ^ 2 + ($(i + 3) - $(i + 1)) ^ 2
			if (l > best) { best = l; x = ($i + $(i + 2)) / 2
				y = ($(i + 1) + $(i + 3)) / 2 }
		}
		done = 1
	}
	END { z = 0.95 * 700 / 30
		printf "%d %d", 500 + (x - 20) * z, 350 - (y - 15) * z }' "$tmp/lock")
{
	echo "shot $tmp/t0.ppm"
	echo "!$X pcbview click $at"
} | view "$P/pcbview" $L
b0=$(near "$tmp/t0.ppm" 217 114 114)
b1=$(near "$tmp/shot.ppm" 217 114 114)
corner=$(od -An -tu1 -v -w3 -j 16 -N $((16 * 1000 * 3)) "$tmp/shot.ppm" |
	awk '(NR - 1) % 1000 >= 850 && $1 > 200 && $2 > 200 && $3 > 200 { n++ }
	END { print n + 0 }')
name="pcbview selects a track at $at: brightened ($b0 to $b1), its net named"
check sh -c 'test "$1" -eq 0 && test "$2" -gt 10 && test "$3" -gt 0 &&
	test -z "$(cat "$4/out")"' - "$b0" "$b1" "$corner" "$tmp"
name="pcbview lives through clicks all over the board"
grid pcbview | view "$P/pcbview" $L
check test -s "$tmp/shot.ppm"
# The zones as outlines (as they start): outlined, the top copper is
# as filled (its pixels, red 200 52 52), the plane's fill gone (green
# 127 200 127) but for its outline, and filled again it is back.
count() {
	od -An -tu1 -v -w3 "$1" | awk -v r="$2" -v g="$3" -v b="$4" \
		'$1 == r && $2 == g && $3 == b { n++ } END { print n + 0 }'
}
{
	echo "zones fill"
	echo "shot $tmp/z0.ppm"
	echo "zones outline"
	echo "shot $tmp/z1.ppm"
	echo "zones fill"
} | view "$P/pcbview" $tmp/v/in1.vec $tmp/v/top.vec
r0=$(count "$tmp/z0.ppm" 200 52 52)
r1=$(count "$tmp/z1.ppm" 200 52 52)
g0=$(count "$tmp/z0.ppm" 127 200 127)
g1=$(count "$tmp/z1.ppm" 127 200 127)
name="pcbview z: fills go ($g0 to $g1), copper stays ($r0, $r1), back"
check sh -c 'cmp -s "$1/z0.ppm" "$1/shot.ppm" && test $2 -gt 0 &&
	test $2 = $3 && test $5 -gt 0 && test $5 -lt $(($4 / 10))' \
	- "$tmp" "$r0" "$r1" "$g0" "$g1"

# The outline the same width on the screen at every zoom: on the row
# through the middle of the board's left side (its board 0 0 40 30, so
# at the fit the side stands at x 57), as the wheel zooms about it, the
# light of its pixels near x 57, summed, within a third of the fit's.
edge_light() {
	od -An -tu1 -v -w3 -j $((16 + (350 * 1000 + 37) * 3)) -N 120 \
		"$tmp/shot.ppm" | awk '{ n += $1 } END { print n + 0 }'
}
for z in 0 10 21; do
	echo "!$X pcbview wheel $z 57 350" | view "$P/pcbview" \
		$tmp/v/bottom.vec $tmp/v/edge.vec
	eval "e$z=$(edge_light)"
done
name="pcbview outline as wide at 1, 9 and 108 times: $e0 $e10 $e21"
check awk -v a="$e0" -v b="$e10" -v c="$e21" 'BEGIN {
	exit !(a > 100 && b > a * 2 / 3 && b < a * 4 / 3 &&
	       c > a * 2 / 3 && c < a * 4 / 3) }'
rm -f "$tmp/times"
drive pcbview | view "$P/pcbview" $L
t=$(median "$tmp/times")
name="pcbview frame $t ms, budget $(budget pcbview) ms"
check awk -v t="$t" -v b="$(budget pcbview)" 'BEGIN { exit !(t <= b) }'
exit $fail
