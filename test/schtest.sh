#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# schtest.sh -- Run the tests of schdraw, schnet and netdiff
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/schtest.sh DIR
# The programs are those in DIR, and schconn (test/schconn.c), built
# there by eda.mk.  Each test/sch_*.sch must give the netlist of its
# .expected, draw without a word on stderr, and its drawing must join
# what its netlist joins (schconn); a drawing with a wire taken out, or
# with two pins traded, must not; each test/bad_sch_*.sch must fail
# with the message of its .expected; a sheet template with more room
# round bodies draws another sheet, still without a fault; the netdiff
# cases compare by hand-made netlists.  It prints ok or FAIL for each
# and exits non-zero if any fails.

cd "$(dirname "$0")/.." || exit 1
P=${1:?usage: test/schtest.sh DIR}
L=lib
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

for f in test/sch_*.sch; do
	b=$(basename "$f" .sch)
	name="schnet $b"
	check sh -c '"$1/schnet" -L "$2" "$3" > "$4/nl" &&
		diff -u "$5" "$4/nl"' - "$P" "$L" "$f" "$tmp" "test/$b.expected"
	name="schdraw $b"
	check sh -c '"$1/schdraw" -L "$2" "$3" > "$4/vec" 2> "$4/err" &&
		! test -s "$4/err" && grep -q "^page " "$4/vec" &&
		"$1/vecps" "$4/vec" | grep -q "^showpage"' \
		- "$P" "$L" "$f" "$tmp"
	name="schconn $b: the drawing joins what the netlist joins"
	check sh -c '"$1/schdraw" -L "$2" "$3" > "$4/vec" &&
		"$1/schnet" -L "$2" "$3" > "$4/nl" &&
		"$1/schconn" "$4/vec" "$4/nl"' - "$P" "$L" "$f" "$tmp"
done

# Drawings made wrong on purpose: the first wire of a net taken out, and
# the places of the first two pins of a part traded; schconn must tell
# each.
"$P/schdraw" -L "$L" test/sch_filter.sch > "$tmp/good.vec"
"$P/schnet" -L "$L" test/sch_filter.sch > "$tmp/good.nl"
awk '!cut && $1 == "line" && net { cut = 1; next }
	$1 == "net" { net = NF > 1 } { print }' "$tmp/good.vec" > "$tmp/cut.vec"
name="schconn tells a wire missing"
check sh -c '! "$1/schconn" "$2/cut.vec" "$2/good.nl" > "$2/out" &&
	grep -q "is not drawn joined to" "$2/out"' - "$P" "$tmp"
awk '$1 == "pin" && !a { a = $2; ax = $3; ay = $4; next }
	$1 == "pin" && a && !b && index($2, substr(a, 1, index(a, "."))) == 1 {
		b = $2; print "pin " b " " ax " " ay; print "pin " a " " $3 " " $4
		next }
	{ print }' "$tmp/good.vec" > "$tmp/swap.vec"
name="schconn tells two pins traded"
check sh -c '! "$1/schconn" "$2/swap.vec" "$2/good.nl" > "$2/out" &&
	grep -q "is drawn joined to" "$2/out"' - "$P" "$tmp"

# Designators wherever a part is named, but never in the source: the
# brackets of the drawing and the traces of -v.
name="schdraw names its parts by designator too"
check sh -c 'grep -qx "part buf.rf R701" "$1/good.vec" &&
	"$2/schdraw" -v -L "$3" test/sch_filter.sch 2>&1 > /dev/null |
	grep -q "^test/sch_filter.sch:[0-9]*: buf.rf (R701) at "' \
	- "$tmp" "$P" "$L"

# Every part shows its designator; the ferrites of the kind the sheet
# uses most go without their value alone.
name="the common ferrites keep their designators, not their values"
check awk '$1 == "part" && NF == 3 { p = $2 } $1 == "part" && NF == 1 { p = "" }
	p ~ /^v(pos|neg)\.fb$/ && $1 == "text" && $NF == "FB" { fb[p] = 1 }
	p ~ /^v(pos|neg)\.fb$/ && $1 == "text" && $NF == "600R" { bad = 1 }
	p == "buf.rf" && $1 == "text" && $NF == "1k" { rf = 1 }
	END { exit !(fb["vpos.fb"] && fb["vneg.fb"] && !bad && rf) }' \
	"$tmp/good.vec"

# The room the drawing keeps is the sheet template's: one with more room
# round bodies draws another sheet, which keeps it.
mkdir "$tmp/wide" "$tmp/wide/symbols"
cp -r test/sheets "$tmp/wide"
{ echo "sheet wide"; cat test/sch_filter.sch; } > "$tmp/wide/w.sch"
name="schdraw keeps the room of its sheet template"
check sh -c '"$1/schdraw" -L "$2" "$3/wide/w.sch" > "$3/w.vec" 2> "$3/err" &&
	! test -s "$3/err" && ! cmp -s "$3/w.vec" "$3/good.vec"' \
	- "$P" "$L" "$tmp"

for f in test/bad_sch_*.sch; do
	b=$(basename "$f" .sch)
	name="schnet $b"
	check sh -c '! "$1/schnet" -L "$2" "$3" > /dev/null 2> "$4/err" &&
		diff -u "$5" "$4/err"' \
		- "$P" "$L" "$f" "$tmp" "test/$b.expected"
done

name="netdiff same"
check sh -c '"$1/netdiff" test/netdiff_same.nl test/sch_filter.expected \
	> "$2/out" && ! test -s "$2/out"' - "$P" "$tmp"
name="netdiff short"
check sh -c '! "$1/netdiff" test/netdiff_short.nl test/sch_filter.expected \
	> "$2/out" && diff -u test/netdiff_short.expected "$2/out"' \
	- "$P" "$tmp"
exit $fail
