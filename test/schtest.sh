#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# schtest.sh -- Run the tests of schdraw, schnet and netdiff
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/schtest.sh DIR
# The programs are those in DIR, and schconn (test/schconn.c), built
# there by make.  Each test/sch_*.sch must give the netlist of its
# .expected, draw without a word on stderr, and its drawing must join
# what its netlist joins (schconn); a drawing with a wire taken out, or
# with two pins traded, must not; each test/bad_sch_*.sch must fail
# with the message of its .expected; a sheet template with more room
# round bodies draws another sheet, still without a fault; a source of
# two pages draws two, each port of the sheet naming its pages, and
# prints as two; the netdiff cases compare by hand-made netlists.  It prints ok or FAIL for each
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
check sh -c 'grep -qx "part filter.buf.rf R101" "$1/good.vec" &&
	"$2/schdraw" -v -L "$3" test/sch_filter.sch 2>&1 > /dev/null |
	grep -q "^test/sch_filter.sch:[0-9]*: filter.buf.rf (R101) at "' \
	- "$tmp" "$P" "$L"

# Every part shows its designator; the ferrites of the kind the sheet
# uses most go without their value alone.
name="the common ferrites keep their designators, not their values"
check awk '$1 == "part" && NF == 3 { p = $2 } $1 == "part" && NF == 1 { p = "" }
	p ~ /^filter\.v(pos|neg)\.fb$/ && $1 == "text" && $NF ~ /^FB[0-9]+$/ { fb[p] = 1 }
	p ~ /^filter\.v(pos|neg)\.fb$/ && $1 == "text" && $NF == "600R" { bad = 1 }
	p == "filter.buf.rf" && $1 == "text" && $NF == "1K" { rf = 1 }
	END { exit !(fb["filter.vpos.fb"] && fb["filter.vneg.fb"] && !bad && rf) }' \
	"$tmp/good.vec"

# The room the drawing keeps is the sheet template's: one with more room
# round bodies draws another sheet, which keeps it.
mkdir "$tmp/wide" "$tmp/wide/symbols"
cp -r test/sheets "$tmp/wide"
awk '{ print } $1 == "page" { print "sheet wide" }' test/sch_filter.sch \
	> "$tmp/wide/w.sch"
name="schdraw keeps the room of its sheet template"
check sh -c '"$1/schdraw" -L "$2" "$3/wide/w.sch" > "$3/w.vec" 2> "$3/err" &&
	! test -s "$3/err" && ! cmp -s "$3/w.vec" "$3/good.vec"' \
	- "$P" "$L" "$tmp"

# A source of two pages: a page record of each number, by their order,
# each port of the sheet named with every page it is on, and a page of
# PostScript for each.
"$P/schdraw" -L "$L" test/sch_pages.sch > "$tmp/pages.vec"
name="schdraw draws each page from its page record"
check sh -c 'grep -q "^page [0-9]* [0-9]* 1$" "$1" &&
	grep -q "^page [0-9]* [0-9]* 2$" "$1" &&
	test "$(grep -c "^page " "$1")" = 2' - "$tmp/pages.vec"
name="schdraw names the pages of a port of the sheet, its own too"
check awk '$1 == "page" { n = $4; s = "" }
	$1 == "text" { s = s $NF } $1 != "text" && $1 != "flag" { s = "" }
	$1 == "flag" && $2 == "MID" && $NF == "port" && s == "MID(1,2)" { on[n] = 1 }
	END { exit !(on[1] && on[2]) }' "$tmp/pages.vec"
name="vecps prints a page for each page record"
check sh -c '"$1/vecps" "$2" > "$3/pages.ps" &&
	grep -q "^%%Pages: 2$" "$3/pages.ps" &&
	test "$(grep -c "^showpage" "$3/pages.ps")" = 2' \
	- "$P" "$tmp/pages.vec" "$tmp"

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
