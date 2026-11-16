#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# smdraw.sh -- Make the drawing of a part from its drawing file
# Copyright (c) 2026 Jakob Kastelic

# Usage: smdraw part.draw part.st > part.ps
# For each view record of the drawing file, smview projects the part and
# smdim adds the dimensions, at the scale of the view, in the units of
# the drawing and with the indented records under it; smsheet lays the
# views on the sheets (see smsheet.c for the drawing file).  smsheet -v
# reads the drawing file and lists the views for this script.  Without a
# scale record, the views are made at the scales 2, 1.5, 1, .75, .5,
# .375 and .25, from the largest that the paper of the drawing file
# could hold, until smsheet has a scale for each sheet.  smview draws
# the curves of a solid within .001 in on the paper at the largest
# scale the view may take: its own, that of the scale record, or 2.
# The programs are those next to this script, or in the directory
# PLICAD_BIN.
# For a PDF with the lettering of osifont, of the fonts of the library:
#   ps2pdf -sFONTPATH=$PLICAD/lib/fonts part.ps part.pdf

# Nothing it runs depends on the locale.
LC_ALL=C
export LC_ALL
P=${PLICAD_BIN:-$(dirname "$0")}
if test $# -ne 2 || test ! -r "$1" || test ! -r "$2"; then
	echo "usage: smdraw part.draw part.st > part.ps" >&2
	exit 1
fi
draw=$1
fold=$2
tmp=$(mktemp -d) || exit 1
trap 'rm -rf "$tmp"' EXIT

# Writes the view read last, with its dimensions; smview runs once for
# each view, whatever the number of scales tried.  A view with a scale
# of its own leaves a mark.  Without dims, the view is projected only.
# Passes on the messages of a program for the view record at line vln
# of the drawing file: each that names no place in the drawing file
# gets its line and its view in front, in place of the part and the
# view that smview names.
where() {
	awk -v at="$draw:$vln: view $view: " -v draw="$draw:" -v v="$view" '
		index($0, draw) == 1 { print; next }
		{ s = $0; k = index(s, ": view " v ": ")
		  if (k && index(substr(s, 1, k), " ") == 0)
			s = substr(s, k + length(v) + 9)
		  print at s }' >&2
}

flush() {
	test -n "$view" || return 0
	n=$((n + 1))
	if test ! -s "$tmp/$n.view"; then
		"$P/smview" $hidden $cut $unmarked -t "$chord" "$view" "$fold" \
			> "$tmp/$n.view" 2> "$tmp/msg"
		ok=$?
		where < "$tmp/msg"
		test $ok -eq 0 || return 1
	fi
	test "$vscale" = "$each" || : > "$tmp/$n.own"
	test -n "$dims" || return 0
	{
		cat "$tmp/$n.view"
		printf '%s' "$rules"
	} | "$P/smdim" -s "$vscale" -u "$units" 2> "$tmp/msg"
	ok=$?
	where < "$tmp/msg"
	return $ok
}

# Writes the views of the drawing file at the scale $1, as smsheet -v
# lists them in $tmp/views.
views() {
	each=$1
	view=
	n=0
	while IFS= read -r line; do
		case $line in
		view\ *)
			flush || return 1
			set -- $line
			vln=$2
			view=$3
			hidden=$4
			test "$hidden" = - && hidden=
			vscale=$each
			test "$5" = - || vscale=$5
			top=${fixed:-2}
			test "$5" = - || top=$5
			chord=$(awk -v s="$top" 'BEGIN { print 0.0254 / s }')
			cut=
			test "${6:--}" = - || cut="-c $6"
			rules=
			;;
		"#line "* | " "*)
			rules="$rules$line
"
			;;
		esac
	done < "$tmp/views"
	flush
}

# Writes into $tmp/all one round of the views for each scale of $@, the
# largest first.  A round is made once; a scale at which a view cannot
# be made, for want of room for a callout, gives none.
rounds() {
	: > "$tmp/all"
	for s in "$@"; do
		if test ! -e "$tmp/round.$s"; then
			views "$s" > "$tmp/round.$s" 2> "$tmp/err.$s" ||
				: > "$tmp/round.$s"
		fi
		cat "$tmp/round.$s" >> "$tmp/all"
	done
}

# Lays the rounds on sheets; with none, says why each scale gave none,
# or lets smsheet say what the drawing file lacks.
sheets() {
	if test ! -s "$tmp/all" && cat "$tmp"/err.* 2> /dev/null |
		grep -q .; then
		cat "$tmp"/err.* | awk '!seen[$0]++' >&2
		exit 1
	fi
	"$P/smsheet" "$draw" "$tmp/all"
}

# smsheet reads the drawing file first, and says what is wrong with it
# by its line, and with -r gives the room of its paper.
room=$("$P/smsheet" -r "$draw") || exit 1
"$P/smsheet" -v "$draw" > "$tmp/views" || exit 1
units=$(awk '$1 == "units" { print $2 ($3 == "-" ? "" : "," $3) }' \
	"$tmp/views")
fixed=$(awk '$1 == "scale" && $2 != "-" { print $2 }' "$tmp/views")
unmarked=$(awk '$1 == "unmarked" && $2 != "-" { print "-u " $2 }' \
	"$tmp/views")
dims=1
if test -n "$fixed"; then
	rounds "$fixed"
	sheets
	exit
fi

# The views are made at the scales, largest first, at which a view
# alone fits the room of the paper (smsheet -r) before its dimensions,
# and laid on the sheets after each; the first that lays them all
# gives the drawing.  If none does, the last says why.
dims=
views 1 > /dev/null || exit 1
dims=1
most=$(for f in "$tmp"/*.view; do
	test -e "$f" && test ! -e "${f%.view}.own" && grep '^box' "$f"
done | awk -v room="$room" '{ split(room, r, " ")
	w = r[1] * 25.4 / ($4 - $2); h = r[2] * 25.4 / ($5 - $3)
	m = w < h ? w : h; if (!n++ || m < most) most = m }
	END { print n ? most : 2 }')
set --
for s in 2 1.5 1 0.75 0.5 0.375 0.25; do
	if test $# -eq 0 && test $s != 0.25 &&
		awk -v s=$s -v most="$most" 'BEGIN { exit !(s > most) }'; then
		continue
	fi
	set -- "$@" $s
	rounds "$@"
	if test -s "$tmp/all" && sheets > "$tmp/ps" 2> "$tmp/why"; then
		cat "$tmp/ps"
		exit 0
	fi
done
if test -s "$tmp/all"; then
	cat "$tmp/why" >&2
else
	cat "$tmp"/err.* | awk '!seen[$0]++' >&2
fi
exit 1
