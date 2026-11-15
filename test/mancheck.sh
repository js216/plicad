#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# mancheck.sh -- Check that each program has a manual page that fits it
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/mancheck.sh BIN PROG...
# Each program has man/man1/PROG.1, which groff renders without a
# warning, and whose SYNOPSIS names the program and the same options as
# its usage message.  Each page of man/man5 renders without a warning.
# It prints the first fault, and exits with 1.

S=$1
shift
fail() {
	echo "FAIL: $*"
	exit 1
}
for f in man/man1/*.1 man/man5/*.5; do
	w=$(groff -man -Tutf8 -ww -z "$f" 2>&1)
	test -z "$w" || fail "$f: $w"
done
for p in "$@"; do
	page=man/man1/$p.1
	test -f "$page" || fail "$p: no manual page"
	syn=$(groff -man -Tascii -P-cbou "$page" 2>/dev/null |
		awk '/^[A-Z]/ { on = $0 == "SYNOPSIS"; next } on')
	echo "$syn" | grep -qw "$p" || fail "$p: the SYNOPSIS names not $p"
	use=$({ "$S/$p"; "$S/$p" -%; } 2>&1 < /dev/null |
		sed -n 's/^usage: //p;s/^       //p' | sort -u)
	test -n "$use" || fail "$p: no usage message"
	a=$(echo "$use" | grep -o -- '\[*-[a-zA-Z]\b' | tr -d '[' | sort -u)
	b=$(echo "$syn" | grep -o -- ' -[a-zA-Z]\b' | tr -d ' ' | sort -u)
	test "$a" = "$b" || fail "$p: options $(echo $a) in its usage," \
		"$(echo $b) in its page"
done
