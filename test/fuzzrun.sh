#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# fuzzrun.sh -- Build a sanitized snapshot of PliCad and fuzz it
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/fuzzrun.sh WORK [SEED]
#
# Copies src/ and the Makefile to WORK/src_snap, builds the sanitized
# programs there, copies them to WORK/bin and runs fuzz.py on them until
# it is killed.  The build and the fuzzer run at nice 19 in a scope of
# freecad.slice with 2 GB of memory and no swap.  The tree of the
# project stays untouched.  WORK/fuzz.log grows by one line per case;
# test/fuzz_history.py WORK/fuzz.log summarises it.

set -eu

if [ $# -lt 1 ]; then
	echo "usage: fuzzrun.sh WORK [SEED]" >&2
	exit 1
fi
here=$(cd "$(dirname "$0")" && pwd)
top=$(dirname "$here")
mkdir -p "$1"
work=$(cd "$1" && pwd)
shift
case "$work" in
"$top" | "$top"/*)
	echo "fuzzrun.sh: WORK must lie outside $top" >&2
	exit 1
	;;
esac
snap=$work/src_snap
cap="systemd-run --user --slice=freecad.slice --scope -q -p MemoryMax=2G
	-p MemorySwapMax=0 nice -n 19"

rm -rf "$snap"
mkdir -p "$snap"
cp -r "$top/src" "$top/Makefile" "$snap/"
progs=$(printf 'fuzz-progs:\n\t@echo $(PROGS)\n' |
	make -s -C "$snap" -f Makefile -f - fuzz-progs)
targets=""
for p in $progs; do
	targets="$targets build/san/$p"
done
# Without -fanalyzer and the other warnings the build takes seconds.
$cap make -s -C "$snap" -j2 WARN=-std=c99 $targets

rm -rf "$work/bin"
mkdir -p "$work/bin"
for p in $progs; do
	cp "$snap/build/san/$p" "$work/bin/"
done
echo "fuzzrun.sh: fuzzing $work/bin, log $work/fuzz.log" >&2
exec $cap /usr/bin/python3 "$here/fuzz.py" "$work/bin" "$work" "$@"
