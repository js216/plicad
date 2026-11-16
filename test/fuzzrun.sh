#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# fuzzrun.sh -- Build a sanitized snapshot of PliCad and fuzz it
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/fuzzrun.sh WORK [SEED]
#
# Copies src/ and the Makefile to WORK/src_snap, builds the sanitized
# programs there, copies them to WORK/bin and runs fuzz.py on them
# until it is killed.  The build and the fuzzer run at nice 19 with 2 GB
# of memory and no swap: in a scope of freecad.slice, or under a systemd
# service (INVOCATION_ID set), in the service itself, so that the fuzzer
# dies with it.  One fuzzer at a time works in WORK: the lock on
# WORK/fuzz.lock, held until fuzz.py exits, makes a second one fail at
# once.  The tree of the project stays untouched.  WORK/fuzz.log grows
# by one line per case; test/fuzz_history.py WORK/fuzz.log summarizes
# it.

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
exec 9> "$work/fuzz.lock"
if ! flock -n 9; then
	echo "fuzzrun.sh: another fuzzer works in $work" >&2
	exit 1
fi
snap=$work/src_snap
if [ -n "${INVOCATION_ID:-}" ]; then
	cap="nice -n 19"
else
	cap="systemd-run --user --slice=freecad.slice --scope -q
	-p MemoryMax=2G -p MemorySwapMax=0 nice -n 19"
fi

rm -rf "$snap"
mkdir -p "$snap"
cp -r "$top/src" "$top/Makefile" "$snap/"
progs=$(printf 'fuzz-progs:\n\t@echo $(PROGS)\n' |
	make -s -C "$snap" -f Makefile -f - fuzz-progs)
targets=""
for p in $progs; do
	targets="$targets build/$p"
done
$cap make -s -C "$snap" -j1 $targets

rm -rf "$work/bin"
mkdir -p "$work/bin"
for p in $progs; do
	cp "$snap/build/$p" "$work/bin/"
done
echo "fuzzrun.sh: fuzzing $work/bin, log $work/fuzz.log" >&2
exec $cap /usr/bin/python3 "$here/fuzz.py" "$work/bin" "$work" "$@"
