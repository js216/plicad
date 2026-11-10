#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# fuzzd.sh -- Keep the fuzzer running until a STOP file appears
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/fuzzd.sh WORK
#
# Runs test/fuzzrun.sh WORK over and over: whenever the fuzzer exits,
# for whatever reason, the loop waits and starts it again, with a fresh
# snapshot of the tree.  A failed build or a quick exit doubles the wait,
# up to an hour; a run of an hour or more resets it to a minute.  The
# loop ends only when WORK/STOP exists: touch WORK/STOP, then kill the
# fuzzer, or wait for its next exit.  Each start and exit goes to
# WORK/fuzzd.log, and each exit also to WORK/findings.log, so a watcher
# of that file hears of it.

set -u

if [ $# -ne 1 ]; then
	echo "usage: fuzzd.sh WORK" >&2
	exit 1
fi
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$1"
work=$(cd "$1" && pwd)
wait=60

say() {
	echo "$(date +%Y-%m-%dT%H:%M:%S%z) fuzzd $*" >> "$work/fuzzd.log"
}

rm -f "$work/STOP"
while [ ! -e "$work/STOP" ]; do
	say "START"
	t0=$(date +%s)
	"$here/fuzzrun.sh" "$work"
	rc=$?
	t1=$(date +%s)
	[ -e "$work/STOP" ] && break
	say "EXIT $rc after $((t1 - t0)) s; restart in $wait s"
	echo "$(date +%Y-%m-%dT%H:%M:%S%z) fuzzd EXIT $rc after" \
		"$((t1 - t0)) s, restart in $wait s" >> "$work/findings.log"
	sleep "$wait"
	if [ $((t1 - t0)) -lt 3600 ]; then
		wait=$((wait * 2))
		[ $wait -gt 3600 ] && wait=3600
	else
		wait=60
	fi
done
say "STOP"
