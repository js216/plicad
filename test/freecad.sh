#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# freecad.sh -- Run FreeCADCmd headless, capped, with a throwaway home
# Copyright (c) 2026 Jakob Kastelic

# Usage: test/freecad.sh SCRIPT.py ARG...
# FREECAD names the FreeCADCmd program.  The FreeCAD user home is a fresh
# directory under build/, so the FreeCAD files of the user stay
# untouched, and the run is capped at 4 GB of memory, without swap.

FREECAD=${FREECAD:-/home/jk/prog/freecad/build/bin/FreeCADCmd}
mkdir -p build
home=$(mktemp -d -p "$PWD/build" fchome.XXXXXX)
systemd-run --user --slice=freecad.slice --scope -q -p MemoryMax=4G \
	-p MemorySwapMax=0 nice -n 19 env FREECAD_USER_HOME="$home" \
	PYTHONDONTWRITEBYTECODE=1 HOME="$home" XDG_CACHE_HOME="$home/cache" \
	XDG_CONFIG_HOME="$home/config" QT_QPA_PLATFORM=offscreen \
	"$FREECAD" "$@"
status=$?
rm -rf "$home"
exit $status
