# SPDX-License-Identifier: GPL-3.0-only
# sch_pages.sch -- A divider over two pages, joined by a port
# Copyright (c) 2026 Jakob Kastelic

# The upper half of a divider on one page, the lower on the other; the
# port MID of the sheet is said on both, so its pins are one net.  A
# circuit made up for the tests.

page upper
title Upper half
resistor r 10K
supply +5V at r.1
port MID r.2

page lower
resistor r 4.7K
ground at r.2
port MID r.1
