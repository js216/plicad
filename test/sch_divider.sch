# SPDX-License-Identifier: GPL-3.0-only
# sch_divider.sch -- A divider with a filter capacitor and a clamp
# Copyright (c) 2026 Jakob Kastelic

# A circuit made up for the tests, without groups; the line goes on
# down before its branches are said.

page divider
title Divider

resistor top 10k
resistor bottom 4.7k down from top.2
capacitor c 1uF right from top.2
diode clamp right from top.2
ground at bottom.2
ground at c.2
supply 3V3 at clamp.2
supply +5V at top.1
