# SPDX-License-Identifier: GPL-3.0-only
# sch_feedback.sch -- An inverting amplifier, its feedback over the top
# Copyright (c) 2026 Jakob Kastelic

# The feedback resistor goes between the inverting input and the
# output, above all that stands between them; a bulk capacitor stands
# under the output jack, joined to it by nothing.  A circuit made up for
# the tests.

page 3 feedback
title Inverting amplifier

opamp u LMH6609 flip
resistor rin 1k left from u.in-
resistor rfb 10k between u.in- and u.out up
ground at u.in+
bnc out BNC right from u.out
capacitor bulk 10uF down of out
supply +5V at bulk.1
ground at bulk.2
supply +5V at u.v+
supply -5V at u.v-
port IN rin.2
