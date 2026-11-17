# SPDX-License-Identifier: GPL-3.0-only
# sch_room.sch -- A feedback return alongside a decoupling branch
# Copyright (c) 2026 Jakob Kastelic

# The branch must keep its room from the return, by a longer lead, and
# the return its shape.
page room
title Room
port IN amp.in+
opamp amp LMH6609
resistor rf 1k down from amp.out
resistor rg 1k left from amp.in-
wire rf.2 to amp.in-
ground at rg.2
ferrite fbn 742863160 left from amp.v-
capacitor cn 100n right from fbn.1
supply -5V at fbn.2
ground at cn.2
ferrite fbp 742863160 left from amp.v+
capacitor cp 100n right from fbp.1
supply +5V at fbp.2
ground at cp.2
port OUT amp.out
