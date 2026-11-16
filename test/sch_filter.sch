# SPDX-License-Identifier: GPL-3.0-only
# sch_filter.sch -- A buffer with a gain of 2 into an RC low-pass
# Copyright (c) 2026 Jakob Kastelic

# The supplies are decoupled at the pins; a circuit made up for the tests.

page 7 filter
title Filtered buffer
field Rev 1

port IN buf.in

group buf
  note non-inverting gain of 2
  opamp amp LMH6609
  resistor rf 1k down from amp.out
  resistor rg 1k down from amp.in-
  wire rf.2 to amp.in- down
  ground at rg.2
  port in amp.in+
  port out amp.out
  port vp amp.v+
  port vn amp.v-
end

group vpos
  capacitor c 100nF right from buf.vp
  ferrite fb 600R up from buf.vp
  supply +12V at fb.2
  ground at c.2
end

group vneg
  capacitor c 100nF right from buf.vn
  ferrite fb 600R down from buf.vn
  supply -12V at fb.2
  ground at c.2
end

group lpf
  note one pole at 1.6 kHz
  resistor r 10k right from buf.out
  capacitor c 10nF down from r.2
  capacitor c2 10nF beside c
  ground at c.2
  ground at c2.2
  port out r.2
end

port OUT lpf.out
