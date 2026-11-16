# SPDX-License-Identifier: GPL-3.0-only
# netdiff_same.nl -- The buffer of sch_filter.sch, written by hand
# Copyright (c) 2026 Jakob Kastelic

# Other designators, names and order, other forms of the values, and the
# passive parts turned round: netdiff finds no difference.

part X1 ti/lmh6609 value LMH6609
part RA resistor value 1.00K swap 1,2
part RB resistor value 1000 swap 1,2
part RC resistor value 10.0K swap 1,2
part CA capacitor value 0.1u swap 1,2
part CB capacitor value 100n swap 1,2
part CC capacitor value 0.01uF swap 1,2
part CD capacitor value 10n swap 1,2
part F1 ferrite value 600 swap 1,2
part F2 ferrite value 600R swap 1,2
net GND global CA.1 CB.1 CC.2 CD.1 RB.1
net +12V global F1.1
net -12V global F2.2
net IN global X1.3
net OUT global CC.1 CD.2 RC.1
net n1 RB.2 RA.1 X1.2
net n2 RA.2 RC.2 X1.6
net n3 CA.2 F1.2 X1.7
net n4 CB.2 F2.1 X1.4
