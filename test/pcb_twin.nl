# SPDX-License-Identifier: GPL-3.0-only
# pcb_twin.nl -- Two alike RC filters, for the tests of a layout by groups
# Copyright (c) 2026 Jakob Kastelic
part R1 resistor value 1k swap 1,2 name a.r
part C1 capacitor value 10nF swap 1,2 name a.c
part R2 resistor value 1k swap 1,2 name b.r
part C2 capacitor value 10nF swap 1,2 name b.c
net GND global C1.2 C2.2
net a.in R1.1
net a.out C1.1 R1.2
net b.in R2.1
net b.out C2.1 R2.2
