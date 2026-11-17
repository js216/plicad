# SPDX-License-Identifier: GPL-3.0-only
# bad_sch_rails.sch -- Two supplies on one net
# Copyright (c) 2026 Jakob Kastelic

page bad
resistor r 1k
supply +5V at r.1
wire r.1 to r.2
supply +3V3 at r.2
