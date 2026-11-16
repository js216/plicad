# SPDX-License-Identifier: GPL-3.0-only
# eda.mk -- The build of the schematic and layout programs
# Copyright (c) 2026 Jakob Kastelic

# Usage: make -f eda.mk [O=DIR] [eda | san]
# The programs of schematics and boards keep their own build, apart from
# the Makefile's: each is its main file and the readers it needs.  They
# go into O (build/eda by default), with the helpers of the tests
# (test/xkey.c, test/schconn.c).  The target san builds the viewers into
# O/san with the address and undefined-behavior sanitizers, unstripped,
# for the viewers a user runs.

include Makefile

O ?= build/eda
EDA := schdraw schnet netdiff pcbplace pcbroute pcbcheck pcbgerber pcbvec \
	gbrvec vecps schview pcbview
HELP := xkey schconn

EFLAGS := $(CFLAGS)
ELD := $(LDFLAGS)
VSAN := -fsanitize=address,undefined -fno-omit-frame-pointer -g -O1 -rdynamic

eda: $(addprefix $(O)/,$(EDA) $(HELP))
san: $(O)/san/schview $(O)/san/pcbview

$(O)/san/%.o: src/%.c
	@mkdir -p $(O)/san
	$(CC) $(filter-out -fanalyzer,$(WARN)) $(DEFS) $(VSAN) -MMD -MP \
		-c $< -o $@

$(O)/san/schview $(O)/san/pcbview: $(O)/san/%: $(O)/san/%.o \
	$(O)/san/vgview.o $(O)/san/vecgl.o
	$(CC) $(VSAN) $^ -lX11 -lGL -lm -o $@

$(O)/%.o: src/%.c
	@mkdir -p $(O)
	$(CC) $(EFLAGS) -c $< -o $@

$(O)/schdraw $(O)/schnet: $(O)/%: $(O)/%.o $(O)/schsrc.o
	$(CC) $(ELD) $^ -lm -o $@

$(O)/pcbplace $(O)/pcbroute $(O)/pcbcheck: $(O)/%: $(O)/%.o $(O)/pcbsrc.o
	$(CC) $(ELD) $^ -lm -o $@

$(O)/pcbgerber $(O)/pcbvec: $(O)/%: $(O)/%.o $(O)/pcbplot.o $(O)/pcbsrc.o
	$(CC) $(ELD) $^ -lm -o $@

$(O)/netdiff $(O)/gbrvec $(O)/vecps: $(O)/%: $(O)/%.o
	$(CC) $(ELD) $^ -lm -o $@

$(O)/schview $(O)/pcbview: $(O)/%: $(O)/%.o $(O)/vgview.o $(O)/vecgl.o
	$(CC) $(ELD) $^ -lX11 -lGL -lm -o $@

$(O)/xkey: test/xkey.c
	@mkdir -p $(O)
	$(CC) $(filter-out -fanalyzer,$(WARN)) $< -lXtst -lXdamage -lX11 -o $@

$(O)/schconn: test/schconn.c
	@mkdir -p $(O)
	$(CC) $(filter-out -fanalyzer,$(WARN)) $< -lm -o $@

.PHONY: eda san
.DEFAULT_GOAL := eda
.PRECIOUS: $(O)/%.o $(O)/san/%.o

-include $(wildcard $(O)/*.d $(O)/san/*.d)
