// SPDX-License-Identifier: GPL-3.0-only
// pcbplot.h -- What each layer of a board draws, for pcbgerber and pcbvec
// Copyright (c) 2026 Jakob Kastelic

// The layers of a laid-out board as calls of a few primitives, so the
// Gerbers and the vectors of a viewer come from one description.

#ifndef PCBPLOT_H
#define PCBPLOT_H

#include "pcbsrc.h"

// The primitives a writer gives: a layer file begins (its name without
// an extension, and its function in the terms of Gerber X2) and ends;
// the part and the net of what follows (NULL for none); the polarity; a
// flash of a circle (rect 0) or rectangle w by h at (x, y); a stroke of
// width w; a region; and where a pad of a copper layer is, by its pin
// (REF.PIN), for a writer that names pins (NULL for none).
struct pcb_plot {
	void (*begin)(const char *name, const char *function);
	void (*end)(void);
	void (*part)(const char *name);
	void (*net)(const char *name);
	void (*dark)(int dark);
	void (*flash)(int rect, double x, double y, double w, double h);
	void (*line)(double w, const double *xy, size_t n);
	void (*region)(const double *xy, size_t n);
	void (*pin)(const char *name, double x, double y);
};

// Draws each copper layer (a signal layer its pads, tracks and vias; a
// plane the outline 0.3 mm in, cleared by the clearance round what is
// not of its net), the outline, the mask and paste of the top, the
// courtyards and the silkscreen of each side.
void pcb_plot(const struct pcb *b, const struct pcb_plot *p);

#endif
