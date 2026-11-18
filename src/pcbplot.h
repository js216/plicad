// SPDX-License-Identifier: GPL-3.0-only
// pcbplot.h -- What each layer of a board draws, for pcbgerber and pcbvec
// Copyright (c) 2026 Jakob Kastelic

// The layers of a laid-out board as calls of a few primitives, so the
// Gerbers and the vectors of a viewer come from one description.

#ifndef PCBPLOT_H
#define PCBPLOT_H

#include "pcbsrc.h"

// A layer file: its name without an extension, and its function in the
// terms of Gerber X2.
struct plot_file {
	const char *name;
	const char *function;
};

// A flash of a circle (rect 0) or a rectangle w by h at (x, y).
struct plot_flash {
	int rect;
	double x, y, w, h;
};

// Where a pad of a copper layer is: its pin (REF.PIN) and its center.
struct plot_pin {
	const char *name;
	double x, y;
};

// The primitives a writer gives, each with the writer's own ctx: a layer
// file begins and ends; the part and the net of what follows (NULL for
// none); the polarity; a flash; a stroke of width w; a region; and
// where a pad is, for a writer that names pins (NULL for none).
struct pcb_plot {
	void (*begin)(void *ctx, struct plot_file f);
	void (*end)(void *ctx);
	void (*part)(void *ctx, const char *name);
	void (*net)(void *ctx, const char *name);
	void (*dark)(void *ctx, int dark);
	void (*flash)(void *ctx, struct plot_flash f);
	void (*line)(void *ctx, double w, const double *xy, size_t n);
	void (*region)(void *ctx, const double *xy, size_t n);
	void (*pin)(void *ctx, struct plot_pin p);
};

// Draws each copper layer (a signal layer its pads, tracks and vias; a
// plane the outline 0.3 mm in, cleared by the clearance round what is
// not of its net), the outline, the mask and paste of the top, the
// courtyards and the silkscreen of each side, by the primitives of p
// with ctx.
void pcb_plot(const struct pcb *b, const struct pcb_plot *p, void *ctx);

#endif
