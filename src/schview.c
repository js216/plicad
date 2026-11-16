// SPDX-License-Identifier: GPL-3.0-only
// schview.c -- Show a schematic sheet in a window
// Copyright (c) 2026 Jakob Kastelic

// Usage: schview SHEET.vec
//
// Shows a sheet as vec(5), from schdraw, in an X11 window by OpenGL,
// khaki on black (vecgl.h).  It opens fitted to the sheet, as Space
// fits it; the wheel zooms round the pointer and the left button drags
// the view.  A click within 3 pixels of the box of an element of a part
// (its symbol's body, a text of it) selects the part, of the smallest
// such element, and prints "select NAME" on stdout; a click on none
// clears the selection here alone, and prints nothing.  The selected
// part is drawn in light cyan; c centers it (again: zooms in two
// times), ? shows or hides a help of the keys, and q quits.  Commands
// come on stdin, one a line: "select NAME", "select", "center" and
// "shot FILE" (the next frame as a picture).  So two viewers
// cross-probe through two FIFOs, each one's stdout the other's stdin.

#include "vecgl.h"
#include <GL/gl.h>

static const char *const help[] = {
	"left drag       move",
	"wheel           zoom",
	"click           select a part, print its name; on none, clear",
	"space           fit all",
	"F5              read the file again",
	"Esc             deselect, hide this help",
	"c               center the selected part, again: zoom in",
	"?               this help, on and off",
	"q               quit",
	NULL,
};

static struct vg_file sheet;

// Khaki (#BBBB99), light cyan (#A0FFFF) for the part selected, or black
// for what clears; their relative luminances are 0.48 and 0.86.
static void color(const struct vg_run *r)
{
	static const double khaki[3] = { 0xbb / 255.0, 0xbb / 255.0,
					 0x99 / 255.0 };
	static const double cyan[3] = { 0xa0 / 255.0, 1, 1 };
	static const double black[3] = { 0, 0, 0 };

	vg_color(!r->dark			   ? black :
		 r->part >= 0 && r->part == vg_sel ? cyan :
						     khaki,
		 1);
}

static int all(const struct vg_run *r)
{
	return r->part >= -1;
}

static void draw(void)
{
	vg_draw(&sheet, all, color);
}

static int pick(double x, double y, double tol)
{
	int best = -1;
	double area = 1e300;

	vg_pick(&sheet, x, y, tol, &best, &area);
	return best;
}

static const char *path;

static void reload(void)
{
	vg_free(&sheet);
	vg_load(&sheet, path, 0);
}

int main(int argc, char **argv)
{
	static const struct vg_viewer v = { .name = "schview",
					    .help = help,
					    .draw = draw,
					    .pick = pick,
					    .reload = reload };
	int i = vg_start(argc, argv, &v);

	vg_solid = 1; // A line thinner than a pixel stays a pixel, whole.
	vg_fill = 1; // Fitted, the frame of the sheet meets the window.
	path = argv[i];
	vg_load(&sheet, path, 0);
	vg_loop(sheet.box);
	return 0;
}
