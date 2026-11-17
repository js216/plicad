// SPDX-License-Identifier: GPL-3.0-only
// vecgl.h -- Vector files on the GPU, and the viewer round them (vgview.c)
// Copyright (c) 2026 Jakob Kastelic

// A vec(5) file is read once: a polygon or a text is cut into triangles
// (by its ears, the holes of the glyphs clear), a line or a disk into
// segments that a shader shades by their distance (a disk is a segment
// of no length).  Both go to the GPU in the order of the file, as runs
// of records alike, so a frame is a few calls and zooming or panning
// only changes the matrix.  The viewer round them (vg_start, vg_loop)
// is what schview and pcbview share: the window, the view, the mouse,
// the common keys and commands, the selection and the stdin of
// cross-probing.

#ifndef VECGL_H
#define VECGL_H

#include <X11/Xlib.h>
#include <stddef.h>
#include <stdio.h>

// A run: its triangles and segments in the buffers; dark or clear, of a
// zone's fill, the outline of a zone, a via's hole, the part (-1 none),
// the net (-1 none, vg_net), of a text record, and of a sheet's frame
// (the records from a page record to its first part or net).
struct vg_run {
	int first, count, sfirst, scount;
	int dark, zone, edge, via, part, net, text, frame;
};

// An element of a part, for picking: the box of what it draws (a body,
// a text, a pad, an outline).
struct vg_elem {
	int part;
	double b[4];
};

struct vg_file {
	unsigned buf, sbuf; // The triangles, and the segments.
	struct vg_run *run;
	struct vg_elem *elem;
	size_t nrun, nelem, ntri, nseg;
	double box[4]; // Of all it draws.
};

// What a viewer gives: its name (the window's class), the ground, its
// help, and how it draws, picks and does its own keys, commands, clicks
// and pointer moves (each returns whether it did something).
struct vg_viewer {
	const char *name;
	double bg[3];
	const char *const *help;
	void (*draw)(void);
	void (*overlay)(void);
	int (*pick)(double x, double y, double tol);
	int (*key)(KeySym k);
	int (*command)(const char *s);
	int (*click)(int x, int y);
	int (*move)(int x, int y);
	void (*reload)(void); // F5: read the files again, the view kept.
	// The window's title is the directory of the first file, not the
	// file (the layers of a board).
	int dir_title;
};

// The view: the point at the center, pixels a unit, the window, the
// part selected (-1 none), the font's cell, and the pixels at the top a
// viewer keeps for itself (the help stands under them).
extern double vg_x, vg_y, vg_zoom;
// The share of the window that fit gives the box, 0.95 unless set.
extern double vg_fill;
extern int vg_w, vg_h, vg_sel, vg_font_w, vg_font_h, vg_top;
// A line thinner than a pixel is drawn a pixel wide at full strength,
// not fainter (a board's outline).
extern int vg_solid;
// Strokes that meet take the brighter of the two, so a joint of light
// lines on a dark ground shows no seam (a sheet), rather than one laid
// over the other.
extern int vg_brighter;
// A line of width 0, or every line of a file read while vg_hair is set,
// is a hairline: 1.5 pixels at every zoom (an outline, not a thing).
extern int vg_hair;

// Opens the display and the window; returns the index of the first
// file argument.
int vg_start(int argc, char **argv, const struct vg_viewer *v);
// Reads the file into f; a disk of radius via or less is a via's hole.
void vg_load(struct vg_file *f, const char *path, double via);
// Frees what f holds, on the GPU and off it, and empties it.
void vg_free(struct vg_file *f);
// Reads a vec stream into f, as vg_load reads a file.
void vg_load_stream(struct vg_file *f, FILE *in, double via);
// The width of a text in the font (R or I) of the glyph table, at a size
// of 1 (its em), or in the font of strokes S at a capital height of 1.
double vg_text_width(char font, const char *s);
// The name of the layer a file draws, from its layer record (before any
// drawing, vec(5)), into name; "" if it has none.
void vg_layer(const char *path, char *name, size_t size);
// Fits the box into the window and runs the viewer.
void vg_loop(const double box[4]);
// Fits the view to box, which Space fits from then on (a new page).
void vg_fit(const double box[4]);
// Draws the runs of f that want takes, each in the color each sets.
void vg_draw(const struct vg_file *f, int (*want)(const struct vg_run *),
	     void (*each)(const struct vg_run *));
// The part of the element of f under (x, y), the smallest, or else of
// the nearest within tol, if it scores less than *area: it becomes *best
// and its score *area (its area, or for one near a far larger number by
// its distance).
void vg_pick(const struct vg_file *f, double x, double y, double tol, int *best,
	     double *area);
// Finds the glyph table, ../lib/fonts/nimbussans.txt from the program,
// and makes the shader of the segments (in a context of OpenGL).
void vg_init(const char *argv0);
// The box of the elements of a part in all the files read.
void vg_part_box(int part, double b[4]);
// The index of the part of the name, added if new; its name.
int vg_part(const char *s);
const char *vg_part_name(int i);
// The index of the net of the name, from its net records, added if new;
// its name.
int vg_net(const char *s);
const char *vg_net_name(int i);
// Sets the color, in sRGB and opacity; the frame blends in linear light.
void vg_color(const double c[3], double a);
// Draws text in a black box in the upper right corner of the window.
void vg_corner(const char *s);
// Titles window w by the absolute path of path (WM_NAME and
// _NET_WM_NAME), or with dir by its directory, a slash at the end.
void vg_title(Display *d, Window w, const char *path, int dir);
// Draws text from pixel (x, y) of the window, from its lower left, in
// the font fixed, in the color of glColor.
void vg_text(int x, int y, const char *s);

#endif
