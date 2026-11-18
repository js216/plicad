// SPDX-License-Identifier: GPL-3.0-only
// vecgl.h -- Vector files on the GPU, and the viewer round them (vgview.c)
// Copyright (c) 2026 Jakob Kastelic

// A vec(5) file is read once: a polygon is cut into triangles (by its
// ears), a line, a disk or the strokes of a text into segments that a
// shader shades by their distance (a disk is a segment of no length).  Both go to the GPU in the order of the file, as runs
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
	// The triangles, and the segments: their ends, and their half widths
	// and corners.
	unsigned buf, sbuf, hbuf;
	struct vg_run *run;
	struct vg_elem *elem;
	size_t nrun, nelem, ntri, nseg;
	double box[4]; // Of all it draws.
};

struct vg;

// What a viewer gives: its name (the window's class), the ground, its
// help, and how it draws, picks and does its own keys, commands, clicks
// and pointer moves (each returns whether it did something).
struct vg_viewer {
	const char *name;
	double bg[3];
	const char *const *help;
	void (*draw)(struct vg *vg);
	void (*overlay)(struct vg *vg);
	int (*pick)(struct vg *vg, double x, double y, double tol);
	int (*key)(struct vg *vg, KeySym k);
	int (*command)(struct vg *vg, const char *s);
	int (*click)(struct vg *vg, int x, int y);
	int (*move)(struct vg *vg, int x, int y);
	void (*reload)(struct vg *vg); // F5: the files again, the view kept.
	// The window's title is the directory of the first file, not the
	// file (the layers of a board).
	int dir_title;
};

enum {
	VG_SHOT_LEN = 4096, // Room for the path of a shot.
	VG_FILES = 64 // The files kept for centering a part.
};

// A viewer: the view, the window, the files read and the drawing on the
// GPU, and what the viewer round it keeps (user).
struct vg {
	// The view: the point at the center, pixels a unit, the window, the
	// part selected (-1 none), and the pixels at the top a viewer keeps
	// for itself (the help stands under them).
	double x, y, zoom;
	// The share of the window that fit gives the box, 0.95 unless set.
	double fill;
	int w, h, sel, top;
	// A line thinner than a pixel is drawn a pixel wide at full
	// strength, not fainter (a board's outline).
	int solid;
	// Strokes that meet take the brighter of the two, so a joint of
	// light lines on a dark ground shows no seam (a sheet), rather than
	// one laid over the other.
	int brighter;
	// A line of width 0, or every line of a file read while hair is
	// set, is a hairline: 1.5 pixels at every zoom (an outline, not a
	// thing).
	int hair;
	void *user;
	// The window (vgview.c): the viewer, the display, whether it is in
	// sRGB, the help shown, a frame due to fit and to draw, a file
	// loading, the path of a shot to take, and the box Space fits.
	const struct vg_viewer *viewer;
	Display *dpy;
	Window win;
	int srgb, help, fresh, redraw, loading;
	char shot[VG_SHOT_LEN];
	double fit_box[4];
	// A drag: the button down, whether it moved, the last point and the
	// monitor of the pointer; the view a part was last centered at, and
	// the part.
	int down, moved, lx, ly, mon[4];
	double last[3];
	int last_sel;
	// The names of the parts, nets and classes of nets (vecgl.c).
	char **parts, **nets, **classes;
	size_t nparts, nnets;
	// The file being read: its triangles' points, its segments (six
	// corners a segment, each of both ends in sv and of the half width
	// and the corner in hc; nsv corners), the file, the state of its
	// records and the open run, the record's box, the element it may
	// join, and whether it is a sheet (has a page).
	float *pt, *sv, *hc;
	size_t npt, capt, nsv, capsv;
	struct vg_file *file;
	struct vg_run state, cur;
	double rb[4];
	long open_elem;
	int sheet;
	struct vg_file *files[VG_FILES]; // Those read, for centering a part.
	size_t nfiles;
	// The shader of the segments.
	unsigned prog;
	int u_view, u_zoom, u_solid, u_brighter, a_seg, a_hc;
};

// Sets up vg: the view at its first, what the viewer keeps user.
void vg_setup(struct vg *vg, void *user);
// Opens the display and the window; returns the index of the first
// file argument.
int vg_start(struct vg *vg, int argc, char **argv, const struct vg_viewer *v);
// Reads the file into f; a disk of radius via or less is a via's hole.
void vg_load(struct vg *vg, struct vg_file *f, const char *path, double via);
// Frees what f holds, on the GPU and off it, and empties it.
void vg_free(struct vg *vg, struct vg_file *f);
// Reads a vec stream into f, as vg_load reads a file.
void vg_load_stream(struct vg *vg, struct vg_file *f, FILE *in, double via);
// The name of the layer a file draws, from its layer record (before any
// drawing, vec(5)), into name; "" if it has none.
void vg_layer(const char *path, char *name, size_t size);
// A point of the drawing.
struct vg_point {
	double x, y;
};
// The point of the drawing at the pixel (px, py) of the window.
struct vg_point vg_point_at(const struct vg *vg, int px, int py);
// Fits the box into the window and runs the viewer.
void vg_loop(struct vg *vg, const double box[4]);
// Fits the view to box, which Space fits from then on (a new page).
void vg_fit(struct vg *vg, const double box[4]);
// Draws the runs of f that want takes, each in the color each sets.
void vg_draw(struct vg *vg, const struct vg_file *f,
	     int (*want)(struct vg *vg, const struct vg_run *),
	     void (*each)(struct vg *vg, const struct vg_run *));
// The part of the element of f under (x, y), the smallest, or else of
// the nearest within tol, if it scores less than *area: it becomes *best
// and its score *area (its area, or for one near a far larger number by
// its distance).
void vg_pick(const struct vg_file *f, double x, double y, double tol, int *best,
	     double *area);
// Makes the shader of the segments (in a context of OpenGL).
void vg_init(struct vg *vg);
// The box of the elements of a part in all the files read.
void vg_part_box(struct vg *vg, int part, double b[4]);
// The index of the part of the name, added if new; its name.
int vg_part(struct vg *vg, const char *s);
const char *vg_part_name(struct vg *vg, int i);
// The index of the net of the name, from its net records, added if new;
// its name.
int vg_net(struct vg *vg, const char *s);
const char *vg_net_name(struct vg *vg, int i);
// The index of the net of a net record, s the text after "net ": NAME
// [class CLASS], its class kept; cuts s at its name.  The class of a
// net, or NULL for none.
int vg_net_record(struct vg *vg, char *s);
const char *vg_net_class(struct vg *vg, int i);
// Sets the color, in sRGB and opacity; the frame blends in linear light.
void vg_color(struct vg *vg, const double c[3], double a);
// The text the viewers write over the drawing, in pixels: in the stroke
// font (stroke.h), its capitals VG_CAP high, its pen VG_PEN of them
// wide, whatever the zoom.
#define VG_CAP 14.0
#define VG_PEN 0.11
// Draws text in the stroke font from pixel (x, y) of the window, the
// left end of its baseline, from its lower left, in the color of glColor.
void vg_stroke(double x, double y, const char *s);
// Draws text in the stroke font in a black box in the upper right
// corner of the window.
void vg_corner(struct vg *vg, const char *s);
// Titles window w by path as given (WM_NAME and _NET_WM_NAME), or with
// dir by its directory, a slash at the end (./ for none).
void vg_title(Display *d, Window w, const char *path, int dir);

#endif
