// SPDX-License-Identifier: GPL-3.0-only
// pcbview.c -- Show the layers of a board in a window
// Copyright (c) 2026 Jakob Kastelic

// Usage: pcbview LAYER.vec...
//
// Shows the vec(5) files in an X11 window by OpenGL (vecgl.h), the
// first underneath, in the colors of KiCad's default theme; each file's
// layer comes from its layer record, by KiCad's name of it (F.Cu,
// In1.Cu, F.Silkscreen, Edge.Cuts, ...), as pcbvec and gbrvec write it.  It
// opens fitted to all the layers; the wheel zooms, the left button
// drags.  A click within 3 pixels of the box of an element of a part on
// a layer shown (a pad, its silk, its courtyard) selects the part, of
// the smallest such element, prints "select NAME", and draws it
// brightened as KiCad does; a click on none clears it here alone.  The
// keys t and b show or hide the top and bottom copper, 1 to 9 the inner
// layers, y the courtyards, s the silkscreen, m the solder mask, p the
// paste, e the edge, d the holes;
// a shows all and A hides all; a layer key with Shift (T, B, S, M, P,
// Y, E, D, and ! to ( for 1 to 9) moves that layer on top of the others,
// shown, the others keeping their order; z draws the zones filled or as outlines;
// l shows or hides the bar of the layers at the top left, whose keys do
// as clicked and are named as the pointer passes; c, Space, ? and q are
// vecgl's.  Commands come on stdin, a line each: those of vecgl, and
// "layer top", "layer bottom", "layer N", "layer silk", "layer edge",
// "layer drill", "layer all", "layer none", "courtyard", "zones fill",
// "zones outline" and "bar".
//
// The opaque layers are drawn from the top down at depths, so a pixel is
// drawn about once; of each, the clear records go first into the
// stencil, the dark ones before them where it is not set, the dark ones
// after them over all (a plane: its clearances, then the pads in them).

#define _POSIX_C_SOURCE 200809L // For getline, open_memstream, fmemopen.

#include "vecgl.h"
#include <GL/gl.h>
#include <X11/keysym.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *const help[] = {
	"left drag       move",
	"wheel           zoom",
	"click           select a part, print its name; on none, clear",
	"click a track   select its connected track, its net named",
	"space           fit all",
	"F5              read the files again",
	"Esc             deselect, hide this help",
	"t b             top, bottom copper on and off",
	"1 ... 9         inner layer N on and off",
	"T B S M P Y E D that layer on top (shown), the rest in order",
	"! ... (         the same for inner layer 1 to 9",
	"y s e d         courtyards, silkscreen, edge, holes on and off",
	"m p             solder mask, paste on and off",
	"z               zones filled or outlined",
	"a A             all layers on, all off",
	"c               center the selected part, again: zoom in",
	"l               the bar of the layers, on and off",
	"bar             click a key: as the key; the pointer names it",
	"?               this help, on and off",
	"q               quit",
	NULL,
};

// The names on a layer of copper, as KiCad shows them once they can be
// read: of the pads (the number and the net) and of the tracks (the net,
// along them), each by the size of its text, a file of text for each
// size twice as large as the one before (NAMES of them, from 1/256 mm),
// so a frame draws only those large enough, and they go to the GPU once.
#define NAMES 14
#define READ 7.0 // The size of text, in pixels, that can be read.

// A track of a copper layer, as its file draws it: a line of a net out
// of any part, its points, width and the group of the tracks joined to
// it end to end, not through a pad (KiCad's connected track).
struct track {
	char net[256];
	double *xy, w;
	int n;
	size_t group;
};

struct names {
	struct vg_file pad[NAMES], track[NAMES];
	struct track *tracks;
	size_t ntracks;
	double (*pads)[4]; // The boxes of the pads.
	size_t npads;
};

// A layer: its file, shown or not, the key that shows it (0: always
// shown), its color and opacity, where its clear runs are, and its
// names (copper only).
struct layer {
	struct vg_file f;
	int shown, key;
	size_t rank; // Its place in the order of drawing, the highest on top.
	double rgba[4];
	size_t c0, c1;
	struct names *names;
};

static struct layer *ls;
static size_t nls;
// The track selected: its layer (or -1), its net, and the file of its
// lines, drawn brightened over its layer.
static long sel_layer = -1;
static char sel_net[256];
static struct vg_file sel_file;
static int bar_on = 1, outline, hover, pass;
static const struct layer *now;
static const double via_hole[3] = { 227 / 255.0, 183 / 255.0, 46 / 255.0 };

// KiCad's default colors, by the layer the file draws (its layer
// record): the key, color and opacity.  A file of no layer known is
// gray.
static void style(const char *path, struct layer *l)
{
	static const struct {
		const char *name;
		int key, r, g, b;
		double a;
	} t[] = {
		{ "F.Courtyard", 'y', 255, 38, 226, 1 },
		{ "B.Courtyard", 'y', 38, 233, 255, 1 },
		{ "F.Silkscreen", 's', 242, 237, 161, 1 },
		{ "B.Silkscreen", 's', 232, 178, 167, 1 },
		{ "F.Mask", 'm', 216, 100, 255, 0.4 },
		{ "B.Mask", 'm', 2, 255, 238, 0.4 },
		{ "F.Paste", 'p', 180, 160, 154, 0.9 },
		{ "B.Paste", 'p', 0, 194, 194, 0.9 },
		{ "F.Cu", 't', 200, 52, 52, 1 },
		{ "In1.Cu", '1', 127, 200, 127, 1 },
		{ "In2.Cu", '2', 206, 125, 44, 1 },
		{ "In3.Cu", '3', 79, 203, 203, 1 },
		{ "In4.Cu", '4', 219, 98, 139, 1 },
		{ "B.Cu", 'b', 77, 127, 196, 1 },
		{ "Edge.Cuts", 'e', 208, 210, 205, 1 },
		{ "Drill", 'd', 194, 194, 0, 1 },
	};
	char name[64];
	size_t i = 0;

	vg_layer(path, name, sizeof name);
	while (i < sizeof t / sizeof *t && strcmp(name, t[i].name)) {
		i++;
	}
	if (i == sizeof t / sizeof *t) {
		l->rgba[0] = l->rgba[1] = l->rgba[2] = 0.5;
		l->rgba[3] = 1;
		return;
	}
	l->key = t[i].key;
	l->rgba[0] = t[i].r / 255.0;
	l->rgba[1] = t[i].g / 255.0;
	l->rgba[2] = t[i].b / 255.0;
	l->rgba[3] = t[i].a;
}

// The runs drawn in a pass: 0 the dark before the clear ones (in the
// stencil's holes), 1 the clear (into the stencil), 2 the dark after; a
// zone's outline only when the zones are outlines, its fill only else.
static int want(const struct vg_run *r)
{
	size_t i = (size_t)(r - now->f.run);

	return (r->edge ? outline : !(outline && r->zone)) &&
	       (pass == 0 ? i < now->c0 && r->dark :
		pass == 1 ? i >= now->c0 && i <= now->c1 && !r->dark :
			    i > now->c1 && r->dark);
}

static int dark(const struct vg_run *r)
{
	return r->dark;
}

// KiCad's color of a selected item: brightened by a factor that grows
// with the brightness, or where that changes little, darkened with its
// blue raised.
static void brighten(const double c[3], double s[3])
{
	double br = c[0] * 0.299 + c[1] * 0.587 + c[2] * 0.117;
	double f = fmin(1, 0.25 + pow(br, 3));

	for (int i = 0; i < 3; i++) {
		s[i] = c[i] * (1 - f) + f;
	}
	if (fabs(s[0] * 0.299 + s[1] * 0.587 + s[2] * 0.117 - br) < 0.05) {
		s[0] = c[0] * 0.8;
		s[1] = c[1] * 0.8;
		s[2] = c[2] * (1 - f) + f;
	}
}

// The color of a run: brightened for the selected part, a via's hole's,
// or the layer's.
static void color(const struct vg_run *r)
{
	const double *c = r->via ? via_hole : now->rgba;
	double s[3], br = c[0] * 0.299 + c[1] * 0.587 + c[2] * 0.117;

	brighten(c, s);
	vg_color(r->part == vg_sel && vg_sel >= 0 && br >= 0.05 ? s : c,
		 now->rgba[3]);
}

// The color of the selected track: its layer's, brightened.
static void bright(const struct vg_run *r)
{
	double s[3];

	(void)r;
	brighten(now->rgba, s);
	vg_color(s, 1);
}

static void draw_layer(const struct layer *l, double depth)
{
	now = l;
	glDepthRange(depth, depth);
	glColorMask(0, 0, 0, 0);
	glDepthMask(GL_FALSE);
	glStencilFunc(GL_ALWAYS, 1, 1);
	pass = 1;
	vg_draw(&l->f, want, color);
	glColorMask(1, 1, 1, 1);
	glDepthMask(l->rgba[3] >= 1);
	glStencilFunc(GL_NOTEQUAL, 1, 1);
	pass = 0;
	vg_draw(&l->f, want, color);
	glStencilFunc(GL_ALWAYS, 1, 1);
	pass = 2;
	vg_draw(&l->f, want, color);
	if (l->c0 < l->f.nrun) { // The stencil cleared where it was set.
		glColorMask(0, 0, 0, 0);
		glDepthMask(GL_FALSE);
		glStencilFunc(GL_ALWAYS, 0, 1);
		pass = 1;
		vg_draw(&l->f, want, color);
		glColorMask(1, 1, 1, 1);
	}
}

// The names: their holes into the stencil, then the text where the
// stencil is not set, then the stencil cleared again; at the depth of
// their layer, so the copper above hides them.
static int dark_run(const struct vg_run *r)
{
	return r->dark;
}

static int clear_run(const struct vg_run *r)
{
	return !r->dark;
}

static const double *name_color;

static void name_ink(const struct vg_run *r)
{
	(void)r;
	vg_color(name_color, name_color[3]);
}

static void draw_names(const struct vg_file *f)
{
	glColorMask(0, 0, 0, 0);
	glStencilFunc(GL_ALWAYS, 1, 1);
	vg_draw(f, clear_run, name_ink);
	glColorMask(1, 1, 1, 1);
	glStencilFunc(GL_NOTEQUAL, 1, 1);
	vg_draw(f, dark_run, name_ink);
	glColorMask(0, 0, 0, 0);
	glStencilFunc(GL_ALWAYS, 0, 1);
	vg_draw(f, clear_run, name_ink);
	glColorMask(1, 1, 1, 1);
}

// KiCad's colors of the names: of pads, white at 90%; of tracks, white
// at 70%.
static void names_of(const struct layer *l, double depth)
{
	static const double pad_ink[4] = { 1, 1, 1, 0.9 };
	static const double track_ink[4] = { 1, 1, 1, 0.7 };

	glDepthRange(depth, depth);
	glDepthMask(GL_FALSE);
	for (int b = 0; b < NAMES; b++) {
		// The least size of the text of the file, on the screen.
		if (ldexp(1, b - 8) * vg_zoom < READ) {
			continue;
		}
		name_color = pad_ink;
		draw_names(&l->names->pad[b]);
		name_color = track_ink;
		draw_names(&l->names->track[b]);
	}
}

// The rank above all: each Shift of a layer key gives its layers the
// next, so they go on top and the rest keep their order.
static size_t ranks;

// The key of the layer on top: the shown one of the highest rank (the
// edge, drawn over all, aside); 0 for none.
static int top_key(void)
{
	size_t best = 0;
	int k = 0;

	for (size_t i = 0; i < nls; i++) {
		if (ls[i].shown && ls[i].key && ls[i].key != 'e' &&
		    (!k || ls[i].rank > best)) {
			best = ls[i].rank;
			k = ls[i].key;
		}
	}
	return k;
}

// The depth of layer i by its rank, the higher nearer.
static double depth_of(size_t i, double within)
{
	return 1 - ((double)ls[i].rank + within) / (double)(ranks + 2);
}

static void draw(void)
{
	glEnable(GL_STENCIL_TEST);
	glEnable(GL_DEPTH_TEST);
	// The coverage of a fragment becomes that of its samples, so the
	// stencil and the depth take the soft edge sample by sample, and a
	// sample it covers takes the whole color: strokes that overlap make
	// their union, with no seam.
	glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
	glEnable(GL_SAMPLE_ALPHA_TO_ONE);
	glDepthFunc(GL_LEQUAL);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	for (size_t i = nls; i-- > 0;) {
		if (ls[i].shown && ls[i].rgba[3] >= 1 && ls[i].key != 'e') {
			draw_layer(&ls[i], depth_of(i, 1));
		}
	}
	glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
	glDisable(GL_SAMPLE_ALPHA_TO_ONE);
	for (size_t i = 0; i < nls; i++) {
		if (ls[i].shown && ls[i].rgba[3] < 1) {
			draw_layer(&ls[i], depth_of(i, 1));
		}
	}
	if (vg_sel >= 0) {
		sel_layer = -1; // A part selected since.
	}
	if (sel_layer >= 0 && ls[sel_layer].shown) {
		glDisable(GL_STENCIL_TEST);
		glDepthRange(depth_of((size_t)sel_layer, 1.25),
			     depth_of((size_t)sel_layer, 1.25));
		glDepthMask(GL_FALSE);
		now = &ls[sel_layer];
		vg_draw(&sel_file, dark, bright);
		glEnable(GL_STENCIL_TEST);
	}
	for (size_t i = 0; i < nls; i++) {
		if (ls[i].shown && ls[i].names) {
			names_of(&ls[i], depth_of(i, 1.5));
		}
	}
	glDepthMask(GL_TRUE);
	glDepthRange(0, 1);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_STENCIL_TEST);
	// The outline over all.
	for (size_t i = 0; i < nls; i++) {
		if (ls[i].shown && ls[i].key == 'e') {
			now = &ls[i];
			vg_draw(&ls[i].f, dark, color);
		}
	}
}

// The keys of the bar, from the top down, each with its last layer (the
// one on top), and z last; returns how many.
static int bar_keys(int keys[16], const struct layer *of[16])
{
	int n = 0;

	for (const char *o = "t123456789bsmpyed"; *o; o++) {
		for (size_t i = nls; i-- > 0;) {
			if (ls[i].key == *o) {
				keys[n] = *o;
				of[n++] = &ls[i];
				break;
			}
		}
	}
	keys[n] = 'z';
	of[n++] = NULL;
	return n;
}

// The key of the bar at a point, or 0.
static int bar_at(int x, int y)
{
	int keys[16], n = bar_keys(keys, (const struct layer *[16]){ 0 });
	int i = x / (vg_font_w + 6);

	return bar_on && x >= 0 && y >= 0 && y < vg_font_h + 4 && i < n ?
		       keys[i] :
		       0;
}

// The bar: a shown layer's key on its color, in black or white as the
// color is light or dark, a hidden one's plain, z on gray while the
// zones are filled, the key of the layer on top underlined; the name of
// the key under the pointer under it.
static void overlay(void)
{
	static const double gray[4] = { 0.8, 0.8, 0.8, 1 };
	static const char *const names[] = {
		"t top copper (F.Cu)", "b bottom copper (B.Cu)",
		"s silkscreen",	       "m solder mask",
		"p solder paste",      "y courtyards",
		"e board edge",	       "d drill holes",
		"z zones: filled",     "Z zones: outlines",
	};
	const struct layer *of[16];
	int keys[16], n = bar_keys(keys, of), cw = vg_font_w + 6;
	int top = top_key();
	int ch = vg_font_h + 4;

	// The net of the selected track, in the upper right corner.
	if (sel_layer >= 0) {
		int w = (int)strlen(sel_net) * vg_font_w + 8;

		glColor3d(0.1, 0.1, 0.1);
		glRecti(vg_w - w - 4, vg_h, vg_w, vg_h - ch);
		glColor3d(1, 1, 1);
		vg_text(vg_w - w, vg_h - ch + 4, sel_net);
	}

	for (int i = 0; bar_on && i < n; i++) {
		const double *c = of[i] ? of[i]->rgba : gray;
		int on = of[i] ? of[i]->shown : !outline;
		char k[2] = { (char)keys[i], 0 }, name[64];
		double lum = c[0] * 0.299 + c[1] * 0.587 + c[2] * 0.114;

		glColor3d(1, 1, 1);
		if (on) {
			glColor3dv(c);
			glRecti(i * cw, vg_h, (i + 1) * cw, vg_h - ch);
			glColor3d(lum <= 0.5, lum <= 0.5, lum <= 0.5);
		}
		vg_text(i * cw + 3, vg_h - ch + 4, k);
		// The layer drawn on top, underlined.
		if (keys[i] == top) {
			glRecti(i * cw + 2, vg_h - ch + 3,
				i * cw + 4 + vg_font_w, vg_h - ch + 1);
		}
		if (keys[i] != hover) {
			continue;
		}
		snprintf(name, sizeof name, "inner %c (In%c.Cu)", k[0], k[0]);
		for (size_t j = 0; j < sizeof names / sizeof *names; j++) {
			if (names[j][0] ==
			    (keys[i] == 'z' && outline ? 'Z' : k[0])) {
				snprintf(name, sizeof name, "%s", names[j] + 2);
			}
		}
		glColor3d(0.1, 0.1, 0.1);
		glRecti(i * cw, vg_h - ch - 2,
			i * cw + 8 + (int)strlen(name) * vg_font_w,
			vg_h - 2 * ch - 2);
		glColor3d(1, 1, 1);
		vg_text(i * cw + 4, vg_h - 2 * ch + 2, name);
	}
}

// A pad of a layer's file: its box, its net and its pin's number.
struct pad {
	double b[4];
	char net[256], num[64];
};

// The text of a name, centered at (x, y), turned a degrees and upright,
// of the size s, into the stream of its size.
static void name(FILE **out, char **buf, size_t *len, const char *text,
		 double x, double y, double a, double s)
{
	int b = 0;
	double r = a * 3.14159265358979323846 / 180;

	while (b < NAMES && ldexp(1, b - 7) <= s) {
		b++; // The file of sizes from 2^(b - 8) mm.
	}
	if (s < ldexp(1, -8) || b >= NAMES || !*text) {
		return;
	}
	if (!out[b]) {
		out[b] = open_memstream(&buf[b], &len[b]);
		if (!out[b]) {
			return;
		}
	}
	// The middle of the em on (x, y): the baseline 0.35 em under it.
	fprintf(out[b], "text R %.5f %.3f 1 %.5f %.5f %s\n", s, a,
		x + 0.35 * s * sin(r), y - 0.35 * s * cos(r), text);
}

// The names of a pad as KiCad lays them out: its number over its net,
// along its longer side, each as large as fits.
static void pad_names(FILE **out, char **buf, size_t *len, const struct pad *p)
{
	double w = p->b[2] - p->b[0], h = p->b[3] - p->b[1];
	double x = (p->b[0] + p->b[2]) / 2, y = (p->b[1] + p->b[3]) / 2;
	int up = h > 1.2 * w;
	double along = up ? h : w, across = up ? w : h, a = up ? 90 : 0;
	double ux = up ? -1 : 0, uy = up ? 0 : 1; // Across, the upper side.
	double tn = vg_text_width('R', p->num), tw = vg_text_width('R', p->net);

	if (*p->num && *p->net) {
		double sn = fmin(across * 0.4, 0.85 * along / fmax(tn, 0.1));
		double sw = fmin(across * 0.3, 0.85 * along / fmax(tw, 0.1));

		name(out, buf, len, p->num, x + ux * across * 0.22,
		     y + uy * across * 0.22, a, sn);
		name(out, buf, len, p->net, x - ux * across * 0.2,
		     y - uy * across * 0.2, a, sw);
	} else if (*p->num || *p->net) {
		const char *t = *p->num ? p->num : p->net;
		double tt = *p->num ? tn : tw;

		name(out, buf, len, t, x, y, a,
		     fmin(across * 0.6, 0.85 * along / fmax(tt, 0.1)));
	}
}

// The names of a track: its net along each of its segments long enough
// for it, its text 0.7 of its width, upright.
static void track_names(FILE **out, char **buf, size_t *len, const char *net,
			const double *v, int n)
{
	double s = v[0] * 0.7, tw = vg_text_width('R', net) * s;

	for (int k = 1; k + 3 < n; k += 2) {
		double dx = v[k + 2] - v[k], dy = v[k + 3] - v[k + 1];
		double a = atan2(dy, dx) * 180 / 3.14159265358979323846;

		if (hypot(dx, dy) < tw * 1.25 || s <= 0) {
			continue;
		}
		if (a > 90.001 || a < -89.999) {
			a += a > 0 ? -180 : 180; // Upright.
		}
		name(out, buf, len, net, (v[k] + v[k + 2]) / 2,
		     (v[k + 1] + v[k + 3]) / 2, a, s);
	}
}

// Whether (x, y) is in a pad of the layer.
static int in_pad(const struct names *nm, double x, double y)
{
	for (size_t i = 0; i < nm->npads; i++) {
		const double *b = nm->pads[i];

		if (x >= b[0] - 1e-6 && x <= b[2] + 1e-6 && y >= b[1] - 1e-6 &&
		    y <= b[3] + 1e-6) {
			return 1;
		}
	}
	return 0;
}

// Groups the tracks of a layer as KiCad's connected track: of one net,
// joined end to end where the end is in no pad.
static void group_tracks(struct names *nm)
{
	for (int changed = 1; changed;) {
		changed = 0;
		for (size_t i = 0; i < nm->ntracks; i++) {
			struct track *a = &nm->tracks[i];

			for (size_t j = 0; j < nm->ntracks; j++) {
				struct track *b = &nm->tracks[j];

				if (a->group == b->group ||
				    strcmp(a->net, b->net)) {
					continue;
				}
				for (int e = 0; e < 2; e++) {
					const double *p =
						&a->xy[e ? 2 * a->n - 2 : 0];

					for (int f = 0; f < 2; f++) {
						const double *q =
							&b->xy[f ? 2 * b->n - 2 :
								   0];

						if (hypot(p[0] - q[0],
							  p[1] - q[1]) < 1e-4 &&
						    !in_pad(nm, p[0], p[1])) {
							size_t g =
								a->group < b->group ?
									a->group :
									b->group;

							a->group = b->group = g;
							changed = 1;
						}
					}
				}
			}
		}
	}
}

// Makes the names of a copper layer from its file: its pads (polygons
// and disks of a part under a net), their numbers (its pin records) and
// its tracks (the lines of a net out of any part).
static struct names *make_names(const char *path)
{
	FILE *in = fopen(path, "r"), *out[2][NAMES];
	char *line = NULL, *buf[2][NAMES], net[256] = "";
	size_t cap = 0, len[2][NAMES], npad = 0;
	struct pad *pads = NULL;
	struct names *nm = calloc(1, sizeof *nm);
	int in_part = 0;

	if (!in || !nm) {
		fprintf(stderr, "%s: cannot read\n", path);
		exit(2);
	}
	memset(out, 0, sizeof out);
	memset(buf, 0, sizeof buf);
	memset(len, 0, sizeof len);
	while (getline(&line, &cap, in) > 0) {
		double v[512], px, py;
		int n = 0;
		char *p = line + strcspn(line, " \n"), *end, pin[256];

		line[strcspn(line, "\n")] = 0;
		if (!strncmp(line, "net", 3) && (!line[3] || line[3] == ' ')) {
			snprintf(net, sizeof net, "%s",
				 line[3] ? line + 4 : "");
			continue;
		}
		if (!strncmp(line, "part", 4) && (!line[4] || line[4] == ' ')) {
			in_part = line[4] == ' ';
			continue;
		}
		if (sscanf(line, "pin %255s %lf %lf", pin, &px, &py) == 3) {
			for (size_t i = 0; i < npad; i++) {
				const double *b = pads[i].b;

				if (fabs((b[0] + b[2]) / 2 - px) < 1e-3 &&
				    fabs((b[1] + b[3]) / 2 - py) < 1e-3 &&
				    strrchr(pin, '.')) {
					snprintf(pads[i].num,
						 sizeof pads[i].num, "%s",
						 strrchr(pin, '.') + 1);
				}
			}
			continue;
		}
		if (strncmp(line, "line ", 5) && strncmp(line, "poly ", 5) &&
		    strncmp(line, "disk ", 5)) {
			continue;
		}
		while (n < 512 && (v[n] = strtod(p, &end), end != p)) {
			p = end;
			n++;
		}
		if (line[0] == 'l' && !in_part && *net) {
			struct track *t;

			track_names(out[1], buf[1], len[1], net, v, n);
			if (n < 5) {
				continue;
			}
			t = realloc(nm->tracks,
				    (nm->ntracks + 1) * sizeof *nm->tracks);
			if (!t) {
				fprintf(stderr, "out of memory\n");
				exit(2);
			}
			nm->tracks = t;
			t = &nm->tracks[nm->ntracks];
			snprintf(t->net, sizeof t->net, "%s", net);
			t->w = v[0];
			t->n = (n - 1) / 2;
			t->xy = malloc((size_t)(n - 1) * sizeof *t->xy);
			if (!t->xy) {
				continue;
			}
			memcpy(t->xy, v + 1, (size_t)(n - 1) * sizeof *t->xy);
			t->group = nm->ntracks++;
		} else if (in_part && (line[0] == 'p' || n == 3) && n >= 3) {
			struct pad *q;

			pads = realloc(pads, (npad + 1) * sizeof *pads);
			if (!pads) {
				fprintf(stderr, "out of memory\n");
				exit(2);
			}
			q = &pads[npad++];
			memset(q, 0, sizeof *q);
			snprintf(q->net, sizeof q->net, "%s", net);
			q->b[0] = q->b[1] = 1e300;
			q->b[2] = q->b[3] = -1e300;
			for (int k = 0; k + 1 < n && n != 3; k += 2) {
				q->b[0] = fmin(q->b[0], v[k]);
				q->b[1] = fmin(q->b[1], v[k + 1]);
				q->b[2] = fmax(q->b[2], v[k]);
				q->b[3] = fmax(q->b[3], v[k + 1]);
			}
			if (n == 3) {
				q->b[0] = v[0] - v[2];
				q->b[1] = v[1] - v[2];
				q->b[2] = v[0] + v[2];
				q->b[3] = v[1] + v[2];
			}
		}
	}
	for (size_t i = 0; i < npad; i++) {
		pad_names(out[0], buf[0], len[0], &pads[i]);
	}
	nm->pads = malloc((npad + 1) * sizeof *nm->pads);
	for (size_t i = 0; nm->pads && i < npad; i++) {
		memcpy(nm->pads[i], pads[i].b, sizeof pads[i].b);
	}
	nm->npads = nm->pads ? npad : 0;
	group_tracks(nm);
	for (int k = 0; k < 2; k++) {
		for (int b = 0; b < NAMES; b++) {
			FILE *f;

			if (!out[k][b]) {
				continue;
			}
			fclose(out[k][b]);
			f = fmemopen(buf[k][b], len[k][b], "r");
			if (f) {
				vg_load_stream(k ? &nm->track[b] : &nm->pad[b],
					       f, -1);
				fclose(f);
			}
			free(buf[k][b]);
		}
	}
	free(pads);
	free(line);
	fclose(in);
	return nm;
}

static int pick(double x, double y, double tol)
{
	int best = -1;
	double area = 1e300;

	for (size_t i = 0; i < nls; i++) {
		if (ls[i].shown) {
			vg_pick(&ls[i].f, x, y, tol, &best, &area);
		}
	}
	return best;
}

// Shows or hides the layers of a key, or all (a) or none (A), or draws
// the zones the other way.
static int key(KeySym k)
{
	static const char shifted[] = "TBSMPYED!@#$%^&*(";
	static const char plain[] = "tbsmpyed123456789";
	const char *at = k < 128 && k ? strchr(shifted, (int)k) : NULL;
	int any = k == 'z' || k == 'a' || k == 'A';

	// A shifted layer key: that layer on top, shown; again, back.
	if (at) {
		int lk = plain[at - shifted];

		ranks++;
		for (size_t i = 0; i < nls; i++) {
			if (ls[i].key == lk) {
				ls[i].shown = 1;
				ls[i].rank = ranks;
			}
		}
		return 1;
	}

	for (size_t i = 0; (k == 'a' || k == 'A') && i < nls; i++) {
		ls[i].shown = k == 'a';
	}
	outline ^= k == 'z';
	bar_on ^= k == 'l';
	for (size_t i = 0; i < nls; i++) {
		if (ls[i].key && ls[i].key == (int)k) {
			ls[i].shown = !ls[i].shown;
			any = 1;
		}
	}
	return any || k == 'l';
}

static int command(const char *s)
{
	static const char *const t[] = {
		"layer top",   "t", "layer bottom", "b", "layer silk", "s",
		"layer mask",  "m", "layer paste",  "p", "layer edge", "e",
		"layer drill", "d", "courtyard",    "y", "bar",	       "l",
		"layer all",   "a", "layer none",   "A"
	};
	if (!strncmp(s, "layer ", 6) && s[6] >= '1' && s[6] <= '9' && !s[7]) {
		key((KeySym)s[6]);
	} else if (!strncmp(s, "zones ", 6)) {
		outline = s[6] == 'o';
	}
	for (size_t i = 0; i < sizeof t / sizeof *t; i += 2) {
		if (!strcmp(s, t[i])) {
			key((KeySym)t[i + 1][0]);
		}
	}
	return 1;
}

// The track under a point of the board within tol, on the copper shown
// from the top down, out of the pads (a pad is its part's): its layer and
// index, or 0.
static int track_at(double x, double y, double tol, size_t *layer, size_t *t)
{
	for (size_t i = nls; i-- > 0;) {
		const struct names *nm = ls[i].names;

		for (size_t k = 0; nm && ls[i].shown && k < nm->ntracks; k++) {
			const struct track *r = &nm->tracks[k];

			for (int j = 1; j < r->n; j++) {
				const double *a = &r->xy[2 * j - 2],
					     *b = &r->xy[2 * j];
				double dx = b[0] - a[0], dy = b[1] - a[1];
				double l = dx * dx + dy * dy, u;

				u = l > 0 ? ((x - a[0]) * dx +
					     (y - a[1]) * dy) /
						    l :
					    0;
				u = u < 0 ? 0 : u > 1 ? 1 : u;
				if (hypot(x - a[0] - u * dx,
					  y - a[1] - u * dy) <=
					    r->w / 2 + tol &&
				    !in_pad(nm, x, y)) {
					*layer = i;
					*t = k;
					return 1;
				}
			}
		}
	}
	return 0;
}

// Selects the connected track of track t of layer l: its lines into a
// file of their own, drawn brightened.
static void select_track(size_t l, size_t t)
{
	const struct names *nm = ls[l].names;
	char *buf = NULL;
	size_t len = 0;
	FILE *out = open_memstream(&buf, &len), *in;

	if (!out) {
		return;
	}
	for (size_t k = 0; k < nm->ntracks; k++) {
		const struct track *r = &nm->tracks[k];

		if (r->group != nm->tracks[t].group) {
			continue;
		}
		fprintf(out, "line %.5f", r->w);
		for (int j = 0; j < 2 * r->n; j++) {
			fprintf(out, " %.5f", r->xy[j]);
		}
		fprintf(out, "\n");
	}
	fclose(out);
	in = fmemopen(buf, len, "r");
	vg_free(&sel_file);
	if (in) {
		vg_load_stream(&sel_file, in, -1);
		fclose(in);
	}
	free(buf);
	sel_layer = (long)l;
	snprintf(sel_net, sizeof sel_net, "%s", nm->tracks[t].net);
}

// A click: on a key of the bar, as the key; on a track, out of the pads,
// its connected track selected, the part cleared; on neither, the track
// cleared, and the part is the viewer's to pick.
static int click(int x, int y)
{
	int k = bar_at(x, y);
	double wx = vg_x + (x - vg_w / 2) / vg_zoom;
	double wy = vg_y - (y - vg_h / 2) / vg_zoom;
	size_t l, t;

	if (k) {
		return key((KeySym)k);
	}
	sel_layer = -1;
	if (track_at(wx, wy, 3 / vg_zoom, &l, &t)) {
		vg_sel = -1;
		select_track(l, t);
		return 1;
	}
	return 0;
}

// The pointer over the bar, or just off it: the name of its key.
static int move(int x, int y)
{
	int k = bar_at(x, y), changed = k != hover;

	hover = k;
	return changed;
}

static char **paths; // The files of the layers.

// Reads the file of layer i: its drawing, and for copper its names.
static void load_layer(size_t i)
{
	vg_hair = ls[i].key == 'e' || ls[i].key == 'y';
	vg_load(&ls[i].f, paths[i], ls[i].key == 'd' ? 0.2 : -1);
	vg_hair = 0;
	if (ls[i].key == 't' || ls[i].key == 'b' ||
	    (ls[i].key >= '1' && ls[i].key <= '9')) {
		ls[i].names = make_names(paths[i]);
	}
	ls[i].c0 = ls[i].f.nrun;
	ls[i].c1 = 0;
	for (size_t k = 0; k < ls[i].f.nrun; k++) {
		if (!ls[i].f.run[k].dark) {
			ls[i].c0 = ls[i].c0 < k ? ls[i].c0 : k;
			ls[i].c1 = k;
		}
	}
}

// F5: every layer read again, shown and ordered as it was.
static void reload(void)
{
	for (size_t i = 0; i < nls; i++) {
		vg_free(&ls[i].f);
		if (ls[i].names) {
			for (int b = 0; b < NAMES; b++) {
				vg_free(&ls[i].names->pad[b]);
				vg_free(&ls[i].names->track[b]);
			}
			for (size_t t = 0; t < ls[i].names->ntracks; t++) {
				free(ls[i].names->tracks[t].xy);
			}
			free(ls[i].names->tracks);
			free(ls[i].names);
			ls[i].names = NULL;
		}
		load_layer(i);
	}
	sel_layer = -1;
}

int main(int argc, char **argv)
{
	static const struct vg_viewer v = { .name = "pcbview",
					    .reload = reload,
					    .bg = { 0, 0, 0 },
					    .help = help,
					    .draw = draw,
					    .overlay = overlay,
					    .pick = pick,
					    .key = key,
					    .command = command,
					    .click = click,
					    .move = move };
	int a = vg_start(argc, argv, &v);
	double box[4] = { 1e300, 1e300, -1e300, -1e300 };

	vg_top = vg_font_h + 4; // The bar.
	vg_solid = 1;
	nls = (size_t)(argc - a);
	ranks = nls;
	ls = calloc(nls, sizeof *ls);
	paths = argv + a;
	for (size_t i = 0; ls && i < nls; i++) {
		style(paths[i], &ls[i]);
		ls[i].rank = i;
		ls[i].shown = 1;
		load_layer(i);
		for (int j = 0; j < 4; j++) {
			box[j] = j < 2 ? fmin(box[j], ls[i].f.box[j]) :
					 fmax(box[j], ls[i].f.box[j]);
		}
	}
	vg_loop(box);
	return 0;
}
