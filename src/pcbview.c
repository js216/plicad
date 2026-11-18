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
// shown, the others keeping their order; z draws the zones filled or as
// outlines (outlines at first); Esc clears the part and the track
// selected; l shows or hides the bar of the layers at the top left, whose keys do
// as clicked and are named as the pointer passes; c, C, Space, ? and q
// are vecgl's.  The window's title is the directory of the files.  Commands come on stdin, a line each: those of vecgl, and
// "layer top", "layer bottom", "layer N", "layer silk", "layer edge",
// "layer drill", "layer all", "layer none", "courtyard", "zones fill",
// "zones outline" and "bar".
//
// The opaque layers are drawn from the top down at depths, so a pixel is
// drawn about once; of each, the clear records go first into the
// stencil, the dark ones before them where it is not set, the dark ones
// after them over all (a plane: its clearances, then the pads in them).

#include "stroke.h"
#include "util.h"
#include "vecgl.h"
#include <GL/gl.h>
#include <X11/X.h>
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
	"Esc             deselect parts and tracks, hide this help",
	"t b             top, bottom copper on and off",
	"1 ... 9         inner layer N on and off",
	"T B S M P Y E D that layer on top (shown), the rest in order",
	"! ... (         the same for inner layer 1 to 9",
	"y s e d         courtyards, silkscreen, edge, holes on and off",
	"m p             solder mask, paste on and off",
	"z               zones filled or outlined (at first outlined)",
	"a A             all layers on, all off",
	"c               center the selected part, again: zoom in",
	"C               zoom out",
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
enum {
	NAMES = 14,
	NAME_EXP = 8, // The first file's sizes from 2^-NAME_EXP mm.
	NET_LEN = 512, // Room for the text of the corner: a net and more.
	NET_NAME_LEN = 256, // Room for the name of a net, of its class, of
	CLASS_LEN = 64, // a pin, and of its number.
	PIN_LEN = 256,
	NUM_LEN = 64,
	LAYER_LEN = 64, // Room for the name of a layer, and of a key.
	KEY_NAME_LEN = 64,
	NUMS = 512, // The numbers of a record read, the most.
	TRACK_LEAST = 5, // A track's numbers, the least: its width, two points.
	BAR_MAX = 16, // The keys of the bar, the most.
	KEY_MARGIN = 5, // The margin of a key's character, either side,
	KEY_TOP = 6, // and above its capital.
	NAME_MARGIN = 8, // The margin of a key's name in its box.
	ASCII_END = 128,
	HALF_TURN = 180, // Degrees.
	QUARTER_TURN = 90
};
#define NONE_YET 1e300 // Beyond any box.
#define READ 5.0 // The capital height, in pixels, of text that can be read.
#define PI 3.14159265358979323846
#define RGB_MAX 255.0 // A component of a color, at full.
#define GRAY 0.5 // The color of a layer not known.
// KiCad's weights of red, green and blue in the brightness: of a color
// selected (its blue as KiCad has it), and of a key's color in the bar.
#define LUMA_R 0.299
#define LUMA_G 0.587
#define LUMA_B 0.117
#define LUMA_B_BAR 0.114
#define LIGHT 0.5 // A key's color this bright or less takes white text.
#define BRIGHT_LEAST 0.25 // The least factor of brightening.
#define BRIGHT_SAME 0.05 // A change of brightness this small is none,
#define DARKEN 0.8 // and the color darkened by this instead.
#define DARK_MOST 0.05 // A color this dark or darker is not brightened.
// The depths of the selected track and of the names within their
// layer's.
#define SEL_WITHIN 1.25
#define NAMES_WITHIN 1.5
#define NAME_GROUND 0.1 // The gray of the box of a key's name.
// KiCad's names of a pad: upright if this much taller than wide; the
// heights of its number and net as a share of the side across, and of a
// name alone; their offsets across from the middle; the share of the
// side along they fill, the width of a text no less than TEXT_LEAST.
#define TALL 1.2
#define NUM_HEIGHT 0.29
#define NET_HEIGHT 0.22
#define ONE_HEIGHT 0.43
#define NUM_OFF 0.22
#define NET_OFF 0.2
#define ALONG_FILL 0.85
#define TEXT_LEAST 0.1
#define TRACK_ROOM 1.25 // A segment this many times its name long takes it.
// The angles of a text upright, a little past a quarter turn up.
#define UPRIGHT_MOST 90.001
#define UPRIGHT_LEAST (-89.999)
#define PAD_SLACK 1e-6 // Of the box of a pad, a point in it.
#define JOIN_TOL 1e-4 // Ends this near meet.
#define PIN_TOL 1e-3 // A pin this near the middle of a pad is its.
#define MM_PER_MIL 0.0254
#define VIA_HOLE 0.2 // A drill hole of this radius or less is a via's.

// A track of a copper layer, as its file draws it: a line of a net out
// of any part, its points, width and the group of the tracks joined to
// it end to end, not through a pad (KiCad's connected track).
struct track {
	char net[NET_NAME_LEN], klass[CLASS_LEN]; // Its net, its class or "".
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

// The board shown: the viewer; the layers, their files; the track
// selected: its layer (or -1), its net, and the file of its lines, drawn
// brightened over its layer; whether the bar and the outline show, the
// layer key hovered, the pass of the drawing and the layer drawn; the
// color of the names drawn; and the rank above all: each Shift of a
// layer key gives its layers the next, so they go on top and the rest
// keep their order.
struct pcbv {
	struct vg vg;
	struct layer *ls;
	size_t nls;
	char **paths;
	long sel_layer;
	char sel_net[NET_LEN];
	struct vg_file sel_file;
	int bar_on, outline, hover, pass;
	const struct layer *now;
	const double *name_color;
	size_t ranks;
};
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
	char name[LAYER_LEN];
	size_t i = 0;

	vg_layer(path, name, sizeof name);
	while (i < sizeof t / sizeof *t && strcmp(name, t[i].name) != 0) {
		i++;
	}
	if (i == sizeof t / sizeof *t) {
		l->rgba[0] = l->rgba[1] = l->rgba[2] = GRAY;
		l->rgba[3] = 1;
		return;
	}
	l->key = t[i].key;
	l->rgba[0] = t[i].r / RGB_MAX;
	l->rgba[1] = t[i].g / RGB_MAX;
	l->rgba[2] = t[i].b / RGB_MAX;
	l->rgba[3] = t[i].a;
}

// The runs drawn in a pass: 0 the dark before the clear ones (in the
// stencil's holes), 1 the clear (into the stencil), 2 the dark after; a
// zone's outline only when the zones are outlines, its fill only else.
static int want(struct vg *vg, const struct vg_run *r)
{
	struct pcbv *pv = vg->user;

	size_t i = (size_t)(r - pv->now->f.run);

	if (!(r->edge ? pv->outline : !(pv->outline && r->zone))) {
		return 0;
	}
	if (pv->pass == 0) {
		return i < pv->now->c0 && r->dark;
	}
	if (pv->pass == 1) {
		return i >= pv->now->c0 && i <= pv->now->c1 && !r->dark;
	}
	return i > pv->now->c1 && r->dark;
}

static int dark(struct vg *vg, const struct vg_run *r)
{
	(void)vg;
	return r->dark;
}

// KiCad's color of a selected item: brightened by a factor that grows
// with the brightness, or where that changes little, darkened with its
// blue raised.
static void brighten(const double c[3], double s[3])
{
	double br = (c[0] * LUMA_R) + (c[1] * LUMA_G) + (c[2] * LUMA_B);
	double f = fmin(1, BRIGHT_LEAST + pow(br, 3));

	for (int i = 0; i < 3; i++) {
		s[i] = c[i] * (1 - f) + f;
	}
	if (fabs((s[0] * LUMA_R) + (s[1] * LUMA_G) + (s[2] * LUMA_B) - br) <
	    BRIGHT_SAME) {
		s[0] = c[0] * DARKEN;
		s[1] = c[1] * DARKEN;
		s[2] = c[2] * (1 - f) + f;
	}
}

// The color of a run: brightened for the selected part, a via's hole's,
// or the layer's.
static void color(struct vg *vg, const struct vg_run *r)
{
	struct pcbv *pv = vg->user;

	const double *c = r->via ? via_hole : pv->now->rgba;
	double s[3];
	double br = (c[0] * LUMA_R) + (c[1] * LUMA_G) + (c[2] * LUMA_B);

	brighten(c, s);
	vg_color(&pv->vg,
		 r->part == pv->vg.sel && pv->vg.sel >= 0 && br >= DARK_MOST ?
			 s :
			 c,
		 pv->now->rgba[3]);
}

// The color of the selected track: its layer's, brightened.
static void bright(struct vg *vg, const struct vg_run *r)
{
	struct pcbv *pv = vg->user;

	double s[3];

	(void)r;
	brighten(pv->now->rgba, s);
	vg_color(&pv->vg, s, 1);
}

static void draw_layer(struct pcbv *pv, const struct layer *l, double depth)
{
	pv->now = l;
	glDepthRange(depth, depth);
	glColorMask(0, 0, 0, 0);
	glDepthMask(GL_FALSE);
	glStencilFunc(GL_ALWAYS, 1, 1);
	pv->pass = 1;
	vg_draw(&pv->vg, &l->f, want, color);
	glColorMask(1, 1, 1, 1);
	glDepthMask(l->rgba[3] >= 1);
	glStencilFunc(GL_NOTEQUAL, 1, 1);
	pv->pass = 0;
	vg_draw(&pv->vg, &l->f, want, color);
	glStencilFunc(GL_ALWAYS, 1, 1);
	pv->pass = 2;
	vg_draw(&pv->vg, &l->f, want, color);
	if (l->c0 < l->f.nrun) { // The stencil cleared where it was set.
		glColorMask(0, 0, 0, 0);
		glDepthMask(GL_FALSE);
		glStencilFunc(GL_ALWAYS, 0, 1);
		pv->pass = 1;
		vg_draw(&pv->vg, &l->f, want, color);
		glColorMask(1, 1, 1, 1);
	}
}

// The names, at the depth of their layer, so the copper above hides
// them; each pixel takes their ink once, where the stencil is not yet
// set, so the strokes that overlap make their union.

static void name_ink(struct vg *vg, const struct vg_run *r)
{
	struct pcbv *pv = vg->user;

	(void)r;
	vg_color(&pv->vg, pv->name_color, pv->name_color[3]);
}

static void draw_names(struct pcbv *pv, const struct vg_file *f)
{
	glStencilFunc(GL_NOTEQUAL, 1, 1);
	vg_draw(&pv->vg, f, dark, name_ink);
}

// KiCad's colors of the names: of pads, white at 90%; of tracks, white
// at 70%.
static void names_of(struct pcbv *pv, const struct layer *l, double depth)
{
	static const double pad_ink[4] = { 1, 1, 1, 0.9 };
	static const double track_ink[4] = { 1, 1, 1, 0.7 };

	glDepthRange(depth, depth);
	glDepthMask(GL_FALSE);
	// The copper left the stencil set where it is dark.
	glClear(GL_STENCIL_BUFFER_BIT);
	for (int b = 0; b < NAMES; b++) {
		// The least size of the text of the file, on the screen.
		if (ldexp(1, b - NAME_EXP) * pv->vg.zoom < READ) {
			continue;
		}
		pv->name_color = pad_ink;
		draw_names(pv, &l->names->pad[b]);
		pv->name_color = track_ink;
		draw_names(pv, &l->names->track[b]);
	}
}

// The key of the layer on top: the shown one of the highest rank (the
// edge, drawn over all, aside); 0 for none.
static int top_key(struct pcbv *pv)
{
	size_t best = 0;
	int k = 0;

	for (size_t i = 0; i < pv->nls; i++) {
		if (pv->ls[i].shown && pv->ls[i].key && pv->ls[i].key != 'e' &&
		    (!k || pv->ls[i].rank > best)) {
			best = pv->ls[i].rank;
			k = pv->ls[i].key;
		}
	}
	return k;
}

// The depth of layer i by its rank, the higher nearer.
static double depth_of(struct pcbv *pv, size_t i, double within)
{
	return 1 -
	       (((double)pv->ls[i].rank + within) / (double)(pv->ranks + 2));
}

static void draw(struct vg *vg)
{
	struct pcbv *pv = vg->user;

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
	for (size_t i = pv->nls; i-- > 0;) {
		if (pv->ls[i].shown && pv->ls[i].rgba[3] >= 1 &&
		    pv->ls[i].key != 'e') {
			draw_layer(pv, &pv->ls[i], depth_of(pv, i, 1));
		}
	}
	glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
	glDisable(GL_SAMPLE_ALPHA_TO_ONE);
	for (size_t i = 0; i < pv->nls; i++) {
		if (pv->ls[i].shown && pv->ls[i].rgba[3] < 1) {
			draw_layer(pv, &pv->ls[i], depth_of(pv, i, 1));
		}
	}
	if (pv->vg.sel >= 0) {
		pv->sel_layer = -1; // A part selected since.
	}
	if (pv->sel_layer >= 0 && pv->ls[pv->sel_layer].shown) {
		glDisable(GL_STENCIL_TEST);
		glDepthRange(depth_of(pv, (size_t)pv->sel_layer, SEL_WITHIN),
			     depth_of(pv, (size_t)pv->sel_layer, SEL_WITHIN));
		glDepthMask(GL_FALSE);
		pv->now = &pv->ls[pv->sel_layer];
		vg_draw(&pv->vg, &pv->sel_file, dark, bright);
		glEnable(GL_STENCIL_TEST);
	}
	for (size_t i = 0; i < pv->nls; i++) {
		if (pv->ls[i].shown && pv->ls[i].names) {
			names_of(pv, &pv->ls[i], depth_of(pv, i, NAMES_WITHIN));
		}
	}
	glDepthMask(GL_TRUE);
	glDepthRange(0, 1);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_STENCIL_TEST);
	// The outline over all.
	for (size_t i = 0; i < pv->nls; i++) {
		if (pv->ls[i].shown && pv->ls[i].key == 'e') {
			pv->now = &pv->ls[i];
			vg_draw(&pv->vg, &pv->ls[i].f, dark, color);
		}
	}
}

// The keys of the bar, from the top down, each with its last layer (the
// one on top), and z last; returns how many.
static int bar_keys(struct pcbv *pv, int keys[BAR_MAX],
		    const struct layer *of[BAR_MAX])
{
	int n = 0;

	for (const char *o = "t123456789bsmpyed"; *o; o++) {
		for (size_t i = pv->nls; i-- > 0;) {
			if (pv->ls[i].key == *o) {
				keys[n] = (unsigned char)*o;
				of[n++] = &pv->ls[i];
				break;
			}
		}
	}
	keys[n] = 'z';
	of[n++] = NULL;
	return n;
}

// The cells of the bar's keys, in pixels: a character of the stroke font
// and margins 5 wide; as tall as vg_corner's box, the descenders in it.
#define KEY_H ((VG_CAP * 4 / 3) + 12)

static double key_w(void)
{
	return stroke_width("m", VG_CAP) + (2 * KEY_MARGIN);
}

// The key of the bar at a point, or 0.
static int bar_at(struct pcbv *pv, int x, int y)
{
	int keys[BAR_MAX];
	int n = bar_keys(pv, keys, (const struct layer *[BAR_MAX]){ 0 });
	int i = (int)(x / key_w());

	return pv->bar_on && x >= 0 && y >= 0 && y < KEY_H && i < n ? keys[i] :
								      0;
}

// The bar: a shown layer's key on its color, in black or white as the
// color is light or dark, a hidden one's plain, z on gray while the
// zones are filled, the key of the layer on top underlined; the name of
// the key under the pointer under it.
static void overlay(struct vg *vg)
{
	struct pcbv *pv = vg->user;

	static const double gray[4] = { 0.8, 0.8, 0.8, 1 };
	static const char *const names[] = {
		"t top copper (F.Cu)", "b bottom copper (B.Cu)",
		"s silkscreen",	       "m solder mask",
		"p solder paste",      "y courtyards",
		"e board edge",	       "d drill holes",
		"z zones: filled",     "Z zones: outlines",
	};
	const struct layer *of[BAR_MAX];
	int keys[BAR_MAX];
	int n = bar_keys(pv, keys, of);
	int top = top_key(pv);
	double cw = key_w();
	double ch = KEY_H;

	// The net of the selected track, in the upper right corner.
	if (pv->sel_layer >= 0) {
		vg_corner(&pv->vg, pv->sel_net);
	}

	for (int i = 0; pv->bar_on && i < n; i++) {
		const double *c = of[i] ? of[i]->rgba : gray;
		int on = of[i] ? of[i]->shown : !pv->outline;
		char k[2] = { (char)keys[i], 0 };
		char name[KEY_NAME_LEN];
		double lum =
			(c[0] * LUMA_R) + (c[1] * LUMA_G) + (c[2] * LUMA_B_BAR);

		glColor3d(1, 1, 1);
		if (on) {
			glColor3dv(c);
			glRectd(i * cw, pv->vg.h, (i + 1) * cw, pv->vg.h - ch);
			glColor3d(lum <= LIGHT, lum <= LIGHT, lum <= LIGHT);
		}
		vg_stroke((i * cw) + KEY_MARGIN, pv->vg.h - KEY_TOP - VG_CAP,
			  k);
		// The layer drawn on top, underlined under the descenders.
		if (keys[i] == top) {
			glRectd((i * cw) + 4, pv->vg.h - ch + 2,
				((i + 1) * cw) - 4, pv->vg.h - ch + 4);
		}
		if (keys[i] != pv->hover) {
			continue;
		}
		// Both fit: the names are short.
		(void)snprintf(name, sizeof name, "inner %c (In%c.Cu)", k[0],
			       k[0]);
		for (size_t j = 0; j < sizeof names / sizeof *names; j++) {
			if (names[j][0] ==
			    (keys[i] == 'z' && pv->outline ? 'Z' : k[0])) {
				(void)snprintf(name, sizeof name, "%s",
					       names[j] + 2);
			}
		}
		glColor3d(NAME_GROUND, NAME_GROUND, NAME_GROUND);
		glRectd(i * cw, pv->vg.h - ch - 2,
			(i * cw) + (2 * NAME_MARGIN) +
				stroke_width(name, VG_CAP),
			pv->vg.h - (2 * ch) - 2);
		glColor3d(1, 1, 1);
		vg_stroke((i * cw) + NAME_MARGIN,
			  pv->vg.h - ch - NAME_MARGIN - VG_CAP, name);
	}
}

// A pad of a layer's file: its box, its net and its pin's number.
struct pad {
	double b[4];
	char net[NET_NAME_LEN], num[NUM_LEN];
};

// The files of the names of one kind, of pads or of tracks: the text of
// each size as it is written, a temporary file opened when a name of the
// size comes.
struct sizes {
	FILE *out[NAMES];
};

// The text of a name, centered at (x, y), turned a degrees and upright,
// of the size s, into the stream of its size.
static void name(struct sizes *o, const char *text, double x, double y,
		 double a, double s)
{
	int b = 0;
	double r = a * PI / HALF_TURN;

	while (b < NAMES && ldexp(1, b + 1 - NAME_EXP) <= s) {
		b++; // The file of sizes from 2^(b - NAME_EXP) mm.
	}
	if (s < ldexp(1, -NAME_EXP) || b >= NAMES || !*text) {
		return;
	}
	if (!o->out[b]) {
		o->out[b] = tmpfile();
		if (!o->out[b]) {
			return;
		}
	}
	// The middle of the capitals on (x, y); an error shows as the file
	// is read back.
	(void)fprintf(o->out[b], "text %.5f %.3f 1 %.5f %.5f %s\n", s, a,
		      x + (s * sin(r) / 2), y - (s * cos(r) / 2), text);
}

// The names of a pad as KiCad lays them out: its number over its net,
// along its longer side, each as large as fits.
static void pad_names(struct sizes *o, const struct pad *p)
{
	double w = p->b[2] - p->b[0];
	double h = p->b[3] - p->b[1];
	double x = (p->b[0] + p->b[2]) / 2;
	double y = (p->b[1] + p->b[3]) / 2;
	int up = h > TALL * w;
	double along = up ? h : w;
	double across = up ? w : h;
	double a = up ? QUARTER_TURN : 0;
	double ux = up ? -1 : 0;
	double uy = up ? 0 : 1; // Across, the upper side.
	double tn = stroke_width(p->num, 1);
	double tw = stroke_width(p->net, 1);

	if (*p->num && *p->net) {
		double sn = fmin(across * NUM_HEIGHT,
				 ALONG_FILL * along / fmax(tn, TEXT_LEAST));
		double sw = fmin(across * NET_HEIGHT,
				 ALONG_FILL * along / fmax(tw, TEXT_LEAST));

		name(o, p->num, x + (ux * across * NUM_OFF),
		     y + (uy * across * NUM_OFF), a, sn);
		name(o, p->net, x - (ux * across * NET_OFF),
		     y - (uy * across * NET_OFF), a, sw);
	} else if (*p->num || *p->net) {
		const char *t = *p->num ? p->num : p->net;
		double tt = *p->num ? tn : tw;

		name(o, t, x, y, a,
		     fmin(across * ONE_HEIGHT,
			  ALONG_FILL * along / fmax(tt, TEXT_LEAST)));
	}
}

// The names of a track: its net along each of its segments long enough
// for it, its capitals half its width high, upright.
static void track_names(struct sizes *o, const char *net, const double *v,
			int n)
{
	double s = v[0] / 2;
	double tw = stroke_width(net, s);

	for (int k = 1; k + 3 < n; k += 2) {
		double dx = v[k + 2] - v[k];
		double dy = v[k + 3] - v[k + 1];
		double a = atan2(dy, dx) * HALF_TURN / PI;

		if (hypot(dx, dy) < tw * TRACK_ROOM || s <= 0) {
			continue;
		}
		if (a > UPRIGHT_MOST || a < UPRIGHT_LEAST) {
			a += a > 0 ? -HALF_TURN : HALF_TURN; // Upright.
		}
		name(o, net, (v[k] + v[k + 2]) / 2, (v[k + 1] + v[k + 3]) / 2,
		     a, s);
	}
}

// Whether (x, y) is in a pad of the layer.
static int in_pad(const struct names *nm, double x, double y)
{
	for (size_t i = 0; i < nm->npads; i++) {
		const double *b = nm->pads[i];

		if (x >= b[0] - PAD_SLACK && x <= b[2] + PAD_SLACK &&
		    y >= b[1] - PAD_SLACK && y <= b[3] + PAD_SLACK) {
			return 1;
		}
	}
	return 0;
}

// The end e (0 the first, 1 the last) of track t.
static const double *end_of(const struct track *t, int e)
{
	return &t->xy[e ? (2 * t->n) - 2 : 0];
}

// Whether an end of a meets an end of b out of the pads.
static int ends_meet(const struct names *nm, const struct track *a,
		     const struct track *b)
{
	for (int e = 0; e < 2; e++) {
		const double *p = end_of(a, e);

		for (int f = 0; f < 2; f++) {
			const double *q = end_of(b, f);

			if (hypot(p[0] - q[0], p[1] - q[1]) < JOIN_TOL &&
			    !in_pad(nm, p[0], p[1])) {
				return 1;
			}
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
				    strcmp(a->net, b->net) != 0 ||
				    !ends_meet(nm, a, b)) {
					continue;
				}
				a->group = b->group = a->group < b->group ?
							      a->group :
							      b->group;
				changed = 1;
			}
		}
	}
}

// Whether s starts with pre.
static int starts(const char *s, const char *pre)
{
	return !strncmp(s, pre, strlen(pre));
}

// What make_names keeps as it reads a layer: the names, the streams of
// the names of the pads and of the tracks, the net and its class of the
// records that follow, the pads, and whether the records are a part's.
struct reader {
	struct names *nm;
	struct sizes out[2];
	char net[NET_NAME_LEN], klass[CLASS_LEN];
	struct pad *pads;
	size_t npad;
	int in_part;
};

static void out_of_memory(void)
{
	(void)fprintf(stderr, "out of memory\n");
	exit(2);
}

static void no_temp(void)
{
	(void)fprintf(stderr, "pcbview: cannot write a temporary file\n");
	exit(2);
}

// A net record: net NAME [class CLASS].
static void net_record(struct reader *rd, char *line)
{
	char *c = line + 3 + !!line[3];

	c += strcspn(c, " ");
	*rd->klass = 0;
	if (starts(c, " class ")) {
		xsnprintf(rd->klass, "%s", c + strlen(" class "));
	}
	*c = 0;
	xsnprintf(rd->net, "%s", line[3] ? line + 4 : "");
}

// A pin record: the number of the pin to the pads it is in the middle of.
static void pin_record(struct reader *rd, const char *line)
{
	const char *r = line;
	char pin[PIN_LEN];
	double px = NAN;
	double py = NAN;

	if (scan(&r, "pin %255s %lf %lf", pin, &px, &py) != 3 ||
	    !strrchr(pin, '.')) {
		return;
	}
	for (size_t i = 0; i < rd->npad; i++) {
		const double *b = rd->pads[i].b;

		if (fabs(((b[0] + b[2]) / 2) - px) < PIN_TOL &&
		    fabs(((b[1] + b[3]) / 2) - py) < PIN_TOL) {
			xsnprintf(rd->pads[i].num, "%s", strrchr(pin, '.') + 1);
		}
	}
}

// A line of a net out of any part, its n numbers v: its names, and a
// track of it.
static void track_record(struct reader *rd, const double *v, int n)
{
	struct names *nm = rd->nm;
	struct track *t = NULL;

	track_names(&rd->out[1], rd->net, v, n);
	if (n < TRACK_LEAST) {
		return;
	}
	t = realloc(nm->tracks, (nm->ntracks + 1) * sizeof *nm->tracks);
	if (!t) {
		out_of_memory();
	}
	nm->tracks = t;
	t = &nm->tracks[nm->ntracks];
	memcpy(t->net, rd->net, sizeof t->net);
	memcpy(t->klass, rd->klass, sizeof t->klass);
	t->w = v[0];
	t->n = (n - 1) / 2;
	t->xy = malloc((size_t)(n - 1) * sizeof *t->xy);
	if (!t->xy) {
		return;
	}
	memcpy(t->xy, v + 1, (size_t)(n - 1) * sizeof *t->xy);
	t->group = nm->ntracks++;
}

// A polygon or a disk of a part, its n numbers v: a pad, its box.
static void pad_record(struct reader *rd, const double *v, int n)
{
	struct pad *q = realloc(rd->pads, (rd->npad + 1) * sizeof *rd->pads);

	if (!q) {
		out_of_memory();
	}
	rd->pads = q;
	q = &rd->pads[rd->npad++];
	memset(q, 0, sizeof *q);
	memcpy(q->net, rd->net, sizeof q->net);
	q->b[0] = q->b[1] = NONE_YET;
	q->b[2] = q->b[3] = -NONE_YET;
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

// A record of the layer.
static void record(struct reader *rd, char *line)
{
	double v[NUMS];
	int n = 0;
	char *p = line + strcspn(line, " \n");
	char *end = NULL;

	line[strcspn(line, "\n")] = 0;
	if (starts(line, "net") && (!line[3] || line[3] == ' ')) {
		net_record(rd, line);
		return;
	}
	if (starts(line, "part") && (!line[4] || line[4] == ' ')) {
		rd->in_part = line[4] == ' ';
		return;
	}
	if (starts(line, "pin")) {
		pin_record(rd, line);
		return;
	}
	if (!starts(line, "line ") && !starts(line, "poly ") &&
	    !starts(line, "disk ")) {
		return;
	}
	while (n < NUMS && (v[n] = strtod(p, &end), end != p)) {
		p = end;
		n++;
	}
	if (line[0] == 'l' && !rd->in_part && *rd->net) {
		track_record(rd, v, n);
	} else if (rd->in_part && (line[0] == 'p' || n == 3) && n >= 3) {
		pad_record(rd, v, n);
	}
}

// Reads the streams of the names into their files.
static void load_names(struct pcbv *pv, struct names *nm, struct sizes *out)
{
	for (int k = 0; k < 2; k++) {
		for (int b = 0; b < NAMES; b++) {
			FILE *f = out[k].out[b];

			if (!f) {
				continue;
			}
			if (!stream_rewind(f)) {
				no_temp();
			}
			vg_load_stream(&pv->vg, k ? &nm->track[b] : &nm->pad[b],
				       f, -1);
			(void)fclose(f);
		}
	}
}

// Makes the names of a copper layer from its file: its pads (polygons
// and disks of a part under a net), their numbers (its pin records) and
// its tracks (the lines of a net out of any part).
static struct names *make_names(struct pcbv *pv, const char *path)
{
	FILE *in = fopen(path, "r");
	char *line = NULL;
	size_t cap = 0;
	struct reader rd;

	memset(&rd, 0, sizeof rd);
	rd.nm = calloc(1, sizeof *rd.nm);
	if (!in || !rd.nm) {
		(void)fprintf(stderr, "%s: cannot read\n", path);
		exit(2);
	}
	while (get_line(&line, &cap, in) > 0) {
		record(&rd, line);
	}
	for (size_t i = 0; i < rd.npad; i++) {
		pad_names(&rd.out[0], &rd.pads[i]);
	}
	rd.nm->pads = malloc((rd.npad + 1) * sizeof *rd.nm->pads);
	for (size_t i = 0; rd.nm->pads && i < rd.npad; i++) {
		memcpy(rd.nm->pads[i], rd.pads[i].b, sizeof rd.pads[i].b);
	}
	rd.nm->npads = rd.nm->pads ? rd.npad : 0;
	group_tracks(rd.nm);
	load_names(pv, rd.nm, rd.out);
	free(rd.pads);
	free(line);
	(void)fclose(in);
	return rd.nm;
}

static int pick(struct vg *vg, double x, double y, double tol)
{
	struct pcbv *pv = vg->user;

	int best = -1;
	double area = NONE_YET;

	for (size_t i = 0; i < pv->nls; i++) {
		if (pv->ls[i].shown) {
			vg_pick(&pv->ls[i].f, x, y, tol, &best, &area);
		}
	}
	return best;
}

// Shows or hides the layers of a key, or all (a) or none (A), or draws
// the zones the other way.
static int key(struct vg *vg, KeySym k)
{
	struct pcbv *pv = vg->user;

	static const char shifted[] = "TBSMPYED!@#$%^&*(";
	static const char plain[] = "tbsmpyed123456789";
	const char *at = k < ASCII_END && k ? strchr(shifted, (int)k) : NULL;
	int any = k == 'z' || k == 'a' || k == 'A';

	// Esc clears the track and its net too.
	if (k == XK_Escape) {
		pv->sel_layer = -1;
		return 1;
	}
	// A shifted layer key: that layer on top, shown; again, back.
	if (at) {
		int lk = (unsigned char)plain[at - shifted];

		pv->ranks++;
		for (size_t i = 0; i < pv->nls; i++) {
			if (pv->ls[i].key == lk) {
				pv->ls[i].shown = 1;
				pv->ls[i].rank = pv->ranks;
			}
		}
		return 1;
	}

	for (size_t i = 0; (k == 'a' || k == 'A') && i < pv->nls; i++) {
		pv->ls[i].shown = k == 'a';
	}
	if (k == 'z') {
		pv->outline = !pv->outline;
	}
	if (k == 'l') {
		pv->bar_on = !pv->bar_on;
	}
	for (size_t i = 0; i < pv->nls; i++) {
		if (pv->ls[i].key && pv->ls[i].key == (int)k) {
			pv->ls[i].shown = !pv->ls[i].shown;
			any = 1;
		}
	}
	return any || k == 'l';
}

static int command(struct vg *vg, const char *s)
{
	struct pcbv *pv = vg->user;

	static const char *const t[] = {
		"layer top",   "t", "layer bottom", "b", "layer silk", "s",
		"layer mask",  "m", "layer paste",  "p", "layer edge", "e",
		"layer drill", "d", "courtyard",    "y", "bar",	       "l",
		"layer all",   "a", "layer none",   "A"
	};
	if (starts(s, "layer ")) {
		const char *n = s + strlen("layer ");

		if (n[0] >= '1' && n[0] <= '9' && !n[1]) {
			key(&pv->vg, (KeySym)n[0]);
		}
	} else if (starts(s, "zones ")) {
		pv->outline = s[strlen("zones ")] == 'o';
	}
	for (size_t i = 0; i < sizeof t / sizeof *t; i += 2) {
		if (!strcmp(s, t[i])) {
			key(&pv->vg, (KeySym)t[i + 1][0]);
		}
	}
	return 1;
}

// The track under a point of the board within tol, on the copper shown
// from the top down, out of the pads (a pad is its part's), and its layer
// into *layer; NULL for none.
static const struct track *track_at(struct pcbv *pv, double x, double y,
				    double tol, size_t *layer)
{
	for (size_t i = pv->nls; i-- > 0;) {
		const struct names *nm = pv->ls[i].names;

		for (size_t k = 0; nm && pv->ls[i].shown && k < nm->ntracks;
		     k++) {
			const struct track *r = &nm->tracks[k];

			for (int j = 1; j < r->n; j++) {
				const double *a = &r->xy[(2 * j) - 2];
				const double *b = &r->xy[2 * (size_t)j];
				double dx = b[0] - a[0];
				double dy = b[1] - a[1];
				double l = (dx * dx) + (dy * dy);
				double u = NAN;

				u = l > 0 ? ((x - a[0]) * dx +
					     (y - a[1]) * dy) /
						    l :
					    0;
				if (u < 0) {
					u = 0;
				} else if (u > 1) {
					u = 1;
				}
				if (hypot(x - a[0] - (u * dx),
					  y - a[1] - (u * dy)) <=
					    r->w / 2 + tol &&
				    !in_pad(nm, x, y)) {
					*layer = i;
					return r;
				}
			}
		}
	}
	return NULL;
}

// The text of the corner for track t selected: CLASS:NET (NET for a net
// of no class), the width of t, and the length of all the tracks of its
// net on the layers of copper, in mm and in mil.
static void corner_text(struct pcbv *pv, const struct track *t)
{
	double len = 0;

	for (size_t i = 0; i < pv->nls; i++) {
		for (size_t k = 0;
		     pv->ls[i].names && k < pv->ls[i].names->ntracks; k++) {
			const struct track *r = &pv->ls[i].names->tracks[k];

			for (int j = 0; !strcmp(r->net, t->net) && j + 1 < r->n;
			     j++) {
				len += hypot(r->xy[(2 * j) + 2] -
						     r->xy[2 * (size_t)j],
					     r->xy[(2 * j) + 3] -
						     r->xy[(2 * j) + 1]);
			}
		}
	}
	// A text too long for the corner is cut.
	if (snprintf(
		    pv->sel_net, sizeof pv->sel_net,
		    "%s%s%s  width %.3f mm = %.1f mil  routed %.2f mm = %.1f mil",
		    t->klass, *t->klass ? ":" : "", t->net, t->w,
		    t->w / MM_PER_MIL, len, len / MM_PER_MIL) < 0) {
		*pv->sel_net = 0;
	}
}

// Selects the connected track of track t of layer l: its lines into a
// file of their own, drawn brightened.
static void select_track(struct pcbv *pv, size_t l, const struct track *t)
{
	const struct names *nm = pv->ls[l].names;
	FILE *out = tmpfile();

	if (!out) {
		return;
	}
	// An error of a write shows as the file is read back.
	for (size_t k = 0; k < nm->ntracks; k++) {
		const struct track *r = &nm->tracks[k];

		if (r->group != t->group) {
			continue;
		}
		(void)fprintf(out, "line %.5f", r->w);
		for (int j = 0; j < 2 * r->n; j++) {
			(void)fprintf(out, " %.5f", r->xy[j]);
		}
		(void)fprintf(out, "\n");
	}
	if (!stream_rewind(out)) {
		no_temp();
	}
	vg_free(&pv->vg, &pv->sel_file);
	vg_load_stream(&pv->vg, &pv->sel_file, out, -1);
	(void)fclose(out);
	pv->sel_layer = (long)l;
	corner_text(pv, t);
}

// A click: on a key of the bar, as the key; on a track, out of the pads,
// its connected track selected, the part cleared; on neither, the track
// cleared, and the part is the viewer's to pick.
static int click(struct vg *vg, int x, int y)
{
	struct pcbv *pv = vg->user;

	int k = bar_at(pv, x, y);
	struct vg_point w = vg_point_at(&pv->vg, x, y);
	size_t l = 0;
	const struct track *t = NULL;

	if (k) {
		return key(&pv->vg, (KeySym)k);
	}
	pv->sel_layer = -1;
	t = track_at(pv, w.x, w.y, 3 / pv->vg.zoom, &l);
	if (t) {
		pv->vg.sel = -1;
		select_track(pv, l, t);
		return 1;
	}
	return 0;
}

// The pointer over the bar, or just off it: the name of its key.
static int move(struct vg *vg, int x, int y)
{
	struct pcbv *pv = vg->user;

	int k = bar_at(pv, x, y);
	int changed = k != pv->hover;

	pv->hover = k;
	return changed;
}

// Reads the file of layer i: its drawing, and for copper its names.
static void load_layer(struct pcbv *pv, size_t i)
{
	pv->vg.hair = pv->ls[i].key == 'e' || pv->ls[i].key == 'y';
	vg_load(&pv->vg, &pv->ls[i].f, pv->paths[i],
		pv->ls[i].key == 'd' ? VIA_HOLE : -1);
	pv->vg.hair = 0;
	if (pv->ls[i].key == 't' || pv->ls[i].key == 'b' ||
	    (pv->ls[i].key >= '1' && pv->ls[i].key <= '9')) {
		pv->ls[i].names = make_names(pv, pv->paths[i]);
	}
	// The first and last clear runs; with none, the runs after the
	// last are none either, so no run is drawn twice.
	pv->ls[i].c0 = pv->ls[i].f.nrun;
	pv->ls[i].c1 = pv->ls[i].f.nrun;
	for (size_t k = 0; k < pv->ls[i].f.nrun; k++) {
		if (!pv->ls[i].f.run[k].dark) {
			pv->ls[i].c0 = pv->ls[i].c0 < k ? pv->ls[i].c0 : k;
			pv->ls[i].c1 = k;
		}
	}
}

// F5: every layer read again, shown and ordered as it was.
static void reload(struct vg *vg)
{
	struct pcbv *pv = vg->user;

	for (size_t i = 0; i < pv->nls; i++) {
		vg_free(&pv->vg, &pv->ls[i].f);
		if (pv->ls[i].names) {
			for (int b = 0; b < NAMES; b++) {
				vg_free(&pv->vg, &pv->ls[i].names->pad[b]);
				vg_free(&pv->vg, &pv->ls[i].names->track[b]);
			}
			for (size_t t = 0; t < pv->ls[i].names->ntracks; t++) {
				free(pv->ls[i].names->tracks[t].xy);
			}
			free(pv->ls[i].names->tracks);
			free(pv->ls[i].names);
			pv->ls[i].names = NULL;
		}
		load_layer(pv, i);
	}
	pv->sel_layer = -1;
}

int main(int argc, char **argv)
{
	static const struct vg_viewer v = { .name = "pcbview",
					    .dir_title = 1,
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
	struct pcbv ctx;
	int a = 0;
	double box[4] = { NONE_YET, NONE_YET, -NONE_YET, -NONE_YET };

	memset(&ctx, 0, sizeof ctx);
	vg_setup(&ctx.vg, &ctx);
	ctx.sel_layer = -1;
	ctx.bar_on = 1;
	ctx.outline = 1;
	a = vg_start(&ctx.vg, argc, argv, &v);

	ctx.vg.top = (int)ceil(KEY_H); // The bar.
	ctx.vg.solid = 1;
	ctx.nls = (size_t)(argc - a);
	ctx.ranks = ctx.nls;
	ctx.ls = calloc(ctx.nls, sizeof *ctx.ls);
	ctx.paths = argv + a;
	for (size_t i = 0; ctx.ls && i < ctx.nls; i++) {
		style(ctx.paths[i], &ctx.ls[i]);
		ctx.ls[i].rank = i;
		ctx.ls[i].shown = 1;
		load_layer(&ctx, i);
		for (int j = 0; j < 4; j++) {
			box[j] = j < 2 ? fmin(box[j], ctx.ls[i].f.box[j]) :
					 fmax(box[j], ctx.ls[i].f.box[j]);
		}
	}
	vg_loop(&ctx.vg, box);
	return 0;
}
