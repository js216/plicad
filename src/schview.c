// SPDX-License-Identifier: GPL-3.0-only
// schview.c -- Show a schematic sheet in a window
// Copyright (c) 2026 Jakob Kastelic

// Usage: schview SHEET.vec
//
// Shows the pages of a schematic as vec(5), from schdraw, one at a time
// (each starts at its page record; PgDn and PgUp turn them, g and a page
// number and Enter goes to it, a click on a sheet port's circle or name
// goes to the first other page it lists, on a number of its list to
// that page), in an X11 window by OpenGL,
// on black, its text and its frame khaki, the rest a whiter gray
// (vecgl.h).  It opens fitted to the sheet, as Space
// fits it; the wheel zooms round the pointer and the left button drags
// the view.  A click within 3 pixels of the box of an element of a part
// (its symbol's body, a text of it) selects the part, of the smallest
// such element, and prints "select NAME" on stdout; a click on none
// clears the selection here alone, and prints nothing.  A click on a
// wire, a dot or a pin's end out of any part, or on a global's mark or
// name, selects the net, drawn light cyan with its flags, its name in
// the upper right corner, after its class and a colon if it has one
// (power:outputs.ch1.gain.vp).  A note is a khaki box with an N; the pointer
// over it opens it: its text shows, wrapped, at the upper left, and its
// badge and its group's parts are lit magenta.  The texts over the
// drawing and the help are in the stroke font, at one size whatever
// the zoom.  The selected part is drawn in light cyan, and its page
// shows when it is newly selected; c centers it (again: zooms in two
// times), C zooms out two times, F5 reads the file again (Loading... in
// the upper right meanwhile), ? shows or hides a help of the keys, and
// q quits.  The
// window's title is the absolute path of the file.  Commands come on
// stdin, one a line: "select NAME", "select", "center",
// "shot FILE" (the next frame as a picture), and "page next", "page
// prev" and "page N".  So two viewers
// cross-probe through two FIFOs, each one's stdout the other's stdin.

#include "stroke.h"
#include "util.h"
#include "vecgl.h"
#include <GL/gl.h>
#include <X11/X.h>
#include <X11/keysym.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	DECIMAL = 10, // The base of decimal numbers.
	SPAN_LEN = 64, // Room for the text of a span, the name of a flag's
	FLAG_NET_LEN = 128, // net, a note's group and a flag's kind.
	GROUP_LEN = 512,
	KIND_LEN = 16,
	SPANS = 32, // The spans of a flag, the most.
	LINE_LEN = 4096, // Room for a line of a page.
	RECORD_NUMS = 8, // Room for the numbers of a record,
	WIRE_FIELDS = 5, // and those of a wire read: its width, two points.
	NOTE_LINES = 16, // The lines of a note's text shown, the most, and
	NOTE_LINE_LEN = 80, // room for one.
	WRAP = 60, // A note's text wraps at a space at about WRAP
	WRAP_LEAST = 20, // characters, no fewer than WRAP_LEAST.
	NOTE_SLACK =
		24, // A line of a note is narrower than the window by this.
	PAD_X = 8, // The margin of text in a box, left of it,
	PAD_Y = 6, // and above its capitals.
	GO_LEN = 64 // Room for the page number typed.
};
#define LEAD (VG_CAP * 5 / 3) // The lines of text a capital and 2/3 apart.
#define FAR 1e300 // Past any box.
// The box of a span's text, below and above its baseline, as a share of
// its size.
#define BELOW 0.25
#define ABOVE 0.75
#define PIN_TOL 1e-3 // A wire's end this near a pin's meets it.
#define TYPING_GRAY 0.8 // The page number typed.

// Khaki (#BBBB99) for the text, the frame and the notes; magenta
// (#FF80FF) for the note open and its group.
static const double khaki[3] = { 0xbb / 255.0, 0xbb / 255.0, 0x99 / 255.0 };
static const double magenta[3] = { 1, 0x80 / 255.0, 1 };

static const char *const help[] = {
	"left drag       move",
	"wheel           zoom",
	"click           select a part, print its name; on none, clear",
	"click a wire    select its net (a dot, a pin's end, a global too)",
	"space           fit all",
	"PgDn PgUp       next, previous page",
	"g N Enter       go to page N",
	"click a port    the first other page it lists; a number, that page",
	"F5              read the file again",
	"Esc             deselect the part and the net, hide this help",
	"c               center the selected part, again: zoom in",
	"C               zoom out",
	"?               this help, on and off",
	"q               quit",
	NULL,
};

// A span of the text of a flag: its text and box.
struct span {
	char t[SPAN_LEN];
	double b[4];
};

// The flags of the pages (vec(5) flag NET X Y port|global): the name of
// its net, the end of its pin, whether a port of the sheet, the size of
// its text, and its text's spans (a port's name, then each piece of its
// list of pages), by page.
struct flag {
	char net[FLAG_NET_LEN];
	double x, y, size;
	int port, nspan;
	struct span span[SPANS];
	size_t page;
};

// What a click selects a net by: a wire, a dot or a pin's end, of the
// net (vg_net), by page; a pin's end is a point, a dot a disk.
struct hit {
	double a[2], b[2], r;
	int net;
	size_t page;
};

// The notes (vec(5) note X Y GROUP TEXT): the middle of the box, its
// group (- outside any), its text and page.
#define NOTE_BOX 10.0
struct note {
	double x, y;
	char group[GROUP_LEN], *text;
	size_t page;
};

// The sheet shown: the viewer; its file; the pages, each read from its
// page record to the next, the one shown, its number (that of its page
// record, or else its place), and a page number being typed after g;
// the flags; what a click selects a net by, and the net selected (-1
// none); the notes, the boxes of each page drawn as a file of their
// own, the note under the pointer (-1 none), and the badge of the note
// under the pointer (of note badge_of), made anew when it changes; the
// part selected that the page was last turned to.
struct schv {
	struct vg vg;
	const char *path;
	struct vg_file **pages;
	int *numbers;
	size_t npages, now;
	int typing;
	struct flag *flags;
	size_t nflags;
	struct hit *hits;
	size_t nhits;
	int sel_net;
	struct note *notes;
	size_t nnotes;
	struct vg_file **note_files;
	long hover;
	struct vg_file *open_file;
	long badge_of;
	int shown;
};

// Whether part k is of the group of the note under the pointer: its
// name the group, or the group and a dot before the rest.
static int in_hover_group(struct schv *sv, int k)
{
	const char *g = NULL;
	const char *name = NULL;
	size_t n = 0;

	if (sv->hover < 0 || k < 0) {
		return 0;
	}
	g = sv->notes[sv->hover].group;
	name = vg_part_name(&sv->vg, k);
	n = strlen(g);
	return strcmp(g, "-") != 0 && !strncmp(name, g, n) &&
	       (!name[n] || name[n] == '.');
}

// The parts of the group of the note under the pointer magenta
// (#FF80FF); else the text
// rest of the drawing a whiter gray (#D4D4D0), light cyan (#A0FFFF) for
// the part or the net selected (its wires, dots and flags), or black
// for what clears.
static void color(struct vg *vg, const struct vg_run *r)
{
	struct schv *sv = vg->user;

	static const double gray[3] = { 0xd4 / 255.0, 0xd4 / 255.0,
					0xd0 / 255.0 };
	static const double cyan[3] = { 0xa0 / 255.0, 1, 1 };
	static const double black[3] = { 0, 0, 0 };
	const double *c = gray;

	if (!r->dark) {
		c = black;
	} else if (in_hover_group(sv, r->part)) {
		c = magenta;
	} else if ((r->part >= 0 && r->part == sv->vg.sel) ||
		   (r->net >= 0 && r->net == sv->sel_net)) {
		c = cyan;
	} else if (r->text || r->frame) {
		c = khaki;
	}
	vg_color(&sv->vg, c, 1);
}

static int all(struct vg *vg, const struct vg_run *r)
{
	(void)vg;
	return r->part >= -1;
}

// The page holding part k, or now.
static size_t page_of(struct schv *sv, int k)
{
	for (size_t p = 0; k >= 0 && p < sv->npages; p++) {
		for (size_t e = 0; e < sv->pages[p]->nelem; e++) {
			if (sv->pages[p]->elem[e].part == k) {
				return p;
			}
		}
	}
	return sv->now;
}

// The page after the one shown, or before it (none before the first).
static size_t next_page(const struct schv *sv, int forward)
{
	if (forward) {
		return sv->now + 1;
	}
	return sv->now ? sv->now - 1 : sv->now;
}

static void turn(struct schv *sv, size_t p)
{
	if (p < sv->npages && p != sv->now) {
		sv->now = p;
		vg_fit(&sv->vg, sv->pages[sv->now]->box);
	}
}

static void note_color(struct vg *vg, const struct vg_run *r)
{
	struct schv *sv = vg->user;

	(void)r;
	vg_color(&sv->vg, khaki, 1);
}

// The badge of the note open, magenta as the parts it lights.
static void open_color(struct vg *vg, const struct vg_run *r)
{
	struct schv *sv = vg->user;

	(void)r;
	vg_color(&sv->vg, magenta, 1);
}

static void badge(FILE *out, const struct note *nt);

// Closes a stream of text written, or exits if a write failed.
static void close_text(FILE *out)
{
	int failed = ferror(out);

	if (fclose(out) || failed) {
		exit(2);
	}
}

// p grown to n bytes, or an exit.
static void *grow(void *p, size_t n)
{
	void *q = realloc(p, n);

	if (!q) {
		exit(2);
	}
	return q;
}

static void open_badge(struct schv *sv)
{
	struct vg_file *f = sv->open_file;
	char *text = NULL;
	size_t len = 0;
	FILE *out = NULL;
	FILE *in = NULL;

	if (sv->hover != sv->badge_of || !f) {
		if (f) {
			vg_free(&sv->vg, f);
			free(f);
			f = sv->open_file = NULL;
		}
		sv->badge_of = sv->hover;
		if (sv->hover < 0) {
			return;
		}
		out = open_memstream(&text, &len);
		if (!out) {
			exit(2);
		}
		badge(out, &sv->notes[sv->hover]);
		close_text(out);
		in = fmemopen(text, len, "r");
		f = calloc(1, sizeof *f);
		if (!in || !f) {
			exit(2);
		}
		vg_load_stream(&sv->vg, f, in, 0);
		(void)fclose(in);
		free(text);
		sv->open_file = f;
	}
	if (sv->hover >= 0) {
		vg_draw(&sv->vg, f, all, open_color);
	}
}

static void draw(struct vg *vg)
{
	struct schv *sv = vg->user;

	// A part newly selected, from outside or here, shows its page; the
	// pages turn freely after.
	if (sv->vg.sel != sv->shown) {
		sv->shown = sv->vg.sel;
		turn(sv, page_of(sv, sv->vg.sel));
	}
	vg_draw(&sv->vg, sv->pages[sv->now], all, color);
	if (sv->note_files[sv->now]) {
		vg_draw(&sv->vg, sv->note_files[sv->now], all, note_color);
	}
	open_badge(sv);
}

// The text of the note open, in the stroke font in a black box at the
// upper left of the window, under the bar, wrapped at about 60
// characters and inside the window.
static void note_text(struct schv *sv, const char *t)
{
	char line[NOTE_LINES][NOTE_LINE_LEN];
	int n = 0;
	double w = 0;
	double lh = LEAD;
	double room = sv->vg.w - NOTE_SLACK;
	double y = NAN;

	while (*t && n < NOTE_LINES) {
		size_t len = strlen(t);
		size_t cut = len;
		char *sp = NULL;

		if (len > WRAP) {
			cut = WRAP;
			while (cut > WRAP_LEAST && t[cut] != ' ') {
				cut--;
			}
			if (t[cut] != ' ') {
				cut = WRAP;
			}
		}
		// It fits: cut is WRAP at most.
		(void)snprintf(line[n], sizeof line[n], "%.*s", (int)cut, t);
		// Shorter, at a space, while wider than the window.
		while (stroke_width(line[n], VG_CAP) > room &&
		       (sp = strrchr(line[n], ' '))) {
			*sp = 0;
		}
		cut = strlen(line[n]);
		w = fmax(w, stroke_width(line[n], VG_CAP));
		n++;
		t += cut;
		t += strspn(t, " ");
	}
	y = sv->vg.h - sv->vg.top;
	glColor3d(0, 0, 0);
	glRectd(0, y, w + (2 * PAD_X), y - (n * lh) - (2 * PAD_Y));
	glColor3d(1, 1, 1);
	for (int k = 0; k < n; k++) {
		vg_stroke(PAD_X, y - PAD_Y - VG_CAP - (k * lh), line[k]);
	}
}

// The page number being typed after g, at the top left, and the name of
// the net selected at the top right.
static void overlay(struct vg *vg)
{
	struct schv *sv = vg->user;

	static const double typing_gray[3] = { TYPING_GRAY, TYPING_GRAY,
					       TYPING_GRAY };
	char t[GO_LEN];

	if (sv->typing >= 0) {
		// It fits: the text and an int.
		(void)snprintf(t, sizeof t, "go to:");
		if (sv->typing > 0) {
			(void)snprintf(t + strlen(t), sizeof t - strlen(t),
				       " %d", sv->typing);
		}
		vg_color(&sv->vg, typing_gray, 1);
		vg_stroke(PAD_X, sv->vg.h - PAD_Y - VG_CAP, t);
	}
	if (sv->sel_net >= 0) {
		const char *c = vg_net_class(&sv->vg, sv->sel_net);
		size_t size = strlen(vg_net_name(&sv->vg, sv->sel_net)) +
			      (c ? strlen(c) + 1 : 0) + 1;
		char *name = malloc(size);

		// CLASS:NET, or NET for a net of no class; it fits, the room
		// made for it.
		if (name) {
			(void)snprintf(name, size, "%s%s%s", c ? c : "",
				       c ? ":" : "",
				       vg_net_name(&sv->vg, sv->sel_net));
			vg_corner(&sv->vg, name);
			free(name);
		}
	}
	if (sv->hover >= 0) {
		note_text(sv, sv->notes[sv->hover].text);
	}
}

// The commands of stdin past vecgl's: "page next", "page prev" and
// "page N".
static int command(struct vg *vg, const char *c)
{
	struct schv *sv = vg->user;

	int n = 0;
	const char *r = c;

	if (!strcmp(c, "page next") || !strcmp(c, "page prev")) {
		turn(sv, next_page(sv, c[strlen("page ")] == 'n'));
	} else if (scan(&r, "page %d", &n) == 1) {
		for (size_t p = 0; p < sv->npages; p++) {
			if (sv->numbers[p] == n) {
				turn(sv, p);
			}
		}
	}
	return 1;
}

// Shows page p, at the zoom as it was, centered on its port of the name.
static void go_to(struct schv *sv, size_t p, const char *net)
{
	double z = sv->vg.zoom;

	turn(sv, p);
	sv->vg.zoom = z;
	for (size_t j = 0; j < sv->nflags; j++) {
		if (sv->flags[j].page == p && sv->flags[j].port &&
		    !strcmp(sv->flags[j].net, net)) {
			sv->vg.x = sv->flags[j].x;
			sv->vg.y = sv->flags[j].y;
			return;
		}
	}
}

// The page of a number, or npages.
static size_t page_numbered(struct schv *sv, int n)
{
	for (size_t p = 0; p < sv->npages; p++) {
		if (sv->numbers[p] == n) {
			return p;
		}
	}
	return sv->npages;
}

// The first page a port lists other than the one shown, or npages.
static size_t first_other(struct schv *sv, const struct flag *f)
{
	for (int k = 1; k < f->nspan; k++) {
		if (isdigit((unsigned char)f->span[k].t[0]) &&
		    (int)strtol(f->span[k].t, NULL, DECIMAL) !=
			    sv->numbers[sv->now]) {
			return page_numbered(sv, (int)strtol(f->span[k].t, NULL,
							     DECIMAL));
		}
	}
	return sv->npages;
}

static int in_box(const double b[4], double x, double y, double tol)
{
	return x >= b[0] - tol && x <= b[2] + tol && y >= b[1] - tol &&
	       y <= b[3] + tol;
}

// A click on a flag: on a port's circle or name, the first other page
// it lists, centered on its port there at the zoom as it was; on a
// number of its list, that page (its own, or a bracket or a comma,
// nothing); on a global's mark or name, its net selected.  Returns 0
// if the click is on none.
static int flag_click(struct schv *sv, double x, double y, double tol)
{
	for (size_t i = 0; i < sv->nflags; i++) {
		const struct flag *f = &sv->flags[i];
		int at = -1;

		if (f->page != sv->now) {
			continue;
		}
		if (hypot(f->x - x, f->y - y) <= f->size / 4 + tol) {
			at = 0;
		}
		for (int k = 0; at < 0 && k < f->nspan; k++) {
			at = in_box(f->span[k].b, x, y, tol) ? k : -1;
		}
		if (at < 0) {
			continue;
		}
		if (!f->port) {
			sv->sel_net = vg_net(&sv->vg, f->net);
			sv->vg.sel = -1;
		} else if (at == 0) {
			size_t p = first_other(sv, f);

			if (p < sv->npages) {
				go_to(sv, p, f->net);
			}
		} else if (isdigit((unsigned char)f->span[at].t[0]) &&
			   (int)strtol(f->span[at].t, NULL, DECIMAL) !=
				   sv->numbers[sv->now]) {
			size_t p = page_numbered(
				sv, (int)strtol(f->span[at].t, NULL, DECIMAL));

			if (p < sv->npages) {
				go_to(sv, p, f->net);
			}
		}
		return 1;
	}
	return 0;
}

static int pick(struct vg *vg, double x, double y, double tol);

// The pointer moved: the note under it, if any, shown; the selection
// unchanged.
static int move(struct vg *vg, int px, int py)
{
	struct schv *sv = vg->user;

	long was = sv->hover;
	struct vg_point w = vg_point_at(&sv->vg, px, py);
	double x = w.x;
	double y = w.y;

	sv->hover = -1;
	for (size_t i = 0; px >= 0 && i < sv->nnotes; i++) {
		if (sv->notes[i].page == sv->now &&
		    fabs(sv->notes[i].x - x) <= NOTE_BOX / 2 &&
		    fabs(sv->notes[i].y - y) <= NOTE_BOX / 2) {
			sv->hover = (long)i;
		}
	}
	return sv->hover != was || sv->hover >= 0;
}

// A click: on a flag, as flag_click; on a part, the part (vgview's); on
// a wire, a dot or a pin's end out of any part, within 3 pixels, its
// net selected and the part cleared; on nothing, both cleared.
static int click(struct vg *vg, int px, int py)
{
	struct schv *sv = vg->user;

	struct vg_point w = vg_point_at(&sv->vg, px, py);
	double x = w.x;
	double y = w.y;
	double tol = 3 / sv->vg.zoom;

	if (flag_click(sv, x, y, tol)) {
		return 1;
	}
	sv->sel_net = -1;
	if (pick(&sv->vg, x, y, tol) >= 0) {
		return 0;
	}
	for (size_t i = 0; i < sv->nhits; i++) {
		const struct hit *h = &sv->hits[i];
		double dx = h->b[0] - h->a[0];
		double dy = h->b[1] - h->a[1];
		double l = (dx * dx) + (dy * dy);
		double u = NAN;

		if (h->page != sv->now) {
			continue;
		}
		u = l > 0 ? ((x - h->a[0]) * dx + (y - h->a[1]) * dy) / l : 0;
		if (u < 0) {
			u = 0;
		} else if (u > 1) {
			u = 1;
		}
		if (hypot(x - h->a[0] - (u * dx), y - h->a[1] - (u * dy)) <=
		    h->r + tol) {
			sv->sel_net = h->net;
			sv->vg.sel = -1;
			return 1;
		}
	}
	return 0;
}

static int key(struct vg *vg, KeySym k)
{
	struct schv *sv = vg->user;

	if (k == XK_Escape) {
		sv->sel_net = -1;
		sv->typing = -1;
		return 1;
	}
	if (k == XK_Next || k == XK_Prior) {
		turn(sv, next_page(sv, k == XK_Next));
	} else if (k == XK_g) {
		sv->typing = 0;
	} else if (sv->typing >= 0 && k >= XK_0 && k <= XK_9) {
		sv->typing = (sv->typing * DECIMAL) + (int)(k - XK_0);
	} else if (sv->typing >= 0 && (k == XK_Return || k == XK_KP_Enter)) {
		for (size_t p = 0; p < sv->npages; p++) {
			if (sv->numbers[p] == sv->typing) {
				turn(sv, p);
			}
		}
		sv->typing = -1;
	} else {
		sv->typing = -1;
		return 0;
	}
	return 1;
}

static int pick(struct vg *vg, double x, double y, double tol)
{
	struct schv *sv = vg->user;

	int best = -1;
	double area = FAR;

	vg_pick(sv->pages[sv->now], x, y, tol, &best, &area);
	return best;
}

// Whether s starts with pre.
static int starts(const char *s, const char *pre)
{
	return !strncmp(s, pre, strlen(pre));
}

// What read_page keeps from line to line: the page, the spans of the
// text records just before (and the size of the last), the net of the
// records, and the first hit of the page.
struct page_reader {
	size_t pg;
	struct span sp[SPANS];
	double size;
	int nsp;
	int net;
	size_t first;
};

// A text record, a span of the flag that may follow; whether it is one.
static int span_record(struct page_reader *pr, const char *line)
{
	const char *r = line;
	struct span *sp = NULL;
	double sz = NAN;
	double x = NAN;
	double y = NAN;
	double w = NAN;
	int al = 0;
	int at = 0;
	size_t n = 0;

	if (scan(&r, "text %lf %*f %d %lf %lf %n", &sz, &al, &x, &y, &at) !=
		    4 ||
	    !at || pr->nsp >= SPANS) {
		return 0;
	}
	sp = &pr->sp[pr->nsp];
	// The text, cut at the room of a span.
	n = strlen(line + at);
	n = n < sizeof sp->t - 1 ? n : sizeof sp->t - 1;
	memcpy(sp->t, line + at, n);
	sp->t[n] = 0;
	w = stroke_width(sp->t, sz);
	x -= w * al / 2;
	sp->b[0] = x;
	sp->b[1] = y - (BELOW * sz);
	sp->b[2] = x + w;
	sp->b[3] = y + (ABOVE * sz);
	pr->size = sz;
	pr->nsp++;
	return 1;
}

// A note record: note X Y GROUP TEXT.
static void note_record(struct schv *sv, const char *line, size_t pg)
{
	const char *r = line;
	struct note nt;
	int o = 0;

	memset(&nt, 0, sizeof nt);
	if (scan(&r, "note %lf %lf %511s %n", &nt.x, &nt.y, nt.group, &o) ==
		    3 &&
	    o) {
		nt.text = strdup(line + o);
		nt.page = pg;
		sv->notes =
			grow(sv->notes, (sv->nnotes + 1) * sizeof *sv->notes);
		if (!nt.text) {
			exit(2);
		}
		sv->notes[sv->nnotes++] = nt;
	}
}

// A flag record: flag NET X Y KIND, its spans the text records before.
static void flag_record(struct schv *sv, const char *line,
			const struct page_reader *pr)
{
	const char *r = line;
	char kind[KIND_LEN] = "";
	struct flag f;

	memset(&f, 0, sizeof f);
	if (scan(&r, "flag %127s %lf %lf %15s", f.net, &f.x, &f.y, kind) >= 3) {
		f.port = !strcmp(kind, "port");
		f.size = pr->size;
		f.page = pr->pg;
		f.nspan = pr->nsp;
		memcpy(f.span, pr->sp, (size_t)pr->nsp * sizeof *pr->sp);
		sv->flags =
			grow(sv->flags, (sv->nflags + 1) * sizeof *sv->flags);
		sv->flags[sv->nflags++] = f;
	}
}

static void add_hit(struct schv *sv, const struct hit *h)
{
	sv->hits = grow(sv->hits, (sv->nhits + 1) * sizeof *sv->hits);
	sv->hits[sv->nhits++] = *h;
}

// A line (wire) or a disk of a net, p its numbers after the width of a
// line, w half that width or the disk's radius: a wire, segment by
// segment; a dot, a point of its radius.
static void wire_record(struct schv *sv, const struct page_reader *pr, int wire,
			const char *p, double w)
{
	char *end = NULL;
	double q[2];
	double r[2];
	int k = 0;

	while (k < 2 && (q[k] = strtod(p, &end), end != p)) {
		p = end;
		k++;
	}
	if (k < 2) {
		return;
	}
	do {
		struct hit h;

		r[0] = q[0];
		r[1] = q[1];
		if (wire) {
			r[0] = strtod(p, &end);
			if (end == p) {
				break;
			}
			p = end;
			r[1] = strtod(p, &end);
			p = end;
		}
		h.a[0] = q[0];
		h.a[1] = q[1];
		h.b[0] = r[0];
		h.b[1] = r[1];
		h.r = w;
		h.net = pr->net;
		h.page = pr->pg;
		add_hit(sv, &h);
		q[0] = r[0];
		q[1] = r[1];
	} while (wire);
}

// A pin's end at (x, y), of the net of the wire of the page that ends
// there.
static void pin_end(struct schv *sv, const struct page_reader *pr, double x,
		    double y)
{
	for (size_t i = pr->first; i < sv->nhits; i++) {
		if ((hypot(sv->hits[i].a[0] - x, sv->hits[i].a[1] - y) <
			     PIN_TOL ||
		     hypot(sv->hits[i].b[0] - x, sv->hits[i].b[1] - y) <
			     PIN_TOL)) {
			struct hit h = {
				{ x, y }, { x, y }, 0, sv->hits[i].net, pr->pg
			};

			add_hit(sv, &h);
			break;
		}
	}
}

// A line of the page that is not a span.
static void page_record(struct schv *sv, struct page_reader *pr, char *line)
{
	const char *r = line;
	double v[RECORD_NUMS];
	char t[SPAN_LEN];
	double x = NAN;
	double y = NAN;

	if (starts(line, "note ")) {
		note_record(sv, line, pr->pg);
	}
	if (starts(line, "flag ")) {
		flag_record(sv, line, pr);
	}
	pr->nsp = 0;
	if (starts(line, "net") && (!line[3] || line[3] == ' ')) {
		pr->net = line[3] ? vg_net_record(&sv->vg, line + 4) : -1;
		return;
	}
	if (pr->net >= 0 && scan(&r, "line %lf %lf %lf %lf %lf", &v[0], &v[1],
				 &v[2], &v[3], &v[4]) == WIRE_FIELDS) {
		char *end = NULL;
		double w = strtod(line + strlen("line "), &end) / 2;

		wire_record(sv, pr, 1, end, w);
		return;
	}
	r = line;
	if (pr->net >= 0 &&
	    scan(&r, "disk %lf %lf %lf", &v[1], &v[2], &v[0]) == 3) {
		wire_record(sv, pr, 0, line + strlen("disk "), v[0]);
		return;
	}
	r = line;
	if (scan(&r, "pin %63s %lf %lf", t, &x, &y) == 3) {
		pin_end(sv, pr, x, y);
	}
}

// Reads what a click finds on page pg of the text buf: the flags and
// their texts' spans (the text records just before a flag record are
// its), and the wires, dots and pin ends of the nets, a pin by the wire
// that meets it.
static void read_page(struct schv *sv, size_t pg, const char *buf, size_t len)
{
	struct page_reader pr;

	memset(&pr, 0, sizeof pr);
	pr.pg = pg;
	pr.net = -1;
	pr.first = sv->nhits;
	for (const char *l = buf; l && l < buf + len;) {
		const char *e = memchr(l, '\n', (size_t)(buf + len - l));
		size_t n = e ? (size_t)(e - l) : (size_t)(buf + len - l);
		char line[LINE_LEN];

		n = n < sizeof line - 1 ? n : sizeof line - 1;
		memcpy(line, l, n);
		line[n] = 0;
		l = e ? e + 1 : NULL;
		if (!span_record(&pr, line)) {
			page_record(sv, &pr, line);
		}
	}
}

// The badge of a note: its box with an N in it.
static void badge(FILE *out, const struct note *nt)
{
	double h = NOTE_BOX / 2;

	// An error of a write shows as the stream closes.
	(void)fprintf(out,
		      "line 0.6 %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
		      "%.3f %.3f\n",
		      nt->x - h, nt->y - h, nt->x + h, nt->y - h, nt->x + h,
		      nt->y + h, nt->x - h, nt->y + h, nt->x - h, nt->y - h);
	(void)fprintf(out, "text %.3f 0 1 %.3f %.3f N\n", NOTE_BOX / 2, nt->x,
		      nt->y - (NOTE_BOX / 4));
}

// The boxes of the notes of page pg, each with an N in it, into a file of
// their own.
static void note_boxes(struct schv *sv, size_t pg)
{
	char *text = NULL;
	size_t len = 0;
	FILE *out = open_memstream(&text, &len);
	FILE *in = NULL;

	sv->note_files = (struct vg_file **)grow(
		(void *)sv->note_files, (pg + 1) * sizeof *sv->note_files);
	if (!out) {
		exit(2);
	}
	sv->note_files[pg] = NULL;
	for (size_t i = 0; i < sv->nnotes; i++) {
		const struct note *nt = &sv->notes[i];

		if (nt->page == pg) {
			badge(out, nt);
		}
	}
	close_text(out);
	in = len ? fmemopen(text, len, "r") : NULL;
	if (in) {
		sv->note_files[pg] = calloc(1, sizeof **sv->note_files);
		if (!sv->note_files[pg]) {
			exit(2);
		}
		vg_load_stream(&sv->vg, sv->note_files[pg], in, 0);
		(void)fclose(in);
	}
	free(text);
}

// The number of the page of the text buf: of its page record, page W H
// N, or else its place.
static int page_number(const struct schv *sv, const char *buf)
{
	const char *pg = starts(buf, "page ") ? buf : strstr(buf, "\npage ");
	int w = 0;
	int h = 0;
	int num = 0;

	pg = pg && *pg == '\n' ? pg + 1 : pg;
	return pg && scan(&pg, "page %d %d %d", &w, &h, &num) == 3 ?
		       num :
		       (int)sv->npages + 1;
}

// Takes the text buf of a page, len long and a NUL after, as the next
// page.
static void add_page(struct schv *sv, char *buf, size_t len)
{
	FILE *m = fmemopen(buf, len, "r");

	sv->pages = (struct vg_file **)grow(
		(void *)sv->pages, (sv->npages + 1) * sizeof *sv->pages);
	sv->numbers = grow(sv->numbers, (sv->npages + 1) * sizeof *sv->numbers);
	if (!m) {
		exit(2);
	}
	sv->pages[sv->npages] = calloc(1, sizeof **sv->pages);
	if (!sv->pages[sv->npages]) {
		exit(2);
	}
	vg_load_stream(&sv->vg, sv->pages[sv->npages], m, 0);
	(void)fclose(m);
	read_page(sv, sv->npages, buf, len);
	note_boxes(sv, sv->npages);
	sv->numbers[sv->npages] = page_number(sv, buf);
	sv->npages++;
}

// Reads the file, a page from each page record to the next.
static void load(struct schv *sv)
{
	FILE *in = fopen(sv->path, "r");
	char *line = NULL;
	char *buf = NULL;
	size_t cap = 0;
	size_t len = 0;
	size_t bcap = 0;
	ssize_t n = 0;
	int paged = 0;

	sv->npages = 0;
	sv->nflags = 0;
	sv->nhits = 0;
	for (size_t i = 0; i < sv->nnotes; i++) {
		free(sv->notes[i].text);
	}
	sv->nnotes = 0;
	sv->hover = -1;
	sv->sel_net = -1;
	if (!in) {
		(void)fprintf(stderr, "schview: cannot read %s\n", sv->path);
		exit(2);
	}
	for (int more = 1; more;) {
		n = getline(&line, &cap, in);
		more = n > 0;
		// A page ends at the next page record; what comes before the
		// first is the first page's.
		if ((!more || (starts(line, "page ") && paged)) && len) {
			add_page(sv, buf, len);
			len = 0;
			paged = 0;
		}
		if (more && starts(line, "page ")) {
			paged = 1;
		}
		if (more) {
			if (len + (size_t)n + 1 > bcap) {
				bcap = 2 * (len + (size_t)n + 1);
				buf = grow(buf, bcap);
			}
			memcpy(buf + len, line, (size_t)n);
			len += (size_t)n;
			buf[len] = 0; // For finding its page record.
		}
	}
	free(line);
	free(buf);
	(void)fclose(in);
	if (!sv->npages) {
		(void)fprintf(stderr, "schview: %s has nothing to show\n",
			      sv->path);
		exit(2);
	}
	if (sv->now >= sv->npages) {
		sv->now = 0;
	}
}

static void reload(struct vg *vg)
{
	struct schv *sv = vg->user;

	for (size_t p = 0; p < sv->npages; p++) {
		vg_free(&sv->vg, sv->pages[p]);
		free(sv->pages[p]);
		if (sv->note_files[p]) {
			vg_free(&sv->vg, sv->note_files[p]);
			free(sv->note_files[p]);
		}
	}
	sv->badge_of = -2; // The notes are new: the badge open is made anew.
	load(sv);
}

int main(int argc, char **argv)
{
	static const struct vg_viewer v = { .name = "schview",
					    .help = help,
					    .draw = draw,
					    .pick = pick,
					    .overlay = overlay,
					    .key = key,
					    .command = command,
					    .click = click,
					    .move = move,
					    .reload = reload };
	struct schv ctx;
	int i = 0;

	memset(&ctx, 0, sizeof ctx);
	vg_setup(&ctx.vg, &ctx);
	ctx.typing = -1;
	ctx.sel_net = -1;
	ctx.hover = -1;
	ctx.badge_of = -1;
	ctx.shown = -1;
	i = vg_start(&ctx.vg, argc, argv, &v);

	ctx.vg.solid = 1; // A line thinner than a pixel stays a pixel, whole.
	ctx.vg.fill = 1; // Fitted, the frame of the sheet meets the window.
	ctx.path = argv[i];
	load(&ctx);
	vg_loop(&ctx.vg, ctx.pages[ctx.now]->box);
	return 0;
}
