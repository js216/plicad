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
// drawing are in the stroke font, at one size whatever the zoom; the
// help in the font fixed.  The selected part is drawn in light cyan, and its page shows when it is newly
// selected; c centers it (again: zooms in two times), C zooms out two
// times, F5 reads the file again (Loading... in the upper right
// meanwhile), ? shows or hides a help of the keys, and q quits.  The
// window's title is the absolute path of the file.  Commands come on
// stdin, one a line: "select NAME", "select", "center",
// "shot FILE" (the next frame as a picture), and "page next", "page
// prev" and "page N".  So two viewers
// cross-probe through two FIFOs, each one's stdout the other's stdin.

#define _POSIX_C_SOURCE 200809L // For fmemopen, open_memstream, strdup.

#include "stroke.h"
#include "vecgl.h"
#include <GL/gl.h>
#include <X11/keysym.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

// The pages, each read from its page record to the next, the one shown,
// its number (that of its page record, or else its place), and a page
// number being typed after g.
static struct vg_file **pages;
static int *numbers;
static size_t npages, now;
static int typing = -1;

// A span of the text of a flag: its text and box.
struct span {
	char t[64];
	double b[4];
};

// The flags of the pages (vec(5) flag NET X Y port|global): the name of
// its net, the end of its pin, whether a port of the sheet, the size of
// its text, and its text's spans (a port's name, then each piece of its
// list of pages), by page.
struct flag {
	char net[128];
	double x, y, size;
	int port, nspan;
	struct span span[32];
	size_t page;
};
static struct flag *flags;
static size_t nflags;

// What a click selects a net by: a wire, a dot or a pin's end, of the
// net (vg_net), by page; a pin's end is a point, a dot a disk.
struct hit {
	double a[2], b[2], r;
	int net;
	size_t page;
};
static struct hit *hits;
static size_t nhits;
static int sel_net = -1; // The net selected, or -1.

// The notes (vec(5) note X Y GROUP TEXT): the middle of the box, its
// group (- outside any), its text and page; the boxes of each page, drawn
// as a file of their own; the note under the pointer (-1 none).
#define NOTE_BOX 10.0
struct note {
	double x, y;
	char group[512], *text;
	size_t page;
};
static struct note *notes;
static size_t nnotes;
static struct vg_file **note_files;
static long hover = -1;

// Whether part k is of the group of the note under the pointer: its
// name the group, or the group and a dot before the rest.
static int in_hover_group(int k)
{
	const char *g, *name;
	size_t n;

	if (hover < 0 || k < 0) {
		return 0;
	}
	g = notes[hover].group;
	name = vg_part_name(k);
	n = strlen(g);
	return strcmp(g, "-") && !strncmp(name, g, n) &&
	       (!name[n] || name[n] == '.');
}

// The parts of the group of the note under the pointer magenta
// (#FF80FF); else the text
// rest of the drawing a whiter gray (#D4D4D0), light cyan (#A0FFFF) for
// the part or the net selected (its wires, dots and flags), or black
// for what clears.
static void color(const struct vg_run *r)
{
	static const double khaki[3] = { 0xbb / 255.0, 0xbb / 255.0,
					 0x99 / 255.0 };
	static const double gray[3] = { 0xd4 / 255.0, 0xd4 / 255.0,
					0xd0 / 255.0 };
	static const double cyan[3] = { 0xa0 / 255.0, 1, 1 };
	static const double magenta[3] = { 1, 0x80 / 255.0, 1 };
	static const double black[3] = { 0, 0, 0 };

	vg_color(!r->dark		 ? black :
		 in_hover_group(r->part) ? magenta :
		 (r->part >= 0 && r->part == vg_sel) ||
				 (r->net >= 0 && r->net == sel_net) ?
					   cyan :
		 r->text || r->frame ? khaki :
				       gray,
		 1);
}

static int all(const struct vg_run *r)
{
	return r->part >= -1;
}

// The page holding part k, or now.
static size_t page_of(int k)
{
	for (size_t p = 0; k >= 0 && p < npages; p++) {
		for (size_t e = 0; e < pages[p]->nelem; e++) {
			if (pages[p]->elem[e].part == k) {
				return p;
			}
		}
	}
	return now;
}

static void turn(size_t p)
{
	if (p < npages && p != now) {
		now = p;
		vg_fit(pages[now]->box);
	}
}

static void note_color(const struct vg_run *r)
{
	(void)r;
	vg_color((const double[3]){ 0xbb / 255.0, 0xbb / 255.0, 0x99 / 255.0 },
		 1);
}

// The badge of the note open, magenta as the parts it lights.
static void open_color(const struct vg_run *r)
{
	(void)r;
	vg_color((const double[3]){ 1, 0x80 / 255.0, 1 }, 1);
}

static void badge(FILE *out, const struct note *nt);

// The badge of the note under the pointer (of note badge_of), made anew
// when it changes.
static struct vg_file *open_file;
static long badge_of = -1;

static void open_badge(void)
{
	struct vg_file *f = open_file;
	char *text = NULL;
	size_t len = 0;
	FILE *out, *in;

	if (hover != badge_of || !f) {
		if (f) {
			vg_free(f);
			free(f);
			f = open_file = NULL;
		}
		badge_of = hover;
		if (hover < 0) {
			return;
		}
		out = open_memstream(&text, &len);
		if (!out) {
			exit(2);
		}
		badge(out, &notes[hover]);
		fclose(out);
		in = fmemopen(text, len, "r");
		f = calloc(1, sizeof *f);
		if (!in || !f) {
			exit(2);
		}
		vg_load_stream(f, in, 0);
		fclose(in);
		free(text);
		open_file = f;
	}
	if (hover >= 0) {
		vg_draw(f, all, open_color);
	}
}

static void draw(void)
{
	static int shown = -1; // The selection last turned to.

	// A part newly selected, from outside or here, shows its page; the
	// pages turn freely after.
	if (vg_sel != shown) {
		shown = vg_sel;
		turn(page_of(vg_sel));
	}
	vg_draw(pages[now], all, color);
	if (note_files[now]) {
		vg_draw(note_files[now], all, note_color);
	}
	open_badge();
}

// The text of the note open, in the stroke font in a black box at the
// upper left of the window, under the bar, wrapped at about 60
// characters and inside the window.
static void note_text(const char *t)
{
	char line[16][80];
	int n = 0;
	double w = 0, lh = VG_CAP * 5 / 3, room = vg_w - 24, y;

	while (*t && n < 16) {
		size_t len = strlen(t), cut = len;
		char *sp;

		if (len > 60) {
			cut = 60;
			while (cut > 20 && t[cut] != ' ') {
				cut--;
			}
			if (t[cut] != ' ') {
				cut = 60;
			}
		}
		snprintf(line[n], sizeof line[n], "%.*s", (int)cut, t);
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
	y = vg_h - vg_top;
	glColor3d(0, 0, 0);
	glRectd(0, y, w + 16, y - n * lh - 12);
	glColor3d(1, 1, 1);
	for (int k = 0; k < n; k++) {
		vg_stroke(8, y - 6 - VG_CAP - k * lh, line[k]);
	}
}

// The page number being typed after g, at the top left, and the name of
// the net selected at the top right.
static void overlay(void)
{
	char t[64];

	if (typing >= 0) {
		snprintf(t, sizeof t, "go to:");
		if (typing > 0) {
			snprintf(t + strlen(t), sizeof t - strlen(t), " %d",
				 typing);
		}
		vg_color((const double[3]){ 0.8, 0.8, 0.8 }, 1);
		vg_text(4, vg_h - vg_font_h - 2, t);
	}
	if (sel_net >= 0) {
		const char *c = vg_net_class(sel_net);
		char *name = malloc(strlen(vg_net_name(sel_net)) +
				    (c ? strlen(c) + 1 : 0) + 1);

		// CLASS:NET, or NET for a net of no class.
		if (name) {
			sprintf(name, "%s%s%s", c ? c : "", c ? ":" : "",
				vg_net_name(sel_net));
			vg_corner(name);
			free(name);
		}
	}
	if (hover >= 0) {
		note_text(notes[hover].text);
	}
}

// The commands of stdin past vecgl's: "page next", "page prev" and
// "page N".
static int command(const char *c)
{
	int n;

	if (!strcmp(c, "page next") || !strcmp(c, "page prev")) {
		turn(c[5] == 'n' ? now + 1 : now ? now - 1 : now);
	} else if (sscanf(c, "page %d", &n) == 1) {
		for (size_t p = 0; p < npages; p++) {
			if (numbers[p] == n) {
				turn(p);
			}
		}
	}
	return 1;
}

// Shows page p, at the zoom as it was, centered on its port of the name.
static void go_to(size_t p, const char *net)
{
	double z = vg_zoom;

	turn(p);
	vg_zoom = z;
	for (size_t j = 0; j < nflags; j++) {
		if (flags[j].page == p && flags[j].port &&
		    !strcmp(flags[j].net, net)) {
			vg_x = flags[j].x;
			vg_y = flags[j].y;
			return;
		}
	}
}

// The page of a number, or npages.
static size_t page_numbered(int n)
{
	for (size_t p = 0; p < npages; p++) {
		if (numbers[p] == n) {
			return p;
		}
	}
	return npages;
}

// The first page a port lists other than the one shown, or npages.
static size_t first_other(const struct flag *f)
{
	for (int k = 1; k < f->nspan; k++) {
		if (isdigit((unsigned char)f->span[k].t[0]) &&
		    atoi(f->span[k].t) != numbers[now]) {
			return page_numbered(atoi(f->span[k].t));
		}
	}
	return npages;
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
static int flag_click(double x, double y, double tol)
{
	for (size_t i = 0; i < nflags; i++) {
		const struct flag *f = &flags[i];
		int at = -1;

		if (f->page != now) {
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
			sel_net = vg_net(f->net);
			vg_sel = -1;
		} else if (at == 0) {
			size_t p = first_other(f);

			if (p < npages) {
				go_to(p, f->net);
			}
		} else if (isdigit((unsigned char)f->span[at].t[0]) &&
			   atoi(f->span[at].t) != numbers[now]) {
			size_t p = page_numbered(atoi(f->span[at].t));

			if (p < npages) {
				go_to(p, f->net);
			}
		}
		return 1;
	}
	return 0;
}

static int pick(double x, double y, double tol);

// The pointer moved: the note under it, if any, shown; the selection
// unchanged.
static int move(int px, int py)
{
	long was = hover;
	double x = vg_x + (px - vg_w / 2) / vg_zoom;
	double y = vg_y - (py - vg_h / 2) / vg_zoom;

	hover = -1;
	for (size_t i = 0; px >= 0 && i < nnotes; i++) {
		if (notes[i].page == now &&
		    fabs(notes[i].x - x) <= NOTE_BOX / 2 &&
		    fabs(notes[i].y - y) <= NOTE_BOX / 2) {
			hover = (long)i;
		}
	}
	return hover != was || hover >= 0;
}

// A click: on a flag, as flag_click; on a part, the part (vgview's); on
// a wire, a dot or a pin's end out of any part, within 3 pixels, its
// net selected and the part cleared; on nothing, both cleared.
static int click(int px, int py)
{
	double x = vg_x + (px - vg_w / 2) / vg_zoom;
	double y = vg_y - (py - vg_h / 2) / vg_zoom, tol = 3 / vg_zoom;

	if (flag_click(x, y, tol)) {
		return 1;
	}
	sel_net = -1;
	if (pick(x, y, tol) >= 0) {
		return 0;
	}
	for (size_t i = 0; i < nhits; i++) {
		const struct hit *h = &hits[i];
		double dx = h->b[0] - h->a[0], dy = h->b[1] - h->a[1];
		double l = dx * dx + dy * dy, u;

		if (h->page != now) {
			continue;
		}
		u = l > 0 ? ((x - h->a[0]) * dx + (y - h->a[1]) * dy) / l : 0;
		u = u < 0 ? 0 : u > 1 ? 1 : u;
		if (hypot(x - h->a[0] - u * dx, y - h->a[1] - u * dy) <=
		    h->r + tol) {
			sel_net = h->net;
			vg_sel = -1;
			return 1;
		}
	}
	return 0;
}

static int key(KeySym k)
{
	if (k == XK_Escape) {
		sel_net = -1;
		typing = -1;
		return 1;
	}
	if (k == XK_Next || k == XK_Prior) {
		turn(k == XK_Next ? now + 1 : now ? now - 1 : now);
	} else if (k == XK_g) {
		typing = 0;
	} else if (typing >= 0 && k >= XK_0 && k <= XK_9) {
		typing = typing * 10 + (int)(k - XK_0);
	} else if (typing >= 0 && (k == XK_Return || k == XK_KP_Enter)) {
		for (size_t p = 0; p < npages; p++) {
			if (numbers[p] == typing) {
				turn(p);
			}
		}
		typing = -1;
	} else {
		typing = -1;
		return 0;
	}
	return 1;
}

static int pick(double x, double y, double tol)
{
	int best = -1;
	double area = 1e300;

	vg_pick(pages[now], x, y, tol, &best, &area);
	return best;
}

static const char *path;

// The width of a text record's text, in its font at its size.
static double width_of(char font, const char *t, double size)
{
	return vg_text_width(font, t) * size;
}

// Reads what a click finds on page pg of the text buf: the flags and
// their texts' spans (the text records just before a flag record are
// its), and the wires, dots and pin ends of the nets, a pin by the wire
// that meets it.
static void read_page(const char *buf, size_t len, size_t pg)
{
	struct span sp[32];
	double size = 0;
	int nsp = 0, net = -1;
	size_t first = nhits;

	for (const char *l = buf; l && l < buf + len;) {
		const char *e = memchr(l, '\n', (size_t)(buf + len - l));
		size_t n = e ? (size_t)(e - l) : (size_t)(buf + len - l);
		char line[4096], kind[16] = "", t[64], font = 'R';
		double v[8], sz, x, y;
		int al, at = 0;

		n = n < sizeof line - 1 ? n : sizeof line - 1;
		memcpy(line, l, n);
		line[n] = 0;
		l = e ? e + 1 : NULL;
		if (sscanf(line, "text %c %lf %*f %d %lf %lf %n", &font, &sz,
			   &al, &x, &y, &at) == 5 &&
		    at && nsp < 32) {
			double w;

			snprintf(t, sizeof t, "%s", line + at);
			w = width_of(font, t, sz);
			x -= w * al / 2;
			snprintf(sp[nsp].t, sizeof sp[nsp].t, "%s", t);
			sp[nsp].b[0] = x;
			sp[nsp].b[1] = y - 0.25 * sz;
			sp[nsp].b[2] = x + w;
			sp[nsp].b[3] = y + 0.75 * sz;
			size = sz;
			nsp++;
			continue;
		}
		if (!strncmp(line, "note ", 5)) {
			struct note nt;
			int o = 0;

			memset(&nt, 0, sizeof nt);
			if (sscanf(line, "note %lf %lf %511s %n", &nt.x, &nt.y,
				   nt.group, &o) == 3 &&
			    o) {
				nt.text = strdup(line + o);
				nt.page = pg;
				notes = realloc(notes,
						(nnotes + 1) * sizeof *notes);
				if (!notes || !nt.text) {
					exit(2);
				}
				notes[nnotes++] = nt;
			}
		}
		if (!strncmp(line, "flag ", 5)) {
			struct flag f;

			memset(&f, 0, sizeof f);
			if (sscanf(line, "flag %127s %lf %lf %15s", f.net, &f.x,
				   &f.y, kind) >= 3) {
				f.port = !strcmp(kind, "port");
				f.size = size;
				f.page = pg;
				f.nspan = nsp;
				memcpy(f.span, sp, (size_t)nsp * sizeof *sp);
				flags = realloc(flags,
						(nflags + 1) * sizeof *flags);
				if (!flags) {
					exit(2);
				}
				flags[nflags++] = f;
			}
		}
		nsp = 0;
		if (!strncmp(line, "net", 3) && (!line[3] || line[3] == ' ')) {
			net = line[3] ? vg_net_record(line + 4) : -1;
		} else if (net >= 0 &&
			   (sscanf(line, "line %lf %lf %lf %lf %lf", &v[0],
				   &v[1], &v[2], &v[3], &v[4]) == 5 ||
			    sscanf(line, "disk %lf %lf %lf", &v[1], &v[2],
				   &v[0]) == 3)) {
			// A wire, segment by segment; a dot, a point of its
			// radius.
			const char *p = line + 5;
			char *end;
			double w = line[0] == 'l' ? strtod(p, &end) / 2 : v[0];
			double q[2], r[2];
			int k = 0;

			if (line[0] == 'l') {
				p = end;
			}
			while (k < 2 && (q[k] = strtod(p, &end), end != p)) {
				p = end;
				k++;
			}
			if (k < 2) {
				continue;
			}
			do {
				struct hit h;

				r[0] = q[0];
				r[1] = q[1];
				if (line[0] == 'l') {
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
				h.net = net;
				h.page = pg;
				hits = realloc(hits,
					       (nhits + 1) * sizeof *hits);
				if (!hits) {
					exit(2);
				}
				hits[nhits++] = h;
				q[0] = r[0];
				q[1] = r[1];
			} while (line[0] == 'l');
		} else if (sscanf(line, "pin %63s %lf %lf", t, &x, &y) == 3) {
			// A pin's end, of the net of the wire that ends there.
			for (size_t i = first; i < nhits; i++) {
				if ((hypot(hits[i].a[0] - x, hits[i].a[1] - y) <
					     1e-3 ||
				     hypot(hits[i].b[0] - x, hits[i].b[1] - y) <
					     1e-3)) {
					struct hit h = { { x, y },
							 { x, y },
							 0,
							 hits[i].net,
							 pg };

					hits = realloc(hits,
						       (nhits + 1) *
							       sizeof *hits);
					if (!hits) {
						exit(2);
					}
					hits[nhits++] = h;
					break;
				}
			}
		}
	}
}

// The badge of a note: its box with an N in it.
static void badge(FILE *out, const struct note *nt)
{
	double h = NOTE_BOX / 2;

	fprintf(out,
		"line 0.6 %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
		"%.3f %.3f\n",
		nt->x - h, nt->y - h, nt->x + h, nt->y - h, nt->x + h,
		nt->y + h, nt->x - h, nt->y + h, nt->x - h, nt->y - h);
	fprintf(out, "text S %.3f 0 1 %.3f %.3f N\n", NOTE_BOX / 2, nt->x,
		nt->y - NOTE_BOX / 4);
}

// The boxes of the notes of page pg, each with an N in it, into a file of
// their own.
static void note_boxes(size_t pg)
{
	char *text = NULL;
	size_t len = 0;
	FILE *out = open_memstream(&text, &len), *in;

	note_files = realloc(note_files, (pg + 1) * sizeof *note_files);
	if (!note_files || !out) {
		exit(2);
	}
	note_files[pg] = NULL;
	for (size_t i = 0; i < nnotes; i++) {
		const struct note *nt = &notes[i];

		if (nt->page == pg) {
			badge(out, nt);
		}
	}
	fclose(out);
	in = len ? fmemopen(text, len, "r") : NULL;
	if (in) {
		note_files[pg] = calloc(1, sizeof **note_files);
		if (!note_files[pg]) {
			exit(2);
		}
		vg_load_stream(note_files[pg], in, 0);
		fclose(in);
	}
	free(text);
}

// Reads the file, a page from each page record to the next.
static void load(void)
{
	FILE *in = fopen(path, "r");
	char *line = NULL, *buf = NULL;
	size_t cap = 0, len = 0, bcap = 0;
	ssize_t n;
	int paged = 0;

	npages = 0;
	nflags = 0;
	nhits = 0;
	for (size_t i = 0; i < nnotes; i++) {
		free(notes[i].text);
	}
	nnotes = 0;
	hover = -1;
	sel_net = -1;
	if (!in) {
		fprintf(stderr, "schview: cannot read %s\n", path);
		exit(2);
	}
	for (int more = 1; more;) {
		n = getline(&line, &cap, in);
		more = n > 0;
		// A page ends at the next page record; what comes before the
		// first is the first page's.
		if ((!more || (!strncmp(line, "page ", 5) && paged)) && len) {
			FILE *m = fmemopen(buf, len, "r");
			int w, h, num;
			const char *pg;

			pages = realloc(pages, (npages + 1) * sizeof *pages);
			numbers = realloc(numbers,
					  (npages + 1) * sizeof *numbers);
			if (!pages || !numbers || !m) {
				exit(2);
			}
			pages[npages] = calloc(1, sizeof **pages);
			if (!pages[npages]) {
				exit(2);
			}
			vg_load_stream(pages[npages], m, 0);
			fclose(m);
			read_page(buf, len, npages);
			note_boxes(npages);
			// Its number from its page record, page W H N.
			pg = !strncmp(buf, "page ", 5) ? buf :
							 strstr(buf, "\npage ");
			pg = pg && *pg == '\n' ? pg + 1 : pg;
			numbers[npages] = pg && sscanf(pg, "page %d %d %d", &w,
						       &h, &num) == 3 ?
						  num :
						  (int)npages + 1;
			npages++;
			len = 0;
			paged = 0;
		}
		paged |= more && !strncmp(line, "page ", 5);
		if (more) {
			if (len + (size_t)n + 1 > bcap) {
				bcap = 2 * (len + (size_t)n + 1);
				buf = realloc(buf, bcap);
				if (!buf) {
					exit(2);
				}
			}
			memcpy(buf + len, line, (size_t)n);
			len += (size_t)n;
		}
	}
	free(line);
	free(buf);
	fclose(in);
	if (!npages) {
		fprintf(stderr, "schview: %s has nothing to show\n", path);
		exit(2);
	}
	if (now >= npages) {
		now = 0;
	}
}

static void reload(void)
{
	for (size_t p = 0; p < npages; p++) {
		vg_free(pages[p]);
		free(pages[p]);
		if (note_files[p]) {
			vg_free(note_files[p]);
			free(note_files[p]);
		}
	}
	badge_of = -2; // The notes are new: the badge open is made anew.
	load();
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
	int i = vg_start(argc, argv, &v);

	vg_solid = 1; // A line thinner than a pixel stays a pixel, whole.
	vg_fill = 1; // Fitted, the frame of the sheet meets the window.
	path = argv[i];
	load();
	vg_loop(pages[now]->box);
	return 0;
}
