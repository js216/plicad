// SPDX-License-Identifier: GPL-3.0-only
// schview.c -- Show a schematic sheet in a window
// Copyright (c) 2026 Jakob Kastelic

// Usage: schview SHEET.vec
//
// Shows the pages of a schematic as vec(5), from schdraw, one at a time
// (each starts at its page record; PgDn and PgUp turn them, g and a page
// number and Enter goes to it, a click on a sheet port goes to the next
// page with a port of its name), in an X11 window by OpenGL,
// khaki on black (vecgl.h).  It opens fitted to the sheet, as Space
// fits it; the wheel zooms round the pointer and the left button drags
// the view.  A click within 3 pixels of the box of an element of a part
// (its symbol's body, a text of it) selects the part, of the smallest
// such element, and prints "select NAME" on stdout; a click on none
// clears the selection here alone, and prints nothing.  The selected
// part is drawn in light cyan, and its page shows when it is newly
// selected; c centers it (again: zooms in two times), C zooms out two
// times, F5 reads the file again (Loading... in the upper right
// meanwhile), ? shows or hides a help of the keys, and q quits.  The
// window's title is the absolute path of the file.  Commands come on
// stdin, one a line: "select NAME", "select", "center",
// "shot FILE" (the next frame as a picture), and "page next", "page
// prev" and "page N".  So two viewers
// cross-probe through two FIFOs, each one's stdout the other's stdin.

#define _POSIX_C_SOURCE 200809L // For fmemopen.

#include "vecgl.h"
#include <GL/gl.h>
#include <X11/keysym.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

static const char *const help[] = {
	"left drag       move",
	"wheel           zoom",
	"click           select a part, print its name; on none, clear",
	"space           fit all",
	"PgDn PgUp       next, previous page",
	"g N Enter       go to page N",
	"click a port    its next page with the port",
	"F5              read the file again",
	"Esc             deselect, hide this help",
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

// The flags of the pages (vec(5) flag NET X Y): a sheet port's name and
// the end of its pin, by page.
struct flag {
	char net[128];
	double x, y;
	size_t page;
};
static struct flag *flags;
static size_t nflags;

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
}

// The number of the page, at the top left: N / of all.
static void overlay(void)
{
	char t[64];

	if (npages > 1) {
		snprintf(t, sizeof t, "page %d (%zu of %zu)%s", numbers[now],
			 now + 1, npages, typing >= 0 ? "  go to:" : "");
		if (typing > 0) {
			snprintf(t + strlen(t), sizeof t - strlen(t), " %d",
				 typing);
		}
		vg_color((const double[3]){ 0.8, 0.8, 0.8 }, 1);
		vg_text(4, vg_h - vg_font_h - 2, t);
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

// A click on a sheet port: its next page with a port of the name shows,
// centered on that port at the zoom as it was.
static int click(int px, int py)
{
	double x = vg_x + (px - vg_w / 2) / vg_zoom;
	double y = vg_y - (py - vg_h / 2) / vg_zoom, tol = 6 / vg_zoom;

	for (size_t i = 0; i < nflags; i++) {
		if (flags[i].page != now || fabs(flags[i].x - x) > tol ||
		    fabs(flags[i].y - y) > tol) {
			continue;
		}
		for (size_t d = 1; d < npages; d++) {
			size_t p = (now + d) % npages;

			for (size_t j = 0; j < nflags; j++) {
				if (flags[j].page == p &&
				    !strcmp(flags[j].net, flags[i].net)) {
					double z = vg_zoom;

					turn(p);
					vg_zoom = z;
					vg_x = flags[j].x;
					vg_y = flags[j].y;
					return 1;
				}
			}
		}
	}
	return 0;
}

static int key(KeySym k)
{
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
			for (const char *l = buf; l && l < buf + len;) {
				struct flag f = { "", 0, 0, npages };

				if (sscanf(l, "flag %127s %lf %lf", f.net, &f.x,
					   &f.y) == 3) {
					flags = realloc(flags,
							(nflags + 1) *
								sizeof *flags);
					if (!flags) {
						exit(2);
					}
					flags[nflags++] = f;
				}
				l = memchr(l, '\n', (size_t)(buf + len - l));
				l = l ? l + 1 : NULL;
			}
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
	}
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
					    .reload = reload };
	int i = vg_start(argc, argv, &v);

	vg_solid = 1; // A line thinner than a pixel stays a pixel, whole.
	vg_fill = 1; // Fitted, the frame of the sheet meets the window.
	path = argv[i];
	load();
	vg_loop(pages[now]->box);
	return 0;
}
