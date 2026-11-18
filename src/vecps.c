// SPDX-License-Identifier: GPL-3.0-only
// vecps.c -- Draw vector files as pages of PostScript
// Copyright (c) 2026 Jakob Kastelic

// Usage: vecps [-w X0 Y0 X1 Y1 | -r X0 Y0 X1 Y1 | -p PART]... [-n]
//              FILE[=R,G,B[,line]]... > out.ps
//
// Draws each vec(5) file, the first underneath, in its color (each of
// R, G, B from 0 to 1, black by default): filled, or with line, its
// outlines only, so a file drawn over another shows both.  The window
// (-w, in the units of the files) is the region drawn, by default all
// of the first file; it fills a letter page, landscape.  A first file
// with page records is drawn as those pages instead, each at its size and
// scale 1: page k of each file, its records from its k-th page record to
// the next (those before the first are of the first), on page k of the
// PostScript.  Clear polarity erases what its file drew before and
// nothing of the other files (vec(5)).  Text, and every name it writes,
// is drawn stroke by stroke in the stroke font (stroke.h).  With -n the
// names of the boxes of the first file are written in them.  The
// window by default holds all the files, their lines' widths and
// disks' radii counted.  Each port of the sheet of the first file links
// to pages as schview(1) follows its clicks: its circle and name to the
// first page it lists other than its own, each number of its list but
// its own to that page (pdfmark /Link, /Fit, no border); a global links
// nowhere.  Note records are for the viewers: none is printed.
//
// A region render (-r, or -p) crops to a window and names what is in it:
// -r X0 Y0 X1 Y1 is the window; -p PART (by its name or designator,
// again for more, or a list with commas) the box of the parts named,
// grown by a fifth of its size and 1 unit.  Each part in the window
// shows its designator (from its part bracket) in its box, and each
// track and pad of a net its net's name, along the track or in the pad,
// sized to it but readable on the page.

#include "stroke.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define HALF_TURN 180.0 // In degrees,
#define FULL_TURN 360.0 // a full turn,
#define QUARTER 90.0 // a quarter,
#define THREE_QUARTERS 270.0 // and three.
#define NONE_YET 1e300 // Beyond any box.
#define OUTLINE_W 0.05 // The width of the lines of a file drawn in outline.
#define LINK_BELOW 0.25 // A link of a text reaches this far below it,
#define LINK_ABOVE 0.75 // and above, in its capital heights.
#define NAME_BOX_MOST 50.0 // The widest box a name is written in.
#define NAME_PT 5.0 // The capitals of the names of -n, in points.
#define LABEL_LEAST 2.0 // The least and the most capital height of a
#define LABEL_MOST 5.0 // label on the page, in points.
#define NET_GRAY 0.15 // The gray of the name of a net.
#define TRACK_FIT 0.5 // A net's name on a track, to its width.
#define PAD_FIT 0.43 // A net's name in a pad, to its shorter side,
#define PAD_ROOM 0.85 // and the room beyond its width along the longer.
#define REF_SPACE 1.2 // A designator's width, to the box of its part,
#define REF_ROOM 1.6 // and the room beyond it,
#define REF_FIT 0.22 // its height to the shorter side of the box,
#define REF_TALL 1.5 // and how tall a box is for it to stand up.
#define LETTER_W 792.0 // A letter page, landscape, in points,
#define LETTER_H 612.0
#define PAGE_EDGE 36.0 // and the margins of both its sides.
#define GROW_SHARE 5.0 // The window of -p: the parts grown by a fifth.
enum {
	DECIMAL = 10, // The base of decimal numbers.
	LINE = 1048576, // The longest line of a file.
	SPAN_LEN = 64, // Room for the text of a span,
	SPANS = 32, // and the most spans of a port.
	NAME_LEN = 512, // Room for a name,
	KIND_LEN = 16, // the kind of a flag,
	REF_LEN = 64, // a designator,
	PARTS_LINE = 1200, // a line of a part or name record,
	PATH_LEN = 4096, // and a path.
	NUMBERS = 4096, // The most numbers of a record named on,
	TRACK_NUMBERS = 5, // of a track that is named,
	PAD_NUMBERS = 6, // and of a pad of more than a disk.
	TEXT_FIELDS = 5, // The numbers of a text record,
	NAME_FIELDS = 5 // and the fields of a name record.
};

// Copies the first n bytes of src, or as many as fit, into dst.
static void copy_prefix(char *dst, size_t size, const char *src, size_t n)
{
	n = n < size - 1 ? n : size - 1;
	memcpy(dst, src, n);
	dst[n] = 0;
}

// Prints the message and exits; as a macro, the exit shows to the
// analyzer.
#define fail(f, msg) ((void)fprintf(stderr, "%s: %s\n", f, msg), exit(2))

// Whether s starts with pre.
static int starts(const char *s, const char *pre)
{
	return !strncmp(s, pre, strlen(pre));
}

// A point, in the units of the files.
struct pt {
	double x, y;
};

// Writes text in the stroke font, its capitals size high, turned a
// degrees counterclockwise about (x, y), from which it runs flush left
// (align 0), centered (1) or flush right (2).
static void strokes(const char *t, double size, double a, int align, double x,
		    double y)
{
	double *sg = NULL;
	size_t n = stroke_text(t, size, &sg);

	printf("gsave %.3f %.3f translate %.3f rotate %.4f 0 translate "
	       "%.4f setlinewidth newpath",
	       x, y, a, -stroke_width(t, size) * align / 2, STROKE_PEN * size);
	for (size_t j = 0; j < n; j++) {
		printf(" %.4f %.4f moveto %.4f %.4f lineto", sg[4 * j],
		       sg[(4 * j) + 1], sg[(4 * j) + 2], sg[(4 * j) + 3]);
	}
	printf(" stroke grestore\n");
	free(sg);
}

// The number of page records of a file, and the size of its page k
// (from 1), if it has one, into size.
static int page_of(const char *path, int k, double size[2])
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE);
	int n = 0;
	double pw = NAN;
	double ph = NAN;

	while (f && buf && fgets(buf, LINE, f)) {
		const char *r = buf;

		if (scan(&r, "page %lf %lf", &pw, &ph) == 2 && ++n == k) {
			size[0] = pw;
			size[1] = ph;
		}
	}
	if (f) {
		(void)fclose(f);
	}
	free(buf);
	return n;
}

// Grows e by the extent of a file: of its drawing records alone, each
// line by its half width and each disk by its radius.
static void extent(const char *path, double e[4])
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE);

	if (!f || !buf) {
		fail(path, "cannot read");
	}
	while (fgets(buf, LINE, f)) {
		char *s = buf + strlen("line ");
		char *end = NULL;
		const char *r = buf;
		double v[3];
		double h = 0;
		int k = 0;

		if (scan(&r, "disk %lf %lf %lf", &v[0], &v[1], &v[2]) == 3) {
			e[0] = fmin(e[0], v[0] - v[2]);
			e[1] = fmin(e[1], v[1] - v[2]);
			e[2] = fmax(e[2], v[0] + v[2]);
			e[3] = fmax(e[3], v[1] + v[2]);
			continue;
		}
		if (!starts(buf, "line ") && !starts(buf, "poly ")) {
			continue;
		}
		if (buf[0] == 'l') {
			h = strtod(s, &end) / 2;
			s = end;
		}
		for (;; k++) {
			double x = strtod(s, &end);

			if (end == s) {
				break;
			}
			e[k % 2] = fmin(e[k % 2], x - h);
			e[(k % 2) + 2] = fmax(e[(k % 2) + 2], x + h);
			s = end;
		}
	}
	(void)fclose(f);
	free(buf);
}

// A text record of a sheet, kept for the links of the flag after it.
struct span {
	char t[SPAN_LEN];
	double size, x, y;
};

// Writes a link of a box to page n of the PDF, as pdfmark.
static void link(double x0, double y0, double x1, double y1, int n)
{
	printf("[ /Rect [ %.3f %.3f %.3f %.3f ] /Border [ 0 0 0 ] /Page %d "
	       "/View [ /Fit ] /Subtype /Link /ANN pdfmark\n",
	       x0, y0, x1, y1, n);
}

// The spans of the port read so far, of the file drawn.
struct pen {
	struct span sp[SPANS];
	int nsp;
};

// The page a span of a port's list links to, other than k: its number,
// or 0.
static int span_page(const struct span *sp, int k)
{
	int to = 0;

	if (sp->t[0] >= '0' && sp->t[0] <= '9') {
		to = (int)strtol(sp->t, NULL, DECIMAL);
	}
	return to != k ? to : 0;
}

// The links of a port of the sheet at at on page k, as schview follows
// its clicks: its circle and name to the first page it lists other than
// k, each number of its list but k to that page; its spans are the text
// records before its flag record (vec(5)), the right of each measured
// in the stroke font.
static void port_links(const struct pen *pn, struct pt at, int k)
{
	const struct span *sp = pn->sp;
	int n = pn->nsp;
	int first = 0;

	for (int i = 1; i < n && !first; i++) {
		first = span_page(&sp[i], k);
	}
	for (int i = 0; i < n; i++) {
		int to = !i ? first : span_page(&sp[i], k);

		if (to) {
			link(sp[i].x, sp[i].y - (LINK_BELOW * sp[i].size),
			     sp[i].x + stroke_width(sp[i].t, sp[i].size),
			     sp[i].y + (LINK_ABOVE * sp[i].size), to);
		}
	}
	if (first && n) {
		double r = sp[0].size / 4;

		link(at.x - r, at.y - r, at.x + r, at.y + r, first);
	}
}

// How a file is drawn: its color, filled or outlined, the capital
// height of its names (0: none), its page (0 all), and whether its
// ports link.
struct look {
	const double *rgb;
	int outline;
	double names;
	int k;
	int links;
};

// Keeps the texts before a flag of a port as its spans, and writes the
// links of the port at its flag.
static void track_port(char *buf, const struct look *lk, struct pen *pn)
{
	int i = pn->nsp < SPANS ? pn->nsp : SPANS - 1;
	const char *r = buf;
	char net[NAME_LEN];
	char kind[KIND_LEN];
	struct pt at = { NAN, NAN };
	int sat = 0;

	if (scan(&r, "text %lf %*f %*d %lf %lf %n", &pn->sp[i].size,
		 &pn->sp[i].x, &pn->sp[i].y, &sat) == 3 &&
	    sat) {
		struct span *q = &pn->sp[i];

		buf[strcspn(buf, "\n")] = 0;
		copy_prefix(q->t, sizeof q->t, buf + sat, strlen(buf + sat));
		pn->nsp += pn->nsp < SPANS;
		return;
	}
	r = buf;
	if (scan(&r, "flag %511s %lf %lf %15s", net, &at.x, &at.y, kind) == 4 &&
	    !strcmp(kind, "port") && lk->links) {
		port_links(pn, at, lk->k);
	}
	pn->nsp = 0;
}

// Writes the points of a line or poly record from s as a path.
static void put_points(const char *s)
{
	char *end = NULL;
	int first = 1;

	for (;;) {
		double x = strtod(s, &end);
		double y = NAN;

		if (end == s) {
			break;
		}
		s = end;
		y = strtod(s, &end);
		s = end;
		printf(" %.4f %.4f %s", x, y, first ? "moveto" : "lineto");
		first = 0;
	}
}

// Draws a text record, or a name record's name, its capitals names
// high.
static void put_text(char *buf, double names)
{
	const char *r = buf;

	if (starts(buf, "text ")) {
		double size = NAN;
		double angle = NAN;
		double x = NAN;
		double y = NAN;
		int align = 0;
		int at = 0;

		if (scan(&r, "text %lf %lf %d %lf %lf %n", &size, &angle,
			 &align, &x, &y, &at) == TEXT_FIELDS &&
		    at > 0) {
			buf[strcspn(buf, "\n")] = 0;
			strokes(buf + at, size, angle, align, x, y);
		}
	} else if (starts(buf, "name") && names > 0) {
		char name[NAME_LEN];
		double b[4];

		if (scan(&r, "name %511s %lf %lf %lf %lf", name, &b[0], &b[1],
			 &b[2], &b[3]) == NAME_FIELDS &&
		    b[2] - b[0] < NAME_BOX_MOST) {
			printf("gsave 0 setgray\n");
			strokes(name, names, 0, 1, (b[0] + b[2]) / 2,
				((b[1] + b[3]) / 2) - (names / 2));
			printf("grestore\n");
		}
	}
}

// Draws a record of a file as lk has it; on a page, the ports link.
static void record(char *buf, const struct look *lk, struct pen *pn)
{
	char *s = buf + strcspn(buf, " \n");
	char *end = NULL;
	const double *rgb = lk->rgb;

	if (lk->k) {
		track_port(buf, lk, pn);
	} else {
		pn->nsp = 0;
	}
	if (starts(buf, "dark")) {
		printf("%.3f %.3f %.3f setrgbcolor\n", rgb[0], rgb[1], rgb[2]);
	} else if (starts(buf, "clear")) {
		if (!lk->outline) {
			printf("1 setgray\n");
		}
	} else if (starts(buf, "disk")) {
		double v[3] = { 0, 0, 0 };

		for (int n = 0; n < 3; n++) {
			v[n] = strtod(s, &end);
			s = end;
		}
		printf("newpath %.4f %.4f %.4f 0 360 arc closepath %s\n", v[0],
		       v[1], v[2], lk->outline ? "stroke" : "fill");
	} else if (starts(buf, "line")) {
		double w = strtod(s, &end);

		printf("%.4f setlinewidth newpath",
		       lk->outline ? OUTLINE_W : w);
		put_points(end);
		printf(" stroke\n");
	} else if (starts(buf, "poly")) {
		printf("newpath");
		put_points(s);
		printf(" closepath %s\n", lk->outline ? "stroke" : "fill");
	} else {
		put_text(buf, lk->names);
	}
}

// Clips off each segment of the line record at s, its width first.
static void clip_line(const char *s)
{
	char *end = NULL;
	double w = strtod(s, &end);
	double x0 = 0;
	double y0 = 0;
	int first = 1;

	s = end;
	// A segment at a time: the outline of one is simple.
	for (;;) {
		double x = strtod(s, &end);
		double y = NAN;

		if (end == s) {
			break;
		}
		s = end;
		y = strtod(s, &end);
		s = end;
		if (!first) {
			printf("newpath %.4f setlinewidth %.4f %.4f moveto "
			       "%.4f %.4f lineto strokepath Cbox eoclip "
			       "newpath\n",
			       w, x0, y0, x, y);
		}
		x0 = x;
		y0 = y;
		first = 0;
	}
}

// Clips off the copper a clear record takes (a poly, a disk or a line,
// each segment of it): the clip is what it was less the record, as the
// even-odd interior of the box Cbox and the record's outline.
static void clip_off(const char *buf)
{
	const char *s = buf + strcspn(buf, " \n");

	if (starts(buf, "disk")) {
		char *end = NULL;
		double v[3];

		for (int n = 0; n < 3; n++) {
			v[n] = strtod(s, &end);
			s = end;
		}
		printf("newpath Cbox %.4f %.4f moveto %.4f %.4f %.4f 0 360 arc "
		       "closepath eoclip newpath\n",
		       v[0] + v[2], v[1], v[0], v[1], v[2]);
	} else if (starts(buf, "poly")) {
		printf("newpath Cbox");
		put_points(s);
		printf(" closepath eoclip newpath\n");
	} else if (starts(buf, "line")) {
		clip_line(s);
	}
}

// The records of a file: its lines, and how many of them clear.
struct lines {
	char **line;
	size_t n;
	size_t nclear;
};

// Reads the records of page k (from 1) of the file, or all for k 0.
static struct lines read_lines(const char *path, int k)
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE);
	struct lines ls = { NULL, 0, 0 };
	int now = 1;
	int seen = 0;

	if (!f || !buf) {
		fail(path, "cannot read");
	}
	while (fgets(buf, LINE, f)) {
		char **more = NULL;

		if (starts(buf, "page ") && seen++) {
			now++;
		}
		if (k && now != k) {
			continue;
		}
		more = (char **)realloc((void *)ls.line,
					(ls.n + 1) * sizeof *ls.line);
		if (!more) {
			fail(path, "out of memory");
		}
		ls.line = more;
		ls.line[ls.n] = strdup(buf);
		if (!ls.line[ls.n]) {
			fail(path, "out of memory");
		}
		if (starts(buf, "clear")) {
			ls.nclear++;
		}
		ls.n++;
	}
	(void)fclose(f);
	free(buf);
	return ls;
}

// The first record from i of ls that starts with what, or the end.
static size_t next_of(const struct lines *ls, size_t i, const char *what)
{
	size_t j = i;

	while (j < ls->n && !starts(ls->line[j], what)) {
		j++;
	}
	return j;
}

// A run of records, i up to j.
struct run {
	size_t i, j;
};

// Draws the run ru of dark records of ls, clipped off every clear
// record after it.
static void draw_run(const struct lines *ls, struct run ru,
		     const struct look *lk, struct pen *pn)
{
	size_t i = ru.i;
	size_t j = ru.j;
	int dark = 1;

	printf("gsave\n");
	for (size_t m = j; m < ls->n; m++) {
		if (starts(ls->line[m], "dark")) {
			dark = 1;
		} else if (starts(ls->line[m], "clear")) {
			dark = 0;
		}
		if (!dark) {
			clip_off(ls->line[m]);
		}
	}
	for (size_t m = i; m < j; m++) {
		record(ls->line[m], lk, pn);
	}
	printf("grestore\n");
}

// Draws a file as lk has it: its page lk->k (from 1), or all of it for
// k 0.  What it clears (vec(5)) erases what it drew before and nothing
// of the other files: each run of its dark records is drawn clipped off
// every clear record after it.
static void draw(const char *path, const struct look *lk)
{
	struct lines ls = read_lines(path, lk->k);
	struct pen pn = { .nsp = 0 };

	printf("%.3f %.3f %.3f setrgbcolor\n", lk->rgb[0], lk->rgb[1],
	       lk->rgb[2]);
	if (!ls.nclear || lk->outline) {
		for (size_t i = 0; i < ls.n; i++) {
			record(ls.line[i], lk, &pn);
		}
	} else {
		// The box of the clip, about all the page shows.
		printf("gsave clippath pathbbox grestore /Cy1 exch def "
		       "/Cx1 exch def /Cy0 exch def /Cx0 exch def\n"
		       "/Cbox { Cx0 Cy0 moveto Cx1 Cy0 lineto Cx1 Cy1 lineto "
		       "Cx0 Cy1 lineto closepath } def\n");
		for (size_t i = 0, j = 0; i < ls.n; i = j) {
			if (starts(ls.line[i], "clear")) {
				// Its records clip the runs before it.
				j = next_of(&ls, i + 1, "dark");
				continue;
			}
			j = next_of(&ls, i, "clear");
			draw_run(&ls, (struct run){ i, j }, lk, &pn);
		}
	}
	for (size_t i = 0; i < ls.n; i++) {
		free(ls.line[i]);
	}
	free((void *)ls.line);
}

// The parts the files name: by name, their designators and boxes.
struct part {
	char name[NAME_LEN], ref[REF_LEN];
	double b[4];
	int boxed;
};

struct parts {
	struct part *p;
	size_t n;
};

static struct part *part_of(struct parts *ps, const char *name)
{
	struct part *more = NULL;

	for (size_t i = 0; i < ps->n; i++) {
		if (!strcmp(ps->p[i].name, name)) {
			return &ps->p[i];
		}
	}
	more = realloc(ps->p, (ps->n + 1) * sizeof *ps->p);
	if (!more) {
		fail("vecps", "out of memory");
	}
	ps->p = more;
	memset(&ps->p[ps->n], 0, sizeof *ps->p);
	copy_prefix(ps->p[ps->n].name, sizeof ps->p[0].name, name,
		    strlen(name));
	ps->p[ps->n].b[0] = ps->p[ps->n].b[1] = NONE_YET;
	ps->p[ps->n].b[2] = ps->p[ps->n].b[3] = -NONE_YET;
	return &ps->p[ps->n++];
}

// Reads the parts of a file: their brackets (part NAME REF) and boxes.
static void read_parts(struct parts *ps, const char *path)
{
	FILE *f = fopen(path, "r");
	char buf[PARTS_LINE];
	char name[NAME_LEN];
	char ref[REF_LEN];
	double b[4];

	if (!f) {
		fail(path, "cannot read");
	}
	while (fgets(buf, sizeof buf, f)) {
		const char *r = buf;

		if (scan(&r, "part %511s %63s", name, ref) == 2) {
			copy_prefix(part_of(ps, name)->ref, sizeof ref, ref,
				    strlen(ref));
			continue;
		}
		r = buf;
		if (scan(&r, "name %511s %lf %lf %lf %lf", name, &b[0], &b[1],
			 &b[2], &b[3]) == NAME_FIELDS) {
			struct part *p = part_of(ps, name);

			for (int k = 0; k < 4; k++) {
				p->b[k] = k < 2 ? fmin(p->b[k], b[k]) :
						  fmax(p->b[k], b[k]);
			}
			p->boxed = 1;
		}
	}
	(void)fclose(f);
}

// How a label is written: turned angle degrees, its capitals size high
// (in the units of the files), in the gray.
struct label_style {
	double angle, size, gray;
};

// Writes a label: text, its middle at at, as st has it, upright.
static void label(const char *text, struct pt at, struct label_style st)
{
	double a = fmod(st.angle + FULL_TURN, FULL_TURN);
	double size = st.size;
	double r = NAN;

	if (a > QUARTER && a <= THREE_QUARTERS) {
		a -= HALF_TURN; // Upright.
	}
	r = a * PI / HALF_TURN;
	printf("gsave %.3f setgray\n", st.gray);
	strokes(text, size, a, 1, at.x + (size / 2 * sin(r)),
		at.y - (size / 2 * cos(r)));
	printf("grestore\n");
}

static int inside(const double w[4], double x, double y)
{
	return x >= w[0] && x <= w[2] && y >= w[1] && y <= w[3];
}

// The capital height of a label on the page: fitted to what it names
// (fit, in the units of the files), but no smaller than LABEL_LEAST
// points and no larger than LABEL_MOST, so it is read on the page.
static double fitted(double fit, double scale)
{
	return fmin(fmax(fit * scale, LABEL_LEAST), LABEL_MOST) / scale;
}

// A region named on: its window, and its scale on the page.
struct region {
	const double *w;
	double scale;
};

// Names a track of net, its width v[0] and its points v[1] .. v[n-1]:
// along its longest segment, in its middle.
static void name_track(const char *net, const double *v, int n,
		       const struct region *rg)
{
	int best = 1;
	double len = -1;

	for (int k = 1; k + 3 < n; k += 2) {
		double l = hypot(v[k + 2] - v[k], v[k + 3] - v[k + 1]);

		if (l > len) {
			len = l;
			best = k;
		}
	}
	{
		double x = (v[best] + v[best + 2]) / 2;
		double y = (v[best + 1] + v[best + 3]) / 2;
		double a = atan2(v[best + 3] - v[best + 1],
				 v[best + 2] - v[best]) *
			   HALF_TURN / PI;
		double size = fitted(v[0] * TRACK_FIT, rg->scale);

		if (inside(rg->w, x, y) && len > stroke_width(net, size)) {
			struct label_style st = { a, size, NET_GRAY };

			label(net, (struct pt){ x, y }, st);
		}
	}
}

// Names a pad of net, a disk of 3 numbers v or a polygon of n: in its
// middle, along its longer side.
static void name_pad(const char *net, const double *v, int n,
		     const struct region *rg)
{
	double b[4] = { NONE_YET, NONE_YET, -NONE_YET, -NONE_YET };

	if (n == 3) {
		b[0] = v[0] - v[2];
		b[1] = v[1] - v[2];
		b[2] = v[0] + v[2];
		b[3] = v[1] + v[2];
	}
	for (int k = 0; n != 3 && k + 1 < n; k += 2) {
		b[0] = fmin(b[0], v[k]);
		b[1] = fmin(b[1], v[k + 1]);
		b[2] = fmax(b[2], v[k]);
		b[3] = fmax(b[3], v[k + 1]);
	}
	if (inside(rg->w, (b[0] + b[2]) / 2, (b[1] + b[3]) / 2)) {
		double bw = b[2] - b[0];
		double bh = b[3] - b[1];
		double lo = fmin(bw, bh);
		double hi = fmax(bw, bh);
		double tw = stroke_width(net, 1) + PAD_ROOM;
		struct label_style st = {
			bh > bw ? QUARTER : 0,
			fitted(fmin(lo * PAD_FIT, hi / tw), rg->scale), NET_GRAY
		};

		label(net, (struct pt){ (b[0] + b[2]) / 2, (b[1] + b[3]) / 2 },
		      st);
	}
}

// Takes a record of the brackets of nets and parts: net NAME [class
// CLASS] names the net, part NAME opens a part and part closes it.
// Returns 0 for another record.
static int bracket(const char *buf, char net[NAME_LEN], int *in_part)
{
	if (starts(buf, "net") && (!buf[3] || buf[3] == ' ')) {
		// net NAME [class CLASS]: the name.
		copy_prefix(net, NAME_LEN, buf + (buf[3] ? 4 : 0),
			    buf[3] ? strcspn(buf + 4, " ") : 0);
		return 1;
	}
	if (starts(buf, "part") && (!buf[4] || buf[4] == ' ')) {
		*in_part = buf[4] == ' ';
		return 1;
	}
	return 0;
}

// Names what lies in the window: each part's designator in its box, and
// each net's name along its tracks and in its pads.
static void names_in(const char *path, const struct region *rg)
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE);
	double *v = malloc(NUMBERS * sizeof *v);
	char net[NAME_LEN] = "";
	int in_part = 0;

	if (!f || !buf || !v) {
		fail(path, "cannot read");
	}
	while (fgets(buf, LINE, f)) {
		int n = 0;
		char *s = buf + strcspn(buf, " \n");
		char *end = NULL;

		buf[strcspn(buf, "\n")] = 0;
		if (bracket(buf, net, &in_part) || !*net ||
		    (!starts(buf, "line ") && !starts(buf, "poly ") &&
		     !starts(buf, "disk "))) {
			continue;
		}
		while (n < NUMBERS && (v[n] = strtod(s, &end), end != s)) {
			s = end;
			n++;
		}
		if (starts(buf, "line ") && n >= TRACK_NUMBERS && !in_part) {
			name_track(net, v, n, rg);
		} else if (in_part &&
			   (n >= PAD_NUMBERS || (n == 3 && buf[0] == 'd'))) {
			name_pad(net, v, n, rg);
		}
	}
	(void)fclose(f);
	free(buf);
	free(v);
}

static void usage(void)
{
	(void)fprintf(
		stderr,
		"usage: vecps [-w X0 Y0 X1 Y1 | -r X0 Y0 X1 Y1 | -p PART]... "
		"[-n] FILE[=R,G,B[,line]]... > out.ps\n");
	exit(2);
}

// The options: the window and whether it is given, the names of -n,
// whether a region is rendered, the parts of -p, and the first file.
struct opts {
	double win[4];
	int have, names, region;
	char **want;
	size_t nwant;
	int a0;
};

// Reads the options of argv.
static void read_opts(int argc, char **argv, struct opts *o)
{
	int i = 1;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if ((!strcmp(argv[i], "-w") || !strcmp(argv[i], "-r")) &&
		    i + 4 < argc) {
			o->region = o->region || argv[i][1] == 'r';
			for (int k = 0; k < 4; k++) {
				o->win[k] = strtod(argv[++i], NULL);
			}
			o->have = 1;
		} else if (!strcmp(argv[i], "-p") && i + 1 < argc) {
			char *save = NULL;

			o->region = 1;
			for (char *t = strtok_r(argv[++i], ",", &save); t;
			     t = strtok_r(NULL, ",", &save)) {
				char **more = (char **)realloc(
					(void *)o->want,
					(o->nwant + 1) * sizeof *o->want);

				if (!more) {
					fail("vecps", "out of memory");
				}
				o->want = more;
				o->want[o->nwant++] = t;
			}
		} else if (!strcmp(argv[i], "-n")) {
			o->names = 1;
		} else {
			usage();
		}
	}
	if (i == argc) {
		usage();
	}
	o->a0 = i;
}

// The window of the parts of -p: the box of the parts named, grown by a
// fifth of its size and 1 unit.
static void parts_window(const struct parts *ps, struct opts *o)
{
	double b[4] = { NONE_YET, NONE_YET, -NONE_YET, -NONE_YET };
	double g = NAN;

	for (size_t k = 0; k < o->nwant; k++) {
		int found = 0;

		for (size_t p = 0; p < ps->n; p++) {
			const struct part *q = &ps->p[p];

			if (!q->boxed || (strcmp(q->name, o->want[k]) != 0 &&
					  strcmp(q->ref, o->want[k]) != 0)) {
				continue;
			}
			for (int j = 0; j < 4; j++) {
				b[j] = j < 2 ? fmin(b[j], q->b[j]) :
					       fmax(b[j], q->b[j]);
			}
			found = 1;
		}
		if (!found) {
			fail(o->want[k], "no such part");
		}
	}
	g = fmax(b[2] - b[0], b[3] - b[1]) / GROW_SHARE + 1;
	o->win[0] = b[0] - g;
	o->win[1] = b[1] - g;
	o->win[2] = b[2] + g;
	o->win[3] = b[3] + g;
	o->have = 1;
}

// The path of the file of argument k.
static void path_of(char path[PATH_LEN], const char *arg)
{
	copy_prefix(path, PATH_LEN, arg, strcspn(arg, "="));
}

// Draws the files of argv from o->a0 on, as page pg of k (0 all), at
// the scale.
static void draw_files(char **argv, int argc, const struct opts *o,
		       const struct look *page)
{
	for (int k = o->a0; k < argc; k++) {
		char path[PATH_LEN];
		char *eq = strchr(argv[k], '=');
		double rgb[3] = { 0, 0, 0 };
		struct look lk = *page;
		const char *r = eq ? eq + 1 : NULL;

		copy_prefix(path, sizeof path, argv[k],
			    eq ? (size_t)(eq - argv[k]) : strlen(argv[k]));
		if (r &&
		    scan(&r, "%lf,%lf,%lf", &rgb[0], &rgb[1], &rgb[2]) != 3) {
			fail(argv[k], "FILE[=R,G,B[,line]]");
		}
		lk.rgb = rgb;
		lk.outline = eq && strstr(eq, ",line") != NULL;
		lk.names = k == o->a0 ? page->names : 0;
		lk.links = k == o->a0;
		draw(path, &lk);
	}
}

// Names the designators of the parts in the window.
static void name_parts(const struct parts *ps, const struct region *rg)
{
	for (size_t p = 0; p < ps->n; p++) {
		const struct part *q = &ps->p[p];
		double x = (q->b[0] + q->b[2]) / 2;
		double y = (q->b[1] + q->b[3]) / 2;

		if (q->boxed && *q->ref && inside(rg->w, x, y)) {
			double bw = q->b[2] - q->b[0];
			double bh = q->b[3] - q->b[1];
			double tw = (REF_SPACE * stroke_width(q->ref, 1)) +
				    REF_ROOM;
			struct label_style st = {
				bh > REF_TALL * bw ? QUARTER : 0,
				fitted(fmin(fmin(bw, bh) * REF_FIT,
					    fmax(bw, bh) / tw),
				       rg->scale),
				0
			};

			label(q->ref, (struct pt){ x, y }, st);
		}
	}
}

// The default window: the extent of all the files, and the pages of the
// first file and the size of its first, into size.  Returns the pages.
static int whole_window(char **argv, int argc, struct opts *o, double size[2])
{
	char path[PATH_LEN];
	int page = 0;

	path_of(path, argv[o->a0]);
	page = page_of(path, 1, size);
	o->win[0] = o->win[1] = NONE_YET;
	o->win[2] = o->win[3] = -NONE_YET;
	for (int k = o->a0; k < argc; k++) {
		path_of(path, argv[k]);
		extent(path, o->win);
	}
	return page;
}

int main(int argc, char **argv)
{
	struct opts o;
	struct parts ps = { NULL, 0 };
	double size[2] = { LETTER_W, LETTER_H };
	double scale = NAN;
	int page = 0;

	memset(&o, 0, sizeof o);
	read_opts(argc, argv, &o);
	for (int k = o.a0; o.region && k < argc; k++) {
		char path[PATH_LEN];

		path_of(path, argv[k]);
		read_parts(&ps, path);
	}
	if (o.nwant) {
		parts_window(&ps, &o);
	}
	if (!o.have) {
		page = whole_window(argv, argc, &o, size);
	}
	printf("%%!PS-Adobe-3.0\n%%%%BoundingBox: 0 0 %.0f %.0f\n%%%%Pages: %d\n"
	       "%%%%EndComments\n",
	       size[0], size[1], page ? page : 1);
	printf("/pdfmark where { pop } { userdict /pdfmark /cleartomark load "
	       "put } ifelse\n");
	// A sheet is drawn page by page, each at its size and scale 1.
	for (int pg = 1; pg <= (page ? page : 1); pg++) {
		double pw = NAN;
		double ph = NAN;
		struct look lk = { NULL, 0, 0, page ? pg : 0, 0 };
		struct region rg = { o.win, 0 };

		if (page) {
			char path[PATH_LEN];

			path_of(path, argv[o.a0]);
			(void)page_of(path, pg, size);
			o.win[0] = o.win[1] = 0;
			o.win[2] = size[0];
			o.win[3] = size[1];
			scale = 1;
		} else {
			scale = fmin(
				(size[0] - PAGE_EDGE) / (o.win[2] - o.win[0]),
				(size[1] - PAGE_EDGE) / (o.win[3] - o.win[1]));
		}
		pw = size[0];
		ph = size[1];
		printf("%%%%Page: %d %d\n", pg, pg);
		printf("<< /PageSize [ %.0f %.0f ] >> setpagedevice\n", pw, ph);
		printf("1 setlinecap 1 setlinejoin\n");
		if (!page) {
			printf("newpath 18 18 %.0f %.0f rectclip\n",
			       pw - PAGE_EDGE, ph - PAGE_EDGE);
		}
		printf("%.4f %.4f translate %.6f dup scale %.4f %.4f "
		       "translate\n",
		       pw / 2, ph / 2, scale, -(o.win[0] + o.win[2]) / 2,
		       -(o.win[1] + o.win[3]) / 2);
		lk.names = o.names ? NAME_PT / scale : 0;
		draw_files(argv, argc, &o, &lk);
		rg.scale = scale;
		for (int k = o.a0; o.region && k < argc; k++) {
			char path[PATH_LEN];

			path_of(path, argv[k]);
			names_in(path, &rg);
		}
		if (o.region) {
			name_parts(&ps, &rg);
		}
		printf("showpage\n");
	}
	printf("%%%%EOF\n");
	free(ps.p);
	free((void *)o.want);
	return 0;
}
