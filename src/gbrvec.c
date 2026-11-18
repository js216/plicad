// SPDX-License-Identifier: GPL-3.0-only
// gbrvec.c -- Turn a Gerber or Excellon file into plain vectors
// Copyright (c) 2026 Jakob Kastelic

// Usage: gbrvec FILE > FILE.vec
//
// Reads an RS-274X (Gerber) file, or an Excellon drill file, and writes
// what it draws as vec(5): one record per line, in mm,
//   layer NAME              the layer, first: KiCad's name of the file
//                           function (TF.FileFunction) of Gerber X2,
//                           F.Cu, In1.Cu, B.Cu, F.Mask, Edge.Cuts, ...,
//                           and Drill for a drill file
//   dark | clear            the polarity of what follows
//   disk X Y R              a filled circle
//   line W X Y X Y ...      a stroke of width W, round at its ends
//   poly X Y X Y ...        a filled polygon
//   name NAME X0 Y0 X1 Y1   the box round all a part's objects
//   zone NET ... end        the fill of a zone of copper
//   part NAME ... part      the objects of a part
// A part's objects are those under its component attribute (TO.C) or
// a pad attribute (TO.P, by its part);
// the names come last, sorted.  A zone is a region under a net
// attribute (TO.N) and what follows it under that attribute.  Gerber: the format and units (FS, MO),
// the standard apertures (C, R, O) and aperture macros (primitives 1,
// 4, 5, 20 and 21, with the arithmetic of their parameters), draws,
// flashes, arcs (G02, G03, as short lines) and regions (G36, G37), and
// the polarity (LP).  Excellon: tools and holes, as disks.

#include "util.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define HALF_TURN 180.0 // In degrees,
#define FULL_TURN 360.0 // and a full turn.
#define MM_PER_INCH 25.4
#define SCALE_FIRST 1e-6 // The scale of a coordinate before an FS.
#define NONE_YET 1e300 // Beyond any box.
enum {
	DECIMAL = 10, // The base of decimal numbers.
	ARC = 256, // Segments to a full circle: smooth however far a viewer zooms.
	TYPE_LEN = 64, // Room for the type of an aperture,
	APER_PARAMS = 16, // its parameters,
	MACRO_NAME_LEN = 64, // the name of a macro,
	MACRO_LEN = 8192, // its primitives,
	PRIM_PARAMS = 256, // and the parameters of one.
	NAME_LEN = 512, // Room for the name of a part or a net.
	BOX_XY = 8, // The coordinates of the corners of a rectangle.
	CONTOUR_FIRST = 256, // The first room for the points of a region.
	TOOLS = 1000, // The tools of a drill file.
	FUNCTION_LEN = 128, // Room for a file function,
	LAYER_LEN = 64 // and for the name of a layer.
};
// The aperture macro primitives: their codes, and the parameters each
// needs; the place of their parameters.
enum {
	PRIM_CIRCLE = 1,
	PRIM_OUTLINE = 4,
	PRIM_POLYGON = 5,
	PRIM_VECTOR_LINE = 20,
	PRIM_CENTER_LINE = 21,
	CIRCLE_PARAMS = 5, // 1, exposure, diameter, x, y
	OUTLINE_PARAMS = 5, // 4, exposure, n, x0, y0, ..., rotation
	OUTLINE_XY = 3,
	POLYGON_PARAMS = 6, // 5, exposure, n, x, y, diameter, [rotation]
	POLYGON_D = 5,
	POLYGON_ROT = 6,
	POLYGON_MOST = 64,
	VECTOR_PARAMS = 7, // 20, exposure, width, x1, y1, x2, y2, [rotation]
	VECTOR_X2 = 5,
	VECTOR_Y2 = 6,
	CENTER_PARAMS = 6, // 21, exposure, width, height, x, y, [rotation]
	CENTER_Y = 5,
	CENTER_ROT = 6
};

struct aper {
	int code;
	char type[TYPE_LEN];
	double p[APER_PARAMS];
	int np;
};

struct macro {
	char name[MACRO_NAME_LEN];
	char body[MACRO_LEN]; // The primitives, '*' between them.
};

struct named {
	char *name;
	double box[4];
};

// A point, in mm.
struct pt {
	double x, y;
};

// The file being read: its apertures and macros, the boxes of the parts
// named, the points of the region being read, the part and the net
// under way, whether in a zone and whether it has cleared, the units,
// and the polarity written last (-1 none yet).
struct gbr {
	const char *file;
	struct aper *aps;
	size_t naps;
	struct macro *macs;
	size_t nmacs;
	struct named *names;
	size_t nnames;
	double *contour;
	size_t nc, cap;
	char current_name[NAME_LEN];
	char net[NAME_LEN];
	int in_zone, zone_clear;
	double unit, scale;
	int dark;
};

// Prints the message and exits; as a macro, the exit shows to the
// analyzer.
#define fail(g, msg) \
	((void)fprintf(stderr, "%s: %s\n", (g)->file, msg), exit(2))

// The whole number of steps at or over v.
static int steps(double v)
{
	double c = ceil(v);

	return (int)c;
}

static void *grow(const struct gbr *g, void *p, size_t n, size_t size)
{
	void *q = realloc(p, (n + 1) * size);

	if (!q) {
		fail(g, "out of memory");
	}
	memset((char *)q + (n * size), 0, size);
	return q;
}

// Copies the first n bytes of src, or as many as fit, into dst.
static void copy_prefix(char *dst, size_t size, const char *src, size_t n)
{
	n = n < size - 1 ? n : size - 1;
	memcpy(dst, src, n);
	dst[n] = 0;
}

// Grows the box of the part being drawn, if any, by p and r round it.
static void touch(struct gbr *g, struct pt p, double r)
{
	size_t i = 0;
	struct named *nm = NULL;

	if (!*g->current_name) {
		return;
	}
	for (i = 0;
	     i < g->nnames && strcmp(g->names[i].name, g->current_name) != 0;
	     i++) {
	}
	if (i == g->nnames) {
		size_t len = strlen(g->current_name) + 1;

		g->names = grow(g, g->names, g->nnames, sizeof *g->names);
		g->names[i].name = malloc(len);
		if (!g->names[i].name) {
			fail(g, "out of memory");
		}
		memcpy(g->names[i].name, g->current_name, len);
		g->names[i].box[0] = g->names[i].box[1] = NONE_YET;
		g->names[i].box[2] = g->names[i].box[3] = -NONE_YET;
		g->nnames++;
	}
	nm = &g->names[i];
	nm->box[0] = fmin(nm->box[0], p.x - r);
	nm->box[1] = fmin(nm->box[1], p.y - r);
	nm->box[2] = fmax(nm->box[2], p.x + r);
	nm->box[3] = fmax(nm->box[3], p.y + r);
}

static void end_zone(struct gbr *g)
{
	if (g->in_zone) {
		puts("end");
		g->in_zone = g->zone_clear = 0;
	}
}

static void set_dark(struct gbr *g, int d)
{
	if (g->in_zone && d && g->zone_clear) {
		end_zone(g);
	}
	g->zone_clear = g->zone_clear || (g->in_zone && !d);
	if (d != g->dark) {
		puts(d ? "dark" : "clear");
		g->dark = d;
	}
}

static void disk(struct gbr *g, struct pt p, double r)
{
	printf("disk %.6g %.6g %.6g\n", p.x, p.y, r);
	touch(g, p, r);
}

static void poly(struct gbr *g, const double *xy, int n)
{
	printf("poly");
	for (size_t i = 0; i < (size_t)n; i++) {
		struct pt p = { xy[2 * i], xy[(2 * i) + 1] };

		printf(" %.6g %.6g", p.x, p.y);
		touch(g, p, 0);
	}
	printf("\n");
}

// A rectangle w by h at (x, y).
struct box {
	double x, y, w, h;
};

// The rectangle b, turned rot degrees round the origin.
static void rect(struct gbr *g, struct box b, double rot)
{
	double xy[BOX_XY];
	double c = cos(rot * PI / HALF_TURN);
	double s = sin(rot * PI / HALF_TURN);
	const double dx[4] = { -1, 1, 1, -1 };
	const double dy[4] = { -1, -1, 1, 1 };

	for (size_t i = 0; i < 4; i++) {
		double px = b.x + (dx[i] * b.w / 2);
		double py = b.y + (dy[i] * b.h / 2);

		xy[2 * i] = px * c - py * s;
		xy[(2 * i) + 1] = px * s + py * c;
	}
	poly(g, xy, 4);
}

// A group of the arithmetic of a macro parameter being read: the sum so
// far and the operator after it (0 none yet), the product so far and
// the operator after it, and whether the group is negated.
struct group {
	double sum, prod;
	char addop, mulop;
	int neg;
};

// A number, or the parameter $n of vars (0 if there is none such), at
// *s, which is set past it.
static double number(const char **s, const double *vars)
{
	char *end = NULL;
	double v = NAN;

	if (**s == '$') {
		long k = strtol(*s + 1, &end, DECIMAL);

		*s = end;
		return k >= 1 && k <= APER_PARAMS ? vars[k - 1] : 0;
	}
	v = strtod(*s, &end);
	*s = end;
	return v;
}

// Folds the factor v into the product of group gr.
static void fold_factor(struct group *gr, double v)
{
	if (gr->mulop == 'x') {
		gr->prod *= v;
	} else if (gr->mulop == '/') {
		gr->prod /= v;
	} else {
		gr->prod = v;
	}
	gr->mulop = 0;
}

// Folds the product of group gr into its sum.
static void fold_term(struct group *gr)
{
	if (gr->addop == '+') {
		gr->sum += gr->prod;
	} else if (gr->addop == '-') {
		gr->sum -= gr->prod;
	} else {
		gr->sum = gr->prod;
	}
	gr->addop = 0;
}

// The signs before an operand at *s, which is set past them and the
// blanks: whether they negate it.
static int signs(const char **s)
{
	int neg = 0;

	for (;;) {
		while (**s == ' ') {
			(*s)++;
		}
		if (**s == '-') {
			neg = !neg;
		} else if (**s != '+') {
			return neg;
		}
		(*s)++;
	}
}

// The arithmetic of a macro parameter at *s: numbers, $n of vars, + - x
// / and (), x and / before + and -, each left to right; a group open at
// the end closes there.  *s is set past it.
// After an operand v at *p of the groups st[0] .. st[*n-1]: the
// operator after it, or the groups it closes.  Returns 1 if an operand
// follows, 0 at the end, with the value in *v.
static int after_operand(struct group *st, size_t *n, const char **p, double *v)
{
	for (;;) {
		struct group *gr = &st[*n - 1];

		fold_factor(gr, *v);
		if (**p == 'x' || **p == 'X' || **p == '/') {
			gr->mulop = **p == '/' ? '/' : 'x';
			(*p)++;
			return 1;
		}
		fold_term(gr);
		if (**p == '+' || **p == '-') {
			gr->addop = *(*p)++;
			return 1;
		}
		*v = gr->sum;
		if (*n == 1) {
			return 0;
		}
		if (**p == ')') {
			(*p)++;
		}
		*v = gr->neg ? -*v : *v;
		(*n)--;
	}
}

static double calc(const struct gbr *g, const char **s, const double *vars)
{
	struct group *st = grow(g, NULL, 0, sizeof *st);
	size_t n = 1;
	const char *p = *s;
	double v = NAN;

	for (;;) {
		int neg = signs(&p);

		if (*p == '(') {
			p++;
			st = grow(g, st, n, sizeof *st);
			st[n++].neg = neg;
			continue;
		}
		v = number(&p, vars);
		v = neg ? -v : v;
		if (!after_operand(st, &n, &p, &v)) {
			break;
		}
	}
	free(st);
	*s = p;
	return v;
}

// A primitive of a macro flashed: its parameters, a[0] its code, and
// where it is flashed.
struct prim {
	const double *a;
	int n;
	struct pt at;
};

// The outline primitive: a polygon of a[2] points, turned.
static void prim_outline(struct gbr *g, const struct prim *pr)
{
	const double *a = pr->a;
	int k = (int)a[2];
	double *xy = malloc((size_t)(k + 1) * 2 * sizeof *xy);
	double rot = pr->n > OUTLINE_PARAMS + 2 * k ?
			     a[OUTLINE_PARAMS + (2 * k)] :
			     0;
	double c = cos(rot * PI / HALF_TURN);
	double s = sin(rot * PI / HALF_TURN);

	if (!xy) {
		fail(g, "out of memory");
	}
	for (int i = 0; i <= k && 4 + 2 * i < pr->n; i++) {
		size_t j = (size_t)i;
		double px = a[OUTLINE_XY + (2 * j)] * g->unit;
		double py = a[OUTLINE_XY + 1 + (2 * j)] * g->unit;

		xy[2 * j] = pr->at.x + px * c - py * s;
		xy[(2 * j) + 1] = pr->at.y + px * s + py * c;
	}
	poly(g, xy, k);
	free(xy);
}

// The vector line primitive: a stroke of width a[2], square at its ends.
static void prim_vector_line(struct gbr *g, const struct prim *pr)
{
	const double *a = pr->a;
	double unit = g->unit;
	double x1 = a[3] * unit;
	double y1 = a[4] * unit;
	double x2 = a[VECTOR_X2] * unit;
	double y2 = a[VECTOR_Y2] * unit;
	double w = a[2] * unit;
	double l = hypot(x2 - x1, y2 - y1);
	double ux = l > 0 ? (x2 - x1) / l : 1;
	double uy = l > 0 ? (y2 - y1) / l : 0;
	double xy[BOX_XY] = { x1 - (uy * w / 2), y1 + (ux * w / 2),
			      x2 - (uy * w / 2), y2 + (ux * w / 2),
			      x2 + (uy * w / 2), y2 - (ux * w / 2),
			      x1 + (uy * w / 2), y1 - (ux * w / 2) };

	for (size_t i = 0; i < 4; i++) {
		xy[2 * i] += pr->at.x;
		xy[(2 * i) + 1] += pr->at.y;
	}
	poly(g, xy, 4);
}

// The center line primitive: a rectangle a[2] by a[3] at (a[4], a[5]),
// turned.
static void prim_center_line(struct gbr *g, const struct prim *pr)
{
	const double *a = pr->a;
	double unit = g->unit;
	double xy[BOX_XY];
	double rot = pr->n > CENTER_PARAMS ? a[CENTER_ROT] : 0;
	double c = cos(rot * PI / HALF_TURN);
	double s = sin(rot * PI / HALF_TURN);
	const double dx[4] = { -1, 1, 1, -1 };
	const double dy[4] = { -1, -1, 1, 1 };

	for (size_t i = 0; i < 4; i++) {
		double px = (a[4] * unit) + (dx[i] * a[2] * unit / 2);
		double py = (a[CENTER_Y] * unit) + (dy[i] * a[3] * unit / 2);

		xy[2 * i] = pr->at.x + px * c - py * s;
		xy[(2 * i) + 1] = pr->at.y + px * s + py * c;
	}
	poly(g, xy, 4);
}

// The polygon primitive: a regular polygon of a[2] corners (3 to
// POLYGON_MOST) on a circle of diameter a[5] at (a[3], a[4]), turned.
static void prim_polygon(struct gbr *g, const struct prim *pr)
{
	const double *a = pr->a;
	double unit = g->unit;
	int k = (int)a[2];
	double xy[2 * POLYGON_MOST] = { 0 };
	double rot = pr->n > POLYGON_ROT ? a[POLYGON_ROT] : 0;

	if (k < 3) {
		k = 3;
	} else if (k > POLYGON_MOST) {
		k = POLYGON_MOST;
	}
	for (int i = 0; i < k; i++) {
		double t = (rot + FULL_TURN * i / k) * PI / HALF_TURN;
		size_t j = (size_t)i;

		xy[2 * j] = pr->at.x + a[3] * unit +
			    a[POLYGON_D] * unit / 2 * cos(t);
		xy[(2 * j) + 1] = pr->at.y + a[4] * unit +
				  a[POLYGON_D] * unit / 2 * sin(t);
	}
	poly(g, xy, k);
}

// Flashes the primitive pr, if it is one of those handled.
static void flash_prim(struct gbr *g, const struct prim *pr)
{
	const double *a = pr->a;
	int code = (int)a[0];
	int n = pr->n;

	if (code == PRIM_CIRCLE && n >= CIRCLE_PARAMS) {
		struct pt p = { pr->at.x + (a[3] * g->unit),
				pr->at.y + (a[4] * g->unit) };

		disk(g, p, a[2] * g->unit / 2);
	} else if (code == PRIM_OUTLINE && n >= OUTLINE_PARAMS) {
		prim_outline(g, pr);
	} else if (code == PRIM_VECTOR_LINE && n >= VECTOR_PARAMS) {
		prim_vector_line(g, pr);
	} else if (code == PRIM_CENTER_LINE && n >= CENTER_PARAMS) {
		prim_center_line(g, pr);
	} else if (code == PRIM_POLYGON && n >= POLYGON_PARAMS) {
		prim_polygon(g, pr);
	}
}

// Reads the parameters of the primitive at s into a, up to PRIM_PARAMS;
// returns their number.
static int prim_params(const struct gbr *g, const char *s, const double *p,
		       double *a)
{
	int n = 0;

	while (*s && n < PRIM_PARAMS) {
		a[n++] = calc(g, &s, p);
		while (*s == ' ' || *s == '\n' || *s == '\r') {
			s++;
		}
		if (*s != ',') {
			break;
		}
		s++;
	}
	return n;
}

// Flashes macro m with parameters p at at.
static void flash_macro(struct gbr *g, const struct macro *m, const double *p,
			struct pt at)
{
	size_t len = strlen(m->body) + 1;
	char *body = malloc(len);
	char *prim = NULL;
	char *rest = body;

	if (!body) {
		fail(g, "out of memory");
	}
	memcpy(body, m->body, len);
	for (prim = token(&rest, "*"); prim; prim = token(&rest, "*")) {
		double a[PRIM_PARAMS];
		struct prim pr = { a, 0, at };

		while (isspace((unsigned char)*prim)) {
			prim++;
		}
		if (!*prim || *prim == '0' || *prim == '$') {
			continue; // A comment, or a variable defined: not handled.
		}
		pr.n = prim_params(g, prim, p, a);
		if (pr.n >= 1) {
			flash_prim(g, &pr);
		}
	}
	free(body);
}

static const struct aper *find_aper(const struct gbr *g, int code)
{
	for (size_t i = 0; i < g->naps; i++) {
		if (g->aps[i].code == code) {
			return &g->aps[i];
		}
	}
	return NULL;
}

static void flash(struct gbr *g, const struct aper *a, struct pt at)
{
	double x = at.x;
	double y = at.y;

	if (!a) {
		return;
	}
	if (!strcmp(a->type, "C")) {
		disk(g, at, a->p[0] * g->unit / 2);
	} else if (!strcmp(a->type, "R") && a->np >= 2) {
		struct box b = { x, y, a->p[0] * g->unit, a->p[1] * g->unit };

		rect(g, b, 0);
	} else if (!strcmp(a->type, "O") && a->np >= 2) {
		double w = a->p[0] * g->unit;
		double h = a->p[1] * g->unit;
		double r = fmin(w, h) / 2;

		if (w > h) {
			printf("line %.6g %.6g %.6g %.6g %.6g\n", 2 * r,
			       x - (w / 2) + r, y, x + (w / 2) - r, y);
		} else {
			printf("line %.6g %.6g %.6g %.6g %.6g\n", 2 * r, x,
			       y - (h / 2) + r, x, y + (h / 2) - r);
		}
		touch(g, at, fmax(w, h) / 2);
	} else {
		for (size_t i = 0; i < g->nmacs; i++) {
			if (!strcmp(g->macs[i].name, a->type)) {
				flash_macro(g, &g->macs[i], a->p, at);
			}
		}
	}
}

// Reads an aperture definition, ADD...
static void read_aperture(struct gbr *g, char *s)
{
	struct aper a;
	char *p = s + 3;
	char *c = NULL;

	memset(&a, 0, sizeof a);
	a.code = (int)strtol(p, &p, DECIMAL);
	c = strchr(p, ',');
	copy_prefix(a.type, sizeof a.type, p,
		    c ? (size_t)(c - p) : strcspn(p, "*"));
	while (c && a.np < APER_PARAMS) {
		a.p[a.np++] = strtod(c + 1, &c);
		if (*c != 'X') {
			break;
		}
	}
	g->aps = grow(g, g->aps, g->naps, sizeof *g->aps);
	g->aps[g->naps++] = a;
}

// Reads an aperture macro, AM...
static void read_macro(struct gbr *g, const char *s)
{
	struct macro m;
	const char *star = strchr(s, '*');

	if (!star) {
		return;
	}
	copy_prefix(m.name, sizeof m.name, s + 2, (size_t)(star - s - 2));
	copy_prefix(m.body, sizeof m.body, star + 1, strlen(star + 1));
	g->macs = grow(g, g->macs, g->nmacs, sizeof *g->macs);
	g->macs[g->nmacs++] = m;
}

// Reads an object attribute or its end: TO.C, TO.P, TO.N or TD.
static void attribute(struct gbr *g, const char *s)
{
	size_t at = strlen("TO.C,");

	if (!strncmp(s, "TO.C,", at) || !strncmp(s, "TO.P,", at)) {
		char was[NAME_LEN];

		memcpy(was, g->current_name, sizeof was);
		copy_prefix(g->current_name, sizeof g->current_name, s + at,
			    strcspn(s + at, ",*"));
		if (strcmp(was, g->current_name) != 0) {
			printf("part %s\n", g->current_name);
		}
	} else if (!strncmp(s, "TO.N,", at)) {
		end_zone(g);
		copy_prefix(g->net, sizeof g->net, s + at,
			    strcspn(s + at, ",*"));
	} else if (!strncmp(s, "TD", 2)) {
		end_zone(g);
		if (*g->current_name) {
			puts("part");
		}
		*g->current_name = 0;
		*g->net = 0;
	}
}

// Reads an extended command, %...%.
static void extended(struct gbr *g, char *s)
{
	if (!strncmp(s, "FS", 2)) {
		const char *x = strchr(s, 'X');

		if (x && isdigit((unsigned char)x[2])) {
			g->scale = pow(DECIMAL, -(x[2] - '0'));
		}
	} else if (!strncmp(s, "MOIN", 4)) {
		g->unit = MM_PER_INCH;
	} else if (!strncmp(s, "MOMM", 4)) {
		g->unit = 1;
	} else if (!strncmp(s, "LP", 2)) {
		set_dark(g, s[2] == 'D');
	} else if (!strncmp(s, "AD", 2) && s[2] == 'D') {
		read_aperture(g, s);
	} else if (!strncmp(s, "AM", 2)) {
		read_macro(g, s);
	} else {
		attribute(g, s);
	}
}

static long coord(const char *s, char key, int *found)
{
	const char *p = strchr(s, key);

	*found = p != NULL;
	return p ? strtol(p + 1, NULL, DECIMAL) : 0;
}

// Where the reading of a Gerber file stands: the point, the
// interpolation (1 straight, 2 clockwise, 3 counterclockwise), whether
// in a region, the operation, and the aperture.
struct state {
	struct pt at;
	int interp, region, code;
	const struct aper *ap;
};

// An operation's coordinates: where it goes, and the offset of the
// center of an arc.
struct move {
	struct pt to;
	struct pt ij;
};

// Adds the point p to the points of the region.
static void add_contour(struct gbr *g, struct pt p)
{
	if (g->nc + 1 >= g->cap) {
		g->cap = g->cap ? 2 * g->cap : CONTOUR_FIRST;
		g->contour = grow(g, g->contour, (2 * g->cap) - 1,
				  sizeof *g->contour);
	}
	g->contour[2 * g->nc] = p.x;
	g->contour[(2 * g->nc) + 1] = p.y;
	g->nc++;
}

// The points of an arc of the state st by the move mv into pts; returns
// their number of coordinates.
static int arc_points(const struct state *st, const struct move *mv,
		      double *pts)
{
	double x = st->at.x;
	double y = st->at.y;
	double ccx = x + mv->ij.x;
	double ccy = y + mv->ij.y;
	double r = hypot(x - ccx, y - ccy);
	double a0 = atan2(y - ccy, x - ccx);
	double a1 = atan2(mv->to.y - ccy, mv->to.x - ccx);
	int k = 0;
	int np = 0;

	if (st->interp == 3 && a1 <= a0) {
		a1 += 2 * PI;
	}
	if (st->interp == 2 && a1 >= a0) {
		a1 -= 2 * PI;
	}
	k = steps(fabs(a1 - a0) / (2 * PI) * ARC);
	if (k < 1) {
		k = 1;
	} else if (k > ARC) {
		k = ARC;
	}
	for (int i = 0; i <= k; i++) {
		double t = a0 + ((a1 - a0) * i / k);

		pts[np++] = ccx + r * cos(t);
		pts[np++] = ccy + r * sin(t);
	}
	return np;
}

// The draw of the state st by the move mv: straight, or an arc; into
// the region, or as a line of the aperture.
static void draw(struct gbr *g, const struct state *st, const struct move *mv)
{
	double pts[2 * (ARC + 2)];
	int np = 0;

	if (st->interp > 1) {
		np = arc_points(st, mv, pts);
	} else {
		pts[np++] = st->at.x;
		pts[np++] = st->at.y;
		pts[np++] = mv->to.x;
		pts[np++] = mv->to.y;
	}
	if (st->region) {
		for (int i = g->nc ? 2 : 0; i < np; i += 2) {
			add_contour(g, (struct pt){ pts[i], pts[i + 1] });
		}
	} else if (st->ap) {
		const struct aper *ap = st->ap;
		double w = !strcmp(ap->type, "C") || ap->np < 2 ?
				   ap->p[0] * g->unit :
				   fmin(ap->p[0], ap->p[1]) * g->unit;

		printf("line %.6g", w);
		for (int i = 0; i < np; i += 2) {
			printf(" %.6g %.6g", pts[i], pts[i + 1]);
			touch(g, (struct pt){ pts[i], pts[i + 1] }, w / 2);
		}
		printf("\n");
	}
}

// Takes the G codes of the command s: the interpolation, and the start
// and end of a region.
static void g_codes(struct gbr *g, struct state *st, const char *s)
{
	if (strstr(s, "G01")) {
		st->interp = 1;
	}
	if (strstr(s, "G02")) {
		st->interp = 2;
	}
	if (strstr(s, "G03")) {
		st->interp = 3;
	}
	if (strstr(s, "G36")) {
		st->region = 1;
		g->nc = 0;
		if (*g->net && !g->in_zone) {
			printf("zone %s\n", g->net);
			g->in_zone = 1;
		}
	}
	if (strstr(s, "G37")) {
		if (g->nc > 2) {
			poly(g, g->contour, (int)g->nc);
		}
		st->region = 0;
		g->nc = 0;
	}
}

// Carries out the operation of the command s: a D code, with its
// coordinates.
static void operation(struct gbr *g, struct state *st, const char *s)
{
	const char *d = strrchr(s, 'D');
	int fx = 0;
	int fy = 0;
	int fi = 0;
	int fj = 0;
	long lx = coord(s, 'X', &fx);
	long ly = coord(s, 'Y', &fy);
	long li = coord(s, 'I', &fi);
	long lj = coord(s, 'J', &fj);
	struct move mv = { { fx ? (double)lx * g->scale * g->unit : st->at.x,
			     fy ? (double)ly * g->scale * g->unit : st->at.y },
			   { fi ? (double)li * g->scale * g->unit : 0,
			     fj ? (double)lj * g->scale * g->unit : 0 } };

	if (d) {
		st->code = (int)strtol(d + 1, NULL, DECIMAL);
	} else if (!fx && !fy) {
		st->code = -1;
	}
	if (st->code >= DECIMAL) {
		st->ap = find_aper(g, st->code);
	} else if (st->code == 3) {
		flash(g, st->ap, mv.to);
	} else if (st->code == 2) {
		if (st->region && g->nc > 2) {
			poly(g, g->contour, (int)g->nc);
		}
		g->nc = 0;
	} else if (st->code == 1) {
		draw(g, st, &mv);
	}
	if (st->code == 2 && st->region) {
		g->nc = 0;
		add_contour(g, mv.to);
	}
	st->at = mv.to;
}

static void gerber(struct gbr *g, char *text)
{
	struct state st = { { 0, 0 }, 1, 0, -1, NULL };
	char *s = text;

	set_dark(g, 1);
	while (*s) {
		char *end = NULL;

		while (isspace((unsigned char)*s)) {
			s++;
		}
		if (!*s) {
			break;
		}
		if (*s == '%') {
			end = strchr(s + 1, '%');
			if (!end) {
				break;
			}
			*end = 0;
			extended(g, s + 1);
			s = end + 1;
			continue;
		}
		end = strchr(s, '*');
		if (!end) {
			break;
		}
		*end = 0;
		if (s[0] != 'G' || s[1] != '0' || s[2] != '4') {
			g_codes(g, &st, s);
			operation(g, &st, s);
		}
		s = end + 1;
	}
}

static void excellon(struct gbr *g, char *text)
{
	double tools[TOOLS] = { 0 };
	double d = 0;
	double u = 1;
	char *line = NULL;
	char *rest = text;

	set_dark(g, 1);
	for (line = token(&rest, "\n"); line; line = token(&rest, "\n")) {
		if (!strncmp(line, "INCH", 4)) {
			u = MM_PER_INCH;
		} else if (line[0] == 'T') {
			int t = (int)strtol(line + 1, NULL, DECIMAL);

			if (t >= 0 && t < TOOLS) {
				const char *c = strchr(line, 'C');

				if (c) {
					tools[t] = strtod(c + 1, NULL) * u;
				}
				d = tools[t];
			}
		} else if (line[0] == 'X') {
			const char *yp = strchr(line, 'Y');

			if (yp) {
				struct pt p = { strtod(line + 1, NULL) * u,
						strtod(yp + 1, NULL) * u };

				disk(g, p, d / 2);
			}
		}
	}
}

// The number of a copper layer of a file function Copper,Ln..., into
// *n; returns 0 for another function.
static int copper_layer(const char *f, int *n)
{
	const char *lead = "Copper,L";
	size_t at = strlen(lead);
	char *end = NULL;
	long v = 0;

	if (strncmp(f, lead, at) != 0) {
		return 0;
	}
	v = strtol(f + at, &end, DECIMAL);
	if (end == f + at) {
		return 0;
	}
	*n = (int)v;
	return 1;
}

// Whether f starts with the text pre.
static int starts(const char *f, const char *pre)
{
	return !strncmp(f, pre, strlen(pre));
}

// KiCad's name of the layer of a file function of Gerber X2: F.Cu,
// In1.Cu, ..., B.Cu, F.Mask, F.Paste, F.Silkscreen, F.Courtyard (B. on
// the bottom), Edge.Cuts; "" for another.  The name fits in LAYER_LEN.
static void layer_name(const char *fn, char name[LAYER_LEN])
{
	char f[FUNCTION_LEN];
	const char *side = NULL;
	const char *pos = NULL;
	int n = 0;

	copy_prefix(f, sizeof f, fn, strcspn(fn, "*%\r\n"));
	side = strstr(f, ",Bot") ? "B" : "F";
	pos = strchr(f, ',') ? strchr(strchr(f, ',') + 1, ',') : NULL;
	*name = 0;
	if (copper_layer(f, &n)) {
		if (pos && starts(pos, ",Top")) {
			(void)snprintf(name, LAYER_LEN, "F.Cu");
		} else if (pos && starts(pos, ",Bot")) {
			(void)snprintf(name, LAYER_LEN, "B.Cu");
		} else {
			(void)snprintf(name, LAYER_LEN, "In%d.Cu", n - 1);
		}
	} else if (starts(f, "Soldermask,")) {
		(void)snprintf(name, LAYER_LEN, "%s.Mask", side);
	} else if (starts(f, "Paste,")) {
		(void)snprintf(name, LAYER_LEN, "%s.Paste", side);
	} else if (starts(f, "Legend,")) {
		(void)snprintf(name, LAYER_LEN, "%s.Silkscreen", side);
	} else if (starts(f, "Other,Courtyard")) {
		(void)snprintf(name, LAYER_LEN, "%s.Courtyard", side);
	} else if (starts(f, "Profile")) {
		(void)snprintf(name, LAYER_LEN, "Edge.Cuts");
	}
}

static int by_name(const void *lhs, const void *rhs)
{
	return strcmp(((const struct named *)lhs)->name,
		      ((const struct named *)rhs)->name);
}

// The text of the file of g, all of it.
static char *read_all(const struct gbr *g)
{
	FILE *f = fopen(g->file, "rb");
	char *text = NULL;
	long n = -1;

	if (!f || fseek(f, 0, SEEK_END)) {
		fail(g, "cannot read");
	}
	n = ftell(f);
	if (n < 0 || fseek(f, 0, SEEK_SET)) {
		fail(g, "cannot read");
	}
	text = malloc((size_t)n + 1);
	if (!text || fread(text, 1, (size_t)n, f) != (size_t)n) {
		fail(g, "cannot read");
	}
	text[n] = 0;
	(void)fclose(f);
	return text;
}

// Writes the layer of the file, if it names one.
static void write_layer(const char *text)
{
	const char *key = "TF.FileFunction,";
	const char *fn = strstr(text, key);
	char layer[LAYER_LEN] = "";

	if (fn) {
		layer_name(fn + strlen(key), layer);
	}
	if (strstr(text, "M48")) {
		(void)snprintf(layer, sizeof layer, "Drill");
	}
	if (*layer) {
		printf("layer %s\n", layer);
	}
}

int main(int argc, char **argv)
{
	struct gbr g;
	char *text = NULL;

	if (argc != 2) {
		(void)fprintf(stderr, "usage: gbrvec FILE > FILE.vec\n");
		return 2;
	}
	memset(&g, 0, sizeof g);
	g.file = argv[1];
	g.unit = 1;
	g.scale = SCALE_FIRST;
	g.dark = -1;
	text = read_all(&g);
	printf("# vec of %s\n",
	       strrchr(g.file, '/') ? strrchr(g.file, '/') + 1 : g.file);
	write_layer(text);
	if (strstr(text, "M48")) {
		excellon(&g, text);
	} else {
		gerber(&g, text);
		end_zone(&g);
	}
	if (g.nnames) {
		qsort(g.names, g.nnames, sizeof *g.names, by_name);
	}
	for (size_t i = 0; i < g.nnames; i++) {
		printf("name %s %.6g %.6g %.6g %.6g\n", g.names[i].name,
		       g.names[i].box[0], g.names[i].box[1], g.names[i].box[2],
		       g.names[i].box[3]);
	}
	free(text);
	for (size_t i = 0; i < g.nnames; i++) {
		free(g.names[i].name);
	}
	free(g.names);
	free(g.aps);
	free(g.macs);
	free(g.contour);
	return 0;
}
