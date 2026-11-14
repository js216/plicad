// SPDX-License-Identifier: GPL-3.0-only
// smdim.c -- Add dimensions and callouts to a drawing view
// Copyright (c) 2026 Jakob Kastelic

// Usage: smview VIEW part.fold | smdim [-s SCALE] > part.dim
//
// Reads one view of smview, in mm, and writes it at SCALE in inches on
// the paper, with the origin at the datum, the lower left corner of the
// view.  The fill, line and arc records stay as they are but for the
// units; the view record gains the scale, and these records come new:
//   line c v X0 Y0 X1 Y1           (the centre marks of the holes)
//   thin X0 Y0 X1 Y1               (a thin line of a dimension)
//   text X Y ANGLE ALIGN HEIGHT STRING...
//   arrow X Y DX DY                (a head at X Y that points along DX DY)
//   dot X Y
//   flag X Y N                     (the number of a note, in a box)
//   dim ord AXIS MM STRING         (what each dimension says, for tests:
//   dim hole MM COUNT STRING        the place or size in the model, in
//   dim radius MM COUNT STRING      mm, and the text on the paper)
//   dim lin AXIS MM STRING
//   dim pitch XMM YMM STRING
//   dim bend ANGLE STRING
//   hatch X0 Y0 X1 Y1
//   crowd D                        (the farthest that the text of an
//                                   ordinate stands from its place)
//   far N                          (the leaders longer than REACH or
//                                   past the gap of an ordinate)
//   clear X0 Y0 X1 Y1              (a text on a hatch, where its lines
//                                   stop)
// The thick and radius records pass through, in mm.  ALIGN is l, c or
// r: where X Y lies in the text, at half its height.  In a STRING, %%c
// is the diameter sign, %%v the countersink sign, %%d the degree sign
// and %%p the plus and minus sign.
//
// The dimensions are in inches, to three decimals, without the zero
// before the point.  On each axis an ordinate from the datum goes to
// the far end of the view, to every hole seen along its axis, to every
// straight edge of a cut-out and every centre line along the other
// axis, and to the other straight edges that lie more than CLOSE sheet
// thicknesses from the nearest of those: the faces of one wall and the
// ends of a bend share one.  Of a row of equal holes at one pitch only
// the first two and the last get theirs, and the callout of the holes
// gives the pitch.  The holes of one size share a callout with their
// count, and so do the arcs of one radius.  In the flat view, each bend
// line says UP or DOWN, toward the eye or away, and the angle.  No
// text lies on another: the ordinates spread out along their row, and
// each callout goes to a free place inside the view or round it, with
// a leader no longer than REACH inches if there is such a place, the
// shortest one, and a slanted one before a level or an upright one.
// No leader crosses a text, another leader, or an ordinate text with
// ROW_GAP inches to each side of it.
//
// A line "#line N NAME" makes the messages about the records after it
// name line N of NAME, and so on.  These records, after the view on the
// input, change the result:
//   drop x|y|d|r STRING            (leave out the dimension that says so)
//   lin x|y A B [PLUS MINUS]       (a linear dimension between two
//                                   ordinates, with its tolerances)
//   leader X Y STRING...           (a note with a dot at X Y, in inches
//   flag X Y N [STRING...]          from the datum; the box of note N,
//                                   and a text after it)
//   hatch X0 Y0 X1 Y1              (thin lines across the material in
//                                   this rectangle; it passes through)

#include "font.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEXT_H 0.12 // The height of the text, in inches.
#define SMALL_H 0.09 // The height of a tolerance and of a bend note.
#define PITCH (1.6 * TEXT_H) // From one ordinate to the next.
#define LEAD (1.5 * TEXT_H) // From one line of a callout to the next.
#define ROW 0.32 // From the view to the text of an ordinate.
#define JOG0 0.08 // Where the line of a moved ordinate turns,
#define JOG1 0.24 // and where it is straight again.
#define GAP 0.03 // Between a line and what it points at.
#define PAD 0.04 // Round each text.
#define MARK 0.06 // The reach of a centre mark past its hole.
#define LAND 0.15 // The level start of a leader.
#define STEP 0.2 // From one place for a callout to the next.
#define RING 0.5 // From one ring of places to the next.
#define FLAG_W 0.24 // The side of the box of a flag.
#define HEAD_R 0.025 // The reach of a dot, or of the head of an arrow.
#define SLANT 3.0 // How much longer a level or upright leader counts.
#define REACH 2.5 // The length a leader should not pass,
#define STUB 0.25 // and the least it has, for its head to show.
#define ROW_GAP 0.25 // A leader keeps this far from an ordinate text.
#define CELL 0.25 // The cells of the grid that sorts the obstacles.
#define DOT 2e-4 // A line shorter than this, in inches, is none.
#define ONE 2e-5 // Ordinates closer than this are one, in inches.
#define FULL 359.0 // Sweep above which an arc is a circle, in degrees.
#define THOU 1000.0 // Thousandths in one inch.
#define CLOSE 2.5 // Sheets between two edges with an ordinate each.
#define SHORT_EDGE 1.05 // An edge of a cut-out is longer, in sheets.

enum {
	RINGS = 12, // The rings of places tried for a callout.
	LEVELS = 3, // The levels of spot_free.
	NOTE_LINES = 3, // The most lines of a callout.
	WORDS = 12, // The most fields of a record.
	FILL_NUMS = 5, // The most numbers of a fill record.
	LABEL = 160, // Room for one line of a callout.
	SHORT = 48, // Room for the text of one number.
	LONG_LEN = 24, // Room for the digits of a long.
	UPRIGHT = 90, // The angle of a text that reads from the right.
	DECIMAL = 10 // The base of the number of a #line record.
};

// The ranks of the places for an ordinate: an end of the view, a hole,
// a centre line, an edge of a cut-out and another edge.
enum { R_END, R_HOLE, R_CENTRE, R_CUT, R_EDGE };

struct pt {
	double x, y;
};

struct rect {
	double x0, y0, x1, y1;
};

struct seg {
	char kind, vis;
	struct pt a, b;
};

struct arc {
	char kind, vis;
	struct pt c;
	double r, a0, a1;
};

// A hole, with the diameter dc and the angle of its countersink, at
// the near or the far face; dc is 0 without one.
struct hole {
	struct pt c;
	double d, dc, ang;
	int far;
};

// A bend line of the flat view, from a to b, and the angle of the bend.
struct bend {
	struct pt a, b;
	double angle;
};

// A fill record: its word, and its n numbers.
struct fill {
	char op;
	double x[FILL_NUMS];
	size_t n;
};

// A drop, lin, hatch, leader or flag record, as text.
struct rule {
	char *s;
	const char *what; // Where it stands: the name of its file,
	unsigned line; // and its line there.
};

// A place for an ordinate: p along its axis, from q across it, at the
// low or the high side of the view, with its rank and, for an edge, its
// length.
struct item {
	double p, q, len;
	int rank, hi;
};

struct items {
	struct item *v;
	size_t n, cap;
};

// A piece of a leader, from a to b.
struct lead {
	struct pt a, b;
};

// What a text inside the view must not lie on: the segment from a to
// b, or with box set the rectangle with those corners.
struct obstacle {
	struct pt a, b;
	int box;
};

// A rectangle of cells of the grid over the view: columns x0 to x1 and
// rows y0 to y1.
struct cells {
	long x0, y0, x1, y1;
};

// A point a callout may lead to, and the radius of what is there.
struct target {
	struct pt at;
	double r;
};

// A callout: its lines of text, or the number of its flag and the
// text after the box, and the points it may lead to.  Its leader ends in a dot or an arrow.
struct note {
	const char *line[NOTE_LINES]; // In own, or in the input.
	char own[NOTE_LINES][LABEL];
	int nlines, flag, dot;
	struct target *to;
	size_t n, cap;
};

struct label {
	char s[SHORT];
};

struct view {
	const char *name, *part; // In the input.
	double k; // Inches on the paper for one mm.
	double scale, thick, radius;
	double crowd; // The farthest an ordinate moved from its place.
	struct rect box; // The view, and then all that is drawn.
	struct rect frame; // The view, and then the view with its ordinates.
	struct fill *fill;
	size_t nfill, capfill;
	struct seg *seg;
	size_t nseg, capseg;
	struct arc *arc;
	size_t narc, caparc;
	struct hole *hole;
	size_t nhole, caphole;
	struct bend *bend;
	size_t nbend, capbend;
	struct rule *rule;
	size_t nrule, caprule;
	struct rule *names; // The names of #line records.
	size_t nnames, capnames;
	const struct rule *at; // The rule under way, if any.
	struct rect *text; // The room each text takes.
	size_t ntext, captext;
	struct rect *row; // Each ordinate text, with the gaps beside it.
	size_t nrow, caprow;
	struct lead *lead; // The leaders drawn so far.
	size_t nlead, caplead;
	struct lead *ext; // The lines of the ordinates.
	size_t next, capext;
	struct rect *hatch; // The hatched rectangles.
	size_t nhatch, caphatch;
	struct obstacle *obst; // What a text inside the view keeps off,
	size_t nobst, capobst;
	size_t *cell, *in_cell; // by the cells of a grid over the view,
	long cols, rows; // which has this many columns and rows.
	struct rect inner; // The view itself.
	int far; // The leaders longer than REACH or past a gap.
	struct pt *pin; // The points the leader and flag records name.
	size_t npin, cappin;
	int level; // What spot_free allows; see there.
	double *place[2]; // Every place an ordinate may stand, by axis.
	size_t nplace[2], capplace[2];
};

// A record under reading: its line, its fields, and the next one.
struct reader {
	const char *what; // The name of the input, for messages.
	const char *text; // The whole line.
	unsigned line;
	char *w[WORDS + 1];
	size_t n, k;
};

static const char *word(struct reader *r)
{
	if (r->k >= r->n) {
		die("%s:%u: %s: too few fields", r->what, r->line, r->w[0]);
	}
	return r->w[r->k++];
}

static double num(struct reader *r)
{
	const char *w = word(r);
	char *end = NULL;
	double x = strtod(w, &end);

	if (end == w || *end || !num_ok(x)) {
		die("%s:%u: %s is not a number", r->what, r->line, w);
	}
	return x;
}

static struct pt point(struct reader *r)
{
	struct pt p = { 0, 0 };

	p.x = num(r);
	p.y = num(r);
	return p;
}

// The letter of a kind or a visibility: a field of one character.
static char letter(struct reader *r)
{
	const char *w = word(r);

	if (strlen(w) != 1) {
		die("%s:%u: %s is not one letter", r->what, r->line, w);
	}
	return w[0];
}

// The text of v inches: three decimals, no zero before the point.
static struct label inches(double v)
{
	long n = lround(fabs(v) * THOU);
	long thou = lround(THOU);
	long whole = n / thou;
	long rest = n % thou;
	struct label l;
	char head[LONG_LEN] = "";

	if (whole) {
		xsnprintf(head, "%ld", whole);
	}
	xsnprintf(l.s, "%s%s.%03ld", v < 0 && n ? "-" : "", head, rest);
	return l;
}

// The rectangle with the corners a and b.
static struct rect corners(struct pt a, struct pt b)
{
	struct rect r = { fmin(a.x, b.x), fmin(a.y, b.y), fmax(a.x, b.x),
			  fmax(a.y, b.y) };

	return r;
}

// Grows the box of the view to cover r.
static void cover(struct view *v, struct rect r)
{
	v->box.x0 = fmin(v->box.x0, r.x0);
	v->box.y0 = fmin(v->box.y0, r.y0);
	v->box.x1 = fmax(v->box.x1, r.x1);
	v->box.y1 = fmax(v->box.y1, r.y1);
}

// Grows the box of the view to cover the leader from a to b, with the
// dot or the head of an arrow at b.
static void cover_line(struct view *v, struct pt a, struct pt b)
{
	struct rect r = corners(a, b);

	r.x0 -= HEAD_R;
	r.y0 -= HEAD_R;
	r.x1 += HEAD_R;
	r.y1 += HEAD_R;
	cover(v, r);
}

// Makes nt a callout with no text and no target, its lines in its own
// room.
static void init_note(struct note *nt)
{
	memset(nt, 0, sizeof(*nt));
	for (int i = 0; i < NOTE_LINES; i++) {
		nt->line[i] = nt->own[i];
	}
}

// Takes the room r for a text.
static void take(struct view *v, struct rect r)
{
	PUSH(v->text, v->ntext, v->captext, r);
	cover(v, r);
}

static int rects_meet(struct rect a, struct rect b)
{
	return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}

// Whether r is free of every text so far.
static int is_free(const struct view *v, struct rect r)
{
	for (size_t i = 0; i < v->ntext; i++) {
		if (rects_meet(r, v->text[i])) {
			return 0;
		}
	}
	return 1;
}

// Whether the segment from a to b passes through r: by the range of t
// along the segment that lies between each pair of sides.  A segment
// whose box lies beside r is soon told.
static int seg_meets(struct pt a, struct pt b, struct rect r)
{
	const double lo[2] = { r.x0, r.y0 };
	const double hi[2] = { r.x1, r.y1 };
	const double o[2] = { a.x, a.y };
	const double d[2] = { b.x - a.x, b.y - a.y };
	struct span t = { 0, 1 };

	if (fmax(a.x, b.x) <= r.x0 || fmin(a.x, b.x) >= r.x1 ||
	    fmax(a.y, b.y) <= r.y0 || fmin(a.y, b.y) >= r.y1) {
		return 0;
	}
	for (int i = 0; i < 2; i++) {
		if (fabs(d[i]) < ONE) {
			t.lo = o[i] > lo[i] && o[i] < hi[i] ? t.lo : HUGE_VAL;
		} else {
			double t0 = (lo[i] - o[i]) / d[i];
			double t1 = (hi[i] - o[i]) / d[i];

			t.lo = fmax(t.lo, fmin(t0, t1));
			t.hi = fmin(t.hi, fmax(t0, t1));
		}
	}
	return t.lo < t.hi;
}

static void thin(struct pt a, struct pt b)
{
	printf("thin %.4f %.4f %.4f %.4f\n", a.x, a.y, b.x, b.y);
}

static void arrow(struct pt at, struct pt d)
{
	printf("arrow %.4f %.4f %.4f %.4f\n", at.x, at.y, d.x, d.y);
}

// A text: where, at what angle, how aligned and how high.
struct style {
	struct pt at;
	int angle;
	char align;
	double h;
};

static void text(struct style st, const char *s)
{
	printf("text %.4f %.4f %d %c %.3f %s\n", st.at.x, st.at.y, st.angle,
	       st.align, st.h, s);
}

static void read_curve(struct view *v, struct reader *r)
{
	int is_arc = r->w[0][0] == 'a';
	struct arc a = { 0, 0, { 0, 0 }, 0, 0, 0 };
	struct seg s = { 0, 0, { 0, 0 }, { 0, 0 } };

	a.kind = s.kind = letter(r);
	a.vis = s.vis = letter(r);
	if (is_arc) {
		a.c = point(r);
		a.r = num(r);
		a.a0 = num(r);
		a.a1 = num(r);
		PUSH(v->arc, v->narc, v->caparc, a);
	} else {
		s.a = point(r);
		s.b = point(r);
		PUSH(v->seg, v->nseg, v->capseg, s);
	}
}

static void read_hole(struct view *v, struct reader *r)
{
	struct hole h = { { 0, 0 }, 0, 0, 0, 0 };

	h.c = point(r);
	h.d = num(r);
	if (r->k < r->n) {
		if (strcmp(word(r), "csink") != 0) {
			die("%s:%u: expected csink DC ANGLE near|far", r->what,
			    r->line);
		}
		h.dc = num(r);
		h.ang = num(r);
		h.far = strcmp(word(r), "far") == 0;
	}
	PUSH(v->hole, v->nhole, v->caphole, h);
}

static void read_bend(struct view *v, struct reader *r)
{
	struct bend b = { { 0, 0 }, { 0, 0 }, 0 };

	b.a = point(r);
	b.b = point(r);
	b.angle = num(r);
	PUSH(v->bend, v->nbend, v->capbend, b);
}

static void read_box(struct view *v, struct reader *r)
{
	struct pt lo = point(r);
	struct pt hi = point(r);

	v->box = corners(lo, hi);
}

// Reads a fill record: its word and the numbers after it.
static void read_fill(struct view *v, struct reader *r)
{
	struct fill f = { r->w[0][0], { 0 }, 0 };

	if (r->n - 1 > FILL_NUMS) {
		die("%s:%u: too many fields", r->what, r->line);
	}
	while (r->k < r->n) {
		f.x[f.n++] = num(r);
	}
	PUSH(v->fill, v->nfill, v->capfill, f);
}

static void read_name(struct view *v, struct reader *r)
{
	v->name = word(r);
	v->part = word(r);
}

static void read_thick(struct view *v, struct reader *r)
{
	v->thick = num(r);
}

static void read_radius(struct view *v, struct reader *r)
{
	v->radius = num(r);
}

// Keeps a drop, lin, leader or flag record for later, as text.
static void read_rule(struct view *v, struct reader *r)
{
	struct rule rule = { xstrdup(r->text), r->what, r->line };

	PUSH(v->rule, v->nrule, v->caprule, rule);
	r->k = r->n;
}

// Reads the record in r, by its first word.
static void read_record(struct view *v, struct reader *r)
{
	static const struct {
		const char *name;
		void (*read)(struct view *v, struct reader *r);
	} kinds[] = {
		{ "view", read_name },	   { "thick", read_thick },
		{ "radius", read_radius }, { "bend", read_bend },
		{ "box", read_box },	   { "line", read_curve },
		{ "arc", read_curve },	   { "hole", read_hole },
		{ "fill", read_fill },	   { "m", read_fill },
		{ "l", read_fill },	   { "a", read_fill },
		{ "z", read_fill },	   { "drop", read_rule },
		{ "lin", read_rule },	   { "leader", read_rule },
		{ "flag", read_rule },	   { "hatch", read_rule },
	};
	const char *k = word(r);
	size_t i = 0;

	while (i < sizeof(kinds) / sizeof(*kinds) &&
	       strcmp(k, kinds[i].name) != 0) {
		i++;
	}
	if (i == sizeof(kinds) / sizeof(*kinds)) {
		die("%s:%u: unknown record %s", r->what, r->line, k);
	}
	kinds[i].read(v, r);
	if (r->k != r->n) {
		die("%s:%u: too many fields", r->what, r->line);
	}
}

// Splits line l into the fields of r; returns 0 for a blank line or a
// comment.
static int split_record(struct reader *r, char *l)
{
	r->line++;
	r->n = split_words(l, r->w, WORDS);
	r->k = 0;
	if (r->n > WORDS) {
		die("%s:%u: too many fields", r->what, r->line);
	}
	return r->n && r->w[0][0] != '#';
}

// Keeps the name s for the messages of the records after it.
static const char *name_of(struct view *v, const char *s)
{
	struct rule kept = { xstrdup(s), NULL, 0 };

	PUSH(v->names, v->nnames, v->capnames, kept);
	return kept.s;
}

// Reads the view.  A line "#line N NAME" says that the next line is line
// N of the file NAME, for the messages.
static void read_input(struct view *v, char *input)
{
	struct reader r = { "stdin", NULL, 0, { NULL }, 0, 0 };
	char *rest = input;
	char *l = NULL;

	while ((l = next_line(&rest)) != NULL) {
		char *copy = NULL;

		if (strncmp(l, "#line ", strlen("#line ")) == 0) {
			char *end = NULL;
			unsigned long n =
				strtoul(l + strlen("#line "), &end, DECIMAL);

			if (end == l + strlen("#line ") || *end != ' ' || !n) {
				die("%s:%u: expected #line N NAME", r.what,
				    r.line + 1);
			}
			r.what = name_of(v, end + 1);
			r.line = (unsigned)(n - 1);
			continue;
		}
		copy = xstrdup(l);
		r.text = copy;
		if (split_record(&r, l)) {
			read_record(v, &r);
		}
		free(copy);
	}
	if (!v->name[0] || v->box.x1 <= v->box.x0 || v->box.y1 <= v->box.y0) {
		die("stdin: no view with a box");
	}
}

static struct pt to_paper(const struct view *v, struct pt p)
{
	struct pt q = { (p.x - v->box.x0) * v->k, (p.y - v->box.y0) * v->k };

	return q;
}

// Prints the fill records on the paper.
static void print_fills(const struct view *v)
{
	for (size_t i = 0; i < v->nfill; i++) {
		const struct fill *f = &v->fill[i];
		struct pt p = { f->x[0], f->x[1] };

		p = to_paper(v, p);
		if (f->n == 0) {
			printf("%s\n", f->op == 'f' ? "fill" : "z");
		} else if (f->n == 2) {
			printf("%c %.4f %.4f\n", f->op, p.x, p.y);
		} else if (f->n == FILL_NUMS) {
			printf("a %.4f %.4f %.4f %.3f %.3f\n", p.x, p.y,
			       f->x[2] * v->k, f->x[3], f->x[4]);
		} else {
			die("stdin: a malformed %c record", f->op);
		}
	}
}

// Prints the centre mark of hole h: a cross that reaches past it.
static void print_mark(const struct view *v, const struct hole *h)
{
	double r = (fmax(h->d, h->far ? 0 : h->dc) * v->k / 2) + MARK;

	printf("line c v %.4f %.4f %.4f %.4f\n", h->c.x - r, h->c.y, h->c.x + r,
	       h->c.y);
	printf("line c v %.4f %.4f %.4f %.4f\n", h->c.x, h->c.y - r, h->c.x,
	       h->c.y + r);
}

// Moves the view to the paper, and prints its lines, arcs and marks;
// a line or an arc shorter than DOT, which would print with no length,
// goes.
static void print_curves(struct view *v)
{
	struct pt far = { v->box.x1, v->box.y1 };

	size_t n = 0;

	for (size_t i = 0; i < v->nseg; i++) {
		struct seg *s = &v->seg[i];

		s->a = to_paper(v, s->a);
		s->b = to_paper(v, s->b);
		if (hypot(s->b.x - s->a.x, s->b.y - s->a.y) >= DOT) {
			v->seg[n++] = *s;
			printf("line %c %c %.4f %.4f %.4f %.4f\n", s->kind,
			       s->vis, s->a.x, s->a.y, s->b.x, s->b.y);
		}
	}
	v->nseg = n;
	n = 0;
	for (size_t i = 0; i < v->narc; i++) {
		struct arc *a = &v->arc[i];

		a->c = to_paper(v, a->c);
		a->r *= v->k;
		if (a->r * (a->a1 - a->a0) * DEG >= DOT) {
			v->arc[n++] = *a;
			printf("arc %c %c %.4f %.4f %.4f %.3f %.3f\n", a->kind,
			       a->vis, a->c.x, a->c.y, a->r, a->a0, a->a1);
		}
	}
	v->narc = n;
	for (size_t i = 0; i < v->nhole; i++) {
		v->hole[i].c = to_paper(v, v->hole[i].c);
		print_mark(v, &v->hole[i]);
	}
	for (size_t i = 0; i < v->nbend; i++) {
		v->bend[i].a = to_paper(v, v->bend[i].a);
		v->bend[i].b = to_paper(v, v->bend[i].b);
	}
	far = to_paper(v, far);
	v->box.x0 = v->box.y0 = 0;
	v->box.x1 = far.x;
	v->box.y1 = far.y;
	v->frame = v->box;
	v->inner = v->box;
}

// Whether a drop record of kind k names the text s.
static int dropped(const struct view *v, const char *k, const char *s)
{
	static const char head[] = "drop ";
	size_t nh = sizeof(head) - 1;
	size_t nk = strlen(k);

	for (size_t i = 0; i < v->nrule; i++) {
		const char *r = v->rule[i].s;

		r += strspn(r, " \t");
		if (strncmp(r, head, nh) == 0 && strncmp(r + nh, k, nk) == 0 &&
		    r[nh + nk] == ' ' && strcmp(r + nh + nk + 1, s) == 0) {
			return 1;
		}
	}
	return 0;
}

// The coordinate of p along the axis, and the one across it.
static double along(int axis, struct pt p)
{
	return axis ? p.y : p.x;
}

static double across(int axis, struct pt p)
{
	return axis ? p.x : p.y;
}

// The point at p along the axis and q across it.
static struct pt pq(int axis, double p, double q)
{
	struct pt at = { axis ? q : p, axis ? p : q };

	return at;
}

// The far side of the view across the axis; the near one is at 0.
static double far_side(const struct view *v, int axis)
{
	return axis ? v->frame.x1 : v->frame.y1;
}

// Whether a hole of the size of h stands d from it along the axis.
static int hole_at(const struct view *v, int axis, const struct hole *h,
		   double d)
{
	for (size_t i = 0; i < v->nhole; i++) {
		const struct hole *g = &v->hole[i];

		if (fabs(g->d - h->d) < ONE &&
		    fabs(along(axis, g->c) - along(axis, h->c) - d) < ONE) {
			return 1;
		}
	}
	return 0;
}

// The pitch of the row of holes that hole h is inside along the axis,
// or 0 if it is inside none: among the places of the holes of its
// size, the next place on each side is equally far, and so is the one
// before that.  The first two holes and the last of a row keep their
// ordinates, and give its pitch and its length.
static double in_pattern(const struct view *v, int axis, const struct hole *h)
{
	struct span next = { HUGE_VAL, HUGE_VAL };

	for (size_t i = 0; i < v->nhole; i++) {
		const struct hole *g = &v->hole[i];
		double d = along(axis, g->c) - along(axis, h->c);

		if (fabs(g->d - h->d) < ONE && d < -ONE) {
			next.lo = fmin(next.lo, -d);
		}
		if (fabs(g->d - h->d) < ONE && d > ONE) {
			next.hi = fmin(next.hi, d);
		}
	}
	if (fabs(next.hi - next.lo) < ONE &&
	    hole_at(v, axis, h, -2 * next.lo)) {
		return next.lo;
	}
	return 0;
}

// Adds an item for the feature that runs from x.q to q1 across the
// axis, at the side of the view it is nearer to.
static void add_item(struct view *v, struct items *it, int axis, struct item x,
		     double q1)
{
	struct span q = { fmin(x.q, q1), fmax(x.q, q1) };

	x.hi = x.rank != R_END && far_side(v, axis) - q.hi < q.lo;
	x.q = x.hi ? q.hi : q.lo;
	if (x.rank == R_END) {
		x.q = 0;
	}
	PUSH(it->v, it->n, it->cap, x);
	PUSH(v->place[axis], v->nplace[axis], v->capplace[axis], x.p);
}

// Lists the edges and centre lines that may get an ordinate on the
// axis: the visible ones square to it, but for the short ends of the
// sheet, which mark no place of their own.
static void list_edges(struct view *v, struct items *it, int axis)
{
	for (size_t i = 0; i < v->nseg; i++) {
		const struct seg *s = &v->seg[i];
		struct item x = { along(axis, s->a), across(axis, s->a), 0,
				  R_CENTRE, 0 };
		double sheet = v->thick * v->k;
		double least = 0;

		x.len = fabs(across(axis, s->b) - x.q);
		if (s->kind == 'i') {
			x.rank = R_CUT;
			least = SHORT_EDGE * sheet;
		}
		if (s->kind == 'e') {
			x.rank = R_EDGE;
			least = 2 * sheet;
		}
		if (strchr("eic", s->kind) && s->vis == 'v' &&
		    fabs(along(axis, s->b) - x.p) < ONE && x.len > least) {
			add_item(v, it, axis, x, across(axis, s->b));
		}
	}
}

// Lists every place for an ordinate on the axis.
static void list_items(struct view *v, struct items *it, int axis)
{
	struct item x = { 0, 0, 0, R_END, 0 };

	add_item(v, it, axis, x, 0);
	x.p = axis ? v->frame.y1 : v->frame.x1;
	add_item(v, it, axis, x, 0);
	x.rank = R_HOLE;
	for (size_t i = 0; i < v->nhole; i++) {
		x.p = along(axis, v->hole[i].c);
		x.q = across(axis, v->hole[i].c);
		if (in_pattern(v, axis, &v->hole[i]) < ONE) {
			add_item(v, it, axis, x, x.q);
		}
	}
	list_edges(v, it, axis);
}

static int cmp_rank(const void *lhs, const void *rhs)
{
	const struct item *a = lhs;
	const struct item *b = rhs;

	if (a->rank != b->rank) {
		return a->rank - b->rank;
	}
	if (fabs(a->len - b->len) > ONE) {
		return a->len > b->len ? -1 : 1;
	}
	return (a->p > b->p) - (a->p < b->p);
}

static int cmp_place(const void *lhs, const void *rhs)
{
	const struct item *a = lhs;
	const struct item *b = rhs;

	if (a->hi != b->hi) {
		return a->hi - b->hi;
	}
	return (a->p > b->p) - (a->p < b->p);
}

// Whether item x gets an ordinate, next to the n items of it that do:
// its place is new, and for a plain edge no edge, centre line or end of
// the view is within CLOSE sheet thicknesses of it.
static int keeps(const struct view *v, const struct items *it, size_t n,
		 const struct item *x)
{
	for (size_t k = 0; k < n; k++) {
		double d = fabs(it->v[k].p - x->p);

		if (d < ONE || (x->rank == R_EDGE && it->v[k].rank != R_HOLE &&
				d < CLOSE * v->thick * v->k)) {
			return 0;
		}
	}
	return 1;
}

// Keeps the items that get an ordinate, the lower ranks first, and none
// that a drop record names.
static void choose_items(const struct view *v, struct items *it, int axis)
{
	size_t n = 0;

	qsort(it->v, it->n, sizeof(*it->v), cmp_rank);
	for (size_t i = 0; i < it->n; i++) {
		const struct item *x = &it->v[i];

		if (keeps(v, it, n, x) &&
		    !dropped(v, axis ? "y" : "x", inches(x->p / v->scale).s)) {
			it->v[n++] = *x;
		}
	}
	it->n = n;
}

// A block of places that stand PITCH apart: their number, and the sum
// of where each would put the first of them.
struct block {
	double sum;
	size_t n;
};

static double block_at(const struct block *b)
{
	return b->sum / (double)b->n;
}

// Moves the sorted places p apart, each as little as may be, until
// none is closer than PITCH to the next: with q[i] = p[i] - i PITCH,
// the nearest row that does not fall is the answer.  Each block of
// places that would fall takes its mean.
static void spread(double *p, size_t n)
{
	struct block *b = xmalloc((n + 1) * sizeof(*b));
	size_t nb = 0;
	size_t i = 0;

	for (size_t k = 0; k < n; k++) {
		b[nb].sum = p[k] - ((double)k * PITCH);
		b[nb++].n = 1;
		while (nb > 1 && block_at(&b[nb - 2]) > block_at(&b[nb - 1])) {
			b[nb - 2].sum += b[nb - 1].sum;
			b[nb - 2].n += b[nb - 1].n;
			nb--;
		}
	}
	for (size_t j = 0; j < nb; j++) {
		for (size_t k = 0; k < b[j].n; k++, i++) {
			p[i] = block_at(&b[j]) + ((double)i * PITCH);
		}
	}
	free(b);
}

// Keeps the n places p, spread PITCH apart, between lo and hi: from
// the end that must hold, pushed on, then from the other end, pushed
// back, if all have room between the two.  The end that need not hold
// is hi if free_hi is set, else lo.
static void keep_between(double *p, size_t n, struct span b, int free_hi)
{
	int fits = (double)(n - 1) * PITCH <= b.hi - b.lo;

	if (n == 0) {
		return;
	}
	if (free_hi || fits) {
		p[0] = fmax(p[0], b.lo);
		for (size_t i = 1; i < n; i++) {
			p[i] = fmax(p[i], p[i - 1] + PITCH);
		}
	}
	if (!free_hi || fits) {
		p[n - 1] = fmin(p[n - 1], b.hi);
		for (size_t i = n - 1; i-- > 0;) {
			p[i] = fmin(p[i], p[i + 1] - PITCH);
		}
	}
}

// Draws the ordinate of item x with its text at l along the axis: a
// line from the feature out of the view, a jog to l, and the text.
static void draw_ordinate(struct view *v, int axis, const struct item *x,
			  double l)
{
	double s = x->hi ? 1 : -1;
	double edge = x->hi ? far_side(v, axis) : 0;
	struct label t = inches(x->p / v->scale);
	double row = edge + (s * ROW);
	double end = row + (s * text_width(t.s, TEXT_H));
	struct style st = { pq(axis, l, row), axis ? 0 : UPRIGHT,
			    x->hi ? 'l' : 'r', TEXT_H };
	struct rect wide;

	if (fabs(x->q - edge) > GAP || fabs(l - x->p) < ONE) {
		struct lead e = { pq(axis, x->p, x->q + (s * GAP)),
				  pq(axis, x->p, edge + (s * JOG0)) };

		thin(e.a, e.b);
		PUSH(v->ext, v->next, v->capext, e);
	}
	thin(pq(axis, x->p, edge + (s * JOG0)), pq(axis, l, edge + (s * JOG1)));
	thin(pq(axis, l, edge + (s * JOG1)), pq(axis, l, row - (s * GAP)));
	text(st, t.s);
	take(v, corners(pq(axis, l - (TEXT_H / 2) - PAD, row),
			pq(axis, l + (TEXT_H / 2) + PAD, end)));
	wide = corners(pq(axis, l - (TEXT_H / 2) - ROW_GAP, row),
		       pq(axis, l + (TEXT_H / 2) + ROW_GAP, end));
	PUSH(v->row, v->nrow, v->caprow, wide);
	printf("dim ord %c %.4f %s\n", axis ? 'y' : 'x', x->p / v->k, t.s);
}

// How far across the axis the part reaches along its datum line, the
// line at 0 on the axis: the far end of the lines that lie on it, or
// the far side of the view if none does.  The line of the datum
// ordinate at the high side starts there.
static double datum_reach(const struct view *v, int axis)
{
	double q = -HUGE_VAL;

	for (size_t i = 0; i < v->nseg; i++) {
		const struct seg *s = &v->seg[i];

		if (s->vis == 'v' && s->kind != 'c' &&
		    fabs(along(axis, s->a)) < ONE &&
		    fabs(along(axis, s->b)) < ONE) {
			q = fmax(q,
				 fmax(across(axis, s->a), across(axis, s->b)));
		}
	}
	return isfinite(q) ? q : far_side(v, axis);
}

// Draws the ordinates of one axis: the datum and the far end at the
// low side, and at the high side the datum if anything else is there.
static void ordinates(struct view *v, int axis)
{
	struct items it = { NULL, 0, 0 };
	size_t n_lo = 0;
	double *p = NULL;
	double half = (TEXT_H / 2) + PAD;
	struct span b = { half - ROW, far_side(v, !axis) + ROW - half };

	list_items(v, &it, axis);
	choose_items(v, &it, axis);
	qsort(it.v, it.n, sizeof(*it.v), cmp_place);
	while (n_lo < it.n && !it.v[n_lo].hi) {
		n_lo++;
	}
	if (n_lo < it.n) {
		struct item x = it.v[0];

		x.hi = 1;
		x.q = datum_reach(v, axis);
		PUSH(it.v, it.n, it.cap, x);
		qsort(it.v, it.n, sizeof(*it.v), cmp_place);
	}
	p = xmalloc((it.n + 1) * sizeof(*p));
	for (size_t i = 0; i < it.n; i++) {
		p[i] = it.v[i].p;
	}
	spread(p, n_lo);
	spread(p + n_lo, it.n - n_lo);
	keep_between(p, n_lo, b, axis);
	keep_between(p + n_lo, it.n - n_lo, b, !axis);
	for (size_t i = 0; i < it.n; i++) {
		draw_ordinate(v, axis, &it.v[i], p[i]);
		v->crowd = fmax(v->crowd, fabs(p[i] - it.v[i].p));
	}
	free(p);
	free(it.v);
}

// Reads an end of a linear dimension, in inches from the datum, and
// dies unless an ordinate may stand there.  Returns it on the paper.
static double lin_end(const struct view *v, struct reader *r, int axis)
{
	double p = num(r) * v->scale;

	for (size_t i = 0; i < v->nplace[axis]; i++) {
		if (fabs(v->place[axis][i] - p) < 1 / THOU / 2) {
			return p;
		}
	}
	die("%s:%u: lin %c: no edge or hole at %s", r->what, r->line,
	    axis ? 'y' : 'x', inches(p / v->scale).s);
}

// Draws the linear dimension of the rule in r, below or left of all
// that is drawn so far, where its lines to the two ends start: the
// text over the middle of its line, and the two tolerances after the
// text, one above the other.
static void linear(struct view *v, struct reader *r)
{
	const char *ax = word(r);
	int axis = strcmp(ax, "y") == 0;
	double up = axis ? -1 : 1;
	double a = lin_end(v, r, axis);
	double b = lin_end(v, r, axis);
	double row = (axis ? v->box.x0 : v->box.y0) - ROW;
	struct label t = inches(fabs(b - a) / v->scale);
	double end = ((a + b) / 2) + (text_width(t.s, TEXT_H) / 2) + PAD;
	double top = row + (up * 2 * TEXT_H);
	double tol_w = 0;
	struct style st = { pq(axis, (a + b) / 2, row + (up * TEXT_H)),
			    axis ? UPRIGHT : 0, 'c', TEXT_H };

	if (!axis && strcmp(ax, "x") != 0) {
		die("%s:%u: expected lin x|y A B [PLUS MINUS]", r->what,
		    r->line);
	}
	thin(pq(axis, a, row + ROW - GAP), pq(axis, a, row - JOG0));
	thin(pq(axis, b, row + ROW - GAP), pq(axis, b, row - JOG0));
	arrow(pq(axis, a, row), pq(axis, a - b, 0));
	arrow(pq(axis, b, row), pq(axis, b - a, 0));
	thin(pq(axis, a, row), pq(axis, b, row));
	text(st, t.s);
	printf("dim lin %c %.4f %s\n", axis ? 'y' : 'x', fabs(b - a) / v->k,
	       t.s);
	st.align = 'l';
	st.h = SMALL_H;
	for (int k = 1; r->k < r->n; k = -1) {
		const char *tol = word(r);

		st.at = pq(axis, end,
			   row + (up * (TEXT_H + (k * ((SMALL_H / 2) + GAP)))));
		text(st, tol);
		top = row + (up * (TEXT_H + (2 * SMALL_H)));
		tol_w = fmax(tol_w, text_width(tol, SMALL_H) + PAD);
	}
	take(v, corners(pq(axis, (a + b) - end, row - (up * PAD)),
			pq(axis, end + tol_w, top)));
}

// Adds the point at, with the radius r of what is there, to note nt.
static void add_target(struct note *nt, struct pt at, double r)
{
	struct target t = { at, r };

	PUSH(nt->to, nt->n, nt->cap, t);
}

// A place for a note: the leader meets the text at at, at half the
// height of its first line, and the text runs right or left from there.
struct spot {
	struct pt at;
	int right;
};

// The room the text of note nt takes at spot s.
static struct rect note_rect(const struct note *nt, struct spot s)
{
	double w = nt->flag ? FLAG_W : 0;
	double h = nt->flag ? FLAG_W : TEXT_H;
	double side = s.right ? 1 : -1;
	struct pt a = { s.at.x + (side * (GAP - PAD)), s.at.y + (h / 2) + PAD };
	struct pt b = { 0, s.at.y - (h / 2) - PAD - ((nt->nlines - 1) * LEAD) };

	for (int i = 0; i < nt->nlines && !nt->flag; i++) {
		w = fmax(w, text_width(nt->line[i], TEXT_H));
	}
	if (nt->flag && nt->line[1][0]) {
		w += GAP + text_width(nt->line[1], TEXT_H);
	}
	b.x = s.at.x + (side * (GAP + w + PAD));
	return corners(a, b);
}

// Where the leader of a note at spot s turns toward its point.
static struct pt landing(struct spot s)
{
	struct pt p = { s.at.x + (s.right ? -LAND : LAND), s.at.y };

	return p;
}

// The width of rectangle r.
static double dims_of(struct rect r)
{
	return r.x1 - r.x0;
}

// Whether the segments from a to b and from c to d cross.
static int segs_cross(struct pt a, struct pt b, struct pt c, struct pt d)
{
	double abc = ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x));
	double abd = ((b.x - a.x) * (d.y - a.y)) - ((b.y - a.y) * (d.x - a.x));
	double cda = ((d.x - c.x) * (a.y - c.y)) - ((d.y - c.y) * (a.x - c.x));
	double cdb = ((d.x - c.x) * (b.y - c.y)) - ((d.y - c.y) * (b.x - c.x));

	return (abc > 0) != (abd > 0) && (cda > 0) != (cdb > 0);
}

// Whether the room r lies on obstacle o.
static int on_obstacle(const struct obstacle *o, struct rect r)
{
	if (o->box) {
		return rects_meet(r, corners(o->a, o->b));
	}
	return seg_meets(o->a, o->b, r);
}

// The cells of the grid of v that the room r reaches, as the rectangle
// of their columns and rows.
static struct cells cells_of(const struct view *v, struct rect r)
{
	struct cells c = { 0, 0, 0, 0 };

	c.x0 = lround(floor(fmax((r.x0 - v->inner.x0) / CELL, 0)));
	c.y0 = lround(floor(fmax((r.y0 - v->inner.y0) / CELL, 0)));
	c.x1 = lround(floor(
		fmin((r.x1 - v->inner.x0) / CELL, (double)(v->cols - 1))));
	c.y1 = lround(floor(
		fmin((r.y1 - v->inner.y0) / CELL, (double)(v->rows - 1))));
	return c;
}

// Counts obstacle o of v in each cell its box reaches: for cell k, in
// v->cell[k + 1].
static void count_obstacle(struct view *v, size_t o)
{
	struct cells c = cells_of(v, corners(v->obst[o].a, v->obst[o].b));

	for (long y = c.y0; y <= c.y1; y++) {
		for (long x = c.x0; x <= c.x1; x++) {
			v->cell[(y * v->cols) + x + 1]++;
		}
	}
}

// Files obstacle o of v under each cell its box reaches, at the place
// v->cell[k] of cell k, which moves on by one.
static void file_obstacle(struct view *v, size_t o)
{
	struct cells c = cells_of(v, corners(v->obst[o].a, v->obst[o].b));

	for (long y = c.y0; y <= c.y1; y++) {
		for (long x = c.x0; x <= c.x1; x++) {
			v->in_cell[v->cell[(y * v->cols) + x]++] = o;
		}
	}
}

static void add_obstacle(struct view *v, struct obstacle o)
{
	PUSH(v->obst, v->nobst, v->capobst, o);
}

// Lists what a text inside the view must keep off: its lines, the
// lines of its ordinates, and each arc as its square with room for a
// centre mark and a gap.  A text may lie on a hatch, whose lines stop
// round it (print_clear).
static void list_obstacles(struct view *v)
{
	struct obstacle o = { { 0, 0 }, { 0, 0 }, 0 };

	for (size_t i = 0; i < v->nseg; i++) {
		o.a = v->seg[i].a;
		o.b = v->seg[i].b;
		add_obstacle(v, o);
	}
	for (size_t i = 0; i < v->next; i++) {
		o.a = v->ext[i].a;
		o.b = v->ext[i].b;
		add_obstacle(v, o);
	}
	o.box = 1;
	for (size_t i = 0; i < v->narc; i++) {
		const struct arc *a = &v->arc[i];

		o.a = pq(0, a->c.x - a->r - MARK - PAD,
			 a->c.y - a->r - MARK - PAD);
		o.b = pq(0, a->c.x + a->r + MARK + PAD,
			 a->c.y + a->r + MARK + PAD);
		add_obstacle(v, o);
	}
}

// Sorts the obstacles of v by the cells of a grid over the view:
// v->in_cell[v->cell[k] .. v->cell[k + 1]] are those of cell k.
static void sort_obstacles(struct view *v)
{
	size_t n = 0;

	v->cols = lround(ceil((v->inner.x1 - v->inner.x0) / CELL)) + 1;
	v->rows = lround(ceil((v->inner.y1 - v->inner.y0) / CELL)) + 1;
	n = (size_t)(v->cols * v->rows);
	v->cell = xcalloc(n + 2, sizeof(*v->cell));
	for (size_t i = 0; i < v->nobst; i++) {
		count_obstacle(v, i);
	}
	for (size_t k = 0; k < n; k++) {
		v->cell[k + 1] += v->cell[k];
	}
	v->in_cell = xmalloc((v->cell[n] + 1) * sizeof(*v->in_cell));
	for (size_t i = 0; i < v->nobst; i++) {
		file_obstacle(v, i);
	}
	for (size_t k = n; k > 0; k--) {
		v->cell[k] = v->cell[k - 1];
	}
	v->cell[0] = 0;
}

// Whether the room r is clear of the obstacles of the view: a text
// there lies on none of them.
static int is_clear(const struct view *v, struct rect r)
{
	struct cells c = cells_of(v, r);

	for (long y = c.y0; y <= c.y1; y++) {
		for (long x = c.x0; x <= c.x1; x++) {
			size_t k = (size_t)((y * v->cols) + x);

			for (size_t i = v->cell[k]; i < v->cell[k + 1]; i++) {
				if (on_obstacle(&v->obst[v->in_cell[i]], r)) {
					return 0;
				}
			}
		}
	}
	return 1;
}

// Whether note nt may stand at spot s with its leader to target t: the
// text lies on no text, on no leader and on no point of a leader or a
// flag record, whose leader must reach it, and the leader crosses no
// text, no leader, and no ordinate text or the gap beside it.  At
// v->level 1 the leader may pass the gap beside an ordinate text, and
// at level 2 cross another leader too.
static int spot_free(const struct view *v, const struct note *nt, struct spot s,
		     const struct target *t)
{
	struct rect r = note_rect(nt, s);
	struct pt l = landing(s);

	for (size_t i = 0; i < v->npin; i++) {
		if (v->pin[i].x > r.x0 && v->pin[i].x < r.x1 &&
		    v->pin[i].y > r.y0 && v->pin[i].y < r.y1) {
			return 0;
		}
	}
	for (size_t i = 0; i < v->ntext; i++) {
		if (rects_meet(r, v->text[i]) ||
		    seg_meets(l, t->at, v->text[i]) ||
		    seg_meets(l, s.at, v->text[i])) {
			return 0;
		}
	}
	for (size_t i = 0; i < v->nrow && v->level < 1; i++) {
		if (seg_meets(l, t->at, v->row[i])) {
			return 0;
		}
	}
	for (size_t i = 0; i < v->nlead; i++) {
		const struct lead *o = &v->lead[i];

		if (seg_meets(o->a, o->b, r) ||
		    (v->level < 2 && (segs_cross(l, t->at, o->a, o->b) ||
				      segs_cross(l, s.at, o->a, o->b)))) {
			return 0;
		}
	}
	return 1;
}

// Spot i on the ring f round the view: up the right side, up the left
// side, then along the top and the bottom, where the text runs away
// from the middle.  Returns 0 past the last one.
static int ring_spot(const struct rect *f, size_t i, struct spot *s)
{
	size_t ny = (size_t)((f->y1 - f->y0) / STEP) + 1;
	size_t nx = (size_t)((f->x1 - f->x0) / STEP) + 1;

	if (i < 2 * ny) {
		s->right = i < ny;
		s->at.x = s->right ? f->x1 : f->x0;
		s->at.y = f->y0 + ((double)(i % ny) * STEP);
		return 1;
	}
	i -= 2 * ny;
	s->at.x = f->x0 + ((double)(i % nx) * STEP);
	s->at.y = i < nx ? f->y1 : f->y0;
	s->right = s->at.x > (f->x0 + f->x1) / 2;
	return i < 2 * nx;
}

// Draws note nt at spot s, with its leader to target t.
static void draw_note(struct view *v, const struct note *nt, struct spot s,
		      const struct target *t)
{
	double side = s.right ? 1 : -1;
	struct pt l = landing(s);
	struct pt at = t->at;
	double d = hypot(at.x - l.x, at.y - l.y);
	struct pt dir = { (at.x - l.x) / d, (at.y - l.y) / d };
	struct style st = { { s.at.x + (side * GAP), s.at.y },
			    0,
			    s.right ? 'l' : 'r',
			    TEXT_H };

	at.x -= dir.x * t->r;
	at.y -= dir.y * t->r;
	thin(s.at, l);
	thin(l, at);
	if (nt->dot) {
		printf("dot %.4f %.4f\n", at.x, at.y);
	} else {
		arrow(at, dir);
	}
	if (nt->flag) {
		printf("flag %.4f %.4f %s\n",
		       s.at.x + (side * (GAP + (FLAG_W / 2))), s.at.y,
		       nt->line[0]);
		st.at.x += side * (FLAG_W + GAP);
		text(st, nt->line[1]);
	}
	for (int i = 0; i < nt->nlines && !nt->flag; i++) {
		text(st, nt->line[i]);
		st.at.y -= LEAD;
	}
	take(v, note_rect(nt, s));
	cover_line(v, l, at);
}

// The best spot for a note so far: its place and target, the length
// of its leader, and what the leader counts: its length, more for one
// that runs level or upright.
struct choice {
	struct spot s;
	const struct target *t;
	double len, cost;
};

// Tries spot s for note nt with each of its targets, and keeps the
// best in c: a leader within REACH before a longer one, then the one
// that counts least; none shorter than STUB.
static void try_spot(const struct view *v, const struct note *nt, struct spot s,
		     struct choice *c)
{
	struct pt l = landing(s);

	for (size_t k = 0; k < nt->n; k++) {
		const struct target *t = &nt->to[k];
		double dx = l.x - t->at.x;
		double dy = l.y - t->at.y;
		double run = hypot(dx, dy) - t->r;
		double cost = (run + PAD) *
			      (SLANT - (2 * fabs(sin(2 * atan2(dy, dx)))));
		int nearer = run <= REACH && c->len > REACH;
		int farther = run > REACH && c->len <= REACH;

		if (run > STUB && !farther && (nearer || cost < c->cost) &&
		    spot_free(v, nt, s, t)) {
			c->s = s;
			c->t = t;
			c->len = run;
			c->cost = cost;
		}
	}
}

// The room in which a leader of note nt may start and stay within
// REACH: the box of its targets, grown by REACH and by the largest of
// their radii.
static struct rect near_room(const struct note *nt)
{
	struct rect r = { HUGE_VAL, HUGE_VAL, -HUGE_VAL, -HUGE_VAL };
	double most = 0;

	for (size_t k = 0; k < nt->n; k++) {
		r.x0 = fmin(r.x0, nt->to[k].at.x);
		r.y0 = fmin(r.y0, nt->to[k].at.y);
		r.x1 = fmax(r.x1, nt->to[k].at.x);
		r.y1 = fmax(r.y1, nt->to[k].at.y);
		most = fmax(most, nt->to[k].r);
	}
	r.x0 -= REACH + most;
	r.y0 -= REACH + most;
	r.x1 += REACH + most;
	r.y1 += REACH + most;
	return r;
}

// Tries the spots inside the view for note nt, a grid of them with the
// text to either side: those where the text lies on no line of the
// view and, with near set, the leader starts in the room near.
static void try_inside(const struct view *v, const struct note *nt,
		       struct choice *c, const struct rect *near)
{
	long nx = lround(floor((v->inner.x1 - v->inner.x0) / STEP));
	long ny = lround(floor((v->inner.y1 - v->inner.y0) / STEP));

	for (long i = 0; i < nx * ny; i++) {
		double x = v->inner.x0 + ((double)(i % nx) * STEP);
		double y = v->inner.y0 + (floor((double)i / (double)nx) * STEP);

		for (int right = 0; right < 2; right++) {
			struct spot s = { { x, y }, right };
			struct rect r = note_rect(nt, s);
			struct pt l = landing(s);

			if (near && (l.x < near->x0 || l.x > near->x1 ||
				     l.y < near->y0 || l.y > near->y1)) {
				continue;
			}
			if (r.x0 > v->inner.x0 && r.x1 < v->inner.x1 &&
			    r.y0 > v->inner.y0 && r.y1 < v->inner.y1 &&
			    is_clear(v, r)) {
				try_spot(v, nt, s, c);
			}
		}
	}
}

// Tries the spots on the rings round the view for note nt, from the
// nearest out, until c holds a leader within REACH.
static void try_rings(const struct view *v, const struct note *nt,
		      struct choice *c)
{
	struct spot s;

	for (int ring = 0; ring < RINGS && c->len > REACH; ring++) {
		double m = STEP + (ring * RING);
		struct rect f = { v->frame.x0 - m, v->frame.y0 - m,
				  v->frame.x1 + m, v->frame.y1 + m };

		for (size_t i = 0; ring_spot(&f, i, &s); i++) {
			try_spot(v, nt, s, c);
		}
	}
}

// Dies for note nt, which has no room: by the line of its record, or
// for a callout, by its view.
static void no_room(const struct view *v, const struct note *nt,
		    const struct choice *c)
{
	double w = dims_of(note_rect(nt, c->s));

	if (v->at) {
		die("%s:%u: no room for the note %s, %.1f in wide: each place "
		    "on %d rings round the view lies on a text or a leader",
		    v->at->what, v->at->line, nt->line[0], w, RINGS);
	}
	die("view %s: no room for the callout %s, %.1f in wide: each place "
	    "on %d rings round the view lies on a text or a leader",
	    v->name, nt->line[0], w, RINGS);
}

// Puts note nt at its best free spot: inside the view, or on the rings
// round it, from the nearest out to the first that gives a leader
// within REACH; if there is none, at the next level of spot_free.  A
// leader longer than REACH, or one that only a later level allows,
// counts in v->far; after the loop, v->level is one past the level
// that gave the spot.  The spots inside from which no leader can stay
// within REACH come into it only if the others give none.  Frees the
// targets of the note.
static void place_note(struct view *v, struct note *nt)
{
	static const struct choice none = {
		{ { 0, 0 }, 0 }, NULL, HUGE_VAL, HUGE_VAL
	};
	struct choice c = none;
	struct rect near = near_room(nt);
	struct lead l = { { 0, 0 }, { 0, 0 } };

	for (v->level = 0; v->level < LEVELS && !c.t; v->level++) {
		try_inside(v, nt, &c, &near);
		if (c.len > REACH) {
			c = none;
			try_inside(v, nt, &c, NULL);
		}
		try_rings(v, nt, &c);
	}
	if (!c.t) {
		no_room(v, nt, &c);
	}
	v->far += c.len > REACH || v->level > 1;
	draw_note(v, nt, c.s, c.t);
	l.a = c.s.at;
	l.b = landing(c.s);
	PUSH(v->lead, v->nlead, v->caplead, l);
	l.a = c.t->at;
	PUSH(v->lead, v->nlead, v->caplead, l);
	free(nt->to);
	nt->to = NULL;
}

// Whether r covers the middle of a bend line other than b, where the
// leader of its own note may have to end.
static int on_bend(const struct view *v, const struct bend *b, struct rect r)
{
	for (size_t i = 0; i < v->nbend; i++) {
		const struct bend *o = &v->bend[i];
		struct pt mid = { (o->a.x + o->b.x) / 2,
				  (o->a.y + o->b.y) / 2 };

		if (o != b && mid.x > r.x0 && mid.x < r.x1 && mid.y > r.y0 &&
		    mid.y < r.y1) {
			return 1;
		}
	}
	return 0;
}

// Tries to write the note s of bend b along its line: at the share
// way.x of the way along it, on the side of the line that the sign of
// way.y names.  Returns 0 if the text would lie on a text or on a line
// of the view or, away from the middle of the line, pass its end.
static int bend_text(struct view *v, const struct bend *b, const char *s,
		     struct pt way)
{
	struct pt d = { b->b.x - b->a.x, b->b.y - b->a.y };
	int axis = fabs(d.x) < fabs(d.y);
	double w = (text_width(s, SMALL_H) / 2) + PAD;
	double run = fabs(along(axis, d));
	double p = along(axis, b->a) + (way.x * along(axis, d));
	double q = across(axis, b->a) + (way.y * (axis ? -SMALL_H : SMALL_H));
	struct style st = { pq(axis, p, q), axis ? UPRIGHT : 0, 'c', SMALL_H };
	struct rect r = corners(pq(axis, p - w, q - SMALL_H),
				pq(axis, p + w, q + SMALL_H));
	struct rect inside = { r.x0 + GAP, r.y0 + GAP, r.x1 - GAP, r.y1 - GAP };

	if ((way.x < MID || way.x > MID) &&
	    fabs(way.x - MID) * run > (run / 2) - w) {
		return 0;
	}
	if (!is_free(v, r) || on_bend(v, b, r) || !is_clear(v, inside)) {
		return 0;
	}
	text(st, s);
	take(v, r);
	return 1;
}

// Writes along each bend line of the flat view which way it bends and
// how far: UP toward the eye.  The text stands at the first place on
// either side of the line that is free of texts and lines, or with a
// leader if the line has none.
static void bend_notes(struct view *v)
{
	static const struct pt ways[] = { { 0.5, 1 },  { 0.5, -1 },
					  { 0.35, 1 }, { 0.35, -1 },
					  { 0.65, 1 }, { 0.65, -1 },
					  { 0.2, 1 },  { 0.2, -1 },
					  { 0.8, 1 },  { 0.8, -1 } };

	for (size_t i = 0; i < v->nbend; i++) {
		const struct bend *b = &v->bend[i];
		struct pt mid = { (b->a.x + b->b.x) / 2,
				  (b->a.y + b->b.y) / 2 };
		struct note nt;
		size_t k = 0;

		init_note(&nt);
		nt.nlines = 1;
		xsnprintf(nt.own[0], "%s %ld%%%%d",
			  b->angle > 0 ? "UP" : "DOWN", lround(fabs(b->angle)));
		printf("dim bend %g %s\n", b->angle, nt.line[0]);
		while (k < sizeof(ways) / sizeof(*ways) &&
		       !bend_text(v, b, nt.line[0], ways[k])) {
			k++;
		}
		if (k == sizeof(ways) / sizeof(*ways)) {
			add_target(&nt, mid, 0);
			place_note(v, &nt);
		}
	}
}

// The text of the count n of a callout, after the separator sep.
static struct label times(const char *sep, size_t n)
{
	struct label l = { "" };

	if (n > 1) {
		xsnprintf(l.s, "%s%zuX", sep, n);
	}
	return l;
}

// Whether two holes are of one kind, and share a callout.
static int same_hole(const struct hole *a, const struct hole *b)
{
	return fabs(a->d - b->d) < ONE && fabs(a->dc - b->dc) < ONE &&
	       fabs(a->ang - b->ang) < ONE && a->far == b->far;
}

// The pitch along the axis of the rows of the holes like h, or 0 if
// they stand in none.
static double row_pitch(const struct view *v, int axis, const struct hole *h)
{
	double pitch = 0;

	for (size_t i = 0; i < v->nhole; i++) {
		if (same_hole(&v->hole[i], h)) {
			pitch = fmax(pitch, in_pattern(v, axis, &v->hole[i]));
		}
	}
	return pitch;
}

// Adds a line to callout nt with the pitch of the rows of the holes
// like h, if they stand in rows: one number for one pitch both ways,
// or each with its axis.
static void pitch_line(const struct view *v, const struct hole *h,
		       struct note *nt)
{
	double x = row_pitch(v, 0, h);
	double y = row_pitch(v, 1, h);
	int same = fabs(x - y) < ONE;
	int split = x > 0 && y > 0 && !same;
	struct label a = inches((x > 0 ? x : y) / v->scale);
	struct label b = { "" };
	const char *axis = x > 0 ? " X" : " Y";

	if (split) {
		b = inches(y / v->scale);
	}
	if (x > 0 || y > 0) {
		xsnprintf(nt->own[nt->nlines], "PITCH %s%s%s%s%s", a.s,
			  same ? "" : axis, split ? ", " : "", b.s,
			  split ? " Y" : "");
		printf("dim pitch %.4f %.4f %s\n", x / v->k, y / v->k,
		       nt->line[nt->nlines]);
		nt->nlines++;
	}
}

// Writes the callout nt of the holes like h: the diameter and the
// count, and for a countersink a second line with its diameter and
// angle.
static void hole_text(const struct hole *h, struct note *nt)
{
	struct label d = inches(h->d / INCH);
	struct label dc = inches(h->dc / INCH);

	nt->nlines = 1;
	if (h->dc > 0) {
		xsnprintf(nt->own[0], "%%%%c%s THRU", d.s);
		xsnprintf(nt->own[1], "%%%%v%%%%c%s X %ld%%%%d%s%s", dc.s,
			  lround(h->ang), h->far ? " FARSIDE" : "",
			  times(", ", nt->n).s);
		nt->nlines = 2;
	} else {
		xsnprintf(nt->own[0], "%%%%c%s%s", d.s, times(" ", nt->n).s);
	}
}

// Whether hole i is the first of its kind.
static int first_hole(const struct view *v, size_t i)
{
	for (size_t k = 0; k < i; k++) {
		if (same_hole(&v->hole[k], &v->hole[i])) {
			return 0;
		}
	}
	return 1;
}

// Places one callout for each kind of hole, with the count.
static void hole_notes(struct view *v)
{
	for (size_t i = 0; i < v->nhole; i++) {
		const struct hole *h = &v->hole[i];
		struct note nt;

		if (!first_hole(v, i) ||
		    dropped(v, "d", inches(h->d / INCH).s)) {
			continue;
		}
		init_note(&nt);
		for (size_t k = i; k < v->nhole; k++) {
			const struct hole *g = &v->hole[k];

			if (same_hole(g, h)) {
				double d = fmax(g->d, g->far ? 0 : g->dc);

				add_target(&nt, g->c, d * v->k / 2);
			}
		}
		hole_text(h, &nt);
		printf("dim hole %.4f %zu %s\n", h->d, nt.n, nt.line[0]);
		pitch_line(v, h, &nt);
		place_note(v, &nt);
	}
}

// Whether arc a is a rounded corner to call out: a visible arc of the
// outline or of a cut-out, less than a circle.  The rim of a hole and
// an arc hidden in part are of another kind.
static int is_round(const struct arc *a)
{
	return (a->kind == 'e' || a->kind == 'i') && a->vis == 'v' &&
	       a->a1 - a->a0 < FULL;
}

// Whether arc a is the first rounded corner of its radius; with centre
// set, the first of its radius about its centre.
static int first_round(const struct view *v, const struct arc *a, int centre)
{
	if (!is_round(a)) {
		return 0;
	}
	for (const struct arc *b = v->arc; b < a; b++) {
		if (is_round(b) && fabs(b->r - a->r) < ONE &&
		    (!centre ||
		     hypot(b->c.x - a->c.x, b->c.y - a->c.y) < ONE)) {
			return 0;
		}
	}
	return 1;
}

// Places one callout for each radius of the rounded corners, with the
// number of corners: an arrow to the middle of one of their arcs.
static void round_notes(struct view *v)
{
	for (size_t i = 0; i < v->narc; i++) {
		const struct arc *a = &v->arc[i];
		struct label r = inches(a->r / v->scale);
		struct note nt;

		if (!first_round(v, a, 0) || dropped(v, "r", r.s)) {
			continue;
		}
		init_note(&nt);
		for (size_t k = i; k < v->narc; k++) {
			const struct arc *b = &v->arc[k];
			double mid = (b->a0 + b->a1) / 2 * DEG;
			struct pt at = { b->c.x + (b->r * cos(mid)),
					 b->c.y + (b->r * sin(mid)) };

			if (first_round(v, b, 1) && fabs(b->r - a->r) < ONE) {
				add_target(&nt, at, 0);
			}
		}
		nt.nlines = 1;
		xsnprintf(nt.own[0], "R%s%s", r.s, times(" ", nt.n).s);
		printf("dim radius %.4f %zu %s\n", a->r / v->k, nt.n,
		       nt.line[0]);
		place_note(v, &nt);
	}
}

// Places the note of the leader or flag rule in r: its text is the
// rest of the line s after the point, and for a flag after the number
// of its note.
static void rule_note(struct view *v, struct reader *r, const char *s)
{
	struct note nt;
	struct pt at = point(r);

	init_note(&nt);
	at.x *= v->scale;
	at.y *= v->scale;
	add_target(&nt, at, 0);
	nt.dot = 1;
	nt.nlines = 1;
	nt.flag = r->w[0][0] == 'f';
	for (int i = 0; i < 3 + nt.flag; i++) {
		s += strspn(s, " \t");
		s += strcspn(s, " \t");
	}
	s += strspn(s, " \t");
	if (nt.flag) {
		nt.line[0] = word(r);
	} else if (!*s) {
		die("%s:%u: a leader without its text", r->what, r->line);
	}
	nt.line[nt.flag] = s;
	place_note(v, &nt);
}

// Reads the hatch record in r: prints it on the paper, and keeps its
// rectangle.
static void hatch_rule(struct view *v, struct reader *r)
{
	struct pt a = point(r);
	struct pt b = point(r);
	struct rect h = corners(a, b);

	h.x0 *= v->scale;
	h.y0 *= v->scale;
	h.x1 *= v->scale;
	h.y1 *= v->scale;
	printf("hatch %.4f %.4f %.4f %.4f\n", h.x0, h.y0, h.x1, h.y1);
	PUSH(v->hatch, v->nhatch, v->caphatch, h);
}

// Keeps the point of the leader or flag record in r, which no callout
// text may cover.
static void pin_rule(struct view *v, struct reader *r)
{
	struct pt at = point(r);

	at.x *= v->scale;
	at.y *= v->scale;
	PUSH(v->pin, v->npin, v->cappin, at);
}

// Follows the lin and hatch records, and notes the points of the
// leader and flag records, if lin is set; or follows the leader and
// flag records, which come after the callouts.
static void rules(struct view *v, int lin)
{
	for (size_t i = 0; i < v->nrule; i++) {
		char *copy = xstrdup(v->rule[i].s);
		struct reader r = { v->rule[i].what, NULL, v->rule[i].line - 1,
				    { NULL },	     0,	   0 };
		const char *k = NULL;
		int note = 0;

		split_record(&r, copy);
		k = word(&r);
		note = strcmp(k, "leader") == 0 || strcmp(k, "flag") == 0;
		if (lin && strcmp(k, "lin") == 0) {
			linear(v, &r);
		} else if (lin && strcmp(k, "hatch") == 0) {
			hatch_rule(v, &r);
		} else if (lin && note) {
			pin_rule(v, &r);
		} else if (note) {
			v->at = &v->rule[i];
			rule_note(v, &r, v->rule[i].s);
			v->at = NULL;
		}
		free(copy);
	}
}

// Prints a clear record for each text that lies on a hatch.
static void print_clear(const struct view *v)
{
	for (size_t i = 0; i < v->ntext; i++) {
		const struct rect *t = &v->text[i];

		for (size_t k = 0; k < v->nhatch; k++) {
			if (rects_meet(*t, v->hatch[k])) {
				printf("clear %.4f %.4f %.4f %.4f\n", t->x0,
				       t->y0, t->x1, t->y1);
				break;
			}
		}
	}
}

static void free_view(struct view *v)
{
	for (size_t i = 0; i < v->nrule; i++) {
		free(v->rule[i].s);
	}
	for (size_t i = 0; i < v->nnames; i++) {
		free(v->names[i].s);
	}
	free(v->names);
	free(v->fill);
	free(v->rule);
	free(v->seg);
	free(v->arc);
	free(v->hole);
	free(v->bend);
	free(v->text);
	free(v->row);
	free(v->lead);
	free(v->ext);
	free(v->pin);
	free(v->hatch);
	free(v->obst);
	free(v->cell);
	free(v->in_cell);
	free(v->place[0]);
	free(v->place[1]);
}

int main(int argc, char **argv)
{
	struct reader args = { "-s", NULL, 0, { NULL }, 1, 0 };
	struct view v;
	char *input = NULL;

	memset(&v, 0, sizeof(v));
	v.name = v.part = "";
	v.scale = 1;
	if (argc == 3 && strcmp(argv[1], "-s") == 0) {
		args.w[0] = argv[2];
		v.scale = num(&args);
	}
	if ((argc != 1 && argc != 3) || !(v.scale > 0)) {
		die("usage: smview VIEW part.fold | smdim [-s SCALE] > part.dim");
	}
	v.k = v.scale / INCH;
	input = read_file("-");
	read_input(&v, input);
	printf("view %s %s %g\nthick %.4f\n", v.name, v.part, v.scale, v.thick);
	if (v.radius > 0) {
		printf("radius %.4f\n", v.radius);
	}
	print_fills(&v);
	print_curves(&v);
	ordinates(&v, 0);
	ordinates(&v, 1);
	rules(&v, 1);
	v.frame = v.box;
	list_obstacles(&v);
	sort_obstacles(&v);
	bend_notes(&v);
	hole_notes(&v);
	round_notes(&v);
	rules(&v, 0);
	print_clear(&v);
	printf("far %d\ncrowd %.3f\nbox %.4f %.4f %.4f %.4f\n", v.far, v.crowd,
	       v.box.x0, v.box.y0, v.box.x1, v.box.y1);
	free_view(&v);
	free(input);
	close_stdout();
	return 0;
}
