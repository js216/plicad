// SPDX-License-Identifier: GPL-3.0-only
// mould.c -- Report the walls and the drafts of exact solids for moulding
// Copyright (c) 2026 Jakob Kastelic

// Usage: mould x|y|z STREAM|- [PIN ...] > report
//
// Reads a stream (stream.h) and prints, for each solid, what a mould
// that opens along the axis given needs to know:
//   part NAME
//   pull AXIS
//   wall thin T at X Y Z
//   wall thick T at X Y Z
//   side plus faces N draft D
//   group plus KEY faces N draft D
//   side minus faces N draft D
//   group minus KEY faces N draft D
//   flat NAME
//   rim NAME
//   turn NAME
//   undercut NAME at X Y Z
//   pin NAME at X Y Z
//
// The solid is tried at points of its faces, STEP apart or closer.  The
// thin wall is the least way through the solid from such a point,
// straight in along the normal, to a face across: one whose normal
// there turns more than a third of a turn from that of the point.  A
// way out through a nearer face is the corner of an edge, and no wall.
// The thick wall is the diameter of the
// greatest ball that lies in the solid and touches it at such a point;
// the ball is held against the points alone, so its size holds to about
// STEP squared over the wall.
//
// A face is drawn toward the side its normal leans to along the axis,
// plus or minus.  Its draft is the least angle, over the face and its edges,
// between the face and the axis, in degrees.  A side counts its faces
// and gives the least draft of those that have one; so does each group
// of them, the faces of one extrusion, by the name before the first dot,
// or the rounds, by KEY round, in the order they come.  The faces that
// have none are listed: flat, along the axis throughout; rim, where the
// draft falls to 0 at an edge, as a round does where it runs out on a
// flat face; turn, where the face leans both ways, and the line where
// the mould parts crosses it.  An undercut is a face with a point that
// the mould cannot leave: the solid stands in the way of it toward the
// side of the face, or both ways for a point that leans to neither.
// One point is given.  An undercut face whose name matches a PIN, a
// pattern with * for any run of characters, is one that a side pin
// makes, a pin, and no fault.  Of spots of one size the first is given, of the
// faces in their order.

#include "geom.h"
#include "isect.h"
#include "shape.h"
#include "stream.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STEP 1.5 // The most between two points of a face, in mm.
#define OFF 1e-6 // How far off a face a ray starts, in mm.
#define LEAN 1e-9 // The sine of a draft that counts as none.
#define MIDDLE 0.5 // A point of the grid lies in the middle of its step.
#define TIE 1e-6 // Walls this near in size are one size, in mm.
#define ACROSS (-0.5) // The cosine of the turn to a face across a wall.
#define HALF_TURN 180.0 // In degrees.

enum {
	PINS = 3, // The place of the first PIN among the arguments.
	GRID_MIN = 3, // The least points across a face, each way.
	GRID_MAX = 96, // The most.
	EDGE_PTS = 16, // The pieces of an edge for the box and the draft.
	LEN_PTS = 8 // The pieces of a face for its size.
};

// A point of a face: its place, the outward normal there, the face, and
// the way through the solid.
struct pt {
	struct vec p, n;
	size_t f;
	double through;
};

struct pts {
	struct pt *v;
	size_t n, cap;
};

// What the points of a face say of its draft: the least and the
// greatest lean of its normal along the axis, and an undercut point.
struct lean {
	double lo, hi;
	int under;
	struct vec at;
};

// The work on one solid.
struct job {
	const struct shape *sh;
	struct vec axis;
	struct box *box;
	struct pts pts;
	struct lean *lean;
	char **pin; // The patterns of the faces that side pins make.
	int npin;
};

static struct vec out_normal(const struct shape *sh, size_t f, struct vec p)
{
	return mul(surf_normal(&sh->f[f].s, p), sh->f[f].sense);
}

// Whether the ray from o along d, of length tmax, may pass through box
// b.
static int ray_in_box(struct vec o, struct vec d, double tmax, struct box b)
{
	const double oo[3] = { o.x, o.y, o.z };
	const double dd[3] = { d.x, d.y, d.z };
	const double lo[3] = { b.lo.x, b.lo.y, b.lo.z };
	const double hi[3] = { b.hi.x, b.hi.y, b.hi.z };
	double t0 = 0;
	double t1 = tmax;

	for (int k = 0; k < 3; k++) {
		double a = (lo[k] - oo[k]) / dd[k];
		double z = (hi[k] - oo[k]) / dd[k];

		if (fabs(dd[k]) < LEAN) {
			if (oo[k] < lo[k] || oo[k] > hi[k]) {
				return 0;
			}
			continue;
		}
		t0 = fmax(t0, fmin(a, z));
		t1 = fmin(t1, fmax(a, z));
	}
	return t0 <= t1;
}

// The way along the ray from o along d, a unit vector, to the first
// face of the solid, which goes into *hit; HUGE_VAL if it meets none.
static double first_hit(const struct job *j, struct vec o, struct vec d,
			size_t *hit)
{
	const struct shape *sh = j->sh;
	struct cpiece pc;
	double best = HUGE_VAL;

	memset(&pc, 0, sizeof(pc));
	pc.c.kind = CV_LINE;
	pc.c.o = o;
	pc.c.x = d;
	pc.t0 = 0;
	pc.t1 = len(sub(j->box[sh->nf].hi, j->box[sh->nf].lo));
	for (size_t f = 0; f < sh->nf; f++) {
		double t[ROOTS_MAX];
		size_t n = 0;

		if (!ray_in_box(o, d, fmin(pc.t1, best), j->box[f])) {
			continue;
		}
		n = curve_surf(&pc, &sh->f[f].s, t);
		for (size_t i = 0; i < n; i++) {
			if (t[i] > 0 && t[i] < best &&
			    face_has(sh, f, add(o, mul(d, t[i])))) {
				best = t[i];
				*hit = f;
			}
		}
	}
	return best;
}

// Takes the lean of the normal of face f at p into its record.
static double note_lean(struct job *j, size_t f, struct vec p)
{
	double l = dot(out_normal(j->sh, f, p), j->axis);

	j->lean[f].lo = fmin(j->lean[f].lo, l);
	j->lean[f].hi = fmax(j->lean[f].hi, l);
	return l;
}

// The box of the parameters of face f, by points of its edges, which
// also give their leans.
static void uv_box(struct job *j, const struct facemap *m, struct uv *lo,
		   struct uv *hi)
{
	const struct shape *sh = j->sh;
	const struct sface *fc = &sh->f[m->f];

	lo->u = HUGE_VAL;
	lo->v = HUGE_VAL;
	hi->u = -HUGE_VAL;
	hi->v = -HUGE_VAL;
	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n * (EDGE_PTS + 1); k++) {
			const struct sedge *e =
				&sh->e[sh->u[lp->first + (k / (EDGE_PTS + 1))].e];
			double s =
				(double)(k % (EDGE_PTS + 1)) / (double)EDGE_PTS;
			struct vec p =
				curve_at(&e->c, e->t0 + ((e->t1 - e->t0) * s));
			struct uv q = fm_uv(m, p);

			(void)note_lean(j, m->f, p);
			lo->u = fmin(lo->u, q.u);
			lo->v = fmin(lo->v, q.v);
			hi->u = fmax(hi->u, q.u);
			hi->v = fmax(hi->v, q.v);
		}
	}
}

// The count of points across a stretch of a face from a to b in its
// parameters, by its length through LEN_PTS pieces.
static size_t grid_count(const struct surf *s, struct uv a, struct uv b)
{
	double l = 0;
	struct vec prev = surf_at(s, a);
	size_t n = GRID_MIN;

	for (int i = 1; i <= LEN_PTS; i++) {
		double w = (double)i / (double)LEN_PTS;
		struct uv q = { a.u + ((b.u - a.u) * w),
				a.v + ((b.v - a.v) * w) };
		struct vec p = surf_at(s, q);

		l += len(sub(p, prev));
		prev = p;
	}
	while (n < GRID_MAX && l / (double)n > STEP) {
		n++;
	}
	return n;
}

// The share of a stretch at point i of n, half a step in from its ends.
static double share(size_t i, size_t n)
{
	return ((double)i + MIDDLE) / (double)n;
}

// Adds the points of face f: a grid over the box of its parameters,
// less the points outside it.
static void face_points(struct job *j, size_t f)
{
	const struct shape *sh = j->sh;
	const struct surf *s = &sh->f[f].s;
	struct facemap m;
	struct uv lo;
	struct uv hi;
	struct uv a;
	struct uv b;
	size_t nu = 0;
	size_t nv = 0;

	facemap_init(&m, sh, f);
	uv_box(j, &m, &lo, &hi);
	a.u = lo.u;
	a.v = (lo.v + hi.v) / 2;
	b.u = hi.u;
	b.v = a.v;
	nu = grid_count(s, a, b);
	a.u = (lo.u + hi.u) / 2;
	a.v = lo.v;
	b.u = a.u;
	b.v = hi.v;
	nv = grid_count(s, a, b);
	for (size_t iu = 0; iu < nu; iu++) {
		for (size_t iv = 0; iv < nv; iv++) {
			struct uv q = { lo.u + ((hi.u - lo.u) * share(iu, nu)),
					lo.v + ((hi.v - lo.v) *
						share(iv, nv)) };
			struct pt x = { surf_at(s, q), v3(0, 0, 0), f, 0 };

			if (face_has(sh, f, x.p)) {
				x.n = out_normal(sh, f, x.p);
				PUSH(j->pts.v, j->pts.n, j->pts.cap, x);
			}
		}
	}
}

// Whether the solid stands in the way of the point x as the mould
// leaves: toward the side its face leans to there, or both ways if it
// leans to neither.
static int held(const struct job *j, const struct pt *x)
{
	double l = dot(x->n, j->axis);
	struct vec o = add(x->p, mul(x->n, OFF));
	size_t f = 0;
	int fore = first_hit(j, o, j->axis, &f) < HUGE_VAL;
	int back = first_hit(j, o, mul(j->axis, -1), &f) < HUGE_VAL;

	if (l > LEAN) {
		return fore;
	}
	return l < -LEAN ? back : fore && back;
}

// The diameter of the greatest ball in the solid that touches it at
// point x and is clear of every other point, if above least; else any
// value not above least.
static double ball_at(const struct pts *ps, const struct pt *x, double least)
{
	double d = x->through;

	for (size_t k = 0; k < ps->n && d > least; k++) {
		struct vec to = sub(ps->v[k].p, x->p);
		double in = -dot(to, x->n);

		if (in > OFF) {
			d = fmin(d, dot(to, to) / in);
		}
	}
	return d;
}

static void print_at(const char *what, double t, struct vec p)
{
	printf("%s %.3f at %.3f %.3f %.3f\n", what, t, p.x + 0.0, p.y + 0.0,
	       p.z + 0.0);
}

// Whether the way through the solid from point x, of length t, ends on
// face f across a wall.
static int across(const struct job *j, const struct pt *x, double t, size_t f)
{
	struct vec q = sub(x->p, mul(x->n, t));

	return t < HUGE_VAL && dot(out_normal(j->sh, f, q), x->n) < ACROSS;
}

// Prints the thin and the thick wall.
static void print_walls(struct job *j)
{
	struct pts *ps = &j->pts;
	size_t thin = NONE;
	size_t thick = 0;
	double ball = 0;

	for (size_t i = 0; i < ps->n; i++) {
		struct pt *x = &ps->v[i];
		size_t f = 0;

		x->through = first_hit(j, sub(x->p, mul(x->n, OFF)),
				       mul(x->n, -1), &f);
		if (across(j, x, x->through, f) &&
		    (thin == NONE || x->through < ps->v[thin].through - TIE)) {
			thin = i;
		}
		if (held(j, x) && !j->lean[x->f].under) {
			j->lean[x->f].under = 1;
			j->lean[x->f].at = x->p;
		}
	}
	for (size_t i = 0; i < ps->n; i++) {
		double d = ball_at(ps, &ps->v[i], ball + TIE);

		if (d > ball + TIE && d < HUGE_VAL) {
			ball = d;
			thick = i;
		}
	}
	if (thin != NONE) {
		print_at("wall thin", ps->v[thin].through, ps->v[thin].p);
	}
	if (ps->n) {
		print_at("wall thick", ball, ps->v[thick].p);
	}
}

// The draft of a lean, in degrees.
static double draft_of(double l)
{
	return asin(fmin(1, fabs(l))) * HALF_TURN / PI;
}

// The least lean of face f toward side, 1 for plus and -1 for minus,
// or -1 if the face is not drawn that way.
static double side_lean(const struct job *j, size_t f, int side)
{
	double a = side * j->lean[f].lo;
	double b = side * j->lean[f].hi;
	double lo = fmin(a, b);

	return lo > -LEAN && fmax(a, b) > LEAN ? lo : -1;
}

// The length of the key of the group of a face of that name: the name
// before the first dot; 0 for a round.
static size_t key_len(const char *name)
{
	return strchr(name, '+') ? 0 : strcspn(name, ".");
}

// Whether the faces of names a and b fall in one group: of one
// extrusion, by their keys, or both rounds.
static int one_group(const char *a, const char *b)
{
	size_t n = key_len(a);

	return n == key_len(b) && !strncmp(a, b, n);
}

// Prints the count and the least draft of the faces drawn toward side
// in the group of face g, if g is the first of them.
static void print_group(const struct job *j, size_t g, int side)
{
	const struct shape *sh = j->sh;
	size_t n = 0;
	double least = 1;

	if (side_lean(j, g, side) < 0) {
		return;
	}
	for (size_t f = 0; f < sh->nf; f++) {
		double l = one_group(sh->f[f].name, sh->f[g].name) ?
				   side_lean(j, f, side) :
				   -1;

		if (f < g && !(l < 0)) {
			return;
		}
		if (!(l < 0)) {
			n++;
			least = l > LEAN ? fmin(least, l) : least;
		}
	}
	if (n) {
		const char *name = sh->f[g].name;
		size_t k = key_len(name);

		printf("group %s %.*s faces %zu draft %.3f\n",
		       side > 0 ? "plus" : "minus",
		       k ? (int)k : (int)strlen("round"), k ? name : "round", n,
		       draft_of(least));
	}
}

// Prints one side, 1 for plus and -1 for minus: the count of its faces
// and the least draft of those that have one, then the same for each
// group of them.
static void print_side(const struct job *j, int side)
{
	size_t n = 0;
	double least = 1;

	for (size_t f = 0; f < j->sh->nf; f++) {
		double l = side_lean(j, f, side);

		if (!(l < 0)) {
			n++;
			least = l > LEAN ? fmin(least, l) : least;
		}
	}
	printf("side %s faces %zu draft %.3f\n", side > 0 ? "plus" : "minus", n,
	       draft_of(least));
	for (size_t g = 0; g < j->sh->nf; g++) {
		print_group(j, g, side);
	}
}

// What a face without a draft is: flat, turn or rim; NULL for a face
// with one.
static const char *no_draft(const struct lean *l)
{
	if (l->lo < -LEAN && l->hi > LEAN) {
		return "turn";
	}
	if (l->lo > -LEAN && l->hi < LEAN) {
		return "flat";
	}
	return fmin(fabs(l->lo), fabs(l->hi)) > LEAN ? NULL : "rim";
}

// Prints face f, which the mould cannot leave: a pin if a pattern has
// its name, else an undercut.
static void print_held(const struct job *j, size_t f)
{
	const char *name = j->sh->f[f].name;
	struct vec p = j->lean[f].at;
	const char *what = "undercut";

	for (int i = 0; i < j->npin; i++) {
		if (glob(j->pin[i], name)) {
			what = "pin";
		}
	}
	printf("%s %s at %.3f %.3f %.3f\n", what, name, p.x + 0.0, p.y + 0.0,
	       p.z + 0.0);
}

// Prints the sides, then the faces without a draft and the undercuts.
static void print_drafts(const struct job *j)
{
	const struct shape *sh = j->sh;

	print_side(j, 1);
	print_side(j, -1);
	for (size_t f = 0; f < sh->nf; f++) {
		const char *what = no_draft(&j->lean[f]);

		if (what) {
			printf("%s %s\n", what, sh->f[f].name);
		}
	}
	for (size_t f = 0; f < sh->nf; f++) {
		if (j->lean[f].under) {
			print_held(j, f);
		}
	}
}

static void report(const struct shape *sh, int argc, char **argv)
{
	struct job j;
	const char *axis = argv[1];

	memset(&j, 0, sizeof(j));
	j.sh = sh;
	j.pin = argv + PINS;
	j.npin = argc - PINS;
	j.axis = v3(axis[0] == 'x', axis[0] == 'y', axis[0] == 'z');
	j.box = xmalloc((sh->nf + 1) * sizeof(*j.box));
	j.lean = xmalloc((sh->nf + 1) * sizeof(*j.lean));
	j.box[sh->nf] = box_empty();
	for (size_t f = 0; f < sh->nf; f++) {
		j.box[f] = face_box(sh, f);
		box_pad(&j.box[f], STEP);
		box_add(&j.box[sh->nf], j.box[f].lo);
		box_add(&j.box[sh->nf], j.box[f].hi);
		j.lean[f].lo = HUGE_VAL;
		j.lean[f].hi = -HUGE_VAL;
		j.lean[f].under = 0;
		j.lean[f].at = v3(0, 0, 0);
	}
	for (size_t f = 0; f < sh->nf; f++) {
		face_points(&j, f);
	}
	for (size_t i = 0; i < j.pts.n; i++) {
		(void)note_lean(&j, j.pts.v[i].f, j.pts.v[i].p);
	}
	printf("part %s\npull %s\n", sh->name, axis);
	print_walls(&j);
	print_drafts(&j);
	free(j.box);
	free(j.lean);
	free(j.pts.v);
}

int main(int argc, char **argv)
{
	struct stream st;

	if (argc < PINS || strlen(argv[1]) != 1 || !strchr("xyz", argv[1][0])) {
		die("usage: mould x|y|z STREAM|- [PIN ...]");
	}
	stream_read(&st, argv[2]);
	stream_refuse_left(&st);
	for (size_t i = 0; i < st.nsh; i++) {
		report(&st.sh[i], argc, argv);
	}
	stream_free(&st);
	close_stdout();
	return 0;
}
