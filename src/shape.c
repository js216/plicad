// SPDX-License-Identifier: GPL-3.0-only
// shape.c -- Exact solids: vertices, edges on curves, faces on surfaces
// Copyright (c) 2026 Jakob Kastelic

#include "shape.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	VEC_WORDS = 3, // Numbers of a point or a direction.
	FRAME_WORDS = 9, // o, n and x.
	EDGE_HEAD = 6, // e V0 V1 T0 T1 KIND
	SIZES_MAX = 2, // The most sizes of a surface.
	RECORD_MAX = 32 // The most words of a record but a loop.
};

static const char *const curve_names[CV_KINDS] = { "line", "circle",
						   "ellipse" };
static const char *const surf_names[SF_KINDS] = { "plane", "cylinder", "cone",
						  "sphere", "torus" };
// The sizes after the frame of each kind of curve and surface.
static const size_t curve_sizes[CV_KINDS] = { 0, 1, 2 };
static const size_t surf_sizes[SF_KINDS] = { 0, 1, 2, 1, 2 };

struct vec frame_y(struct vec n, struct vec x)
{
	return cross(n, x);
}

// The direction at angle u about the frame of s.
static struct vec dir_at(struct vec x, struct vec y, double u)
{
	return add(mul(x, cos(u)), mul(y, sin(u)));
}

struct vec curve_at(const struct curve *c, double t)
{
	struct vec y = frame_y(c->n, c->x);

	switch (c->kind) {
	case CV_LINE:
		return add(c->o, mul(c->x, t));
	case CV_CIRCLE:
		return add(c->o, mul(dir_at(c->x, y, t), c->a));
	default:
		return add(c->o, add(mul(c->x, c->a * cos(t)),
				     mul(y, c->b * sin(t))));
	}
}

struct vec curve_tan(const struct curve *c, double t)
{
	struct vec y = frame_y(c->n, c->x);

	switch (c->kind) {
	case CV_LINE:
		return c->x;
	case CV_CIRCLE:
		return mul(dir_at(y, mul(c->x, -1), t), c->a);
	default:
		return add(mul(c->x, -c->a * sin(t)), mul(y, c->b * cos(t)));
	}
}

double curve_param(const struct curve *c, struct vec p)
{
	struct vec d = sub(p, c->o);
	struct vec y = frame_y(c->n, c->x);
	double px = dot(d, c->x);
	double py = dot(d, y);

	switch (c->kind) {
	case CV_LINE:
		return px;
	case CV_CIRCLE:
		return atan2(py, px);
	default:
		return atan2(py / c->b, px / c->a);
	}
}

struct vec surf_at(const struct surf *s, struct uv q)
{
	struct vec y = frame_y(s->n, s->x);
	struct vec e = dir_at(s->x, y, q.u);

	switch (s->kind) {
	case SF_PLANE:
		return add(s->o, add(mul(s->x, q.u), mul(y, q.v)));
	case SF_CYL:
		return add(s->o, add(mul(s->n, q.v), mul(e, s->r)));
	case SF_CONE:
		return add(s->o,
			   add(mul(s->n, q.v), mul(e, s->r + (s->r2 * q.v))));
	case SF_SPHERE:
		return add(s->o, add(mul(e, s->r * cos(q.v)),
				     mul(s->n, s->r * sin(q.v))));
	default:
		return add(s->o, add(mul(e, s->r + (s->r2 * cos(q.v))),
				     mul(s->n, s->r2 * sin(q.v))));
	}
}

struct uv surf_param(const struct surf *s, struct vec p)
{
	struct vec d = sub(p, s->o);
	struct vec y = frame_y(s->n, s->x);
	double px = dot(d, s->x);
	double py = dot(d, y);
	double h = dot(d, s->n);
	struct uv q = { atan2(py, px), h };

	if (s->kind == SF_PLANE) {
		q.u = px;
		q.v = py;
	} else if (s->kind == SF_SPHERE) {
		q.v = atan2(h, hypot(px, py));
	} else if (s->kind == SF_TORUS) {
		q.v = atan2(h, hypot(px, py) - s->r);
	}
	return q;
}

struct vec surf_normal(const struct surf *s, struct vec p)
{
	struct vec d = sub(p, s->o);
	struct vec radial = sub(d, mul(s->n, dot(d, s->n)));
	struct vec e = unit(radial);

	switch (s->kind) {
	case SF_PLANE:
		return s->n;
	case SF_CYL:
		return e;
	case SF_CONE:
		return unit(sub(e, mul(s->n, s->r2)));
	case SF_SPHERE:
		return unit(d);
	default:
		return unit(sub(d, mul(e, s->r)));
	}
}

double surf_dist(const struct surf *s, struct vec p)
{
	struct vec d = sub(p, s->o);
	double h = dot(d, s->n);
	double rho = len(sub(d, mul(s->n, h)));

	switch (s->kind) {
	case SF_PLANE:
		return h;
	case SF_CYL:
		return rho - s->r;
	case SF_CONE:
		return (rho - s->r - (s->r2 * h)) / sqrt(1 + (s->r2 * s->r2));
	case SF_SPHERE:
		return len(d) - s->r;
	default:
		return hypot(rho - s->r, h) - s->r2;
	}
}

double surf_dv(const struct surf *s, struct line3 t)
{
	struct vec p = t.o;
	struct vec d = t.d;
	struct vec r = sub(p, s->o);
	double h = dot(r, s->n);
	struct vec rad = sub(r, mul(s->n, h));
	double rho = len(rad);
	struct vec e = mul(rad, 1 / rho);
	double dh = dot(d, s->n);
	double dr = dot(d, e);
	double q = s->kind == SF_TORUS ? rho - s->r : rho;

	switch (s->kind) {
	case SF_PLANE:
		return dot(d, frame_y(s->n, s->x));
	case SF_CYL:
	case SF_CONE:
		return dh;
	default:
		return ((q * dh) - (h * dr)) / ((q * q) + (h * h));
	}
}

double angle_near(double a, double ref)
{
	return a - (2 * PI * floor(((a - ref) / (2 * PI)) + MID));
}

double curve_dist(const struct curve *c, struct vec p)
{
	return len(sub(p, curve_at(c, curve_param(c, p))));
}

double turn_from(double t, double from)
{
	return t + (2 * PI * ceil((from - t) / (2 * PI)));
}

int curve_turns(const struct curve *c)
{
	return c->kind != CV_LINE;
}

void shape_init(struct shape *sh, const char *name)
{
	memset(sh, 0, sizeof(*sh));
	sh->name = xstrdup(name);
}

void shape_free(struct shape *sh)
{
	for (size_t i = 0; i < sh->nf; i++) {
		free(sh->f[i].name);
	}
	free(sh->name);
	free(sh->v);
	free(sh->e);
	free(sh->u);
	free(sh->l);
	free(sh->f);
	memset(sh, 0, sizeof(*sh));
}

size_t shape_vertex(struct shape *sh, struct vec p)
{
	for (size_t i = 0; i < sh->nv; i++) {
		if (len(sub(sh->v[i], p)) < SH_EPS) {
			return i;
		}
	}
	PUSH(sh->v, sh->nv, sh->capv, p);
	return sh->nv - 1;
}

size_t shape_edge(struct shape *sh, struct sedge e)
{
	PUSH(sh->e, sh->ne, sh->cape, e);
	return sh->ne - 1;
}

size_t shape_piece(struct shape *sh, const struct curve *c, struct span t,
		   int *fwd)
{
	size_t va = shape_vertex(sh, curve_at(c, t.lo));
	size_t vb = shape_vertex(sh, curve_at(c, t.hi));
	struct vec m = curve_at(c, (t.lo + t.hi) / 2);
	struct sedge e;

	for (size_t i = 0; i < sh->ne; i++) {
		const struct sedge *x = &sh->e[i];

		if (((x->v0 == va && x->v1 == vb) ||
		     (x->v0 == vb && x->v1 == va)) &&
		    len(sub(edge_at(x, MID), m)) < SH_EPS) {
			*fwd = va != vb ? x->v0 == va :
					  dot(curve_tan(&x->c, x->t0),
					      curve_tan(c, t.lo)) > 0;
			return i;
		}
	}
	e.c = *c;
	e.t0 = t.lo;
	e.t1 = t.hi;
	e.v0 = va;
	e.v1 = vb;
	*fwd = 1;
	return shape_edge(sh, e);
}

void shape_face(struct shape *sh, struct surf s, int sense, const char *name)
{
	struct sface f = { s, sense, sh->nl, 0, xstrdup(name) };

	PUSH(sh->f, sh->nf, sh->capf, f);
}

void shape_use(struct shape *sh, size_t e, int fwd)
{
	struct suse u = { e, fwd };

	PUSH(sh->u, sh->nu, sh->capu, u);
}

void shape_end_loop(struct shape *sh, size_t first)
{
	struct sloop l = { first, sh->nu - first };

	PUSH(sh->l, sh->nl, sh->capl, l);
	sh->f[sh->nf - 1].nloops++;
}

struct urange face_uses(const struct shape *sh, size_t f)
{
	const struct sface *fc = &sh->f[f];
	struct urange r = { 0, 0 };

	if (fc->nloops) {
		const struct sloop *last = &sh->l[fc->loop0 + fc->nloops - 1];

		r.first = sh->l[fc->loop0].first;
		r.end = last->first + last->n;
	}
	return r;
}

struct vec edge_at(const struct sedge *e, double s)
{
	return curve_at(&e->c, e->t0 + ((e->t1 - e->t0) * s));
}

static void put_num(FILE *out, double v)
{
	(void)fprintf(out, " %.17g", v + 0.0);
}

static void put_vec(FILE *out, struct vec v)
{
	put_num(out, v.x);
	put_num(out, v.y);
	put_num(out, v.z);
}

static void put_curve(FILE *out, const struct curve *c)
{
	(void)fprintf(out, " %s", curve_names[c->kind]);
	put_vec(out, c->o);
	if (c->kind != CV_LINE) {
		put_vec(out, c->n);
	}
	put_vec(out, c->x);
	if (curve_sizes[c->kind] > 0) {
		put_num(out, c->a);
	}
	if (curve_sizes[c->kind] > 1) {
		put_num(out, c->b);
	}
}

static void put_surf(FILE *out, const struct surf *s)
{
	const double sizes[SIZES_MAX] = { s->r, s->r2 };

	(void)fprintf(out, " %s", surf_names[s->kind]);
	put_vec(out, s->o);
	put_vec(out, s->n);
	put_vec(out, s->x);
	for (size_t i = 0; i < surf_sizes[s->kind] && i < SIZES_MAX; i++) {
		put_num(out, sizes[i]);
	}
}

void shape_write(FILE *out, const struct shape *sh)
{
	(void)fprintf(out, "brep %s\n", sh->name);
	for (size_t i = 0; i < sh->nv; i++) {
		(void)fprintf(out, "v");
		put_vec(out, sh->v[i]);
		(void)fprintf(out, "\n");
	}
	for (size_t i = 0; i < sh->ne; i++) {
		const struct sedge *e = &sh->e[i];

		(void)fprintf(out, "e %zu %zu", e->v0, e->v1);
		put_num(out, e->t0);
		put_num(out, e->t1);
		put_curve(out, &e->c);
		(void)fprintf(out, "\n");
	}
	for (size_t i = 0; i < sh->nf; i++) {
		const struct sface *f = &sh->f[i];

		(void)fprintf(out, "f %s %d", f->name, f->sense);
		put_surf(out, &f->s);
		(void)fprintf(out, "\n");
		for (size_t l = 0; l < f->nloops; l++) {
			const struct sloop *lp = &sh->l[f->loop0 + l];

			(void)fprintf(out, "l");
			for (size_t k = 0; k < lp->n; k++) {
				const struct suse *u = &sh->u[lp->first + k];

				(void)fprintf(out, " %c%zu", u->fwd ? '+' : '-',
					      u->e);
			}
			(void)fprintf(out, "\n");
		}
	}
	(void)fprintf(out, "end\n");
}

// The words of a record from words w.
struct record {
	char **w;
	size_t n;
	const char *where;
};

static double word_num(const struct record *r, size_t i)
{
	char *end = NULL;
	double v = 0;

	if (i >= r->n) {
		die("%s: a number is missing", r->where);
	}
	v = strtod(r->w[i], &end);
	if (*end || end == r->w[i] || !num_ok(v)) {
		die("%s: bad number %s", r->where, r->w[i]);
	}
	return v;
}

// Word i of r as the index of one of n items.
static size_t word_index(size_t n, const struct record *r, size_t i)
{
	double v = word_num(r, i);

	if (v < 0 || v >= (double)n || floor(v) < v) {
		die("%s: no such item %s", r->where, r->w[i]);
	}
	return (size_t)v;
}

static struct vec word_vec(const struct record *r, size_t i)
{
	return v3(word_num(r, i), word_num(r, i + 1), word_num(r, i + 2));
}

static int kind_of(const char *w, const char *const *names, int n)
{
	for (int k = 0; k < n; k++) {
		if (strcmp(w, names[k]) == 0) {
			return k;
		}
	}
	return -1;
}

// Dies unless the frame n, x of a record is a pair of unit vectors at
// right angles.
static void check_frame(const struct record *r, struct vec n, struct vec x)
{
	if (fabs(len(n) - 1) > SH_EPS || fabs(len(x) - 1) > SH_EPS ||
	    fabs(dot(n, x)) > SH_EPS) {
		die("%s: the frame is not two unit vectors at right angles",
		    r->where);
	}
}

static void read_edge(struct shape *sh, const struct record *r)
{
	struct sedge e;
	size_t i = EDGE_HEAD;
	int k = 0;

	memset(&e, 0, sizeof(e));
	e.v0 = word_index(sh->nv, r, 1);
	e.v1 = word_index(sh->nv, r, 2);
	e.t0 = word_num(r, 3);
	e.t1 = word_num(r, 4);
	k = r->n > EDGE_HEAD - 1 ?
		    kind_of(r->w[EDGE_HEAD - 1], curve_names, CV_KINDS) :
		    -1;
	if (k < 0) {
		die("%s: unknown curve", r->where);
	}
	e.c.kind = k;
	e.c.o = word_vec(r, i);
	i += VEC_WORDS;
	if (k != CV_LINE) {
		e.c.n = word_vec(r, i);
		i += VEC_WORDS;
	}
	e.c.x = word_vec(r, i);
	i += VEC_WORDS;
	e.c.a = curve_sizes[k] > 0 ? word_num(r, i) : 0;
	e.c.b = curve_sizes[k] > 1 ? word_num(r, i + 1) : 0;
	if (r->n != i + curve_sizes[k] || !(e.t1 > e.t0)) {
		die("%s: bad edge", r->where);
	}
	if (k != CV_LINE) {
		check_frame(r, e.c.n, e.c.x);
	}
	(void)shape_edge(sh, e);
}

static void read_face(struct shape *sh, const struct record *r)
{
	struct surf s;
	double sizes[SIZES_MAX] = { 0, 0 };
	int k = r->n > 3 ? kind_of(r->w[3], surf_names, SF_KINDS) : -1;
	int sense = 0;

	if (k < 0) {
		die("%s: unknown surface", r->where);
	}
	memset(&s, 0, sizeof(s));
	s.kind = k;
	if (strcmp(r->w[2], "1") == 0) {
		sense = 1;
	} else if (strcmp(r->w[2], "-1") == 0) {
		sense = -1;
	} else {
		die("%s: the sense of a face is 1 or -1", r->where);
	}
	s.o = word_vec(r, 4);
	s.n = word_vec(r, 4 + VEC_WORDS);
	s.x = word_vec(r, 4 + (2 * VEC_WORDS));
	check_frame(r, s.n, s.x);
	if (r->n != 4 + FRAME_WORDS + surf_sizes[k]) {
		die("%s: bad face", r->where);
	}
	for (size_t i = 0; i < surf_sizes[k] && i < SIZES_MAX; i++) {
		sizes[i] = word_num(r, 4 + FRAME_WORDS + i);
	}
	s.r = sizes[0];
	s.r2 = sizes[1];
	shape_face(sh, s, sense, r->w[1]);
}

// Reads the uses of a loop from the words of s, which follow "l", one
// at a time, so that a loop takes any number of them.
static void read_loop(struct shape *sh, char *s, const char *where)
{
	size_t first = sh->nu;
	char *w[1];

	if (!sh->nf) {
		die("%s: a loop needs a face", where);
	}
	while ((w[0] = split_first(&s)) != NULL) {
		struct record one = { w, 1, where };
		char c = w[0][0];

		if (c != '+' && c != '-') {
			die("%s: a loop item is +E or -E", where);
		}
		w[0]++;
		shape_use(sh, word_index(sh->ne, &one, 0), c == '+');
	}
	if (sh->nu == first) {
		die("%s: a loop needs an edge", where);
	}
	shape_end_loop(sh, first);
}

int shape_record(struct shape *sh, char *line, const char *where)
{
	char *w[RECORD_MAX + 1];
	char *rest = line;
	size_t n = 0;
	struct record r = { w, 0, where };

	w[0] = split_first(&rest);
	if (!w[0]) {
		die("%s: an empty record", where);
	}
	if (strcmp(w[0], "l") == 0) {
		read_loop(sh, rest, where);
		return 0;
	}
	n = 1 + split_words(rest, w + 1, RECORD_MAX - 1);
	if (n > RECORD_MAX) {
		die("%s: a record of too many words", where);
	}
	r.n = n;
	if (strcmp(w[0], "end") == 0) {
		return 1;
	}
	if (strcmp(w[0], "v") == 0 && n == 1 + VEC_WORDS) {
		struct vec p = word_vec(&r, 1);

		PUSH(sh->v, sh->nv, sh->capv, p);
	} else if (strcmp(w[0], "e") == 0) {
		read_edge(sh, &r);
	} else if (strcmp(w[0], "f") == 0 && n >= 4) {
		read_face(sh, &r);
	} else {
		die("%s: unknown record %s", where, w[0]);
	}
	return 0;
}

void shape_check(const struct shape *sh, const char *where)
{
	int *fwd = xcalloc(sh->ne + 1, sizeof(*fwd));
	int *back = xcalloc(sh->ne + 1, sizeof(*back));

	for (size_t i = 0; i < sh->nu; i++) {
		if (sh->u[i].fwd) {
			fwd[sh->u[i].e]++;
		} else {
			back[sh->u[i].e]++;
		}
	}
	for (size_t i = 0; i < sh->ne; i++) {
		if (fwd[i] != 1 || back[i] != 1) {
			die("%s: edge %zu of %s has %d uses forward and %d "
			    "back",
			    where, i, sh->name, fwd[i], back[i]);
		}
	}
	free(fwd);
	free(back);
}

// The face of each use.
static size_t *use_faces(const struct shape *sh)
{
	size_t *uf = xmalloc((sh->nu + 1) * sizeof(*uf));

	for (size_t f = 0; f < sh->nf; f++) {
		for (size_t l = 0; l < sh->f[f].nloops; l++) {
			const struct sloop *lp = &sh->l[sh->f[f].loop0 + l];

			for (size_t k = 0; k < lp->n; k++) {
				uf[lp->first + k] = f;
			}
		}
	}
	return uf;
}

// The root of the set of face f.
static size_t root(size_t *up, size_t f)
{
	while (up[f] != f) {
		up[f] = up[up[f]];
		f = up[f];
	}
	return f;
}

size_t shape_shells(const struct shape *sh, size_t *shell)
{
	size_t *uf = use_faces(sh);
	size_t *first = xmalloc((sh->ne + 1) * sizeof(*first));
	size_t *up = xmalloc((sh->nf + 1) * sizeof(*up));
	size_t n = 0;

	for (size_t e = 0; e < sh->ne; e++) {
		first[e] = NONE;
	}
	for (size_t f = 0; f < sh->nf; f++) {
		up[f] = f;
	}
	for (size_t i = 0; i < sh->nu; i++) {
		size_t e = sh->u[i].e;

		if (first[e] == NONE) {
			first[e] = uf[i];
		} else {
			up[root(up, uf[i])] = root(up, first[e]);
		}
	}
	for (size_t f = 0; f < sh->nf; f++) {
		shell[f] = NONE;
	}
	for (size_t f = 0; f < sh->nf; f++) {
		size_t r = root(up, f);

		if (shell[r] == NONE) {
			shell[r] = n++;
		}
		shell[f] = shell[r];
	}
	free(uf);
	free(first);
	free(up);
	return n;
}
