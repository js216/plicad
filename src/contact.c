// SPDX-License-Identifier: GPL-3.0-only
// contact.c -- Whether two solids of a folded part meet, exactly
// Copyright (c) 2026 Jakob Kastelic

// The two covers meet in a convex polytope P.  If a corner or the middle
// of P lies in both materials, the solids meet.  If P lies wholly where
// one of them has no material (in a hole, outside the outline of its
// faces, inside or outside the shell of a bend, outside a hole of a
// cut), they do not meet there.  Otherwise P splits in two halves, each
// tried in turn, until it is smaller than CS_TOUCH, where the two
// materials come that close.

#include "contact.h"
#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>

#define ON 1e-9 // How far outside a half-space a corner may lie, in mm.

enum {
	POLY_MAX = 80, // Half-spaces of a polytope, with its splits.
	VERT_MAX = 512, // Corners of a polytope.
	QUAD = 4 // Corners of the cross section of a prism.
};

// A convex polytope: the points in all of its half-spaces.
struct poly {
	struct halfspace h[POLY_MAX];
	size_t n;
};

static void add_plane(struct csolid *s, struct vec n, double d)
{
	struct halfspace h = { n, d + CS_TOUCH };

	s->h[s->nh++] = h;
}

void csolid_prism(struct csolid *s, const struct vec *pt)
{
	struct vec ax = sub(pt[4], pt[0]);
	struct vec c = v3(0, 0, 0);
	struct vec u = unit(ax);

	s->nh = 0;
	for (int i = 0; i < QUAD; i++) {
		c = add(c, mul(pt[i], 1 / (double)QUAD));
	}
	add_plane(s, u, dot(u, pt[4]));
	add_plane(s, mul(u, -1), -dot(u, pt[0]));
	for (int i = 0; i < QUAD; i++) {
		struct vec e = sub(pt[(i + 1) % QUAD], pt[i]);
		struct vec n = cross(ax, e);

		if (!(len(n) > ON)) {
			continue;
		}
		n = unit(n);
		if (dot(n, sub(c, pt[i])) > 0) {
			n = mul(n, -1);
		}
		add_plane(s, n, dot(n, pt[i]));
	}
}

void csolid_box(struct csolid *s, struct box b)
{
	s->nh = 0;
	for (int a = 0; a < 3; a++) {
		struct vec e = axis_vec(a, 1);

		add_plane(s, e, comp(b.hi, a));
		add_plane(s, mul(e, -1), -comp(b.lo, a));
	}
}

// The point where the planes of h0, h1 and h2 meet; 0 if they do not
// meet in one point.
static int corner(const struct halfspace *h0, const struct halfspace *h1,
		  const struct halfspace *h2, struct vec *out)
{
	struct vec c12 = cross(h1->n, h2->n);
	double det = dot(h0->n, c12);
	struct vec p;

	if (fabs(det) < ON) {
		return 0;
	}
	p = add(mul(c12, h0->d), add(mul(cross(h2->n, h0->n), h1->d),
				     mul(cross(h0->n, h1->n), h2->d)));
	*out = mul(p, 1 / det);
	return 1;
}

static int inside_poly(const struct poly *P, struct vec p)
{
	for (size_t i = 0; i < P->n; i++) {
		if (dot(P->h[i].n, p) > P->h[i].d + ON) {
			return 0;
		}
	}
	return 1;
}

// Adds p to the n corners v unless it is one of them.
static void add_corner(struct vec *v, size_t *n, struct vec p)
{
	for (size_t i = 0; i < *n; i++) {
		if (len(sub(v[i], p)) < ON) {
			return;
		}
	}
	if (*n < VERT_MAX) {
		v[(*n)++] = p;
	}
}

// The corners of P, into v; returns their count.
static size_t corners(const struct poly *P, struct vec *v)
{
	size_t n = 0;

	for (size_t i = 0; i < P->n; i++) {
		for (size_t j = i + 1; j < P->n; j++) {
			for (size_t k = j + 1; k < P->n; k++) {
				struct vec p;

				if (corner(&P->h[i], &P->h[j], &P->h[k], &p) &&
				    inside_poly(P, p)) {
					add_corner(v, &n, p);
				}
			}
		}
	}
	return n;
}

// 2D geometry, in the flat of a face or across an axis.

static double cross2(struct vec2 o, struct vec2 a, struct vec2 b)
{
	return ((a.x - o.x) * (b.y - o.y)) - ((a.y - o.y) * (b.x - o.x));
}

static int cmp_vec2(const void *lhs, const void *rhs)
{
	const struct vec2 *a = lhs;
	const struct vec2 *b = rhs;

	if (a->x < b->x || a->x > b->x) {
		return a->x < b->x ? -1 : 1;
	}
	if (a->y < b->y || a->y > b->y) {
		return a->y < b->y ? -1 : 1;
	}
	return 0;
}

// The convex hull of the n points q, counterclockwise, in place; returns
// its count of corners.
static size_t hull(struct vec2 *q, size_t n)
{
	struct vec2 *h = xmalloc((2 * n + 1) * sizeof(*h));
	size_t k = 0;

	qsort(q, n, sizeof(*q), cmp_vec2);
	for (size_t i = 0; i < n; i++) {
		while (k >= 2 && cross2(h[k - 2], h[k - 1], q[i]) <= 0) {
			k--;
		}
		h[k++] = q[i];
	}
	for (size_t i = n - 1, t = k + 1; i-- > 0;) {
		while (k >= t && cross2(h[k - 2], h[k - 1], q[i]) <= 0) {
			k--;
		}
		h[k++] = q[i];
	}
	k = k > 1 ? k - 1 : k;
	for (size_t i = 0; i < k; i++) {
		q[i] = h[i];
	}
	free(h);
	return k;
}

static double seg_dist2(struct vec2 p, struct vec2 a, struct vec2 b)
{
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	double l2 = (dx * dx) + (dy * dy);
	double t = l2 > 0 ? (((p.x - a.x) * dx) + ((p.y - a.y) * dy)) / l2 : 0;

	t = fmin(1, fmax(0, t));
	return hypot(p.x - a.x - (t * dx), p.y - a.y - (t * dy));
}

// The distance from p to the convex polygon q of n corners; 0 inside.
static double poly_dist(struct vec2 p, const struct vec2 *q, size_t n)
{
	double d = HUGE_VAL;
	int in = n >= 3;

	for (size_t i = 0; i < n; i++) {
		struct vec2 a = q[i];
		struct vec2 b = q[(i + 1) % n];

		in = in && cross2(a, b, p) >= 0;
		d = fmin(d, seg_dist2(p, a, b));
	}
	return in ? 0 : d;
}

// Whether the segments ab and cd meet.
static int segs_meet(struct vec2 a, struct vec2 b, struct vec2 c, struct vec2 d)
{
	double d1 = cross2(c, d, a);
	double d2 = cross2(c, d, b);
	double d3 = cross2(a, b, c);
	double d4 = cross2(a, b, d);

	if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
	    ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
		return 1;
	}
	return seg_dist2(a, c, d) < ON || seg_dist2(b, c, d) < ON ||
	       seg_dist2(c, a, b) < ON || seg_dist2(d, a, b) < ON;
}

// Whether the angle of the point p about the centre of arc e of part p
// lies on the arc.
static int on_arc(const struct part *pt, const struct edge *e, struct vec2 p)
{
	struct arc_ang a = arc_angles(pt, e);
	double t = atan2(p.y - e->cy, p.x - e->cx) - a.a0;

	t = fmod(fmod(t, 2 * PI) + (2 * PI), 2 * PI);
	return t <= a.da + ON;
}

// Whether the segment ab meets arc e of part p.
static int seg_meets_arc(const struct part *p, const struct edge *e,
			 struct vec2 a, struct vec2 b)
{
	struct arc_ang ar = arc_angles(p, e);
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	double fx = a.x - e->cx;
	double fy = a.y - e->cy;
	double qa = (dx * dx) + (dy * dy);
	double qb = 2 * ((fx * dx) + (fy * dy));
	double qc = (fx * fx) + (fy * fy) - (ar.r * ar.r);
	double disc = (qb * qb) - (4 * qa * qc);

	if (!(qa > 0) || disc < 0) {
		return 0;
	}
	for (int k = -1; k <= 1; k += 2) {
		double t = (-qb + (k * sqrt(disc))) / (2 * qa);
		struct vec2 x = { a.x + (t * dx), a.y + (t * dy) };

		if (t >= -ON && t <= 1 + ON && on_arc(p, e, x)) {
			return 1;
		}
	}
	return 0;
}

// The distance from q to edge e of part p.
static double edge_dist(const struct part *p, const struct edge *e,
			struct vec2 q)
{
	struct vec2 a = p->verts[e->v0];
	struct vec2 b = p->verts[e->v1];
	struct arc_ang ar;

	if (!e->arc) {
		return seg_dist2(q, a, b);
	}
	ar = arc_angles(p, e);
	if (on_arc(p, e, q)) {
		return fabs(hypot(q.x - e->cx, q.y - e->cy) - ar.r);
	}
	return fmin(hypot(q.x - a.x, q.y - a.y), hypot(q.x - b.x, q.y - b.y));
}

// Whether edge e of part p bounds face f: on one side of it only.
static int bounds(const struct edge *e, size_t f)
{
	return (e->lf == f) != (e->rf == f);
}

// Whether the flat point q lies in face f of part p or within CS_TOUCH
// of its outline, holes aside.
static int near_outline(const struct part *p, size_t f, struct vec2 q)
{
	if (point_in_outline(p, f, q)) {
		return 1;
	}
	for (size_t k = p->fe_start[f]; k < p->fe_start[f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];

		if (bounds(e, f) && edge_dist(p, e, q) <= CS_TOUCH) {
			return 1;
		}
	}
	return 0;
}

// Whether the flat point q, at depth q.z, lies in the material of face f
// of part p, or within CS_TOUCH of it.
static int near_face(const struct part *p, size_t f, struct vec q)
{
	struct vec2 at = { q.x, q.y };

	if (!near_outline(p, f, at)) {
		return 0;
	}
	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];

		if (h->face == f && hypot(q.x - h->cx, q.y - h->cy) <
					    hole_radius(p, h, q.z) - CS_TOUCH) {
			return 0;
		}
	}
	return 1;
}

static int in_range(double v, double lo, double hi)
{
	return v >= lo - CS_TOUCH && v <= hi + CS_TOUCH;
}

// Whether the point w of the world lies in the flat cell of slab or
// bend s.
static int has_flat(const struct csolid *s, struct vec w, struct vec *q)
{
	return finv(s->p, s->fi, w, q) && in_range(q->x, s->x0, s->x1) &&
	       in_range(q->y, s->y0, s->y1) && in_range(q->z, -s->p->t, 0);
}

// Whether the world point w lies in the material of s, or within
// CS_TOUCH of it.
static int has(const struct csolid *s, struct vec w)
{
	struct vec q;
	int au = s->a == 0 ? 1 : 0;
	int av = s->a == 2 ? 1 : 2;

	if (s->kind == CS_HOLE || s->kind == CS_RECT) {
		double u = comp(w, au);
		double v = comp(w, av);

		if (!in_range(comp(w, s->a), s->z0, s->z1)) {
			return 0;
		}
		if (s->kind == CS_HOLE) {
			return hypot(u - s->cu, v - s->cv) <= s->r + CS_TOUCH;
		}
		return in_range(u, s->x0, s->x1) && in_range(v, s->y0, s->y1);
	}
	if (!has_flat(s, w, &q)) {
		return 0;
	}
	if (s->kind == CS_BEND) {
		return 1;
	}
	for (size_t f = 0; f < s->p->nfaces; f++) {
		if (s->p->faces[f] == s->mi && near_face(s->p, f, q)) {
			return 1;
		}
	}
	return 0;
}

// Whether the polygon q of n corners, counterclockwise, misses the
// outline of face f of part p: no corner inside the other, and no edges
// that cross.
static int misses_outline(const struct part *p, size_t f, const struct vec2 *q,
			  size_t n)
{
	for (size_t i = 0; i < n; i++) {
		if (point_in_outline(p, f, q[i])) {
			return 0;
		}
	}
	for (size_t k = p->fe_start[f]; k < p->fe_start[f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];
		struct vec2 a = p->verts[e->v0];

		if (!bounds(e, f)) {
			continue;
		}
		if (poly_dist(a, q, n) < ON) {
			return 0;
		}
		for (size_t i = 0; i < n; i++) {
			struct vec2 c = q[i];
			struct vec2 d = q[(i + 1) % n];

			if (e->arc ? seg_meets_arc(p, e, c, d) :
				     segs_meet(a, p->verts[e->v1], c, d)) {
				return 0;
			}
		}
	}
	return 1;
}

// Whether the n flat points q, at their depths, all lie in hole h of
// part p: in its cylinder, or all in the cone of its countersink.
static int in_hole(const struct part *p, const struct hole *h,
		   const struct vec *q, size_t n)
{
	int cyl = 1;
	int cone = h->side != 0;

	for (size_t i = 0; i < n; i++) {
		double r = hypot(q[i].x - h->cx, q[i].y - h->cy);
		double d = h->side > 0 ? -q[i].z : q[i].z + p->t;
		double rc = h->rc - ((h->rc - h->r) * d / h->h);

		cyl = cyl && r < h->r;
		cone = cone && r < rc;
	}
	return cyl || cone;
}

// Whether the polytope of the n corners w lies where slab s has no
// material: in one hole, or off the outline of every face of its map.
static int slab_misses(const struct csolid *s, const struct vec *w, size_t n)
{
	const struct part *p = s->p;
	struct vec *q = xmalloc((n + 1) * sizeof(*q));
	struct vec2 *q2 = xmalloc((n + 1) * sizeof(*q2));
	size_t m = 0;
	int off = 1;

	for (size_t i = 0; i < n; i++) {
		(void)finv(p, s->fi, w[i], &q[i]);
		q2[i].x = q[i].x;
		q2[i].y = q[i].y;
	}
	m = hull(q2, n);
	for (size_t f = 0; f < p->nfaces; f++) {
		if (p->faces[f] == s->mi) {
			off = off && misses_outline(p, f, q2, m);
		}
	}
	for (size_t i = 0; i < p->nholes && !off; i++) {
		off = p->faces[p->holes[i].face] == s->mi &&
		      in_hole(p, &p->holes[i], q, n);
	}
	free(q);
	free(q2);
	return off;
}

// The distance from the line ax to the polytope of the n corners w.
static double line_dist(struct line3 ax, const struct vec *w, size_t n)
{
	struct vec o = ax.o;
	struct vec d = ax.d;
	struct vec e1 = cross(d, fabs(d.x) < MID ? v3(1, 0, 0) : v3(0, 1, 0));
	struct vec e2;
	struct vec2 *q = xmalloc((n + 1) * sizeof(*q));
	struct vec2 z = { 0, 0 };
	double r = 0;
	size_t m = 0;

	e1 = unit(e1);
	e2 = cross(unit(d), e1);
	for (size_t i = 0; i < n; i++) {
		q[i].x = dot(sub(w[i], o), e1);
		q[i].y = dot(sub(w[i], o), e2);
	}
	m = hull(q, n);
	r = poly_dist(z, q, m);
	free(q);
	return r;
}

// Whether the polytope of the n corners w lies inside the inner surface
// of bend s, or outside its outer one.
static int bend_misses(const struct csolid *s, const struct vec *w, size_t n)
{
	struct vec d = unit(s->axis.d);
	int inner = 1;

	for (size_t i = 0; i < n; i++) {
		struct vec r = sub(w[i], s->axis.o);

		inner = inner && len(sub(r, mul(d, dot(r, d)))) < s->r0;
	}
	return inner || line_dist(s->axis, w, n) > s->r1;
}

// Whether the polytope of the n corners w lies where s has no material.
static int misses(const struct csolid *s, const struct vec *w, size_t n)
{
	if (s->kind == CS_SLAB) {
		return slab_misses(s, w, n);
	}
	if (s->kind == CS_BEND) {
		return bend_misses(s, w, n);
	}
	if (s->kind == CS_HOLE) {
		struct line3 ax = { axis_vec(s->a == 0 ? 1 : 0, s->cu),
				    axis_vec(s->a, 1) };

		ax.o = add(ax.o, axis_vec(s->a == 2 ? 1 : 2, s->cv));
		return line_dist(ax, w, n) > s->r;
	}
	return 0;
}

// The middle of the n corners w, and the axis and middle of the longest
// side of their box.
struct split {
	struct vec mid;
	int axis;
	double at, size;
};

static struct split split_of(const struct vec *w, size_t n)
{
	struct box b = box_empty();
	struct split s = { v3(0, 0, 0), 0, 0, 0 };

	for (size_t i = 0; i < n; i++) {
		box_add(&b, w[i]);
		s.mid = add(s.mid, mul(w[i], 1 / (double)n));
	}
	for (int a = 0; a < 3; a++) {
		double l = comp(b.hi, a) - comp(b.lo, a);

		if (l > s.size) {
			s.size = l;
			s.axis = a;
			s.at = (comp(b.hi, a) + comp(b.lo, a)) / 2;
		}
	}
	return s;
}

// What part of polytope p tells about a and b: 1 if they meet in it, 0
// if they do not, -1 if it must split in the halves lo and hi.
static int judge(const struct csolid *a, const struct csolid *b,
		 const struct poly *p, struct halfspace *lo,
		 struct halfspace *hi)
{
	struct vec *w = xmalloc(VERT_MAX * sizeof(*w));
	size_t n = corners(p, w);
	struct split s;
	int r = 0;

	if (!n) {
		free(w);
		return 0;
	}
	s = split_of(w, n);
	for (size_t i = 0; i <= n && !r; i++) {
		struct vec q = i < n ? w[i] : s.mid;

		r = has(a, q) && has(b, q);
	}
	if (!r && !misses(a, w, n) && !misses(b, w, n)) {
		r = s.size < CS_TOUCH || p->n + 1 >= POLY_MAX ? 1 : -1;
	}
	lo->n = axis_vec(s.axis, 1);
	lo->d = s.at;
	hi->n = axis_vec(s.axis, -1);
	hi->d = -s.at;
	free(w);
	return r;
}

int csolids_meet(const struct csolid *a, const struct csolid *b)
{
	struct poly *stack = xmalloc((POLY_MAX + 2) * sizeof(*stack));
	size_t sp = 1;
	int r = 0;

	stack[0].n = 0;
	for (size_t i = 0; i < a->nh; i++) {
		stack[0].h[stack[0].n++] = a->h[i];
	}
	for (size_t i = 0; i < b->nh; i++) {
		stack[0].h[stack[0].n++] = b->h[i];
	}
	while (sp && !r) {
		const struct poly *p = &stack[--sp];
		struct halfspace lo;
		struct halfspace hi;
		int j = judge(a, b, p, &lo, &hi);

		if (j < 0) {
			stack[sp + 1] = *p;
			stack[sp].h[stack[sp].n++] = lo;
			stack[sp + 1].h[stack[sp + 1].n++] = hi;
			sp += 2;
		}
		r = j > 0;
	}
	free(stack);
	return r;
}
