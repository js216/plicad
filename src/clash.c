// SPDX-License-Identifier: GPL-3.0-only
// clash.c -- Whether two exact solids overlap, by samples of their faces
// Copyright (c) 2026 Jakob Kastelic

#include "clash.h"
#include "geom.h"
#include "isect.h"
#include "mesh.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CLASH_ANGLE 15.0 // The angle of the triangles sampled, in degrees.
// How far a piece of a triangle may lie from its exact surface, with the
// margin of a sample: a face of the other solid farther than this from
// the box of a piece does not cut the piece of the surface under it.
#define REACH ((2 * CLASH_CHORD) + CLASH_MARGIN)
#define WALL_MIN 1e-5 // A hit nearer than this along a normal is no wall.
#define WALL_TURN 0.5 // The cosine of the most turn of the faces of a wall.
#define ARC_PIECE (PI / 4) // The widest piece of a curved edge, in radians.
#define STEP_MIN (2 * CLASH_CHORD) // The least room between samples.
#define TREE_SIDE 4.0 // The longest side of a triangle in the tree, in mm.
#define STRIP_STEP 0.8 // The room between samples of a narrow piece, in steps.
#define CROSS_CLEAR 1e-5 // How far a crossing of a face stays off its edges.
#define CROSS_SINE 1e-6 // The least sine of a crossing of a face.
#define CROSS_RUN (4 * CLASH_MARGIN) // How far an edge runs past a crossing.
#define CROSS_GROW 4.0 // The next run past a crossing, in runs before.
#define HALF 0.5
#define MEMO_REACH 8.0 // How far, in steps, a face memo looks for edges.
#define QUICK_CLEAR 4.0 // How near, in chords, an edge spoils a quick test.
#define SMALL 4.0 // A near piece under this many steps has near halves.
#define SIDE_GROW 4.0 // The next reach for the nearest face, in reaches.
#define FLAT 1e-12 // A ray this near square to an axis runs across it.

enum {
	LEAF = 4, // The most faces of a leaf of the tree.
	TODO_ROOM = 64, // The first room for the nodes still to split.
	SPLIT_MAX = 40, // The most halvings of a piece.
	STACK_MAX = (2 * SPLIT_MAX) + 2, // Room for a walk down a tree.
	ARC_PIECES = 8, // The most pieces of a full turn of an edge.
	STRIP_MAX = 1000000, // The most samples of a narrow piece.
	SIDE_GROWTHS = 4, // The reaches for the nearest face of a sample.
	CROSS_RUNS = 4 // The runs past a crossing.
};

static double coord(struct vec v, int a)
{
	if (a == 0) {
		return v.x;
	}
	return a == 1 ? v.y : v.z;
}

// The faces idx[lo .. hi) of a node of the tree.
struct range {
	size_t lo, hi;
};

static double box_key(const struct box *fb, size_t f, int a)
{
	return coord(fb[f].lo, a) + coord(fb[f].hi, a);
}

// Puts the face of the k-th least key along axis a in idx[k], those of
// lesser keys before it and greater after, in idx[lo .. hi).
static void select_k(size_t *idx, struct range r, size_t k,
		     const struct box *fb, int a)
{
	size_t lo = r.lo;
	size_t hi = r.hi;

	while (hi - lo > 1) {
		double pv = box_key(fb, idx[k], a);
		size_t i = lo;
		size_t j = hi - 1;

		while (i <= j) {
			while (box_key(fb, idx[i], a) < pv) {
				i++;
			}
			while (box_key(fb, idx[j], a) > pv) {
				j--;
			}
			if (i <= j) {
				size_t x = idx[i];

				idx[i] = idx[j];
				idx[j] = x;
				i++;
				if (j == 0) {
					break;
				}
				j--;
			}
		}
		if (k <= j && j + 1 < hi) {
			hi = j + 1;
		} else if (k >= i && i > lo) {
			lo = i;
		} else {
			return;
		}
	}
}

// The box of the items idx[lo .. hi) of tree t.
static struct box range_box(const struct clash_tree *t, struct range r)
{
	struct box x = box_empty();

	for (size_t i = r.lo; i < r.hi; i++) {
		box_add(&x, t->items[t->idx[i]].lo);
		box_add(&x, t->items[t->idx[i]].hi);
	}
	return x;
}

// The longest side of box x: 0, 1 or 2 for x, y or z.
static int longest(struct box x)
{
	struct vec d = sub(x.hi, x.lo);

	if (d.x >= d.y && d.x >= d.z) {
		return 0;
	}
	return d.y >= d.z ? 1 : 2;
}

static void push_node(struct clash_tree *t, struct range r)
{
	struct clash_node nd = { range_box(t, r), r.lo, r.hi, 0 };

	PUSH(t->node, t->nnode, t->capnode, nd);
}

// Builds the tree of the n boxes items, each node split at the middle
// of its longest side until it holds LEAF items at most.
static void tree_build(struct clash_tree *t, const struct box *items, size_t n)
{
	size_t captodo = TODO_ROOM;
	size_t *todo = xmalloc(captodo * sizeof(*todo));
	size_t ntodo = 0;

	memset(t, 0, sizeof(*t));
	t->items = items;
	t->idx = xmalloc((n + 1) * sizeof(*t->idx));
	for (size_t i = 0; i < n; i++) {
		t->idx[i] = i;
	}
	{
		struct range all = { 0, n };

		push_node(t, all);
	}
	todo[ntodo++] = 0;
	while (ntodo) {
		size_t k = todo[--ntodo];
		struct clash_node nd = t->node[k];
		size_t mid = (nd.lo + nd.hi) / 2;
		struct range r = { nd.lo, nd.hi };
		struct range lo = { nd.lo, mid };
		struct range hi = { mid, nd.hi };

		if (nd.hi - nd.lo <= LEAF) {
			continue;
		}
		select_k(t->idx, r, mid, items, longest(nd.b));
		t->node[k].left = t->nnode;
		push_node(t, lo);
		push_node(t, hi);
		PUSH(todo, ntodo, captodo, t->nnode - 2);
		PUSH(todo, ntodo, captodo, t->nnode - 1);
	}
	free(todo);
}

static void tree_free(struct clash_tree *t)
{
	free(t->node);
	free(t->idx);
	memset(t, 0, sizeof(*t));
}

// A query of a tree: a box, or a ray up to tmax.
struct query {
	int ray;
	struct box x;
	struct line3 l;
	double tmax;
};

// Whether the ray of q meets box x.
static int ray_meets(const struct query *q, struct box x)
{
	double t0 = 0;
	double t1 = q->tmax;

	for (int a = 0; a < 3; a++) {
		double o = coord(q->l.o, a);
		double d = coord(q->l.d, a);
		double lo = coord(x.lo, a);
		double hi = coord(x.hi, a);

		if (fabs(d) < FLAT) {
			if (o < lo || o > hi) {
				return 0;
			}
			continue;
		}
		{
			double ta = (lo - o) / d;
			double tb = (hi - o) / d;

			t0 = fmax(t0, fmin(ta, tb));
			t1 = fmin(t1, fmax(ta, tb));
		}
		if (t0 > t1) {
			return 0;
		}
	}
	return 1;
}

static int query_meets(const struct query *q, struct box x)
{
	return q->ray ? ray_meets(q, x) : box_meet(q->x, x);
}

// Adds to out the items of the leaf nd of t whose boxes q meets, as
// tree_find does; returns their count, or 1 at the first one if out is
// NULL.
static size_t leaf_find(const struct clash_tree *t, const struct clash_node *nd,
			const struct query *q, const size_t *to,
			struct face_list *out)
{
	size_t n = 0;

	for (size_t i = nd->lo; i < nd->hi; i++) {
		size_t f = t->idx[i];

		if (!query_meets(q, t->items[f])) {
			continue;
		}
		if (!out) {
			return 1;
		}
		f = to ? to[f] : f;
		if (!out->n || out->f[out->n - 1] != f) {
			PUSH(out->f, out->n, out->cap, f);
		}
		n++;
	}
	return n;
}

// Adds to out the items of t whose boxes q meets, or with out NULL
// returns 1 at the first one; returns their count.  With to, it adds
// to[item] in place of the item, once each in a row.
static size_t tree_find(const struct clash_tree *t, const struct query *q,
			const size_t *to, struct face_list *out)
{
	size_t stack[STACK_MAX];
	size_t top = 0;
	size_t n = 0;

	if (!t->nnode) {
		return 0;
	}
	stack[top++] = 0;
	while (top) {
		const struct clash_node *nd = &t->node[stack[--top]];

		if (!query_meets(q, nd->b)) {
			continue;
		}
		if (nd->left && top + 2 <= STACK_MAX) {
			stack[top++] = nd->left;
			stack[top++] = nd->left + 1;
			continue;
		}
		n += leaf_find(t, nd, q, to, out);
		if (!out && n) {
			return 1;
		}
	}
	return n;
}

// The faces of the body ctx whose boxes the ray meets (isect.h).
static void ray_faces(const void *ctx, struct line3 ray, double tmax,
		      struct face_list *out)
{
	const struct clash_body *b = ctx;
	struct query q = { 1, box_empty(), ray, tmax };

	(void)tree_find(&b->ftree, &q, NULL, out);
}

// The least distance through the solid from point c of face f, along the
// normal into the solid, to a face that it meets from inside, its normal
// turned against that at c by more than WALL_TURN: the thickness of a
// wall between two faces that face away from each other.  HUGE_VAL if
// there is none.
static double wall_at(const struct clash_body *b, size_t f, struct vec c)
{
	const struct sface *fc = &b->sh->f[f];
	struct vec q = surf_at(&fc->s, surf_param(&fc->s, c));
	struct vec in = mul(surf_normal(&fc->s, q), -(double)fc->sense);
	struct query qr = { 1, box_empty(), { q, in }, 0 };
	struct face_list fs = { NULL, 0, 0 };
	double best = HUGE_VAL;
	struct ray_seg rs = { { q, in }, 0 };

	qr.tmax = rs.tmax = 2 * len(sub(b->box.hi, b->box.lo));
	(void)tree_find(&b->ftree, &qr, NULL, &fs);
	for (size_t i = 0; i < fs.n; i++) {
		double t[ROOTS_MAX];
		int unsafe = 0;
		size_t n = ray_face_hits(&b->cache, fs.f[i], rs, t, &unsafe);

		for (size_t k = 0; k < n; k++) {
			const struct sface *g = &b->sh->f[fs.f[i]];
			struct vec h = add(q, mul(in, t[k]));
			struct vec gn =
				mul(surf_normal(&g->s, h), (double)g->sense);

			if (t[k] > WALL_MIN && dot(gn, in) > WALL_TURN) {
				best = fmin(best, t[k]);
			}
		}
	}
	free(fs.f);
	return best;
}

// The thinnest wall of the solid, along the normals at the middle of the
// triangles of its faces.
static double thinnest(const struct clash_body *b)
{
	double thin = HUGE_VAL;

	for (size_t i = 0; i < b->m.nt; i++) {
		const struct mesh_tri *t = &b->m.t[i];
		struct vec c = mul(add(add(b->m.v[t->v[0]], b->m.v[t->v[1]]),
				       b->m.v[t->v[2]]),
				   1.0 / 3);

		thin = fmin(thin, wall_at(b, t->face, c));
	}
	return thin;
}

// The two faces of each edge.
static void edge_faces(struct clash_body *b)
{
	const struct shape *sh = b->sh;

	b->ef = xmalloc((sh->ne + 1) * sizeof(*b->ef));
	for (size_t e = 0; e < sh->ne; e++) {
		b->ef[e][0] = b->ef[e][1] = NONE;
	}
	for (size_t f = 0; f < sh->nf; f++) {
		struct urange r = face_uses(sh, f);

		for (size_t k = r.first; k < r.end; k++) {
			size_t e = sh->u[k].e;

			b->ef[e][b->ef[e][0] == NONE ? 0 : 1] = f;
		}
	}
}

// A narrow piece of a triangle: its longest side from a along ab, the
// foot of the far corner on that side as a share of it, and the height
// up from the foot to that corner.
struct strip {
	struct vec a, ab, up;
	double tf;
};

// A piece of a triangle to sample or to box, and the halvings that made it.
struct piece_tri {
	struct vec p[3];
	int depth, near;
};

// The longest side of piece x, as the index of its first corner; its
// length goes to l.
static size_t longest_side(const struct piece_tri *x, double *l)
{
	size_t k = 0;
	double lk = -1;

	for (size_t j = 0; j < 3; j++) {
		double lj = len(sub(x->p[(j + 1) % 3], x->p[j]));

		if (lj > lk) {
			lk = lj;
			k = j;
		}
	}
	*l = lk;
	return k;
}

// The two halves of piece x, cut through the middle of side k, into
// two[0] and two[1].
static void halve(const struct piece_tri *x, size_t k, struct piece_tri *two)
{
	struct vec m = mul(add(x->p[k], x->p[(k + 1) % 3]), 1.0 / 2);

	two[0].p[0] = x->p[k];
	two[0].p[1] = m;
	two[0].p[2] = x->p[(k + 2) % 3];
	two[1].p[0] = m;
	two[1].p[1] = x->p[(k + 1) % 3];
	two[1].p[2] = x->p[(k + 2) % 3];
	two[0].depth = two[1].depth = x->depth + 1;
	two[0].near = two[1].near = 1;
}

// The room of the arrays of the pieces of a body.
struct room {
	size_t cap, capf, nf;
};

// Adds the box of piece x of face f to b, with its corners and its face.
static void add_box(struct clash_body *b, const struct piece_tri *x, size_t f,
		    struct room *rm)
{
	struct box bx = box_empty();
	size_t capp = rm->cap;

	for (int j = 0; j < 3; j++) {
		box_add(&bx, x->p[j]);
	}
	box_pad(&bx, CLASH_CHORD);
	PUSH(b->tbox, b->ntbox, rm->cap, bx);
	if (capp != rm->cap) {
		b->tpt = xrealloc(b->tpt, rm->cap * sizeof(*b->tpt));
	}
	memcpy(b->tpt[b->ntbox - 1], x->p, sizeof(x->p));
	PUSH(b->tface, rm->nf, rm->capf, f);
}

// The boxes of the triangles of b, each cut into pieces no side of
// which is longer than TREE_SIDE, grown by the chord, and their faces:
// small boxes, which the tree tells apart.
static void tri_boxes(struct clash_body *b)
{
	struct room rm = { 0, 0, 0 };

	for (size_t i = 0; i < b->m.nt; i++) {
		struct piece_tri stack[SPLIT_MAX + 2];
		size_t top = 1;

		for (int k = 0; k < 3; k++) {
			stack[0].p[k] = b->m.v[b->m.t[i].v[k]];
		}
		stack[0].depth = 0;
		while (top) {
			struct piece_tri x = stack[--top];
			double l = 0;
			size_t k = longest_side(&x, &l);

			if (l > TREE_SIDE && x.depth < SPLIT_MAX) {
				halve(&x, k, &stack[top]);
				top += 2;
			} else {
				add_box(b, &x, b->m.t[i].face, &rm);
			}
		}
	}
}

// The distance from p to the triangle t.
static double tri_dist(const struct vec *t, struct vec p)
{
	struct vec ab = sub(t[1], t[0]);
	struct vec ac = sub(t[2], t[0]);
	struct vec n = cross(ab, ac);
	double nn = dot(n, n);
	struct vec ap = sub(p, t[0]);
	double u = 0;
	double v = 0;
	double best = HUGE_VAL;

	if (!(nn > 0)) {
		return HUGE_VAL;
	}
	// The foot of p in the plane, by its weights on ab and ac.
	u = dot(cross(ap, ac), n) / nn;
	v = dot(cross(ab, ap), n) / nn;
	if (u >= 0 && v >= 0 && u + v <= 1) {
		return fabs(dot(ap, n)) / sqrt(nn);
	}
	for (int k = 0; k < 3; k++) {
		struct vec a = t[k];
		struct vec d = sub(t[(k + 1) % 3], a);
		double s = dot(sub(p, a), d) / fmax(dot(d, d), FLAT);

		s = fmin(1, fmax(0, s));
		best = fmin(best, len(sub(p, add(a, mul(d, s)))));
	}
	return best;
}

// The quick test of a face of the body ctx (isect.h): p, a point of the
// surface of face f, lies in the face if it lies within twice the chord
// of a triangle of its mesh, and out of it if not, provided no edge comes
// within QUICK_CLEAR chords; else it cannot tell.
static int quick_has(const void *ctx, size_t f, struct vec p)
{
	const struct clash_body *b = ctx;
	struct query q = { 0, { p, p }, { p, p }, 0 };
	size_t stack[STACK_MAX];
	size_t top = 0;

	box_pad(&q.x, QUICK_CLEAR * CLASH_CHORD);
	if (tree_find(&b->etree, &q, NULL, NULL)) {
		return -1;
	}
	q.x.lo = q.x.hi = p;
	stack[top++] = 0;
	while (top && b->ttree.nnode) {
		const struct clash_node *nd = &b->ttree.node[stack[--top]];

		if (!box_has(nd->b, p)) {
			continue;
		}
		if (nd->left && top + 2 <= sizeof(stack) / sizeof(stack[0])) {
			stack[top++] = nd->left;
			stack[top++] = nd->left + 1;
			continue;
		}
		for (size_t i = nd->lo; i < nd->hi; i++) {
			size_t k = b->ttree.idx[i];

			if (b->tface[k] == f && box_has(b->tbox[k], p) &&
			    tri_dist(b->tpt[k], p) <= 2 * CLASH_CHORD) {
				return 1;
			}
		}
	}
	return 0;
}

void clash_init(struct clash_body *b, const struct shape *sh, double thin)
{
	struct mesh_opts o = { CLASH_CHORD, CLASH_ANGLE };

	memset(b, 0, sizeof(*b));
	b->sh = sh;
	b->box = box_empty();
	for (size_t i = 0; i < sh->nv; i++) {
		box_add(&b->box, sh->v[i]);
	}
	for (size_t i = 0; i < sh->ne; i++) {
		struct box e = edge_box3(&sh->e[i]);

		box_add(&b->box, e.lo);
		box_add(&b->box, e.hi);
	}
	shape_cache_init(&b->cache, sh);
	b->fbox = b->cache.fbox;
	tree_build(&b->ftree, b->fbox, sh->nf);
	edge_faces(b);
	mesh_init(&b->m);
	mesh_shape(sh, o, &b->m);
	tri_boxes(b);
	tree_build(&b->ttree, b->tbox, b->ntbox);
	b->cache.quick = quick_has;
	b->cache.qctx = b;
	tree_build(&b->etree, b->cache.ebox, sh->ne);
	b->thin = thin > 0 ? thin : thinnest(b);
}

void clash_free(struct clash_body *b)
{
	mesh_free(&b->m);
	shape_cache_free(&b->cache);
	tree_free(&b->ftree);
	tree_free(&b->ttree);
	tree_free(&b->etree);
	free(b->tbox);
	free(b->tface);
	free(b->tpt);
	free(b->ef);
	memset(b, 0, sizeof(*b));
}

// The work of one side of a check: the sampled solid, the other one,
// the motion from the frame of the first into that of the other, the
// room between samples, the count and its limit, and room for a list of
// faces.
struct side {
	const struct clash_body *a, *b;
	struct rigid ab;
	double h;
	size_t n, max;
	struct face_list near, edges;
	struct face_memo *memo; // One for each face of b.
};

// What a face of the other solid is known to hold: whether the point at
// lies in it, and a ball about at, of radius clear, that no edge of the
// face reaches; clear is 0 before the first test.
struct face_memo {
	struct vec at;
	int in;
	double clear;
};

// The distance from p to box x, 0 inside it.
static double box_dist(struct box x, struct vec p)
{
	double dx = fmax(0, fmax(x.lo.x - p.x, p.x - x.hi.x));
	double dy = fmax(0, fmax(x.lo.y - p.y, p.y - x.hi.y));
	double dz = fmax(0, fmax(x.lo.z - p.z, p.z - x.hi.z));

	return sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

// Whether q, a point of the surface of face f of w->b, lies in the face,
// as face_has says: the same as for a point tested before if q lies in
// its ball clear of the edges, the edges of w->b near it, or MEMO_REACH
// steps.
static int memo_has(struct side *w, size_t f, struct vec q)
{
	struct face_memo *m = &w->memo[f];
	double clear = HUGE_VAL;
	struct query qe = { 0, { q, q }, { q, q }, 0 };

	if (m->clear > 0 && len(sub(q, m->at)) < m->clear) {
		return m->in;
	}
	box_pad(&qe.x, MEMO_REACH * w->h);
	w->edges.n = 0;
	(void)tree_find(&w->b->etree, &qe, NULL, &w->edges);
	clear = MEMO_REACH * w->h;
	for (size_t i = 0; i < w->edges.n; i++) {
		clear = fmin(clear,
			     box_dist(w->b->cache.ebox[w->edges.f[i]], q));
	}
	m->at = q;
	m->in = face_has_c(&w->b->cache, f, q);
	m->clear = clear - (2 * SH_EPS);
	return m->in;
}

// The point of surface s nearest p: on a cone, the foot of p on the
// line of the cone in the plane of p and the axis.
static struct vec foot(const struct surf *s, struct vec p)
{
	struct vec d = sub(p, s->o);
	double h = dot(d, s->n);
	struct vec radial = sub(d, mul(s->n, h));
	double rho = len(radial);
	struct vec e = rho > 0 ? mul(radial, 1 / rho) : s->x;
	double k = s->r2;
	double hf = (((rho - s->r) * k) + h) / (1 + (k * k));

	if (s->kind != SF_CONE) {
		return surf_at(s, surf_param(s, p));
	}
	return add(s->o, add(mul(s->n, hf), mul(e, s->r + (k * hf))));
}

static int cmp_size(const void *lhs, const void *rhs)
{
	size_t a = *(const size_t *)lhs;
	size_t b = *(const size_t *)rhs;

	return (a > b) - (a < b);
}

// Leaves each face of the list once.
static void dedupe(struct face_list *l)
{
	size_t k = 0;

	if (l->n < 2) {
		return;
	}
	qsort(l->f, l->n, sizeof(*l->f), cmp_size);
	for (size_t i = 0; i < l->n; i++) {
		if (!k || l->f[k - 1] != l->f[i]) {
			l->f[k++] = l->f[i];
		}
	}
	l->n = k;
}

// The side of p, in the frame of w->b, near the faces in w->near: 1
// inside w->b, 0 outside or on a face; or -1 if no face lies within
// reach, -2 if an edge may lie nearer than the nearest face.
// If the point q of the boundary nearest p lies inside a face, away
// from its edges, p lies on the side of the face that p - q points to.
static int near_side(struct side *w, struct vec p, double reach)
{
	const struct clash_body *b = w->b;
	const struct shape *sh = b->sh;
	double best = HUGE_VAL;
	size_t bf = NONE;
	struct vec bq = p;
	struct box ball;
	int tried = 0;

	for (size_t m = w->near.n; m > 0; m--) {
		double d0 = HUGE_VAL;
		size_t i0 = 0;
		struct vec q0 = p;
		size_t x = 0;

		// The nearest foot of the faces not tried yet, the first m;
		// a face tried goes to the end.
		for (size_t i = 0; i < m; i++) {
			struct vec q = foot(&sh->f[w->near.f[i]].s, p);
			double d = len(sub(p, q));

			if (d < d0) {
				d0 = d;
				i0 = i;
				q0 = q;
			}
		}
		if (d0 > reach) {
			break;
		}
		if (memo_has(w, w->near.f[i0], q0)) {
			best = d0;
			bf = w->near.f[i0];
			bq = q0;
			break;
		}
		tried = 1;
		x = w->near.f[i0];
		w->near.f[i0] = w->near.f[m - 1];
		w->near.f[m - 1] = x;
	}
	if (bf == NONE) {
		return tried ? -2 : -1;
	}
	if (best < SH_EPS) {
		return 0;
	}
	ball.lo = ball.hi = p;
	box_pad(&ball, best + (2 * SH_EPS));
	{
		struct query qe = { 0, ball, { p, p }, 0 };

		if (tree_find(&b->etree, &qe, NULL, NULL)) {
			return -2;
		}
	}
	return dot(sub(p, bq), surf_normal(&sh->f[bf].s, bq)) *
		       sh->f[bf].sense <
	       0;
}

// Whether p, in the frame of w->b, lies inside it, not on one of its
// faces: by the nearest face, looked for ever farther, SIDE_GROWTHS times,
// if it tells, else by rays.
static int in_other(struct side *w, struct vec p)
{
	const struct clash_body *b = w->b;
	struct query q = { 0, { p, p }, { p, p }, 0 };
	struct ray_faces rf = { ray_faces, b, &b->cache };
	int k = -1;

	if (!box_has(b->box, p)) {
		return 0;
	}
	for (int g = 0; k == -1 && g < SIDE_GROWTHS; g++) {
		double r = REACH * pow(SIDE_GROW, g);
		struct query qr = { 0, { p, p }, { p, p }, 0 };

		box_pad(&qr.x, r);
		w->near.n = 0;
		(void)tree_find(&b->ttree, &qr, b->tface, &w->near);
		dedupe(&w->near);
		k = near_side(w, p, r);
	}
	if (k >= 0) {
		return k;
	}
	box_pad(&q.x, SH_EPS);
	w->near.n = 0;
	(void)tree_find(&b->ftree, &q, NULL, &w->near);
	for (size_t i = 0; i < w->near.n; i++) {
		if (face_on_c(&b->cache, w->near.f[i], p)) {
			return 0;
		}
	}
	return solid_has_with(b->sh, p, b->box, &rf);
}

// Counts the point p of w->a, moved d along dir, if it lies in w->b.
static void count_at(struct side *w, struct vec p, struct vec dir)
{
	struct vec at = rig_pt(w->ab, add(p, mul(dir, CLASH_MARGIN)));

	if (w->n < w->max && in_other(w, at)) {
		w->n++;
	}
}

// Counts the sample of face f of w->a near p, on its surface; returns 0
// if it lies off the face, where a triangle of the mesh crosses a curved
// edge, and leaves it.  Such a point can lie inside a body that only
// touches the face, as a countersink touches a cone in it.
static int sample(struct side *w, size_t f, struct vec p)
{
	const struct sface *fc = &w->a->sh->f[f];
	struct vec q = surf_at(&fc->s, surf_param(&fc->s, p));

	if (!face_has_c(&w->a->cache, f, q)) {
		return 0;
	}
	count_at(w, q, mul(surf_normal(&fc->s, q), -(double)fc->sense));
	return 1;
}

// Whether a face of w->b comes within REACH of the box x, in the frame
// of w->b.
static int near_other(const struct side *w, struct box x)
{
	struct query q = { 0, x, { x.lo, x.lo }, 0 };

	box_pad(&q.x, REACH);
	return tree_find(&w->b->ttree, &q, NULL, NULL) > 0;
}

// What to do with a piece whose box, in the frame of w->b, is x: 0 to
// leave it, outside w->b; 1 to sample it once, as no face of w->b
// comes near it; 2 to sample it closely.
static int piece_kind(const struct side *w, struct box x)
{
	struct box y = x;

	box_pad(&y, REACH);
	if (!box_meet(y, w->b->box)) {
		return 0;
	}
	return near_other(w, x) ? 2 : 1;
}

// How far a surface s strays, within a piece size across, from the
// plane of three of its points there: no more than size^2 / (2 r) for a
// least radius of curvature r; for a cone, whose radius shrinks to its
// tip, the chord of the mesh.
static double bulge(const struct surf *s, double size)
{
	double r = s->r;

	if (s->kind == SF_PLANE) {
		return 0;
	}
	if (s->kind == SF_CONE) {
		return CLASH_CHORD;
	}
	if (s->kind == SF_TORUS) {
		r = fmin(s->r2, fabs(s->r - s->r2));
	}
	if (s->kind == SF_PIPE) {
		r = fmin(s->r2, fabs(pipe_bend(s) - s->r2));
	}
	return r > 0 ? size * size / (2 * r) : HUGE_VAL;
}

// Whether no face of w->b cuts the piece x of face f of w->a, its box
// bx in the frame of w->b, though some come near: each face of w->b near
// it leaves all of the piece on one side of its surface, CLASH_MARGIN or
// more off, or holds all of it in its plane.  As each face lies in its
// surface, the faces near the piece then stay off it, but where one
// holds it.  The surface of f strays from the piece by no more than the
// chord, nor than the distance of its corners from it and its bulge.
// The distance from a plane is linear along the piece, so its corners
// tell; that from a curved surface changes by no
// more than the distance moved, so a piece whose corners all lie
// farther from it than its size lies off it.  The piece then lies
// wholly on one side of w->b, or on a face of it, as one sample tells.
static int plain_piece(struct side *w, size_t f, const struct piece_tri *x,
		       struct box bx)
{
	const struct shape *sh = w->b->sh;
	struct query q = { 0, bx, { bx.lo, bx.lo }, 0 };
	const struct surf *as = &w->a->sh->f[f].s;
	int flat = as->kind == SF_PLANE;
	double size = 0;
	double off = 0;
	double tol = 0;
	struct vec c[3];

	box_pad(&q.x, REACH);
	w->near.n = 0;
	(void)tree_find(&w->b->ttree, &q, w->b->tface, &w->near);
	dedupe(&w->near);
	for (int k = 0; k < 3; k++) {
		c[k] = rig_pt(w->ab, x->p[k]);
		size = fmax(size, len(sub(x->p[(k + 1) % 3], x->p[k])));
		off = fmax(off, fabs(surf_dist(as, x->p[k])));
	}
	tol = fmin(CLASH_CHORD, off + bulge(as, size)) + CLASH_MARGIN + SH_EPS;
	for (size_t i = 0; i < w->near.n; i++) {
		const struct surf *s = &sh->f[w->near.f[i]].s;
		int above = 0;
		int below = 0;
		int on = 0;

		for (int k = 0; k < 3; k++) {
			double d = s->kind == SF_PLANE ?
					   dot(s->n, sub(c[k], s->o)) :
					   surf_dist(s, c[k]);
			double t = s->kind == SF_PLANE ? tol : tol + size;

			above += d > t;
			below += d < -t;
			on += flat && s->kind == SF_PLANE && fabs(d) < SH_EPS;
		}
		if (above != 3 && below != 3 && on != 3) {
			return 0;
		}
	}
	return 1;
}

// The point of a narrow piece at the share t of its longest side,
// lifted by the share z of its height there.
static struct vec strip_at(const struct strip *st, double t, double z)
{
	double share = 1;

	if (t <= st->tf && st->tf > 0) {
		share = t / st->tf;
	} else if (t > st->tf && st->tf < 1) {
		share = (1 - t) / (1 - st->tf);
	}

	return add(add(st->a, mul(st->ab, t)), mul(st->up, share * z));
}

// Samples the piece x of face f near w->b if it is narrow: if its height
// over its longest side k is w->h or less, as halving it would make
// slivers.  The piece goes in sections along that side, SMALL w->h long;
// a section far from w->b takes one sample, or none outside its box, and
// a near one samples STRIP_STEP w->h apart, each halfway up the piece
// above it, so that no point of the piece lies farther than 0.65 w->h
// from one.  Returns 0 for a piece not narrow.
static int thin_piece(struct side *w, size_t f, const struct piece_tri *x,
		      size_t k)
{
	struct strip st;
	struct vec c = x->p[(k + 2) % 3];
	double l = 0;
	size_t ns = 1;

	st.a = x->p[k];
	st.ab = sub(x->p[(k + 1) % 3], st.a);
	l = len(st.ab);
	if (!(l > 0)) {
		return 0;
	}
	st.tf = dot(sub(c, st.a), st.ab) / (l * l);
	st.up = sub(c, add(st.a, mul(st.ab, st.tf)));
	if (len(st.up) > w->h) {
		return 0;
	}
	while ((double)ns * SMALL * w->h < l && ns < STRIP_MAX) {
		ns++;
	}
	for (size_t j = 0; j < ns && w->n < w->max; j++) {
		double t0 = (double)j / (double)ns;
		double t1 = (double)(j + 1) / (double)ns;
		struct box bx = box_empty();
		int kind = 0;
		size_t n = 1;

		box_add(&bx, rig_pt(w->ab, strip_at(&st, t0, 0)));
		box_add(&bx, rig_pt(w->ab, strip_at(&st, t1, 0)));
		box_add(&bx, rig_pt(w->ab, strip_at(&st, t0, 1)));
		box_add(&bx, rig_pt(w->ab, strip_at(&st, t1, 1)));
		if (st.tf > t0 && st.tf < t1) {
			box_add(&bx, rig_pt(w->ab, c));
		}
		kind = piece_kind(w, bx);
		if (kind == 0) {
			continue;
		}
		while (kind == 2 &&
		       (double)n * STRIP_STEP * w->h < l / (double)ns) {
			n++;
		}
		for (size_t i = 0; i < n && w->n < w->max; i++) {
			double t = t0 +
				   ((t1 - t0) * ((double)i + HALF) / (double)n);

			(void)sample(w, f, strip_at(&st, t, HALF));
		}
	}
	return 1;
}

// Samples the triangle p of face f: a piece far from the faces of w->b
// once, at its middle, and a piece near them halved until no side is
// longer than w->h, at its middle.  A piece whose middle lies off the
// face is halved on, down to w->h.
static void sample_tri(struct side *w, size_t f, const struct vec *p)
{
	struct piece_tri stack[SPLIT_MAX + 2];
	size_t top = 1;

	memcpy(stack[0].p, p, sizeof(stack[0].p));
	stack[0].near = 0;
	stack[0].depth = 0;
	while (top && w->n < w->max) {
		struct piece_tri x = stack[--top];
		struct box bx = box_empty();
		double l = 0;
		size_t k = longest_side(&x, &l);
		int kind = 0;

		for (size_t i = 0; i < 3; i++) {
			box_add(&bx, rig_pt(w->ab, x.p[i]));
		}
		kind = x.near && l < SMALL * w->h ? 2 : piece_kind(w, bx);
		if (kind == 0) {
			continue;
		}
		if ((kind == 1 || l <= w->h || x.depth >= SPLIT_MAX ||
		     plain_piece(w, f, &x, bx)) &&
		    (sample(w, f,
			    mul(add(add(x.p[0], x.p[1]), x.p[2]), 1.0 / 3)) ||
		     l <= w->h || x.depth >= SPLIT_MAX)) {
			continue;
		}
		if (!thin_piece(w, f, &x, k)) {
			halve(&x, k, &stack[top]);
			top += 2;
		}
	}
}

// The direction into the solid at point p of edge e of w->a: between
// the normals of its two faces, turned inward.
static struct vec edge_in(const struct side *w, size_t e, struct vec p)
{
	const struct shape *sh = w->a->sh;
	struct vec d = v3(0, 0, 0);

	for (int k = 0; k < 2; k++) {
		size_t f = w->a->ef[e][k];

		if (f != NONE) {
			const struct sface *fc = &sh->f[f];

			d = sub(d,
				mul(surf_normal(&fc->s, p), (double)fc->sense));
		}
	}
	return len(d) > 0 ? unit(d) : d;
}

// A piece of an edge, from t0 to t1, and the halvings that made it.
struct piece_edge {
	double t0, t1;
	int depth;
};

// Whether the edge se of w->a, as pc has it in the frame of w->b, runs
// into w->b on one side of its crossing at t of a face of w->b: a point
// of the edge CROSS_RUN on, or 4, 16 or 64 times that, moved
// CLASH_MARGIN into w->a, lies in w->b.  So a crossing counts only where
// the edge goes into w->b as deep as a sample of a face must.
static int runs_in(struct side *w, const struct sedge *se,
		   const struct cpiece *pc, double t)
{
	size_t e = (size_t)(se - w->a->sh->e);
	double speed = len(curve_tan(&pc->c, t));

	for (int g = 0; g < CROSS_RUNS; g++) {
		double run = CROSS_RUN * pow(CROSS_GROW, g);
		for (int k = -1; k <= 1; k += 2) {
			double tk = t + (k * run / speed);
			struct vec at = curve_at(&pc->c, tk);
			struct vec in = rig_dir(
				w->ab, edge_in(w, e, curve_at(&se->c, tk)));

			if (tk >= se->t0 && tk <= se->t1 &&
			    in_other(w, add(at, mul(in, CLASH_MARGIN)))) {
				return 1;
			}
		}
	}
	return 0;
}

// Counts the places where the piece x of edge se of w->a, its box bx in
// the frame of w->b, passes through a face of w->b: inside the face and
// away from its edges, at an angle, so that the edge runs from outside
// w->b into it there, however short the run inside.
static void edge_crossings(struct side *w, const struct sedge *se,
			   struct piece_edge x, struct box bx)
{
	const struct clash_body *b = w->b;
	struct query q = { 0, bx, { bx.lo, bx.lo }, 0 };
	struct cpiece pc = { se->c, x.t0, x.t1 };

	pc.c.o = rig_pt(w->ab, pc.c.o);
	pc.c.n = rig_dir(w->ab, pc.c.n);
	pc.c.x = rig_dir(w->ab, pc.c.x);
	box_pad(&q.x, REACH);
	w->near.n = 0;
	(void)tree_find(&b->ttree, &q, b->tface, &w->near);
	dedupe(&w->near);
	for (size_t i = 0; i < w->near.n && w->n < w->max; i++) {
		size_t f = w->near.f[i];
		const struct sface *fc = &b->sh->f[f];
		double t[ROOTS_MAX];
		size_t n = curve_surf(&pc, &fc->s, t);

		for (size_t k = 0; k < n && w->n < w->max; k++) {
			struct vec at = curve_at(&pc.c, t[k]);
			struct vec tan = curve_tan(&pc.c, t[k]);
			struct query qe = { 0, { at, at }, { at, at }, 0 };

			box_pad(&qe.x, CROSS_CLEAR);
			if (!box_has(b->fbox[f], at) || !(len(tan) > 0) ||
			    fabs(dot(unit(tan), surf_normal(&fc->s, at))) <
				    CROSS_SINE ||
			    tree_find(&b->etree, &qe, NULL, NULL) ||
			    !face_has_c(&b->cache, f, at)) {
				continue;
			}
			if (runs_in(w, se, &pc, t[k])) {
				w->n++;
			}
		}
	}
}

// Samples edge e of w->a as sample_tri samples a triangle.
static void sample_edge(struct side *w, size_t e)
{
	const struct sedge *se = &w->a->sh->e[e];
	struct piece_edge stack[SPLIT_MAX + ARC_PIECES + 2];
	size_t top = 0;
	double turns = se->c.kind == CV_LINE ? 0 :
					       (se->t1 - se->t0) / ARC_PIECE;
	size_t np = 1;

	while (np < ARC_PIECES && (double)np < turns) {
		np++;
	}
	for (size_t i = 0; i < np; i++) {
		double s = (se->t1 - se->t0) / (double)np;
		struct piece_edge x = { se->t0 + ((double)i * s),
					se->t0 + ((double)(i + 1) * s), 0 };

		stack[top++] = x;
	}
	while (top && w->n < w->max) {
		struct piece_edge x = stack[--top];
		double tm = (x.t0 + x.t1) / 2;
		const struct vec p[3] = { curve_at(&se->c, x.t0),
					  curve_at(&se->c, tm),
					  curve_at(&se->c, x.t1) };
		double l = len(sub(p[2], p[0]));
		struct box bx = box_empty();
		int kind = 0;

		for (int i = 0; i < 3; i++) {
			box_add(&bx, rig_pt(w->ab, p[i]));
		}
		box_pad(&bx, l / 2);
		kind = piece_kind(w, bx);
		if (kind == 0) {
			continue;
		}
		if (kind == 1 || l <= w->h || x.depth >= SPLIT_MAX) {
			count_at(w, p[1], edge_in(w, e, p[1]));
			if (x.depth == 0 && kind == 2) {
				count_at(w, p[0], edge_in(w, e, p[0]));
				count_at(w, p[2], edge_in(w, e, p[2]));
			}
			if (kind == 2) {
				edge_crossings(w, se, x, bx);
			}
			continue;
		}
		{
			struct piece_edge a = { x.t0, tm, x.depth + 1 };
			struct piece_edge b = { tm, x.t1, x.depth + 1 };

			stack[top++] = a;
			stack[top++] = b;
		}
	}
}

// Counts the samples of the faces and edges of w->a that lie in w->b.
static void sample_side(struct side *w)
{
	const struct mesh *m = &w->a->m;

	for (size_t i = 0; i < m->nt && w->n < w->max; i++) {
		const struct mesh_tri *t = &m->t[i];
		struct vec p[3];

		for (int k = 0; k < 3; k++) {
			p[k] = m->v[t->v[k]];
		}
		sample_tri(w, t->face, p);
	}
	for (size_t e = 0; e < w->a->sh->ne && w->n < w->max; e++) {
		sample_edge(w, e);
	}
}

size_t clash_count(struct clash_pose a, struct clash_pose b, size_t max)
{
	struct box wa = box_moved(a.b->box, a.m);
	struct box wb = box_moved(b.b->box, b.m);
	struct side w;
	size_t n = 0;

	box_pad(&wa, CLASH_MARGIN);
	if (!box_meet(wa, wb)) {
		return 0;
	}
	memset(&w, 0, sizeof(w));
	w.h = fmax(STEP_MIN, fmin(CLASH_GRID, fmin(a.b->thin, b.b->thin) / 2));
	for (int k = 0; k < 2 && n < max; k++) {
		struct clash_pose x = k ? b : a;
		struct clash_pose y = k ? a : b;

		w.a = x.b;
		w.b = y.b;
		w.ab = rig_mul(rig_inv(y.m), x.m);
		w.n = 0;
		w.max = max - n;
		w.memo = xcalloc(y.b->sh->nf + 1, sizeof(*w.memo));
		sample_side(&w);
		free(w.memo);
		n += w.n;
	}
	free(w.near.f);
	free(w.edges.f);
	return n;
}

// The least distance up to limit from the points of the mesh of x.b,
// moved by ab into the frame of y, to the pieces of the triangles of y.
static double gap_one(const struct clash_body *x, struct rigid ab,
		      const struct clash_body *y, double limit)
{
	struct face_list near = { NULL, 0, 0 };
	double best = limit;

	for (size_t i = 0; i < x->m.nv; i++) {
		struct vec p = rig_pt(ab, x->m.v[i]);
		struct query q = { 0, { p, p }, { p, p }, 0 };

		box_pad(&q.x, best);
		near.n = 0;
		(void)tree_find(&y->ttree, &q, NULL, &near);
		for (size_t k = 0; k < near.n; k++) {
			best = fmin(best, tri_dist(y->tpt[near.f[k]], p));
		}
	}
	free(near.f);
	return best;
}

double clash_gap(struct clash_pose a, struct clash_pose b, double limit)
{
	struct box wa = box_moved(a.b->box, a.m);
	struct box wb = box_moved(b.b->box, b.m);
	double g = limit;

	box_pad(&wa, limit);
	if (!box_meet(wa, wb)) {
		return limit;
	}
	g = gap_one(a.b, rig_mul(rig_inv(b.m), a.m), b.b, g);
	return gap_one(b.b, rig_mul(rig_inv(a.m), b.m), a.b, g);
}
