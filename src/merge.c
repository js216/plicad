// SPDX-License-Identifier: GPL-3.0-only
// merge.c -- Join the faces of a solid on one surface, and their edges
// Copyright (c) 2026 Jakob Kastelic

// Two faces on one surface that share an edge are one face: the edge
// goes, and their loops join.  A round face stays under a whole turn, so
// the two halves of a cylinder stay two.  Then two edges on one curve
// that meet at a vertex of no other edge are one edge, but the halves of
// a whole circle.

#include "merge.h"
#include "geom.h"
#include "isect.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TURN_ROOM 0.5 // The least gap of a merged round face, in radians.
#define SAME_DIR 1e-9 // The sine of two directions that are one.

enum { SAMPLES = 16 }; // Points of an edge for the turn of a face.

// A loop as a list of uses, and a face as a list of loops.
struct mloop {
	struct suse *u;
	size_t n, cap;
};

struct mface {
	struct mloop *l;
	size_t n, cap;
	int dead;
};

// Two places in a loop, lo before hi.
struct span_z {
	size_t lo, hi;
};

// Where a use lies: its face, its loop and its place there.
struct at {
	size_t f, l, i;
};

static void loop_push(struct mloop *l, struct suse u)
{
	PUSH(l->u, l->n, l->cap, u);
}

static struct mface *faces_of(const struct shape *sh)
{
	struct mface *mf = xcalloc(sh->nf + 1, sizeof(*mf));

	for (size_t f = 0; f < sh->nf; f++) {
		for (size_t k = 0; k < sh->f[f].nloops; k++) {
			const struct sloop *lp = &sh->l[sh->f[f].loop0 + k];
			struct mloop ml = { NULL, 0, 0 };

			for (size_t i = 0; i < lp->n; i++) {
				loop_push(&ml, sh->u[lp->first + i]);
			}
			PUSH(mf[f].l, mf[f].n, mf[f].cap, ml);
		}
	}
	return mf;
}

// Finds the use of edge e that runs forward, or back, among the nf
// faces mf; returns 0 if there is none.
static int find_use(const struct mface *mf, size_t nf, struct suse u,
		    struct at *out)
{
	for (size_t f = 0; f < nf; f++) {
		for (size_t l = 0; l < mf[f].n && !mf[f].dead; l++) {
			for (size_t i = 0; i < mf[f].l[l].n; i++) {
				const struct suse *x = &mf[f].l[l].u[i];

				if (x->e == u.e && x->fwd == u.fwd) {
					*out = (struct at){ f, l, i };
					return 1;
				}
			}
		}
	}
	return 0;
}

// Whether the surfaces a and b, of planes, cylinders and cones, are
// one.
static int one_surface(const struct surf *a, const struct surf *b)
{
	struct curve c[2];
	size_t n = 0;

	return a->kind == b->kind && a->kind <= SF_CONE &&
	       surf_surf(a, b, c, &n) == SS_SAME;
}

// Adds to a the angles u on surface s of points of the edges of face f
// of mf.
static void face_angles(const struct shape *sh, const struct mface *f,
			const struct surf *s, double **a, size_t *n,
			size_t *cap)
{
	for (size_t l = 0; l < f->n; l++) {
		for (size_t k = 0; k < f->l[l].n; k++) {
			const struct sedge *e = &sh->e[f->l[l].u[k].e];

			for (int i = 0; i <= SAMPLES; i++) {
				struct vec p =
					edge_at(e, (double)i / (double)SAMPLES);
				double u = surf_param(s, p).u;

				u -= 2 * PI * floor(u / (2 * PI));
				PUSH(*a, *n, *cap, u);
			}
		}
	}
}

// Whether the faces fa and fb of mf, round faces on the surface s,
// leave a gap in the turn about its axis when joined.
static int leaves_gap(const struct shape *sh, const struct mface *fa,
		      const struct mface *fb, const struct surf *s)
{
	double *a = NULL;
	size_t n = 0;
	size_t cap = 0;
	double gap = 0;

	face_angles(sh, fa, s, &a, &n, &cap);
	face_angles(sh, fb, s, &a, &n, &cap);
	sort_doubles(a, n);
	for (size_t i = 0; i < n; i++) {
		double next = i + 1 < n ? a[i + 1] : a[0] + (2 * PI);

		gap = fmax(gap, next - a[i]);
	}
	free(a);
	return gap > TURN_ROOM;
}

// Whether edge e of sh lies between two faces, fa and fb, that are one
// face: on one surface, facing one way, and under a whole turn together.
static int one_face(const struct shape *sh, const struct mface *mf, size_t e,
		    struct at ua, struct at ub)
{
	const struct sface *a = &sh->f[ua.f];
	const struct sface *b = &sh->f[ub.f];
	struct vec m = edge_at(&sh->e[e], MID);

	if (!one_surface(&a->s, &b->s)) {
		return 0;
	}
	if (dot(mul(surf_normal(&a->s, m), a->sense),
		mul(surf_normal(&b->s, m), b->sense)) < MID) {
		return 0;
	}
	return a->s.kind == SF_PLANE ||
	       leaves_gap(sh, &mf[ua.f], &mf[ub.f], &a->s);
}

// Adds to out the uses of loop l after place i, round to the one before
// it.
static void rest_of(const struct mloop *l, size_t i, struct mloop *out)
{
	for (size_t k = 1; k < l->n; k++) {
		loop_push(out, l->u[(i + k) % l->n]);
	}
}

// Makes the loops la and lb, with a use of one edge at the places ia
// and ib, one loop in la, without the edge; lb stays empty.
static void join_loops(struct mloop *la, size_t ia, struct mloop *lb, size_t ib)
{
	struct mloop one = { NULL, 0, 0 };

	rest_of(la, ia, &one);
	rest_of(lb, ib, &one);
	free(la->u);
	free(lb->u);
	*la = one;
	lb->u = NULL;
	lb->n = 0;
}

// Cuts loop l of face f, with both uses of one edge at the places lo
// and hi, into the two loops between them; one may be empty.
static void split_loop(struct mface *f, size_t l, struct span_z at)
{
	struct mloop *la = &f->l[l];
	struct mloop one = { NULL, 0, 0 };
	struct mloop two = { NULL, 0, 0 };

	for (size_t k = at.lo + 1; k < at.hi; k++) {
		loop_push(&one, la->u[k]);
	}
	for (size_t k = at.hi + 1; k < la->n + at.lo; k++) {
		loop_push(&two, la->u[k % la->n]);
	}
	free(la->u);
	*la = one;
	if (two.n) {
		PUSH(f->l, f->n, f->cap, two);
	}
}

// Takes the edge with its uses at a and b out of the faces mf: two loops
// become one, or one loop two; the loops of the face of b go to that of
// a.
static void kill_edge(struct mface *mf, struct at a, struct at b)
{
	if (a.f == b.f && a.l == b.l) {
		struct span_z at = { a.i < b.i ? a.i : b.i,
				     a.i < b.i ? b.i : a.i };

		split_loop(&mf[a.f], a.l, at);
		return;
	}
	join_loops(&mf[a.f].l[a.l], a.i, &mf[b.f].l[b.l], b.i);
	if (a.f == b.f) {
		return;
	}
	for (size_t k = 0; k < mf[b.f].n; k++) {
		if (mf[b.f].l[k].n) {
			PUSH(mf[a.f].l, mf[a.f].n, mf[a.f].cap, mf[b.f].l[k]);
		}
	}
	free(mf[b.f].l);
	mf[b.f].l = NULL;
	mf[b.f].n = 0;
	mf[b.f].dead = 1;
}

// Joins the faces of sh on one surface, in mf.  An edge with a face
// that an earlier join made of two on both its sides goes as well.
static void join_faces(const struct shape *sh, struct mface *mf)
{
	for (size_t e = 0; e < sh->ne; e++) {
		struct at a;
		struct at b;
		struct suse fw = { e, 1 };
		struct suse bk = { e, 0 };

		if (!find_use(mf, sh->nf, fw, &a) ||
		    !find_use(mf, sh->nf, bk, &b)) {
			continue;
		}
		if (b.f < a.f) {
			// The first face keeps its name.
			struct at t = a;

			a = b;
			b = t;
		}
		if (a.f == b.f || one_face(sh, mf, e, a, b)) {
			kill_edge(mf, a, b);
		}
	}
}

// Whether the curves a and b are one line.
static int one_line(const struct curve *a, const struct curve *b)
{
	return a->kind == CV_LINE && b->kind == CV_LINE &&
	       len(cross(a->x, b->x)) < SAME_DIR &&
	       len(cross(sub(b->o, a->o), a->x)) < SH_EPS;
}

// The edges at each vertex of the faces mf: their count, and two of
// them.
struct vuse {
	size_t n;
	size_t e[2];
};

static void count_ends(const struct shape *sh, const struct mface *mf,
		       struct vuse *vu)
{
	int *seen = xcalloc(sh->ne + 1, sizeof(*seen));

	for (size_t f = 0; f < sh->nf; f++) {
		for (size_t l = 0; l < mf[f].n; l++) {
			for (size_t i = 0; i < mf[f].l[l].n; i++) {
				seen[mf[f].l[l].u[i].e] = 1;
			}
		}
	}
	for (size_t e = 0; e < sh->ne; e++) {
		const size_t v[2] = { sh->e[e].v0, sh->e[e].v1 };

		for (int k = 0; k < 2 && seen[e]; k++) {
			if (vu[v[k]].n < 2) {
				vu[v[k]].e[vu[v[k]].n] = e;
			}
			vu[v[k]].n++;
		}
	}
	free(seen);
}

// Makes the two edges of vu, on one line and joined at vertex v of sh
// alone, the one edge that is its first, in the faces mf.
static void join_edges(struct shape *sh, struct mface *mf, size_t v,
		       const struct vuse *vu)
{
	size_t a = vu->e[0];
	size_t b = vu->e[1];
	struct sedge *ea = &sh->e[a];
	const struct sedge *eb = &sh->e[b];
	size_t far = eb->v0 == v ? eb->v1 : eb->v0;
	double t = curve_param(&ea->c, sh->v[far]);
	int at_end = ea->v1 == v;

	if (at_end) {
		ea->t1 = t;
		ea->v1 = far;
	} else {
		ea->t0 = t;
		ea->v0 = far;
	}
	for (size_t f = 0; f < sh->nf; f++) {
		for (size_t l = 0; l < mf[f].n; l++) {
			struct mloop *lp = &mf[f].l[l];
			size_t n = 0;

			for (size_t i = 0; i < lp->n; i++) {
				if (lp->u[i].e != b) {
					lp->u[n++] = lp->u[i];
				}
			}
			lp->n = n;
		}
	}
}

// Joins the edges of sh on one line that meet at a vertex of no other
// edge, in the faces mf.
static void join_all_edges(struct shape *sh, struct mface *mf)
{
	for (size_t v = 0; v < sh->nv; v++) {
		struct vuse *vu = xcalloc(sh->nv + 1, sizeof(*vu));
		size_t a = 0;
		size_t b = 0;

		count_ends(sh, mf, vu);
		a = vu[v].e[0];
		b = vu[v].e[1];
		if (vu[v].n == 2 && a != b &&
		    one_line(&sh->e[a].c, &sh->e[b].c)) {
			join_edges(sh, mf, v, &vu[v]);
		}
		free(vu);
	}
}

// Joins the faces of sh that share an edge on one surface into one face,
// which keeps the name of the first, and then the edges on one line
// that meet at a vertex of no other edge.
void shape_merge(struct shape *sh)
{
	struct mface *mf = faces_of(sh);
	struct shape out;

	join_faces(sh, mf);
	join_all_edges(sh, mf);
	shape_init(&out, sh->name);
	for (size_t f = 0; f < sh->nf; f++) {
		if (mf[f].dead) {
			continue;
		}
		shape_face(&out, sh->f[f].s, sh->f[f].sense, sh->f[f].name);
		for (size_t l = 0; l < mf[f].n; l++) {
			const struct mloop *lp = &mf[f].l[l];
			size_t first = out.nu;

			for (size_t i = 0; i < lp->n; i++) {
				const struct sedge *e = &sh->e[lp->u[i].e];
				int fwd = 1;
				size_t ne = shape_piece(
					&out, &e->c,
					(struct span){ e->t0, e->t1 }, &fwd);

				shape_use(&out, ne, lp->u[i].fwd == fwd);
			}
			if (lp->n) {
				shape_end_loop(&out, first);
			}
			free(lp->u);
		}
		free(mf[f].l);
	}
	free(mf);
	shape_free(sh);
	*sh = out;
}
