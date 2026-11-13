// SPDX-License-Identifier: GPL-3.0-only
// split.c -- A face of an exact solid cut into pieces along curves on it
// Copyright (c) 2026 Jakob Kastelic

// The edges of the face and the cuts break at every vertex that lies on
// them.  Both ways of each piece go into a graph in the parameters of
// the surface, ordered round each vertex by the way they leave it.  A
// walk that turns as far right as it can at each vertex keeps a part on
// its left and goes round it once.  A walk round a part counterclockwise
// is an outer loop; one that goes clockwise is a hole of the smallest
// outer loop round it, or the outside of the face.  A part counts if a
// point inside it lies inside the face.

#include "split.h"
#include "geom.h"
#include "isect.h"
#include "shape.h"
#include "util.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PROBE 1e-4 // How far along a piece its way out of a vertex is read.
#define LEFT 1e-6 // How far left of a loop a point inside its part lies.
#define SHIFT 1e-8 // A shift of a ray off a vertex, in mm, well below LEFT.
#define AT_VERTEX 1e-10 // A ray this near a vertex meets it, in mm.

enum {
	GAUSS = 4, // Points of the rule for the area of a loop.
	AREA_PIECES = 16, // Pieces of an edge for the area.
	SCAN_SAMPLES = 8, // Points of each edge for a scan line.
	NUDGES = 4, // Tries of a ray in a loop.
	UNSAFE = INT_MIN // A winding number of a ray through a vertex.
};

static const double gx[GAUSS] = { -0.8611363115940526, -0.3399810435848563,
				  0.3399810435848563, 0.8611363115940526 };
static const double gw[GAUSS] = { 0.3478548451374538, 0.6521451548625461,
				  0.6521451548625461, 0.3478548451374538 };

// A piece of the graph: its ends, and whether it came from the face.
struct gpiece {
	struct pedge pe;
	size_t va, vb;
};

struct graph {
	const struct shape *sh;
	size_t f;
	struct facemap fm;
	struct vec *v;
	size_t nv, capv;
	struct gpiece *p;
	size_t np, capp;
	size_t *out; // Half-edges out of each vertex, sorted by angle.
	size_t *out0; // Where the half-edges of vertex i start in out.
	double *ang; // The angle of each half-edge out of its vertex.
	size_t *next;
	int *used;
};

static size_t vertex_of(struct graph *g, struct vec p)
{
	for (size_t i = 0; i < g->nv; i++) {
		if (len(sub(g->v[i], p)) < SH_EPS) {
			return i;
		}
	}
	PUSH(g->v, g->nv, g->capv, p);
	return g->nv - 1;
}

// The parameter of p on piece c, an angle moved into [t0, t0 + 2 pi).
static double param_on(const struct cpiece *c, struct vec p)
{
	double t = curve_param(&c->c, p);

	return curve_turns(&c->c) ? turn_from(t, c->t0) : t;
}

struct cutpt {
	double t;
	struct vec p;
};

static void sort_cuts(struct cutpt *k, size_t n)
{
	for (size_t i = 1; i < n; i++) {
		struct cutpt x = k[i];
		size_t j = i;

		for (; j > 0 && k[j - 1].t > x.t; j--) {
			k[j] = k[j - 1];
		}
		k[j] = x;
	}
}

// Whether the graph holds a piece from va to vb, either way, through
// the point m.
static int have_piece(const struct graph *g, size_t va, size_t vb, struct vec m)
{
	for (size_t i = 0; i < g->np; i++) {
		const struct gpiece *q = &g->p[i];

		if (((q->va == va && q->vb == vb) ||
		     (q->va == vb && q->vb == va)) &&
		    len(sub(curve_at(&q->pe.p.c, (q->pe.p.t0 + q->pe.p.t1) / 2),
			    m)) < SH_EPS) {
			return 1;
		}
	}
	return 0;
}

static void add_piece(struct graph *g, const struct cpiece *c, double t0,
		      double t1)
{
	struct gpiece q;

	if (!(t1 - t0 > 0)) {
		return;
	}
	q.pe.p.c = c->c;
	q.pe.p.t0 = t0;
	q.pe.p.t1 = t1;
	q.pe.a = curve_at(&c->c, t0);
	q.pe.b = curve_at(&c->c, t1);
	q.va = vertex_of(g, q.pe.a);
	q.vb = vertex_of(g, q.pe.b);
	if (q.va == q.vb &&
	    len(sub(curve_at(&c->c, (t0 + t1) / 2), q.pe.a)) < SH_EPS) {
		return;
	}
	if (!have_piece(g, q.va, q.vb, curve_at(&c->c, (t0 + t1) / 2))) {
		PUSH(g->p, g->np, g->capp, q);
	}
}

// Adds the piece c to the graph, broken at the vertices on it.
static void add_broken(struct graph *g, const struct cpiece *c)
{
	struct cutpt *k = NULL;
	size_t n = 0;
	size_t cap = 0;
	double t = c->t0;
	struct vec a = curve_at(&c->c, c->t0);
	struct vec b = curve_at(&c->c, c->t1);

	for (size_t i = 0; i < g->nv; i++) {
		struct cutpt x = { param_on(c, g->v[i]), g->v[i] };

		if (len(sub(x.p, a)) > SH_EPS && len(sub(x.p, b)) > SH_EPS &&
		    x.t > c->t0 && x.t < c->t1 &&
		    len(sub(curve_at(&c->c, x.t), x.p)) < SH_EPS) {
			PUSH(k, n, cap, x);
		}
	}
	sort_cuts(k, n);
	for (size_t i = 0; i < n; i++) {
		add_piece(g, c, t, k[i].t);
		t = k[i].t;
	}
	add_piece(g, c, t, c->t1);
	free(k);
}

// The direction in the parameters in which half-edge h leaves its
// vertex, as an angle.
static double leave_angle(const struct graph *g, size_t h)
{
	const struct cpiece *c = &g->p[h / 2].pe.p;
	int fwd = !(h % 2);
	double dt = (c->t1 - c->t0) * PROBE;
	struct vec at = curve_at(&c->c, fwd ? c->t0 : c->t1);
	struct vec to = curve_at(&c->c, fwd ? c->t0 + dt : c->t1 - dt);
	struct uv qa = fm_uv(&g->fm, at);
	struct uv qb = fm_uv(&g->fm, to);

	return atan2(qb.v - qa.v, qb.u - qa.u);
}

static size_t head(const struct graph *g, size_t h)
{
	return h % 2 ? g->p[h / 2].va : g->p[h / 2].vb;
}

static size_t tail(const struct graph *g, size_t h)
{
	return h % 2 ? g->p[h / 2].vb : g->p[h / 2].va;
}

// Sorts the half-edges out of each vertex by angle, and links each to
// the next one round its part.
static void link(struct graph *g)
{
	size_t nh = 2 * g->np;
	size_t *pos = xmalloc((nh + 1) * sizeof(*pos));

	g->out0 = xcalloc(g->nv + 2, sizeof(*g->out0));
	g->out = xmalloc((nh + 1) * sizeof(*g->out));
	g->ang = xmalloc((nh + 1) * sizeof(*g->ang));
	g->next = xmalloc((nh + 1) * sizeof(*g->next));
	g->used = xcalloc(nh + 1, sizeof(*g->used));
	for (size_t h = 0; h < nh; h++) {
		g->out0[tail(g, h) + 1]++;
		g->ang[h] = leave_angle(g, h);
	}
	for (size_t i = 0; i < g->nv; i++) {
		g->out0[i + 1] += g->out0[i];
	}
	for (size_t i = 0; i < g->nv; i++) {
		pos[i] = g->out0[i];
	}
	for (size_t h = 0; h < nh; h++) {
		g->out[pos[tail(g, h)]++] = h;
	}
	for (size_t i = 0; i < g->nv; i++) {
		size_t a = g->out0[i];
		size_t b = g->out0[i + 1];

		for (size_t k = a + 1; k < b; k++) {
			size_t x = g->out[k];
			size_t j = k;

			for (; j > a && g->ang[g->out[j - 1]] > g->ang[x];
			     j--) {
				g->out[j] = g->out[j - 1];
			}
			g->out[j] = x;
		}
		for (size_t k = a; k < b; k++) {
			pos[g->out[k]] = k;
		}
	}
	for (size_t h = 0; h < nh; h++) {
		size_t w = head(g, h);
		size_t t = pos[h ^ 1U];
		size_t a = g->out0[w];
		size_t b = g->out0[w + 1];

		g->next[h] = g->out[t == a ? b - 1 : t - 1];
	}
	free(pos);
}

// A walk round a part: its half-edges and its area in the parameters.
struct cycle {
	size_t *h;
	size_t n;
	double area;
	size_t owner;
};

// Twice the area term of Green's theorem along half-edge h: u dv.
static double half_area(const struct graph *g, size_t h)
{
	const struct cpiece *c = &g->p[h / 2].pe.p;
	double a = h % 2 ? c->t1 : c->t0;
	double b = h % 2 ? c->t0 : c->t1;
	double step = (b - a) / (double)AREA_PIECES;
	struct uv prev = fm_uv(&g->fm, curve_at(&c->c, a));
	double sum = 0;

	for (int i = 0; i < AREA_PIECES; i++) {
		for (int j = 0; j < GAUSS; j++) {
			double t = a + (step * (i + ((1 + gx[j]) / 2)));
			struct vec p = curve_at(&c->c, t);
			struct vec d =
				mul(curve_tan(&c->c, t), step > 0 ? 1 : -1);
			struct uv q = fm_uv(&g->fm, p);
			const struct surf *s = &g->sh->f[g->f].s;

			sum += q.u * surf_dv(s, (struct line3){ p, d }) *
			       gw[j] * fabs(step) / 2;
			prev = q;
		}
	}
	(void)prev;
	return sum;
}

static struct cycle walk(struct graph *g, size_t h0)
{
	struct cycle c = { NULL, 0, 0, NONE };
	size_t cap = 0;
	size_t h = h0;

	do {
		g->used[h] = 1;
		PUSH(c.h, c.n, cap, h);
		c.area += half_area(g, h);
		h = g->next[h];
	} while (h != h0 && c.n <= 2 * g->np);
	if (h != h0) {
		die("internal error: a part of face %s does not close",
		    g->sh->f[g->f].name);
	}
	return c;
}

// The winding number of cycle c about the point p, by the crossings of
// the ray along +u, -1 round a hole; UNSAFE if the ray meets the cycle
// at a vertex.
static int cycle_wind(const struct graph *g, const struct cycle *c,
		      struct vec p)
{
	const struct surf *s = &g->sh->f[g->f].s;
	struct uv q = fm_uv(&g->fm, p);
	struct plane3 pl = { s->n, dot(s->n, p) };
	int w = 0;

	if (s->kind == SF_PLANE) {
		pl.n = frame_y(s->n, s->x);
		pl.d = dot(pl.n, p);
	}
	for (size_t i = 0; i < c->n; i++) {
		const struct cpiece *pc = &g->p[c->h[i] / 2].pe.p;
		int fwd = !(c->h[i] % 2);
		double t[ROOTS_MAX];
		size_t n = curve_plane(pc, pl, t);

		for (size_t k = 0; k < n; k++) {
			struct vec r = curve_at(&pc->c, t[k]);
			struct uv qr = fm_uv(&g->fm, r);
			struct line3 at = { r, curve_tan(&pc->c, t[k]) };
			int dv = cross_sign(s, pl, at) * (fwd ? 1 : -1);

			if (fabs(qr.v - q.v) > SH_EPS || qr.u <= q.u) {
				continue;
			}
			if (len(sub(r, curve_at(&pc->c, pc->t0))) < AT_VERTEX ||
			    len(sub(r, curve_at(&pc->c, pc->t1))) < AT_VERTEX) {
				return UNSAFE;
			}
			w += dv;
		}
	}
	return w;
}

// Whether p lies inside cycle c: the winding, tried with p moved a
// little if the ray meets a vertex.
static int in_cycle(const struct graph *g, const struct cycle *c, struct vec p)
{
	const struct surf *s = &g->sh->f[g->f].s;
	struct vec dir = iso_plane(s, p).n;

	for (int i = 0; i < NUDGES; i++) {
		int w = cycle_wind(g, c, add(p, mul(dir, SHIFT * i)));

		if (w != UNSAFE) {
			return w != 0;
		}
	}
	die("internal error: no ray in face %s misses the corners",
	    g->sh->f[g->f].name);
}

// A point just left of the middle of the first half-edge of cycle c, in
// the part that c bounds.
static struct vec cycle_point(const struct graph *g, const struct cycle *c)
{
	const struct cpiece *pc = &g->p[c->h[0] / 2].pe.p;
	double sg = c->h[0] % 2 ? -1 : 1;
	double tm = (pc->t0 + pc->t1) / 2;
	double dt = (pc->t1 - pc->t0) * PROBE * sg;
	struct uv a = fm_uv(&g->fm, curve_at(&pc->c, tm));
	struct uv b = fm_uv(&g->fm, curve_at(&pc->c, tm + dt));
	double du = b.u - a.u;
	double dv = b.v - a.v;
	double l = hypot(du, dv);
	struct uv q = { a.u - (dv / l * LEFT), a.v + (du / l * LEFT) };

	return surf_at(&g->sh->f[g->f].s, q);
}

// The v of the points of the pieces of cycle c, into a list.
static void cycle_vs(const struct graph *g, const struct cycle *c, double **v,
		     size_t *n, size_t *cap)
{
	for (size_t i = 0; i < c->n; i++) {
		const struct cpiece *pc = &g->p[c->h[i] / 2].pe.p;

		for (int k = 0; k <= SCAN_SAMPLES; k++) {
			double t = pc->t0 + ((pc->t1 - pc->t0) * k /
					     (double)SCAN_SAMPLES);
			double x = fm_uv(&g->fm, curve_at(&pc->c, t)).v;

			PUSH(*v, *n, *cap, x);
		}
	}
}

// A point of the surface at parameter v, for its iso-line.
static struct vec iso_point(const struct graph *g, double v)
{
	struct uv q = { g->fm.ref.u, v };

	return surf_at(&g->sh->f[g->f].s, q);
}

// The cycles of a face.
struct cycles {
	struct cycle *c;
	size_t n, cap;
};

// Whether p lies in the part of outer cycle c less its holes.
static int in_part(const struct graph *g, const struct cycles *cs, size_t c,
		   struct vec p)
{
	if (!in_cycle(g, &cs->c[c], p)) {
		return 0;
	}
	for (size_t i = 0; i < cs->n; i++) {
		if (cs->c[i].owner == c && in_cycle(g, &cs->c[i], p)) {
			return 0;
		}
	}
	return 1;
}

// A list of the u of points.
struct ulist {
	double *u;
	size_t n, cap;
};

// Adds to us the u of the crossings of cycle c with the iso-line at v
// in the plane pl.
static void crossings(const struct graph *g, const struct cycle *c,
		      struct plane3 pl, double v, struct ulist *us)
{
	for (size_t k = 0; k < c->n; k++) {
		const struct cpiece *pc = &g->p[c->h[k] / 2].pe.p;
		double t[ROOTS_MAX];
		size_t nr = curve_plane(pc, pl, t);

		for (size_t r = 0; r < nr; r++) {
			struct uv q = fm_uv(&g->fm, curve_at(&pc->c, t[r]));

			if (fabs(q.v - v) < SH_EPS) {
				PUSH(us->u, us->n, us->cap, q.u);
			}
		}
	}
}

// The u of the crossings of the cycles of part c with the iso-line at
// v, sorted.
static void scan_us(const struct graph *g, double v, const struct cycles *cs,
		    size_t c, struct ulist *us)
{
	const struct surf *s = &g->sh->f[g->f].s;
	struct vec p = iso_point(g, v);
	struct plane3 pl = iso_plane(s, p);

	for (size_t i = 0; i < cs->n; i++) {
		if (i == c || cs->c[i].owner == c) {
			crossings(g, &cs->c[i], pl, v, us);
		}
	}
	sort_doubles(us->u, us->n);
}

// A point inside part c: on the iso-line through the widest gap of the
// v of its points, in the middle of a stretch inside it.
static struct vec part_point(const struct graph *g, const struct cycles *cs,
			     size_t c)
{
	double *vs = NULL;
	size_t nv = 0;
	size_t cap = 0;
	double best = 0;
	double v = 0;
	struct ulist us = { NULL, 0, 0 };

	for (size_t i = 0; i < cs->n; i++) {
		if (i == c || cs->c[i].owner == c) {
			cycle_vs(g, &cs->c[i], &vs, &nv, &cap);
		}
	}
	sort_doubles(vs, nv);
	for (size_t i = 0; i + 1 < nv; i++) {
		if (vs[i + 1] - vs[i] > best) {
			best = vs[i + 1] - vs[i];
			v = (vs[i] + vs[i + 1]) / 2;
		}
	}
	free(vs);
	scan_us(g, v, cs, c, &us);
	for (size_t i = 0; i + 1 < us.n; i++) {
		struct uv q = { (us.u[i] + us.u[i + 1]) / 2, v };
		struct vec p = surf_at(&g->sh->f[g->f].s, q);

		if (us.u[i + 1] - us.u[i] > SH_EPS && in_part(g, cs, c, p)) {
			free(us.u);
			return p;
		}
	}
	die("internal error: no point inside a part of face %s",
	    g->sh->f[g->f].name);
}

// Finds the smallest outer cycle round each hole cycle.
static void nest(const struct graph *g, const struct cycles *cs)
{
	struct cycle *cy = cs->c;
	size_t n = cs->n;

	for (size_t i = 0; i < n; i++) {
		struct vec m;

		if (cy[i].area > 0) {
			continue;
		}
		m = cycle_point(g, &cy[i]);
		for (size_t j = 0; j < n; j++) {
			if (cy[j].area > 0 && in_cycle(g, &cy[j], m) &&
			    (cy[i].owner == NONE ||
			     cy[j].area < cy[cy[i].owner].area)) {
				cy[i].owner = j;
			}
		}
	}
}

static void add_loop(struct region *r, const struct cycle *c)
{
	PUSH(r->loop0, r->nloops, r->caploops, r->nu);
	for (size_t i = 0; i < c->n; i++) {
		struct puse u = { c->h[i] / 2, !(c->h[i] % 2) };

		PUSH(r->u, r->nu, r->capu, u);
	}
}

// Adds to out the part of outer cycle c, if it lies inside the face.
static void add_region(const struct graph *g, const struct cycles *cs, size_t c,
		       struct parts *out)
{
	struct region r;

	memset(&r, 0, sizeof(r));
	r.inside = part_point(g, cs, c);
	if (!face_has(g->sh, g->f, r.inside)) {
		return;
	}
	add_loop(&r, &cs->c[c]);
	for (size_t j = 0; j < cs->n; j++) {
		if (cs->c[j].owner == c) {
			add_loop(&r, &cs->c[j]);
		}
	}
	PUSH(out->r, out->nr, out->capr, r);
}

// The parts of the graph inside the face.
static void parts_of(struct graph *g, struct parts *out)
{
	struct cycles cs = { NULL, 0, 0 };

	for (size_t h = 0; h < 2 * g->np; h++) {
		if (!g->used[h]) {
			PUSH(cs.c, cs.n, cs.cap, walk(g, h));
		}
	}
	nest(g, &cs);
	for (size_t i = 0; i < cs.n; i++) {
		if (cs.c[i].area > 0) {
			add_region(g, &cs, i, out);
		}
	}
	for (size_t i = 0; i < cs.n; i++) {
		free(cs.c[i].h);
	}
	free(cs.c);
}

// Cuts face f of sh along the n pieces of curves cut, which lie on its
// surface, each from a face boundary or another cut to another, into
// its parts.
void split_face(const struct shape *sh, size_t f, const struct pedge *cut,
		size_t n, struct parts *out)
{
	struct graph g;
	const struct sface *fc = &sh->f[f];

	memset(&g, 0, sizeof(g));
	memset(out, 0, sizeof(*out));
	g.sh = sh;
	g.f = f;
	facemap_init(&g.fm, sh, f);
	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			const struct sedge *e = &sh->e[sh->u[lp->first + k].e];

			(void)vertex_of(&g, sh->v[e->v0]);
			(void)vertex_of(&g, sh->v[e->v1]);
		}
	}
	for (size_t i = 0; i < n; i++) {
		(void)vertex_of(&g, cut[i].a);
		(void)vertex_of(&g, cut[i].b);
	}
	for (size_t l = 0; l < fc->nloops; l++) {
		const struct sloop *lp = &sh->l[fc->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			const struct sedge *e = &sh->e[sh->u[lp->first + k].e];
			struct cpiece c = { e->c, e->t0, e->t1 };

			add_broken(&g, &c);
		}
	}
	for (size_t i = 0; i < n; i++) {
		add_broken(&g, &cut[i].p);
	}
	link(&g);
	parts_of(&g, out);
	for (size_t i = 0; i < g.np; i++) {
		PUSH(out->e, out->ne, out->cape, g.p[i].pe);
	}
	free(g.v);
	free(g.p);
	free(g.out);
	free(g.out0);
	free(g.ang);
	free(g.next);
	free(g.used);
}

void free_parts_of(struct parts *p)
{
	for (size_t i = 0; i < p->nr; i++) {
		free(p->r[i].u);
		free(p->r[i].loop0);
	}
	free(p->r);
	free(p->e);
	memset(p, 0, sizeof(*p));
}
