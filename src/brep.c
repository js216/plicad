// SPDX-License-Identifier: GPL-3.0-only
// brep.c -- Faces, edges and vertices of a folded part
// Copyright (c) 2026 Jakob Kastelic

#include "brep.h"
#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define ALONG 1e-9 // Sine below which a line runs along a bend axis.
#define INSIDE 1e-3 // A step into a face, in mm, to tell its side.
#define PARALLEL 1e-12 // 1 - cosine below which two normals agree.
#define COPLANAR 1e-7 // Distance below which two planes are one, in mm.

static size_t add_vert(struct brep *b, struct vec p)
{
	PUSH(b->v, b->nv, b->capv, p);
	return b->nv - 1;
}

static size_t add_edge(struct brep *b, struct bedge e)
{
	PUSH(b->e, b->ne, b->cape, e);
	return b->ne - 1;
}

static void use(struct brep *b, size_t e, int fwd)
{
	struct buse u = { e, fwd };

	PUSH(b->u, b->nu, b->capu, u);
}

static void end_loop(struct brep *b, size_t first)
{
	struct bloop l = { first, b->nu - first };

	PUSH(b->l, b->nl, b->capl, l);
}

// Where a face comes from: F_TOP .. F_HOLE, and the index of what.
struct origin {
	int from;
	size_t idx;
};

static size_t start_face(struct brep *b, struct bface f, struct origin o)
{
	f.from = o.from;
	f.idx = o.idx;
	f.loop0 = b->nl;
	f.nloops = 0;
	PUSH(b->f, b->nf, b->capf, f);
	return b->nf - 1;
}

static struct bface face_of(int type, struct vec o, struct vec n, struct vec x,
			    double r, int sense)
{
	struct bface f = { type, o, n, x, r, 0, sense, 0, 0, F_TOP, 0 };

	return f;
}

static struct bedge line_edge(size_t v0, size_t v1)
{
	struct bedge e = { v0, v1, C_LINE, { 0, 0, 0 }, { 0, 0, 0 }, 0, 0 };

	return e;
}

// Returns the face that gives an edge its geometry, a slab if possible.
static size_t geo_face(const struct part *p, const struct edge *e)
{
	size_t f = e->lf != NONE ? e->lf : e->rf;

	if (e->rf != NONE && p->maps[p->faces[f]].bend &&
	    !p->maps[p->faces[e->rf]].bend) {
		f = e->rf;
	}
	return f;
}

// The circle of a 2D arc on slab face f, at depth z.
static struct bedge slab_arc(const struct part *p, const struct edge *e,
			     size_t f, struct bedge out, double z)
{
	struct arc_ang a = arc_angles(p, e);

	out.type = C_CIRCLE;
	out.c = fmap(p, f, e->cx, e->cy, z);
	out.n = fnormal(p, f);
	out.r = a.r;
	out.sweep = a.da;
	return out;
}

// The 3D edge of 2D edge e across bend face f: a circle about the axis.
static struct bedge bend_arc(const struct part *p, const struct edge *e,
			     size_t f, struct bedge out, const struct brep *b)
{
	const struct map *m = &p->maps[p->faces[f]];
	struct vec2 s = p->verts[e->v0];
	struct vec2 t = p->verts[e->v1];
	double du = ((t.x - s.x) * m->mx) + ((t.y - s.y) * m->my);
	struct line3 ax = bend_axis(p, p->faces[f]);
	struct vec o = ax.o;
	struct vec a = ax.d;
	struct vec q = b->v[out.v0];

	out.type = C_CIRCLE;
	out.c = add(o, mul(a, dot(sub(q, o), a)));
	out.r = len(sub(q, out.c));
	out.n = mul(a, du * m->ang > 0 ? 1 : -1);
	out.sweep = fabs(du / m->ba * m->ang) * DEG;
	return out;
}

// The 3D edge of 2D edge e at depth z, from the ends of out, a line.  An
// edge along a bend line is a line; an edge across a bend is an arc.
static struct bedge edge3(const struct part *p, const struct edge *e, size_t f,
			  struct bedge out, const struct brep *b, double z)
{
	const struct map *m = &p->maps[p->faces[f]];
	struct vec2 s = p->verts[e->v0];
	struct vec2 t = p->verts[e->v1];
	double dx = t.x - s.x;
	double dy = t.y - s.y;

	if (!m->bend) {
		return e->arc ? slab_arc(p, e, f, out, z) : out;
	}
	if (e->arc) {
		die("%s: an arc inside a bend zone", p->name);
	}
	if (fabs((dx * m->my) - (dy * m->mx)) > ALONG * hypot(dx, dy)) {
		return out;
	}
	return bend_arc(p, e, f, out, b);
}

struct half {
	size_t e;
	int fwd;
	size_t from, to;
};

// Lists the edges of 2D face f, each oriented with the face on its left.
static void push_half(struct half **h, size_t *n, size_t *cap, struct half x)
{
	PUSH(*h, *n, *cap, x);
}

static size_t face_halves(const struct part *p, size_t f, struct half **out)
{
	struct half *h = NULL;
	size_t n = 0;
	size_t cap = 0;

	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];
		struct half fw = { i, 1, e->v0, e->v1 };
		struct half bw = { i, 0, e->v1, e->v0 };

		if (e->lf == f) {
			push_half(&h, &n, &cap, fw);
		}
		if (e->rf == f) {
			push_half(&h, &n, &cap, bw);
		}
	}
	*out = h;
	return n;
}

static size_t next_half(const struct half *h, size_t n, const int *done,
			size_t c)
{
	for (size_t j = 0; j < n; j++) {
		if (!done[j] && h[j].from == h[c].to) {
			return j;
		}
	}
	return NONE;
}

// Finds the boundary loops of 2D face f.  Loop l is out[starts[l] ..
// starts[l + 1]].
static size_t face_loops(const struct part *p, size_t f, struct half **out,
			 size_t **starts)
{
	struct half *h = NULL;
	size_t n = face_halves(p, f, &h);
	struct half *ord = xmalloc((n + 1) * sizeof(*ord));
	size_t *st = xmalloc((n + 2) * sizeof(*st));
	int *done = xcalloc(n + 1, sizeof(*done));
	size_t k = 0;
	size_t nl = 0;

	for (size_t i = 0; i < n; i++) {
		size_t c = i;

		if (done[i]) {
			continue;
		}
		st[nl++] = k;
		while (c != NONE) {
			done[c] = 1;
			ord[k++] = h[c];
			if (next_half(h, n, done, c) == NONE &&
			    h[c].to != h[i].from) {
				die("%s: face %zu has an open boundary",
				    p->name, f);
			}
			c = next_half(h, n, done, c);
		}
	}
	st[nl] = k;
	free(done);
	free(h);
	*out = ord;
	*starts = st;
	return nl;
}

// The vertices and edges of the new solid, by their 2D sources.
struct ctx {
	const struct part *p;
	struct brep *b;
	size_t *tv; // 2D vertex to top vertex.
	size_t *bv; // 2D vertex to bottom vertex.
	size_t *ve; // 2D vertex to vertical edge, NONE until needed.
	size_t *et; // 2D edge to top edge.
	size_t *eb; // 2D edge to bottom edge.
	size_t *ht; // Hole to top circle.
	size_t *hb; // Hole to bottom circle.
	size_t *hm; // Hole to the circle where its countersink ends.
};

static void make_vertices(struct ctx *c)
{
	const struct part *p = c->p;
	size_t *vface = xmalloc((p->nverts + 1) * sizeof(*vface));

	for (size_t i = 0; i < p->nverts; i++) {
		vface[i] = NONE;
	}
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];
		size_t f = geo_face(p, e);

		if (vface[e->v0] == NONE) {
			vface[e->v0] = f;
		}
		if (vface[e->v1] == NONE) {
			vface[e->v1] = f;
		}
	}
	for (size_t i = 0; i < p->nverts; i++) {
		struct vec2 q = p->verts[i];

		if (vface[i] == NONE) {
			die("%s: vertex %zu is on no edge", p->name, i);
		}
		c->tv[i] = add_vert(c->b, fmap(p, vface[i], q.x, q.y, 0));
		c->bv[i] = add_vert(c->b, fmap(p, vface[i], q.x, q.y, -p->t));
		c->ve[i] = NONE;
	}
	free(vface);
}

// Adds the circle of radius r round hole h at depth z.
static size_t hole_circle(struct ctx *c, const struct hole *h, double z,
			  double r)
{
	const struct part *p = c->p;
	struct bedge e = line_edge(0, 0);

	e.type = C_CIRCLE;
	e.n = fnormal(p, h->face);
	e.r = r;
	e.sweep = 2 * PI;
	e.v0 = add_vert(c->b, fmap(p, h->face, h->cx + r, h->cy, z));
	e.v1 = e.v0;
	e.c = fmap(p, h->face, h->cx, h->cy, z);
	return add_edge(c->b, e);
}

static void make_edges(struct ctx *c)
{
	const struct part *p = c->p;

	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];
		size_t f = geo_face(p, e);

		struct bedge top = line_edge(c->tv[e->v0], c->tv[e->v1]);
		struct bedge bot = line_edge(c->bv[e->v0], c->bv[e->v1]);

		c->et[i] = add_edge(c->b, edge3(p, e, f, top, c->b, 0));
		c->eb[i] = add_edge(c->b, edge3(p, e, f, bot, c->b, -p->t));
	}
	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];
		double zm = h->side > 0 ? -h->h : -p->t + h->h;

		c->ht[i] = hole_circle(c, h, 0, h->side > 0 ? h->rc : h->r);
		c->hb[i] = hole_circle(c, h, -p->t, h->side < 0 ? h->rc : h->r);
		c->hm[i] = h->side ? hole_circle(c, h, zm, h->r) : NONE;
	}
}

// The vertical edge at 2D vertex k, top to bottom.
static size_t vertical(struct ctx *c, size_t k)
{
	if (c->ve[k] == NONE) {
		c->ve[k] = add_edge(c->b, line_edge(c->tv[k], c->bv[k]));
	}
	return c->ve[k];
}

// Surface of the top (k = 0) or bottom (k = 1) of 2D face f.
static struct bface cap_surface(const struct part *p, size_t f, int k)
{
	const struct map *m = &p->maps[p->faces[f]];
	double z = k ? -p->t : 0;
	struct line3 ax;
	struct vec o;
	struct vec a;
	double za = 0;

	if (!m->bend) {
		struct vec n = fnormal(p, f);

		return face_of(S_PLANE, fmap(p, f, 0, 0, z), k ? mul(n, -1) : n,
			       fxdir(p, f), 0, 1);
	}
	ax = bend_axis(p, p->faces[f]);
	o = ax.o;
	a = ax.d;
	za = bend_za(p, m);
	return face_of(S_CYLINDER, o, a,
		       unit(rig_dir(map_rig(p, m->parent), v3(0, 0, z - za))),
		       fabs(z - za), (m->ang > 0) == (k != 0) ? 1 : -1);
}

static void cap_faces(struct ctx *c, size_t f)
{
	const struct part *p = c->p;
	struct brep *b = c->b;
	struct half *hs = NULL;
	size_t *st = NULL;
	size_t nl = face_loops(p, f, &hs, &st);

	for (int k = 0; k < 2; k++) {
		struct origin o = { k ? F_BOTTOM : F_TOP, f };
		size_t fi = start_face(b, cap_surface(p, f, k), o);

		for (size_t l = 0; l < nl; l++) {
			size_t first = b->nu;

			if (!k) {
				for (size_t j = st[l]; j < st[l + 1]; j++) {
					use(b, c->et[hs[j].e], hs[j].fwd);
				}
			} else {
				for (size_t j = st[l + 1]; j-- > st[l];) {
					use(b, c->eb[hs[j].e], !hs[j].fwd);
				}
			}
			end_loop(b, first);
			b->f[fi].nloops++;
		}
		for (size_t i = 0; i < p->nholes; i++) {
			size_t first = b->nu;

			if (p->holes[i].face != f) {
				continue;
			}
			use(b, k ? c->hb[i] : c->ht[i], k);
			end_loop(b, first);
			b->f[fi].nloops++;
		}
	}
	free(hs);
	free(st);
}

// An outline edge, oriented from "from" to "to" with the material on its
// left, and the surface of its side face.
struct side {
	size_t e;
	int fwd;
	size_t from, to;
	struct bface s;
};

// The plane of a free straight edge of a bend zone along its axis: it
// holds the edge and the thickness, and faces away from the zone.
static struct bface bend_end(const struct ctx *c, const struct side *d,
			     struct vec top)
{
	const struct part *p = c->p;
	const struct edge *e = &p->edges[d->e];
	size_t f = e->lf != NONE ? e->lf : e->rf;
	struct vec2 a = p->verts[d->from];
	struct vec2 q = p->verts[d->to];
	double l = hypot(q.x - a.x, q.y - a.y);
	struct vec2 in = { -(q.y - a.y) / l, (q.x - a.x) / l };
	struct vec down = unit(sub(c->b->v[c->bv[d->from]], top));
	struct vec along = unit(sub(c->b->v[c->tv[d->to]], top));
	struct vec n = unit(cross(along, down));
	struct vec w =
		fmap(p, f, a.x + (in.x * INSIDE), a.y + (in.y * INSIDE), 0);

	if (dot(n, sub(w, top)) > 0) {
		n = mul(n, -1);
	}
	return face_of(S_PLANE, top, n, down, 0, 1);
}

static struct bface side_surface(const struct ctx *c, const struct side *d)
{
	const struct part *p = c->p;
	const struct edge *e = &p->edges[d->e];
	size_t f = e->lf != NONE ? e->lf : e->rf;
	const struct map *m = &p->maps[p->faces[f]];
	struct vec2 a = p->verts[d->from];
	struct vec2 q = p->verts[d->to];
	double dx = q.x - a.x;
	double dy = q.y - a.y;
	double l = hypot(dx, dy);
	struct rigid r = map_rig(p, m->bend ? m->parent : p->faces[f]);
	struct vec out = rig_dir(r, v3(dy / l, -dx / l, 0));
	struct vec top;

	if (!m->bend && e->arc) {
		return face_of(S_CYLINDER, fmap(p, f, e->cx, e->cy, 0),
			       fnormal(p, f), fxdir(p, f),
			       hypot(a.x - e->cx, a.y - e->cy),
			       d->fwd ? 1 : -1);
	}
	if (!m->bend) {
		return face_of(S_PLANE, c->b->v[c->tv[d->from]], out,
			       rig_dir(r, v3(dx / l, dy / l, 0)), 0, 1);
	}
	top = c->b->v[c->tv[d->from]];
	if (fabs((dx * m->my) - (dy * m->mx)) > ALONG * l) {
		return bend_end(c, d, top);
	}
	return face_of(S_PLANE, top, out,
		       unit(sub(c->b->v[c->bv[d->from]], top)), 0, 1);
}

static bool same_plane(const struct bface *a, const struct bface *b)
{
	return a->type == S_PLANE && b->type == S_PLANE &&
	       dot(a->n, b->n) > 1 - PARALLEL &&
	       fabs(dot(a->n, sub(b->o, a->o))) < COPLANAR;
}

static size_t outline_sides(const struct ctx *c, struct side **out)
{
	const struct part *p = c->p;
	struct side *s = NULL;
	size_t n = 0;
	size_t cap = 0;

	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];
		struct side d;

		if ((e->lf == NONE) == (e->rf == NONE)) {
			continue;
		}
		d.e = i;
		d.fwd = e->lf != NONE;
		d.from = d.fwd ? e->v0 : e->v1;
		d.to = d.fwd ? e->v1 : e->v0;
		d.s = side_surface(c, &d);
		PUSH(s, n, cap, d);
	}
	*out = s;
	return n;
}

// Returns the index of the side of the n sides s that starts at the end
// of side d.
static size_t next_side(const struct side *s, size_t n, const struct side *d)
{
	for (size_t j = 0; j < n; j++) {
		if (s[j].from == d->to) {
			return j;
		}
	}
	die("internal error: open outline");
}

// Makes one face of a run of sides in one plane.  Its loop runs along the
// bottom edges, up at the end, back along the top edges and down.
static void side_face(struct ctx *c, const struct side *s, const size_t *run,
		      size_t n)
{
	struct brep *b = c->b;
	struct origin o = { F_SIDE, s[run[0]].e };
	size_t fi = start_face(b, s[run[0]].s, o);
	size_t first = b->nu;

	for (size_t k = 0; k < n; k++) {
		use(b, c->eb[s[run[k]].e], s[run[k]].fwd);
	}
	use(b, vertical(c, s[run[n - 1]].to), 0);
	for (size_t k = n; k-- > 0;) {
		use(b, c->et[s[run[k]].e], !s[run[k]].fwd);
	}
	use(b, vertical(c, s[run[0]].from), 1);
	end_loop(b, first);
	b->f[fi].nloops = 1;
}

static void side_faces(struct ctx *c)
{
	struct side *s = NULL;
	size_t n = outline_sides(c, &s);
	size_t *nxt = xmalloc((n + 1) * sizeof(*nxt));
	size_t *run = xmalloc((n + 1) * sizeof(*run));
	bool *start = xcalloc(n + 1, sizeof(*start));
	bool *done = xcalloc(n + 1, sizeof(*done));

	for (size_t i = 0; i < n; i++) {
		nxt[i] = next_side(s, n, &s[i]);
	}
	for (size_t i = 0; i < n; i++) {
		if (!same_plane(&s[i].s, &s[nxt[i]].s)) {
			start[nxt[i]] = true;
		}
	}
	for (size_t i = 0; i < n; i++) {
		size_t n_run = 0;

		if (!start[i]) {
			continue;
		}
		for (size_t j = i; n_run == 0 || !start[j]; j = nxt[j]) {
			run[n_run++] = j;
			done[j] = true;
		}
		side_face(c, s, run, n_run);
	}
	for (size_t i = 0; i < n; i++) {
		if (!done[i]) {
			die("%s: an outline lies in one plane", c->p->name);
		}
	}
	free(s);
	free(nxt);
	free(run);
	free(start);
	free(done);
}

// A band of a hole between the circles up (toward the top face) and
// down: a cylinder, or a cone of a countersink that widens up if wide is
// 1 and down if it is -1.
struct band {
	size_t up, down;
	int wide;
};

// Adds the face of band bd of hole i.
static void hole_band(struct ctx *c, size_t i, struct band bd)
{
	size_t up = bd.up;
	size_t down = bd.down;
	int wide = bd.wide;
	struct origin o = { F_HOLE, i };
	const struct part *p = c->p;
	const struct hole *h = &p->holes[i];
	struct brep *b = c->b;
	struct vec n = fnormal(p, h->face);
	double z = wide > 0 ? -h->h : -p->t + h->h;
	struct bface f = face_of(S_CYLINDER, fmap(p, h->face, h->cx, h->cy, 0),
				 n, fxdir(p, h->face), h->r, -1);
	size_t fi = 0;
	size_t first = 0;

	if (wide) {
		f.type = S_CONE;
		f.o = fmap(p, h->face, h->cx, h->cy, z);
		f.n = mul(n, wide);
		f.r2 = (h->rc - h->r) / h->h;
	}
	fi = start_face(b, f, o);
	first = b->nu;
	use(b, up, 1);
	end_loop(b, first);
	first = b->nu;
	use(b, down, 0);
	end_loop(b, first);
	b->f[fi].nloops = 2;
}

static void hole_faces(struct ctx *c)
{
	const struct part *p = c->p;

	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];

		struct band top = { c->ht[i], c->hm[i], h->side > 0 };
		struct band bot = { c->hm[i], c->hb[i], h->side < 0 ? -1 : 0 };
		struct band all = { c->ht[i], c->hb[i], 0 };

		if (h->side) {
			hole_band(c, i, top);
			hole_band(c, i, bot);
		} else {
			hole_band(c, i, all);
		}
	}
}

void brep_build(const struct part *p, struct brep *b)
{
	struct ctx c;
	size_t nv = p->nverts + 1;
	size_t ne = p->nedges + 1;
	size_t nh = p->nholes + 1;

	memset(b, 0, sizeof(*b));
	c.p = p;
	c.b = b;
	c.tv = xmalloc(nv * sizeof(*c.tv));
	c.bv = xmalloc(nv * sizeof(*c.bv));
	c.ve = xmalloc(nv * sizeof(*c.ve));
	c.et = xmalloc(ne * sizeof(*c.et));
	c.eb = xmalloc(ne * sizeof(*c.eb));
	c.ht = xmalloc(nh * sizeof(*c.ht));
	c.hb = xmalloc(nh * sizeof(*c.hb));
	c.hm = xmalloc(nh * sizeof(*c.hm));
	make_vertices(&c);
	make_edges(&c);
	for (size_t f = 0; f < p->nfaces; f++) {
		cap_faces(&c, f);
	}
	side_faces(&c);
	hole_faces(&c);
	free(c.tv);
	free(c.bv);
	free(c.ve);
	free(c.et);
	free(c.eb);
	free(c.ht);
	free(c.hb);
	free(c.hm);
}

void brep_free(struct brep *b)
{
	free(b->v);
	free(b->e);
	free(b->u);
	free(b->l);
	free(b->f);
	memset(b, 0, sizeof(*b));
}
