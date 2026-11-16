// SPDX-License-Identifier: GPL-3.0-only
// coat.c -- The triangles of a folded part split along its coats
// Copyright (c) 2026 Jakob Kastelic

#include "coat.h"
#include "brep.h"
#include "fold.h"
#include "geom.h"
#include "mesh.h"
#include "util.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ON_LINE 1e-9 // A point this near a line lies on it, in mm.
#define HASH_MUL 0x9e3779b97f4a7c15ULL // Spreads the bits of a key.

enum {
	PIECE_MAX = 64, // The most corners of a piece of a triangle.
	HASH_MIN = 64 // The least room of the table of cut points.
};

// A line of the flat blank: the points where a x + b y = c.
struct cline {
	double a, b, c;
};

// The cut of edge lo..hi of the mesh by line k, at vertex id.
struct cut_at {
	size_t lo, hi, k, id;
	int used;
};

// The work: the part, the mesh, the lines, the flat point of each vertex
// once known, and the table of the cuts.
struct work {
	const struct part *p;
	struct mesh *m;
	struct cline *ln;
	size_t nln;
	struct vec *flat;
	unsigned char *known;
	size_t nflat;
	struct cut_at *cut;
	size_t ncut, capcut;
};

// A piece of a triangle: its corners, vertices of the mesh.
struct piece {
	size_t v[PIECE_MAX];
	size_t n;
};

// The face of the flat blank of a face of the solid, and its kind.
struct origin {
	size_t f;
	int from;
};

static struct origin origin_of(const struct part *p, struct face_from x)
{
	struct origin o = { x.idx, x.from };

	if (x.from == F_SIDE) {
		const struct edge *e = &p->edges[x.idx];

		o.f = e->lf != NONE ? e->lf : e->rf;
	} else if (x.from == F_HOLE) {
		o.f = p->holes[x.idx].face;
	}
	return o;
}

static void grow_flat(struct work *w)
{
	if (w->known && w->nflat >= w->m->nv) {
		return;
	}
	w->flat = xrealloc(w->flat, (w->m->nv + 1) * sizeof(*w->flat));
	w->known = xrealloc(w->known, w->m->nv + 1);
	memset(w->known + w->nflat, 0, w->m->nv - w->nflat);
	w->nflat = w->m->nv;
}

// The flat point of vertex v, on face o if not known yet.
static struct vec flat_of(struct work *w, size_t v, struct origin o)
{
	grow_flat(w);
	if (!w->known[v]) {
		struct vec q = v3(0, 0, 0);

		(void)finv(w->p, o.f, w->m->v[v], &q);
		w->flat[v] = q;
		w->known[v] = 1;
	}
	return w->flat[v];
}

static double side_of(const struct cline *l, struct vec q)
{
	return (l->a * q.x) + (l->b * q.y) - l->c;
}

static size_t slot_of(const struct work *w, size_t lo, size_t hi, size_t k)
{
	uint64_t h = (lo * HASH_MUL) ^ (hi << 1U) ^ (k * (HASH_MUL >> 3U));

	return h % w->capcut;
}

static void grow_cuts(struct work *w)
{
	struct work old = *w;

	w->capcut = w->capcut ? 2 * w->capcut : HASH_MIN;
	w->cut = xcalloc(w->capcut, sizeof(*w->cut));
	w->ncut = 0;
	for (size_t i = 0; i < old.capcut; i++) {
		if (old.cut[i].used) {
			size_t s = slot_of(w, old.cut[i].lo, old.cut[i].hi,
					   old.cut[i].k);

			while (w->cut[s].used) {
				s = (s + 1) % w->capcut;
			}
			w->cut[s] = old.cut[i];
			w->ncut++;
		}
	}
	free(old.cut);
}

// An edge of a piece, from vertex a to vertex b.
struct seg {
	size_t a, b;
};

// The vertex where line k cuts the edge e of a piece on face o, made
// once for the edge whichever way it runs.
static size_t cut_vertex(struct work *w, struct seg e, size_t k,
			 struct origin o)
{
	size_t lo = e.a < e.b ? e.a : e.b;
	size_t hi = e.a < e.b ? e.b : e.a;
	size_t s = 0;
	struct vec fl;
	struct vec fh;
	struct vec q;
	double dl = 0;
	double t = 0;
	struct vec at;

	if (2 * (w->ncut + 1) > w->capcut) {
		grow_cuts(w);
	}
	s = slot_of(w, lo, hi, k);
	while (w->cut[s].used) {
		const struct cut_at *c = &w->cut[s];

		if (c->lo == lo && c->hi == hi && c->k == k) {
			return c->id;
		}
		s = (s + 1) % w->capcut;
	}
	fl = flat_of(w, lo, o);
	fh = flat_of(w, hi, o);
	dl = side_of(&w->ln[k], fl);
	t = dl / (dl - side_of(&w->ln[k], fh));
	q = add(fl, mul(sub(fh, fl), t));
	if (o.from == F_NEAR || o.from == F_FAR) {
		at = fmap(w->p, o.f, q.x, q.y, o.from == F_NEAR ? 0 : -w->p->t);
	} else {
		at = add(w->m->v[lo], mul(sub(w->m->v[hi], w->m->v[lo]), t));
	}
	PUSH(w->m->v, w->m->nv, w->m->capv, at);
	grow_flat(w);
	w->flat[w->m->nv - 1] = q;
	w->known[w->m->nv - 1] = 1;
	w->cut[s].lo = lo;
	w->cut[s].hi = hi;
	w->cut[s].k = k;
	w->cut[s].id = w->m->nv - 1;
	w->cut[s].used = 1;
	w->ncut++;
	return w->m->nv - 1;
}

// Cuts piece x by line k into the pieces on its two sides, in out[0]
// and out[1]; returns 0 if the line misses it, which then stays whole.
static int cut_piece(struct work *w, const struct piece *x, size_t k,
		     struct origin o, struct piece *out)
{
	double d[PIECE_MAX];
	int below = 0;
	int above = 0;

	for (size_t i = 0; i < x->n; i++) {
		d[i] = side_of(&w->ln[k], flat_of(w, x->v[i], o));
		below = below || d[i] < -ON_LINE;
		above = above || d[i] > ON_LINE;
	}
	if (!below || !above) {
		return 0;
	}
	out[0].n = out[1].n = 0;
	for (size_t i = 0; i < x->n; i++) {
		size_t j = (i + 1) % x->n;

		if (d[i] <= ON_LINE && out[0].n < PIECE_MAX) {
			out[0].v[out[0].n++] = x->v[i];
		}
		if (d[i] >= -ON_LINE && out[1].n < PIECE_MAX) {
			out[1].v[out[1].n++] = x->v[i];
		}
		if ((d[i] < -ON_LINE && d[j] > ON_LINE) ||
		    (d[i] > ON_LINE && d[j] < -ON_LINE)) {
			struct seg e = { x->v[i], x->v[j] };
			size_t c = cut_vertex(w, e, k, o);

			if (out[0].n < PIECE_MAX && out[1].n < PIECE_MAX) {
				out[0].v[out[0].n++] = c;
				out[1].v[out[1].n++] = c;
			}
		}
	}
	return 1;
}

// Whether the flat point q lies in the polygon of coat c.
static int in_coat(const struct coat *c, struct vec q)
{
	int in = 0;

	for (size_t i = 0, j = c->n - 1; i < c->n; j = i++) {
		struct vec2 a = c->pt[i];
		struct vec2 b = c->pt[j];

		if ((a.y > q.y) != (b.y > q.y) &&
		    q.x < a.x + ((b.x - a.x) * (q.y - a.y) / (b.y - a.y))) {
			in = !in;
		}
	}
	return in;
}

// The coat whose material piece x of face o takes, or NONE.
static size_t coat_of(struct work *w, const struct piece *x, struct origin o)
{
	struct vec m = v3(0, 0, 0);
	size_t fin = NONE;

	for (size_t i = 0; i < x->n; i++) {
		m = add(m, flat_of(w, x->v[i], o));
	}
	m = mul(m, 1.0 / (double)x->n);
	for (size_t i = 0; i < w->p->ncoats; i++) {
		const struct coat *c = &w->p->coats[i];
		int on = (o.from == F_NEAR && c->side > 0) ||
			 (o.from == F_FAR && c->side < 0) ||
			 ((o.from == F_SIDE || o.from == F_HOLE) && c->edges);

		if (on && in_coat(c, m)) {
			fin = i;
		}
	}
	return fin;
}

// The lines of the sides of every coat.
static void coat_lines(struct work *w)
{
	size_t cap = 0;

	for (size_t i = 0; i < w->p->ncoats; i++) {
		const struct coat *c = &w->p->coats[i];

		for (size_t k = 0; k < c->n; k++) {
			struct vec2 a = c->pt[k];
			struct vec2 b = c->pt[(k + 1) % c->n];
			struct cline l = { -(b.y - a.y), b.x - a.x, 0 };

			l.c = (l.a * a.x) + (l.b * a.y);
			if (fabs(l.a) > 0 || fabs(l.b) > 0) {
				PUSH(w->ln, w->nln, cap, l);
			}
		}
	}
}

// The triangles made so far, and the coat of each.
struct tris {
	struct mesh_tri *t;
	size_t *fin;
	size_t n, cap;
};

// The face of a piece and the coat it takes.
struct piece_of {
	size_t face, coat;
};

// Adds the triangles of piece x, of the face and with the coat of of.
static void add_piece(struct tris *out, const struct piece *x,
		      struct piece_of of)
{
	for (size_t k = 1; k + 1 < x->n; k++) {
		struct mesh_tri t = { { x->v[0], x->v[k], x->v[k + 1] },
				      of.face };
		size_t cap = out->cap;

		PUSH(out->t, out->n, out->cap, t);
		if (cap != out->cap) {
			out->fin = xrealloc(out->fin,
					    out->cap * sizeof(*out->fin));
		}
		out->fin[out->n - 1] = of.coat;
	}
}

// Cuts triangle t of the mesh into pieces by every line, and adds the
// triangles of the pieces, with their coats, to out.
static void cut_tri(struct work *w, struct mesh_tri t, struct origin o,
		    struct tris *out)
{
	struct piece *ps = xmalloc(sizeof(*ps) * 2);
	size_t np = 1;
	size_t capp = 2;

	ps[0].n = 3;
	memcpy(ps[0].v, t.v, sizeof(t.v));
	for (size_t k = 0; k < w->nln; k++) {
		for (size_t i = 0, m = np; i < m; i++) {
			struct piece two[2];

			if (cut_piece(w, &ps[i], k, o, two)) {
				ps[i] = two[0];
				PUSH(ps, np, capp, two[1]);
			}
		}
	}
	for (size_t i = 0; i < np; i++) {
		add_piece(out, &ps[i],
			  (struct piece_of){ t.face, coat_of(w, &ps[i], o) });
	}
	free(ps);
}

void coat_mesh(const struct part *p, const struct face_from *from,
	       struct mesh *m, size_t **fin)
{
	struct work w;
	struct tris out = { NULL, NULL, 0, 0 };

	memset(&w, 0, sizeof(w));
	w.p = p;
	w.m = m;
	coat_lines(&w);
	for (size_t i = 0; i < m->nt; i++) {
		struct origin o = origin_of(p, from[m->t[i].face]);

		cut_tri(&w, m->t[i], o, &out);
	}
	free(m->t);
	m->t = out.t;
	m->nt = out.n;
	m->capt = out.cap;
	*fin = out.fin;
	free(w.ln);
	free(w.flat);
	free(w.known);
	free(w.cut);
}
