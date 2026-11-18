// SPDX-License-Identifier: GPL-3.0-only
// interfere.c -- Print the pairs of placed parts whose material overlaps
// Copyright (c) 2026 Jakob Kastelic

// Usage: interfere [-j N] assembly.st
//
// The pairs are counted in N processes, 4 by default; the output is the
// same for any N.
// It prints one line per colliding pair: "NAME NAME points N".  The
// material of each part is sampled on a 1 mm grid in its flat faces at
// three depths, and N counts the samples that collide, each once, up to
// POINTS_MAX: past that the count tells nothing more, and parts that lie
// wholly in each other would take long to count.  A sample collides if
// it lies inside the other part, 1 um or more from its faces.  Parts
// that only touch do not collide.  The sampling can miss an overlap
// thinner than the grid.  A pair with a solid part (place(1)) is checked
// on the exact solids of both instead, sampled on their faces and edges
// (clash.h), and N counts those samples.
//
// A pair that does not collide, with a gap between them over the chord
// of clash_gap, prints "NAME NAME gap G tolerance T" if the tolerances
// of the two may close it: T, the sum of the drifts of the two (drift),
// is more than G, the least distance between their exact solids.

#include "clash.h"
#include "fold.h"
#include "stream.h"
#include "geom.h"
#include "mesh.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define GRID 1.0
#define MARGIN 1e-3

enum {
	POINTS_MAX = 10000, // The most samples counted for a pair.
	WORKERS = 4, // The processes that count the pairs.
	WORKERS_MAX = 64,
	DECIMAL = 10,
	LINE_LEN = 96, // Room for a line of a worker.
	RECORD_WORDS = 64 // The most words of a vary record of a face.
};

// A worker: its number w of nw, and the pipe it writes to.
struct workers {
	size_t w, nw;
	int fd;
};

struct body {
	struct part *p;
	struct box *flat; // Flat box of each face, z unused.
	struct box *world; // World box of each face.
	struct box box; // World box of the part.
	int solid; // Whether sh and cb hold the solid of the part.
	struct shape sh; // Its exact solid in the world (clash.h).
	struct clash_body cb;
	double drift; // The most its tolerances move any of it (drift).
};

// Makes the exact solid of b, if it has none yet.
static void make_solid(struct body *b)
{
	if (b->solid) {
		return;
	}
	part_shape(b->p, &b->sh);
	clash_init(&b->cb, &b->sh, b->p->solid ? 0 : b->p->t);
	b->solid = 1;
}

static void flat_boxes(struct body *b)
{
	const struct part *p = b->p;

	b->flat = xmalloc((p->nfaces + 1) * sizeof(*b->flat));
	for (size_t f = 0; f < p->nfaces; f++) {
		b->flat[f] = box_empty();
	}
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];
		struct vec a = v3(p->verts[e->v0].x, p->verts[e->v0].y, 0);
		struct vec c = v3(p->verts[e->v1].x, p->verts[e->v1].y, 0);

		if (e->lf != NONE) {
			box_add(&b->flat[e->lf], a);
			box_add(&b->flat[e->lf], c);
		}
		if (e->rf != NONE) {
			box_add(&b->flat[e->rf], a);
			box_add(&b->flat[e->rf], c);
		}
	}
}

// Finds the world boxes of the faces and of the part.  The corners of a
// bend zone are inside the bulge of its arc, so the box is larger by the
// bend radius and the thickness.
static void world_boxes(struct body *b)
{
	const struct part *p = b->p;

	b->world = xmalloc((p->nfaces + 1) * sizeof(*b->world));
	b->box = box_empty();
	for (size_t f = 0; f < p->nfaces; f++) {
		const struct box *e = &b->flat[f];
		const struct map *m = &p->maps[p->faces[f]];
		struct box x = box_empty();
		struct box s = *e;

		s.lo.z = 0;
		s.hi.z = -p->t;
		for (unsigned k = 0; k < BOX_CORNERS; k++) {
			struct vec c = box_corner(s, k);

			box_add(&x, fmap(p, f, c.x, c.y, c.z));
		}
		box_pad(&x, m->bend ? m->r + p->t : 0);
		b->world[f] = x;
		box_add(&b->box, x.lo);
		box_add(&b->box, x.hi);
	}
}

// Is the flat point q inside face f, MARGIN or more from its edges and
// holes?
static int deep_in_face(const struct part *p, size_t f, struct vec q)
{
	for (int a = 0; a < 3; a++) {
		for (int k = -1; k <= 1; k += 2) {
			if (!point_in_slab(p, f,
					   add(q, axis_vec(a, k * MARGIN)))) {
				return 0;
			}
		}
	}
	return point_in_slab(p, f, q);
}

// The faces of b whose boxes meet the box of one face of another part,
// and the box of the common parts.
struct near {
	size_t *g;
	struct box *c; // The common box of face g[i] and the other face.
	size_t n;
};

// Is w inside the material of b, MARGIN or more from its faces?  Only
// the faces of nb can hold it.
static int inside(const struct body *b, const struct near *nb, struct vec w)
{
	const struct part *p = b->p;

	for (size_t i = 0; i < nb->n; i++) {
		size_t f = nb->g[i];
		struct vec q;

		if (!box_has(b->world[f], w)) {
			continue;
		}
		if (!finv(p, f, w, &q) || q.z > -MARGIN ||
		    q.z < -p->t + MARGIN) {
			continue;
		}
		if (deep_in_face(p, f, q)) {
			return 1;
		}
	}
	return 0;
}

// Returns the part of the flat box of face f with an image that can lie
// in the box w.
static struct box sample_box(const struct body *a, size_t f, struct box w)
{
	const struct part *p = a->p;
	struct box e = a->flat[f];
	struct box s = box_empty();

	if (p->maps[p->faces[f]].bend) {
		return e;
	}
	for (unsigned k = 0; k < BOX_CORNERS; k++) {
		struct vec q;

		finv(p, f, box_corner(w, k), &q);
		box_add(&s, v3(q.x, q.y, 0));
	}
	e.lo = v3(fmax(e.lo.x, s.lo.x), fmax(e.lo.y, s.lo.y), 0);
	e.hi = v3(fmin(e.hi.x, s.hi.x), fmin(e.hi.y, s.hi.y), 0);
	return e;
}

// The number of grid points lo + GRID (i + 1/2) below hi.
static size_t grid_n(double lo, double hi)
{
	double c = ceil(((hi - lo) / GRID) - MID);

	return c > 0 ? (size_t)c : 0;
}

// Point i of the grid from lo.
static double grid_at(double lo, size_t i)
{
	return lo + (GRID * ((double)i + MID));
}

// The first grid point from lo at or above v, and its count n.
static size_t grid_from(double lo, double v, size_t n)
{
	double c = fmin(floor(((v - lo) / GRID) - MID), (double)n);

	return c > 0 ? (size_t)c : 0;
}

// A face f of a sampled: its map, and the box of the grid of samples
// against all of b.
struct sampled {
	const struct body *a;
	size_t f;
	struct box e;
	struct rigid m;
	int bend;
};

// The grid points of a sampled face, on its grid, that can lie in a
// world box.
struct span2 {
	size_t i0, i1, j0, j1, nx;
};

static struct span2 grid_span(const struct sampled *sf, struct box nb)
{
	struct box e = sf->e;
	struct box r = sample_box(sf->a, sf->f, nb);
	struct span2 s = { 0, 0, 0, 0, grid_n(e.lo.x, e.hi.x) };
	size_t ny = grid_n(e.lo.y, e.hi.y);

	s.i0 = grid_from(e.lo.x, r.lo.x, s.nx);
	s.i1 = grid_from(e.lo.x, r.hi.x + (2 * GRID), s.nx);
	s.j0 = grid_from(e.lo.y, r.lo.y, ny);
	s.j1 = grid_from(e.lo.y, r.hi.y + (2 * GRID), ny);
	return s;
}

// Whether w lies in the common box of a face of nb before face i.
static int seen_before(const struct near *nb, size_t i, struct vec w)
{
	for (size_t h = 0; h < i; h++) {
		if (box_has(nb->c[h], w)) {
			return 1;
		}
	}
	return 0;
}

// The world point of the sample of s at the flat point q.
static struct vec sample_at(const struct sampled *s, struct vec q)
{
	return s->bend ? fmap(s->a->p, s->f, q.x, q.y, q.z) : rig_pt(s->m, q);
}

// Adds to *n the samples of s in the common box of face nb->g[c] that no
// earlier face of nb holds, inside b, up to POINTS_MAX.
static void box_overlap(const struct sampled *s, const struct body *b,
			const struct near *nb, size_t c, unsigned long *n)
{
	const struct part *p = s->a->p;
	struct span2 sp = grid_span(s, nb->c[c]);

	for (size_t j = sp.j0; j < sp.j1 && *n < POINTS_MAX; j++) {
		for (size_t i = sp.i0; i < sp.i1 && *n < POINTS_MAX; i++) {
			for (int k = 1; k <= 3; k++) {
				struct vec q = v3(grid_at(s->e.lo.x, i),
						  grid_at(s->e.lo.y, j),
						  -p->t * k / 4);
				struct vec w = sample_at(s, q);

				if (box_has(nb->c[c], w) &&
				    box_has(b->box, w) &&
				    !seen_before(nb, c, w) &&
				    inside(b, nb, w) &&
				    point_in_slab(p, s->f, q)) {
					(*n)++;
				}
			}
		}
	}
}

// Counts the samples of face f of a inside b, from n on up to
// POINTS_MAX.  The samples lie on the grid of the flat box of f that can
// meet b; each counts once, in the first common box of f with a face of
// nb that holds it.
static unsigned long face_overlap(const struct body *a, size_t f,
				  const struct body *b, const struct near *nb,
				  unsigned long n)
{
	const struct part *p = a->p;
	struct sampled s = { a, f, sample_box(a, f, b->box),
			     map_rig(p, p->faces[f]),
			     p->maps[p->faces[f]].bend };

	for (size_t c = 0; c < nb->n && n < POINTS_MAX; c++) {
		box_overlap(&s, b, nb, c, &n);
	}
	return n;
}

// The common box of a and b, empty if they do not meet.
static int box_common(struct box a, struct box b, struct box *out)
{
	out->lo = v3(fmax(a.lo.x, b.lo.x), fmax(a.lo.y, b.lo.y),
		     fmax(a.lo.z, b.lo.z));
	out->hi = v3(fmin(a.hi.x, b.hi.x), fmin(a.hi.y, b.hi.y),
		     fmin(a.hi.z, b.hi.z));
	return out->lo.x <= out->hi.x && out->lo.y <= out->hi.y &&
	       out->lo.z <= out->hi.z;
}

// The faces of b near the box x, into nb.
static void near_faces(const struct body *b, struct box x, struct near *nb)
{
	nb->n = 0;
	for (size_t g = 0; g < b->p->nfaces; g++) {
		if (box_common(x, b->world[g], &nb->c[nb->n])) {
			nb->g[nb->n++] = g;
		}
	}
}

// Counts the samples of the material of a inside b, each once, from n
// on up to POINTS_MAX.
static unsigned long overlap(const struct body *a, const struct body *b,
			     unsigned long n)
{
	struct near nb = { xmalloc((b->p->nfaces + 1) * sizeof(size_t)),
			   xmalloc((b->p->nfaces + 1) * sizeof(struct box)),
			   0 };

	for (size_t f = 0; f < a->p->nfaces && n < POINTS_MAX; f++) {
		if (!box_meet(a->world[f], b->box)) {
			continue;
		}
		near_faces(b, a->world[f], &nb);
		if (nb.n) {
			n = face_overlap(a, f, b, &nb, n);
		}
	}
	free(nb.g);
	free(nb.c);
	return n;
}

// The most that the end hi of parameter t of folded part p moves a
// corner of its blank on either face, or a hole of it, in mm.
static double blank_drift(const struct part *p, size_t t, int hi)
{
	struct part q;
	double most = 0;

	vary_part(p, t, hi, &q);
	for (size_t f = 0; f < p->nfaces; f++) {
		for (size_t k = p->fe_start[f]; k < p->fe_start[f + 1]; k++) {
			const struct edge *e = &p->edges[p->fe[k]];

			for (unsigned end = 0; end < 4; end++) {
				size_t v = end & 1U ? e->v1 : e->v0;
				double z = end & 2U ? -p->t : 0;
				struct vec a = fmap(p, f, p->verts[v].x,
						    p->verts[v].y, z);
				struct vec b = fmap(&q, f, q.verts[v].x,
						    q.verts[v].y,
						    end & 2U ? -q.t : 0);

				most = fmax(most, len(sub(b, a)));
			}
		}
	}
	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *a = &p->holes[i];
		const struct hole *b = &q.holes[i];

		most = fmax(most, hypot(b->cx - a->cx, b->cy - a->cy) +
					  fabs(b->r - a->r) +
					  fabs(b->rc - a->rc));
	}
	free_variant(&q);
	return most;
}

// The most that the move of a face from surface a to b moves a point of
// it within reach of the origin of a: along its normal, or round, by the
// turn of its axis.
static double face_drift(const struct surf *a, const struct surf *b,
			 double reach)
{
	struct vec d = sub(b->o, a->o);
	double turn = len(sub(b->n, a->n)) * reach;

	if (a->kind == SF_PLANE) {
		return fabs(dot(a->n, d)) + turn;
	}
	if (a->kind == SF_CYL) {
		return len(sub(d, mul(a->n, dot(a->n, d)))) +
		       fabs(b->r - a->r) + turn;
	}
	return len(d) + fabs(b->r - a->r) + (fabs(b->r2 - a->r2) * reach) +
	       fabs(b->r3 - a->r3) + turn;
}

// The most that the vary record rec of a face of solid part p moves a
// point of the face, the solid reach across.
static double solid_drift(const struct part *p, const char *rec, double reach)
{
	char *copy = xstrdup(rec);
	char *w[RECORD_WORDS + 1];
	size_t n = split_words(copy, w, RECORD_WORDS);
	size_t at = 2;
	struct surf a;
	struct surf b;

	if (n < 3 || n > RECORD_WORDS || strcmp(w[0], "face") != 0) {
		die("interfere: part %s: bad vary record %s", p->name, rec);
	}
	a = surf_read(w, n, &at, p->name);
	b = surf_read(w, n, &at, p->name);
	free(copy);
	return face_drift(&a, &b, reach);
}

// The drift of part p: over the parameters of its ranges, the sum of the
// most that either end of each moves any point of it.  Its tolerances
// can move it no further.
static double drift(const struct part *p)
{
	double sum = 0;
	double reach = 0;

	if (p->solid) {
		struct box b = box_empty();

		for (size_t i = 0; i < p->solid->nv; i++) {
			box_add(&b, p->solid->v[i]);
		}
		reach = p->solid->nv ? len(sub(b.hi, b.lo)) : 0;
	}
	for (size_t t = 0; t < p->rg.ntol; t++) {
		double most = 0;

		for (int hi = 0; !p->solid && hi < 2; hi++) {
			most = fmax(most, blank_drift(p, t, hi));
		}
		for (size_t v = 0; p->solid && v < p->rg.nvary; v++) {
			if (p->rg.vary[v].tol == t) {
				most = fmax(most,
					    solid_drift(p, p->rg.vary[v].rec,
							reach));
			}
		}
		sum += most;
	}
	return sum;
}

// A pair of parts whose boxes meet, as far apart as the drifts of the
// two, its count, and its gap, if its tolerances may close it, or else
// HUGE_VAL.
struct pair {
	size_t i, j;
	unsigned long k;
	double gap;
};

// The count of colliding samples of pair q, up to POINTS_MAX, and the
// gap of a pair that does not collide, if their drifts may close it.
static unsigned long pair_count(struct body *bs, struct pair *q)
{
	struct body *a = &bs[q->i];
	struct body *b = &bs[q->j];
	double close = a->drift + b->drift;
	unsigned long k = 0;

	q->gap = HUGE_VAL;
	if (box_meet(a->box, b->box) && (a->p->solid || b->p->solid)) {
		struct clash_pose x = { &a->cb, rig_id() };
		struct clash_pose y = { &b->cb, rig_id() };

		k = clash_count(x, y, POINTS_MAX);
	} else if (box_meet(a->box, b->box)) {
		k = overlap(b, a, overlap(a, b, 0));
	}
	if (!k && close > CLASH_CHORD) {
		struct clash_pose x = { &a->cb, rig_id() };
		struct clash_pose y = { &b->cb, rig_id() };
		double g = 0;

		make_solid(a);
		make_solid(b);
		g = clash_gap(x, y, close);
		if (g > CLASH_CHORD && g < close) {
			q->gap = g;
		}
	}
	return k < POINTS_MAX ? k : POINTS_MAX;
}

// Writes all n bytes of b to fd.
static void write_all(int fd, const char *b, size_t n)
{
	while (n) {
		ssize_t k = write(fd, b, n);

		if (k <= 0) {
			die("interfere: cannot write to the parent");
		}
		b += k;
		n -= (size_t)k;
	}
}

// Counts the pairs w, w + nw, w + 2 nw ... of qs into the pipe of wk, a
// line "INDEX COUNT GAP" for each that collides or whose drifts may
// close its gap.
static void count_some(struct body *bs, struct pair *qs, size_t nq,
		       struct workers wk)
{
	for (size_t q = wk.w; q < nq; q += wk.nw) {
		unsigned long k = pair_count(bs, &qs[q]);

		if (k || isfinite(qs[q].gap)) {
			char line[LINE_LEN];

			xsnprintf(line, "%zu %lu %.17g\n", q, k, qs[q].gap);
			write_all(wk.fd, line, strlen(line));
		}
	}
	(void)close(wk.fd);
}

// Reads the lines "INDEX COUNT GAP" of a worker from fd into qs.
static void read_counts(int fd, struct pair *qs, size_t nq)
{
	char *buf = NULL;
	size_t n = 0;
	size_t cap = 0;
	char chunk[LINE_LEN];
	ssize_t k = 0;
	char *s = NULL;

	while ((k = read(fd, chunk, sizeof(chunk))) > 0) {
		for (ssize_t i = 0; i < k; i++) {
			PUSH(buf, n, cap, chunk[i]);
		}
	}
	PUSH(buf, n, cap, 0);
	(void)close(fd);
	s = buf;
	while (*s) {
		char *end = NULL;
		unsigned long q = strtoul(s, &end, DECIMAL);
		unsigned long c = strtoul(end, &end, DECIMAL);
		double g = strtod(end, &s);

		if (q < nq) {
			qs[q].k = c;
			qs[q].gap = g;
		}
		s += strspn(s, "\n");
	}
	free(buf);
}

// Counts the pairs qs in nw processes, each pair in one, and waits for
// them all.  The bodies are made before, and the processes share them.
static void count_all(struct body *bs, struct pair *qs, size_t nq, size_t nw)
{
	pid_t *pid = xcalloc(nw + 1, sizeof(*pid));
	int *fd = xcalloc(nw + 1, sizeof(*fd));
	int failed = 0;

	(void)fflush(stdout);
	for (size_t w = 0; w < nw; w++) {
		int p[2];

		if (pipe(p) != 0) {
			die("interfere: no pipe");
		}
		pid[w] = fork();
		if (pid[w] < 0) {
			die("interfere: cannot fork");
		}
		if (pid[w] == 0) {
			struct workers wk = { w, nw, p[1] };

			(void)close(p[0]);
			count_some(bs, qs, nq, wk);
			_exit(0);
		}
		(void)close(p[1]);
		fd[w] = p[0];
	}
	for (size_t w = 0; w < nw; w++) {
		int st = 0;

		read_counts(fd[w], qs, nq);
		if (waitpid(pid[w], &st, 0) < 0 || !WIFEXITED(st) ||
		    WEXITSTATUS(st) != 0) {
			failed = 1;
		}
	}
	free(pid);
	free(fd);
	if (failed) {
		die("interfere: a worker failed");
	}
}

// The pairs of the n bodies bs whose boxes meet, into *out, with the
// solids of those that need them made; returns their count.
static size_t list_pairs(struct body *bs, size_t n, struct pair **out)
{
	struct pair *qs = NULL;
	size_t nq = 0;
	size_t cap = 0;

	for (size_t i = 0; i < n; i++) {
		for (size_t j = i + 1; j < n; j++) {
			struct pair q = { i, j, 0, HUGE_VAL };
			struct box a = bs[i].box;

			box_pad(&a, bs[i].drift + bs[j].drift);
			if (!box_meet(a, bs[j].box)) {
				continue;
			}
			if (bs[i].p->solid || bs[j].p->solid) {
				make_solid(&bs[i]);
				make_solid(&bs[j]);
			}
			PUSH(qs, nq, cap, q);
		}
	}
	*out = qs;
	return nq;
}

static void report_pairs(struct body *bs, size_t n, size_t nw)
{
	struct pair *qs = NULL;
	size_t nq = list_pairs(bs, n, &qs);

	if (nw > 1 && nq > 1) {
		count_all(bs, qs, nq, nw < nq ? nw : nq);
	} else {
		for (size_t q = 0; q < nq; q++) {
			qs[q].k = pair_count(bs, &qs[q]);
		}
	}
	for (size_t q = 0; q < nq; q++) {
		if (qs[q].k) {
			printf("%s %s points %lu\n", bs[qs[q].i].p->name,
			       bs[qs[q].j].p->name, qs[q].k);
		} else if (isfinite(qs[q].gap)) {
			printf("%s %s gap %.3f tolerance %.3f\n",
			       bs[qs[q].i].p->name, bs[qs[q].j].p->name,
			       qs[q].gap,
			       bs[qs[q].i].drift + bs[qs[q].j].drift);
		}
	}
	free(qs);
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	struct body *bs = NULL;
	size_t n = 0;

	size_t nw = WORKERS;
	int a = 1;

	if (argc == 4 && strcmp(argv[1], "-j") == 0) {
		char *end = NULL;
		unsigned long v = strtoul(argv[2], &end, DECIMAL);

		if (*end || end == argv[2] || v < 1 || v > WORKERS_MAX) {
			die("interfere: -j takes 1 to %d", WORKERS_MAX);
		}
		nw = v;
		a = 3;
	}
	if (argc != a + 1) {
		die("usage: interfere [-j N] assembly.st");
	}
	n = read_parts(argv[a], &ps);
	bs = xcalloc(n + 1, sizeof(*bs));
	for (size_t i = 0; i < n; i++) {
		bs[i].p = &ps[i];
		flat_boxes(&bs[i]);
		world_boxes(&bs[i]);
		if (ps[i].solid) {
			make_solid(&bs[i]);
			bs[i].box = bs[i].cb.box;
		}
		bs[i].drift = drift(&ps[i]);
	}
	report_pairs(bs, n, nw);
	for (size_t i = 0; i < n; i++) {
		free(bs[i].flat);
		free(bs[i].world);
		if (bs[i].solid) {
			clash_free(&bs[i].cb);
			shape_free(&bs[i].sh);
		}
	}
	free(bs);
	free_parts(ps, n);
	close_stdout();
	return 0;
}
