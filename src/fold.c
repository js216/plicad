// SPDX-License-Identifier: GPL-3.0-only
// fold.c -- The folded part format: a flat subdivision with a map per face
// Copyright (c) 2026 Jakob Kastelic

#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	MAXWORDS = 256,
	DECIMAL = 10 // The base of the indices.
};
#define TURN_EPS 1e-12 // Angles closer than this are one, in radians.
#define UNIT_EPS 1e-9 // How far a unit vector may miss length 1.
#define RADIUS_EPS 1e-6 // How far the ends of an arc may differ in radius.
#define ZERO 1e-13 // Numbers closer to 0 than this are written as 0.

double bend_za(const struct part *p, const struct map *m)
{
	return m->ang > 0 ? m->r : -p->t - m->r;
}

struct rigid map_rig(const struct part *p, size_t map)
{
	return rig_mul(p->place, p->maps[map].m);
}

struct line3 bend_axis(const struct part *p, size_t mi)
{
	const struct map *m = &p->maps[mi];
	struct rigid pr = map_rig(p, m->parent);
	struct line3 ax = { rig_pt(pr, v3(m->qx, m->qy, bend_za(p, m))),
			    rig_dir(pr, v3(m->my, -m->mx, 0)) };

	return ax;
}

struct vec fmap(const struct part *p, size_t f, double x, double y, double z)
{
	const struct map *m = &p->maps[p->faces[f]];
	double u = ((x - m->qx) * m->mx) + ((y - m->qy) * m->my);
	struct vec o = v3(m->qx, m->qy, bend_za(p, m));
	struct rigid rot;
	struct line3 ax;

	if (!m->bend) {
		return rig_pt(map_rig(p, p->faces[f]), v3(x, y, z));
	}
	ax.o = o;
	ax.d = v3(m->my, -m->mx, 0);
	rot = rig_rot(ax, u / m->ba * m->ang);
	return rig_pt(rig_mul(map_rig(p, m->parent), rot),
		      v3(x - (u * m->mx), y - (u * m->my), z));
}

// The angle phi of the point rel about the bend axis, from the start of
// the zone, in (-pi, pi].
static double bend_phi(const struct map *m, struct vec rel)
{
	double wm = (rel.x * m->mx) + (rel.y * m->my);
	double phi = atan2(rel.z, wm) - (m->ang > 0 ? -PI / 2 : PI / 2);

	if (phi > PI) {
		phi -= 2 * PI;
	}
	if (phi < -PI) {
		phi += 2 * PI;
	}
	return phi;
}

int finv(const struct part *p, size_t f, struct vec w, struct vec *flat)
{
	const struct map *m = &p->maps[p->faces[f]];
	struct vec q;
	struct vec rel;
	double za = 0;
	double phi = 0;
	double u = 0;
	double v = 0;

	if (!m->bend) {
		*flat = rig_pt(rig_inv(map_rig(p, p->faces[f])), w);
		return 1;
	}
	q = rig_pt(rig_inv(map_rig(p, m->parent)), w);
	za = bend_za(p, m);
	rel = sub(q, v3(m->qx, m->qy, za));
	phi = bend_phi(m, rel);
	u = phi / (m->ang * DEG) * m->ba;
	v = rel.x * m->my - rel.y * m->mx;
	flat->x = m->qx + m->my * v + m->mx * u;
	flat->y = m->qy - m->mx * v + m->my * u;
	rel.z = hypot((rel.x * m->mx) + (rel.y * m->my), rel.z);
	flat->z = m->ang > 0 ? za - rel.z : za + rel.z;
	return fabs(phi) < PI / 2 + fabs(m->ang) * DEG;
}

struct vec fnormal(const struct part *p, size_t f)
{
	return rig_dir(map_rig(p, p->faces[f]), v3(0, 0, 1));
}

struct vec fxdir(const struct part *p, size_t f)
{
	return rig_dir(map_rig(p, p->faces[f]), v3(1, 0, 0));
}

struct arc_ang arc_angles(const struct part *p, const struct edge *e)
{
	struct vec2 s = p->verts[e->v0];
	struct vec2 t = p->verts[e->v1];
	struct arc_ang a;

	a.a0 = atan2(s.y - e->cy, s.x - e->cx);
	a.da = atan2(t.y - e->cy, t.x - e->cx) - a.a0;
	while (a.da <= TURN_EPS) {
		a.da += 2 * PI;
	}
	a.r = hypot(s.x - e->cx, s.y - e->cy);
	return a;
}

double edge_len2(const struct part *p, const struct edge *e)
{
	struct vec2 s = p->verts[e->v0];
	struct vec2 t = p->verts[e->v1];
	struct arc_ang a;

	if (!e->arc) {
		return hypot(t.x - s.x, t.y - s.y);
	}
	a = arc_angles(p, e);
	return a.r * a.da;
}

// Returns twice the signed area under the edge from v0 to v1, by Green's
// theorem.  An arc adds the segment between its chord and itself.
static double green(const struct part *p, const struct edge *e)
{
	struct vec2 s = p->verts[e->v0];
	struct vec2 t = p->verts[e->v1];
	double g = (s.x * t.y) - (t.x * s.y);

	if (e->arc) {
		struct arc_ang a = arc_angles(p, e);

		g += a.r * a.r * (a.da - sin(a.da));
	}
	return g;
}

double face_area2(const struct part *p, size_t f)
{
	double a = 0;

	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];

		if (e->lf == f) {
			a += green(p, e);
		}
		if (e->rf == f) {
			a -= green(p, e);
		}
	}
	a /= 2;
	for (size_t i = 0; i < p->nholes; i++) {
		if (p->holes[i].face == f) {
			a -= PI * p->holes[i].r * p->holes[i].r;
		}
	}
	return a;
}

// Counts the crossings of the ray from (x, y) toward +x with the piece of
// the arc of edge e, radius r, from height sy at angle b0 to height ty at
// b1, where the arc runs monotone in y.  The end heights are those of the
// vertices or of the top and bottom of the circle, so that the half-open
// rule agrees with the edges that meet there.
// A piece of an arc of radius r: from angle b0 at height y0 to angle b1
// at height y1.
struct arc_piece {
	double r, b0, b1, y0, y1;
};

static int piece_crossings(const struct edge *e, struct arc_piece a,
			   struct vec2 p)
{
	double dx = 0;
	double side = 0;

	if ((a.y0 > p.y) == (a.y1 > p.y)) {
		return 0;
	}
	dx = sqrt(fmax(0, (a.r * a.r) - ((p.y - e->cy) * (p.y - e->cy))));
	side = cos((a.b0 + a.b1) / 2) < 0 ? -1 : 1;
	return e->cx + (side * dx) > p.x;
}

// Counts the crossings of an arc with the ray, piece by piece between its
// highest and lowest points.
static int arc_crossings(const struct part *p, const struct edge *e, double x,
			 double y)
{
	struct arc_ang a = arc_angles(p, e);
	struct arc_piece q = { a.r, a.a0, a.a0, p->verts[e->v0].y, 0 };
	struct vec2 at = { x, y };
	double k = ceil(((a.a0 - (PI / 2)) / PI) + TURN_EPS);
	int n = 0;

	for (int i = 0; i < 3; i++) {
		double b = (PI / 2) + ((k + i) * PI);

		if (b >= a.a0 + a.da - TURN_EPS) {
			break;
		}
		q.b1 = b;
		q.y1 = e->cy + (sin(b) > 0 ? a.r : -a.r);
		n += piece_crossings(e, q, at);
		q.b0 = q.b1;
		q.y0 = q.y1;
	}
	q.b1 = a.a0 + a.da;
	q.y1 = p->verts[e->v1].y;
	return n + piece_crossings(e, q, at);
}

// Counts the crossings of an edge with the ray from (x, y) toward +x.
static int crossings(const struct part *p, const struct edge *e, double x,
		     double y)
{
	struct vec2 s = p->verts[e->v0];
	struct vec2 t = p->verts[e->v1];

	if (e->arc) {
		return arc_crossings(p, e, x, y);
	}
	if ((s.y > y) == (t.y > y)) {
		return 0;
	}
	return s.x + (y - s.y) / (t.y - s.y) * (t.x - s.x) > x;
}

void index_faces(struct part *p)
{
	size_t *count = xcalloc(p->nfaces + 2, sizeof(*count));
	size_t *next = xmalloc((p->nfaces + 1) * sizeof(*next));

	free(p->fe_start);
	free(p->fe);
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];

		if (e->lf != NONE) {
			count[e->lf + 1]++;
		}
		if (e->rf != NONE && e->rf != e->lf) {
			count[e->rf + 1]++;
		}
	}
	for (size_t f = 0; f < p->nfaces; f++) {
		count[f + 1] += count[f];
	}
	p->fe_start = count;
	p->fe = xmalloc((count[p->nfaces] + 1) * sizeof(*p->fe));
	for (size_t f = 0; f < p->nfaces; f++) {
		next[f] = count[f];
	}
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];

		if (e->lf != NONE) {
			p->fe[next[e->lf]++] = i;
		}
		if (e->rf != NONE && e->rf != e->lf) {
			p->fe[next[e->rf]++] = i;
		}
	}
	free(next);
}

int point_in_outline(const struct part *p, size_t f, struct vec2 at)
{
	int n = 0;

	for (size_t k = p->fe_start[f]; k < p->fe_start[f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];

		if ((e->lf == f) != (e->rf == f)) {
			n += crossings(p, e, at.x, at.y);
		}
	}
	return n % 2;
}

double hole_radius(const struct part *p, const struct hole *h, double z)
{
	double d = h->side > 0 ? -z : z + p->t; // Below the countersunk face.

	if (!h->side || !(d < h->h)) {
		return h->r;
	}
	return h->rc - ((h->rc - h->r) * fmax(0, d) / h->h);
}

int point_in_face(const struct part *p, size_t f, struct vec2 at)
{
	double x = at.x;
	double y = at.y;

	if (!point_in_outline(p, f, at)) {
		return 0;
	}
	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];

		double dx = x - h->cx;
		double dy = y - h->cy;

		if (h->face == f && (dx * dx) + (dy * dy) < h->r * h->r) {
			return 0;
		}
	}
	return 1;
}

int point_in_slab(const struct part *p, size_t f, struct vec q)
{
	struct vec2 at = { q.x, q.y };

	if (!point_in_outline(p, f, at)) {
		return 0;
	}
	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];

		double dx = q.x - h->cx;
		double dy = q.y - h->cy;
		double r = 0;

		if (h->face != f) {
			continue;
		}
		r = hole_radius(p, h, q.z);
		if ((dx * dx) + (dy * dy) < r * r) {
			return 0;
		}
	}
	return 1;
}

static void check_map(const struct part *p, size_t i, const char *where)
{
	const struct map *m = &p->maps[i];

	if (!m->bend) {
		return;
	}
	if (m->parent >= p->nmaps || p->maps[m->parent].bend) {
		die("%s: the parent of bend map %zu must be a slab map", where,
		    i);
	}
	if (!(m->ba > 0) || !(m->r > 0) || !(fabs(m->ang) > 0) ||
	    !(fabs(m->ang) < HALF_TURN)) {
		die("%s: bend map %zu has a bad width, radius or angle", where,
		    i);
	}
	if (fabs(hypot(m->mx, m->my) - 1) > UNIT_EPS) {
		die("%s: bend map %zu has no unit direction", where, i);
	}
}

static void check_edge(const struct part *p, size_t i, const char *where)
{
	const struct edge *e = &p->edges[i];
	struct vec2 s;
	struct vec2 t;

	if (e->v0 >= p->nverts || e->v1 >= p->nverts) {
		die("%s: edge %zu has no such vertex", where, i);
	}
	if ((e->lf != NONE && e->lf >= p->nfaces) ||
	    (e->rf != NONE && e->rf >= p->nfaces)) {
		die("%s: edge %zu has no such face", where, i);
	}
	if (e->lf == e->rf) {
		die("%s: edge %zu has the same face, or none, on both sides",
		    where, i);
	}
	s = p->verts[e->v0];
	t = p->verts[e->v1];
	if (!e->arc && !(hypot(t.x - s.x, t.y - s.y) > RADIUS_EPS)) {
		die("%s: edge %zu has no length", where, i);
	}
	if (e->arc && fabs(hypot(s.x - e->cx, s.y - e->cy) -
			   hypot(t.x - e->cx, t.y - e->cy)) > RADIUS_EPS) {
		die("%s: arc %zu has ends at different radii", where, i);
	}
}

void check_part(const struct part *p, const char *where)
{
	if (!(p->t > 0)) {
		die("%s: the thickness must be positive", where);
	}
	if (!p->nfaces) {
		die("%s: part %s has no faces", where, p->name);
	}
	for (size_t i = 0; i < p->nmaps; i++) {
		check_map(p, i, where);
	}
	for (size_t i = 0; i < p->nfaces; i++) {
		if (p->faces[i] >= p->nmaps) {
			die("%s: face %zu has no such map", where, i);
		}
	}
	for (size_t i = 0; i < p->nedges; i++) {
		check_edge(p, i, where);
	}
	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];

		if (h->face >= p->nfaces || !(h->r > 0)) {
			die("%s: hole %zu is on no face or has no size", where,
			    i);
		}
		if (h->side &&
		    (!(h->rc > h->r) || !(h->h > 0) || !(h->h < p->t))) {
			die("%s: the countersink of hole %zu needs a radius "
			    "above the hole and a depth inside the sheet",
			    where, i);
		}
	}
	for (size_t i = 0; i < p->nfaces; i++) {
		if (!(face_area2(p, i) > 0)) {
			die("%s: face %zu runs clockwise or has no area", where,
			    i);
		}
	}
}

// The reader of a fold file: its path and line, and the words of the line.
struct reader {
	const char *path;
	unsigned line;
	char *w[MAXWORDS];
	size_t n;
	size_t k; // The next word.
};

static const char *word(struct reader *r)
{
	if (r->k >= r->n) {
		die("%s:%u: too few fields", r->path, r->line);
	}
	return r->w[r->k++];
}

static double num(struct reader *r)
{
	const char *t = word(r);
	char *end = NULL;
	double v = strtod(t, &end);

	if (*end || end == t || !num_ok(v)) {
		die("%s:%u: bad number %s, or beyond 1e6", r->path, r->line, t);
	}
	return v;
}

static size_t index_of(struct reader *r)
{
	const char *t = word(r);
	char *end = NULL;
	unsigned long v = 0;

	if (strcmp(t, "-") == 0) {
		return NONE;
	}
	v = strtoul(t, &end, DECIMAL);
	if (*end || end == t || t[0] == '-') {
		die("%s:%u: bad index %s", r->path, r->line, t);
	}
	return v;
}

// Reads the index of a record; it must count the records of its kind.
static void own_index(struct reader *r, size_t count)
{
	if (index_of(r) != count) {
		die("%s:%u: the index must be %zu", r->path, r->line, count);
	}
}

static void rig_read(struct reader *r, struct rigid *m)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			m->r[i][j] = num(r);
		}
	}
	m->t.x = num(r);
	m->t.y = num(r);
	m->t.z = num(r);
}

static void read_map(struct part *p, struct reader *r)
{
	struct map m = { 0, rig_id(), NONE, 0, 0, 0, 0, 0, 0, 0 };
	const char *kind = NULL;

	own_index(r, p->nmaps);
	kind = word(r);
	if (strcmp(kind, "rigid") == 0) {
		rig_read(r, &m.m);
	} else if (strcmp(kind, "bend") == 0) {
		m.bend = 1;
		m.parent = index_of(r);
		m.qx = num(r);
		m.qy = num(r);
		m.mx = num(r);
		m.my = num(r);
		m.ba = num(r);
		m.ang = num(r);
		m.r = num(r);
	} else {
		die("%s:%u: a map is rigid or bend", r->path, r->line);
	}
	PUSH(p->maps, p->nmaps, p->capmaps, m);
}

static void read_edge(struct part *p, struct reader *r)
{
	struct edge e = { 0, 0, NONE, NONE, 0, 0, 0 };

	own_index(r, p->nedges);
	e.v0 = index_of(r);
	e.v1 = index_of(r);
	e.lf = index_of(r);
	e.rf = index_of(r);
	if (r->k < r->n) {
		if (strcmp(word(r), "arc") != 0) {
			die("%s:%u: expected arc", r->path, r->line);
		}
		e.arc = 1;
		e.cx = num(r);
		e.cy = num(r);
	}
	PUSH(p->edges, p->nedges, p->capedges, e);
}

static void read_face(struct part *p, struct reader *r)
{
	size_t m = 0;

	own_index(r, p->nfaces);
	m = index_of(r);
	PUSH(p->faces, p->nfaces, p->capfaces, m);
}

static void read_vertex(struct part *p, struct reader *r)
{
	struct vec2 v;

	own_index(r, p->nverts);
	v.x = num(r);
	v.y = num(r);
	PUSH(p->verts, p->nverts, p->capverts, v);
}

static void read_hole(struct part *p, struct reader *r)
{
	struct hole h;

	h.face = index_of(r);
	h.cx = num(r);
	h.cy = num(r);
	h.r = num(r);
	h.side = 0;
	h.rc = h.r;
	h.h = 0;
	if (r->k < r->n) {
		const char *k = word(r);
		const char *s = NULL;

		if (strcmp(k, "csink") != 0) {
			die("%s:%u: expected csink RC H top|bottom", r->path,
			    r->line);
		}
		h.rc = num(r);
		h.h = num(r);
		s = word(r);
		h.side = strcmp(s, "top") == 0 ? 1 : -1;
		if (strcmp(s, "top") != 0 && strcmp(s, "bottom") != 0) {
			die("%s:%u: a countersink is at top or bottom", r->path,
			    r->line);
		}
	}
	PUSH(p->holes, p->nholes, p->capholes, h);
}

static void read_colour(struct part *p, struct reader *r)
{
	if (p->colour) {
		die("%s:%u: a second colour", r->path, r->line);
	}
	p->colour = xstrdup(word(r));
	for (int i = 0; i < 3; i++) {
		p->rgb[i] = num(r);
	}
	p->rough = num(r);
	for (int i = 0; i < 3; i++) {
		if (!(p->rgb[i] >= 0 && p->rgb[i] <= 1)) {
			die("%s:%u: a colour runs from 0 to 1", r->path,
			    r->line);
		}
	}
	if (!(p->rough >= 0 && p->rough <= 1)) {
		die("%s:%u: a roughness runs from 0 to 1", r->path, r->line);
	}
}

static void read_record(struct part *p, struct reader *r, const char *k)
{
	if (strcmp(k, "t") == 0) {
		p->t = num(r);
	} else if (strcmp(k, "place") == 0) {
		rig_read(r, &p->place);
	} else if (strcmp(k, "map") == 0) {
		read_map(p, r);
	} else if (strcmp(k, "face") == 0) {
		read_face(p, r);
	} else if (strcmp(k, "vertex") == 0) {
		read_vertex(p, r);
	} else if (strcmp(k, "edge") == 0) {
		read_edge(p, r);
	} else if (strcmp(k, "hole") == 0) {
		read_hole(p, r);
	} else if (strcmp(k, "colour") == 0) {
		read_colour(p, r);
	} else {
		die("%s:%u: unknown record %s", r->path, r->line, k);
	}
	if (r->k != r->n) {
		die("%s:%u: too many fields", r->path, r->line);
	}
}

static struct part new_part(const char *name)
{
	struct part z;

	memset(&z, 0, sizeof(z));
	z.name = xstrdup(name);
	z.place = rig_id();
	return z;
}

static void end_part(struct part *p, const struct reader *r)
{
	char where[WHERE_LEN];

	xsnprintf(where, "%s:%u", r->path, r->line);
	check_part(p, where);
	index_faces(p);
}

// Splits line l of r into its fields and returns the first one, or NULL
// for a blank line or a comment.
static const char *split_record(char *l, struct reader *r)
{
	r->line++;
	r->n = split_words(l, r->w, MAXWORDS);
	r->k = 0;
	if (r->n > MAXWORDS) {
		die("%s:%u: too many fields", r->path, r->line);
	}
	if (!r->n || r->w[0][0] == '#') {
		return NULL;
	}
	return word(r);
}

// Reads record k of the open part p, or dies if no part is open.
// Returns whether the part stays open.
static int in_part(struct part *p, struct reader *r, const char *k)
{
	if (!p) {
		die("%s:%u: %s outside a part", r->path, r->line, k);
	}
	if (strcmp(k, "end") == 0) {
		end_part(p, r);
		return 0;
	}
	read_record(p, r, k);
	return 1;
}

size_t read_fold(const char *path, struct part **out)
{
	char *text = read_file(path);
	char *rest = text;
	char *l = NULL;
	struct part *ps = NULL;
	size_t np = 0;
	size_t cap = 0;
	int open = 0;
	struct reader r = { path, 0, { NULL }, 0, 0 };

	while ((l = next_line(&rest)) != NULL) {
		const char *k = split_record(l, &r);

		if (!k) {
			continue;
		}
		if (strcmp(k, "part") == 0) {
			if (open) {
				die("%s:%u: missing end", path, r.line);
			}
			PUSH(ps, np, cap, new_part(word(&r)));
			open = 1;
		} else {
			open = in_part(open ? &ps[np - 1] : NULL, &r, k);
		}
	}
	free(text);
	if (open) {
		die("%s: missing end", path);
	}
	*out = ps;
	return np;
}

void clear_part(struct part *p)
{
	free(p->name);
	free(p->maps);
	free(p->faces);
	free(p->verts);
	free(p->edges);
	free(p->holes);
	free(p->fe_start);
	free(p->fe);
	free(p->colour);
	memset(p, 0, sizeof(*p));
}

void free_parts(struct part *ps, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		clear_part(&ps[i]);
	}
	free(ps);
}

static void pnum(double x)
{
	printf(" %.12g", fabs(x) < ZERO ? 0.0 : x);
}

static void pidx(size_t i)
{
	if (i == NONE) {
		printf(" -");
	} else {
		printf(" %zu", i);
	}
}

static void rig_write(const struct rigid *r)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			pnum(r->r[i][j]);
		}
	}
	pnum(r->t.x);
	pnum(r->t.y);
	pnum(r->t.z);
}

static void write_map(size_t i, const struct map *m)
{
	printf("map %zu", i);
	if (!m->bend) {
		printf(" rigid");
		rig_write(&m->m);
	} else {
		printf(" bend");
		pidx(m->parent);
		pnum(m->qx);
		pnum(m->qy);
		pnum(m->mx);
		pnum(m->my);
		pnum(m->ba);
		pnum(m->ang);
		pnum(m->r);
	}
	printf("\n");
}

static void write_edge(size_t i, const struct edge *e)
{
	printf("edge %zu %zu %zu", i, e->v0, e->v1);
	pidx(e->lf);
	pidx(e->rf);
	if (e->arc) {
		printf(" arc");
		pnum(e->cx);
		pnum(e->cy);
	}
	printf("\n");
}

void write_fold(const struct part *p)
{
	printf("part %s\nt", p->name);
	pnum(p->t);
	printf("\nplace");
	rig_write(&p->place);
	printf("\n");
	if (p->colour) {
		printf("colour %s", p->colour);
		for (int i = 0; i < 3; i++) {
			pnum(p->rgb[i]);
		}
		pnum(p->rough);
		printf("\n");
	}
	for (size_t i = 0; i < p->nmaps; i++) {
		write_map(i, &p->maps[i]);
	}
	for (size_t i = 0; i < p->nfaces; i++) {
		printf("face %zu %zu\n", i, p->faces[i]);
	}
	for (size_t i = 0; i < p->nverts; i++) {
		printf("vertex %zu", i);
		pnum(p->verts[i].x);
		pnum(p->verts[i].y);
		printf("\n");
	}
	for (size_t i = 0; i < p->nedges; i++) {
		write_edge(i, &p->edges[i]);
	}
	for (size_t i = 0; i < p->nholes; i++) {
		printf("hole %zu", p->holes[i].face);
		pnum(p->holes[i].cx);
		pnum(p->holes[i].cy);
		pnum(p->holes[i].r);
		if (p->holes[i].side) {
			printf(" csink");
			pnum(p->holes[i].rc);
			pnum(p->holes[i].h);
			printf(" %s", p->holes[i].side > 0 ? "top" : "bottom");
		}
		printf("\n");
	}
	printf("end\n");
}
