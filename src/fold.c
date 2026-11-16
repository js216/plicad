// SPDX-License-Identifier: GPL-3.0-only
// fold.c -- The folded part format: a flat subdivision with a map per face
// Copyright (c) 2026 Jakob Kastelic

#include "fold.h"
#include "expr.h"
#include "shape.h"
#include "geom.h"
#include "util.h"
#include <ctype.h>
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
#define DRAFT_MAX 45.0 // A least draft is below this, in degrees.
#define VDI_MAX 45.0 // The roughest grade of VDI 3400.

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
		if (p->holes[i].face == f && !(p->holes[i].blind > 0)) {
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

	if (h->blind > 0) {
		double in = h->bside > 0 ? -z :
					   z + p->t; // Below its open face.

		return in < h->blind ? h->r : 0;
	}
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
	for (size_t i = 0; i < p->nins; i++) {
		if (p->ins[i].hole >= p->nholes) {
			die("%s: insert %s has no such hole", where,
			    p->ins[i].name);
		}
	}
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
		if (h->blind > 0 && (h->side || !(h->blind < p->t))) {
			die("%s: blind hole %zu needs a depth inside the "
			    "sheet and no countersink",
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

// The reader of a record of a part: its place in the stream, as
// "file:line", and the words of the line.
struct reader {
	const char *where;
	char *w[MAXWORDS];
	size_t n;
	size_t k; // The next word.
};

static const char *word(struct reader *r)
{
	if (r->k >= r->n) {
		die("%s: too few fields", r->where);
	}
	return r->w[r->k++];
}

static double num(struct reader *r)
{
	const char *t = word(r);
	char *end = NULL;
	double v = strtod(t, &end);

	if (*end || end == t || !num_ok(v)) {
		die("%s: bad number %s, or beyond 1e6", r->where, t);
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
		die("%s: bad index %s", r->where, t);
	}
	return v;
}

// Reads the index of a record; it must count the records of its kind.
static void own_index(struct reader *r, size_t count)
{
	if (index_of(r) != count) {
		die("%s: the index must be %zu", r->where, count);
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
		die("%s: a map is rigid or bend", r->where);
	}
	PUSH(p->maps, p->nmaps, p->capmaps, m);
}

size_t part_feature(struct part *p, const char *name)
{
	for (size_t i = 0; i < p->nfeats; i++) {
		if (strcmp(p->feats[i].s, name) == 0) {
			return i;
		}
	}
	{
		struct str f = { xstrdup(name) };

		PUSH(p->feats, p->nfeats, p->capfeats, f);
	}
	return p->nfeats - 1;
}

// Reads the feature of a record, "by NAME", if the record goes on.
static size_t read_by(struct part *p, struct reader *r)
{
	if (r->k == r->n) {
		return NONE;
	}
	if (strcmp(word(r), "by") != 0) {
		die("%s: expected by NAME", r->where);
	}
	return part_feature(p, word(r));
}

static void read_edge(struct part *p, struct reader *r)
{
	struct edge e = { 0, 0, NONE, NONE, 0, 0, 0, NONE };

	own_index(r, p->nedges);
	e.v0 = index_of(r);
	e.v1 = index_of(r);
	e.lf = index_of(r);
	e.rf = index_of(r);
	if (r->k < r->n && strcmp(r->w[r->k], "arc") == 0) {
		r->k++;
		e.arc = 1;
		e.cx = num(r);
		e.cy = num(r);
	}
	e.by = read_by(p, r);
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

// Reads near or far as 1 or -1.
static int read_side(struct reader *r)
{
	const char *s = word(r);

	if (strcmp(s, "near") != 0 && strcmp(s, "far") != 0) {
		die("%s: expected near or far", r->where);
	}
	return s[0] == 'n' ? 1 : -1;
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
	h.blind = 0;
	h.bside = 0;
	if (r->k < r->n && strcmp(r->w[r->k], "csink") == 0) {
		r->k++;
		h.rc = num(r);
		h.h = num(r);
		h.side = read_side(r);
	}
	if (r->k < r->n && strcmp(r->w[r->k], "blind") == 0) {
		r->k++;
		h.blind = num(r);
		h.bside = read_side(r);
		if (!(h.blind > 0)) {
			die("%s: a blind hole has a depth above 0", r->where);
		}
	}
	h.by = read_by(p, r);
	PUSH(p->holes, p->nholes, p->capholes, h);
}

// Where a record of a stream stands.
struct at {
	const char *where;
};

// A number of a stream, the word w; ctx is where it stands.
static double plain_num(void *ctx, const char *w)
{
	char *end = NULL;
	double v = strtod(w, &end);

	if (*end || end == w || !num_ok(v)) {
		const struct at *at = ctx;

		die("%s: bad number %s, or beyond 1e6", at->where, w);
	}
	return v;
}

// Puts the n words w into out, of size room, a blank between each two;
// dies if they do not fit.
static void join_words(char *out, size_t room, char **w, size_t n)
{
	size_t k = 0;

	out[0] = 0;
	for (size_t i = 0; i < n; i++) {
		size_t m = strlen(w[i]);

		if (k + m + 2 > room) {
			die("a callout is too long: %s", w[0]);
		}
		if (i) {
			out[k++] = ' ';
		}
		memcpy(out + k, w[i], m + 1);
		k += m;
	}
}

void read_material(struct material *m, char *line, const char *where)
{
	struct reader r;

	memset(&r, 0, sizeof(r));
	r.where = where;
	r.n = split_words(line, r.w, MAXWORDS);
	if (r.n > MAXWORDS || strcmp(word(&r), "material") != 0) {
		die("%s: expected material NAME R G B ROUGHNESS DENSITY",
		    where);
	}
	m->name = xstrdup(word(&r));
	for (int i = 0; i < 3; i++) {
		m->rgb[i] = num(&r);
	}
	m->rough = num(&r);
	m->density = num(&r);
	{
		struct at at = { where };
		struct finish_words fw = { r.w + r.k, r.n - r.k, where,
					   plain_num, &at };

		material_finish(m, &fw);
	}
	check_material(m, where);
}

void material_finish(struct material *m, const struct finish_words *fw)
{
	size_t i = 0;

	m->draft = 0;
	m->vdi = -1;
	m->mark = 0;
	m->callout[0] = 0;
	while (i < fw->n) {
		const char *k = fw->w[i];

		if (i + 1 >= fw->n) {
			die("%s: %s needs a value", fw->where, k);
		}
		if (!strcmp(k, "draft")) {
			m->draft = fw->num(fw->ctx, fw->w[i + 1]);
		} else if (!strcmp(k, "mark") && strlen(fw->w[i + 1]) == 1 &&
			   isupper((unsigned char)fw->w[i + 1][0])) {
			m->mark = fw->w[i + 1][0];
		} else if (!strcmp(k, "vdi")) {
			m->vdi = fw->num(fw->ctx, fw->w[i + 1]);
			if (!(m->vdi >= 0 && m->vdi <= VDI_MAX)) {
				die("%s: a VDI 3400 grade runs from 0 to 45",
				    fw->where);
			}
			xsnprintf(m->callout, "VDI 3400 REF %g", m->vdi);
		} else if (!strcmp(k, "callout")) {
			join_words(m->callout, sizeof(m->callout),
				   fw->w + i + 1, fw->n - i - 1);
			return;
		} else {
			die("%s: expected draft DEG, mark LETTER, vdi GRADE or "
			    "callout TEXT",
			    fw->where);
		}
		i += 2;
	}
}

void check_material(const struct material *m, const char *where)
{
	for (int i = 0; i < 3; i++) {
		if (!(m->rgb[i] >= 0 && m->rgb[i] <= 1)) {
			die("%s: a color runs from 0 to 1", where);
		}
	}
	if (!(m->rough >= 0 && m->rough <= 1)) {
		die("%s: a roughness runs from 0 to 1", where);
	}
	if (!(m->density >= 0 && m->density <= DENSITY_MAX)) {
		die("%s: a density runs from 0 to that of the densest metal",
		    where);
	}
	if (!(m->draft >= 0 && m->draft < DRAFT_MAX)) {
		die("%s: a least draft runs from 0 to 45 degrees", where);
	}
}

// Reads a coat record: MATERIAL near|far edges|- N and N corners.
static void read_coat(struct part *p, struct reader *r)
{
	struct coat c;
	const char *edges = NULL;
	double n = 0;

	memset(&c, 0, sizeof(c));
	c.material = xstrdup(word(r));
	c.side = read_side(r);
	edges = word(r);
	n = num(r);
	if (strcmp(edges, "edges") != 0 && strcmp(edges, "-") != 0) {
		die("%s: a coat takes edges or -", r->where);
	}
	if (!(n >= 3 && n <= (double)(r->n - r->k) / 2)) {
		die("%s: a coat has 3 corners or more", r->where);
	}
	c.edges = edges[0] == 'e';
	c.n = (size_t)n;
	c.pt = xmalloc(c.n * sizeof(*c.pt));
	for (size_t k = 0; k < c.n; k++) {
		c.pt[k].x = num(r);
		c.pt[k].y = num(r);
	}
	PUSH(p->coats, p->ncoats, p->capcoats, c);
}

const struct material *part_material(const struct part *p, const char *name)
{
	for (size_t i = 0; i < p->nmats; i++) {
		if (strcmp(p->mats[i].name, name) == 0) {
			return &p->mats[i];
		}
	}
	return NULL;
}

// Whether a and b differ; neither is NaN.
static int numbers_differ(double a, double b)
{
	return a < b || a > b;
}

int materials_differ(const struct material *a, const struct material *b)
{
	for (int i = 0; i < 3; i++) {
		if (numbers_differ(a->rgb[i], b->rgb[i])) {
			return 1;
		}
	}
	return numbers_differ(a->rough, b->rough) ||
	       numbers_differ(a->density, b->density) ||
	       numbers_differ(a->draft, b->draft) || a->mark != b->mark ||
	       strcmp(a->callout, b->callout) != 0;
}

void set_base(struct part *p, const struct material *m)
{
	free(p->base);
	p->base = xstrdup(m->name);
	memcpy(p->rgb, m->rgb, sizeof(p->rgb));
	p->rough = m->rough;
	p->density = m->density;
}

// Reads a record finish MATERIAL REF... of a solid.
static void read_finish(struct part *p, struct reader *r)
{
	struct face_finish f = { NULL, NULL, 0 };
	size_t cap = 0;

	f.material = xstrdup(word(r));
	if (r->k == r->n) {
		die("%s: finish MATERIAL REF...", r->where);
	}
	while (r->k < r->n) {
		struct str s = { xstrdup(word(r)) };

		PUSH(f.refs, f.n, cap, s);
	}
	PUSH(p->fins, p->nfins, p->capfins, f);
}

// Reads a record thread REF [deep D] DESIGNATION... of a solid.
static void read_thread(struct part *p, struct reader *r)
{
	struct face_thread t = { NULL, NULL, 0 };
	size_t n = 1;
	size_t k = 0;

	t.ref = xstrdup(word(r));
	if (r->k < r->n && strcmp(r->w[r->k], "deep") == 0) {
		r->k++;
		t.deep = num(r);
		if (!(t.deep > 0)) {
			die("%s: the depth of a thread is above 0", r->where);
		}
	}
	if (r->k == r->n) {
		die("%s: thread REF [deep D] DESIGNATION...", r->where);
	}
	for (size_t i = r->k; i < r->n; i++) {
		n += strlen(r->w[i]) + 1;
	}
	t.spec = xmalloc(n);
	for (; r->k < r->n; r->k++) {
		size_t m = strlen(r->w[r->k]);

		if (k) {
			t.spec[k++] = ' ';
		}
		memcpy(t.spec + k, r->w[r->k], m);
		k += m;
	}
	t.spec[k] = 0;
	PUSH(p->threads, p->nthreads, p->capthreads, t);
}

// Reads a record insert NAME TYPE near|far HOLE D H [fills]
// [material M].
static void read_insert(struct part *p, struct reader *r)
{
	struct part_insert in;

	memset(&in, 0, sizeof(in));
	in.name = xstrdup(word(r));
	in.type = xstrdup(word(r));
	in.side = read_side(r);
	in.hole = index_of(r);
	in.body_d = num(r);
	in.body_h = num(r);
	if (r->k < r->n && strcmp(r->w[r->k], "fills") == 0) {
		r->k++;
		in.fills = 1;
	}
	if (r->k < r->n && strcmp(r->w[r->k], "material") == 0) {
		r->k++;
		in.material = xstrdup(word(r));
	}
	if (!(in.body_d > 0) || !(in.body_h >= 0) ||
	    !(in.body_h > 0 || in.fills)) {
		die("%s: an insert needs a body: D above 0, and H above 0 "
		    "unless it fills its hole",
		    r->where);
	}
	PUSH(p->ins, p->nins, p->capins, in);
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
	} else if (strcmp(k, "base") == 0) {
		if (p->base) {
			die("%s: a second base", r->where);
		}
		p->base = xstrdup(word(r));
	} else if (strcmp(k, "coat") == 0) {
		read_coat(p, r);
	} else if (strcmp(k, "finish") == 0) {
		read_finish(p, r);
	} else if (strcmp(k, "thread") == 0) {
		read_thread(p, r);
	} else if (strcmp(k, "insert") == 0) {
		read_insert(p, r);
	} else if (range_record(&p->rg, r->w, r->n, r->where)) {
		r->k = r->n;
	} else {
		die("%s: unknown record %s", r->where, k);
	}
	if (r->k != r->n) {
		die("%s: too many fields", r->where);
	}
}

struct part new_part(const char *name)
{
	struct part z;

	memset(&z, 0, sizeof(z));
	z.name = xstrdup(name);
	z.place = rig_id();
	return z;
}

// Ends part p: a part with no record of a blank, nor of a solid, is an
// empty solid, as a common of two solids apart.
static void end_part(struct part *p, const char *where)
{
	int blank = p->nmaps || p->nfaces || p->nverts || p->nedges ||
		    p->nholes || p->t > 0;

	if (!p->solid && !blank) {
		p->solid = xmalloc(sizeof(*p->solid));
		shape_init(p->solid, p->name);
	}
	if (p->solid) {
		if (blank) {
			die("%s: a part has both a blank and a solid", where);
		}
		shape_check(p->solid, where);
		return;
	}
	check_part(p, where);
	index_faces(p);
}

// Whether k is the key of a record of an exact solid (shape.h).
static int solid_key(const char *k)
{
	return !strcmp(k, "v") || !strcmp(k, "e") || !strcmp(k, "f") ||
	       !strcmp(k, "l");
}

int part_line(struct part *p, char *line, const char *where)
{
	struct reader r;
	char *copy = NULL;
	const char *k = NULL;

	memset(&r, 0, sizeof(r));
	r.where = where;
	copy = xstrdup(line);
	r.n = split_words(copy, r.w, MAXWORDS);
	if (!r.n) {
		free(copy);
		return 1;
	}
	if (r.n > MAXWORDS && !solid_key(r.w[0])) {
		die("%s: too many fields", where);
	}
	k = word(&r);
	if (strcmp(k, "end") == 0) {
		if (r.n != 1) {
			die("%s: too many fields", where);
		}
		free(copy);
		end_part(p, where);
		return 0;
	}
	if (solid_key(k)) {
		free(copy);
		if (!p->solid) {
			p->solid = xmalloc(sizeof(*p->solid));
			shape_init(p->solid, p->name);
		}
		(void)shape_record(p->solid, line, where);
		return 1;
	}
	read_record(p, &r, k);
	free(copy);
	return 1;
}

void free_finish(struct face_finish *f)
{
	free(f->material);
	for (size_t k = 0; k < f->n; k++) {
		free(f->refs[k].s);
	}
	free(f->refs);
}

void free_thread(struct face_thread *t)
{
	free(t->ref);
	free(t->spec);
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
	free(p->base);
	for (size_t i = 0; i < p->ncoats; i++) {
		free(p->coats[i].material);
		free(p->coats[i].pt);
	}
	for (size_t i = 0; i < p->nfins; i++) {
		free_finish(&p->fins[i]);
	}
	free(p->fins);
	for (size_t i = 0; i < p->nthreads; i++) {
		free_thread(&p->threads[i]);
	}
	free(p->threads);
	for (size_t i = 0; i < p->nins; i++) {
		free(p->ins[i].type);
		free(p->ins[i].name);
		free(p->ins[i].material);
	}
	free(p->ins);
	free(p->coats);
	for (size_t i = 0; i < p->nmats; i++) {
		free(p->mats[i].name);
	}
	free(p->mats);
	for (size_t i = 0; i < p->nfeats; i++) {
		free(p->feats[i].s);
	}
	free(p->feats);
	if (p->solid) {
		shape_free(p->solid);
		free(p->solid);
	}
	free_ranges(&p->rg);
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

// Writes " by NAME" for feature f of p, if any.
static void write_by(const struct part *p, size_t f)
{
	if (f != NONE) {
		printf(" by %s", p->feats[f].s);
	}
}

static void write_edge(const struct part *p, size_t i)
{
	const struct edge *e = &p->edges[i];

	printf("edge %zu %zu %zu", i, e->v0, e->v1);
	pidx(e->lf);
	pidx(e->rf);
	if (e->arc) {
		printf(" arc");
		pnum(e->cx);
		pnum(e->cy);
	}
	write_by(p, e->by);
	printf("\n");
}

void write_material(const struct material *m)
{
	printf("material %s", m->name);
	for (int i = 0; i < 3; i++) {
		pnum(m->rgb[i]);
	}
	pnum(m->rough);
	pnum(m->density);
	if (m->draft > 0) {
		printf(" draft");
		pnum(m->draft);
	}
	if (m->mark) {
		printf(" mark %c", m->mark);
	}
	if (m->vdi >= 0) {
		printf(" vdi");
		pnum(m->vdi);
	} else if (m->callout[0]) {
		printf(" callout %s", m->callout);
	}
	printf("\n");
}

static void write_hole(const struct part *p, const struct hole *h)
{
	printf("hole %zu", h->face);
	pnum(h->cx);
	pnum(h->cy);
	pnum(h->r);
	if (h->side) {
		printf(" csink");
		pnum(h->rc);
		pnum(h->h);
		printf(" %s", h->side > 0 ? "near" : "far");
	}
	if (h->blind > 0) {
		printf(" blind");
		pnum(h->blind);
		printf(" %s", h->bside > 0 ? "near" : "far");
	}
	write_by(p, h->by);
	printf("\n");
}

static void write_blank(const struct part *p)
{
	printf("t");
	pnum(p->t);
	printf("\n");
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
		write_edge(p, i);
	}
	for (size_t i = 0; i < p->nholes; i++) {
		write_hole(p, &p->holes[i]);
	}
}

void write_part(const struct part *p)
{
	printf("part %s\n", p->name);
	printf("place");
	rig_write(&p->place);
	printf("\n");
	if (p->base) {
		printf("base %s\n", p->base);
	}
	for (size_t i = 0; i < p->ncoats; i++) {
		const struct coat *c = &p->coats[i];

		printf("coat %s %s %s %zu", c->material,
		       c->side > 0 ? "near" : "far", c->edges ? "edges" : "-",
		       c->n);
		for (size_t k = 0; k < c->n; k++) {
			pnum(c->pt[k].x);
			pnum(c->pt[k].y);
		}
		printf("\n");
	}
	for (size_t i = 0; i < p->nins && !p->bodied; i++) {
		const struct part_insert *in = &p->ins[i];

		printf("insert %s %s %s %zu", in->name, in->type,
		       in->side > 0 ? "near" : "far", in->hole);
		pnum(in->body_d);
		pnum(in->body_h);
		if (in->fills) {
			printf(" fills");
		}
		if (in->material) {
			printf(" material %s", in->material);
		}
		printf("\n");
	}
	for (size_t i = 0; i < p->nfins; i++) {
		printf("finish %s", p->fins[i].material);
		for (size_t k = 0; k < p->fins[i].n; k++) {
			printf(" %s", p->fins[i].refs[k].s);
		}
		printf("\n");
	}
	for (size_t i = 0; i < p->nthreads; i++) {
		const struct face_thread *t = &p->threads[i];

		printf("thread %s", t->ref);
		if (t->deep > 0) {
			printf(" deep");
			pnum(t->deep);
		}
		printf(" %s\n", t->spec);
	}
	if (p->solid) {
		shape_write(stdout, p->solid);
	} else {
		write_blank(p);
	}
	write_ranges(&p->rg);
	printf("end\n");
}

size_t add_tol(struct ranges *g, const char *name, int kind, double lo,
	       double hi)
{
	struct part_tol t = { xstrdup(name), kind, lo, hi };

	PUSH(g->tol, g->ntol, g->captol, t);
	return g->ntol - 1;
}

void add_vary(struct ranges *g, size_t tol, int hi, const char *rec)
{
	struct part_vary v = { tol, hi, xstrdup(rec) };

	PUSH(g->vary, g->nvary, g->capvary, v);
}

// The words w[from .. n - 1] with a blank between each two, a new
// string.
static char *joined(char **w, size_t from, size_t n)
{
	size_t room = 1;
	char *out = NULL;
	size_t k = 0;

	for (size_t i = from; i < n; i++) {
		room += strlen(w[i]) + 1;
	}
	out = xmalloc(room);
	for (size_t i = from; i < n; i++) {
		size_t m = strlen(w[i]);

		if (k) {
			out[k++] = ' ';
		}
		memcpy(out + k, w[i], m);
		k += m;
	}
	out[k] = 0;
	return out;
}

int range_record(struct ranges *g, char **w, size_t n, const char *where)
{
	struct reader r;

	memset(&r, 0, sizeof(r));
	r.where = where;
	if (n > MAXWORDS) {
		die("%s: too many fields", where);
	}
	for (size_t i = 0; i < n; i++) {
		r.w[i] = w[i];
	}
	r.n = n;
	r.k = 1;
	if (strcmp(w[0], "tol") == 0) {
		const char *name = word(&r);
		const char *k = word(&r);
		int kind = KINDS;
		double lo = 0;
		double hi = 0;

		for (int i = 0; i < KINDS; i++) {
			kind = strcmp(k, kind_word(i)) == 0 ? i : kind;
		}
		lo = num(&r);
		hi = num(&r);
		if (kind == KINDS || r.k != n || !(lo <= hi)) {
			die("%s: expected tol NAME number|length|angle LO HI",
			    where);
		}
		(void)add_tol(g, name, kind, lo, hi);
		return 1;
	}
	if (strcmp(w[0], "vary") == 0) {
		const char *name = word(&r);
		const char *end = word(&r);
		size_t t = NONE;
		char *rec = NULL;

		for (size_t i = 0; i < g->ntol; i++) {
			t = strcmp(g->tol[i].name, name) == 0 ? i : t;
		}
		if (t == NONE || (strcmp(end, "-") != 0 && strcmp(end, "+")) ||
		    r.k >= n) {
			die("%s: expected vary NAME -|+ RECORD, after tol NAME",
			    where);
		}
		rec = joined(w, r.k, n);
		add_vary(g, t, end[0] == '+', rec);
		free(rec);
		return 1;
	}
	return 0;
}

void write_ranges(const struct ranges *g)
{
	for (size_t i = 0; i < g->ntol; i++) {
		printf("tol %s %s", g->tol[i].name, kind_word(g->tol[i].kind));
		pnum(g->tol[i].lo);
		pnum(g->tol[i].hi);
		printf("\n");
	}
	for (size_t i = 0; i < g->nvary; i++) {
		const struct part_vary *v = &g->vary[i];

		printf("vary %s %c %s\n", g->tol[v->tol].name,
		       v->hi ? '+' : '-', v->rec);
	}
}

void copy_ranges(struct ranges *to, const struct ranges *from)
{
	size_t base = to->ntol;

	for (size_t i = 0; i < from->ntol; i++) {
		const struct part_tol *t = &from->tol[i];

		(void)add_tol(to, t->name, t->kind, t->lo, t->hi);
	}
	for (size_t i = 0; i < from->nvary; i++) {
		const struct part_vary *v = &from->vary[i];

		add_vary(to, base + v->tol, v->hi, v->rec);
	}
}

void free_ranges(struct ranges *g)
{
	for (size_t i = 0; i < g->ntol; i++) {
		free(g->tol[i].name);
	}
	for (size_t i = 0; i < g->nvary; i++) {
		free(g->vary[i].rec);
	}
	free(g->tol);
	free(g->vary);
	memset(g, 0, sizeof(*g));
}

// A copy of the n items of size each at from, a new array.
static void *copy_of(const void *from, size_t n, size_t each)
{
	void *to = xmalloc((n + 1) * each);

	if (n) {
		memcpy(to, from, n * each);
	}
	return to;
}

// Reads the index of a record of a variant, below count.
static size_t vary_index(struct reader *r, size_t count)
{
	size_t i = index_of(r);

	if (i == NONE || i >= count) {
		die("%s: bad index of a vary record", r->where);
	}
	return i;
}

// Applies the vary record of the words of r to the variant q.
static void vary_record(struct part *q, struct reader *r)
{
	const char *k = word(r);

	if (strcmp(k, "t") == 0) {
		q->t = num(r);
	} else if (strcmp(k, "map") == 0) {
		struct map *m = &q->maps[vary_index(r, q->nmaps)];

		if (strcmp(word(r), "rigid") == 0) {
			rig_read(r, &m->m);
		} else {
			(void)index_of(r);
			m->qx = num(r);
			m->qy = num(r);
			m->mx = num(r);
			m->my = num(r);
			m->ba = num(r);
			m->ang = num(r);
			m->r = num(r);
		}
	} else if (strcmp(k, "vertex") == 0) {
		struct vec2 *v = &q->verts[vary_index(r, q->nverts)];

		v->x = num(r);
		v->y = num(r);
	} else if (strcmp(k, "edge") == 0) {
		struct edge *e = &q->edges[vary_index(r, q->nedges)];

		e->cx = num(r);
		e->cy = num(r);
	} else if (strcmp(k, "hole") == 0) {
		struct hole *h = &q->holes[vary_index(r, q->nholes)];

		h->cx = num(r);
		h->cy = num(r);
		h->r = num(r);
		h->rc = num(r);
		h->h = num(r);
		h->blind = num(r);
	} else if (strcmp(k, "coat") == 0) {
		struct coat *c = &q->coats[vary_index(r, q->ncoats)];

		for (size_t i = 0; i < c->n; i++) {
			c->pt[i].x = num(r);
			c->pt[i].y = num(r);
		}
	} else {
		die("%s: unknown vary record %s", r->where, k);
	}
	if (r->k != r->n) {
		die("%s: too many fields", r->where);
	}
}

void vary_part(const struct part *p, size_t tol, int hi, struct part *q)
{
	*q = *p;
	q->maps = copy_of(p->maps, p->nmaps, sizeof(*p->maps));
	q->verts = copy_of(p->verts, p->nverts, sizeof(*p->verts));
	q->edges = copy_of(p->edges, p->nedges, sizeof(*p->edges));
	q->holes = copy_of(p->holes, p->nholes, sizeof(*p->holes));
	q->coats = copy_of(p->coats, p->ncoats, sizeof(*p->coats));
	for (size_t i = 0; i < p->ncoats; i++) {
		q->coats[i].pt = copy_of(p->coats[i].pt, p->coats[i].n,
					 sizeof(struct vec2));
	}
	for (size_t i = 0; i < p->rg.nvary; i++) {
		const struct part_vary *v = &p->rg.vary[i];
		struct reader r;
		char *copy = NULL;

		if (v->tol != tol || v->hi != hi) {
			continue;
		}
		memset(&r, 0, sizeof(r));
		r.where = p->name;
		copy = xstrdup(v->rec);
		r.n = split_words(copy, r.w, MAXWORDS);
		if (r.n > MAXWORDS) {
			die("%s: too many fields", p->name);
		}
		vary_record(q, &r);
		free(copy);
	}
}

void free_variant(struct part *q)
{
	for (size_t i = 0; i < q->ncoats; i++) {
		free(q->coats[i].pt);
	}
	free(q->maps);
	free(q->verts);
	free(q->edges);
	free(q->holes);
	free(q->coats);
	memset(q, 0, sizeof(*q));
}
