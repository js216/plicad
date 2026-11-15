// SPDX-License-Identifier: GPL-3.0-only
// smview.c -- Project a folded part into the lines of one drawing view
// Copyright (c) 2026 Jakob Kastelic

// Usage: smview [-h] VIEW part.fold > part.view
//
// VIEW is front, back, left, right, top, bottom or flat.  The front view
// looks along +y with z up, the top view looks down -z with y up, and the
// others follow by third-angle projection.  The flat view is the top view
// of the unfolded blank, with a centre line along each bend.
//
// The output has one record per line, in mm, x to the right and y up:
//   view VIEW PART
//   thick T
//   box X0 Y0 X1 Y1
//   fill                   (the start of a region of material,
//   m X Y                   whose loops fill by the even-odd rule:
//   l X Y                   move, line to,
//   a CX CY R A0 A1         arc from angle A0 to A1, in degrees,
//   z                       and close)
//   line KIND VIS X0 Y0 X1 Y1
//   arc KIND VIS CX CY R A0 A1     (counterclockwise, in degrees)
//   hole X Y D [csink DC ANGLE near|far]
//   radius R               (the inside radius of the first bend)
//   bend X0 Y0 X1 Y1 ANGLE (flat view: a bend line, and the angle of the
//                           bend in degrees, above 0 toward the eye)
// KIND is e for a sharp edge of the sheet, i for an edge of a cut-out
// inside it, o for the rim of a hole and for an arc that is hidden in
// part, b for an edge or the outline of a bend, s for the seam where a
// bend leaves a flat face, and c for a centre line.  VIS is v for
// visible and h for hidden; the hidden lines are left out unless -h is
// given.  A hole record stands for each hole
// seen along its axis and not covered.
//
// A line is hidden where material lies between it and the eye.  The
// faces are planes, cylinders and cones, so the test is exact.  A circle
// seen at an angle would be an ellipse, which the output has no record
// for: smview dies, and names the view and the place of the circle.

#include "brep.h"
#include "fold.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TINY 1e-9 // Sine below which two directions are parallel.
#define NEAR 1e-7 // Distance below which two points are one, in mm.
#define SIDE 1e-8 // The step to each side of a curve, in mm.
#define FRONT 1e-3 // A face this much nearer the eye hides, in mm.
#define SAME 1e-6 // Distance below which two pieces are one, in mm.
#define ZERO 5e-7 // Numbers closer to 0 than this print as 0.
#define SLIVER 1e-3 // No curve or piece of one is shorter, in mm,
#define PRINTED 2e-6 // even after its ends are rounded to six places.

enum { K_EDGE, K_INNER, K_HOLE, K_BEND, K_SEAM, K_CENTRE };
static const char kind_code[] = "eiobsc";

// A view: x to the right and y up on the paper, z toward the eye.
struct view {
	const char *name;
	struct vec x, y, z;
};

static const struct view views[] = {
	{ "front", { 1, 0, 0 }, { 0, 0, 1 }, { 0, -1, 0 } },
	{ "back", { -1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } },
	{ "right", { 0, 1, 0 }, { 0, 0, 1 }, { 1, 0, 0 } },
	{ "left", { 0, -1, 0 }, { 0, 0, 1 }, { -1, 0, 0 } },
	{ "top", { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } },
	{ "bottom", { 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 } },
};

// A projected curve.  A segment runs from a to b, with the depth za at a
// and zb at b.  An arc runs counterclockwise about c from the angle a0
// through da, at the depth za.  A segment with a bulge is a circle seen
// edge on: its depth at distance s from c is za + bulge sqrt(r r - s s).
struct curve {
	int arc, kind, bulge;
	int part; // Whether a piece of it is hidden.
	struct vec2 a, b, c;
	double r, a0, da, za, zb;
	struct box box;
};

// The point at t, from 0 to 1 along a curve, where it is cut in two.
struct split {
	size_t c;
	double t;
};

// A piece of curve c between two cuts, with its ends and its middle.
struct piece {
	size_t c;
	double t0, t1;
	struct vec2 p0, pm, p1;
	int hidden, dead;
};

struct scene {
	const struct part *p;
	const struct part *radius; // The part as folded, for its bends.
	struct brep b;
	struct view vw;
	struct curve *c;
	size_t nc, capc;
	struct split *s;
	size_t ns, caps;
	struct piece *pc;
	size_t np, cappc;
	struct box *fbox; // The box of each face on the paper, with depth.
	size_t (*ef)[2]; // The two faces at each edge.
	int *inner; // Whether each edge of the flat part is on a cut-out.
	int show_hidden;
};

static struct vec to_view(const struct view *vw, struct vec p)
{
	return v3(dot(p, vw->x), dot(p, vw->y), dot(p, vw->z));
}

static struct vec2 xy(struct vec p)
{
	struct vec2 q = { p.x, p.y };

	return q;
}

static struct vec2 pt2(double x, double y)
{
	struct vec2 q = { x, y };

	return q;
}

static double dist2(struct vec2 a, struct vec2 b)
{
	return hypot(b.x - a.x, b.y - a.y);
}

// The angle a, turned by whole turns into 0 to 2 pi.
static double wrap(double a)
{
	a = fmod(a, 2 * PI);
	return a < 0 ? a + (2 * PI) : a;
}

static double zero(double x)
{
	return fabs(x) < ZERO ? 0.0 : x;
}

// The 3D segment from a to b.
struct seg3 {
	struct vec a, b;
};

// Whether a curve or a piece of length l is too short to write: one a
// little longer than SLIVER may come out shorter once rounded.
static int sliver(double l)
{
	return l < SLIVER + PRINTED;
}

// Adds curve c, with its box, unless it is shorter than SLIVER.
static void add_curve(struct scene *sc, struct curve c)
{
	if (sliver(c.arc ? c.r * c.da : dist2(c.a, c.b))) {
		return;
	}
	c.box = box_empty();
	if (c.arc) {
		box_add(&c.box, v3(c.c.x - c.r, c.c.y - c.r, 0));
		box_add(&c.box, v3(c.c.x + c.r, c.c.y + c.r, 0));
	} else {
		box_add(&c.box, v3(c.a.x, c.a.y, 0));
		box_add(&c.box, v3(c.b.x, c.b.y, 0));
	}
	box_pad(&c.box, NEAR);
	PUSH(sc->c, sc->nc, sc->capc, c);
}

// Adds the projection of a 3D segment, unless it is seen end on.
static void add_seg(struct scene *sc, struct seg3 s, int kind)
{
	struct vec a = to_view(&sc->vw, s.a);
	struct vec b = to_view(&sc->vw, s.b);
	struct curve c;

	if (dist2(xy(a), xy(b)) < NEAR) {
		return;
	}
	memset(&c, 0, sizeof(c));
	c.kind = kind;
	c.a = xy(a);
	c.b = xy(b);
	c.za = a.z;
	c.zb = b.z;
	add_curve(sc, c);
}

// Adds a circular edge seen along its axis: an arc.
static void add_arc(struct scene *sc, const struct bedge *e, int kind)
{
	const struct view *vw = &sc->vw;
	int ccw = dot(e->n, vw->z) > 0;
	struct vec c = to_view(vw, e->c);
	struct vec s = to_view(vw, sc->b.v[ccw ? e->v0 : e->v1]);
	struct curve a;

	memset(&a, 0, sizeof(a));
	a.arc = 1;
	a.kind = kind;
	a.c = xy(c);
	a.r = e->r;
	a.a0 = wrap(atan2(s.y - c.y, s.x - c.x));
	a.da = e->sweep;
	a.za = c.z;
	add_curve(sc, a);
}

// Adds a circular edge seen edge on.  With e1 across the view and the
// eye along e2, the edge runs from the angle f0 down through its sweep.
// Each part between two multiples of pi is one segment with a bulge.
static void add_edge_on(struct scene *sc, const struct bedge *e, int kind)
{
	const struct view *vw = &sc->vw;
	struct vec e1 = unit(cross(e->n, vw->z));
	struct vec d = sub(sc->b.v[e->v0], e->c);
	double f0 = atan2(dot(d, vw->z), dot(d, e1));
	double lo = f0 - e->sweep;

	while (lo < f0 - (NEAR / e->r)) {
		double hi = fmin(f0, (floor((lo / PI) + TINY) + 1) * PI);
		struct vec a = to_view(vw, add(e->c, mul(e1, e->r * cos(lo))));
		struct vec b = to_view(vw, add(e->c, mul(e1, e->r * cos(hi))));
		struct curve c;

		memset(&c, 0, sizeof(c));
		c.kind = kind;
		c.bulge = sin((lo + hi) / 2) > 0 ? 1 : -1;
		c.a = xy(a);
		c.b = xy(b);
		c.c = xy(to_view(vw, e->c));
		c.r = e->r;
		c.za = a.z;
		if (dist2(c.a, c.b) > NEAR) {
			add_curve(sc, c);
		}
		lo = hi;
	}
}

// Whether brep face f comes from a bend zone.
static int of_bend(const struct scene *sc, const struct bface *f)
{
	const struct part *p = sc->p;
	size_t f2 = f->idx;

	if (f->from == F_HOLE) {
		return 0;
	}
	if (f->from == F_SIDE) {
		const struct edge *e = &p->edges[f->idx];

		f2 = e->lf != NONE ? e->lf : e->rf;
	}
	return p->maps[p->faces[f2]].bend;
}

// The kind of brep edge i, or -1 for a seam between two faces in one
// plane, which is no line at all.
static int edge_kind(const struct scene *sc, size_t i)
{
	const struct bface *f = &sc->b.f[sc->ef[i][0]];
	const struct bface *g = &sc->b.f[sc->ef[i][1]];

	if (f->from == g->from && (f->from == F_TOP || f->from == F_BOTTOM)) {
		return f->type == S_PLANE && g->type == S_PLANE ? -1 : K_SEAM;
	}
	if (of_bend(sc, f) || of_bend(sc, g)) {
		return K_BEND;
	}
	if (f->from == F_HOLE || g->from == F_HOLE) {
		return K_HOLE;
	}
	return (f->from == F_SIDE && sc->inner[f->idx]) ||
			       (g->from == F_SIDE && sc->inner[g->idx]) ?
		       K_INNER :
		       K_EDGE;
}

// Walks the loop of the outline through edge i with the material on
// the left, by the outline edge out[] that leaves each vertex, and sets
// mark[] for each edge on the way.  Returns twice the area the loop
// goes round, by its corners: below 0 for a loop that runs clockwise.
static double walk_loop(const struct part *p, const size_t *out, size_t i,
			int *mark)
{
	double area = 0;
	size_t k = i;

	do {
		const struct edge *e = &p->edges[k];
		int fwd = e->lf != NONE;
		struct vec2 a = p->verts[fwd ? e->v0 : e->v1];
		struct vec2 b = p->verts[fwd ? e->v1 : e->v0];

		area += (a.x * b.y) - (a.y * b.x);
		mark[k] = 1;
		k = out[fwd ? e->v1 : e->v0];
	} while (k != i);
	return area;
}

// Marks the outline edges of the flat part that lie on a cut-out: the
// loops that run clockwise.
static void mark_cutouts(struct scene *sc)
{
	const struct part *p = sc->p;
	size_t *out = xmalloc((p->nverts + 1) * sizeof(*out));
	int *seen = xcalloc(p->nedges + 1, sizeof(*seen));

	sc->inner = xcalloc(p->nedges + 1, sizeof(*sc->inner));
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];

		if ((e->lf == NONE) != (e->rf == NONE)) {
			out[e->lf != NONE ? e->v0 : e->v1] = i;
		}
	}
	for (size_t i = 0; i < p->nedges; i++) {
		const struct edge *e = &p->edges[i];

		if ((e->lf == NONE) != (e->rf == NONE) && !seen[i] &&
		    walk_loop(p, out, i, seen) < 0) {
			walk_loop(p, out, i, sc->inner);
		}
	}
	free(out);
	free(seen);
}

// Notes the two faces at each edge.
static void edge_faces(struct scene *sc)
{
	const struct brep *b = &sc->b;

	sc->ef = xcalloc(b->ne + 1, sizeof(*sc->ef));
	for (size_t f = 0; f < b->nf; f++) {
		const struct bloop *l0 = &b->l[b->f[f].loop0];
		size_t end = b->f[f].nloops ? l0[b->f[f].nloops - 1].first +
						      l0[b->f[f].nloops - 1].n :
					      0;

		for (size_t k = b->f[f].nloops ? l0->first : 0; k < end; k++) {
			const struct buse *u = &b->u[k];

			sc->ef[u->e][u->fwd ? 0 : 1] = f;
		}
	}
}

// Adds the curve of each edge of the solid.
static void add_edges(struct scene *sc)
{
	const struct brep *b = &sc->b;

	for (size_t i = 0; i < b->ne; i++) {
		const struct bedge *e = &b->e[i];
		struct seg3 s = { b->v[e->v0], b->v[e->v1] };
		int kind = edge_kind(sc, i);
		double c = fabs(dot(e->n, sc->vw.z));

		if (kind < 0) {
			continue;
		}
		if (e->type == C_LINE) {
			add_seg(sc, s, kind);
		} else if (c > 1 - TINY) {
			add_arc(sc, e, kind);
		} else if (c < TINY) {
			add_edge_on(sc, e, kind);
		} else {
			die("%s: view %s: the circle about (%g, %g, %g) is "
			    "seen at an angle",
			    sc->p->name, sc->vw.name, zero(e->c.x),
			    zero(e->c.y), zero(e->c.z));
		}
	}
}

// A bend zone of the flat part: its face, its map, and the directions
// of its axis and of its lines of sight.
struct zone {
	size_t f;
	const struct map *m;
	struct line3 ax;
	struct vec g; // Across the view, square to the axis.
};

// The flat point of a zone at s along the axis and u into the zone.
static struct vec2 zone_pt(const struct map *m, double s, double u)
{
	return pt2(m->qx + (m->my * s) + (m->mx * u),
		   m->qy - (m->mx * s) + (m->my * u));
}

static double zone_s(const struct map *m, struct vec2 q)
{
	return ((q.x - m->qx) * m->my) - ((q.y - m->qy) * m->mx);
}

static double zone_u(const struct map *m, struct vec2 q)
{
	return ((q.x - m->qx) * m->mx) + ((q.y - m->qy) * m->my);
}

// Lists into out, sorted, the coordinate of each edge of zone z that
// crosses the line at w: with along, the u of each edge along the axis
// that passes s = w, and without it, the s of each edge across the zone
// that passes u = w.  Returns their number.
static size_t zone_cuts(const struct scene *sc, const struct zone *z, int along,
			double w, double **out)
{
	const struct part *p = sc->p;
	double *v = xmalloc((p->nedges + 1) * sizeof(*v));
	size_t n = 0;

	for (size_t k = p->fe_start[z->f]; k < p->fe_start[z->f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];
		struct vec2 a = p->verts[e->v0];
		struct vec2 b = p->verts[e->v1];
		double s0 = zone_s(z->m, a);
		double s1 = zone_s(z->m, b);
		double u0 = zone_u(z->m, a);
		double u1 = zone_u(z->m, b);
		int is_along = fabs(u1 - u0) < fabs(s1 - s0);

		if (along && is_along && fmin(s0, s1) < w && w < fmax(s0, s1)) {
			v[n++] = u0;
		}
		if (!along && !is_along && fmin(u0, u1) - NEAR < w &&
		    w < fmax(u0, u1) + NEAR) {
			v[n++] = s0;
		}
	}
	sort_doubles(v, n);
	*out = v;
	return n;
}

// The depth z of a zone in the sheet where the eye may see along its
// surface, on the side sign of the axis, and the u there.
struct sight {
	double z, sign, u;
};

// Finds where in zone z the line of sight st touches the surface, if it
// does inside the zone or at its ends.
static int zone_sight(const struct scene *sc, const struct zone *z,
		      struct sight *st)
{
	double rho = fabs(st->z - bend_za(sc->p, z->m));
	struct vec x = add(z->ax.o, mul(z->g, st->sign * rho));
	struct vec flat;

	if (!finv(sc->p, z->f, x, &flat)) {
		return 0;
	}
	st->u = zone_u(z->m, pt2(flat.x, flat.y));
	return st->u > -NEAR && st->u < z->m->ba + NEAR;
}

// Adds the outline of zone z along the line of sight st: a line along
// the axis wherever the zone has material there.
static void zone_outline(struct scene *sc, const struct zone *z,
			 struct sight st)
{
	const struct part *p = sc->p;
	double *s = NULL;
	size_t n = 0;
	double u = 0;

	if (!zone_sight(sc, z, &st)) {
		return;
	}
	u = fmin(fmax(st.u, SAME), z->m->ba - SAME);
	n = zone_cuts(sc, z, 0, st.u, &s);
	for (size_t i = 0; i + 1 < n; i++) {
		struct vec2 a = zone_pt(z->m, s[i], u);
		struct vec2 b = zone_pt(z->m, s[i + 1], u);
		struct vec2 mid = zone_pt(z->m, (s[i] + s[i + 1]) / 2, u);
		struct seg3 l = { fmap(p, z->f, a.x, a.y, st.z),
				  fmap(p, z->f, b.x, b.y, st.z) };

		if (s[i + 1] - s[i] > NEAR && point_in_face(p, z->f, mid)) {
			add_seg(sc, l, K_BEND);
		}
	}
	free(s);
}

// A rectangle of the paper under a face seen from the side: from a to b
// along the axis of the face, and from lo to hi across the view.
struct band {
	struct vec a, b, g;
	double lo, hi;
};

static void fill_band(const struct scene *sc, struct band d)
{
	const struct vec at[4] = { add(d.a, mul(d.g, d.lo)),
				   add(d.b, mul(d.g, d.lo)),
				   add(d.b, mul(d.g, d.hi)),
				   add(d.a, mul(d.g, d.hi)) };

	printf("fill\n");
	for (int i = 0; i < 4; i++) {
		struct vec q = to_view(&sc->vw, at[i]);

		printf("%c %.6f %.6f\n", i ? 'l' : 'm', zero(q.x), zero(q.y));
	}
	printf("z\n");
}

// Fills the part of zone z from s0 to s1 along the axis and from u0 to
// u1 into the zone: across the view it reaches as far as its corners do,
// and as far as a line of sight that touches it between them.
static void zone_fill(const struct scene *sc, const struct zone *z,
		      struct span s, struct span u)
{
	const struct part *p = sc->p;
	struct band d = { add(z->ax.o, mul(z->ax.d, s.lo)),
			  add(z->ax.o, mul(z->ax.d, s.hi)), z->g, HUGE_VAL,
			  -HUGE_VAL };

	for (unsigned k = 0; k < BOX_CORNERS; k++) {
		double depth = k & 1U ? -p->t : 0;
		struct vec2 q = zone_pt(z->m, s.lo, k & 2U ? u.lo : u.hi);
		struct sight st = { depth, k & 2U ? 1 : -1, 0 };
		double l =
			dot(sub(fmap(p, z->f, q.x, q.y, depth), z->ax.o), z->g);

		if (k & 4U) {
			if (!zone_sight(sc, z, &st) || st.u < u.lo ||
			    st.u > u.hi) {
				continue;
			}
			l = st.sign * fabs(depth - bend_za(p, z->m));
		}
		d.lo = fmin(d.lo, l);
		d.hi = fmax(d.hi, l);
	}
	fill_band(sc, d);
}

// Fills zone z, strip by strip between the ends of its edges.
static void zone_fills(const struct scene *sc, const struct zone *z)
{
	const struct part *p = sc->p;
	double *s = xmalloc((2 * p->nedges + 1) * sizeof(*s));
	size_t n = 0;

	for (size_t k = p->fe_start[z->f]; k < p->fe_start[z->f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];

		s[n++] = zone_s(z->m, p->verts[e->v0]);
		s[n++] = zone_s(z->m, p->verts[e->v1]);
	}
	sort_doubles(s, n);
	for (size_t i = 0; i + 1 < n; i++) {
		struct span strip = { s[i], s[i + 1] };
		double *u = NULL;
		size_t nu = 0;

		if (s[i + 1] - s[i] < NEAR) {
			continue;
		}
		nu = zone_cuts(sc, z, 1, (s[i] + s[i + 1]) / 2, &u);
		for (size_t j = 0; j + 1 < nu; j += 2) {
			struct span us = { u[j], u[j + 1] };

			zone_fill(sc, z, strip, us);
		}
		free(u);
	}
	free(s);
}

// Adds the outlines of the bend zones seen from the side, and fills
// them.
static void add_zones(struct scene *sc)
{
	const struct part *p = sc->p;

	for (size_t f = 0; f < p->nfaces; f++) {
		struct zone z = { f,
				  &p->maps[p->faces[f]],
				  { { 0, 0, 0 }, { 0, 0, 0 } },
				  { 0, 0, 0 } };

		if (!z.m->bend) {
			continue;
		}
		z.ax = bend_axis(p, p->faces[f]);
		if (fabs(dot(z.ax.d, sc->vw.z)) > TINY) {
			continue;
		}
		z.g = unit(cross(z.ax.d, sc->vw.z));
		for (unsigned k = 0; k < 4; k++) {
			struct sight st = { k & 1U ? -p->t : 0, k & 2U ? 1 : -1,
					    0 };

			zone_outline(sc, &z, st);
		}
		zone_fills(sc, &z);
	}
}

// A wall through the sheet of slab face f, seen from the side: part of
// a cylinder or a cone about the flat point c, with the radius r0 at the
// depth z0 and r1 at z1, from the angle a0 through da.  The flat unit
// vector g points across the view.
struct wall {
	size_t f;
	struct vec2 c, g;
	double r0, z0, r1, z1, a0, da;
};

// Adds the lines of sight along wall w, and fills it if fill is set.
static void add_wall(struct scene *sc, const struct wall *w, int fill)
{
	const struct part *p = sc->p;
	double ag = atan2(w->g.y, w->g.x);
	double l0 = cos(w->a0 - ag);
	double l1 = cos(w->a0 + w->da - ag);
	struct band d = { fmap(p, w->f, w->c.x, w->c.y, w->z0),
			  fmap(p, w->f, w->c.x, w->c.y, w->z1),
			  sub(fmap(p, w->f, w->c.x + w->g.x, w->c.y + w->g.y,
				   0),
			      fmap(p, w->f, w->c.x, w->c.y, 0)),
			  w->r0 * fmin(l0, l1), w->r0 * fmax(l0, l1) };

	for (int sign = -1; sign <= 1; sign += 2) {
		struct vec2 a = { w->c.x + (sign * w->r0 * w->g.x),
				  w->c.y + (sign * w->r0 * w->g.y) };
		struct vec2 b = { w->c.x + (sign * w->r1 * w->g.x),
				  w->c.y + (sign * w->r1 * w->g.y) };
		struct seg3 s = { fmap(p, w->f, a.x, a.y, w->z0),
				  fmap(p, w->f, b.x, b.y, w->z1) };
		double turn = wrap(ag + (sign < 0 ? PI : 0) - w->a0);

		if (turn > w->da + TINY && turn < (2 * PI) - TINY) {
			continue;
		}
		add_seg(sc, s, K_EDGE);
		d.lo = fmin(d.lo, sign * w->r0);
		d.hi = fmax(d.hi, sign * w->r0);
	}
	if (fill) {
		fill_band(sc, d);
	}
}

// Adds the walls of the arcs and holes of slab face f, seen from the
// side: the arcs of the outline, and each hole with its countersink.
static void add_walls(struct scene *sc, size_t f)
{
	const struct part *p = sc->p;
	struct vec g = unit(cross(fnormal(p, f), sc->vw.z));
	struct vec gf = rig_dir(rig_inv(map_rig(p, p->faces[f])), g);
	struct wall w = { f, { 0, 0 }, { gf.x, gf.y }, 0, 0, 0, -p->t, 0, 0 };

	for (size_t k = p->fe_start[f]; k < p->fe_start[f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];
		struct arc_ang a;

		if (!e->arc || (e->lf != NONE && e->rf != NONE)) {
			continue;
		}
		a = arc_angles(p, e);
		w.c = pt2(e->cx, e->cy);
		w.r0 = w.r1 = a.r;
		w.a0 = a.a0;
		w.da = a.da;
		add_wall(sc, &w, 1);
	}
	w.a0 = 0;
	w.da = 2 * PI;
	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];
		double zm = h->side > 0 ? -h->h : -p->t + h->h;

		if (h->face != f) {
			continue;
		}
		w.c = pt2(h->cx, h->cy);
		w.r0 = w.r1 = h->r;
		w.z0 = h->side > 0 ? zm : 0;
		w.z1 = h->side < 0 ? zm : -p->t;
		add_wall(sc, &w, 0);
		if (h->side) {
			w.z0 = h->side > 0 ? 0 : -p->t;
			w.z1 = zm;
			w.r0 = h->rc;
			add_wall(sc, &w, 0);
		}
	}
}

// Adds the walls of each slab face seen from the side.
static void add_slabs(struct scene *sc)
{
	const struct part *p = sc->p;

	for (size_t f = 0; f < p->nfaces; f++) {
		if (!p->maps[p->faces[f]].bend &&
		    fabs(dot(fnormal(p, f), sc->vw.z)) < TINY) {
			add_walls(sc, f);
		}
	}
}

// The point at t along curve c.
static struct vec2 at(const struct curve *c, double t)
{
	if (c->arc) {
		double a = c->a0 + (t * c->da);

		return pt2(c->c.x + (c->r * cos(a)), c->c.y + (c->r * sin(a)));
	}
	return pt2(c->a.x + (t * (c->b.x - c->a.x)),
		   c->a.y + (t * (c->b.y - c->a.y)));
}

static double depth(const struct curve *c, double t)
{
	double s = dist2(at(c, t), c->c);

	if (c->arc) {
		return c->za;
	}
	if (c->bulge) {
		return c->za +
		       (c->bulge * sqrt(fmax(0, (c->r * c->r) - (s * s))));
	}
	return c->za + (t * (c->zb - c->za));
}

static double length(const struct curve *c)
{
	return c->arc ? c->r * c->da : dist2(c->a, c->b);
}

// Whether q lies on curve c, and where along it.
static int on_curve(const struct curve *c, struct vec2 q, double *t)
{
	double l = length(c);
	double dx = c->b.x - c->a.x;
	double dy = c->b.y - c->a.y;
	double d = 0;

	if (c->arc) {
		d = wrap(atan2(q.y - c->c.y, q.x - c->c.x) - c->a0);
		if (d > c->da && ((2 * PI) - d) * c->r < NEAR) {
			d = 0;
		}
		if (d > c->da && (d - c->da) * c->r < NEAR) {
			d = c->da;
		}
		*t = d / c->da;
		return d <= c->da && fabs(dist2(q, c->c) - c->r) < NEAR;
	}
	d = ((q.x - c->a.x) * dx) + ((q.y - c->a.y) * dy);
	*t = fmin(fmax(d / (l * l), 0), 1);
	return d > -NEAR * l && d < (l + NEAR) * l &&
	       fabs(((q.x - c->a.x) * dy) - ((q.y - c->a.y) * dx)) < NEAR * l;
}

// Cuts curves i and j at q, if q lies on both.
static void cut_at(struct scene *sc, size_t i, size_t j, struct vec2 q)
{
	struct split si = { i, 0 };
	struct split sj = { j, 0 };

	if (!on_curve(&sc->c[i], q, &si.t) || !on_curve(&sc->c[j], q, &sj.t)) {
		return;
	}
	PUSH(sc->s, sc->ns, sc->caps, si);
	PUSH(sc->s, sc->ns, sc->caps, sj);
}

// Cuts i and j at the two points of the line through o along the unit
// vector (dx, dy) that lie h from o, to either side.
static void cut_pair(struct scene *sc, const size_t ij[2], struct vec2 o,
		     struct vec2 d, double h)
{
	cut_at(sc, ij[0], ij[1], pt2(o.x + (h * d.x), o.y + (h * d.y)));
	cut_at(sc, ij[0], ij[1], pt2(o.x - (h * d.x), o.y - (h * d.y)));
}

// Cuts two segments where their lines cross.
static void meet_lines(struct scene *sc, size_t i, size_t j)
{
	const struct curve *c = &sc->c[i];
	const struct curve *e = &sc->c[j];
	struct vec2 d = { c->b.x - c->a.x, c->b.y - c->a.y };
	struct vec2 f = { e->b.x - e->a.x, e->b.y - e->a.y };
	double den = (d.x * f.y) - (d.y * f.x);
	double t = 0;

	if (fabs(den) < TINY * length(c) * length(e)) {
		return;
	}
	t = (((e->a.x - c->a.x) * f.y) - ((e->a.y - c->a.y) * f.x)) / den;
	cut_at(sc, i, j, pt2(c->a.x + (t * d.x), c->a.y + (t * d.y)));
}

// Cuts segment i and arc j where the line meets the circle; a line that
// comes within NEAR of the circle touches it.
static void meet_line_arc(struct scene *sc, size_t i, size_t j)
{
	const struct curve *c = &sc->c[i];
	const struct curve *a = &sc->c[j];
	const size_t ij[2] = { i, j };
	double l = length(c);
	struct vec2 d = { (c->b.x - c->a.x) / l, (c->b.y - c->a.y) / l };
	double s = ((a->c.x - c->a.x) * d.x) + ((a->c.y - c->a.y) * d.y);
	struct vec2 foot = { c->a.x + (s * d.x), c->a.y + (s * d.y) };
	double h2 = (a->r * a->r) - pow(dist2(foot, a->c), 2);

	if (h2 > -2 * a->r * NEAR) {
		cut_pair(sc, ij, foot, d, sqrt(fmax(h2, 0)));
	}
}

// Cuts two arcs where their circles meet.
static void meet_arcs(struct scene *sc, size_t i, size_t j)
{
	const struct curve *a = &sc->c[i];
	const struct curve *b = &sc->c[j];
	const size_t ij[2] = { i, j };
	double d = dist2(a->c, b->c);
	struct vec2 u = { (b->c.x - a->c.x) / d, (b->c.y - a->c.y) / d };
	struct vec2 n = { -u.y, u.x };
	double s = ((d * d) + (a->r * a->r) - (b->r * b->r)) / (2 * d);
	double h2 = (a->r * a->r) - (s * s);

	if (d > NEAR && h2 > -2 * a->r * NEAR) {
		cut_pair(sc, ij, pt2(a->c.x + (s * u.x), a->c.y + (s * u.y)), n,
			 sqrt(fmax(h2, 0)));
	}
}

// Cuts each pair of curves where they cross, and where an end of one
// lies on the other, which covers two curves that share a stretch.
static void cut_curves(struct scene *sc)
{
	for (size_t i = 0; i < sc->nc; i++) {
		for (size_t j = i + 1; j < sc->nc; j++) {
			const struct curve *c = &sc->c[i];
			const struct curve *e = &sc->c[j];

			if (c->box.lo.x > e->box.hi.x ||
			    e->box.lo.x > c->box.hi.x ||
			    c->box.lo.y > e->box.hi.y ||
			    e->box.lo.y > c->box.hi.y) {
				continue;
			}
			if (!c->arc && !e->arc) {
				meet_lines(sc, i, j);
			} else if (c->arc && e->arc) {
				meet_arcs(sc, i, j);
			} else {
				meet_line_arc(sc, c->arc ? j : i,
					      c->arc ? i : j);
			}
			cut_at(sc, i, j, at(c, 0));
			cut_at(sc, i, j, at(c, 1));
			cut_at(sc, i, j, at(e, 0));
			cut_at(sc, i, j, at(e, 1));
		}
	}
}

static int cmp_split(const void *lhs, const void *rhs)
{
	const struct split *a = lhs;
	const struct split *b = rhs;

	if (a->c != b->c) {
		return a->c < b->c ? -1 : 1;
	}
	return (a->t > b->t) - (a->t < b->t);
}

static void add_piece(struct scene *sc, size_t c, struct span t)
{
	struct piece pc = { c, t.lo, t.hi, { 0, 0 }, { 0, 0 }, { 0, 0 }, 0, 0 };

	if ((t.hi - t.lo) * length(&sc->c[c]) < 2 * NEAR) {
		return;
	}
	pc.p0 = at(&sc->c[c], t.lo);
	pc.pm = at(&sc->c[c], (t.lo + t.hi) / 2);
	pc.p1 = at(&sc->c[c], t.hi);
	PUSH(sc->pc, sc->np, sc->cappc, pc);
}

// Makes the pieces of each curve between its cuts.  A cut closer than
// SLIVER to the one before or to the end of the curve is none: no
// piece is that short.
static void make_pieces(struct scene *sc)
{
	size_t k = 0;

	qsort(sc->s, sc->ns, sizeof(*sc->s), cmp_split);
	for (size_t c = 0; c < sc->nc; c++) {
		struct span t = { 0, 0 };
		double l = length(&sc->c[c]);

		for (; k < sc->ns && sc->s[k].c == c; k++) {
			if (sliver((sc->s[k].t - t.lo) * l) ||
			    sliver((1 - sc->s[k].t) * l)) {
				continue;
			}
			t.hi = sc->s[k].t;
			add_piece(sc, c, t);
			t.lo = fmax(t.lo, t.hi);
		}
		t.hi = 1;
		add_piece(sc, c, t);
	}
}

// A point of the paper, as a point of a plane: (x, y) there, by the
// axes x and n cross x of face f.
static struct vec2 in_plane(const struct bface *f, struct vec p)
{
	struct vec d = sub(p, f->o);

	return pt2(dot(d, f->x), dot(d, cross(f->n, f->x)));
}

// Counts the crossings of the ray from q toward +x with use u of plane
// face f.
static int use_crossings(const struct scene *sc, const struct bface *f,
			 const struct buse *u, struct vec2 q)
{
	const struct bedge *e = &sc->b.e[u->e];
	struct vec2 a = in_plane(f, sc->b.v[e->v0]);
	struct vec2 b = in_plane(f, sc->b.v[e->v1]);
	struct vec2 c = in_plane(f, e->c);
	double dy = q.y - c.y;
	double h = 0;
	int n = 0;

	if (e->type == C_LINE) {
		return (a.y <= q.y) != (b.y <= q.y) &&
		       a.x + ((q.y - a.y) / (b.y - a.y) * (b.x - a.x)) > q.x;
	}
	if (fabs(dy) >= e->r) {
		return 0;
	}
	h = sqrt((e->r * e->r) - (dy * dy));
	if (dot(e->n, f->n) < 0) {
		a = b;
	}
	for (int sign = -1; sign <= 1; sign += 2) {
		double turn =
			wrap(atan2(dy, sign * h) - atan2(a.y - c.y, a.x - c.x));

		n += c.x + (sign * h) > q.x && turn < e->sweep;
	}
	return n;
}

// Whether the point x of the plane of face f lies on the face.
static int on_plane_face(const struct scene *sc, const struct bface *f,
			 struct vec x)
{
	struct vec2 q = in_plane(f, x);
	int n = 0;

	for (size_t l = 0; l < f->nloops; l++) {
		const struct bloop *lp = &sc->b.l[f->loop0 + l];

		for (size_t k = 0; k < lp->n; k++) {
			n += use_crossings(sc, f, &sc->b.u[lp->first + k], q);
		}
	}
	return n % 2;
}

// Whether the point x of the cylinder or cone of face f lies on the
// face: on a bend, by its flat point; on the wall of an arc or a hole,
// by its depth in the sheet and its angle.
static int on_round_face(const struct scene *sc, const struct bface *f,
			 struct vec x)
{
	const struct part *p = sc->p;
	struct vec q;

	if (f->from == F_HOLE) {
		const struct hole *h = &p->holes[f->idx];
		double zm = h->side > 0 ? -h->h : -p->t + h->h;
		int cone = f->type == S_CONE;
		double lo = (h->side > 0) == cone ? zm : -p->t;
		double hi = (h->side < 0) == cone ? zm : 0;

		finv(p, h->face, x, &q);
		return h->side ? q.z > lo && q.z < hi : q.z > -p->t && q.z < 0;
	}
	if (f->from == F_SIDE) {
		const struct edge *e = &p->edges[f->idx];
		struct arc_ang a = arc_angles(p, e);

		finv(p, e->lf != NONE ? e->lf : e->rf, x, &q);
		return q.z > -p->t && q.z < 0 &&
		       wrap(atan2(q.y - e->cy, q.x - e->cx) - a.a0) < a.da;
	}
	return finv(p, f->idx, x, &q) &&
	       point_in_face(p, f->idx, pt2(q.x, q.y));
}

// A line of sight: the point o of the paper at depth 0, and the depth
// beyond which a face hides.
struct ray {
	struct vec o;
	double z;
};

// Whether the plane of face f hides along ray r.
static int plane_hides(const struct scene *sc, const struct bface *f,
		       struct ray r)
{
	double c = dot(f->n, sc->vw.z);
	double d = 0;

	if (fabs(c) < TINY) {
		return 0;
	}
	d = dot(f->n, sub(f->o, r.o)) / c;
	return d > r.z && on_plane_face(sc, f, add(r.o, mul(sc->vw.z, d)));
}

// Whether the cylinder or cone of face f hides along ray r.  With h
// along the axis and the rest across it, the radius is r + r2 h.
static int round_hides(const struct scene *sc, const struct bface *f,
		       struct ray r)
{
	struct vec q = sub(r.o, f->o);
	double hq = dot(q, f->n);
	double hw = dot(sc->vw.z, f->n);
	struct vec qa = sub(q, mul(f->n, hq));
	struct vec wa = sub(sc->vw.z, mul(f->n, hw));
	double rq = f->r + (f->r2 * hq);
	double a = dot(wa, wa) - (f->r2 * f->r2 * hw * hw);
	double b = dot(qa, wa) - (f->r2 * rq * hw);
	double disc = (b * b) - (a * (dot(qa, qa) - (rq * rq)));

	if (fabs(a) < TINY || disc < 0) {
		return 0;
	}
	for (int sign = -1; sign <= 1; sign += 2) {
		double d = (-b + (sign * sqrt(disc))) / a;

		if (d > r.z &&
		    on_round_face(sc, f, add(r.o, mul(sc->vw.z, d)))) {
			return 1;
		}
	}
	return 0;
}

// Whether material hides the point q of the paper at depth z.
static int covered(const struct scene *sc, struct vec2 q, double z)
{
	struct ray r = { add(mul(sc->vw.x, q.x), mul(sc->vw.y, q.y)),
			 z + FRONT };

	for (size_t i = 0; i < sc->b.nf; i++) {
		const struct bface *f = &sc->b.f[i];
		const struct box *x = &sc->fbox[i];

		if (q.x < x->lo.x || q.x > x->hi.x || q.y < x->lo.y ||
		    q.y > x->hi.y || x->hi.z < r.z) {
			continue;
		}
		if (f->type == S_PLANE ? plane_hides(sc, f, r) :
					 round_hides(sc, f, r)) {
			return 1;
		}
	}
	return 0;
}

// Finds the box of each face on the paper; a circle counts as its
// square.
static void face_boxes(struct scene *sc)
{
	const struct brep *b = &sc->b;

	sc->fbox = xmalloc((b->nf + 1) * sizeof(*sc->fbox));
	for (size_t i = 0; i < b->nf; i++) {
		sc->fbox[i] = box_empty();
	}
	for (size_t i = 0; i < b->ne; i++) {
		const struct bedge *e = &b->e[i];
		struct box x = box_empty();

		box_add(&x, to_view(&sc->vw, b->v[e->v0]));
		box_add(&x, to_view(&sc->vw, b->v[e->v1]));
		if (e->type == C_CIRCLE) {
			box_add(&x, to_view(&sc->vw, e->c));
			box_pad(&x, e->r);
		}
		for (int k = 0; k < 2; k++) {
			box_add(&sc->fbox[sc->ef[i][k]], x.lo);
			box_add(&sc->fbox[sc->ef[i][k]], x.hi);
		}
	}
}

// Marks each piece hidden if material covers both of its sides.  A
// piece on the rim of what covers it has one side free, and shows.
static void hide_pieces(struct scene *sc)
{
	for (size_t i = 0; i < sc->np; i++) {
		struct piece *pc = &sc->pc[i];
		const struct curve *c = &sc->c[pc->c];
		double tm = (pc->t0 + pc->t1) / 2;
		double l = c->arc ? c->r : length(c);
		struct vec2 n = { (c->b.y - c->a.y) / l,
				  (c->a.x - c->b.x) / l };
		double z = depth(c, tm);

		if (c->arc) {
			n = pt2((pc->pm.x - c->c.x) / l,
				(pc->pm.y - c->c.y) / l);
		}
		n = pt2(n.x * SIDE, n.y * SIDE);
		pc->hidden =
			covered(sc, pt2(pc->pm.x + n.x, pc->pm.y + n.y), z) &&
			covered(sc, pt2(pc->pm.x - n.x, pc->pm.y - n.y), z);
		if (pc->hidden) {
			sc->c[pc->c].part = 1;
		}
	}
}

// A piece, by its number, and the x of its middle.
struct ref {
	size_t i;
	double x;
};

static int cmp_ref(const void *lhs, const void *rhs)
{
	const struct ref *a = lhs;
	const struct ref *b = rhs;

	return (a->x > b->x) - (a->x < b->x);
}

// Whether two pieces are one line on the paper.
static int same_piece(const struct scene *sc, const struct piece *a,
		      const struct piece *b)
{
	return sc->c[a->c].arc == sc->c[b->c].arc &&
	       dist2(a->pm, b->pm) < SAME &&
	       ((dist2(a->p0, b->p0) < SAME && dist2(a->p1, b->p1) < SAME) ||
		(dist2(a->p0, b->p1) < SAME && dist2(a->p1, b->p0) < SAME));
}

// Of the pieces that are one line on the paper, keeps the best: a
// visible one before a hidden one, then the lowest kind.
static void drop_doubles(struct scene *sc)
{
	struct ref *v = xmalloc((sc->np + 1) * sizeof(*v));

	for (size_t i = 0; i < sc->np; i++) {
		v[i].i = i;
		v[i].x = sc->pc[i].pm.x;
	}
	qsort(v, sc->np, sizeof(*v), cmp_ref);
	for (size_t i = 0; i < sc->np; i++) {
		for (size_t j = i + 1; j < sc->np && v[j].x - v[i].x < SAME;
		     j++) {
			struct piece *a = &sc->pc[v[i].i];
			struct piece *b = &sc->pc[v[j].i];
			int ka = (2 * a->hidden) + (sc->c[a->c].kind >= K_BEND);
			int kb = (2 * b->hidden) + (sc->c[b->c].kind >= K_BEND);

			if (a->dead || b->dead || !same_piece(sc, a, b)) {
				continue;
			}
			if (ka == kb) {
				ka = sc->c[a->c].kind;
				kb = sc->c[b->c].kind;
			}
			(kb < ka || (kb == ka && b < a) ? a : b)->dead = 1;
		}
	}
	free(v);
}

// Prints the run of pieces first .. last of one curve as one record.
static void print_run(const struct scene *sc, const struct piece *first,
		      const struct piece *last, struct box *x)
{
	const struct curve *c = &sc->c[first->c];
	double a0 = wrap(c->a0 + (first->t0 * c->da)) / DEG;
	char vis = first->hidden ? 'h' : 'v';

	if (c->arc) {
		double da = (last->t1 - first->t0) * c->da / DEG;

		box_add(x, v3(first->p0.x, first->p0.y, 0));
		box_add(x, v3(last->p1.x, last->p1.y, 0));
		for (int k = 0; k < 4; k++) {
			double turn = wrap((k * PI / 2) - (a0 * DEG));

			if (turn < da * DEG) {
				box_add(x, v3(c->c.x + (c->r * cos(k * PI / 2)),
					      c->c.y + (c->r * sin(k * PI / 2)),
					      0));
			}
		}
		printf("arc %c %c %.6f %.6f %.6f %.6f %.6f\n",
		       kind_code[c->part && c->kind < K_HOLE ? K_HOLE : c->kind],
		       vis, zero(c->c.x), zero(c->c.y), c->r, zero(a0),
		       zero(a0) + da);
		return;
	}
	box_add(x, v3(first->p0.x, first->p0.y, 0));
	box_add(x, v3(last->p1.x, last->p1.y, 0));
	printf("line %c %c %.6f %.6f %.6f %.6f\n", kind_code[c->kind], vis,
	       zero(first->p0.x), zero(first->p0.y), zero(last->p1.x),
	       zero(last->p1.y));
}

// Prints the pieces, each run of one curve and one visibility as one
// record, and then the box of them all.
static void print_pieces(const struct scene *sc)
{
	struct box x = box_empty();
	size_t i = 0;

	while (i < sc->np) {
		const struct piece *a = &sc->pc[i];
		size_t j = i + 1;

		if (a->dead || (a->hidden && !sc->show_hidden)) {
			i++;
			continue;
		}
		while (j < sc->np && sc->pc[j].c == a->c && !sc->pc[j].dead &&
		       sc->pc[j].hidden == a->hidden &&
		       sc->pc[j].t0 - sc->pc[j - 1].t1 < TINY) {
			j++;
		}
		print_run(sc, a, &sc->pc[j - 1], &x);
		i = j;
	}
	if (x.lo.x > x.hi.x) {
		die("%s: view %s: nothing to see", sc->p->name, sc->vw.name);
	}
	printf("box %.6f %.6f %.6f %.6f\n", zero(x.lo.x), zero(x.lo.y),
	       zero(x.hi.x), zero(x.hi.y));
}

// Prints loop lp of a plane face as a closed path.
static void fill_loop(const struct scene *sc, const struct bloop *lp)
{
	const struct brep *b = &sc->b;

	for (size_t k = 0; k < lp->n; k++) {
		const struct buse *u = &b->u[lp->first + k];
		const struct bedge *e = &b->e[u->e];
		struct vec s = to_view(&sc->vw, b->v[u->fwd ? e->v0 : e->v1]);
		struct vec t = to_view(&sc->vw, b->v[u->fwd ? e->v1 : e->v0]);
		struct vec c = to_view(&sc->vw, e->c);
		int ccw = (dot(e->n, sc->vw.z) > 0) == (u->fwd != 0);

		if (!k) {
			printf("m %.6f %.6f\n", zero(s.x), zero(s.y));
		}
		if (e->type == C_LINE) {
			printf("l %.6f %.6f\n", zero(t.x), zero(t.y));
		} else {
			double a0 = atan2(s.y - c.y, s.x - c.x) / DEG;

			printf("a %.6f %.6f %.6f %.6f %.6f\n", zero(c.x),
			       zero(c.y), e->r, zero(a0),
			       zero(a0) + ((ccw ? 1 : -1) * e->sweep / DEG));
		}
	}
	printf("z\n");
}

// Fills each plane face that is not seen edge on.  Back faces count
// too: what the faces cover together is what the part covers.
static void fill_planes(const struct scene *sc)
{
	for (size_t i = 0; i < sc->b.nf; i++) {
		const struct bface *f = &sc->b.f[i];

		if (f->type != S_PLANE || fabs(dot(f->n, sc->vw.z)) < TINY) {
			continue;
		}
		printf("fill\n");
		for (size_t l = 0; l < f->nloops; l++) {
			fill_loop(sc, &sc->b.l[f->loop0 + l]);
		}
	}
}

// A hole seen along its axis: its place on the paper, with its depth.
struct seen {
	struct vec at;
	double d, dc, ang;
	int near;
};

// Prints a record for each hole seen along its axis and not covered,
// one for each place: of two holes in line, the nearer one.
static void print_holes(const struct scene *sc)
{
	const struct part *p = sc->p;
	struct seen *v = xmalloc((p->nholes + 1) * sizeof(*v));
	size_t n = 0;

	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];
		double c = dot(fnormal(p, h->face), sc->vw.z);
		struct seen s = { to_view(&sc->vw,
					  fmap(p, h->face, h->cx, h->cy,
					       c > 0 ? 0 : -p->t)),
				  2 * h->r, 2 * h->rc,
				  2 * atan2(h->rc - h->r, h->h) / DEG,
				  (c > 0) == (h->side > 0) };
		size_t k = 0;

		if (fabs(c) < 1 - TINY || covered(sc, xy(s.at), s.at.z)) {
			continue;
		}
		while (k < n && dist2(xy(v[k].at), xy(s.at)) > SAME) {
			k++;
		}
		if (k == n || v[k].at.z < s.at.z) {
			v[k] = s;
		}
		n += k == n;
	}
	for (size_t k = 0; k < n; k++) {
		printf("hole %.6f %.6f %.6f", zero(v[k].at.x), zero(v[k].at.y),
		       v[k].d);
		if (v[k].dc > v[k].d) {
			printf(" csink %.6f %.3f %s", v[k].dc, v[k].ang,
			       v[k].near ? "near" : "far");
		}
		printf("\n");
	}
	free(v);
}

// Prints the view of part p.
static void print_view(struct scene *sc)
{
	printf("view %s %s\nthick %.6f\n", sc->vw.name, sc->p->name, sc->p->t);
	for (size_t i = 0; i < sc->radius->nmaps; i++) {
		if (sc->radius->maps[i].bend) {
			printf("radius %.6f\n", sc->radius->maps[i].r);
			break;
		}
	}
	brep_build(sc->p, &sc->b);
	edge_faces(sc);
	mark_cutouts(sc);
	face_boxes(sc);
	add_edges(sc);
	add_zones(sc);
	add_slabs(sc);
	fill_planes(sc);
	cut_curves(sc);
	make_pieces(sc);
	hide_pieces(sc);
	drop_doubles(sc);
	print_pieces(sc);
	print_holes(sc);
	brep_free(&sc->b);
	free(sc->c);
	free(sc->s);
	free(sc->pc);
	free(sc->fbox);
	free(sc->ef);
	free(sc->inner);
}

// Prints the centre line of bend face f of the flat part, across the
// face along the bend.
static void bend_line(const struct part *p, size_t f)
{
	const struct map *m = &p->maps[p->faces[f]];
	struct span s = { HUGE_VAL, -HUGE_VAL };
	struct vec2 a;
	struct vec2 b;

	for (size_t k = p->fe_start[f]; k < p->fe_start[f + 1]; k++) {
		const struct edge *e = &p->edges[p->fe[k]];

		s.lo = fmin(s.lo, zone_s(m, p->verts[e->v0]));
		s.hi = fmax(s.hi, zone_s(m, p->verts[e->v0]));
		s.lo = fmin(s.lo, zone_s(m, p->verts[e->v1]));
		s.hi = fmax(s.hi, zone_s(m, p->verts[e->v1]));
	}
	a = zone_pt(m, s.lo, m->ba / 2);
	b = zone_pt(m, s.hi, m->ba / 2);
	printf("line c v %.6f %.6f %.6f %.6f\n", zero(a.x), zero(a.y),
	       zero(b.x), zero(b.y));
	printf("bend %.6f %.6f %.6f %.6f %g\n", zero(a.x), zero(a.y), zero(b.x),
	       zero(b.y), m->ang);
}

// Prints the flat view of p: the top view of the blank, every face in
// the plane z = 0, and the centre lines of the bends.
static void print_flat(const struct part *p, int show_hidden)
{
	struct part flat = *p;
	struct map one;
	struct scene sc;

	memset(&one, 0, sizeof(one));
	one.m = rig_id();
	flat.place = rig_id();
	flat.maps = &one;
	flat.nmaps = 1;
	flat.faces = xcalloc(p->nfaces + 1, sizeof(*flat.faces));
	memset(&sc, 0, sizeof(sc));
	sc.show_hidden = show_hidden;
	sc.p = &flat;
	sc.radius = p;
	sc.vw = views[4];
	sc.vw.name = "flat";
	print_view(&sc);
	for (size_t f = 0; f < p->nfaces; f++) {
		if (p->maps[p->faces[f]].bend) {
			bend_line(p, f);
		}
	}
	free(flat.faces);
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	struct scene sc;
	size_t n = 0;
	size_t k = 0;
	int a = 1;

	memset(&sc, 0, sizeof(sc));
	if (argc > 1 && strcmp(argv[1], "-h") == 0) {
		sc.show_hidden = 1;
		a++;
	}
	if (argc != a + 2 || argv[a][0] == '-') {
		die("usage: smview [-h] VIEW part.fold > part.view");
	}
	while (k < sizeof(views) / sizeof(*views) &&
	       strcmp(argv[a], views[k].name) != 0) {
		k++;
	}
	if (k == sizeof(views) / sizeof(*views) &&
	    strcmp(argv[a], "flat") != 0) {
		die("%s: the views are front, back, left, right, top, bottom "
		    "and flat",
		    argv[a]);
	}
	n = read_fold(argv[a + 1], &ps);
	if (n != 1) {
		die("%s: smview takes a file with one part", argv[a + 1]);
	}
	if (k == sizeof(views) / sizeof(*views)) {
		print_flat(&ps[0], sc.show_hidden);
	} else {
		sc.p = &ps[0];
		sc.radius = &ps[0];
		sc.vw = views[k];
		print_view(&sc);
	}
	free_parts(ps, n);
	close_stdout();
	return 0;
}
