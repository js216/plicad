// SPDX-License-Identifier: GPL-3.0-only
// measure.c -- Print the volume, area, box, holes and flat size of parts
// Copyright (c) 2026 Jakob Kastelic

// Usage: measure [-e REL] STREAM|-
//
// For each folded part of a stream (stream.h) it prints one item per
// line:
//   part NAME
//   volume V
//   area A
//   bbox XMIN YMIN ZMIN XMAX YMAX ZMAX
//   faces N plane P cylinder C [cone K]
//   hole X Y Z d D axis AX AY AZ      (axis components without sign)
//   flat W H area A holes N bends B  (W >= H, the box of the blank)
//   mass M                            (in g, of a part with a material)
//   center X Y Z                      (of its mass)
// The faces line counts the cones only if there are any.  After the
// parts, for each unit of parts with a material, the parts named
// UNIT/..., a subassembly or a file of several parts (place(1)), outer
// units first, a line
//   unit UNIT mass M center X Y Z
// and if any part has a material, a line
//   total mass M center X Y Z
// give the mass and the center of mass of those that have one.  For
// each solid part, the lines are
//   part NAME
//   volume V
//   area A
//   center X Y Z                      of the volume
//   bbox XMIN YMIN ZMIN XMAX YMAX ZMAX
//   faces N plane P cylinder C cone K sphere S torus T
//   mass M                            (of a part with a material)
// without the center and the box for a solid of no faces, as a common
// part of solids apart leaves.  The volume, the area and the center of
// a part are worked out once for each shape of its own content, in its
// own frame, and moved to the place of each part of it: the parts of an
// assembly are mostly copies of a few.  After the records of each part,
//   error volume E area E center E
// gives the bounds of their errors (print_error): each integral to REL
// of the sizes of its terms, without -e a hundredth of the tightest
// tolerance of a length of the part over the diagonal of its box
// (part_accuracy), 1e-6 for a part with none, that of a face of a pipe
// or a sphere by its mesh, or inf where no bound is proven
// (shape_mass_to).

#include "brep.h"
#include "bsolid.h"
#include "expr.h"
#include "fold.h"
#include "geom.h"
#include "integ.h"
#include "isect.h"
#include "mesh.h"
#include "shape.h"
#include "stream.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The keys of a hole: its axis, diameter and center.
enum { K_AX, K_AY, K_AZ, K_D, K_CX, K_CY, K_CZ, K_N };
#define ROUND_TO 1000.0 // Holes sort by keys rounded to 1 / ROUND_TO.
#define ALONG 1e-9 // Sine below which a side runs along a bend axis.
#define MM3_PER_CM3 1000.0
#define ACCURACY 1e-6 // Of each integral, of a part of no tolerance (-e).
#define OF_TOLERANCE 0.01 // Of the tightest tolerance, the accuracy,
#define ACCURACY_MAX 0.01 // which stays below this.
#define ROUNDED 1e-12 // The rounding of a size in closed form, of it.

enum { STEPS = 256 }; // Points of a curve other than a circle, for its box.

struct hole_out {
	double k[K_N];
};

static double r3(double x)
{
	return floor((x * ROUND_TO) + MID) / ROUND_TO;
}

// Orders holes by axis, diameter and center, each rounded to 0.001.
static int cmp_hole(const void *lhs, const void *rhs)
{
	const struct hole_out *x = lhs;
	const struct hole_out *y = rhs;

	for (int i = 0; i < K_N; i++) {
		if (r3(x->k[i]) < r3(y->k[i])) {
			return -1;
		}
		if (r3(x->k[i]) > r3(y->k[i])) {
			return 1;
		}
	}
	return 0;
}

// Grows the box by the extreme points of a circular edge on each axis.
static void circle_box(const struct brep *b, const struct bedge *e,
		       struct box *x)
{
	struct vec u = unit(sub(b->v[e->v0], e->c));
	struct vec w = cross(e->n, u);

	for (int ax = 0; ax < 3; ax++) {
		double th = atan2(comp(w, ax), comp(u, ax));

		for (int k = 0; k < 2; k++) {
			double a = fmod(th + (k * PI) + (4 * PI), 2 * PI);
			struct vec d = add(mul(u, cos(a)), mul(w, sin(a)));

			if (a <= e->sweep) {
				box_add(x, add(e->c, mul(d, e->r)));
			}
		}
	}
}

// Adds the volume and the cap areas of 2D face f.  A bend zone is a
// full-width rectangle, ext long along the bend.
static void face_size(const struct part *p, size_t f, struct solid_size *sz)
{
	const struct map *m = &p->maps[p->faces[f]];
	double a2 = face_area2(p, f);
	double ext = m->bend ? a2 / m->ba : 0;
	double a = fabs(m->ang) * DEG;
	double r1 = m->r + p->t;

	if (!m->bend) {
		sz->vol += a2 * p->t;
		sz->area += 2 * a2;
		return;
	}
	sz->vol += ext * a / 2 * (r1 * r1 - m->r * m->r);
	sz->area += ext * a * (m->r + r1);
}

// Adds the area of the side along outline edge e.  On a bend face, a
// side across the bend is an end cap of the bend, or of the share of
// it that the edge spans (an edge breaks where its feature changes),
// and a side along its axis is a flat strip, a free edge of the zone.
static void side_size(const struct part *p, const struct edge *e, double *area)
{
	size_t f = e->lf != NONE ? e->lf : e->rf;
	const struct map *m = &p->maps[p->faces[f]];
	double r1 = m->r + p->t;
	struct vec2 a = p->verts[e->v0];
	struct vec2 b = p->verts[e->v1];
	double l = edge_len2(p, e);

	if (!m->bend ||
	    fabs(((b.x - a.x) * m->my) - ((b.y - a.y) * m->mx)) > ALONG * l) {
		*area += l * p->t;
	} else {
		*area += fabs(m->ang) * DEG / 2 * (r1 * r1 - m->r * m->r) *
			 fmin(1, l / m->ba);
	}
}

// Adds the wall area of hole h, and takes off what its countersink adds
// to the hole: the volume beyond the cylinder, and the ring of the face;
// or for a blind hole, whose disk the faces keep, its bore.
static void hole_size(const struct part *p, const struct hole *h,
		      struct solid_size *sz)
{
	double r = h->r;
	double rc = h->rc;

	if (h->blind > 0) {
		// The faces keep their disk: its floor takes the place of the
		// mouth, and the hole takes its bore.
		sz->area += 2 * PI * r * h->blind;
		sz->vol -= PI * r * r * h->blind;
		return;
	}
	if (!h->side) {
		sz->area += 2 * PI * r * p->t;
		return;
	}
	sz->area += (2 * PI * r * (p->t - h->h)) +
		    (PI * (r + rc) * hypot(h->h, rc - r)) -
		    (PI * ((rc * rc) - (r * r)));
	sz->vol -= (PI * h->h * ((rc * rc) + (rc * r) + (r * r)) / 3) -
		   (PI * r * r * h->h);
}

static struct solid_size print_size(const struct part *p)
{
	struct solid_size sz = { 0, 0 };

	for (size_t f = 0; f < p->nfaces; f++) {
		face_size(p, f, &sz);
	}
	for (size_t i = 0; i < p->nedges; i++) {
		if ((p->edges[i].lf == NONE) != (p->edges[i].rf == NONE)) {
			side_size(p, &p->edges[i], &sz.area);
		}
	}
	for (size_t i = 0; i < p->nholes; i++) {
		hole_size(p, &p->holes[i], &sz);
	}
	printf("volume %.4f\narea %.4f\n", sz.vol, sz.area);
	return sz;
}

// Prints the bounding box and the face counts of the solid.
static void print_box(const struct part *p)
{
	struct brep b;
	struct box x = box_empty();
	size_t planes = 0;
	size_t cones = 0;

	brep_build(p, &b);
	for (size_t i = 0; i < b.nv; i++) {
		box_add(&x, b.v[i]);
	}
	for (size_t i = 0; i < b.ne; i++) {
		if (b.e[i].type == C_CIRCLE) {
			circle_box(&b, &b.e[i], &x);
		}
	}
	for (size_t i = 0; i < b.nf; i++) {
		planes += b.f[i].type == S_PLANE;
		cones += b.f[i].type == S_CONE;
	}
	printf("bbox %.4f %.4f %.4f %.4f %.4f %.4f\n", x.lo.x + 0.0,
	       x.lo.y + 0.0, x.lo.z + 0.0, x.hi.x + 0.0, x.hi.y + 0.0,
	       x.hi.z + 0.0);
	printf("faces %zu plane %zu cylinder %zu", b.nf, planes,
	       b.nf - planes - cones);
	if (cones) {
		printf(" cone %zu", cones);
	}
	printf("\n");
	brep_free(&b);
}

static size_t sheet_holes(const struct part *p, struct hole_out **out)
{
	struct hole_out *hs = xmalloc((p->nholes + 1) * sizeof(*hs));

	for (size_t i = 0; i < p->nholes; i++) {
		const struct hole *h = &p->holes[i];
		struct vec c = fmap(p, h->face, h->cx, h->cy, -p->t / 2);
		struct vec n = fnormal(p, h->face);
		struct hole_out o = { { fabs(n.x), fabs(n.y), fabs(n.z),
					2 * h->r, c.x, c.y, c.z } };

		hs[i] = o;
	}
	*out = hs;
	return p->nholes;
}

static void print_holes(const struct part *p)
{
	struct hole_out *hs = NULL;
	size_t n = sheet_holes(p, &hs);

	qsort(hs, n, sizeof(*hs), cmp_hole);
	for (size_t i = 0; i < n; i++) {
		const double *k = hs[i].k;

		printf("hole %.4f %.4f %.4f d %.4f axis %.0f %.0f %.0f\n",
		       k[K_CX] + 0.0, k[K_CY] + 0.0, k[K_CZ] + 0.0, k[K_D],
		       k[K_AX], k[K_AY], k[K_AZ]);
	}
	free(hs);
}

static void print_flat(const struct part *p)
{
	struct box x = box_empty();
	double area = 0;
	size_t bends = 0;
	double w = 0;
	double h = 0;

	for (size_t i = 0; i < p->nverts; i++) {
		box_add(&x, v3(p->verts[i].x, p->verts[i].y, 0));
	}
	for (size_t f = 0; f < p->nfaces; f++) {
		area += face_area2(p, f);
	}
	for (size_t i = 0; i < p->nmaps; i++) {
		bends += p->maps[i].bend != 0;
	}
	w = x.hi.x - x.lo.x;
	h = x.hi.y - x.lo.y;
	printf("flat %.4f %.4f area %.3f holes %zu bends %zu\n", fmax(w, h),
	       fmin(w, h), area, p->nholes, bends);
}

// Grows x by the points of edge e: its ends and the points of a conic
// that lie furthest along an axis.
// The volume, area and moment of a shape in its own frame, by its key:
// the bytes of its numbers and of the names of its faces (key_of), and
// the hash of the key.
struct memo {
	char *key;
	size_t n;
	unsigned long hash;
	struct solid_mass m, err;
};

static double accuracy = ACCURACY; // Of each integral, of the part,
static int accuracy_set; // or for every part as -e says.

static struct memo *memos;
static size_t nmemos, capmemos;

// A key under way: its bytes, their number and room.
struct key {
	char *s;
	size_t n, cap;
};

static void put_bytes(struct key *k, const void *p, size_t n)
{
	if (k->n + n > k->cap) {
		k->cap = 2 * (k->n + n) + 64;
		k->s = xrealloc(k->s, k->cap);
	}
	memcpy(k->s + k->n, p, n);
	k->n += n;
}

static void put_double(struct key *k, double x)
{
	x += 0.0; // -0 as 0.
	put_bytes(k, &x, sizeof(x));
}

static void put_size(struct key *k, size_t x)
{
	put_bytes(k, &x, sizeof(x));
}

static void put_vec3(struct key *k, struct vec v)
{
	put_double(k, v.x);
	put_double(k, v.y);
	put_double(k, v.z);
}

// The key of shape sh: all that its mass depends on, field by field.
static struct key key_of(const struct shape *sh)
{
	struct key k = { NULL, 0, 0 };

	put_size(&k, sh->nv);
	put_size(&k, sh->ne);
	put_size(&k, sh->nf);
	for (size_t i = 0; i < sh->nv; i++) {
		put_vec3(&k, sh->v[i]);
	}
	for (size_t i = 0; i < sh->ne; i++) {
		const struct sedge *e = &sh->e[i];

		put_size(&k, (size_t)e->c.kind);
		put_vec3(&k, e->c.o);
		put_vec3(&k, e->c.n);
		put_vec3(&k, e->c.x);
		put_double(&k, e->c.a);
		put_double(&k, e->c.b);
		put_size(&k, e->v0);
		put_size(&k, e->v1);
		put_double(&k, e->t0);
		put_double(&k, e->t1);
	}
	for (size_t i = 0; i < sh->nf; i++) {
		const struct sface *f = &sh->f[i];

		put_size(&k, (size_t)f->s.kind);
		put_vec3(&k, f->s.o);
		put_vec3(&k, f->s.n);
		put_vec3(&k, f->s.x);
		put_double(&k, f->s.r);
		put_double(&k, f->s.r2);
		put_double(&k, f->s.r3);
		put_size(&k, (size_t)(f->sense + 1));
		put_size(&k, f->nloops);
		put_bytes(&k, f->name, strlen(f->name) + 1);
		for (size_t l = 0; l < f->nloops; l++) {
			const struct sloop *lp = &sh->l[f->loop0 + l];

			put_size(&k, lp->n);
			for (size_t j = 0; j < lp->n; j++) {
				const struct suse *u = &sh->u[lp->first + j];

				put_size(&k, u->e);
				put_size(&k, (size_t)u->fwd);
			}
		}
	}
	return k;
}

// The FNV-1a hash of the n bytes s.
static unsigned long hash_of(const char *s, size_t n)
{
	unsigned long h = 2166136261UL;

	for (size_t i = 0; i < n; i++) {
		h = (h ^ (unsigned char)s[i]) * 16777619UL;
	}
	return h;
}

// The volume, area and moment of shape sh in its own frame, worked out
// once for each content.
static struct solid_mass own_mass(const struct shape *sh,
				  struct solid_mass *err)
{
	struct key k = key_of(sh);
	struct memo m;

	put_double(&k, accuracy);
	m.hash = hash_of(k.s, k.n);
	for (size_t i = 0; i < nmemos; i++) {
		if (memos[i].hash == m.hash && memos[i].n == k.n &&
		    !memcmp(memos[i].key, k.s, k.n)) {
			free(k.s);
			*err = memos[i].err;
			return memos[i].m;
		}
	}
	m.key = k.s;
	m.n = k.n;
	m.m = shape_mass_to(sh, accuracy, &m.err);
	PUSH(memos, nmemos, capmemos, m);
	*err = m.err;
	return m.m;
}

// The relative accuracy of the integrals of part p: a hundredth of the
// least half range of a parameter of a length that it leans on, over
// the diagonal of the box of its vertices, or ACCURACY if it has none.
static double part_accuracy(const struct part *p)
{
	struct box b = box_empty();
	double tight = HUGE_VAL;

	for (size_t i = 0; i < p->rg.ntol; i++) {
		const struct part_tol *t = &p->rg.tol[i];

		if (t->kind == KIND_LENGTH && t->hi > t->lo) {
			tight = fmin(tight, (t->hi - t->lo) / 2);
		}
	}
	for (size_t i = 0; p->solid && i < p->solid->nv; i++) {
		box_add(&b, p->solid->v[i]);
	}
	for (size_t i = 0; !p->solid && i < p->nverts; i++) {
		box_add(&b, v3(p->verts[i].x, p->verts[i].y, 0));
	}
	if (!isfinite(tight) || !(b.hi.x >= b.lo.x) ||
	    !(len(sub(b.hi, b.lo)) > 0)) {
		return ACCURACY;
	}
	return fmin(OF_TOLERANCE * tight / len(sub(b.hi, b.lo)), ACCURACY_MAX);
}

// The volume, area and moment of part p, where it stands: of its shape
// in its own frame, moved by its place.
static struct solid_mass solid_mass_of(const struct part *p,
				       struct solid_mass *err)
{
	struct solid_mass m;

	if (p->solid) {
		m = own_mass(p->solid, err);
	} else {
		struct part own = *p;
		struct shape sh;

		own.place = rig_id();
		part_shape(&own, &sh);
		m = own_mass(&sh, err);
		shape_free(&sh);
	}
	if (m.volume != 0) {
		struct vec c = rig_pt(p->place, mul(m.moment, 1 / m.volume));

		m.moment = mul(c, m.volume);
	}
	return m;
}

// The mass of parts, in g, its moment about the origin, and the parts.
struct mass_sum {
	double mass;
	struct vec moment;
	size_t parts;
};

// Prints the mass of part p if it has a material, and the center of its
// mass for a folded part (measure prints that of a solid with its
// volume), and adds them to t.
static void print_mass(const struct part *p, struct solid_mass m,
		       struct mass_sum *t)
{
	double g = 0;
	struct vec c;

	if (!p->base) {
		return;
	}
	g = m.volume * p->density / MM3_PER_CM3;
	c = mul(m.moment, 1 / m.volume);
	printf("mass %.4f\n", g);
	if (!p->solid) {
		printf("center %.4f %.4f %.4f\n", c.x + 0.0, c.y + 0.0,
		       c.z + 0.0);
	}
	t->mass += g;
	t->moment = add(t->moment, mul(c, g));
	t->parts++;
}

// Prints solid part p, its shape sh where it stands.
static void print_shape(const struct shape *sh, struct solid_mass m)
{
	static const char *const names[SF_KINDS] = { "plane", "cylinder",
						     "cone",  "sphere",
						     "torus", "pipe" };
	struct box x = box_empty();
	size_t count[SF_KINDS] = { 0, 0, 0, 0, 0, 0 };
	struct vec c = mul(m.moment, 1 / m.volume);

	for (size_t i = 0; i < sh->ne; i++) {
		struct box e = edge_box3(&sh->e[i]);

		box_add(&x, e.lo);
		box_add(&x, e.hi);
	}
	for (size_t i = 0; i < sh->nf; i++) {
		count[sh->f[i].s.kind]++;
	}
	printf("part %s\nvolume %.4f\narea %.4f\n", sh->name, m.volume, m.area);
	if (sh->nf) {
		printf("center %.4f %.4f %.4f\n", c.x + 0.0, c.y + 0.0,
		       c.z + 0.0);
		printf("bbox %.4f %.4f %.4f %.4f %.4f %.4f\n", x.lo.x + 0.0,
		       x.lo.y + 0.0, x.lo.z + 0.0, x.hi.x + 0.0, x.hi.y + 0.0,
		       x.hi.z + 0.0);
	}
	printf("faces %zu", sh->nf);
	for (int k = 0; k < SF_KINDS; k++) {
		if (k != SF_PIPE || count[k]) {
			printf(" %s %zu", names[k], count[k]);
		}
	}
	printf("\n");
}

// Prints the bounds of the errors of the volume v, the area a and the
// center of the mass m whose bounds are e: of the center, along the
// axis of the largest, and 0 without a center.
static void print_error(double v, double a, struct solid_mass m,
			struct solid_mass e)
{
	double c = 0;

	if (fabs(m.volume) > e.volume) {
		struct vec x = mul(m.moment, 1 / m.volume);
		struct vec d = { e.moment.x + (fabs(x.x) * e.volume),
				 e.moment.y + (fabs(x.y) * e.volume),
				 e.moment.z + (fabs(x.z) * e.volume) };

		c = fmax(d.x, fmax(d.y, d.z)) / (fabs(m.volume) - e.volume);
	}
	printf("error volume %.3g area %.3g center %.3g\n", v, a, c);
}

// Prints the mass line of sum t, after the words head.
static void print_sum(const char *head, struct mass_sum t)
{
	struct vec c = t.mass > 0 ? mul(t.moment, 1 / t.mass) : v3(0, 0, 0);

	printf("%s mass %.4f center %.4f %.4f %.4f\n", head, t.mass, c.x + 0.0,
	       c.y + 0.0, c.z + 0.0);
}

// Whether the name of part p lies in the unit of the first k characters
// of the name u.
static int in_unit(const struct part *p, const char *u, size_t k)
{
	return strncmp(p->name, u, k) == 0 && p->name[k] == '/';
}

// Prints the unit lines of the n parts ps, whose masses are in m.
static void print_units(const struct part *ps, const struct mass_sum *m,
			size_t n)
{
	for (size_t i = 0; i < n; i++) {
		const char *u = ps[i].name;

		for (const char *sl = strchr(u, '/'); sl;
		     sl = strchr(sl + 1, '/')) {
			size_t k = (size_t)(sl - u);
			struct mass_sum t = { 0, v3(0, 0, 0), 0 };
			int first = 1;
			char *head = NULL;

			for (size_t j = 0; j < i && first; j++) {
				first = !in_unit(&ps[j], u, k);
			}
			for (size_t j = i; first && j < n; j++) {
				if (in_unit(&ps[j], u, k)) {
					t.mass += m[j].mass;
					t.moment = add(t.moment, m[j].moment);
					t.parts += m[j].parts;
				}
			}
			if (!first || !t.parts) {
				continue;
			}
			head = xmalloc(k + sizeof("unit "));
			memcpy(head, "unit ", sizeof("unit ") - 1);
			memcpy(head + sizeof("unit ") - 1, u, k);
			head[k + sizeof("unit ") - 1] = 0;
			print_sum(head, t);
			free(head);
		}
	}
}

int main(int argc, char **argv)
{
	struct part *ps = NULL;
	size_t n = 0;
	struct mass_sum total = { 0, v3(0, 0, 0), 0 };
	struct mass_sum *m = NULL;
	char *end = NULL;
	int a = 1;

	if (argc == 4 && !strcmp(argv[1], "-e")) {
		accuracy = strtod(argv[2], &end);
		if (*end || end == argv[2] || !(accuracy > 0) ||
		    !(accuracy < 1)) {
			die("measure: -e takes a relative accuracy above 0 and "
			    "under 1: %s",
			    argv[2]);
		}
		accuracy_set = 1;
		a = 3;
	}
	if (argc != a + 1) {
		die("usage: measure [-e REL] STREAM|-");
	}
	n = read_parts(argv[a], &ps);
	m = xmalloc((n + 1) * sizeof(*m));
	for (size_t i = 0; i < n; i++) {
		struct solid_mass mm = { 0, 0, { 0, 0, 0 } }, ee = mm;

		m[i] = (struct mass_sum){ 0, v3(0, 0, 0), 0 };
		accuracy = accuracy_set ? accuracy : part_accuracy(&ps[i]);
		if (ps[i].solid || ps[i].base) {
			mm = solid_mass_of(&ps[i], &ee);
		}
		if (ps[i].solid) {
			struct shape sh;

			part_shape(&ps[i], &sh);
			print_shape(&sh, mm);
			shape_free(&sh);
			print_mass(&ps[i], mm, &m[i]);
			print_error(ee.volume, ee.area, mm, ee);
		} else {
			struct solid_size sz;

			printf("part %s\n", ps[i].name);
			sz = print_size(&ps[i]);
			print_box(&ps[i]);
			print_holes(&ps[i]);
			print_flat(&ps[i]);
			print_mass(&ps[i], mm, &m[i]);
			print_error(fabs(sz.vol) * ROUNDED,
				    fabs(sz.area) * ROUNDED, mm, ee);
		}
		total.mass += m[i].mass;
		total.moment = add(total.moment, m[i].moment);
		total.parts += m[i].parts;
	}
	print_units(ps, m, n);
	if (total.parts) {
		print_sum("total", total);
	}
	free(m);
	for (size_t i = 0; i < nmemos; i++) {
		free(memos[i].key);
	}
	free(memos);
	free_parts(ps, n);
	close_stdout();
	return 0;
}
