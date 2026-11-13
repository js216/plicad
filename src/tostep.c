// SPDX-License-Identifier: GPL-3.0-only
// tostep.c -- Write parts as a STEP AP214 file of planes, cylinders and tori
// Copyright (c) 2026 Jakob Kastelic

// Usage: tostep part.fold > part.step
//        tostep STREAM|- > part.step
//
// Every part becomes a manifold solid in one shape representation.  Each
// part is in world coordinates, in mm, at its place.  A part with a
// colour gets an AP214 surface style in that colour.  From a stream
// (stream.h), the last tool of a pipe, it writes each solid, one solid
// for each shell, and refuses any source line no tool took, and a shell
// that is a void.  The faces are planes, cylinders, cones, spheres and
// tori.  An ellipse, and an arc of a circle on a sphere, go out as
// several edges on the one curve (pieces_of).

#include "brep.h"
#include "bsolid.h"
#include "fold.h"
#include "geom.h"
#include "integ.h"
#include "shape.h"
#include "stream.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	REAL_LEN = 40, // Room for the text of one real.
	HEAD_IDS = 13 // The entities of the head, #1 to #13.
};
#define ZERO 1e-13 // Reals closer to 0 than this are 0.
#define ARC_MAX (PI / 16) // The longest piece of an edge OCCT must fit.

// Prints a STEP real, which always has a decimal point.
static void real(double x)
{
	char b[REAL_LEN];
	char *e = NULL;
	double v = fabs(x) < ZERO ? 0.0 : x;

	xsnprintf(b, "%.12G", v);
	e = strchr(b, 'E');
	if (strchr(b, '.')) {
		printf("%s", b);
	} else if (e) {
		char tail[REAL_LEN];

		xsnprintf(tail, "%s", e);
		*e = 0;
		printf("%s.%s", b, tail);
	} else {
		printf("%s.", b);
	}
}

// Prints s as a STEP string: in quotes, with each quote doubled.
static void string(const char *s)
{
	printf("'");
	for (; *s; s++) {
		if (*s == '\'') {
			printf("'");
		}
		printf("%c", *s);
	}
	printf("'");
}

static unsigned long triple(unsigned long *id, const char *kind, struct vec p)
{
	printf("#%lu=%s('',(", ++*id, kind);
	real(p.x);
	printf(",");
	real(p.y);
	printf(",");
	real(p.z);
	printf("));\n");
	return *id;
}

// A placement: origin o, axis z and reference direction x.
struct axes {
	struct vec o, z, x;
};

static unsigned long placement(unsigned long *id, struct axes ax)
{
	unsigned long a = triple(id, "CARTESIAN_POINT", ax.o);
	unsigned long b = triple(id, "DIRECTION", unit(ax.z));
	unsigned long c = triple(id, "DIRECTION", unit(ax.x));

	printf("#%lu=AXIS2_PLACEMENT_3D('',#%lu,#%lu,#%lu);\n", ++*id, a, b, c);
	return *id;
}

static unsigned long line(unsigned long *id, const struct brep *b,
			  const struct bedge *e)
{
	struct vec d = sub(b->v[e->v1], b->v[e->v0]);
	unsigned long pt = triple(id, "CARTESIAN_POINT", b->v[e->v0]);
	unsigned long dir = triple(id, "DIRECTION", unit(d));

	printf("#%lu=VECTOR('',#%lu,", ++*id, dir);
	real(len(d));
	printf(");\n#%lu=LINE('',#%lu,#%lu);\n", *id + 1, pt, *id);
	return ++*id;
}

static unsigned long curve(unsigned long *id, const struct brep *b,
			   const struct bedge *e)
{
	struct axes pl;
	unsigned long ax = 0;

	if (e->type == C_LINE) {
		return line(id, b, e);
	}
	pl.o = e->c;
	pl.z = e->n;
	pl.x = sub(b->v[e->v0], e->c);
	ax = placement(id, pl);
	printf("#%lu=CIRCLE('',#%lu,", ++*id, ax);
	real(e->r);
	printf(");\n");
	return *id;
}

static unsigned long loop(unsigned long *id, const struct brep *b,
			  const struct bloop *lp, const unsigned long *eid)
{
	unsigned long first = *id + 1;

	for (size_t k = 0; k < lp->n; k++) {
		const struct buse *u = &b->u[lp->first + k];

		printf("#%lu=ORIENTED_EDGE('',*,*,#%lu,.%c.);\n", ++*id,
		       eid[u->e], u->fwd ? 'T' : 'F');
	}
	printf("#%lu=EDGE_LOOP('',(", ++*id);
	for (unsigned long k = 0; k < lp->n; k++) {
		printf("%s#%lu", k ? "," : "", first + k);
	}
	printf("));\n#%lu=FACE_BOUND('',#%lu,.T.);\n", *id + 1, *id);
	return ++*id;
}

static unsigned long face(unsigned long *id, const struct brep *b,
			  const struct bface *f, const unsigned long *eid)
{
	struct axes pl;
	unsigned long *bounds = xmalloc((f->nloops + 1) * sizeof(*bounds));
	unsigned long ax = 0;
	unsigned long s = 0;

	for (size_t l = 0; l < f->nloops; l++) {
		bounds[l] = loop(id, b, &b->l[f->loop0 + l], eid);
	}
	pl.o = f->o;
	pl.z = f->n;
	pl.x = f->x;
	ax = placement(id, pl);
	s = ++*id;
	if (f->type == S_PLANE) {
		printf("#%lu=PLANE('',#%lu);\n", s, ax);
	} else if (f->type == S_CYLINDER) {
		printf("#%lu=CYLINDRICAL_SURFACE('',#%lu,", s, ax);
		real(f->r);
		printf(");\n");
	} else {
		printf("#%lu=CONICAL_SURFACE('',#%lu,", s, ax);
		real(f->r);
		printf(",");
		real(atan(f->r2));
		printf(");\n");
	}
	printf("#%lu=ADVANCED_FACE('',(", ++*id);
	for (size_t l = 0; l < f->nloops; l++) {
		printf("%s#%lu", l ? "," : "", bounds[l]);
	}
	printf("),#%lu,.%c.);\n", s, f->sense > 0 ? 'T' : 'F');
	free(bounds);
	return *id;
}

// Prints the vertices and edges of b; eid[i] gets the id of edge i.
static void edges(unsigned long *id, const struct brep *b, unsigned long *eid)
{
	unsigned long *vid = xmalloc((b->nv + 1) * sizeof(*vid));

	for (size_t i = 0; i < b->nv; i++) {
		unsigned long pt = triple(id, "CARTESIAN_POINT", b->v[i]);

		printf("#%lu=VERTEX_POINT('',#%lu);\n", ++*id, pt);
		vid[i] = *id;
	}
	for (size_t i = 0; i < b->ne; i++) {
		unsigned long c = curve(id, b, &b->e[i]);

		printf("#%lu=EDGE_CURVE('',#%lu,#%lu,#%lu,.T.);\n", ++*id,
		       vid[b->e[i].v0], vid[b->e[i].v1], c);
		eid[i] = *id;
	}
	free(vid);
}

// A solid written, and its part.
struct solid_ref {
	unsigned long id;
	size_t part;
};

// The solids written so far.
struct solids {
	struct solid_ref *s;
	size_t n, cap;
};

// The faces of a brep: the shell of each, and the id it has in the file.
struct face_ids {
	const size_t *shell;
	const unsigned long *fid;
	size_t n;
};

// Prints the closed shell of the faces fs of shell sh, and returns its
// id.
static unsigned long shell_solid(unsigned long *id, const struct face_ids *fs,
				 size_t sh)
{
	int first = 1;

	printf("#%lu=CLOSED_SHELL('',(", ++*id);
	for (size_t i = 0; i < fs->n; i++) {
		if (fs->shell[i] == sh) {
			printf("%s#%lu", first ? "" : ",", fs->fid[i]);
			first = 0;
		}
	}
	printf("));\n");
	return *id;
}

// Prints part i of ps as one solid for each of its pieces, and adds them
// to sd.
static void solid(unsigned long *id, const struct part *ps, size_t i,
		  struct solids *sd)
{
	struct brep b;
	unsigned long *eid = NULL;
	unsigned long *fid = NULL;
	size_t *shell = NULL;
	size_t n = 0;
	struct face_ids fs;

	brep_build(&ps[i], &b);
	eid = xmalloc((b.ne + 1) * sizeof(*eid));
	fid = xmalloc((b.nf + 1) * sizeof(*fid));
	shell = xmalloc((b.nf + 1) * sizeof(*shell));
	n = brep_shells(&b, shell);
	edges(id, &b, eid);
	for (size_t f = 0; f < b.nf; f++) {
		fid[f] = face(id, &b, &b.f[f], eid);
	}
	fs.shell = shell;
	fs.fid = fid;
	fs.n = b.nf;
	for (size_t k = 0; k < n; k++) {
		unsigned long sh = shell_solid(id, &fs, k);
		struct solid_ref r = { 0, i };

		printf("#%lu=MANIFOLD_SOLID_BREP(", ++*id);
		string(ps[i].name);
		printf(",#%lu);\n", sh);
		r.id = *id;
		PUSH(sd->s, sd->n, sd->cap, r);
	}
	free(eid);
	free(fid);
	free(shell);
	brep_free(&b);
}

// Prints the AP214 style of solid s in the colour of part p, and returns
// the id of its STYLED_ITEM.
static unsigned long style(unsigned long *id, const struct part *p,
			   unsigned long s)
{
	printf("#%lu=COLOUR_RGB(", ++*id);
	string(p->colour);
	for (int i = 0; i < 3; i++) {
		printf(",");
		real(p->rgb[i]);
	}
	printf(");\n#%lu=FILL_AREA_STYLE_COLOUR('',#%lu);\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=FILL_AREA_STYLE('',(#%lu));\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=SURFACE_STYLE_FILL_AREA(#%lu);\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=SURFACE_SIDE_STYLE('',(#%lu));\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=SURFACE_STYLE_USAGE(.BOTH.,#%lu);\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=PRESENTATION_STYLE_ASSIGNMENT((#%lu));\n", *id + 1, *id);
	(*id)++;
	printf("#%lu=STYLED_ITEM('color',(#%lu),#%lu);\n", *id + 1, *id, s);
	return ++*id;
}

static void header(unsigned long *id, const char *name)
{
	printf("ISO-10303-21;\nHEADER;\n"
	       "FILE_DESCRIPTION(('folded sheet metal'),'2;1');\n"
	       "FILE_NAME(");
	string(name);
	printf(",'',(''),(''),'tostep','tostep','');\n"
	       "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));"
	       "\nENDSEC;\nDATA;\n"
	       "#1=APPLICATION_CONTEXT('automotive design');\n"
	       "#2=APPLICATION_PROTOCOL_DEFINITION('international standard',"
	       "'automotive_design',2000,#1);\n"
	       "#3=PRODUCT_CONTEXT('',#1,'mechanical');\n"
	       "#4=PRODUCT(");
	string(name);
	printf(",");
	string(name);
	printf(",'',(#3));\n"
	       "#5=PRODUCT_DEFINITION_FORMATION('','',#4);\n"
	       "#6=PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design');\n"
	       "#7=PRODUCT_DEFINITION('design','',#5,#6);\n"
	       "#8=PRODUCT_DEFINITION_SHAPE('','',#7);\n"
	       "#9=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n"
	       "#10=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n"
	       "#11=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());\n"
	       "#12=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#9,"
	       "'distance_accuracy_value','');\n"
	       "#13=(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
	       "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#12))"
	       "GLOBAL_UNIT_ASSIGNED_CONTEXT((#9,#10,#11))"
	       "REPRESENTATION_CONTEXT('',''));\n");
	*id = HEAD_IDS;
}

// Prints the colours of the parts that have one, as one presentation.
static void styles(unsigned long *id, const struct part *ps,
		   const struct solids *sd)
{
	unsigned long *items = xmalloc((sd->n + 1) * sizeof(*items));
	size_t k = 0;

	for (size_t i = 0; i < sd->n; i++) {
		const struct part *p = &ps[sd->s[i].part];

		if (p->colour) {
			items[k++] = style(id, p, sd->s[i].id);
		}
	}
	if (k) {
		printf("#%lu=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_"
		       "REPRESENTATION('',(",
		       ++*id);
		for (size_t i = 0; i < k; i++) {
			printf("%s#%lu", i ? "," : "", items[i]);
		}
		printf("),#13);\n");
	}
	free(items);
}

// Prints curve c as a STEP curve, and returns its id.
static unsigned long shape_curve(unsigned long *id, const struct curve *c)
{
	static const char *const conic[CV_KINDS] = { "", "CIRCLE", "ELLIPSE" };
	struct axes pl = { c->o, c->n, c->x };
	unsigned long ax = 0;

	if (c->kind == CV_LINE) {
		unsigned long pt = triple(id, "CARTESIAN_POINT", c->o);
		unsigned long dir = triple(id, "DIRECTION", c->x);

		printf("#%lu=VECTOR('',#%lu,1.);\n#%lu=LINE('',#%lu,#%lu);\n",
		       *id + 1, dir, *id + 2, pt, *id + 1);
		*id += 2;
		return *id;
	}
	ax = placement(id, pl);
	printf("#%lu=%s('',#%lu,", ++*id, conic[c->kind], ax);
	real(c->a);
	if (c->kind == CV_ELLIPSE) {
		printf(",");
		real(c->b);
	}
	printf(");\n");
	return *id;
}

// Prints surface s as a STEP surface, and returns its id.  A cone that
// narrows along its axis turns round, for a positive half angle.
static unsigned long shape_surf(unsigned long *id, const struct surf *s)
{
	static const char *const names[SF_KINDS] = {
		"PLANE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE",
		"SPHERICAL_SURFACE", "TOROIDAL_SURFACE"
	};
	struct axes pl = { s->o, s->n, s->x };
	unsigned long ax = 0;

	if (s->kind == SF_CONE && s->r2 < 0) {
		pl.z = mul(s->n, -1);
	}
	ax = placement(id, pl);
	printf("#%lu=%s('',#%lu", ++*id, names[s->kind], ax);
	if (s->kind != SF_PLANE) {
		printf(",");
		real(s->r);
	}
	if (s->kind == SF_CONE) {
		printf(",");
		real(atan(fabs(s->r2)));
	} else if (s->kind == SF_TORUS) {
		printf(",");
		real(s->r2);
	}
	printf(");\n");
	return *id;
}

// The edge curves of an edge of a solid in the file: n of them, with
// ids from first on, in the way of the edge.
struct edge_ids {
	unsigned long first, n;
};

// The number of pieces that edge e of sh goes out in.  OCCT 7.8 knows
// the exact curve in the parameters of a face for a line, for a circle
// on a plane, a cylinder, a cone or a torus, and for a conic on a plane.
// For an ellipse on a cylinder (where two rounds end on each other) and
// for an arc of a circle on a sphere it fits a B-spline to the whole
// edge, up to 4e-6 mm off for a quarter turn, and leaves the tolerance
// of the edge at 1e-7 mm, so that its own check finds the edge off the
// face.  The misfit falls with about the fourth power of the length:
// in pieces of ARC_MAX at most it stays near 1e-9 mm.  The curve and
// its ends are the same; only the file has more edges.
static unsigned long pieces_of(const struct shape *sh, size_t e,
			       const int *on_sphere)
{
	const struct sedge *ed = &sh->e[e];
	double n = 1;

	if (ed->c.kind == CV_ELLIPSE ||
	    (ed->c.kind == CV_CIRCLE && on_sphere[e])) {
		n = ceil((ed->t1 - ed->t0) / ARC_MAX);
	}
	return n < 1 ? 1 : (unsigned long)n;
}

// Prints edge e of sh as its pieces, with new vertices between them,
// and returns their ids; vid holds the ids of the vertices.
static struct edge_ids shape_edge_out(unsigned long *id, const struct sedge *ed,
				      unsigned long n, const unsigned long *vid)
{
	unsigned long c = shape_curve(id, &ed->c);
	unsigned long prev = vid[ed->v0];
	struct edge_ids r = { 0, n };
	unsigned long mid0 = *id + 1;

	for (unsigned long j = 1; j < n; j++) {
		double t = ed->t0 + ((ed->t1 - ed->t0) * (double)j / (double)n);
		unsigned long pt =
			triple(id, "CARTESIAN_POINT", curve_at(&ed->c, t));

		printf("#%lu=VERTEX_POINT('',#%lu);\n", ++*id, pt);
	}
	r.first = *id + 1;
	for (unsigned long j = 0; j < n; j++) {
		unsigned long next = j + 1 < n ? mid0 + (2 * j) + 1 :
						 vid[ed->v1];

		printf("#%lu=EDGE_CURVE('',#%lu,#%lu,#%lu,.T.);\n", ++*id, prev,
		       next, c);
		prev = next;
	}
	return r;
}

static unsigned long shape_face_out(unsigned long *id, const struct shape *sh,
				    size_t fi, const struct edge_ids *eid)
{
	const struct sface *f = &sh->f[fi];
	unsigned long *bounds = xmalloc((f->nloops + 1) * sizeof(*bounds));
	unsigned long s = 0;

	for (size_t l = 0; l < f->nloops; l++) {
		const struct sloop *lp = &sh->l[f->loop0 + l];
		unsigned long first = *id + 1;
		unsigned long last = 0;

		for (size_t k = 0; k < lp->n; k++) {
			const struct suse *u = &sh->u[lp->first + k];
			const struct edge_ids *e = &eid[u->e];

			for (unsigned long j = 0; j < e->n; j++) {
				printf("#%lu=ORIENTED_EDGE('',*,*,#%lu,.%c.);\n",
				       ++*id,
				       e->first + (u->fwd ? j : e->n - 1 - j),
				       u->fwd ? 'T' : 'F');
			}
		}
		last = *id;
		printf("#%lu=EDGE_LOOP('',(", ++*id);
		for (unsigned long k = first; k <= last; k++) {
			printf("%s#%lu", k > first ? "," : "", k);
		}
		printf("));\n#%lu=FACE_BOUND('',#%lu,.T.);\n", *id + 1, *id);
		bounds[l] = ++*id;
	}
	s = shape_surf(id, &f->s);
	printf("#%lu=ADVANCED_FACE(", ++*id);
	string(f->name);
	printf(",(");
	for (size_t l = 0; l < f->nloops; l++) {
		printf("%s#%lu", l ? "," : "", bounds[l]);
	}
	printf("),#%lu,.%c.);\n", s, f->sense > 0 ? 'T' : 'F');
	free(bounds);
	return *id;
}

// Dies if a shell of sh, shell[f] the shell of face f, is a void: its
// volume is below 0.
static void refuse_voids(const struct shape *sh, const size_t *shell, size_t n)
{
	double *vol = xcalloc(n + 1, sizeof(*vol));

	for (size_t f = 0; f < sh->nf; f++) {
		vol[shell[f]] += face_mass(sh, f).volume;
	}
	for (size_t k = 0; k < n; k++) {
		if (vol[k] < 0) {
			die("tostep: solid %s has a void, which tostep does "
			    "not write",
			    sh->name);
		}
	}
	free(vol);
}

// Prints solid sh, one STEP solid for each shell, and adds them to sd.
static void shape_solid(unsigned long *id, const struct shape *sh,
			struct solids *sd)
{
	unsigned long *vid = xmalloc((sh->nv + 1) * sizeof(*vid));
	struct edge_ids *eid = xmalloc((sh->ne + 1) * sizeof(*eid));
	unsigned long *fid = xmalloc((sh->nf + 1) * sizeof(*fid));
	size_t *shell = xmalloc((sh->nf + 1) * sizeof(*shell));
	int *on_sphere = xcalloc(sh->ne + 1, sizeof(*on_sphere));
	size_t n = shape_shells(sh, shell);
	struct face_ids fs = { shell, fid, sh->nf };

	refuse_voids(sh, shell, n);
	for (size_t f = 0; f < sh->nf; f++) {
		struct urange r = face_uses(sh, f);

		for (size_t i = r.first; i < r.end; i++) {
			if (sh->f[f].s.kind == SF_SPHERE) {
				on_sphere[sh->u[i].e] = 1;
			}
		}
	}
	for (size_t i = 0; i < sh->nv; i++) {
		unsigned long pt = triple(id, "CARTESIAN_POINT", sh->v[i]);

		printf("#%lu=VERTEX_POINT('',#%lu);\n", ++*id, pt);
		vid[i] = *id;
	}
	for (size_t i = 0; i < sh->ne; i++) {
		eid[i] = shape_edge_out(id, &sh->e[i],
					pieces_of(sh, i, on_sphere), vid);
	}
	for (size_t f = 0; f < sh->nf; f++) {
		fid[f] = shape_face_out(id, sh, f, eid);
	}
	for (size_t k = 0; k < n; k++) {
		unsigned long shl = shell_solid(id, &fs, k);
		struct solid_ref r = { 0, 0 };

		printf("#%lu=MANIFOLD_SOLID_BREP(", ++*id);
		string(sh->name);
		printf(",#%lu);\n", shl);
		r.id = *id;
		PUSH(sd->s, sd->n, sd->cap, r);
	}
	free(vid);
	free(eid);
	free(fid);
	free(shell);
	free(on_sphere);
}

// Prints the shape representation of the solids sd.
static void representation(unsigned long *id, const char *name,
			   const struct solids *sd)
{
	struct axes pl = { v3(0, 0, 0), v3(0, 0, 1), v3(1, 0, 0) };
	unsigned long origin = placement(id, pl);

	printf("#%lu=ADVANCED_BREP_SHAPE_REPRESENTATION(", ++*id);
	string(name);
	printf(",(#%lu", origin);
	for (size_t i = 0; i < sd->n; i++) {
		printf(",#%lu", sd->s[i].id);
	}
	printf("),#13);\n#%lu=SHAPE_DEFINITION_REPRESENTATION(#8,#%lu);\n",
	       *id + 1, *id);
	(*id)++;
}

// Writes the solids of a stream.
static void stream_step(const char *path)
{
	struct stream st;
	struct solids sd = { NULL, 0, 0 };
	unsigned long id = 0;
	char *name = NULL;

	stream_read(&st, path);
	stream_refuse_left(&st);
	if (!st.nsh) {
		die("%s: no solid", path);
	}
	name = base_name(st.src);
	header(&id, name);
	for (size_t i = 0; i < st.nsh; i++) {
		shape_solid(&id, &st.sh[i], &sd);
	}
	representation(&id, name, &sd);
	printf("ENDSEC;\nEND-ISO-10303-21;\n");
	free(sd.s);
	free(name);
	stream_free(&st);
}

int main(int argc, char **argv)
{
	struct axes pl;
	struct part *ps = NULL;
	size_t n = 0;
	char *name = NULL;
	struct solids sd = { NULL, 0, 0 };
	unsigned long origin = 0;
	unsigned long id = 0;

	if (argc != 2) {
		die("usage: tostep part.fold|STREAM|- > part.step");
	}
	if (stream_file(argv[1])) {
		stream_step(argv[1]);
		close_stdout();
		return 0;
	}
	n = read_fold(argv[1], &ps);
	if (!n) {
		die("%s: no parts", argv[1]);
	}
	name = base_name(argv[1]);
	header(&id, name);
	for (size_t i = 0; i < n; i++) {
		solid(&id, ps, i, &sd);
	}
	pl.o = v3(0, 0, 0);
	pl.z = v3(0, 0, 1);
	pl.x = v3(1, 0, 0);
	origin = placement(&id, pl);
	printf("#%lu=ADVANCED_BREP_SHAPE_REPRESENTATION(", ++id);
	string(name);
	printf(",(#%lu", origin);
	for (size_t i = 0; i < sd.n; i++) {
		printf(",#%lu", sd.s[i].id);
	}
	printf("),#13);\n#%lu=SHAPE_DEFINITION_REPRESENTATION(#8,#%lu);\n",
	       id + 1, id);
	id++;
	styles(&id, ps, &sd);
	printf("ENDSEC;\nEND-ISO-10303-21;\n");
	free(sd.s);
	free(name);
	free_parts(ps, n);
	close_stdout();
	return 0;
}
