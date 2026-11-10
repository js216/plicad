// SPDX-License-Identifier: GPL-3.0-only
// tostep.c -- Write parts as a STEP AP214 file of planes, cylinders and tori
// Copyright (c) 2026 Jakob Kastelic

// Usage: tostep part.fold > part.step
//
// Every part becomes a manifold solid in one shape representation.  Each
// part is in world coordinates, in mm, at its place.  A part with a
// colour gets an AP214 surface style in that colour.

#include "brep.h"
#include "bsolid.h"
#include "fold.h"
#include "geom.h"
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
		die("usage: tostep part.fold > part.step");
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
