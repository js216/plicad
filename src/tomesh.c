// SPDX-License-Identifier: GPL-3.0-only
// tomesh.c -- Write the triangles of parts as an OBJ file
// Copyright (c) 2026 Jakob Kastelic

// Usage: tomesh [-c CHORD] [-a DEG] [-n] STREAM|- > part.obj
//
// Writes each part of a stream (stream.h) as one object of a Wavefront
// OBJ file:
//   o NAME
//   # color R G B          the color of its material, each 0 to 1
//   v X Y Z                 a vertex, in mm, in the world
//   vn X Y Z                with -n, the normal of a face at a vertex
//   f A B C                 a triangle, counterclockwise from outside;
//                           A//N ... with -n
// The vertices count from 1 through the whole file.  No point of a
// triangle lies further than CHORD from its face (0.02 mm by default),
// and no triangle turns by more than DEG degrees (10).  The mesh of each
// solid is closed: its faces share the points of their edges (mesh.h).
// The triangles of a part with coats (fold.h) are cut along the coats
// (coat.h), and those of each material of a coat, or of the finish of
// faces of a solid (part.h), follow the others in a group of their own:
// usemtl MATERIAL, # color R G B if the stream gives the material, or
// for a texture the color of the body, a little darker, and the faces.

#include "coat.h"
#include "fold.h"
#include "geom.h"
#include "mesh.h"
#include "shape.h"
#include "stream.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHORD 0.02 // The chord by default, in mm.
#define ANGLE 10.0 // The angle by default, in degrees.
#define ANGLE_MAX 90.0 // The greatest angle a triangle may turn.
// The grade of VDI 3400 a texture of a callout alone takes, for its
// grain.
#define VDI_DEF 30.0
// The share of the color of the body that a texture shows.
#define TEXTURE_TINT 0.85

// The options and the vertices written so far.
struct out {
	struct mesh_opts o;
	int normals;
	size_t nv, nn;
};

// The normal n of a face at a point of triangle t, or that of t where
// the face has none, as at the apex of a cone.
static struct vec normal(const struct mesh *m, const struct mesh_tri *t,
			 struct vec n)
{
	struct vec a = m->v[t->v[0]];
	struct vec c = cross(sub(m->v[t->v[1]], a), sub(m->v[t->v[2]], a));

	if (len(n) > 1.0 / 2 && num_ok(n.x + n.y + n.z)) {
		return n;
	}
	return len(c) > 0 ? unit(c) : v3(0, 0, 1);
}

// A solid to write: its exact solid, name and color (or NULL), and for
// a folded part with coats, the part and the origins of the faces.
struct solid_in {
	const struct shape *sh;
	const char *name;
	const double *rgb;
	const struct part *p;
	const struct face_from *from;
};

// The finish of a triangle: the name of its material, NULL for the base.
struct tri_fin {
	const char *name;
};

// The normals written so far: for each vertex, the face of its last
// normal and the number of that normal.
struct normals {
	size_t *stamp, *norm;
	size_t n;
};

// Writes the triangles of m whose finish, fin, is name, NULL for the
// color of the part.
static void write_tris(const struct out *w, const struct solid_in *in,
		       const struct mesh *m, const struct tri_fin *fin,
		       const char *name, struct normals *nm)
{
	for (size_t i = 0; i < m->nt; i++) {
		const struct mesh_tri *t = &m->t[i];
		const struct sface *fc = &in->sh->f[t->face];
		const char *f = fin[i].name;

		if (!f != !name || (name && strcmp(f, name) != 0)) {
			continue;
		}
		for (int k = 0; k < 3 && w->normals; k++) {
			struct vec p = m->v[t->v[k]];
			struct vec n = normal(
				m, t, mul(surf_normal(&fc->s, p), fc->sense));

			if (nm->stamp[t->v[k]] == t->face) {
				continue;
			}
			nm->stamp[t->v[k]] = t->face;
			nm->norm[t->v[k]] = ++nm->n;
			printf("vn %.5f %.5f %.5f\n", n.x + 0.0, n.y + 0.0,
			       n.z + 0.0);
		}
		if (w->normals) {
			printf("f %zu//%zu %zu//%zu %zu//%zu\n",
			       w->nv + t->v[0] + 1, w->nn + nm->norm[t->v[0]],
			       w->nv + t->v[1] + 1, w->nn + nm->norm[t->v[1]],
			       w->nv + t->v[2] + 1, w->nn + nm->norm[t->v[2]]);
		} else {
			printf("f %zu %zu %zu\n", w->nv + t->v[0] + 1,
			       w->nv + t->v[1] + 1, w->nv + t->v[2] + 1);
		}
	}
}

// Writes the group of each material of the finishes fin of the
// triangles of m, after its usemtl and its color, if it has one.
static void write_coats(const struct out *w, const struct solid_in *in,
			const struct mesh *m, const struct tri_fin *fin,
			struct normals *nm)
{
	const struct part *p = in->p;

	for (size_t i = 0; p && i < m->nt; i++) {
		const char *name = fin[i].name;
		const struct material *f = name ? part_material(p, name) : NULL;
		int seen = !name;

		for (size_t k = 0; k < i && !seen; k++) {
			seen = fin[k].name && strcmp(fin[k].name, name) == 0;
		}
		if (seen) {
			continue;
		}
		printf("usemtl %s\n", name);
		if (f && f->callout[0]) {
			printf("# texture %.4g\n",
			       f->vdi > 0 ? f->vdi : VDI_DEF);
		}
		if (f && f->callout[0] && in->rgb) {
			printf("# color %.4f %.4f %.4f\n",
			       in->rgb[0] * TEXTURE_TINT,
			       in->rgb[1] * TEXTURE_TINT,
			       in->rgb[2] * TEXTURE_TINT);
		} else if (f && !f->callout[0]) {
			printf("# color %.4f %.4f %.4f\n", f->rgb[0], f->rgb[1],
			       f->rgb[2]);
		}
		write_tris(w, in, m, fin, name, nm);
	}
}

// The finish of each triangle of m of the solid in: the material of its
// coat, or of the finish of its face, or NULL for the base; a new array.
static struct tri_fin *finishes(const struct solid_in *in, const struct mesh *m,
				const size_t *coat)
{
	struct tri_fin *fin = xmalloc((m->nt + 1) * sizeof(*fin));
	const struct part *p = in->p;

	for (size_t i = 0; i < m->nt; i++) {
		const char *name = in->sh->f[m->t[i].face].name;
		const char *f = NULL;

		if (coat && coat[i] != NONE) {
			f = p->coats[coat[i]].material;
		} else if (p && p->nfins) {
			f = finish_of(p->fins, p->nfins, name);
		}
		if (f && p->base && strcmp(f, p->base) == 0) {
			f = NULL;
		}
		fin[i].name = f;
	}
	return fin;
}

// Writes the mesh of a solid, its coats in groups of their own.
static void write_solid(struct out *w, const struct solid_in *in)
{
	struct mesh m;
	struct normals nm = { NULL, NULL, 0 };
	size_t *coat = NULL;
	struct tri_fin *fin = NULL;

	mesh_init(&m);
	mesh_shape(in->sh, w->o, &m);
	if (in->p && in->p->ncoats && in->from) {
		coat_mesh(in->p, in->from, &m, &coat);
	}
	fin = finishes(in, &m, coat);
	printf("o %s\n", in->name);
	if (in->rgb) {
		printf("# color %.4f %.4f %.4f\n", in->rgb[0], in->rgb[1],
		       in->rgb[2]);
	}
	for (size_t i = 0; i < m.nv; i++) {
		printf("v %.5f %.5f %.5f\n", m.v[i].x + 0.0, m.v[i].y + 0.0,
		       m.v[i].z + 0.0);
	}
	nm.stamp = xmalloc((m.nv + 1) * sizeof(size_t));
	nm.norm = xmalloc((m.nv + 1) * sizeof(size_t));
	for (size_t i = 0; i < m.nv; i++) {
		nm.stamp[i] = NONE;
	}
	write_tris(w, in, &m, fin, NULL, &nm);
	write_coats(w, in, &m, fin, &nm);
	w->nv += m.nv;
	w->nn += nm.n;
	free(nm.stamp);
	free(nm.norm);
	free(coat);
	free(fin);
	mesh_free(&m);
}

static double opt_num(const char *s, double lo, double hi)
{
	char *end = NULL;
	double x = strtod(s, &end);

	if (end == s || *end || !(x > lo) || !(x <= hi)) {
		die("tomesh: bad number %s", s);
	}
	return x;
}

int main(int argc, char **argv)
{
	struct out w = { { CHORD, ANGLE }, 0, 0, 0 };
	int i = 1;

	for (; i < argc - 1 && argv[i][0] == '-' && argv[i][1]; i++) {
		if (!strcmp(argv[i], "-n")) {
			w.normals = 1;
		} else if (!strcmp(argv[i], "-c") && i + 2 < argc) {
			w.o.chord = opt_num(argv[++i], 0, HUGE_VAL);
		} else if (!strcmp(argv[i], "-a") && i + 2 < argc) {
			w.o.angle = opt_num(argv[++i], 0, ANGLE_MAX);
		} else {
			break;
		}
	}
	if (i != argc - 1) {
		die("usage: tomesh [-c CHORD] [-a DEG] [-n] STREAM|- > "
		    "part.obj");
	}
	{
		struct part *ps = NULL;
		size_t n = read_parts(argv[i], &ps);

		for (size_t k = 0; k < n; k++) {
			struct shape sh;
			struct face_from *from = NULL;
			struct solid_in in = { &sh, ps[k].name,
					       ps[k].base ? ps[k].rgb : NULL,
					       &ps[k], NULL };

			part_shape_from(&ps[k], &sh, &from);
			in.from = from;
			write_solid(&w, &in);
			free(from);
			shape_free(&sh);
		}
		free_parts(ps, n);
	}
	close_stdout();
	return 0;
}
