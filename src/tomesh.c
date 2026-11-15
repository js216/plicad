// SPDX-License-Identifier: GPL-3.0-only
// tomesh.c -- Write the triangles of parts as an OBJ file
// Copyright (c) 2026 Jakob Kastelic

// Usage: tomesh [-c CHORD] [-a DEG] [-n] part.fold|STREAM|- > part.obj
//
// Writes each part of a fold file, or each solid of a stream (stream.h),
// as one object of a Wavefront OBJ file:
//   o NAME
//   # colour R G B          the colour of a placed part, each 0 to 1
//   v X Y Z                 a vertex, in mm, in the world
//   vn X Y Z                with -n, the normal of a face at a vertex
//   f A B C                 a triangle, counterclockwise from outside;
//                           A//N ... with -n
// The vertices count from 1 through the whole file.  No point of a
// triangle lies further than CHORD from its face (0.02 mm by default),
// and no triangle turns by more than DEG degrees (10).  The mesh of each
// solid is closed: its faces share the points of their edges (mesh.h).

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

// Writes the mesh of solid sh, named name, with its colour if rgb.
static void write_solid(struct out *w, const struct shape *sh, const char *name,
			const double *rgb)
{
	struct mesh m;
	size_t *stamp = NULL;
	size_t *norm = NULL;
	size_t nn = 0;

	mesh_init(&m);
	mesh_shape(sh, w->o, &m);
	printf("o %s\n", name);
	if (rgb) {
		printf("# colour %.4f %.4f %.4f\n", rgb[0], rgb[1], rgb[2]);
	}
	for (size_t i = 0; i < m.nv; i++) {
		printf("v %.5f %.5f %.5f\n", m.v[i].x + 0.0, m.v[i].y + 0.0,
		       m.v[i].z + 0.0);
	}
	stamp = xmalloc((m.nv + 1) * sizeof(size_t));
	norm = xmalloc((m.nv + 1) * sizeof(size_t));
	for (size_t i = 0; i < m.nv; i++) {
		stamp[i] = NONE;
	}
	for (size_t i = 0; i < m.nt; i++) {
		const struct mesh_tri *t = &m.t[i];
		const struct sface *fc = &sh->f[t->face];

		for (int k = 0; k < 3 && w->normals; k++) {
			struct vec p = m.v[t->v[k]];
			struct vec n = normal(
				&m, t, mul(surf_normal(&fc->s, p), fc->sense));

			if (stamp[t->v[k]] == t->face) {
				continue;
			}
			stamp[t->v[k]] = t->face;
			norm[t->v[k]] = ++nn;
			printf("vn %.5f %.5f %.5f\n", n.x + 0.0, n.y + 0.0,
			       n.z + 0.0);
		}
		if (w->normals) {
			printf("f %zu//%zu %zu//%zu %zu//%zu\n",
			       w->nv + t->v[0] + 1, w->nn + norm[t->v[0]],
			       w->nv + t->v[1] + 1, w->nn + norm[t->v[1]],
			       w->nv + t->v[2] + 1, w->nn + norm[t->v[2]]);
		} else {
			printf("f %zu %zu %zu\n", w->nv + t->v[0] + 1,
			       w->nv + t->v[1] + 1, w->nv + t->v[2] + 1);
		}
	}
	w->nv += m.nv;
	w->nn += nn;
	free(stamp);
	free(norm);
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
		die("usage: tomesh [-c CHORD] [-a DEG] [-n] "
		    "part.fold|STREAM|- > part.obj");
	}
	if (stream_file(argv[i])) {
		struct stream st;

		stream_read(&st, argv[i]);
		for (size_t k = 0; k < st.nsh; k++) {
			write_solid(&w, &st.sh[k], st.sh[k].name, NULL);
		}
		stream_free(&st);
	} else {
		struct part *ps = NULL;
		size_t n = read_fold(argv[i], &ps);

		for (size_t k = 0; k < n; k++) {
			struct shape sh;

			part_shape(&ps[k], &sh);
			write_solid(&w, &sh, ps[k].name,
				    ps[k].colour ? ps[k].rgb : NULL);
			shape_free(&sh);
		}
		free_parts(ps, n);
	}
	close_stdout();
	return 0;
}
