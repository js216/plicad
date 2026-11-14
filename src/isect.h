// SPDX-License-Identifier: GPL-3.0-only
// isect.h -- Where curves, surfaces and faces of exact solids meet
// Copyright (c) 2026 Jakob Kastelic

#ifndef ISECT_H
#define ISECT_H

#include "geom.h"
#include "shape.h"
#include <stddef.h>

enum { ROOTS_MAX = 16 }; // The most points where a curve meets a surface.

// A plane n . p = d.
struct plane3 {
	struct vec n;
	double d;
};

// A piece of a curve from t0 to t1.
struct cpiece {
	struct curve c;
	double t0, t1;
};

// The parameters in [t0, t1] where the piece meets the plane, into t;
// returns their count, 0 if the piece lies in it.
size_t curve_plane(const struct cpiece *p, struct plane3 pl, double *t);
// The parameters where the piece meets surface s, into t, at most
// ROOTS_MAX; returns their count.  A piece on s has none.
size_t curve_surf(const struct cpiece *p, const struct surf *s, double *t);
// Whether the piece lies on surface s.
int curve_on_surf(const struct cpiece *p, const struct surf *s);

// How two surfaces meet.
enum { SS_NONE, SS_CURVES, SS_SAME, SS_NOT };
// The curves where surfaces a and b meet, into out (two at most), and
// SS_CURVES; or SS_NONE, SS_SAME for one surface, or SS_NOT where the
// curve is none that surf_surf knows: lines, circles, and the ellipses
// of a plane at a slant to a cylinder and of two cylinders of one
// radius.
int surf_surf(const struct surf *a, const struct surf *b, struct curve *out,
	      size_t *n);

// A face and the angles about its axis: u near ref.u, v near ref.v.
struct facemap {
	const struct shape *sh;
	size_t f;
	struct uv ref;
};

void facemap_init(struct facemap *m, const struct shape *sh, size_t f);
struct uv fm_uv(const struct facemap *m, struct vec p);

// The plane through p, a point of surface s, of the points of s with the
// parameter v of p, or near enough: the iso-line of a ray along u.
struct plane3 iso_plane(const struct surf *s, struct vec p);
// How a curve through at.o with tangent at.d on surface s crosses the
// plane pl of the points with the v of at.o: 1 or -1 as v grows or falls
// along it, 0 where it only touches the plane.
int cross_sign(const struct surf *s, struct plane3 pl, struct line3 at);

// The distance from p to edge e.
double edge_dist(const struct sedge *e, struct vec p);
// Whether p, a point of the surface of face f, lies in f or within
// SH_EPS of its edges.
int face_has(const struct shape *sh, size_t f, struct vec p);
// Whether p lies within SH_EPS of the face f: on its surface and in it.
int face_on(const struct shape *sh, size_t f, struct vec p);
// Whether p lies inside the solid sh; p must lie on none of its faces.
int solid_has(const struct shape *sh, struct vec p);
// The box of face f of sh, grown by SH_EPS.
struct box face_box(const struct shape *sh, size_t f);
// The box of an edge.
struct box edge_box3(const struct sedge *e);
// Whether the faces fa of a and fb of b are known to stay clear of
// each other, so that no curve where their surfaces meet cuts either:
// they lie on the two sides of a plane across an axis and reach it on
// their edges alone; both, made of straight lines, lie apart along an
// axis or a diagonal; one, made of straight lines, lies on one side of
// the plane of the other; one, on a plane, lies inside the cylinder or
// the cone of the other; or both lie on cylinders or cones with
// parallel axes whose circles stay clear of each other.
int faces_clear(const struct shape *a, size_t fa, const struct shape *b,
		size_t fb);

#endif
