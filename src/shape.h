// SPDX-License-Identifier: GPL-3.0-only
// shape.h -- Exact solids: vertices, edges on curves, faces on surfaces
// Copyright (c) 2026 Jakob Kastelic

// A shape is one solid, of one shell or more, with a name.  Its faces lie
// on planes, cylinders, cones, spheres, tori and pipes, its edges on
// lines, circles, ellipses and nodal sections of tori.  An edge runs
// along its curve from vertex
// v0 at parameter t0 to vertex v1 at t1 > t0; a closed edge has v0 == v1.
// A face is the part of its surface inside its loops.  The loops run
// counterclockwise about the outward normal of the face: the outer one
// round the face, the others round its holes, so that the face lies on
// their left, seen from outside.  The outward normal is the natural
// normal of the surface times sense.  Every edge has two uses, in two
// faces, the two ways round.  Each face has a name, for the rules of the
// tools that come after the one that made it.
//
// The text form, one record per line, in this order, inside a part of a
// stream (stream.h), between "part NAME" and "end":
//   v X Y Z                            vertex I, from 0
//   e V0 V1 T0 T1 CURVE                edge I, from 0
//   f NAME SENSE SURFACE               a face; its loops follow
//   l [+|-]E ...                       a loop of the face before
// CURVE is one of
//   line OX OY OZ XX XY XZ             o + t x
//   circle O N X R                     o + r (cos t x + sin t y)
//   ellipse O N X A B                  o + a cos t x + b sin t y
//   nodal O N X R RT                   o + (r - rt) x + u(t) y + rt sin t n
// with y = n x x, and SURFACE one of
//   plane O N X                        o + u x + v y
//   cylinder O N X R                   o + v n + r e(u)
//   cone O N X R TAN                   o + v n + (r + tan v) e(u)
//   sphere O N X R                     o + r (cos v e(u) + sin v n)
//   torus O N X R RT                   o + (r + rt cos v) e(u) + rt sin v n
//   pipe O N X A B RT                  c(u) + rt (cos v w(u) + sin v n)
// where e(u) = cos u x + sin u y, and O, N, X are three numbers each.
// A pipe is the surface of a ball of radius rt whose center runs along
// the ellipse c(u) = o + a cos u x + b sin u y; w(u) is the unit normal
// of the ellipse in its plane, away from o,
//   w(u) = (b cos u x + a sin u y) / sqrt(b^2 cos^2 u + a^2 sin^2 u).
// For a = b it is the torus o, n, x, a, rt.  A face of a pipe lies where
// the tube does not fold over itself, where -rt cos v stays under the
// least radius of curvature of the ellipse, min(a, b)^2 / max(a, b):
// there the point of the ellipse nearest a point of the face is c(u).
// A nodal section is where the torus O N X R RT meets the plane at r - rt
// along x, which touches its inner circle: a figure of eight whose node
// lies at t = 0, the point of touch, with
//   u(t) = sin(t/2) sqrt(2 rt (2 r - rt - rt cos t))
// for t from -2 pi to 2 pi, the half of y > 0 for t > 0.  Its distance
// from the axis is r - rt cos t.  It is no conic, and no rational curve.

#ifndef SHAPE_H
#define SHAPE_H

#include "geom.h"
#include <stddef.h>
#include <stdio.h>

#define SH_EPS 1e-7 // Points closer than this are one, in mm.

enum { CV_LINE, CV_CIRCLE, CV_ELLIPSE, CV_NODAL, CV_KINDS };
enum { SF_PLANE, SF_CYL, SF_CONE, SF_SPHERE, SF_TORUS, SF_PIPE, SF_KINDS };

// A curve: its kind, a frame o, n, x (n is 0 for a line), and its sizes.
struct curve {
	int kind;
	struct vec o, n, x;
	double a, b;
};

// A surface: its kind, a frame o, n, x, and its sizes: r and r2 as in
// the text form, and of a pipe r = a, r3 = b and r2 = rt, as the tube of
// a torus.
struct surf {
	int kind;
	struct vec o, n, x;
	double r, r2, r3;
};

// A point of a surface by its parameters.
struct uv {
	double u, v;
};

struct sedge {
	struct curve c;
	size_t v0, v1;
	double t0, t1;
};

struct suse {
	size_t e;
	int fwd;
};

struct sloop {
	size_t first, n;
};

struct sface {
	struct surf s;
	int sense;
	size_t loop0, nloops;
	char *name;
};

struct shape {
	char *name;
	struct vec *v;
	size_t nv, capv;
	struct sedge *e;
	size_t ne, cape;
	struct suse *u;
	size_t nu, capu;
	struct sloop *l;
	size_t nl, capl;
	struct sface *f;
	size_t nf, capf;
};

// The point of curve c at t, and its derivative there.
struct vec curve_at(const struct curve *c, double t);
struct vec curve_tan(const struct curve *c, double t);
// The parameter of the point p of curve c, the angle of a circle or an
// ellipse in (-pi, pi].
double curve_param(const struct curve *c, struct vec p);
// The parameter of the point of curve c nearest p, where the normal
// plane of c holds p: as curve_param, but for an ellipse, whose angle
// of p is no nearest point off it.
double curve_near(const struct curve *c, struct vec p);
// The distance from p to the whole curve c, where p lies near it.
double curve_dist(const struct curve *c, struct vec p);
// The second axis y = n x x of a frame.
struct vec frame_y(struct vec n, struct vec x);

struct vec surf_at(const struct surf *s, struct uv q);
// The parameters of the point p of surface s, u in (-pi, pi] about the
// axis.
struct uv surf_param(const struct surf *s, struct vec p);
// The natural normal of s at its point p, of unit length.
struct vec surf_normal(const struct surf *s, struct vec p);
// The signed distance from p to s, plus outside, minus inside (along
// the natural normal), exact for a plane, a cylinder and a sphere, and
// zero on the surface for every kind.
double surf_dist(const struct surf *s, struct vec p);

// The point c(u) of the ellipse of pipe s, and in d the unit normal
// w(u) of the ellipse there.
struct line3 pipe_ring(const struct surf *s, double u);
// The parameter u of the point of the ellipse of pipe s nearest p.
double pipe_foot(const struct surf *s, struct vec p);
// The least radius of curvature of the ellipse of pipe s.
double pipe_bend(const struct surf *s);

// The derivative of the parameter v of surface s along the direction
// t.d at its point t.o.
double surf_dv(const struct surf *s, struct line3 t);
// The angle a moved by whole turns to within half a turn of ref.
double angle_near(double a, double ref);
// The angle t moved by whole turns into [from, from + 2 pi).
double turn_from(double t, double from);
// Whether the parameter of curve c is an angle: it is no line.
int curve_turns(const struct curve *c);

// Builds a shape.  A vertex within SH_EPS of one there already is that
// one.
void shape_init(struct shape *sh, const char *name);
void shape_free(struct shape *sh);
// Makes to a copy of from, which stays as it is.
void shape_copy(struct shape *to, const struct shape *from);
// Moves sh by the rigid motion m, a turn and a move, no mirror.
void shape_move(struct shape *sh, struct rigid m);
size_t shape_vertex(struct shape *sh, struct vec p);
size_t shape_edge(struct shape *sh, struct sedge e);
void shape_face(struct shape *sh, struct surf s, int sense, const char *name);
void shape_use(struct shape *sh, size_t e, int fwd);
// Ends the loop of the uses added since first.
void shape_end_loop(struct shape *sh, size_t first);

// The edge of sh along curve c from t.lo to t.hi, made if new, with its
// vertices; *fwd gets whether it runs that way.  An edge is the same if
// it joins the same vertices through the same middle.
size_t shape_piece(struct shape *sh, const struct curve *c, struct span t,
		   int *fwd);

// The uses of the loops of face f: sh->u[first] up to sh->u[end - 1].
// The loops of a face come one after another, and so do their uses.
struct urange {
	size_t first, end;
};
struct urange face_uses(const struct shape *sh, size_t f);

// The point of an edge at the share s of its way from v0, s in [0, 1].
struct vec edge_at(const struct sedge *e, double s);

// Writes the records of the text form, those of a part of a stream
// (stream.h) between "part NAME" and "end".
void shape_write(FILE *out, const struct shape *sh);
// Reads a record of the text form from its line, which it splits;
// returns 1 at "end".  where names the line for errors.
int shape_record(struct shape *sh, char *line, const char *where);
// Dies unless every edge has two uses, the two ways round.
void shape_check(const struct shape *sh, const char *where);
// Numbers the shells of sh, the sets of faces joined by edges, from 0
// into shell[face]; returns their count.
size_t shape_shells(const struct shape *sh, size_t *shell);

#endif
