// SPDX-License-Identifier: GPL-3.0-only
// fold.h -- The folded part format: a flat subdivision with a map per face
// Copyright (c) 2026 Jakob Kastelic

#ifndef FOLD_H
#define FOLD_H

#include "geom.h"
#include <stddef.h>

// A part is a flat planar subdivision: the blank, cut into faces along
// the edges of the bend zones.  Each face has a map from flat (x, y, z),
// with z in [-t, 0], to 3D.  The map of a slab is a rotation and a
// translation.  A bend zone is a full-width rectangle, and its map wraps
// it round the bend axis of its parent slab.  The solid is the image of
// the blank times [-t, 0], and the top face, z = 0, is the reference
// face.
//
// The text form has one record per line, "-" for no face:
//   part NAME
//   t T
//   place r00 r01 r02 r10 r11 r12 r20 r21 r22 tx ty tz
//   colour NAME R G B ROUGHNESS     (optional; each from 0 to 1)
//   map I rigid r00 .. r22 tx ty tz
//   map I bend PARENT qx qy mx my ba angle r
//   face I MAP
//   vertex I X Y
//   edge I V0 V1 LEFT RIGHT [arc CX CY]
//   hole FACE CX CY R [csink RC H top|bottom]
//   end
// Each index I counts from 0 in its kind of record.  An arc runs
// counterclockwise from V0 to V1, and the face on the left of each edge
// sees its boundary counterclockwise.  A bend zone starts on the line
// through (qx, qy) perpendicular to the unit vector (mx, my), which points
// into the zone, and ba is its width.

struct map {
	int bend; // 0 for a slab, 1 for a bend.
	struct rigid m; // Slab: flat to part coordinates.
	size_t parent; // Bend: the map of its parent slab.
	double qx, qy, mx, my, ba, ang, r;
};

struct vec2 {
	double x, y;
};

struct edge {
	size_t v0, v1, lf, rf;
	int arc;
	double cx, cy;
};

// A hole, with an optional countersink of radius rc at a face of the
// sheet, h deep: side 1 at the top face (z = 0), -1 at the bottom one,
// and 0 for none.
struct hole {
	size_t face;
	double cx, cy, r;
	int side;
	double rc, h;
};

struct part {
	char *name;
	double t;
	struct rigid place; // Part to world, for assemblies.
	struct map *maps;
	size_t nmaps, capmaps;
	size_t *faces; // Face to map.
	size_t nfaces, capfaces;
	struct vec2 *verts;
	size_t nverts, capverts;
	struct edge *edges;
	size_t nedges, capedges;
	struct hole *holes;
	size_t nholes, capholes;
	// The edges of face f are fe[fe_start[f] .. fe_start[f + 1]].
	size_t *fe_start;
	size_t *fe;
	char *colour; // The name of its colour, or NULL.
	double rgb[3], rough; // The colour, 0 to 1, and its roughness.
};

// Reads the parts of a fold file; dies on a malformed one.
size_t read_fold(const char *path, struct part **out);
// Frees what part p holds, and zeroes it.
void clear_part(struct part *p);
void free_parts(struct part *ps, size_t n);
void write_fold(const struct part *p);
// Dies unless the part is well formed.
void check_part(const struct part *p, const char *where);

// Maps a flat point of face f to the world.
struct vec fmap(const struct part *p, size_t f, double x, double y, double z);
// Maps a world point to the flat frame of face f.  Returns 0 off the
// surface of the face (for a bend, on the wrong half of the circle).
int finv(const struct part *p, size_t f, struct vec w, struct vec *flat);
// World normal of the top of a slab face.
struct vec fnormal(const struct part *p, size_t f);
// World direction of the flat x axis of a slab face.
struct vec fxdir(const struct part *p, size_t f);
// Bend axis of a bend map: point o and direction a, in the world.
struct line3 bend_axis(const struct part *p, size_t mi);
// Height of the bend axis in the flat frame of the parent.
double bend_za(const struct part *p, const struct map *m);
// World transform of a slab map.
struct rigid map_rig(const struct part *p, size_t map);

// 2D geometry.
// The start angle a0, sweep da and radius r of an arc.
struct arc_ang {
	double a0, da, r;
};
struct arc_ang arc_angles(const struct part *p, const struct edge *e);
double edge_len2(const struct part *p, const struct edge *e);
// Area of face f, without its holes.
double face_area2(const struct part *p, size_t f);
int point_in_face(const struct part *p, size_t f, struct vec2 at);
// Is (x, y) inside the outline of face f, holes aside?
int point_in_outline(const struct part *p, size_t f, struct vec2 at);
// The radius of hole h at depth z in the sheet of p.
double hole_radius(const struct part *p, const struct hole *h, double z);
// Is the flat point q, at depth q.z in the sheet, in the material of face
// f, its holes with their countersinks aside?
int point_in_slab(const struct part *p, size_t f, struct vec q);

// Lists the edges of each face, for point_in_face.  Calls after a change
// of edges rebuild the lists.
void index_faces(struct part *p);

#endif
