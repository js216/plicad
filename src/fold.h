// SPDX-License-Identifier: GPL-3.0-only
// fold.h -- The folded part format: a flat subdivision with a map per face
// Copyright (c) 2026 Jakob Kastelic

#ifndef FOLD_H
#define FOLD_H

#include "geom.h"
#include <stddef.h>

struct shape; // The exact solid of a solid part (shape.h).

// A part is a folded blank or an exact solid (shape.h).  A blank is a
// flat planar subdivision, cut into faces along the edges of the bend
// zones.  Each face has a map from flat (x, y, z), with z in [-t, 0], to
// 3D.  The map of a slab is a rotation and a translation.  A bend zone
// is a full-width rectangle, and its map wraps it round the bend axis of
// its parent slab.  The solid is the image of the blank times [-t, 0],
// and the near face, z = 0, is the reference face.
//
// The text form, a part of a stream (stream.h), has one record per line,
// "-" for no face:
//   part NAME
//   place r00 r01 r02 r10 r11 r12 r20 r21 r22 tx ty tz
//   base MATERIAL    (optional; a material of the stream)
//   coat MATERIAL near|far edges|- N X0 Y0 ... XN-1 YN-1
//                    (any number; a region of the blank, a polygon,
//                    that takes the material on one side, and on the
//                    cut edges in it with edges; the later over the
//                    earlier)
// then the blank:
//   t T
//   map I rigid r00 .. r22 tx ty tz
//   map I bend PARENT qx qy mx my ba angle r
//   face I MAP
//   vertex I X Y
//   edge I V0 V1 LEFT RIGHT [arc CX CY] [by FEATURE]
//   hole FACE CX CY R [csink RC H near|far] [blind D near|far] [by FEATURE]
// or the records of the solid (shape.h), in its own frame, which the
// place puts in the world; and last
//   end
// Each index I counts from 0 in its kind of record.  An arc runs
// counterclockwise from V0 to V1, and the face on the left of each edge
// sees its boundary counterclockwise.  A bend zone starts on the line
// through (qx, qy) perpendicular to the unit vector (mx, my), which points
// into the zone, and ba is its width.  The feature of an edge or a hole
// is the name of the statement of the source that made it.

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
	size_t by; // Its feature, or NONE.
};

// A hole, with an optional countersink of radius rc at a face of the
// sheet, h deep: side 1 at the near face (z = 0), -1 at the far one,
// and 0 for none.  A blind hole goes blind deep from the face of bside,
// 1 near or -1 far, and has no countersink; blind is 0 for a hole
// through the sheet.
struct hole {
	size_t face;
	double cx, cy, r;
	int side;
	double rc, h;
	size_t by; // Its feature, or NONE.
	double blind;
	int bside;
};

// A material: its name, its color and roughness from 0 to 1, and its
// density in g/cm3.
// A material, of a body or of a finish on its faces: its color and
// roughness, 0 to 1, its density in g/cm3, and as a finish, the texture
// of a mold (a callout, from its grade on the VDI 3400 scale if vdi is
// not below 0), the least draft that it needs, and the letter that marks
// it on a drawing.
enum { CALLOUT_LEN = 128 };

struct material {
	char *name;
	double rgb[3], rough, density;
	double draft; // In degrees, 0 for none.
	double vdi;
	char mark; // A capital letter, or 0.
	char callout[CALLOUT_LEN]; // Empty for no texture.
};

// A coat: a region of the flat blank, a polygon of n corners
// counterclockwise or not, on one side of the sheet (1 the near face,
// -1 the far one) and on the cut edges of the blank in it if edges,
// that takes the material of the name.
struct coat {
	char *material;
	int side, edges;
	struct vec2 *pt;
	size_t n;
};

// A finish on faces of a solid: the material of the finish, and the
// references of the faces it takes (stream_face), sets of faces
// written out as their own references.
struct face_finish {
	char *material;
	struct str *refs;
	size_t n;
};

// A thread in the holes of a solid, whose body has them at the tap
// drill: the reference of the faces of the holes (stream_face), the
// designation of the thread, as 4-40 UNC-2B, and its depth from the
// mouth of the hole in mm, or 0 for that of the hole.
struct face_thread {
	char *ref, *spec;
	double deep;
};

// An insert in a hole of a folded part: the name of its type, the side
// of the sheet that its body stands on, 1 the near one and -1 the far
// one, its hole, the diameter and the height of its body, which may be 0
// for a flush one, whether its body fills its hole too (flush, or in a
// blind hole), the material of its body or NULL, and its name.  The
// stream holds all of it, so that no reader needs the table of types.
struct part_insert {
	char *type, *name;
	int side;
	size_t hole;
	double body_d, body_h;
	int fills;
	char *material;
};

#define DENSITY_MAX 30.0 // The densest material, in g/cm3; osmium is 22.6.

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
	char *base; // The name of its material, or NULL.
	double rgb[3], rough; // The color, 0 to 1, and its roughness.
	double density; // The density of its material, in g/cm3.
	struct shape *solid; // The solid of a solid part, or NULL.
	struct coat *coats; // The coats, the later over the earlier.
	size_t ncoats, capcoats;
	struct material *mats; // Its material and those of its coats.
	size_t nmats, capmats;
	struct str *feats; // The names of the features of its edges, holes.
	size_t nfeats, capfeats;
	struct face_finish *fins; // The finishes of its faces, the later
	size_t nfins, capfins; // over the earlier, of a solid.
	struct face_thread *threads; // The threads of its holes, the later
	size_t nthreads, capthreads; // over the earlier, of a solid.
	struct part_insert *ins; // The inserts in its holes.
	size_t nins, capins;
	int body; // Whether it is the body of an insert of the part before.
	int bodied; // Whether the bodies of its inserts follow it.
};

// The material of the name of part p, or NULL.
const struct material *part_material(const struct part *p, const char *name);
// Gives part p material m.
void set_base(struct part *p, const struct material *m);
// Whether materials a and b differ but in their names; none of their
// numbers is NaN.
int materials_differ(const struct material *a, const struct material *b);
// The index of the feature of the name in p, added if new.
size_t part_feature(struct part *p, const char *name);

void free_finish(struct face_finish *f);
void free_thread(struct face_thread *t);

// A part of the name, at home, with nothing in it.
struct part new_part(const char *name);
// Reads one record of part p from line, as where says, and returns 0 at
// its end, which checks it.
int part_line(struct part *p, char *line, const char *where);
// Reads a record "material NAME R G B ROUGHNESS DENSITY [FINISH]" from
// line.
void read_material(struct material *m, char *line, const char *where);
// The words of a line, and how to read a number of them.
struct finish_words {
	char **w;
	size_t n;
	const char *where;
	double (*num)(void *ctx, const char *word);
	void *ctx;
};

// Reads the finish of material m from the words after its density,
// [draft DEG] [mark LETTER] [vdi GRADE | callout TEXT...], and dies on a
// fault.
void material_finish(struct material *m, const struct finish_words *fw);
// Dies unless the color, roughness and density of m are in range.
void check_material(const struct material *m, const char *where);
// Frees what part p holds, and zeroes it.
void clear_part(struct part *p);
void free_parts(struct part *ps, size_t n);
// Writes part p, or material m, as a stream holds it.
void write_part(const struct part *p);
void write_material(const struct material *m);
// Dies unless the part is well formed.
void check_part(const struct part *p, const char *where);

// Maps a flat point of face f to the world.
struct vec fmap(const struct part *p, size_t f, double x, double y, double z);
// Maps a world point to the flat frame of face f.  Returns 0 off the
// surface of the face (for a bend, on the wrong half of the circle).
int finv(const struct part *p, size_t f, struct vec w, struct vec *flat);
// World normal of the near face of a slab face.
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
