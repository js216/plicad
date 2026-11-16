// SPDX-License-Identifier: GPL-3.0-only
// obj.h -- Read the triangles of Wavefront OBJ files
// Copyright (c) 2026 Jakob Kastelic

// An OBJ file holds the vertices (v X Y Z), the normals (vn X Y Z) and
// the faces (f A B C ...) of meshes, each corner of a face a vertex
// index, A/T, A/T/N or A//N, counting from 1 through the file, or back
// from the last one if negative.  A face of more corners than three
// turns into a fan of triangles.  Each o NAME or g NAME starts a part,
// and the comment # color R G B (tomesh(1)) gives its color.  A usemtl
// line starts a group of the faces of the part, and a # color R G B
// after it gives the color of the group, as tomesh writes the coats of a
// part, and a # texture GRADE after it that the group is a texture of
// that grade of VDI 3400 (tomesh(1)).  Other statements (vt, mtllib, s, l and the like) and comments
// go unread, so the files of other programs read as well.

#ifndef OBJ_H
#define OBJ_H

#include <stddef.h>

enum {
	FAULT_LEN = 4400, // Room for a message with a path in it.
	TRI_FLOATS = 9 // The coordinates of a triangle.
};

// Formats the message of a fault into e.
#define fault_at(e, ...) (void)snprintf((e)->msg, sizeof((e)->msg), __VA_ARGS__)

// The message of a failed read.
struct fault {
	char msg[FAULT_LEN];
};

// A part of an OBJ file: its name, its color if it has one, and its
// triangles, nine coordinates each in pos and the normals at their
// corners in nrm, the normal of the triangle where the file has none.
struct obj_part {
	char *name;
	int has_rgb;
	float rgb[3];
	float *pos, *nrm;
	size_t nt, cap;
	float *col; // The color of each triangle, or NULL for that of the
		// part; a color below 0 is that of the part.
	float *grade; // The grade of the texture of each triangle, below 0
		// for none, or NULL for none at all.
};

struct obj_file {
	struct obj_part *p;
	size_t n, cap;
};

// Reads the OBJ file at path into f, or returns -1 with the fault in e.
int obj_read(const char *path, struct obj_file *f, struct fault *e);
void obj_free(struct obj_file *f);

// The whole file at path, NUL terminated, its size in n, or NULL with
// the fault in e.
char *slurp(const char *path, size_t *n, struct fault *e);

#endif
