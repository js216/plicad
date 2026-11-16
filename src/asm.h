// SPDX-License-Identifier: GPL-3.0-only
// asm.h -- The lines of an assembly file and the parts they place
// Copyright (c) 2026 Jakob Kastelic

// An assembly file (assembly(5)), in the language of plicad(7):
//   materials FILE
//   material NAME R G B ROUGHNESS DENSITY [FINISH]
//   step DIST DEG
//   clearance MM
//   part NAME FILE [PLACE] [material MATERIAL]
//        [path STEP... [step DIST DEG] | with NAME]
//   image NAME FILE on PART X0 Y0 Z0 X1 Y1 Z1 X2 Y2 Z2
// where PLACE and each STEP is one of
//   move DX DY DZ
//   turn x|y|z DEG [about X Y Z]
// FILE of a part is a stream, of a folded part (smfold) or of solids
// (fillet), or another assembly file, a subassembly, read the same way
// and placed as one unit.  The part takes the name NAME, or NAME/SUB for
// each part SUB of a file of more than one or of a subassembly, so that
// names nest (in1/box/lid), and the bodies of the inserts of a folded
// part are NAME/INSERT.  The moves and turns of the place apply in order,
// the turns about the origin unless they name a point; those of a path
// bring the part from its start to its place, the turns about the origin
// of the part at its place unless they name a point.  step and clearance
// hold for the paths of their file (assemble(1)).  A file that holds
// itself, through any chain of subassemblies, is refused with the chain.

#ifndef ASM_H
#define ASM_H

#include "fold.h"
#include "geom.h"
#include <stddef.h>

// A move by d, or a turn by deg degrees about the axis x, y or z (0, 1,
// 2) through at, or through the origin of the part at its place if
// !about.
struct asm_step {
	int turn, axis, about;
	struct vec d, at;
	double deg;
};

struct asm_steps {
	struct asm_step *s;
	size_t n, cap;
};

enum { ASM_PART, ASM_UNIT, ASM_END, ASM_IMAGE };

enum { ASM_CORNERS = 9 }; // The numbers of the corners of a picture.

// A line of an assembly, in the order of the files: a part line of a
// stream, the unit of a subassembly, whose lines follow up to its end, or
// a picture.  Its name in its file and under its units, its depth of
// units, its file and the mesh of its stream (MESHDIR/UNITS/NAME.obj),
// the color of its material if it has one, its place in the frame of its
// unit and in the world, its path and the step of it, the line it
// travels with, its parts in the assembly, and its line in the file.
struct asm_line {
	int kind, depth;
	char *name, *full, *file, *mesh;
	int has_rgb;
	double rgb[3];
	struct rigid m, world;
	struct asm_steps place, path;
	int has_path;
	double step[2]; // Its most move in mm and turn in degrees a sample,
	int has_step; // from its own path or its file, if given.
	double clearance; // That of its file, or 0 for none.
	char *with; // The name of the line in its unit, or NULL.
	size_t first, n; // Its parts, out[first .. first + n).
	size_t unit; // The unit it lies in, or NONE.
	char *where;
	char *on; // A picture: the line it lies on, and its corners.
	double corner[ASM_CORNERS];
};

struct assembly {
	struct asm_line *ln;
	size_t nln, capln;
	struct part *out;
	size_t nout, capout;
};

// An assembly file to read: its path, and the directory of its meshes
// for a scene, or NULL.
struct asm_file {
	const char *path, *meshdir;
};

// Reads the assembly file f into a; dies with the file and the line on
// a fault.
void asm_read(struct assembly *a, struct asm_file f);
void asm_free(struct assembly *a);
// The motion of step s of a path of line l, in the frame of its unit.
struct rigid asm_step_rig(const struct asm_line *l, const struct asm_step *s);

#endif
