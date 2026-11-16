// SPDX-License-Identifier: GPL-3.0-only
// insert.h -- Self-clinching hardware of the table of the library
// Copyright (c) 2026 Jakob Kastelic

// The table inserts of the library (plicad-lib(7)) has a row for each
// type of hardware:
//   NAME KIND HOLE SHEET EDGE THREAD BODY_D BODY_H [MATERIAL] [flush]
//       [blind DEPTH]
// its kind, nut, stud or standoff; the diameter of its mounting hole;
// the least sheet; the least distance from the center of the hole to an
// edge or a bend line; its thread; and the diameter and the height of
// its body, a cylinder that stands on the sheet over the hole, which
// takes the material if the row gives one.  A flush type fills its hole
// too, flush with the face on the other side, and may have no body
// over the sheet (BODY_H 0), as a flush nut; a flush stud has its post
// there.  A blind type sits in a hole DEPTH deep from the face it
// stands on, which leaves the other face whole, and fills it, as a
// stud with a concealed head.  A table inserts next to a source adds
// its types to those of the library, before them.

#ifndef INSERT_H
#define INSERT_H

#include "geom.h"
#include "shape.h"

enum { INSERT_TEXT = 64 }; // Room for a word of a row.

struct insert_type {
	char name[INSERT_TEXT], kind[INSERT_TEXT], thread[INSERT_TEXT];
	char material[INSERT_TEXT]; // Empty for none.
	double hole, sheet, edge, body_d, body_h;
	int flush;
	double blind; // The depth of a blind hole, 0 for a hole through.
};

// A type to look up: its name, and the file whose library holds it.
struct insert_ref {
	const char *name, *of;
};

// Finds the type of r in the table inserts next to the file of r, or
// else in that of the library, into t; returns 0, or -1 if neither has
// such a type.  Dies on a table it cannot read.
int insert_type(struct insert_ref r, struct insert_type *t);

enum { BAR_MAX = 2 }; // The most steps of a bar.

// A round bar of n steps along the unit direction n from the point o:
// step k has the radius r[k] from z[k] to z[k + 1] along n.
struct bar {
	struct vec o, n;
	size_t ns;
	double r[BAR_MAX], z[BAR_MAX + 1];
};

// The exact solid of bar b, of the name, into sh: for each step two half
// cylinders, the disks of its ends and the rings between steps.
void bar_shape(struct shape *sh, struct bar b, const char *name);

#endif
