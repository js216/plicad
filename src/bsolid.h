// SPDX-License-Identifier: GPL-3.0-only
// bsolid.h -- The shells of a boundary representation
// Copyright (c) 2026 Jakob Kastelic

#ifndef BSOLID_H
#define BSOLID_H

#include "brep.h"

// Numbers the shells of b, the sets of faces joined by edges, from 0
// into shell[face]; returns their count.
size_t brep_shells(const struct brep *b, size_t *shell);

// The area and the volume of a solid.
struct solid_size {
	double area, vol;
};

#endif
