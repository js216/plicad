// SPDX-License-Identifier: GPL-3.0-only
// integ.h -- Volume, area and center of exact solids, by quadrature
// Copyright (c) 2026 Jakob Kastelic

#ifndef INTEG_H
#define INTEG_H

#include "geom.h"
#include "shape.h"

// The volume and area of a solid, and the integral of p over its
// volume: the center times the volume.
struct solid_mass {
	double volume, area;
	struct vec moment;
};

struct solid_mass shape_mass(const struct shape *sh);
// The share of face f of sh in its volume, area and moment.
struct solid_mass face_mass(const struct shape *sh, size_t f);
// The mass of sh, the bounds of its errors to *err if err is not NULL:
// of the rounding of each sum, 1e-12 of the sizes of its terms, of the
// rule along each ring of a pipe, its pieces halved until the bound is
// within rel of the sizes of its terms, and of the mesh of a face of a
// pipe or a sphere of three or four conics; infinite for any other face
// with an edge of no rule proven (integ.c).
struct solid_mass shape_mass_to(const struct shape *sh, double rel,
				struct solid_mass *err);

#endif
