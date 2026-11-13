// SPDX-License-Identifier: GPL-3.0-only
// integ.h -- Volume, area and centre of exact solids, by quadrature
// Copyright (c) 2026 Jakob Kastelic

#ifndef INTEG_H
#define INTEG_H

#include "geom.h"
#include "shape.h"

// The volume and area of a solid, and the integral of p over its
// volume: the centre times the volume.
struct solid_mass {
	double volume, area;
	struct vec moment;
};

struct solid_mass shape_mass(const struct shape *sh);
// The share of face f of sh in its volume, area and moment.
struct solid_mass face_mass(const struct shape *sh, size_t f);

#endif
