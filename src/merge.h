// SPDX-License-Identifier: GPL-3.0-only
// merge.h -- Join the faces of a solid on one surface, and their edges
// Copyright (c) 2026 Jakob Kastelic

#ifndef MERGE_H
#define MERGE_H

#include "shape.h"

// Joins the faces of sh that share an edge on one surface into one face,
// which keeps the name of the first, and then the edges on one curve
// that meet at a vertex of no other edge.
void shape_merge(struct shape *sh);

#endif
