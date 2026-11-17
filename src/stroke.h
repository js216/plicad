// SPDX-License-Identifier: GPL-3.0-only
// stroke.h -- The stroke font of the silkscreen and the schematics
// Copyright (c) 2026 Jakob Kastelic

// The one font of strokes every program draws text in: the silkscreen
// of a board (pcbsrc), the text of a sheet (schdraw) and the font S of
// vec(5), which the viewers and the printer draw.  Each printable ASCII
// character is drawn on a grid 4 units wide, the capitals and digits 6
// high on the baseline, the lowercase letters 4 (their ascenders 6),
// the descenders 2 under the baseline, each character's strokes drawn
// at 0.88 of the grid's width about its middle; each character advances
// 5 units.  A text's size is the height of its capitals, a unit is a
// sixth of it, and the pen is STROKE_PEN of it wide, round at its ends.

#ifndef STROKE_H
#define STROKE_H

#include <stddef.h>

#define STROKE_PEN 0.15 // The width of the pen, by the capital height.

// The width of a text of capital height h: of its characters' advances,
// but the last's room after it.
double stroke_width(const char *s, double h);
// Whether a text has a character that reaches under the baseline.
int stroke_descends(const char *s);
// The strokes of a text of capital height h, from its baseline's left
// end at the origin, as segments x0 y0 x1 y1 into *seg (freed by the
// caller); returns their number.  A character out of printable ASCII is
// drawn as ?.
size_t stroke_text(const char *s, double h, double **seg);

#endif
