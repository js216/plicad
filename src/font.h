// SPDX-License-Identifier: GPL-3.0-only
// font.h -- The widths of the characters of osifont, for laying out text
// Copyright (c) 2026 Jakob Kastelic

#ifndef FONT_H
#define FONT_H

#include <stddef.h>

// The length in characters of the glyph that string s starts with: 3
// for a %% sign and its letter, else 1; its width at height 1 goes to
// w.  A character osifont lacks has no width.
size_t glyph(const char *s, double *w);

// The width of string s at height h, as the TX of smsheet draws it.
double text_width(const char *s, double h);

#endif
