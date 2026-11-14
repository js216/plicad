// SPDX-License-Identifier: GPL-3.0-only
// font.c -- The widths of the characters of osifont, for laying out text
// Copyright (c) 2026 Jakob Kastelic

#include "font.h"
#include <stddef.h>

#define CAP 1515.0 // The height of a capital, in units of the font.
#define SIGN_W 1100.0 // The width of a %% sign, in units of the font,
#define CSINK_W 1800.0 // and of the countersink sign.

enum { FIRST_CHAR = 32 }; // The first character with a width.

// The width of each character of osifont from the space on, in units
// of the font.
static const short widths[] = {
	890,  450,  801,  1256, 954,  1564, 1258, 417,	522,  522,  950,  952,
	477,  952,  412,  1006, 958,  700,  955,  984,	1051, 1018, 1013, 979,
	1056, 1030, 455,  522,	952,  950,  952,  951,	1660, 1258, 1192, 1004,
	1205, 1003, 1003, 1187, 1210, 452,  847,  1155, 1007, 1364, 1202, 1203,
	1152, 1222, 1155, 1105, 1101, 1206, 1259, 1559, 1262, 1258, 1106, 544,
	946,  544,  798,  952,	646,  986,  1030, 829,	1031, 1015, 899,  1029,
	1031, 447,  446,  1007, 548,  1315, 1011, 1009, 1039, 1039, 834,  954,
	711,  1012, 954,  1253, 951,  951,  950,  644,	452,  644,  954
};

size_t glyph(const char *s, double *w)
{
	if (s[0] == '%' && s[1] == '%' && s[2]) {
		*w = (s[2] == 'v' ? CSINK_W : SIGN_W) / CAP;
		return 3;
	}
	*w = *s >= FIRST_CHAR && *s <= '~' ? widths[*s - FIRST_CHAR] / CAP : 0;
	return 1;
}

double text_width(const char *s, double h)
{
	double n = 0;

	while (*s) {
		double w = 0;

		s += glyph(s, &w);
		n += w;
	}
	return n * h;
}
