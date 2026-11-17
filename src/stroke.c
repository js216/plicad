// SPDX-License-Identifier: GPL-3.0-only
// stroke.c -- The stroke font of the silkscreen and the schematics
// Copyright (c) 2026 Jakob Kastelic

// See stroke.h.

#include "stroke.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NARROW 0.88 // The share of the grid's width a glyph takes.

// The strokes of the characters from the space to the tilde: a string of
// x and y + 2 digits for each point of a stroke, the strokes
// blank-separated (y from -2, the descender, to 6, the capital height).
static const char *const glyph[95] = {
	"", // space
	"2824 2322", // !
	"1817 3837", // "
	"1713 3733 0646 0444", // #
	"4717061535443303 2822", // $
	"0248 0818170708 3343423233", // %
	"42161728373603122244", // &
	"2827", // '
	"38272332", // (
	"18272312", // )
	"2723 0644 0446", // *
	"2723 0545", // +
	"232211", // ,
	"0545", // -
	"2322", // .
	"0248", // /
	"030718384743321203", // 0
	"17282822 1232", // 1
	"07183847460242", // 2
	"0718384746354443321203 1535", // 3
	"32380444", // 4
	"480805354443321203", // 5
	"4738180703123243443505", // 6
	"084812", // 7
	"15060718384746351504031232434435", // 8
	"4515060718384743321203", // 9
	"2625 2322", // :
	"2625 232211", // ;
	"470543", // <
	"0646 0444", // =
	"074503", // >
	"07183847462524 2322", // ?
	"343616144447381807031242", // @
	"0206284642 0545", // A
	"02083847463505 3544433202", // B
	"4738180703123243", // C
	"02082846442202", // D
	"48080242 0535", // E
	"480802 0535", // F
	"47381807031232434525", // G
	"0208 4842 0545", // H
	"1838 2822 1232", // I
	"4843321203", // J
	"0208 4804 1542", // K
	"080242", // L
	"0208254842", // M
	"02084248", // N
	"030718384743321203", // O
	"02083847463505", // P
	"030718384743321203 2442", // Q
	"02083847463505 2542", // R
	"473818070615354443321203", // S
	"0848 2822", // T
	"080312324348", // U
	"082248", // V
	"0812253248", // W
	"0248 0842", // X
	"0825 4825 2522", // Y
	"08480242", // Z
	"38181232", // [
	"0842", // backslash
	"18383212", // ]
	"062846", // ^
	"0141", // _
	"1827", // `
	"4642 4536160503123243", // a
	"0802 0516364543321203", // b
	"4536160503123243", // c
	"4842 4536160503123243", // d
	"04444536160503123243", // e
	"38281712 0636", // f
	"4536160503123243 4641301001", // g
	"0802 0516364542", // h
	"2622 2827", // i
	"3631201001 3837", // j
	"0802 3603 1442", // k
	"18282332", // l
	"0602 05162522 25364542", // m
	"0602 0516364542", // n
	"030516364543321203", // o
	"0600 0516364543321203", // p
	"4640 4536160503123243", // q
	"0602 05163645", // r
	"45361605143443321203", // s
	"18132232 0636", // t
	"0603123243 4642", // u
	"062246", // v
	"0612253246", // w
	"0642 0246", // x
	"0622 4610", // y
	"06460242", // z
	"38272615242332", // {
	"2821", // |
	"18272635242312", // }
	"05163445", // ~
};

static const char *glyph_of(unsigned char c)
{
	return c >= 32 && c < 127 ? glyph[c - 32] : glyph['?' - 32];
}

double stroke_width(const char *s, double h)
{
	size_t n = strlen(s);

	return n ? (5 * (double)n - 1) * h / 6 : 0;
}

int stroke_descends(const char *s)
{
	for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
		for (const char *g = glyph_of(*p); *g; g++) {
			if (*g != ' ' && g[1] && g[1] != ' ' && g[1] < '2') {
				return 1;
			}
			g += *g != ' ';
		}
	}
	return 0;
}

size_t stroke_text(const char *s, double h, double **seg)
{
	double u = h / 6;
	size_t n = 0, cap = 64;

	*seg = malloc(4 * cap * sizeof **seg);
	if (!*seg) {
		fprintf(stderr, "out of memory\n");
		exit(2);
	}
	for (size_t k = 0; s[k]; k++) {
		const char *g = glyph_of((unsigned char)s[k]);
		double x0 = 5 * u * (double)k;

		// A stroke a run of points up to a blank; a segment between
		// each two of them.
		for (; *g; g++) {
			if (*g == ' ' || !g[1] || !g[2] || g[2] == ' ') {
				g += *g != ' ' && g[1];
				continue;
			}
			if (n == cap) {
				cap *= 2;
				*seg = realloc(*seg, 4 * cap * sizeof **seg);
				if (!*seg) {
					fprintf(stderr, "out of memory\n");
					exit(2);
				}
			}
			// Narrowed about the middle of the grid, x 2.
			(*seg)[4 * n] = x0 + (2 + (g[0] - '2') * NARROW) * u;
			(*seg)[4 * n + 1] = (g[1] - '2') * u;
			(*seg)[4 * n + 2] = x0 + (2 + (g[2] - '2') * NARROW) * u;
			(*seg)[4 * n + 3] = (g[3] - '2') * u;
			n++;
			g++;
		}
	}
	return n;
}
