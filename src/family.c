// SPDX-License-Identifier: GPL-3.0-only
// family.c -- Write the part source of a member of a family of the library
// Copyright (c) 2026 Jakob Kastelic

// Usage: family screws SIZE STYLE LENGTH
//        family packages NAME
//        family inserts NAME
//
// Writes on stdout the source (part(5)) of a plain exact solid for a row
// of a table of the library (plicad-lib(7)), the solid named body:
//   screws    a head and a shank, the shank LENGTH long down from z = 0;
//             a flat head is a cone of its angle down from z = 0, its
//             top flush there, and the others a cylinder up from z = 0;
//   packages  a box of L by W, H high, from z = 0 up, centered on the
//             z axis, its length along x;
//   inserts   the body of the hardware, a cylinder from z = 0 up.
// The sizes are those of the table: a file of its name in the current
// directory, or else that of the library.

#include "expr.h"
#include "geom.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { WORDS_MAX = 16, SCREW_WORDS = 6, PACKAGE_WORDS = 4, INSERT_WORDS = 8 };

enum { SCREW_ARGS = 5 }; // family screws SIZE STYLE LENGTH

// The words of a row of the tables screws, packages and inserts.
enum { SCREW_HEAD_D = 2, SCREW_HEAD_H, SCREW_THREAD };
enum { PACKAGE_L = 1, PACKAGE_W, PACKAGE_H };
enum { INSERT_BODY_D = 6, INSERT_BODY_H };

#define HALF_TURN 180.0 // In degrees.

// A row of a table: its words, their count, and the lines and text of
// the table that hold them.
struct row {
	char *w[WORDS_MAX + 1];
	size_t n;
	char *text;
	struct src_line *l;
	size_t nl;
};

// The words a row begins with.
struct key {
	char **w;
	size_t n;
};

// Reads into r the row of the table at path that begins with the words
// of k; returns 0, or -1 if it has none.
static int row_in(const char *path, struct key k, struct row *r)
{
	r->text = read_file(path);
	r->nl = source_lines(r->text, &r->l);
	for (size_t i = 0; i < r->nl; i++) {
		size_t n = split_words(r->l[i].text, r->w, WORDS_MAX);
		size_t m = 0;

		while (m < k.n && m < n && strcmp(r->w[m], k.w[m]) == 0) {
			m++;
		}
		if (m == k.n && n <= WORDS_MAX) {
			r->n = n;
			return 0;
		}
	}
	free_source_lines(r->l, r->nl);
	free(r->text);
	r->l = NULL;
	r->text = NULL;
	r->nl = 0;
	return -1;
}

// Reads into r the row of the table that begins with the words of k:
// of the table of that name in the directory where family runs, which
// adds its rows to those of the library, or else of the library; dies
// if neither has it.
static void find_row(const char *table, struct key k, struct row *r)
{
	struct sibling sb = { "./", table };
	char *near = sibling_path(sb);
	char *lib = lib_file("tables", table);
	FILE *f = fopen(near, "r");
	int found = -1;

	if (f) {
		(void)fclose(f);
		found = row_in(near, k, r);
	}
	if (found < 0) {
		found = row_in(lib, k, r);
	}
	if (found < 0) {
		die("%s: no row %s", lib, k.w[0]);
	}
	free(near);
	free(lib);
}

// The value of the number w of a table, in mm.
static double num(struct names *nm, const char *w)
{
	return eval(nm, w);
}

// A screw of row r, length long.
static void screw(struct names *nm, const struct row *r, double length)
{
	char *const *w = r->w;
	double hd = num(nm, w[SCREW_HEAD_D]);
	double hh = num(nm, w[SCREW_HEAD_H]);
	double td = num(nm, w[SCREW_THREAD]);

	if (r->n > SCREW_WORDS) {
		double draft = num(nm, w[SCREW_WORDS]) / 2;

		printf("circle head %.9g 0 0 along z\n", hd);
		printf("extrude h from 0 to %.9g head draft %.9g\n",
		       -fmin(hh, (hd - td) / 2 / tan(draft * PI / HALF_TURN)),
		       draft);
	} else {
		printf("circle head %.9g 0 0 along z\n", hd);
		printf("extrude h from 0 to %.9g head\n", hh);
	}
	printf("circle shank %.9g 0 0 along z\n", td);
	printf("extrude s from %.9g to 0 shank\n", -length);
	printf("union body h s\n");
}

static void package(struct names *nm, char *const *w)
{
	double l = num(nm, w[PACKAGE_L]) / 2;
	double b = num(nm, w[PACKAGE_W]) / 2;

	printf("profile p along z %.9g %.9g 0 %.9g %.9g 0 %.9g %.9g 0 "
	       "%.9g %.9g 0\n",
	       -l, -b, l, -b, l, b, -l, b);
	printf("extrude body from 0 to %.9g p\n", num(nm, w[PACKAGE_H]));
}

// The body over the sheet of an insert of row w; one that sits flush
// with no body over the sheet has none to give.
static void insert(struct names *nm, char *const *w)
{
	double h = num(nm, w[INSERT_BODY_H]);

	if (!(h > 0)) {
		die("family: insert %s has no body over the sheet", w[0]);
	}
	printf("circle c %.9g 0 0 along z\n", num(nm, w[INSERT_BODY_D]));
	printf("extrude body from 0 to %.9g c\n", h);
}

int main(int argc, char **argv)
{
	struct row r;
	struct names nm;
	int screws = argc == SCREW_ARGS && !strcmp(argv[1], "screws");
	struct key k = { argv + 2, screws ? 2 : 1 };

	memset(&nm, 0, sizeof(nm));
	memset(&r, 0, sizeof(r));
	nm.where = "family";
	if (!screws && (argc != 3 || (strcmp(argv[1], "packages") != 0 &&
				      strcmp(argv[1], "inserts") != 0))) {
		die("usage: family screws SIZE STYLE LENGTH\n"
		    "       family packages NAME\n"
		    "       family inserts NAME");
	}
	find_row(argv[1], k, &r);
	printf("# %s", argv[1]);
	for (int i = 2; i < argc; i++) {
		printf(" %s", argv[i]);
	}
	printf(", from the table %s\n", argv[1]);
	if (screws && r.n >= SCREW_WORDS) {
		double length = eval(&nm, argv[4]);

		if (!(length > 0)) {
			die("family: a length is above 0");
		}
		screw(&nm, &r, length);
	} else if (argv[1][0] == 'p' && r.n >= PACKAGE_WORDS) {
		package(&nm, r.w);
	} else if (argv[1][0] == 'i' && r.n >= INSERT_WORDS) {
		insert(&nm, r.w);
	} else {
		die("family: the row of %s is short", argv[2]);
	}
	free_source_lines(r.l, r.nl);
	free(r.text);
	free_names(&nm);
	close_stdout();
	return 0;
}
