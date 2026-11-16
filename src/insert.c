// SPDX-License-Identifier: GPL-3.0-only
// insert.c -- Self-clinching hardware of the table of the library
// Copyright (c) 2026 Jakob Kastelic

#include "insert.h"
#include "expr.h"
#include "geom.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HALF 0.5 // A normal of less x than this takes its frame from x.

// The words of a row.
enum {
	COL_NAME,
	COL_KIND,
	COL_HOLE,
	COL_SHEET,
	COL_EDGE,
	COL_THREAD,
	COL_BODY_D,
	COL_BODY_H,
	ROW_WORDS // and the words after BODY_H.
};

enum { WORDS_MAX = 16 };

// Copies the word w into the text t of room INSERT_TEXT; dies with where
// if it does not fit.
static void put_text(char *t, const char *w, const char *where)
{
	if (strlen(w) >= INSERT_TEXT) {
		die("%s: a word is too long: %s", where, w);
	}
	memcpy(t, w, strlen(w) + 1);
}

// Reads the words of row w after BODY_H, n words in all, into t:
// [MATERIAL] [flush] [blind DEPTH].
static void read_options(struct names *nm, char **w, size_t n,
			 struct insert_type *t)
{
	const char *where = expr_where(nm);

	put_text(t->material, "", where);
	for (size_t i = ROW_WORDS; i < n; i++) {
		if (strcmp(w[i], "flush") == 0) {
			t->flush = 1;
		} else if (strcmp(w[i], "blind") == 0 && i + 1 < n) {
			t->blind = eval(nm, w[++i]);
			if (!(t->blind > 0)) {
				die("%s: a blind hole has a depth above 0",
				    where);
			}
		} else if (i == ROW_WORDS) {
			put_text(t->material, w[i], where);
		} else {
			die("%s: after BODY_H: [MATERIAL] [flush] "
			    "[blind DEPTH]: %s",
			    where, w[i]);
		}
	}
}

// Reads row w of n words into t, its numbers by nm.
static void read_row(struct names *nm, char **w, size_t n,
		     struct insert_type *t)
{
	const char *where = expr_where(nm);

	if (n < ROW_WORDS) {
		die("%s: NAME KIND HOLE SHEET EDGE THREAD BODY_D BODY_H "
		    "[MATERIAL] [flush] [blind DEPTH]",
		    where);
	}
	if (strcmp(w[COL_KIND], "nut") != 0 &&
	    strcmp(w[COL_KIND], "stud") != 0 &&
	    strcmp(w[COL_KIND], "standoff") != 0) {
		die("%s: a kind is nut, stud or standoff: %s", where,
		    w[COL_KIND]);
	}
	put_text(t->name, w[COL_NAME], where);
	put_text(t->kind, w[COL_KIND], where);
	t->hole = eval(nm, w[COL_HOLE]);
	t->sheet = eval(nm, w[COL_SHEET]);
	t->edge = eval(nm, w[COL_EDGE]);
	put_text(t->thread, w[COL_THREAD], where);
	t->body_d = eval(nm, w[COL_BODY_D]);
	t->body_h = eval(nm, w[COL_BODY_H]);
	t->flush = 0;
	t->blind = 0;
	read_options(nm, w, n, t);
	if (!(t->hole > 0) || !(t->sheet > 0) || !(t->edge > 0) ||
	    !(t->body_d > 0) || !(t->body_h >= 0) ||
	    !(t->body_h > 0 || t->flush || t->blind > 0)) {
		die("%s: the sizes of a type are above 0, but the height of "
		    "a body that fills its hole",
		    where);
	}
	if (t->flush && t->blind > 0) {
		die("%s: a type is flush or blind, not both", where);
	}
}

// Finds the type of the name in the table of path into t; returns 0, or
// -1 if it has none.
static int type_in(const char *path, const char *name, struct insert_type *t)
{
	char *text = read_file(path);
	struct src_line *ls = NULL;
	size_t n = source_lines(text, &ls);
	struct names nm;
	char where[WHERE_LEN];
	int found = -1;

	memset(&nm, 0, sizeof(nm));
	nm.where = where;
	for (size_t i = 0; i < n && found < 0; i++) {
		char *w[WORDS_MAX + 1];
		size_t k = split_words(ls[i].text, w, WORDS_MAX);

		xsnprintf(where, "%s:%u", path, ls[i].no);
		if (k && strcmp(w[0], name) == 0) {
			read_row(&nm, w, k, t);
			found = 0;
		}
	}
	free_names(&nm);
	free_source_lines(ls, n);
	free(text);
	return found;
}

int insert_type(struct insert_ref r, struct insert_type *t)
{
	struct sibling sb = { r.of, "inserts" };
	char *near = sibling_path(sb);
	char *lib = lib_file("tables", "inserts");
	FILE *f = fopen(near, "r");
	int found = -1;

	if (f) {
		(void)fclose(f);
		found = type_in(near, r.name, t);
	}
	if (found < 0) {
		found = type_in(lib, r.name, t);
	}
	free(near);
	free(lib);
	return found;
}

// Adds the use of the piece of curve c from t0 to t1 to the loop under
// way of sh, back from t1 to t0 if back.
static void use(struct shape *sh, const struct curve *c, struct span t,
		int back)
{
	int fwd = 1;
	size_t e = shape_piece(sh, c, t, &fwd);

	shape_use(sh, e, fwd != back);
}

// Adds a loop of the two halves of ring c, back if back.
static void ring_loop(struct shape *sh, const struct curve *c, int back)
{
	size_t first = sh->nu;

	use(sh, c, (struct span){ 0, PI }, back);
	use(sh, c, (struct span){ PI, 2 * PI }, back);
	shape_end_loop(sh, first);
}

// The ring of radius r at the height z along the bar b, x its frame.
static struct curve bar_ring(const struct bar *b, struct vec x, double r,
			     double z)
{
	struct curve c;

	memset(&c, 0, sizeof(c));
	c.kind = CV_CIRCLE;
	c.o = add(b->o, mul(b->n, z));
	c.n = b->n;
	c.x = x;
	c.a = r;
	return c;
}

void bar_shape(struct shape *sh, struct bar b, const char *name)
{
	struct vec x = unit(
		cross(b.n, fabs(b.n.x) < HALF ? v3(1, 0, 0) : v3(0, 1, 0)));
	struct curve lo[BAR_MAX];
	struct curve hi[BAR_MAX];
	size_t ns = b.ns < BAR_MAX ? b.ns : BAR_MAX;

	shape_init(sh, name);
	for (size_t k = 0; k < ns; k++) {
		struct surf side = { SF_CYL, b.o, b.n, x, b.r[k], 0, 0 };
		struct span up = { b.z[k], b.z[k + 1] };
		struct curve seam[2];

		lo[k] = bar_ring(&b, x, b.r[k], b.z[k]);
		hi[k] = bar_ring(&b, x, b.r[k], b.z[k + 1]);
		for (int j = 0; j < 2; j++) {
			memset(&seam[j], 0, sizeof(seam[j]));
			seam[j].kind = CV_LINE;
			seam[j].o = add(b.o, mul(x, j ? -b.r[k] : b.r[k]));
			seam[j].x = b.n;
		}
		// The halves of the side, each over half a turn of u and the
		// height of the step, counterclockwise about the outward
		// normal: the lower ring forward, the upper one back.
		for (int j = 0; j < 2; j++) {
			struct span half = { j ? PI : 0, j ? 2 * PI : PI };
			size_t first = 0;

			shape_face(sh, side, 1, name);
			first = sh->nu;
			use(sh, &lo[k], half, 0);
			use(sh, &seam[!j], up, 0);
			use(sh, &hi[k], half, 1);
			use(sh, &seam[j], up, 1);
			shape_end_loop(sh, first);
		}
	}
	// The disks of the ends, and between two steps a ring that faces
	// the way the bar narrows; each uses its rings the other way from
	// the sides.
	for (size_t k = 0; k <= ns; k++) {
		struct surf disk = { SF_PLANE, b.o, b.n, x, 0, 0, 0 };

		if (k == 0) {
			disk.o = lo[0].o;
			shape_face(sh, disk, -1, name);
			ring_loop(sh, &lo[0], 1);
		} else if (k == ns) {
			disk.o = hi[k - 1].o;
			shape_face(sh, disk, 1, name);
			ring_loop(sh, &hi[k - 1], 0);
		} else {
			int down = b.r[k - 1] > b.r[k];

			disk.o = hi[k - 1].o;
			shape_face(sh, disk, down ? 1 : -1, name);
			ring_loop(sh, down ? &hi[k - 1] : &lo[k], !down);
			ring_loop(sh, down ? &lo[k] : &hi[k - 1], down);
		}
	}
}
