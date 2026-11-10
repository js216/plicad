// SPDX-License-Identifier: GPL-3.0-only
// place.c -- Place folded parts into an assembly
// Copyright (c) 2026 Jakob Kastelic

// Usage: place assembly.txt > assembly.fold
//
// Each line of the assembly file places one part:
//   NAME FILE.fold [rot AXIS DEG]... [at X Y Z] [colour COLOUR]
// The rotations apply in order about the world axes through the origin,
// and then the part moves by (X, Y, Z).  The numbers are expressions as
// in smfold(1), and "include FILE" reads parameters.  "colours FILE"
// reads a colour table of NAME R G B ROUGHNESS lines, each value from 0
// to 1, and the colour of a part names a line of it.  File names are
// relative to the assembly file.  A # starts a comment.  The output holds
// every part, renamed NAME, at its place.

#include "expr.h"
#include "fold.h"
#include "geom.h"
#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { MAXWORDS = 64 };

// The colour table: name, red, green and blue from 0 to 1, roughness.
struct colour {
	char *name;
	double rgb[3], rough;
};

// The assembly being read: its path and its colour table.
struct assembly {
	struct names nm;
	const char *path;
	struct colour *c;
	size_t n, cap;
};

enum { COLOUR_WORDS = 5 }; // NAME R G B ROUGHNESS

// Reads a colour table: lines of NAME R G B ROUGHNESS, # comments.
static void colour_line(char *l, void *ctx)
{
	char *w[COLOUR_WORDS + 1];
	size_t n = split_words(l, w, COLOUR_WORDS + 1);
	struct colour c;
	struct assembly *a = ctx;

	if (!n) {
		return;
	}
	if (n != COLOUR_WORDS) {
		expr_fail(&a->nm, "expected NAME R G B ROUGHNESS", NULL);
	}
	c.name = xstrdup(w[0]);
	for (int i = 0; i < 3; i++) {
		c.rgb[i] = eval(&a->nm, w[1 + i]);
	}
	c.rough = eval(&a->nm, w[COLOUR_WORDS - 1]);
	PUSH(a->c, a->n, a->cap, c);
}

static const struct colour *find_colour(const struct assembly *a,
					const char *name)
{
	for (size_t i = 0; i < a->n; i++) {
		if (strcmp(a->c[i].name, name) == 0) {
			return &a->c[i];
		}
	}
	expr_fail(&a->nm, "no such colour", name);
}

// The words of a line.
struct words {
	char **w;
	size_t n;
};

// Reads the rotations and the move in ws from word from on; the move
// comes last.
static struct rigid placement(struct assembly *a, struct words ws, size_t from)
{
	char **w = ws.w;
	size_t n = ws.n;
	struct rigid m = rig_id();
	struct vec at = v3(0, 0, 0);

	for (size_t i = from; i < n;) {
		if (strcmp(w[i], "rot") == 0 && i + 2 < n &&
		    strlen(w[i + 1]) == 1 && strchr("xyz", w[i + 1][0])) {
			struct line3 ax = { v3(0, 0, 0),
					    axis_vec(w[i + 1][0] - 'x', 1) };

			m = rig_mul(rig_rot(ax, eval(&a->nm, w[i + 2])), m);
			i += 3;
		} else if (strcmp(w[i], "colour") == 0 && i + 1 < n) {
			i += 2;
		} else if (strcmp(w[i], "at") == 0 && i + 3 < n) {
			at = add(at, v3(eval(&a->nm, w[i + 1]),
					eval(&a->nm, w[i + 2]),
					eval(&a->nm, w[i + 3])));
			i += 4;
		} else {
			expr_fail(&a->nm, "expected rot x|y|z DEG or at X Y Z",
				  w[i]);
		}
	}
	m.t = add(m.t, at);
	return m;
}

// Gives part p the colour named after "colour" in w, if any.
static void set_colour(const struct assembly *a, struct part *p, char **w,
		       size_t n)
{
	for (size_t i = 2; i + 1 < n; i++) {
		const struct colour *c = NULL;

		if (strcmp(w[i], "colour") != 0) {
			continue;
		}
		c = find_colour(a, w[i + 1]);
		free(p->colour);
		p->colour = xstrdup(c->name);
		for (int k = 0; k < 3; k++) {
			p->rgb[k] = c->rgb[k];
		}
		p->rough = c->rough;
	}
}

static void place_line(char *l, void *ctx)
{
	struct words ws;
	char *w[MAXWORDS];
	size_t n = split_words(l, w, MAXWORDS);
	char *file = NULL;
	struct sibling sib;
	struct part *ps = NULL;
	struct rigid m;
	size_t np = 0;
	struct assembly *a = ctx;

	if (!n) {
		return;
	}
	if (n > MAXWORDS || n < 2) {
		expr_fail(&a->nm, "expected NAME FILE.fold ...", NULL);
	}
	sib.of = a->path;
	sib.name = w[1];
	file = sibling_path(sib);
	if (strcmp(w[0], "include") == 0) {
		if (n != 2) {
			expr_fail(&a->nm, "include FILE", NULL);
		}
		read_params(&a->nm, file);
		free(file);
		return;
	}
	if (strcmp(w[0], "colours") == 0) {
		if (n != 2) {
			expr_fail(&a->nm, "colours FILE", NULL);
		}
		each_line(&a->nm, file, colour_line, a);
		free(file);
		return;
	}
	ws.w = w;
	ws.n = n;
	m = placement(a, ws, 2);
	np = read_fold(file, &ps);
	if (!np) {
		expr_fail(&a->nm, "the fold file holds no parts", w[1]);
	}
	for (size_t i = 0; i < np; i++) {
		free(ps[i].name);
		ps[i].name = xstrdup(w[0]);
		ps[i].place = rig_mul(m, ps[i].place);
		set_colour(a, &ps[i], w, n);
		write_fold(&ps[i]);
	}
	free_parts(ps, np);
	free(file);
}

int main(int argc, char **argv)
{
	struct assembly a = {
		{ NULL, 0, 0, 0, NULL, NULL, 0, 0 }, NULL, NULL, 0, 0
	};

	if (argc != 2) {
		die("usage: place assembly.txt > assembly.fold");
	}
	a.path = argv[1];
	each_line(&a.nm, a.path, place_line, &a);
	for (size_t i = 0; i < a.n; i++) {
		free(a.c[i].name);
	}
	free(a.c);
	free_names(&a.nm);
	close_stdout();
	return 0;
}
