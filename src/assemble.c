// SPDX-License-Identifier: GPL-3.0-only
// assemble.c -- Replay the assembly sequence and check each motion
// Copyright (c) 2026 Jakob Kastelic

// Usage: assemble [-v] assembly.txt
//
// Reads an assembly file (asm.h) and puts its parts in, in the order of
// its lines, each along its insertion path from its start to its place,
// with the parts that travel with it: a subassembly goes together first,
// by its own lines, and then comes in as one unit along the path of its
// line.  Each motion is tried in samples, against the parts in before it
// in its unit: no sample moves a part more than the step of its path, or
// turns it more than its angle, and none moves any point of a part
// farther than half the thinnest wall of the parts tried, so a part
// cannot pass through another between two samples.  A part with no path
// is tried at its place.  Touching is no clash: the test is that of
// interfere(1), clash.h.
//
// A clash is said on stderr, once for each pair and motion:
//   assembly.txt:N: insertion path of NAME, step K at S mm: PART clashes
//   with OTHER
// (or at S deg for a turn, at its place for a part with no path), and
// with a clearance in the file, a pair that passes nearer than it on the
// way, short of the place, where parts may touch:
//   ...: PART passes D mm from OTHER, under the clearance C
// Exit status 1 for a clash, 0 else.  A path whose file gives no step
// is refused.  With -v, a line on stdout for each motion: path NAME,
// its steps and samples.

#include "asm.h"
#include "clash.h"
#include "fold.h"
#include "geom.h"
#include "mesh.h"
#include "shape.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TOO_NEAR 1e-9 // Gaps this near the clearance are under it.
#define THIN_MIN 1e-3 // The thinnest wall that sets the samples, in mm.
#define HALF_TURN 180.0 // In degrees.

enum { MAX_POINTS = 1 }; // A clash needs one sample.

// A part of the assembly with its exact solid at its place.
struct body {
	struct shape sh;
	struct clash_body cb;
	int in; // Whether it stands in its place now.
};

// The work: the assembly, the bodies of its parts, and what is said.
struct work {
	struct assembly a;
	struct body *b;
	int verbose;
	int clashes;
};

// A motion of parts: the line of its path, the parts that move, and the
// unit whose parts that stand in it tries them against.
struct motion {
	const struct asm_line *l;
	size_t *moving;
	size_t nmoving;
	size_t unit;
};

// A list of indices under way.
struct index_list {
	size_t *v;
	size_t n, cap;
};

static void add_index(struct index_list *l, size_t k)
{
	PUSH(l->v, l->n, l->cap, k);
}

// A place along a path: its step, and the share of it done.
struct at {
	size_t step;
	double share;
};

// A clash of parts: the place along the path, the distance or angle
// there, and the moving part and the one in place it clashes with.
struct hit {
	struct at at;
	double s;
	size_t part, other;
};

// Whether part k lies in the unit of line u, NULL for the whole
// assembly.
static int in_unit(const struct asm_line *u, size_t k)
{
	return !u || (k >= u->first && k < u->first + u->n);
}

// The motion in the world that takes the parts of line l from their
// places to where the place at along its path has them, frame the
// place of the unit of l in the world.  The path brings them from the
// start, its steps undone, to their places.
static struct rigid pose(const struct asm_line *l, struct rigid frame,
			 struct at at)
{
	size_t j = at.step;
	double f = at.share;
	struct rigid all = rig_id();
	struct rigid done = rig_id();

	for (size_t i = 0; i < l->path.n; i++) {
		struct rigid k = asm_step_rig(l, &l->path.s[i]);

		all = rig_mul(k, all);
		if (i < j) {
			done = rig_mul(k, done);
		}
	}
	if (j < l->path.n) {
		struct asm_step s = l->path.s[j];

		s.deg *= f;
		s.d = mul(s.d, f);
		done = rig_mul(asm_step_rig(l, &s), done);
	}
	return rig_mul(rig_mul(frame, rig_mul(done, rig_inv(all))),
		       rig_inv(frame));
}

// The thinnest wall of the parts of motion mv and the parts in.
static double thinnest(const struct work *w, const struct motion *mv)
{
	double t = HUGE_VAL;

	for (size_t k = 0; k < w->a.nout; k++) {
		if (w->b[k].in) {
			t = fmin(t, w->b[k].cb.thin);
		}
	}
	for (size_t i = 0; i < mv->nmoving; i++) {
		t = fmin(t, w->b[mv->moving[i]].cb.thin);
	}
	return fmax(t, THIN_MIN);
}

// The samples of step s of the path of l: so many that none moves the
// parts, of the box b, more than the step, turns them more than its
// angle, or moves a point more than half of thin.
static size_t samples(const struct asm_line *l, const struct asm_step *s,
		      struct box b, double thin)
{
	double n = 1;

	if (s->turn) {
		struct vec at = s->about ? s->at : l->m.t;
		double r = 0;

		for (unsigned k = 0; k < BOX_CORNERS; k++) {
			r = fmax(r, len(sub(box_corner(b, k), at)));
		}
		n = fmax(fabs(s->deg) / l->step[1],
			 fabs(s->deg) * PI / HALF_TURN * r / (thin / 2));
	} else {
		n = len(s->d) / fmin(l->step[0], thin / 2);
	}
	n = ceil(n);
	return n < 1 ? 1 : (size_t)n;
}

// Says the clash h of motion mv.
static void say_clash(struct work *w, const struct motion *mv, struct hit h)
{
	const struct asm_line *l = mv->l;
	const char *pa = w->a.out[h.part].name;
	const char *po = w->a.out[h.other].name;
	size_t j = h.at.step;
	double s = h.s;

	if (!l->has_path) {
		warn("%s: insertion of %s, at its place: %s clashes with %s",
		     l->where, l->full, pa, po);
	} else {
		warn("%s: insertion path of %s, step %zu at %.3g %s: %s clashes "
		     "with %s",
		     l->where, l->full, j + 1, s,
		     l->path.s[j].turn ? "deg" : "mm", pa, po);
	}
	w->clashes++;
}

// Tries motion mv at the place at along its path: says each new clash
// of a pair, and each gap under the clearance; marks the pairs said in
// said.
static void try_at(struct work *w, const struct motion *mv, struct at at,
		   unsigned char *said)
{
	const struct asm_line *l = mv->l;
	const struct asm_line *u = mv->unit == NONE ? NULL : &w->a.ln[mv->unit];
	size_t j = at.step;
	double f = at.share;
	struct rigid frame = mv->unit == NONE ? rig_id() :
						w->a.ln[mv->unit].world;
	struct rigid d = l->has_path ? pose(l, frame, at) : rig_id();
	size_t n = w->a.nout;
	double s = 0;

	if (j < l->path.n) {
		const struct asm_step *st = &l->path.s[j];

		s = f * (st->turn ? fabs(st->deg) : len(st->d));
	}
	for (size_t i = 0; i < mv->nmoving; i++) {
		size_t a = mv->moving[i];
		struct clash_pose x = { &w->b[a].cb, d };

		for (size_t o = 0; o < n; o++) {
			struct clash_pose y = { &w->b[o].cb, rig_id() };

			if (!w->b[o].in || !in_unit(u, o) ||
			    said[(i * n) + o]) {
				continue;
			}
			if (clash_count(x, y, MAX_POINTS)) {
				struct hit h = { at, s, a, o };

				say_clash(w, mv, h);
				said[(i * n) + o] = 1;
			} else if (l->clearance > 0 && j < l->path.n) {
				double g = clash_gap(x, y, l->clearance);

				if (g < l->clearance - TOO_NEAR) {
					warn("%s: %s passes %.3g mm from %s, "
					     "under the clearance %g",
					     l->where, w->a.out[a].name, g,
					     w->a.out[o].name, l->clearance);
					said[(i * n) + o] = 1;
				}
			}
		}
	}
}

// The box of the parts of motion mv at their places.
static struct box moving_box(const struct work *w, const struct motion *mv)
{
	struct box b = box_empty();

	for (size_t i = 0; i < mv->nmoving; i++) {
		struct box x = w->b[mv->moving[i]].cb.box;

		box_add(&b, x.lo);
		box_add(&b, x.hi);
	}
	return b;
}

// Runs motion mv: each sample of each step of its path, then the place.
static void run_motion(struct work *w, struct motion *mv)
{
	const struct asm_line *l = mv->l;
	unsigned char *said = xcalloc((mv->nmoving * w->a.nout) + 1, 1);
	double thin = thinnest(w, mv);
	struct box box = moving_box(w, mv);
	size_t total = 0;

	if (l->has_path && !l->has_step) {
		die("%s: a path needs a step DIST DEG in its file or its line",
		    l->where);
	}
	for (size_t j = 0; l->has_path && j < l->path.n; j++) {
		size_t ns = samples(l, &l->path.s[j], box, thin);

		for (size_t k = 0; k < ns; k++) {
			struct at at = { j, (double)k / (double)ns };

			try_at(w, mv, at, said);
		}
		total += ns;
	}
	{
		struct at end = { l->path.n, 1 };

		try_at(w, mv, end, said);
	}
	if (w->verbose) {
		printf("path %s steps %zu samples %zu\n", l->full, l->path.n,
		       total + 1);
	}
	for (size_t i = 0; i < mv->nmoving; i++) {
		w->b[mv->moving[i]].in = 1;
	}
	free(said);
}

// Adds the parts of line i, and of the lines that travel with it or
// with those, to motion mv.
static void gather(const struct work *w, size_t i, struct motion *mv)
{
	struct index_list todo = { NULL, 0, 0 };
	struct index_list moving = { NULL, 0, 0 };

	add_index(&todo, i);
	for (size_t q = 0; q < todo.n; q++) {
		const struct asm_line *l = &w->a.ln[todo.v[q]];

		for (size_t k = l->first; k < l->first + l->n; k++) {
			add_index(&moving, k);
		}
		for (size_t j = todo.v[q] + 1; j < w->a.nln; j++) {
			const struct asm_line *x = &w->a.ln[j];

			if (x->unit == l->unit && x->with &&
			    x->kind != ASM_END && !strcmp(x->with, l->name)) {
				add_index(&todo, j);
			}
		}
	}
	free(todo.v);
	mv->moving = moving.v;
	mv->nmoving = moving.n;
}

// Runs the motion of line i, a part line or the end of a unit.
static void line_motion(struct work *w, size_t i)
{
	const struct asm_line *l = &w->a.ln[i];
	struct motion mv;

	memset(&mv, 0, sizeof(mv));
	if (l->kind == ASM_END) {
		size_t u = i - 1;

		while (!(w->a.ln[u].kind == ASM_UNIT &&
			 w->a.ln[u].depth == l->depth)) {
			u--;
		}
		i = u;
		l = &w->a.ln[u];
		// The unit stood in its own frame as it went together; it
		// comes in now, against the parts of its own unit.
		for (size_t k = l->first; k < l->first + l->n; k++) {
			w->b[k].in = 0;
		}
	}
	if (l->with) {
		return;
	}
	mv.l = l;
	mv.unit = l->unit;
	gather(w, i, &mv);
	run_motion(w, &mv);
	free(mv.moving);
}

int main(int argc, char **argv)
{
	struct work w;
	int a = 1;

	memset(&w, 0, sizeof(w));
	if (argc == 3 && !strcmp(argv[1], "-v")) {
		w.verbose = 1;
		a = 2;
	} else if (argc != 2) {
		die("usage: assemble [-v] assembly.txt");
	}
	{
		struct asm_file f = { argv[a], NULL };

		asm_read(&w.a, f);
	}
	w.b = xcalloc(w.a.nout + 1, sizeof(*w.b));
	for (size_t k = 0; k < w.a.nout; k++) {
		const struct part *p = &w.a.out[k];

		part_shape(p, &w.b[k].sh);
		clash_init(&w.b[k].cb, &w.b[k].sh, p->solid ? 0 : p->t);
	}
	for (size_t i = 0; i < w.a.nln; i++) {
		if (w.a.ln[i].kind == ASM_PART || w.a.ln[i].kind == ASM_END) {
			line_motion(&w, i);
		}
	}
	for (size_t k = 0; k < w.a.nout; k++) {
		clash_free(&w.b[k].cb);
		shape_free(&w.b[k].sh);
	}
	free(w.b);
	asm_free(&w.a);
	close_stdout();
	return w.clashes ? 1 : 0;
}
