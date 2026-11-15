// SPDX-License-Identifier: GPL-3.0-only
// explode.c -- The explode, the drags and the history of undo of view(1)
// Copyright (c) 2026 Jakob Kastelic

#include "geom.h"
#include "scene.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CENTRE_EPS 1e-9 // Nearer than this to the centre, in mm, is at it.

static struct vec box_mid(struct box b)
{
	return mul(add(b.lo, b.hi), 1.0 / 2);
}

struct box part_box_at(const struct part *p)
{
	struct box b = { add(p->box.lo, p->off), add(p->box.hi, p->off) };

	return b;
}

// The centre of the box of all parts at home.
static struct vec centre(const struct scene *s)
{
	struct box b = box_empty();

	for (size_t i = 0; i < s->np; i++) {
		if (s->p[i].nt) {
			box_add(&b, s->p[i].box.lo);
			box_add(&b, s->p[i].box.hi);
		}
	}
	return b.lo.x <= b.hi.x ? box_mid(b) : v3(0, 0, 0);
}

// The unit line from c out through point q, or 0 0 0.
static struct vec out_of(struct vec c, struct vec q)
{
	struct vec d = sub(q, c);

	return len(d) > CENTRE_EPS ? unit(d) : v3(0, 0, 0);
}

struct vec scene_line_out(const struct scene *s, size_t i)
{
	struct box b = box_empty();

	for (size_t k = 0; k < s->np; k++) {
		if (s->p[k].item == i && s->p[k].nt) {
			box_add(&b, s->p[k].box.lo);
			box_add(&b, s->p[k].box.hi);
		}
	}
	if (b.lo.x > b.hi.x) {
		return v3(0, 0, 0);
	}
	return out_of(centre(s), box_mid(b));
}

void scene_place(struct scene *s)
{
	struct vec c = centre(s);
	struct vec *line = xmalloc((s->nit + 1) * sizeof(*line));

	for (size_t i = 0; i < s->nit; i++) {
		line[i] = scene_line_out(s, i);
	}
	for (size_t i = 0; i < s->np; i++) {
		struct part *p = &s->p[i];
		struct vec d = sub(box_mid(p->box), c);

		p->off = mul(d, s->f);
		if (p->item < s->nit) {
			p->off = add(p->off,
				     mul(line[p->item], s->drag[p->item]));
		}
	}
	free(line);
}

void scene_explode(struct scene *s, double f)
{
	s->f = fmax(f, 0);
	scene_place(s);
}

void scene_home(struct scene *s)
{
	s->f = 0;
	for (size_t i = 0; i < s->nit; i++) {
		s->drag[i] = 0;
	}
	scene_place(s);
}

void scene_reset(struct scene *s)
{
	for (size_t i = 0; i < s->np; i++) {
		s->p[i].shown = 1;
		s->p[i].see = 0;
	}
	scene_home(s);
}

void scene_drag(struct scene *s, size_t i, double d)
{
	if (i < s->nit) {
		s->drag[i] = fmax(d, 0);
	}
	scene_place(s);
}

// Whether a and b differ; neither is NaN.
static int bits_differ(double a, double b)
{
	return a < b || a > b;
}

static void free_state(struct state *st)
{
	free(st->shown);
	free(st->drag);
}

// The state now of s.
static struct state state_now(const struct scene *s)
{
	struct state st;

	st.shown = xmalloc(s->np + 1);
	st.drag = xmalloc((s->nit + 1) * sizeof(double));
	st.f = s->f;
	for (size_t i = 0; i < s->np; i++) {
		st.shown[i] =
			(unsigned char)((unsigned)(s->p[i].shown != 0) |
					((unsigned)(s->p[i].see != 0) << 1U));
	}
	if (s->nit) {
		memcpy(st.drag, s->drag, s->nit * sizeof(double));
	}
	return st;
}

// Whether states a and b of s are one.
static int same(const struct scene *s, const struct state *a,
		const struct state *b)
{
	if (bits_differ(a->f, b->f) || memcmp(a->shown, b->shown, s->np) != 0) {
		return 0;
	}
	for (size_t i = 0; i < s->nit; i++) {
		if (bits_differ(a->drag[i], b->drag[i])) {
			return 0;
		}
	}
	return 1;
}

void scene_commit(struct scene *s)
{
	struct state st = state_now(s);

	if (s->nhist && same(s, &st, &s->hist[s->at])) {
		free_state(&st);
		return;
	}
	while (s->nhist > s->at + 1) {
		free_state(&s->hist[--s->nhist]);
	}
	PUSH(s->hist, s->nhist, s->caphist, st);
	s->at = s->nhist - 1;
}

void scene_history_reset(struct scene *s)
{
	for (size_t i = 0; i < s->nhist; i++) {
		free_state(&s->hist[i]);
	}
	s->nhist = 0;
	s->at = 0;
	scene_commit(s);
}

// Sets s to state k of its history.
static void restore(struct scene *s, size_t k)
{
	const struct state *st = &s->hist[k];

	s->at = k;
	s->f = st->f;
	for (size_t i = 0; i < s->np; i++) {
		unsigned b = st->shown[i];

		s->p[i].shown = (int)(b & 1U);
		s->p[i].see = (int)((b >> 1U) & 1U);
	}
	if (s->nit) {
		memcpy(s->drag, st->drag, s->nit * sizeof(double));
	}
	scene_place(s);
}

int scene_undo(struct scene *s)
{
	if (!s->nhist || s->at == 0) {
		return -1;
	}
	restore(s, s->at - 1);
	return 0;
}

int scene_redo(struct scene *s)
{
	if (s->at + 1 >= s->nhist) {
		return -1;
	}
	restore(s, s->at + 1);
	return 0;
}
