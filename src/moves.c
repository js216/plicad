// SPDX-License-Identifier: GPL-3.0-only
// moves.c -- The sequence, the moves and the history of undo of view(1)
// Copyright (c) 2026 Jakob Kastelic

#include "geom.h"
#include "scene.h"
#include "util.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

struct box part_box_at(const struct part *p)
{
	return box_moved(p->box, p->mot);
}

// Adds unit u to the sequence of s unless it travels with another.
static void close_unit(struct scene *s, size_t u)
{
	if (!s->it[u].with) {
		PUSH(s->seq, s->nseq, s->capseq, u);
	}
}

// The sequence holds the motions of the items as assemble(1) puts them
// in: a unit after the items in it, and no item that travels with
// another or a picture.  The items of a unit follow its own in the
// scene, so a unit closes where an item of another unit comes.
void scene_sequence(struct scene *s)
{
	size_t *open = xmalloc((s->nit + 1) * sizeof(*open));
	size_t n = 0;

	s->nseq = 0;
	for (size_t i = 0; i < s->nit; i++) {
		const struct item *it = &s->it[i];

		while (n && open[n - 1] != it->up) {
			close_unit(s, open[--n]);
		}
		if (it->image) {
			continue;
		}
		if (it->unit) {
			open[n++] = i;
		} else if (!it->with) {
			PUSH(s->seq, s->nseq, s->capseq, i);
		}
	}
	while (n) {
		close_unit(s, open[--n]);
	}
	free(open);
	s->play = (double)s->nseq;
}

// The item of the name full, a mesh or a unit, or NONE.
static size_t item_named(const struct scene *s, const char *full)
{
	for (size_t i = 0; i < s->nit; i++) {
		if (!s->it[i].image && !strcmp(s->it[i].name, full)) {
			return i;
		}
	}
	return NONE;
}

// The motion of the path of item it from its place, done up to the
// share f: its steps undone, then those of f done again.
static struct rigid path_motion(const struct item *it, double f)
{
	struct rigid all = rig_id();
	struct rigid done = rig_id();
	double left = f * (double)it->nsteps;

	for (size_t i = 0; i < it->nsteps; i++) {
		struct pstep st = it->steps[i];
		struct line3 ax = { st.at, st.ax };
		struct rigid k = rig_id();
		double g = fmin(fmax(left - (double)i, 0), 1);

		k.t = st.d;
		all = rig_mul(st.turn ? rig_rot(ax, st.deg) : k, all);
		if (g > 0) {
			k.t = mul(st.d, g);
			done = rig_mul(st.turn ? rig_rot(ax, st.deg * g) : k,
				       done);
		}
	}
	return rig_mul(done, rig_inv(all));
}

// The motion of item i from its place by its own path in the sequence,
// or by that of the item it travels with.
static struct rigid own_motion(const struct scene *s, size_t i)
{
	const struct item *it = &s->it[i];
	size_t k = it->with ? item_named(s, it->with) : i;

	for (size_t q = 0; k != NONE && q < s->nseq; q++) {
		if (s->seq[q] == k) {
			double f = fmin(fmax(s->play - (double)q, 0), 1);

			return f < 1 ? path_motion(&s->it[k], f) : rig_id();
		}
	}
	return rig_id();
}

// The motion of item i from its place by the sequence: its own, and
// then those of its units, the innermost first.
static struct rigid seq_motion(const struct scene *s, size_t i)
{
	struct rigid m = own_motion(s, i);

	for (size_t u = s->it[i].up; u != NONE; u = s->it[u].up) {
		m = rig_mul(own_motion(s, u), m);
	}
	return m;
}

// The motion of item i where it stands: its sequence, then its drag.
static struct rigid item_motion(const struct scene *s, size_t i)
{
	struct rigid m = seq_motion(s, i);

	m.t = add(m.t, s->drag[i]);
	return m;
}

void scene_place(struct scene *s)
{
	for (size_t i = 0; i < s->np; i++) {
		struct part *p = &s->p[i];

		p->mot = p->item < s->nit ? item_motion(s, p->item) : rig_id();
	}
	// A picture moves with the mesh or the unit it lies on.
	for (size_t i = 0; i < s->np; i++) {
		size_t on = s->p[i].item < s->nit ? s->it[s->p[i].item].on :
						    NONE;

		if (on != NONE && s->it[on].unit) {
			s->p[i].mot = item_motion(s, on);
		}
		for (size_t k = 0; on != NONE && !s->it[on].unit && k < s->np;
		     k++) {
			if (s->p[k].item == on) {
				s->p[i].mot = s->p[k].mot;
				break;
			}
		}
	}
}

void scene_play(struct scene *s, double k)
{
	s->play = fmin(fmax(k, 0), (double)s->nseq);
	scene_place(s);
}

void scene_home(struct scene *s)
{
	s->play = (double)s->nseq;
	for (size_t i = 0; i < s->nit; i++) {
		s->drag[i] = v3(0, 0, 0);
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

int scene_in_unit(const char *unit, const char *name)
{
	size_t n = strlen(unit);

	return !strncmp(unit, name, n) && (name[n] == 0 || name[n] == '/');
}

size_t scene_move(struct scene *s, const char *name, struct vec d)
{
	size_t n = 0;

	for (size_t i = 0; i < s->nit; i++) {
		if (!s->it[i].image && scene_in_unit(name, s->it[i].name)) {
			s->drag[i] = add(s->drag[i], d);
			n++;
		}
	}
	for (size_t i = 0; !n && i < s->np; i++) {
		if (!strcmp(s->p[i].name, name)) {
			s->drag[s->p[i].item] = add(s->drag[s->p[i].item], d);
			n++;
		}
	}
	scene_place(s);
	return n;
}

// Whether a and b differ; neither is NaN.
static int bits_differ(double a, double b)
{
	return a < b || a > b;
}

static int moves_differ(struct vec a, struct vec b)
{
	return bits_differ(a.x, b.x) || bits_differ(a.y, b.y) ||
	       bits_differ(a.z, b.z);
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
	st.drag = xmalloc((s->nit + 1) * sizeof(*st.drag));
	st.play = s->play;
	for (size_t i = 0; i < s->np; i++) {
		st.shown[i] =
			(unsigned char)((unsigned)(s->p[i].shown != 0) |
					((unsigned)(s->p[i].see != 0) << 1U));
	}
	if (s->nit) {
		memcpy(st.drag, s->drag, s->nit * sizeof(*st.drag));
	}
	return st;
}

// Whether states a and b of s are one.
static int same(const struct scene *s, const struct state *a,
		const struct state *b)
{
	if (bits_differ(a->play, b->play) ||
	    memcmp(a->shown, b->shown, s->np) != 0) {
		return 0;
	}
	for (size_t i = 0; i < s->nit; i++) {
		if (moves_differ(a->drag[i], b->drag[i])) {
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
	s->play = st->play;
	for (size_t i = 0; i < s->np; i++) {
		unsigned b = st->shown[i];

		s->p[i].shown = (int)(b & 1U);
		s->p[i].see = (int)((b >> 1U) & 1U);
	}
	if (s->nit) {
		memcpy(s->drag, st->drag, s->nit * sizeof(*s->drag));
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
