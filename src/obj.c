// SPDX-License-Identifier: GPL-3.0-only
// obj.c -- Read the triangles of Wavefront OBJ files
// Copyright (c) 2026 Jakob Kastelic

#include "obj.h"
#include "geom.h"
#include "util.h"
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	READ_STEP = 65536, // The least read of a file, in bytes.
	TRIS_START = 4096, // The room for triangles a part starts with.
	FAST_DIGITS = 15, // The most digits a number of the fast path has.
	TEN = 10
};

// The powers of ten that divide the numbers of the fast path, exact.
static const double pow10s[FAST_DIGITS + 1] = { 1e0,  1e1,  1e2,  1e3,
						1e4,  1e5,  1e6,  1e7,
						1e8,  1e9,  1e10, 1e11,
						1e12, 1e13, 1e14, 1e15 };

char *slurp(const char *path, size_t *n, struct fault *e)
{
	FILE *f = fopen(path, "rb");
	size_t cap = READ_STEP;
	char *b = NULL;
	int bad = 0;

	*n = 0;
	if (!f) {
		fault_at(e, "cannot open %s: %s", path, strerror(errno));
		return NULL;
	}
	b = xmalloc(cap);
	for (;;) {
		size_t want = cap - *n - 1;
		size_t k = fread(b + *n, 1, want, f);

		*n += k;
		if (k < want) {
			break;
		}
		cap *= 2;
		b = xrealloc(b, cap);
	}
	bad = ferror(f);
	if (fclose(f) != 0 || bad) {
		free(b);
		fault_at(e, "cannot read %s", path);
		return NULL;
	}
	b[*n] = 0;
	return b;
}

static void obj_part_free(struct obj_part *p)
{
	free(p->name);
	free(p->pos);
	free(p->nrm);
	memset(p, 0, sizeof(*p));
}

void obj_free(struct obj_file *f)
{
	for (size_t i = 0; i < f->n; i++) {
		obj_part_free(&f->p[i]);
	}
	free(f->p);
	memset(f, 0, sizeof(*f));
}

// The points of the file, three floats each.
struct floats {
	float *v;
	size_t n, cap;
};

static void push3(struct floats *a, const double *x)
{
	if (a->n + 3 > a->cap) {
		a->cap = a->cap ? 2 * a->cap : READ_STEP;
		a->v = xrealloc(a->v, a->cap * sizeof(float));
	}
	for (int k = 0; k < 3; k++) {
		a->v[a->n++] = (float)x[k];
	}
}

// The state of a read: the path, the line, the vertices and normals so
// far, and the fault.
struct reader {
	const char *path;
	size_t line;
	struct floats v, vn;
	struct obj_file *f;
	struct fault *e;
};

static void new_part(struct reader *r, const char *name)
{
	struct obj_part p;

	memset(&p, 0, sizeof(p));
	p.name = xstrdup(name);
	PUSH(r->f->p, r->f->n, r->f->cap, p);
}

// The decimal number at s, its end in *end: a fast path for the plain
// numbers of meshes, strtod for the rest.
static double number(char *s, char **end)
{
	char *p = s;
	double sign = 1;
	uint64_t m = 0;
	int digits = 0;
	int scale = 0;

	while (*p == ' ' || *p == '\t') {
		p++;
	}
	if (*p == '-' || *p == '+') {
		sign = *p++ == '-' ? -1 : 1;
	}
	for (; *p >= '0' && *p <= '9' && digits < FAST_DIGITS; p++, digits++) {
		m = (TEN * m) + (uint64_t)(*p - '0');
	}
	if (*p == '.') {
		for (p++; *p >= '0' && *p <= '9' && digits < FAST_DIGITS;
		     p++, digits++) {
			m = (TEN * m) + (uint64_t)(*p - '0');
			scale++;
		}
	}
	if (!digits || (*p >= '0' && *p <= '9') || *p == 'e' || *p == 'E' ||
	    *p == '.' || *p == 'x' || *p == 'X') {
		return strtod(s, end);
	}
	*end = p;
	return sign * (double)m / pow10s[scale];
}

// Reads n numbers after the keyword of the line into x.
static int numbers(struct reader *r, char **s, double *x, int n)
{
	for (int k = 0; k < n; k++) {
		char *end = NULL;

		x[k] = number(*s, &end);
		if (end == *s || !num_ok(x[k])) {
			fault_at(r->e, "%s:%zu: bad number", r->path, r->line);
			return -1;
		}
		*s = end;
	}
	return 0;
}

// The index into a list of n points of the reference at *s, or -1.
static long index_of(struct reader *r, char **s, size_t n)
{
	char *end = NULL;
	long i = strtol(*s, &end, TEN);

	if (end == *s || i == 0 || (i > 0 && (size_t)i > n) ||
	    (i < 0 && (size_t)-(i + 1) >= n)) {
		fault_at(r->e, "%s:%zu: bad index", r->path, r->line);
		return -1;
	}
	*s = end;
	return i > 0 ? i - 1 : (long)n + i;
}

// A corner of a face: its vertex and its normal, or -1 for none.
struct corner {
	long v, n;
};

// Reads the corner at *s.
static int corner(struct reader *r, char **s, struct corner *c)
{
	c->n = -1;
	c->v = index_of(r, s, r->v.n / 3);
	if (c->v < 0) {
		return -1;
	}
	if (**s != '/') {
		return 0;
	}
	(*s)++;
	while (**s && **s != '/' && **s != ' ' && **s != '\t') {
		(*s)++;
	}
	if (**s != '/') {
		return 0;
	}
	(*s)++;
	c->n = index_of(r, s, r->vn.n / 3);
	return c->n < 0 ? -1 : 0;
}

static struct vec point(const struct floats *a, long i)
{
	const float *p = &a->v[3 * (size_t)i];

	return v3((double)p[0], (double)p[1], (double)p[2]);
}

// Adds the triangle of corners c to the last part.
static void add_tri(struct reader *r, const struct corner *c)
{
	struct obj_part *p = &r->f->p[r->f->n - 1];
	struct vec q[3];
	struct vec n;

	for (int k = 0; k < 3; k++) {
		q[k] = point(&r->v, c[k].v);
	}
	n = cross(sub(q[1], q[0]), sub(q[2], q[0]));
	n = len(n) > 0 ? unit(n) : v3(0, 0, 1);
	if (p->nt == p->cap) {
		p->cap = p->cap ? 2 * p->cap : TRIS_START;
		p->pos = xrealloc(p->pos, p->cap * TRI_FLOATS * sizeof(float));
		p->nrm = xrealloc(p->nrm, p->cap * TRI_FLOATS * sizeof(float));
	}
	for (size_t k = 0; k < 3; k++) {
		struct vec m = c[k].n >= 0 ? point(&r->vn, c[k].n) : n;
		float *a = &p->pos[(TRI_FLOATS * p->nt) + (3 * k)];
		float *b = &p->nrm[(TRI_FLOATS * p->nt) + (3 * k)];

		a[0] = (float)q[k].x;
		a[1] = (float)q[k].y;
		a[2] = (float)q[k].z;
		b[0] = (float)m.x;
		b[1] = (float)m.y;
		b[2] = (float)m.z;
	}
	p->nt++;
}

static char *skip_blank(char *s)
{
	while (*s == ' ' || *s == '\t') {
		s++;
	}
	return s;
}

// Reads the face at s into triangles of the last part.
static int face(struct reader *r, char *s)
{
	struct corner c[3];
	size_t n = 0;

	if (!r->f->n) {
		new_part(r, "");
	}
	for (s = skip_blank(s); *s; s = skip_blank(s)) {
		struct corner k;

		if (corner(r, &s, &k) < 0) {
			return -1;
		}
		if (*s && *s != ' ' && *s != '\t') {
			fault_at(r->e, "%s:%zu: bad corner", r->path, r->line);
			return -1;
		}
		if (n < 3) {
			c[n] = k;
		} else {
			c[1] = c[2];
			c[2] = k;
		}
		if (++n >= 3) {
			add_tri(r, c);
		}
	}
	if (n < 3) {
		fault_at(r->e, "%s:%zu: a face of fewer than three corners",
			 r->path, r->line);
		return -1;
	}
	return 0;
}

// Sets the colour of the last part from the comment at s, if it is one.
static void colour(struct reader *r, char *s)
{
	double x[3];
	struct obj_part *p = NULL;

	s = skip_blank(s + 1);
	if (strncmp(s, "colour", strlen("colour")) != 0 || !r->f->n) {
		return;
	}
	s += strlen("colour");
	if (numbers(r, &s, x, 3) < 0) {
		return;
	}
	p = &r->f->p[r->f->n - 1];
	p->has_rgb = 1;
	for (int k = 0; k < 3; k++) {
		p->rgb[k] = (float)fmin(fmax(x[k], 0), 1);
	}
}

// Reads one line.
static int statement(struct reader *r, char *s)
{
	double x[3];
	char *w = skip_blank(s);
	size_t n = strcspn(w, " \t");
	char *rest = w + n;

	if (*w == '#') {
		colour(r, w);
		return 0;
	}
	if (n == 1 && *w == 'v') {
		if (numbers(r, &rest, x, 3) < 0) {
			return -1;
		}
		push3(&r->v, x);
	} else if (n == 2 && !strncmp(w, "vn", 2)) {
		if (numbers(r, &rest, x, 3) < 0) {
			return -1;
		}
		push3(&r->vn, x);
	} else if (n == 1 && *w == 'f') {
		return face(r, rest);
	} else if (n == 1 && (*w == 'o' || *w == 'g')) {
		char *name = skip_blank(rest);
		size_t k = strlen(name);

		while (k && (name[k - 1] == ' ' || name[k - 1] == '\t')) {
			name[--k] = 0;
		}
		if (r->f->n && !r->f->p[r->f->n - 1].nt) {
			free(r->f->p[r->f->n - 1].name);
			r->f->p[r->f->n - 1].name = xstrdup(name);
		} else {
			new_part(r, name);
		}
	}
	return 0;
}

int obj_read(const char *path, struct obj_file *f, struct fault *e)
{
	struct reader r;
	size_t size = 0;
	char *text = slurp(path, &size, e);
	char *s = text;
	char *l = NULL;
	int bad = 0;

	memset(f, 0, sizeof(*f));
	if (!text) {
		return -1;
	}
	if (strlen(text) != size) {
		free(text);
		fault_at(e, "%s: a NUL byte", path);
		return -1;
	}
	memset(&r, 0, sizeof(r));
	r.path = path;
	r.f = f;
	r.e = e;
	while (!bad && (l = next_line(&s)) != NULL) {
		r.line++;
		bad = statement(&r, l) < 0;
	}
	free(text);
	free(r.v.v);
	free(r.vn.v);
	if (bad) {
		obj_free(f);
		return -1;
	}
	return 0;
}
