// SPDX-License-Identifier: GPL-3.0-only
// util.c -- Errors, memory, arrays, words and whole-file reading
// Copyright (c) 2026 Jakob Kastelic

#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void close_stdout(void)
{
	if (fflush(stdout) != 0 || ferror(stdout)) {
		die("cannot write the output");
	}
}

void *xmalloc(size_t n)
{
	void *p = malloc(n ? n : 1);

	if (!p) {
		die("out of memory");
	}
	return p;
}

void *xcalloc(size_t n, size_t size)
{
	void *p = calloc(n ? n : 1, size ? size : 1);

	if (!p) {
		die("out of memory");
	}
	return p;
}

void *xrealloc(void *p, size_t n)
{
	void *q = realloc(p, n ? n : 1);

	if (!q) {
		die("out of memory");
	}
	return q;
}

char *xstrdup(const char *s)
{
	size_t n = strlen(s) + 1;
	char *p = xmalloc(n);

	memcpy(p, s, n);
	return p;
}

enum { READ_CHUNK = 65536 }; // The first buffer of read_file.

char *read_file(const char *path)
{
	int is_stdin = strcmp(path, "-") == 0;
	FILE *f = is_stdin ? stdin : fopen(path, "rb");
	size_t cap = READ_CHUNK;
	size_t n = 0;
	char *b = NULL;
	int bad = 0;

	if (!f) {
		die("cannot open %s", path);
	}
	b = xmalloc(cap);
	for (;;) {
		size_t want = 0;
		size_t k = 0;

		if (cap - n < 2) {
			cap *= 2;
			b = xrealloc(b, cap);
		}
		want = cap - n - 1;
		k = fread(b + n, 1, want, f);
		n += k;
		if (k < want) {
			break;
		}
	}
	bad = ferror(f);
	if (!is_stdin && fclose(f) != 0) {
		bad = 1;
	}
	if (bad) {
		free(b);
		die("cannot read %s", path);
	}
	b[n] = 0;
	return b;
}

char *next_line(char **s)
{
	char *l = *s;
	char *e = NULL;
	size_t n = 0;

	if (!l || !*l) {
		return NULL;
	}
	e = strchr(l, '\n');
	if (e) {
		*e = 0;
		*s = e + 1;
	} else {
		*s = l + strlen(l);
	}
	n = strlen(l);
	if (n && l[n - 1] == '\r') {
		l[n - 1] = 0;
	}
	return l;
}

static int blank(char c)
{
	return c == ' ' || c == '\t';
}

size_t split_words(char *s, char **words, size_t max)
{
	size_t n = 0;

	for (;;) {
		int depth = 0;

		while (blank(*s)) {
			s++;
		}
		if (!*s) {
			return n;
		}
		if (n == max) {
			return max + 1;
		}
		words[n++] = s;
		while (*s && (depth || !blank(*s))) {
			depth += *s == '(';
			depth -= *s == ')';
			s++;
		}
		if (*s) {
			*s++ = 0;
		}
	}
}

char *base_name(const char *path)
{
	const char *b = strrchr(path, '/');
	char *s = xstrdup(b ? b + 1 : path);
	char *dot = strchr(s, '.');

	if (dot) {
		*dot = 0;
	}
	return s;
}

char *sibling_path(struct sibling s)
{
	const char *path = s.of;
	const char *name = s.name;
	const char *slash = strrchr(path, '/');
	size_t dir = slash ? (size_t)(slash - path) + 1 : 0;
	size_t n = strlen(name);
	char *out = NULL;

	if (name[0] == '/' || !dir) {
		return xstrdup(name);
	}
	out = xmalloc(dir + n + 1);
	for (size_t i = 0; i < dir; i++) {
		out[i] = path[i];
	}
	for (size_t i = 0; i <= n; i++) {
		out[dir + i] = name[i];
	}
	return out;
}

int num_ok(double v)
{
	return isfinite(v) && fabs(v) <= NUM_MAX;
}

void sort_doubles(double *v, size_t n)
{
	size_t gap = 1;

	while (gap < n / 3) {
		gap = (3 * gap) + 1;
	}
	for (; gap > 0; gap /= 3) {
		for (size_t i = gap; i < n; i++) {
			double x = v[i];
			size_t j = i;

			for (; j >= gap && v[j - gap] > x; j -= gap) {
				v[j] = v[j - gap];
			}
			v[j] = x;
		}
	}
}
