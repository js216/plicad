// SPDX-License-Identifier: GPL-3.0-only
// util.c -- Errors, memory, arrays, words and whole-file reading
// Copyright (c) 2026 Jakob Kastelic

#include "util.h"
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

enum { DECIMAL_BASE = 10 }; // The base of decimal numbers.

enum { PATH_LEN = 4096 }; // Room for the path of the program.

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

enum {
	READ_CHUNK = 65536, // The first buffer of read_file,
	LINE_FIRST = 128 // and of get_line.
};

// Reads f from where it is to its end into a new string, a NUL after;
// ferror(f) tells whether a read failed.
static char *read_rest(FILE *f)
{
	size_t cap = READ_CHUNK;
	size_t n = 0;
	char *b = xmalloc(cap);

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
	b[n] = 0;
	return b;
}

char *read_file(const char *path)
{
	int is_stdin = strcmp(path, "-") == 0;
	FILE *f = is_stdin ? stdin : fopen(path, "rb");
	char *b = NULL;
	int bad = 0;

	if (!f) {
		die("cannot open %s", path);
	}
	b = read_rest(f);
	bad = ferror(f);
	if (!is_stdin && fclose(f) != 0) {
		bad = 1;
	}
	if (bad) {
		free(b);
		die("cannot read %s", path);
	}
	return b;
}

long get_line(char **line, size_t *cap, FILE *in)
{
	size_t n = 0;
	int c = 0;

	while ((c = getc(in)) != EOF) {
		if (n + 2 > *cap) {
			*cap = *cap ? 2 * *cap : LINE_FIRST;
			*line = xrealloc(*line, *cap);
		}
		(*line)[n++] = (char)c;
		if (c == '\n') {
			break;
		}
	}
	if (!n) {
		return -1;
	}
	(*line)[n] = 0;
	return (long)n;
}

char *token(char **s, const char *delim)
{
	char *t = *s + strspn(*s, delim);
	char *end = t + strcspn(t, delim);

	if (!*t) {
		*s = t;
		return NULL;
	}
	*s = *end ? end + 1 : end;
	*end = 0;
	return t;
}

FILE *text_stream(const char *text, size_t len)
{
	FILE *f = tmpfile();

	if (f && (fwrite(text, 1, len, f) != len || fflush(f) != 0 ||
		  fseek(f, 0, SEEK_SET) != 0)) {
		(void)fclose(f);
		return NULL;
	}
	return f;
}

int stream_rewind(FILE *f)
{
	return !ferror(f) && fflush(f) == 0 && fseek(f, 0, SEEK_SET) == 0;
}

char *stream_text(FILE *f)
{
	char *t = NULL;
	int bad = !stream_rewind(f);

	if (!bad) {
		t = read_rest(f);
		bad = ferror(f);
	}
	if (fclose(f) != 0 || bad) {
		free(t);
		return NULL;
	}
	return t;
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

char *split_first(char **s)
{
	char *p = *s;
	char *word = NULL;
	int depth = 0;

	while (blank(*p)) {
		p++;
	}
	if (!*p) {
		*s = p;
		return NULL;
	}
	word = p;
	while (*p && (depth || !blank(*p))) {
		depth += *p == '(';
		depth -= *p == ')';
		p++;
	}
	if (*p) {
		*p++ = 0;
	}
	*s = p;
	return word;
}

size_t split_words(char *s, char **words, size_t max)
{
	size_t n = 0;
	char *w = NULL;

	while ((w = split_first(&s)) != NULL) {
		if (n == max) {
			return max + 1;
		}
		words[n++] = w;
	}
	return n;
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

char *lib_file(const char *dir, const char *name)
{
	char exe[PATH_LEN];
	ssize_t len = readlink("/proc/self/exe", exe, sizeof exe - 1);
	size_t n = 0;
	char *out = NULL;
	char *slash = NULL;

	if (len < 0 || (size_t)len == sizeof exe - 1) {
		die("cannot find the program, to find %s in the library", name);
	}
	exe[len] = '\0';
	// The library is lib beside the directory of the program.
	for (int up = 0; up < 2; up++) {
		slash = strrchr(exe, '/');
		if (!slash) {
			die("cannot find the library, for %s", name);
		}
		*slash = '\0';
	}
	n = strlen(exe) + strlen(dir) + strlen(name) + sizeof("/lib//");
	out = xmalloc(n);
	(void)snprintf(out, n, "%s/lib/%s%s%s", exe, dir, *dir ? "/" : "",
		       name);
	return out;
}

char *lib_path(struct sibling s, const char *dir)
{
	char *p = sibling_path(s);
	FILE *f = fopen(p, "r");

	if (f || s.name[0] == '/') {
		if (f) {
			(void)fclose(f);
		}
		return p;
	}
	free(p);
	return lib_file(dir, s.name);
}

char *join_names(const char *a, const char *b)
{
	size_t na = strlen(a);
	size_t nb = strlen(b);
	char *s = xmalloc(na + nb + 2);

	for (size_t i = 0; i < na; i++) {
		s[i] = a[i];
	}
	s[na] = '/';
	for (size_t i = 0; i <= nb; i++) {
		s[na + 1 + i] = b[i];
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

// A conversion of a format of scan: suppressed, its width (0 none), and
// its letter.
struct conv {
	int skip;
	size_t width;
	char kind;
};

// The conversion at *fmt, past its %, which is set to its letter.
static struct conv conv_at(const char **fmt)
{
	struct conv c = { 0, 0, 0 };
	const char *f = *fmt;

	if (*f == '*') {
		c.skip = 1;
		f++;
	}
	while (isdigit((unsigned char)*f)) {
		c.width = (c.width * DECIMAL_BASE) + (size_t)(*f - '0');
		f++;
	}
	if (*f == 'l') {
		f++;
	}
	c.kind = *f;
	*fmt = f;
	return c;
}

// Reads the word at *p, a run of up to c->width bytes but blanks, into
// the array of the next argument unless suppressed; *p is set past it.
static int scan_word(const char **p, const struct conv *c, va_list *ap)
{
	const char *s = *p;
	char *out = c->skip ? NULL : va_arg(*ap, char *);
	size_t n = 0;

	while (s[n] && !isspace((unsigned char)s[n]) &&
	       (!c->width || n < c->width)) {
		n++;
	}
	if (!n) {
		return 0;
	}
	if (out) {
		memcpy(out, s, n);
		out[n] = 0;
	}
	*p = s + n;
	return 1;
}

// Reads the number at *p by the conversion c into the next argument
// unless suppressed; *p is set past it.  Returns 0 if there is none.
static int scan_number(const char **p, const struct conv *c, va_list *ap)
{
	char *end = NULL;

	if (c->kind == 'd') {
		long v = strtol(*p, &end, DECIMAL_BASE);

		if (end == *p) {
			return 0;
		}
		if (!c->skip) {
			*va_arg(*ap, int *) = (int)v;
		}
	} else {
		double v = strtod(*p, &end);

		if (end == *p) {
			return 0;
		}
		if (!c->skip) {
			*va_arg(*ap, double *) = v;
		}
	}
	*p = end;
	return 1;
}

int scan(const char **in, const char *fmt, ...)
{
	va_list ap;
	const char *s = *in;
	const char *p = s;
	int got = 0;

	va_start(ap, fmt);
	for (; *fmt; fmt++) {
		struct conv c = { 0, 0, 0 };
		int ok = 0;

		if (isspace((unsigned char)*fmt)) {
			while (isspace((unsigned char)*p)) {
				p++;
			}
			continue;
		}
		if (*fmt != '%') {
			if (*p != *fmt) {
				break;
			}
			p++;
			continue;
		}
		fmt++;
		c = conv_at(&fmt);
		if (c.kind == 'n') {
			*va_arg(ap, int *) = (int)(p - s);
			continue;
		}
		while (isspace((unsigned char)*p)) {
			p++;
		}
		ok = c.kind == 's' ? scan_word(&p, &c, &ap) :
				     scan_number(&p, &c, &ap);
		if (!ok) {
			break;
		}
		got += !c.skip;
	}
	va_end(ap);
	*in = p;
	return got;
}
