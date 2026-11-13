// SPDX-License-Identifier: GPL-3.0-only
// stream.c -- The text that runs down a pipe of tools: solids and source
// Copyright (c) 2026 Jakob Kastelic

#include "stream.h"
#include "expr.h"
#include "shape.h"
#include "util.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char magic[] = "PliCad 1";

enum {
	HEAD_WORDS = 2, // The words of a record outside a solid.
	DECIMAL = 10 // The base of the line numbers.
};

static void add_line(struct stream *st, unsigned no, const char *text)
{
	struct sline l = { no, xstrdup(text), 0 };

	PUSH(st->ln, st->nln, st->capln, l);
}

// Reads the lines of a source file, their comments cut off.
static void read_source(struct stream *st, char *text)
{
	char *rest = text;
	char *l = NULL;
	unsigned no = 0;

	while ((l = next_line(&rest)) != NULL) {
		char *hash = strchr(l, '#');
		size_t n = 0;

		no++;
		if (hash) {
			*hash = 0;
		}
		n = strlen(l);
		while (n && isspace((unsigned char)l[n - 1])) {
			l[--n] = 0;
		}
		if (l[strspn(l, " \t")]) {
			add_line(st, no, l);
		}
	}
}

// Reads one line of a stream: a header, a record of a solid, or a
// source line.
static void stream_record(struct stream *st, char *l, int *in_brep, unsigned no)
{
	char *w[HEAD_WORDS + 1];
	char where[WHERE_LEN];
	size_t n = 0;

	xsnprintf(where, "stream:%u", no);
	if (l[0] == '@') {
		char *end = NULL;
		unsigned long k = strtoul(l + 1, &end, DECIMAL);

		if (end == l + 1 || *end != ' ') {
			die("%s: bad source line", where);
		}
		add_line(st, (unsigned)k, end + 1);
		return;
	}
	if (*in_brep) {
		*in_brep = !shape_record(&st->sh[st->nsh - 1], l, where);
		return;
	}
	n = split_words(l, w, HEAD_WORDS);
	if (!n || n > HEAD_WORDS) {
		die("%s: bad record", where);
	}
	if (strcmp(w[0], "source") == 0 && n == 2) {
		free(st->src);
		st->src = xstrdup(w[1]);
	} else if (strcmp(w[0], "brep") == 0 && n == 2) {
		struct shape sh;

		shape_init(&sh, w[1]);
		PUSH(st->sh, st->nsh, st->capsh, sh);
		*in_brep = 1;
	} else {
		die("%s: unknown record %s", where, w[0]);
	}
}

static void read_stream(struct stream *st, char *text)
{
	char *rest = text;
	char *l = next_line(&rest);
	int in_brep = 0;
	unsigned no = 1;

	(void)l;
	while ((l = next_line(&rest)) != NULL) {
		no++;
		if (*l) {
			stream_record(st, l, &in_brep, no);
		}
	}
	if (in_brep) {
		die("stream: a solid has no end");
	}
	for (size_t i = 0; i < st->nsh; i++) {
		shape_check(&st->sh[i], "stream");
	}
}

static char *read_all(const char *path)
{
	char *buf = NULL;
	size_t n = 0;
	size_t cap = 0;
	int c = 0;

	if (strcmp(path, "-") != 0) {
		return read_file(path);
	}
	while ((c = getchar()) != EOF) {
		PUSH(buf, n, cap, (char)c);
	}
	PUSH(buf, n, cap, 0);
	return buf;
}

void stream_read(struct stream *st, const char *path)
{
	char *text = read_all(path);

	memset(st, 0, sizeof(*st));
	if (strncmp(text, magic, strlen(magic)) == 0) {
		read_stream(st, text);
	} else {
		st->src = xstrdup(path);
		read_source(st, text);
	}
	if (!st->src) {
		die("%s: a stream with no source", path);
	}
	free(text);
}

void stream_write(const struct stream *st)
{
	printf("%s\nsource %s\n", magic, st->src);
	for (size_t i = 0; i < st->nsh; i++) {
		shape_write(stdout, &st->sh[i]);
	}
	for (size_t i = 0; i < st->nln; i++) {
		if (!st->ln[i].gone) {
			printf("@%u %s\n", st->ln[i].no, st->ln[i].text);
		}
	}
}

void stream_free(struct stream *st)
{
	for (size_t i = 0; i < st->nsh; i++) {
		shape_free(&st->sh[i]);
	}
	for (size_t i = 0; i < st->nln; i++) {
		free(st->ln[i].text);
	}
	free(st->sh);
	free(st->ln);
	free(st->src);
	free(st->buf);
	free_names(&st->nm);
	memset(st, 0, sizeof(*st));
}

// Evaluates a line NAME = EXPR, or include FILE; returns 0 if it is
// neither.
static int define_line(struct stream *st, const char *text)
{
	const char *eq = strchr(text, '=');
	size_t a = strspn(text, " \t");
	size_t e = a;
	char name[ARGS_MAX];

	while (text[e] == '_' || isalnum((unsigned char)text[e])) {
		e++;
	}
	if (!eq || e == a || e - a >= sizeof(name) ||
	    eq != text + e + strspn(text + e, " \t")) {
		return 0;
	}
	memcpy(name, text + a, e - a);
	name[e - a] = 0;
	define(&st->nm, name, eval(&st->nm, eq + 1), 0);
	return 1;
}

int stream_line(struct stream *st, size_t i)
{
	const struct sline *l = &st->ln[i];

	xsnprintf(st->where, "%s:%u", st->src, l->no);
	st->nm.where = st->where;
	free(st->buf);
	st->buf = xstrdup(l->text);
	st->nw = split_words(st->buf, st->w, ARGS_MAX);
	if (st->nw > ARGS_MAX) {
		stream_fail(st, "too many words", NULL);
	}
	if (!st->nw || define_line(st, l->text)) {
		return 0;
	}
	return 1;
}

double stream_num(struct stream *st, size_t i)
{
	if (i >= st->nw) {
		stream_fail(st, "a number is missing", NULL);
	}
	return eval(&st->nm, st->w[i]);
}

int stream_is(const struct stream *st, size_t i, const char *w)
{
	return i < st->nw && strcmp(st->w[i], w) == 0;
}

// The solid of that name, or NULL.
static struct shape *stream_find(struct stream *st, const char *name)
{
	for (size_t i = 0; i < st->nsh; i++) {
		if (strcmp(st->sh[i].name, name) == 0) {
			return &st->sh[i];
		}
	}
	return NULL;
}

void stream_add(struct stream *st, struct shape *sh)
{
	if (stream_find(st, sh->name)) {
		stream_fail(st, "a solid of that name exists", sh->name);
	}
	PUSH(st->sh, st->nsh, st->capsh, *sh);
	memset(sh, 0, sizeof(*sh));
}

void stream_take(struct stream *st, const char *name, struct shape *out)
{
	for (size_t i = 0; i < st->nsh; i++) {
		if (strcmp(st->sh[i].name, name) == 0) {
			*out = st->sh[i];
			for (size_t k = i + 1; k < st->nsh; k++) {
				st->sh[k - 1] = st->sh[k];
			}
			st->nsh--;
			return;
		}
	}
	stream_fail(st, "no solid of that name", name);
}

void stream_refuse_left(struct stream *st)
{
	for (size_t i = 0; i < st->nln; i++) {
		if (!st->ln[i].gone && stream_line(st, i)) {
			stream_fail(st, "no tool took this statement",
				    st->w[0]);
		}
	}
}

int stream_file(const char *path)
{
	FILE *f = NULL;
	char head[sizeof(magic)];
	size_t n = 0;

	if (strcmp(path, "-") == 0) {
		return 1;
	}
	f = fopen(path, "r");
	if (!f) {
		die("%s: cannot open", path);
	}
	n = fread(head, 1, strlen(magic), f);
	head[n] = 0;
	(void)fclose(f);
	return strcmp(head, magic) == 0;
}
