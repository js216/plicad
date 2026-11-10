// SPDX-License-Identifier: GPL-3.0-only
// numdiff.c -- Compare two text files field by field with numeric tolerances
// Copyright (c) 2026 Jakob Kastelic

// Usage: numdiff [-t LABEL=rel:X|abs:X]... EXPECTED ACTUAL
//
// The files must have the same lines and fields, and the words must match.
// A number matches within the tolerance of its label, which is the last
// word before it on the line.  A number without a tolerance must be
// equal.  Each mismatch is printed on one line, and the exit status is 1
// if there are any.

#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { MAXFIELDS = 64 };

struct tol {
	char *label;
	int rel;
	double x;
};

// The tolerances of the -t options.
struct tols {
	struct tol *t;
	size_t n, cap;
};

static int parse_tol(struct tols *ts, const char *s)
{
	const char *eq = strchr(s, '=');
	char *end = NULL;
	struct tol t;

	if (!eq || eq == s) {
		return -1;
	}
	if (strncmp(eq + 1, "rel:", 4) != 0 &&
	    strncmp(eq + 1, "abs:", 4) != 0) {
		return -1;
	}
	t.x = strtod(eq + 1 + strlen("rel:"), &end);
	if (*end || end == eq + 1 + strlen("rel:") || !(t.x >= 0)) {
		return -1;
	}
	t.label = xmalloc((size_t)(eq - s) + 1);
	memcpy(t.label, s, (size_t)(eq - s));
	t.label[eq - s] = 0;
	t.rel = eq[1] == 'r';
	PUSH(ts->t, ts->n, ts->cap, t);
	return 0;
}

static double limit(const struct tols *ts, const char *label, double expected)
{
	for (size_t i = ts->n; i-- > 0;) {
		const struct tol *t = &ts->t[i];

		if (strcmp(t->label, label) == 0) {
			return t->rel ? t->x * fabs(expected) : t->x;
		}
	}
	return 0;
}

static int number(const char *s, double *v)
{
	char *end = NULL;

	*v = strtod(s, &end);
	return end != s && !*end;
}

// Compares one line and returns the number of mismatches.
static int compare_line(const struct tols *ts, unsigned no, char *a, char *b)
{
	char *x[MAXFIELDS];
	char *y[MAXFIELDS];
	size_t nx = split_words(a, x, MAXFIELDS);
	size_t ny = split_words(b, y, MAXFIELDS);
	const char *label = "";
	int bad = 0;

	if (nx != ny || nx > MAXFIELDS) {
		printf("line %u: different field counts\n", no);
		return 1;
	}
	for (size_t i = 0; i < nx; i++) {
		double u = 0;
		double v = 0;

		if (number(x[i], &u) && number(y[i], &v)) {
			if (!(fabs(u - v) <= limit(ts, label, u))) {
				printf("line %u: %s %s, expected %s\n", no,
				       label, y[i], x[i]);
				bad++;
			}
		} else if (strcmp(x[i], y[i]) != 0) {
			printf("line %u: %s, expected %s\n", no, y[i], x[i]);
			bad++;
		} else {
			label = x[i];
		}
	}
	return bad;
}

static int compare(const struct tols *ts, const char *pa, const char *pb)
{
	char *ta = read_file(pa);
	char *tb = read_file(pb);
	char *ra = ta;
	char *rb = tb;
	char *la = NULL;
	char *lb = NULL;
	unsigned no = 0;
	int bad = 0;

	for (;;) {
		la = next_line(&ra);
		lb = next_line(&rb);
		no++;
		if (!la || !lb) {
			break;
		}
		bad += compare_line(ts, no, la, lb);
	}
	if (la || lb) {
		printf("line %u: %s has more lines\n", no, la ? pa : pb);
		bad++;
	}
	free(ta);
	free(tb);
	return bad;
}

int main(int argc, char **argv)
{
	int i = 1;
	struct tols ts = { NULL, 0, 0 };
	int bad = 0;

	for (; i + 1 < argc && strcmp(argv[i], "-t") == 0; i += 2) {
		if (parse_tol(&ts, argv[i + 1])) {
			die("numdiff: bad tolerance %s", argv[i + 1]);
		}
	}
	if (argc - i != 2) {
		die("usage: numdiff [-t LABEL=rel:X|abs:X]... EXPECTED ACTUAL");
	}
	bad = compare(&ts, argv[i], argv[i + 1]);
	for (size_t k = 0; k < ts.n; k++) {
		free(ts.t[k].label);
	}
	free(ts.t);
	close_stdout();
	return bad ? EXIT_FAILURE : EXIT_SUCCESS;
}
