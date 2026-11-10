// SPDX-License-Identifier: GPL-3.0-only
// expr.h -- Names and arithmetic expressions with length and angle units
// Copyright (c) 2026 Jakob Kastelic

#ifndef EXPR_H
#define EXPR_H

#include <stddef.h>
#include <stdlib.h>

// Grammar:
//   expr    = term { (+|-) term }
//   term    = unary { (*|/) unary }
//   unary   = (-|+) unary | primary [unit]
//   primary = number | name | name ( expr {, expr} ) | ( expr )
//   unit    = in | mm | deg
// Lengths are in mm (1 in = 25.4 mm).  Angles are in degrees, and "deg"
// is 1.  The functions are floor tan sin cos sqrt max hypot atan2, with
// the trigonometric ones in degrees.  Every value must be finite and no
// larger than 1e6 either way.

// A defined name, its value, and the names its definition uses (indices
// of earlier names).
struct sym {
	char *name;
	double v;
	int from_include;
	int used;
	int pinned;
	size_t *deps;
	size_t ndeps;
};

// The names of a program, and the file and line that it reads, as
// "file:line", for error messages.  A zero struct names has none.
// While read_params runs, the lookups mark no names used but collect
// the names of each definition in seen.
struct names {
	struct sym *syms;
	size_t nsyms, capsyms;
	int reading;
	const char *where;
	size_t *seen;
	size_t nseen, capseen;
};

// The text of where in nm, "" before any file.
const char *expr_where(const struct names *nm);

void expr_fail_msg(const struct names *nm, const char *msg, const char *near);
#define expr_fail(nm, msg, near)                    \
	do {                                        \
		expr_fail_msg((nm), (msg), (near)); \
		exit(EXIT_FAILURE);                 \
	} while (0)

void define(struct names *nm, const char *name, double v, int from_include);
// Pins name to v.  A pinned value has priority over every definition.
void pin(struct names *nm, const char *name, double v);

// Evaluates the string as one expression.
double eval(struct names *nm, const char *s);

// Calls fn for each line of the file, with the # comment cut off and
// where set to the file and line.
void each_line(struct names *nm, const char *path,
	       void (*fn)(char *line, void *ctx), void *ctx);

// Reads "name = expr" lines from a file.
void read_params(struct names *nm, const char *path);

// Calls fn for each name read from include files, in definition order:
// only those used since if only_used, else all.
void each_param(struct names *nm, int only_used,
		void (*fn)(const char *name, void *ctx), void *ctx);

void free_names(struct names *nm);

#endif
