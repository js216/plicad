// SPDX-License-Identifier: GPL-3.0-only
// expr.h -- Names and arithmetic expressions with length and angle units
// Copyright (c) 2026 Jakob Kastelic

#ifndef EXPR_H
#define EXPR_H

#include <setjmp.h>
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
//
// Each value has a range, by interval arithmetic: a number is exact, a
// name has the range of its definition, and each operator and function
// gives the least range of its result over the ranges of its operands.
// A definition NAME = EXPR whose EXPR names no other name is a
// parameter.  A parameter of a length (EXPR has in or mm) or an angle
// (deg and neither of those) takes its range from its own tolerance,
// NAME = EXPR +- TOL or NAME = EXPR + UP - DOWN, or else from the
// statement "tolerance LEN ANGLE" of a file of parameters above it;
// with neither it is an error.  A pure number (EXPR has no unit) is
// exact unless it states a tolerance.  A definition that names names
// takes the range of its expression, and states none.

// What a parameter measures.
enum { KIND_NUMBER, KIND_LENGTH, KIND_ANGLE, KINDS };

// A value and its range, lo <= v <= hi.
struct ival {
	double v, lo, hi;
};

// A defined name, its value and range, and the names its definition
// uses (indices of earlier names).  A parameter has base set, its kind,
// and the file and line of its definition.
struct sym {
	char *name;
	double v, lo, hi;
	int from_include;
	int used;
	int pinned;
	int base, kind;
	char *where;
	size_t *deps;
	size_t ndeps;
};

// The names of a program, and the file and line that it reads, as
// "file:line", for error messages.  A zero struct names has none.
// While read_params runs, the lookups mark no names used but collect
// the names of each definition in seen.  With a trap, an error writes
// its message into msg, of size nmsg, and jumps to the trap instead of
// ending the program.  The tolerances of a length and of an angle in
// force, if tol_set, are those of the last tolerance statement read.  A
// rebuild with a parameter moved sets vary to 1 plus the index of its
// name, and vary_by to the move: that definition takes its value plus
// vary_by, exact.
struct names {
	struct sym *syms;
	size_t nsyms, capsyms;
	int reading;
	const char *where;
	size_t *seen;
	size_t nseen, capseen;
	jmp_buf *trap;
	char *msg;
	size_t nmsg;
	int tol_set;
	double tol_len, tol_ang;
	size_t vary;
	double vary_by;
};

// A line of a source file as the parsers take it: its number in the
// file (the first of its lines, if it runs on) and its text.
struct src_line {
	unsigned no;
	char *text;
};

// Splits text into the lines of a source file (plicad(7)): a # begins
// a comment, which runs to the end of the line; a line that then ends
// in a backslash runs on into the next one, the backslash and the blanks
// round it taken for one blank; lines with nothing left are dropped.
// Returns their number.
size_t source_lines(char *text, struct src_line **out);
void free_source_lines(struct src_line *l, size_t n);

// The text of where in nm, "" before any file.
const char *expr_where(const struct names *nm);

void expr_fail_msg(const struct names *nm, const char *msg, const char *near);
// Jumps to the trap of nm, if it has one.
void expr_trap(const struct names *nm);
#define expr_fail(nm, msg, near)                    \
	do {                                        \
		expr_fail_msg((nm), (msg), (near)); \
		expr_trap(nm);                      \
		exit(EXIT_FAILURE);                 \
	} while (0)

// Defines name by the text of its definition, EXPR with its tolerance,
// if any, as the grammar above says.
void define(struct names *nm, const char *name, const char *text,
	    int from_include);
// Pins name to v, exact.  A pinned value has priority over every
// definition.
void pin(struct names *nm, const char *name, double v);

// Evaluates the string as one expression.
double eval(struct names *nm, const char *s);
// Evaluates the string as one expression, with its range.
struct ival eval_range(struct names *nm, const char *s);

// Calls fn for each line of the file (source_lines), with where set to
// the file and line.
void each_line(struct names *nm, const char *path,
	       void (*fn)(char *line, void *ctx), void *ctx);

// Reads "name = expr" lines from a file, and "tolerance LEN ANGLE".
void read_params(struct names *nm, const char *path);
// Reads the line l if it is a statement tolerance LEN ANGLE, as a file
// of parameters has it, and returns whether it is.
int tolerance_line(struct names *nm, const char *l);

// Calls fn for each parameter of a range that the names used since
// lean on, in definition order, with its index, unless a pinned value
// takes its place.
void each_tolerance(struct names *nm,
		    void (*fn)(const struct sym *s, size_t i, void *ctx),
		    void *ctx);
// The word of a kind: number, length or angle.
const char *kind_word(int kind);

// Calls fn for each name read from include files, in definition order:
// only those used since if only_used, else all.
void each_param(struct names *nm, int only_used,
		void (*fn)(const char *name, void *ctx), void *ctx);

void free_names(struct names *nm);

#endif
