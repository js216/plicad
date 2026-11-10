// SPDX-License-Identifier: GPL-3.0-only
// expr.c -- Names and arithmetic expressions with length and angle units
// Copyright (c) 2026 Jakob Kastelic

#include "expr.h"
#include "geom.h"
#include "util.h"
#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	UNIT_LEN = 8, // Room for a unit name: in, mm, deg.
	NAME_LEN = 64 // Room for a parameter or function name.
};

// The floor() of a quotient like (b - a) / p can come out a hair below an
// exact integer.  FLOOR_SLACK covers that.
#define FLOOR_SLACK 1e-9

const char *expr_where(const struct names *nm)
{
	return nm->where ? nm->where : "";
}

void expr_fail_msg(const struct names *nm, const char *msg, const char *near)
{
	if (near) {
		warn("%s: %s\n  near: %s", expr_where(nm), msg, near);
	} else {
		warn("%s: %s", expr_where(nm), msg);
	}
}

void free_names(struct names *nm)
{
	for (size_t i = 0; i < nm->nsyms; i++) {
		free(nm->syms[i].name);
		free(nm->syms[i].deps);
	}
	free(nm->syms);
	free(nm->seen);
	memset(nm, 0, sizeof(*nm));
}

void define(struct names *nm, const char *name, double v, int from_include)
{
	struct sym s = { xstrdup(name), v, from_include, 0, 0, NULL, 0 };

	PUSH(nm->syms, nm->nsyms, nm->capsyms, s);
}

void pin(struct names *nm, const char *name, double v)
{
	struct sym s = { xstrdup(name), v, 0, 0, 1, NULL, 0 };

	PUSH(nm->syms, nm->nsyms, nm->capsyms, s);
}

// Marks name i used, and the names its definition uses, and theirs; each
// name goes on the stack once, when it is marked.
static void mark_used(struct names *nm, size_t i)
{
	size_t *stack = xmalloc((nm->nsyms + 1) * sizeof(*stack));
	size_t sp = 0;

	if (nm->syms[i].used) {
		free(stack);
		return;
	}
	nm->syms[i].used = 1;
	stack[sp++] = i;
	while (sp) {
		const struct sym *s = &nm->syms[stack[--sp]];

		for (size_t k = 0; k < s->ndeps; k++) {
			struct sym *d = &nm->syms[s->deps[k]];

			if (!d->used) {
				d->used = 1;
				stack[sp++] = s->deps[k];
			}
		}
	}
	free(stack);
}

// Looks up a name and marks it used, with the names its definition uses;
// returns 0 if it is undefined.
static int lookup(struct names *nm, const char *name, double *v)
{
	int found = 0;

	for (size_t i = nm->nsyms; i-- > 0;) {
		if (strcmp(nm->syms[i].name, name) != 0 ||
		    !nm->syms[i].pinned) {
			continue;
		}
		*v = nm->syms[i].v;
		found = 1;
	}
	for (size_t i = nm->nsyms; i-- > 0;) {
		if (strcmp(nm->syms[i].name, name) != 0 || nm->syms[i].pinned) {
			continue;
		}
		if (nm->reading) {
			PUSH(nm->seen, nm->nseen, nm->capseen, i);
		} else {
			mark_used(nm, i);
		}
		if (!found) {
			*v = nm->syms[i].v;
		}
		return 1;
	}
	return found;
}

// Is symbol i the first one of its name?
static int first_of_name(const struct names *nm, size_t i)
{
	for (size_t j = 0; j < i; j++) {
		if (strcmp(nm->syms[j].name, nm->syms[i].name) == 0) {
			return 0;
		}
	}
	return 1;
}

void each_param(struct names *nm, int only_used,
		void (*fn)(const char *name, void *ctx), void *ctx)
{
	for (size_t i = 0; i < nm->nsyms; i++) {
		if (nm->syms[i].from_include &&
		    (!only_used || nm->syms[i].used) && first_of_name(nm, i)) {
			fn(nm->syms[i].name, ctx);
		}
	}
}

static int is_name_start(char c)
{
	return isalpha((unsigned char)c) || c == '_';
}

static int is_name_char(char c)
{
	return isalnum((unsigned char)c) || c == '_';
}

static void skip_space(const char **s)
{
	while (**s == ' ' || **s == '\t') {
		(*s)++;
	}
}

// Copies the name at s into b, of size max, and returns its length, or 0.
static size_t name_at(const char *s, char *b, size_t max)
{
	size_t n = 0;

	if (!is_name_start(*s)) {
		return 0;
	}
	while (is_name_char(s[n]) && n + 1 < max) {
		b[n] = s[n];
		n++;
	}
	b[n] = 0;
	return n;
}

static double apply_unit(const char **s, double v)
{
	char b[UNIT_LEN];
	const char *p = *s;
	size_t n = 0;

	skip_space(&p);
	n = name_at(p, b, sizeof(b));
	if (n == 0 || is_name_char(p[n])) {
		return v;
	}
	if (strcmp(b, "in") == 0) {
		v *= INCH;
	} else if (strcmp(b, "mm") != 0 && strcmp(b, "deg") != 0) {
		return v;
	}
	*s = p + n;
	return v;
}

enum fn {
	F_FLOOR,
	F_TAN,
	F_SIN,
	F_COS,
	F_SQRT,
	F_MAX,
	F_HYPOT,
	F_ATAN2,
	F_COUNT
};

static const struct {
	const char *name;
	size_t nargs;
} fns[F_COUNT] = {
	{ "floor", 1 }, { "tan", 1 }, { "sin", 1 },   { "cos", 1 },
	{ "sqrt", 1 },	{ "max", 2 }, { "hypot", 2 }, { "atan2", 2 },
};

static double apply(enum fn f, const double *a)
{
	const double rad = PI / 180;

	switch (f) {
	case F_FLOOR:
		return floor(a[0] + FLOOR_SLACK);
	case F_TAN:
		return tan(a[0] * rad);
	case F_SIN:
		return sin(a[0] * rad);
	case F_COS:
		return cos(a[0] * rad);
	case F_SQRT:
		return sqrt(a[0]);
	case F_MAX:
		return fmax(a[0], a[1]);
	case F_HYPOT:
		return hypot(a[0], a[1]);
	case F_ATAN2:
		return atan2(a[0], a[1]) / rad;
	case F_COUNT:
	default:
		break;
	}
	return 0;
}

// The function of this name, or F_COUNT if there is none.
static enum fn find_fn(const char *name)
{
	for (int i = 0; i < F_COUNT; i++) {
		if (strcmp(fns[i].name, name) == 0) {
			return (enum fn)i;
		}
	}
	return F_COUNT;
}

static double number(const struct names *nm, const char **s, const char *start)
{
	char *end = NULL;
	double v = strtod(*s, &end);

	if (end == *s) {
		expr_fail(nm, "bad number", start);
	}
	*s = end;
	return v;
}

// The evaluator is a loop over the text with two stacks of fixed depth,
// the values and the pending operators, so deep nesting cannot overflow
// the C stack.  An operator is a binary + - * /, a unary sign, an open
// parenthesis or an open function call.
enum { MAXDEPTH = 64 };

enum op_kind {
	OP_ADD,
	OP_SUB,
	OP_MUL,
	OP_DIV,
	OP_NEG,
	OP_POS,
	OP_PAREN,
	OP_CALL
};

struct op {
	enum op_kind kind;
	enum fn fn; // OP_CALL: the function.
	size_t nargs; // OP_CALL: arguments so far.
	const char *start; // Where the parenthesis or the call starts.
};

struct machine {
	double vals[MAXDEPTH];
	size_t nvals;
	struct op ops[MAXDEPTH];
	size_t nops;
};

static void push_val(const struct names *nm, struct machine *m, double v)
{
	if (m->nvals == MAXDEPTH) {
		expr_fail(nm, "the expression nests too deep", NULL);
	}
	m->vals[m->nvals++] = v;
}

static void push_op(const struct names *nm, struct machine *m, struct op o)
{
	if (m->nops == MAXDEPTH) {
		expr_fail(nm, "the expression nests too deep", NULL);
	}
	m->ops[m->nops++] = o;
}

// The binding of operator k: higher binds first.
static int precedence(enum op_kind k)
{
	if (k == OP_ADD || k == OP_SUB) {
		return 1;
	}
	if (k == OP_MUL || k == OP_DIV) {
		return 2;
	}
	return 3;
}

// Pops the top operator, a sign or a binary one, and applies it.
static void reduce(const struct names *nm, struct machine *m)
{
	struct op o = m->ops[--m->nops];
	double b = m->vals[--m->nvals];

	if (o.kind == OP_NEG || o.kind == OP_POS) {
		push_val(nm, m, o.kind == OP_NEG ? -b : b);
		return;
	}
	m->nvals--;
	if (o.kind == OP_ADD) {
		push_val(nm, m, m->vals[m->nvals] + b);
	} else if (o.kind == OP_SUB) {
		push_val(nm, m, m->vals[m->nvals] - b);
	} else if (o.kind == OP_MUL) {
		push_val(nm, m, m->vals[m->nvals] * b);
	} else {
		push_val(nm, m, m->vals[m->nvals] / b);
	}
}

// Applies the signs and binary operators on top that bind at least as
// tightly as level.
static void reduce_to(const struct names *nm, struct machine *m, int level)
{
	while (m->nops && m->ops[m->nops - 1].kind != OP_PAREN &&
	       m->ops[m->nops - 1].kind != OP_CALL &&
	       precedence(m->ops[m->nops - 1].kind) >= level) {
		reduce(nm, m);
	}
}

// Reads an operand at *s: a sign, an opening parenthesis or call, which
// leave an operand still to come, or a number or a name with its unit.
// Returns whether a value was pushed.
static int operand(struct names *nm, struct machine *m, const char **s)
{
	const char *start = *s;
	struct op o = { OP_PAREN, F_FLOOR, 0, start };
	char b[NAME_LEN];
	size_t n = 0;
	double v = 0;

	if (**s == '-' || **s == '+') {
		o.kind = **s == '-' ? OP_NEG : OP_POS;
		(*s)++;
		push_op(nm, m, o);
		return 0;
	}
	if (**s == '(') {
		(*s)++;
		push_op(nm, m, o);
		return 0;
	}
	if (isdigit((unsigned char)**s) || **s == '.') {
		push_val(nm, m, apply_unit(s, number(nm, s, start)));
		return 1;
	}
	n = name_at(*s, b, sizeof(b));
	if (n == 0) {
		expr_fail(nm, "expected a number or a name", start);
	}
	*s += n;
	skip_space(s);
	if (**s == '(') {
		o.kind = OP_CALL;
		o.fn = find_fn(b);
		if (o.fn == F_COUNT) {
			expr_fail(nm, "unknown function", start);
		}
		o.nargs = 1;
		(*s)++;
		push_op(nm, m, o);
		return 0;
	}
	if (!lookup(nm, b, &v)) {
		expr_fail(nm, "unknown name", start);
	}
	push_val(nm, m, apply_unit(s, v));
	return 1;
}

// Closes the innermost parenthesis or call at a ')' and applies the unit
// after it.
static void close_group(const struct names *nm, struct machine *m,
			const char **s)
{
	struct op o;

	reduce_to(nm, m, 0);
	if (!m->nops) {
		expr_fail(nm, "junk after the expression", *s);
	}
	o = m->ops[--m->nops];
	(*s)++;
	if (o.kind == OP_CALL) {
		double a[2] = { 0, 0 };

		if (o.nargs != fns[o.fn].nargs) {
			expr_fail(nm, "wrong number of function arguments",
				  o.start);
		}
		for (size_t i = o.nargs; i > 0; i--) {
			a[i - 1] = m->vals[--m->nvals];
		}
		push_val(nm, m, apply(o.fn, a));
	}
	push_val(nm, m, apply_unit(s, m->vals[--m->nvals]));
}

// Takes the next argument of the innermost call at a ','.
static void next_arg(const struct names *nm, struct machine *m, const char **s)
{
	reduce_to(nm, m, 0);
	if (!m->nops || m->ops[m->nops - 1].kind != OP_CALL) {
		if (m->nops) {
			expr_fail(nm, "expected )", m->ops[m->nops - 1].start);
		}
		expr_fail(nm, "junk after the expression", *s);
	}
	if (++m->ops[m->nops - 1].nargs > 2) {
		expr_fail(nm, "wrong number of function arguments",
			  m->ops[m->nops - 1].start);
	}
	(*s)++;
}

// Reads an operator at *s; returns 0 at the end of the expression.
static int after_value(const struct names *nm, struct machine *m,
		       const char **s)
{
	static const char sym[] = "+-*/";
	static const enum op_kind kinds[] = { OP_ADD, OP_SUB, OP_MUL, OP_DIV };
	const char *k = **s ? strchr(sym, **s) : NULL;
	struct op o = { OP_ADD, F_FLOOR, 0, *s };

	if (k) {
		o.kind = kinds[k - sym];
		reduce_to(nm, m, precedence(o.kind));
		push_op(nm, m, o);
		(*s)++;
		return 1;
	}
	if (**s == ')') {
		close_group(nm, m, s);
		return 2;
	}
	if (**s == ',') {
		next_arg(nm, m, s);
		return 1;
	}
	return 0;
}

// Evaluates the expression at *s and leaves *s after it.
static double expr(struct names *nm, const char **s)
{
	struct machine m;
	int want_value = 1;

	m.nvals = 0;
	m.nops = 0;
	for (;;) {
		int r = 0;

		skip_space(s);
		if (want_value) {
			want_value = !operand(nm, &m, s);
			continue;
		}
		r = after_value(nm, &m, s);
		if (!r) {
			break;
		}
		want_value = r == 1;
	}
	reduce_to(nm, &m, 0);
	if (m.nops) {
		const struct op *o = &m.ops[m.nops - 1];

		expr_fail(nm,
			  o->kind == OP_CALL ?
				  "expected ) after the function arguments" :
				  "expected )",
			  o->start);
	}
	return m.vals[0];
}

double eval(struct names *nm, const char *s)
{
	const char *p = s;
	double v = expr(nm, &p);

	skip_space(&p);
	if (*p) {
		expr_fail(nm, "junk after the expression", p);
	}
	if (!num_ok(v)) {
		expr_fail(nm, "the value is not a finite number within 1e6", s);
	}
	return v;
}

void each_line(struct names *nm, const char *path,
	       void (*fn)(char *line, void *ctx), void *ctx)
{
	char *text = read_file(path);
	char *rest = text;
	const char *old = nm->where;
	char where[WHERE_LEN];
	unsigned lineno = 0;
	char *l = NULL;

	while ((l = next_line(&rest)) != NULL) {
		char *hash = strchr(l, '#');

		lineno++;
		if (hash) {
			*hash = 0;
		}
		xsnprintf(where, "%s:%u", path, lineno);
		nm->where = where;
		fn(l, ctx);
	}
	nm->where = old;
	free(text);
}

// Reads one "name = expr" line into the names ctx; the name is cut off
// in place.  The type of the each_line callback fixes the char *.
static void param_line(char *l, void *ctx)
{
	struct names *nm = ctx;
	size_t a = strspn(l, " \t");
	size_t e = a;
	size_t q = 0;
	struct sym *s = NULL;

	if (!l[a]) {
		return;
	}
	if (is_name_start(l[e])) {
		while (is_name_char(l[e])) {
			e++;
		}
	}
	q = e + strspn(l + e, " \t");
	if (e == a || l[q] != '=') {
		expr_fail(nm, "expected name = expression", l + a);
	}
	l[e] = 0;
	nm->nseen = 0;
	define(nm, l + a, eval(nm, l + q + 1), 1);
	s = &nm->syms[nm->nsyms - 1];
	s->ndeps = nm->nseen;
	s->deps = xmalloc((nm->nseen + 1) * sizeof(*s->deps));
	for (size_t i = 0; i < nm->nseen; i++) {
		s->deps[i] = nm->seen[i];
	}
}

void read_params(struct names *nm, const char *path)
{
	bool was = nm->reading;

	nm->reading = true;
	each_line(nm, path, param_line, nm);
	nm->reading = was;
}
