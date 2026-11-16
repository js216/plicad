// SPDX-License-Identifier: GPL-3.0-only
// expr.c -- Names and arithmetic expressions with length and angle units
// Copyright (c) 2026 Jakob Kastelic

#include "expr.h"
#include "geom.h"
#include "util.h"
#include <ctype.h>
#include <math.h>
#include <setjmp.h>
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
	if (nm->trap && nm->msg && nm->nmsg) {
		const char *part[] = { expr_where(nm), ": ", msg,
				       near ? "\n  near: " : "",
				       near ? near : "" };
		size_t at = 0;

		for (size_t i = 0; i < sizeof(part) / sizeof(part[0]); i++) {
			size_t k = strlen(part[i]);

			if (at + k >= nm->nmsg) {
				k = nm->nmsg - 1 - at;
			}
			memcpy(nm->msg + at, part[i], k);
			at += k;
		}
		nm->msg[at] = 0;
		return;
	}
	if (near) {
		warn("%s: %s\n  near: %s", expr_where(nm), msg, near);
	} else {
		warn("%s: %s", expr_where(nm), msg);
	}
}

void expr_trap(const struct names *nm)
{
	if (nm->trap) {
		longjmp(*nm->trap, 1);
	}
}

void free_names(struct names *nm)
{
	for (size_t i = 0; i < nm->nsyms; i++) {
		free(nm->syms[i].name);
		free(nm->syms[i].where);
		free(nm->syms[i].deps);
	}
	free(nm->syms);
	free(nm->seen);
	memset(nm, 0, sizeof(*nm));
}

void pin(struct names *nm, const char *name, double v)
{
	struct sym s;

	memset(&s, 0, sizeof(s));
	s.name = xstrdup(name);
	s.v = s.lo = s.hi = v;
	s.pinned = 1;
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

static struct ival exact(double v)
{
	struct ival r = { v, v, v };

	return r;
}

// Looks up a name and marks it used, with the names its definition uses;
// returns 0 if it is undefined.
static int lookup(struct names *nm, const char *name, struct ival *v)
{
	int found = 0;

	for (size_t i = nm->nsyms; i-- > 0;) {
		if (strcmp(nm->syms[i].name, name) != 0 ||
		    !nm->syms[i].pinned) {
			continue;
		}
		*v = exact(nm->syms[i].v);
		found = 1;
	}
	for (size_t i = nm->nsyms; i-- > 0;) {
		const struct sym *s = &nm->syms[i];

		if (strcmp(s->name, name) != 0 || s->pinned) {
			continue;
		}
		if (nm->reading) {
			PUSH(nm->seen, nm->nseen, nm->capseen, i);
		} else {
			mark_used(nm, i);
		}
		if (!found) {
			struct ival r = { s->v, s->lo, s->hi };

			*v = r;
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

// Whether a pinned value takes the place of the name of symbol i.
static int pinned_name(const struct names *nm, size_t i)
{
	for (size_t j = 0; j < nm->nsyms; j++) {
		if (nm->syms[j].pinned &&
		    strcmp(nm->syms[j].name, nm->syms[i].name) == 0) {
			return 1;
		}
	}
	return 0;
}

void each_tolerance(struct names *nm,
		    void (*fn)(const struct sym *s, size_t i, void *ctx),
		    void *ctx)
{
	for (size_t i = 0; i < nm->nsyms; i++) {
		const struct sym *s = &nm->syms[i];

		if (s->base && s->used && !s->pinned &&
		    (s->lo < s->v || s->hi > s->v) && !pinned_name(nm, i)) {
			fn(s, i, ctx);
		}
	}
}

const char *kind_word(int kind)
{
	static const char *const w[KINDS] = { "number", "length", "angle" };

	return kind >= 0 && kind < KINDS ? w[kind] : "number";
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

// What an evaluation saw: whether it named a name, the last two of its
// binary + and - outside parentheses, and the last such + right before
// a - (a tolerance +-).
struct scan {
	int named;
	const char *add[2];
	const char *plus_minus;
};

// A value with its range, and what it measures: the powers of a length
// and of an angle in it, from its units.  A number without a unit
// takes the measure of what it is added to.
struct qty {
	struct ival r;
	int len, ang;
};

static struct qty scale(struct qty v, double k)
{
	struct qty q = { { v.r.v * k, v.r.lo * k, v.r.hi * k }, v.len, v.ang };

	return q;
}

// The kind of what q measures.
static int kind_of(struct qty q)
{
	if (q.len == 1 && q.ang == 0) {
		return KIND_LENGTH;
	}
	return q.len == 0 && q.ang == 1 ? KIND_ANGLE : KIND_NUMBER;
}

static struct qty apply_unit(const char **s, struct qty v)
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
		v = scale(v, INCH);
		v.len++;
	} else if (strcmp(b, "mm") == 0) {
		v.len++;
	} else if (strcmp(b, "deg") == 0) {
		v.ang++;
	} else {
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
	F_MIN,
	F_ABS,
	F_CEIL,
	F_COUNT
};

static const struct {
	const char *name;
	size_t nargs;
} fns[F_COUNT] = {
	{ "floor", 1 }, { "tan", 1 }, { "sin", 1 },   { "cos", 1 },
	{ "sqrt", 1 },	{ "max", 2 }, { "hypot", 2 }, { "atan2", 2 },
	{ "min", 2 },	{ "abs", 1 }, { "ceil", 1 },
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
	case F_MIN:
		return fmin(a[0], a[1]);
	case F_ABS:
		return fabs(a[0]);
	case F_CEIL:
		return ceil(a[0] - FLOOR_SLACK);
	case F_COUNT:
	default:
		break;
	}
	return 0;
}

// Whether a + k period lies in [lo, hi] for some whole k.
static int holds(double lo, double hi, double a, double period)
{
	return a + (ceil((lo - a) / period) * period) <= hi;
}

// The range of |x| over the range of x.
static struct ival abs_range(struct ival x)
{
	struct ival r = { fabs(x.v), fmin(fabs(x.lo), fabs(x.hi)),
			  fmax(fabs(x.lo), fabs(x.hi)) };

	if (x.lo <= 0 && x.hi >= 0) {
		r.lo = 0;
	}
	return r;
}

// The range of atan2(y, x) over the box of the ranges of y and x: at its
// corners, unless it holds the origin, turned whole across the negative
// x axis.
static struct ival atan2_range(const struct names *nm, struct ival y,
			       struct ival x, double v)
{
	const double rad = PI / 180;
	int cut = y.lo <= 0 && y.hi >= 0 && x.hi < 0;
	struct ival r = { v, HUGE_VAL, -HUGE_VAL };

	if (y.lo <= 0 && y.hi >= 0 && x.lo <= 0 && x.hi >= 0) {
		expr_fail(nm, "the range of atan2 holds the origin", NULL);
	}
	for (int k = 0; k < 4; k++) {
		double a =
			atan2(k & 1 ? y.hi : y.lo, k & 2 ? x.hi : x.lo) / rad;

		if (cut && a < 0) {
			a += 360;
		}
		r.lo = fmin(r.lo, a);
		r.hi = fmax(r.hi, a);
	}
	if (cut && v < 0) {
		r.lo -= 360;
		r.hi -= 360;
	}
	return r;
}

// Function f of the arguments a, with the range of its value.
static struct ival apply_range(const struct names *nm, enum fn f,
			       const struct ival *a)
{
	double v[2] = { a[0].v, a[1].v };
	double lo[2] = { a[0].lo, a[1].lo };
	double hi[2] = { a[0].hi, a[1].hi };
	struct ival r = exact(apply(f, v));

	if (!(a[0].lo < a[0].hi) && !(a[1].lo < a[1].hi)) {
		return r;
	}
	switch (f) {
	case F_TAN:
		if (floor((lo[0] + 90) / 180) < floor((hi[0] + 90) / 180)) {
			expr_fail(nm, "the range of tan holds a pole", NULL);
		}
		r.lo = apply(f, lo);
		r.hi = apply(f, hi);
		break;
	case F_SIN:
	case F_COS:
		r.lo = fmin(apply(f, lo), apply(f, hi));
		r.hi = fmax(apply(f, lo), apply(f, hi));
		if (holds(lo[0], hi[0], f == F_SIN ? 90 : 0, 360)) {
			r.hi = 1;
		}
		if (holds(lo[0], hi[0], f == F_SIN ? -90 : 180, 360)) {
			r.lo = -1;
		}
		break;
	case F_ABS:
		r = abs_range(a[0]);
		break;
	case F_HYPOT: {
		struct ival x = abs_range(a[0]);
		struct ival y = abs_range(a[1]);

		r.lo = hypot(x.lo, y.lo);
		r.hi = hypot(x.hi, y.hi);
		break;
	}
	case F_ATAN2:
		r = atan2_range(nm, a[0], a[1], r.v);
		break;
	default:
		// The others rise with each argument.
		r.lo = apply(f, lo);
		r.hi = apply(f, hi);
		break;
	}
	return r;
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
	struct qty vals[MAXDEPTH];
	size_t nvals;
	struct op ops[MAXDEPTH];
	size_t nops;
	struct scan *sc;
};

static void push_val(const struct names *nm, struct machine *m, struct qty v)
{
	if (m->nvals == MAXDEPTH) {
		expr_fail(nm, "the expression nests too deep", NULL);
	}
	v.r.lo = fmin(v.r.lo, v.r.v);
	v.r.hi = fmax(v.r.hi, v.r.v);
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

// The product or the quotient of a and b, with the range of the corners.
static struct ival times(const struct names *nm, struct ival a, struct ival b,
			 int divide)
{
	struct ival r = { divide ? a.v / b.v : a.v * b.v, HUGE_VAL, -HUGE_VAL };

	if (divide && b.lo <= 0 && b.hi >= 0 && b.lo < b.hi) {
		expr_fail(nm, "the range of a divisor holds 0", NULL);
	}
	for (int k = 0; k < 4; k++) {
		double x = k & 1 ? a.hi : a.lo;
		double y = k & 2 ? b.hi : b.lo;
		double q = divide ? x / y : x * y;

		r.lo = fmin(r.lo, q);
		r.hi = fmax(r.hi, q);
	}
	return r;
}

// Pops the top operator, a sign or a binary one, and applies it.
static void reduce(const struct names *nm, struct machine *m)
{
	struct op o = m->ops[--m->nops];
	struct qty b = m->vals[--m->nvals];
	struct qty a;
	struct qty q;

	if (o.kind == OP_NEG || o.kind == OP_POS) {
		q = b;
		if (o.kind == OP_NEG) {
			q.r.v = -b.r.v;
			q.r.lo = -b.r.hi;
			q.r.hi = -b.r.lo;
		}
		push_val(nm, m, q);
		return;
	}
	a = m->vals[--m->nvals];
	q = a.len || a.ang ? a : b;
	if (o.kind == OP_ADD) {
		struct ival r = { a.r.v + b.r.v, a.r.lo + b.r.lo,
				  a.r.hi + b.r.hi };

		q.r = r;
	} else if (o.kind == OP_SUB) {
		struct ival r = { a.r.v - b.r.v, a.r.lo - b.r.hi,
				  a.r.hi - b.r.lo };

		q.r = r;
	} else {
		int k = o.kind == OP_DIV ? -1 : 1;

		q.r = times(nm, a.r, b.r, o.kind == OP_DIV);
		q.len = a.len + (k * b.len);
		q.ang = a.ang + (k * b.ang);
	}
	push_val(nm, m, q);
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
	struct qty v = { { 0, 0, 0 }, 0, 0 };

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
		v.r = exact(number(nm, s, start));
		push_val(nm, m, apply_unit(s, v));
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
	if (!lookup(nm, b, &v.r)) {
		expr_fail(nm, "unknown name", start);
	}
	m->sc->named = 1;
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
		struct ival a[2] = { { 0, 0, 0 }, { 0, 0, 0 } };
		struct qty q = { { 0, 0, 0 }, 0, 0 };

		if (o.nargs != fns[o.fn].nargs) {
			expr_fail(nm, "wrong number of function arguments",
				  o.start);
		}
		for (size_t i = o.nargs; i > 0; i--) {
			q = m->vals[--m->nvals];
			a[i - 1] = q.r;
		}
		// The measure of the first argument, but for the sine and
		// the like, of a number, atan2, of an angle, and sqrt, of
		// half the powers.
		q.r = apply_range(nm, o.fn, a);
		if (o.fn == F_TAN || o.fn == F_SIN || o.fn == F_COS ||
		    o.fn == F_ATAN2) {
			q.len = 0;
			q.ang = o.fn == F_ATAN2;
		} else if (o.fn == F_SQRT) {
			q.len /= 2;
			q.ang /= 2;
		}
		push_val(nm, m, q);
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

// Whether no parenthesis nor call is open.
static int top_level(const struct machine *m)
{
	for (size_t i = 0; i < m->nops; i++) {
		if (m->ops[i].kind == OP_PAREN || m->ops[i].kind == OP_CALL) {
			return 0;
		}
	}
	return 1;
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
		if ((o.kind == OP_ADD || o.kind == OP_SUB) && top_level(m)) {
			m->sc->add[0] = m->sc->add[1];
			m->sc->add[1] = *s;
			if (o.kind == OP_ADD && (*s)[1] == '-') {
				m->sc->plus_minus = *s;
			}
		}
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

// Evaluates the expression at *s and leaves *s after it; sc gathers what
// it sees.
static struct qty expr(struct names *nm, const char **s, struct scan *sc)
{
	struct machine m;
	int want_value = 1;

	m.nvals = 0;
	m.nops = 0;
	m.sc = sc;
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

// Dies unless v and its range are finite numbers within 1e6.
static void check_value(const struct names *nm, struct ival v, const char *s)
{
	if (!num_ok(v.v)) {
		expr_fail(nm, "the value is not a finite number within 1e6", s);
	}
	if (!num_ok(v.lo) || !num_ok(v.hi)) {
		expr_fail(nm, "the range of the value is not finite within 1e6",
			  s);
	}
}

// Evaluates s as one expression, and what it sees into sc.
static struct qty eval_scan(struct names *nm, const char *s, struct scan *sc)
{
	const char *p = s;
	struct qty v;

	memset(sc, 0, sizeof(*sc));
	v = expr(nm, &p, sc);
	skip_space(&p);
	if (*p) {
		expr_fail(nm, "junk after the expression", p);
	}
	check_value(nm, v.r, s);
	return v;
}

struct ival eval_range(struct names *nm, const char *s)
{
	struct scan sc;

	return eval_scan(nm, s, &sc).r;
}

double eval(struct names *nm, const char *s)
{
	return eval_range(nm, s).v;
}

// The tolerance of a definition: the deviations above and below its
// value, and whether the text states it.
struct tol {
	double up, down;
	int stated;
};

// Reads the tolerance at the end of the definition buf, if it states
// one, and cuts it off; sc is what the whole text shows.
static struct tol cut_tolerance(struct names *nm, char *buf,
				const struct scan *sc)
{
	struct tol t = { 0, 0, 0 };

	if (sc->plus_minus) {
		size_t at = (size_t)(sc->plus_minus - buf);

		buf[at] = 0;
		t.up = t.down = eval(nm, buf + at + 2);
		t.stated = 1;
		if (!(t.up >= 0)) {
			expr_fail(nm,
				  "a tolerance +- TOL needs TOL of 0 or more",
				  NULL);
		}
	} else if (!sc->named && sc->add[0] && *sc->add[0] == '+' &&
		   *sc->add[1] == '-') {
		size_t a = (size_t)(sc->add[0] - buf);
		size_t b = (size_t)(sc->add[1] - buf);

		buf[a] = 0;
		buf[b] = 0;
		t.up = eval(nm, buf + a + 1);
		t.down = eval(nm, buf + b + 1);
		t.stated = 1;
		if (!(t.up + t.down >= 0)) {
			expr_fail(nm,
				  "a tolerance + UP - DOWN needs UP + DOWN of "
				  "0 or more",
				  NULL);
		}
	}
	return t;
}

void define(struct names *nm, const char *name, const char *text,
	    int from_include)
{
	char *buf = xstrdup(text);
	struct scan sc;
	struct qty q;
	struct ival v;
	struct tol t;
	struct sym s;

	memset(&s, 0, sizeof(s));
	nm->nseen = 0;
	q = eval_scan(nm, buf, &sc);
	t = cut_tolerance(nm, buf, &sc);
	if (t.stated) {
		q = eval_scan(nm, buf, &sc);
		if (sc.named) {
			expr_fail(nm,
				  "a value of names takes its tolerance from "
				  "them",
				  name);
		}
	}
	v = q.r;
	s.base = !sc.named;
	s.kind = s.base ? kind_of(q) : KIND_NUMBER;
	if (s.base && !t.stated && s.kind != KIND_NUMBER) {
		if (!nm->tol_set) {
			expr_fail(nm,
				  s.kind == KIND_LENGTH ?
					  "a length needs a tolerance: NAME = "
					  "EXPR +- TOL, or a tolerance "
					  "statement above it" :
					  "an angle needs a tolerance: NAME = "
					  "EXPR +- TOL, or a tolerance "
					  "statement above it",
				  name);
		}
		t.up = t.down = s.kind == KIND_LENGTH ? nm->tol_len :
							nm->tol_ang;
	}
	if (s.base) {
		v.lo = v.v - t.down;
		v.hi = v.v + t.up;
		check_value(nm, v, text);
	}
	if (nm->vary == nm->nsyms + 1) {
		v = exact(v.v + nm->vary_by);
	}
	free(buf);
	s.name = xstrdup(name);
	s.v = v.v;
	s.lo = v.lo;
	s.hi = v.hi;
	s.from_include = from_include;
	s.where = s.base ? xstrdup(expr_where(nm)) : NULL;
	s.ndeps = nm->reading ? nm->nseen : 0;
	s.deps = xmalloc((s.ndeps + 1) * sizeof(*s.deps));
	for (size_t i = 0; i < s.ndeps; i++) {
		s.deps[i] = nm->seen[i];
	}
	PUSH(nm->syms, nm->nsyms, nm->capsyms, s);
}

// Ends the line of l at its # comment, and its blanks before the end.
static void cut_comment(char *l)
{
	char *hash = strchr(l, '#');
	size_t n = 0;

	if (hash) {
		*hash = 0;
	}
	n = strlen(l);
	while (n && isspace((unsigned char)l[n - 1])) {
		l[--n] = 0;
	}
}

// The lines of a source so far, and the text of the line that runs on,
// if any, from line first.
struct lines {
	struct src_line *ls;
	size_t n, cap;
	char *run;
	unsigned first;
};

// Cuts the backslash off line l, and the blanks before it; returns
// whether there was one, so that the line runs on.
static int cut_run_on(char *l)
{
	size_t k = strlen(l);

	if (!k || l[k - 1] != '\\') {
		return 0;
	}
	l[--k] = 0;
	while (k && isspace((unsigned char)l[k - 1])) {
		l[--k] = 0;
	}
	return 1;
}

// Adds line l, numbered no, to the line that runs on, with a blank for
// the blanks at its start.
static void run_on(struct lines *x, const char *l, unsigned no)
{
	size_t a = 0;
	size_t lead = strspn(l, " \t");
	size_t k = strlen(l);

	if (!x->run) {
		x->run = xstrdup(l);
		x->first = no;
		return;
	}
	a = strlen(x->run);
	x->run = xrealloc(x->run, a + k + 2);
	x->run[a] = ' ';
	memcpy(x->run + a + 1, l + lead, k + 1 - lead);
}

// Ends the line that runs on: keeps it unless it is blank.
static void end_line(struct lines *x)
{
	if (x->run && x->run[strspn(x->run, " \t")]) {
		struct src_line s = { x->first, x->run };

		PUSH(x->ls, x->n, x->cap, s);
	} else {
		free(x->run);
	}
	x->run = NULL;
}

size_t source_lines(char *text, struct src_line **out)
{
	char *rest = text;
	char *l = NULL;
	struct lines x = { NULL, 0, 0, NULL, 0 };
	unsigned no = 0;

	while ((l = next_line(&rest)) != NULL) {
		int on = 0;

		no++;
		cut_comment(l);
		on = cut_run_on(l);
		run_on(&x, l, no);
		if (!on) {
			end_line(&x);
		}
	}
	end_line(&x);
	*out = x.ls;
	return x.n;
}

void free_source_lines(struct src_line *l, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		free(l[i].text);
	}
	free(l);
}

void each_line(struct names *nm, const char *path,
	       void (*fn)(char *line, void *ctx), void *ctx)
{
	char *text = read_file(path);
	const char *old = nm->where;
	char where[WHERE_LEN];
	struct src_line *ls = NULL;
	size_t n = source_lines(text, &ls);

	for (size_t i = 0; i < n; i++) {
		xsnprintf(where, "%s:%u", path, ls[i].no);
		nm->where = where;
		fn(ls[i].text, ctx);
	}
	nm->where = old;
	free_source_lines(ls, n);
	free(text);
}

// Reads a statement tolerance LEN ANGLE at p, the text after its word:
// the tolerance in force of a length and of an angle.
static void tolerances(struct names *nm, const char *p)
{
	struct scan sc;
	const char *at = p;
	struct qty len;
	struct qty ang;

	memset(&sc, 0, sizeof(sc));
	len = expr(nm, &at, &sc);
	ang = expr(nm, &at, &sc);
	skip_space(&at);
	if (*at) {
		expr_fail(nm, "junk after the expression", at);
	}
	check_value(nm, len.r, p);
	check_value(nm, ang.r, p);
	if (!(len.r.v >= 0) || !(ang.r.v >= 0) || kind_of(len) == KIND_ANGLE ||
	    kind_of(ang) == KIND_LENGTH) {
		expr_fail(nm,
			  "tolerance LEN ANGLE: a length and an angle, 0 or "
			  "more",
			  NULL);
	}
	nm->tol_set = 1;
	nm->tol_len = len.r.v;
	nm->tol_ang = ang.r.v;
}

int tolerance_line(struct names *nm, const char *l)
{
	static const char tw[] = "tolerance";
	size_t a = strspn(l, " \t");

	if (strncmp(l + a, tw, strlen(tw)) != 0 ||
	    (l[a + strlen(tw)] != ' ' && l[a + strlen(tw)] != '\t')) {
		return 0;
	}
	tolerances(nm, l + a + strlen(tw));
	return 1;
}

// Reads one "name = expr" line into the names ctx, or a statement
// tolerance LEN ANGLE; the name is cut off in place.  The type of the
// each_line callback fixes the char *.
static void param_line(char *l, void *ctx)
{
	struct names *nm = ctx;
	size_t a = strspn(l, " \t");
	size_t e = a;
	size_t q = 0;

	if (!l[a]) {
		return;
	}
	if (tolerance_line(nm, l)) {
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
	define(nm, l + a, l + q + 1, 1);
}

void read_params(struct names *nm, const char *path)
{
	bool was = nm->reading;

	nm->reading = true;
	each_line(nm, path, param_line, nm);
	nm->reading = was;
}
