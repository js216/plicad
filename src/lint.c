// SPDX-License-Identifier: GPL-3.0-only
// lint.c -- Check the text of source files against the house rules
// Copyright (c) 2026 Jakob Kastelic

// Usage: lint FILE...
//
// Prints FILE:LINE: PROBLEM for each problem, and exits with 1 if there
// are any.  One job: the mechanical rules for the text of a source file,
// with the license header among them.
//
// Every file:
//   ASCII only, no carriage return, no trailing blank, a final newline.
// Files with comments (C, scripts, part and assembly files, params), but
// the Makefile and the dot files:
//   The header: three comment lines and one blank line,
//     M SPDX-License-Identifier: GPL-3.0-only
//     "M NAME -- DESCRIPTION"
//     M Copyright (c) 2026 Jakob Kastelic
//   where M is // in C and # elsewhere; after a #! line, on lines 2 to 4.
// Files without a one-line comment syntax get no header: Markdown (a #
// starts a heading there) and data files (.expected, .fold, .step,
// .json).  Nor do the Makefile and the dot files, by the rule of the
// user, though their # comments follow the rules of prose.
// All but C (clang-format sets its width):
//   At most 80 columns, a tab to the next multiple of 8.  A Markdown link
//   definition or table row is exempt, and a URL does not count.
// C:
//   No block comment, no typedef of a struct, no #if, #ifdef or #ifndef
//   inside a function body.
// Comments and Markdown prose, outside code:
//   No spaced two dashes (the header line excepted), and no spaced
//   hyphen between two words of three letters or more: a dash is ---
//   or --, unspaced.  A comment or line that starts with a hyphen is a
//   list or an option, and is exempt, as is text in double quotes.  The
//   name in lower case stands only in a path, a file name or a command
//   (a word with "/", "." or "="); prose says PliCad.

#include "util.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The opening of a block comment and the start of a typedef of a
// struct, spelled out so this file has neither.
static const char block_start[] = { '/', '*', 0 };
static const char typedef_struct[] = { 't', 'y', 'p', 'e', 'd', 'e', 'f', ' ',
				       's', 't', 'r', 'u', 'c', 't', 0 };

// The kinds of file: C, text with # comments and the header, Markdown,
// data, and the Makefile and the dot files, # comments without a header.
enum kind { K_C, K_HASH, K_MD, K_DATA, K_BARE };

enum {
	COLUMNS = 80, // The widest line of a text file.
	TAB = 8, // Columns from one tab stop to the next.
	HEAD_LINES = 5, // The header: SPDX, name, copyright, blank, text.
	LINE_LEN = 512, // Room for one line of the header.
	CODE_LEN = 4096, // Room for one line of C without its strings.
	ASCII_MAX = 0x7f // The highest ASCII byte.
};

// The state of one file under check.
struct file {
	const char *path;
	const char *name;
	enum kind kind;
	unsigned shift; // 1 after a #! line.
	int depth; // C brace depth.
	int fence; // Inside a Markdown code block.
	int problems; // Found so far.
};

static void bad(struct file *f, unsigned line, const char *what)
{
	printf("%s:%u: %s\n", f->path, line, what);
	f->problems++;
}

static int ends_with(const char *s, const char *tail)
{
	size_t n = strlen(s);
	size_t m = strlen(tail);

	return n >= m && strcmp(s + n - m, tail) == 0;
}

static enum kind kind_of(const char *name)
{
	static const char *const data[] = { ".expected", ".fold", ".step",
					    ".json" };

	if (ends_with(name, ".c") || ends_with(name, ".h")) {
		return K_C;
	}
	if (strcmp(name, "Makefile") == 0 || name[0] == '.') {
		return K_BARE;
	}
	if (ends_with(name, ".md")) {
		return K_MD;
	}
	for (size_t i = 0; i < sizeof(data) / sizeof(*data); i++) {
		if (ends_with(name, data[i])) {
			return K_DATA;
		}
	}
	return K_HASH;
}

// A header line l, what it should be, and the problem if it is not.
struct header_line {
	const char *l, *want, *what;
};

// Checks one of the first three header lines: the whole line, or for
// line 2 its start.
static void check_header_line(struct file *f, unsigned no, struct header_line h)
{
	size_t n = strlen(h.want);

	if (no == 2 ? strncmp(h.l, h.want, n) != 0 || strlen(h.l) <= n :
		      strcmp(h.l, h.want) != 0) {
		bad(f, no, h.what);
	}
}

// Checks line no, 1 to 3, of the header, l: the comment lines.
static void check_comment_line(struct file *f, unsigned no, const char *l)
{
	char want[LINE_LEN];
	const char *mark = f->kind == K_C ? "//" : "#";
	struct header_line h = { l, want, NULL };

	if (no == 1) {
		xsnprintf(want, "%s SPDX-License-Identifier: GPL-3.0-only",
			  mark);
		h.what = "the SPDX line is missing";
	} else if (no == 2) {
		xsnprintf(want, "%s %s -- ", mark, f->name);
		h.what = "need: NAME -- DESCRIPTION";
	} else {
		xsnprintf(want, "%s Copyright (c) 2026 Jakob Kastelic", mark);
		h.what = "the copyright line is missing";
	}
	check_header_line(f, no, h);
}

// Checks line no of the header, l.
static void check_header(struct file *f, unsigned no, const char *l)
{
	if (no >= 1 && no <= 3) {
		check_comment_line(f, no, l);
	} else if (no == 4 && *l) {
		bad(f, no, "a blank line must follow the header");
	} else if (no == HEAD_LINES && !*l) {
		bad(f, no, "one blank line after the header");
	}
}

// The width of line l, a tab to the next tab stop.
static size_t width(const char *l)
{
	size_t w = 0;

	for (; *l; l++) {
		w = *l == '\t' ? (w / TAB + 1) * TAB : w + 1;
	}
	return w;
}

// Is l a Markdown link definition or a table row, or would it fit in 80
// columns without its URL, which cannot break?
static int long_ok(const char *l)
{
	const char *u = strstr(l, "http");

	if (u && (strncmp(u, "http://", strlen("http://")) == 0 ||
		  strncmp(u, "https://", strlen("https://")) == 0)) {
		size_t n = 0;

		while (u[n] && u[n] != ' ' && u[n] != ')' && u[n] != '>') {
			n++;
		}
		if (width(l) - n <= COLUMNS) {
			return 1;
		}
	}
	while (*l == ' ' || *l == '\t') {
		l++;
	}
	if (*l == '|') {
		return 1;
	}
	if (*l == '[' && strstr(l, "]:")) {
		return 1;
	}
	return 0;
}

static void check_bytes(struct file *f, unsigned no, const char *l)
{
	size_t n = strlen(l);

	for (const char *p = l; *p; p++) {
		if ((unsigned char)*p > ASCII_MAX) {
			bad(f, no, "a byte outside ASCII");
			break;
		}
	}
	if (n && (l[n - 1] == ' ' || l[n - 1] == '\t')) {
		bad(f, no, "trailing blank");
	}
}

// The length of the word ending just before p (alphabetic only), or of
// the one starting at p.
static size_t word_before(const char *start, const char *p)
{
	size_t n = 0;

	while (p > start && isalpha((unsigned char)p[-1])) {
		p--;
		n++;
	}
	return n;
}

static size_t word_after(const char *p)
{
	size_t n = 0;

	while (isalpha((unsigned char)p[n])) {
		n++;
	}
	return n;
}

// Is the word round s[i] a path, a file name or a command word?
static int technical(const char *s, size_t i)
{
	const char *start = s;
	const char *a = s + i;
	const char *b = s + i;

	while (a > start && !isspace((unsigned char)a[-1])) {
		a--;
	}
	while (*b && !isspace((unsigned char)*b)) {
		b++;
	}
	for (const char *q = a; q < b; q++) {
		if (*q == '/' || *q == '=' || (*q == '.' && q + 1 < b)) {
			return 1;
		}
	}
	return 0;
}

// Checks the prose t (a comment or a Markdown line without code).
static void check_prose(struct file *f, unsigned no, const char *t)
{
	const char *s = t;

	while (*s == ' ' || *s == '\t') {
		s++;
	}
	if (*s == '-' || *s == '*') {
		return;
	}
	for (const char *p = s; *p; p++) {
		if (*p == '"') {
			const char *q = strchr(p + 1, '"');

			if (!q) {
				break;
			}
			p = q;
			continue;
		}
		if (strncmp(p, " -- ", 4) == 0) {
			bad(f, no, "a spaced -- (a dash is --- or --)");
		}
		if (strncmp(p, " - ", 3) == 0 && word_before(s, p) >= 3 &&
		    word_after(p + 3) >= 3) {
			bad(f, no, "a spaced - used as a dash");
		}
		if (strncmp(p, "plicad", strlen("plicad")) == 0 &&
		    !technical(s, (size_t)(p - s))) {
			bad(f, no, "write PliCad in prose");
		}
	}
}

// The comment of a C line, or NULL: the text after a // outside a
// string or a character constant.
static const char *c_comment(const char *l)
{
	char quote = 0;

	for (const char *p = l; *p; p++) {
		if (quote) {
			if (*p == '\\' && p[1]) {
				p++;
			} else if (*p == quote) {
				quote = 0;
			}
		} else if (*p == '"' || *p == '\'') {
			quote = *p;
		} else if (p[0] == '/' && p[1] == '/') {
			return p + 2;
		}
	}
	return NULL;
}

// The comment of a # line, or NULL: a # at the start or after a blank,
// outside quotes.
static const char *hash_comment(const char *l)
{
	char quote = 0;

	for (const char *p = l; *p; p++) {
		if (quote) {
			if (*p == quote) {
				quote = 0;
			}
		} else if (*p == '"' || *p == '\'') {
			quote = *p;
		} else if (*p == '#' &&
			   (p == l || p[-1] == ' ' || p[-1] == '\t')) {
			return p + 1;
		}
	}
	return NULL;
}

// Updates the brace depth of C code over line l, outside strings and
// comments.
static int brace_depth(const char *l, int depth)
{
	const char *end = c_comment(l);
	char quote = 0;

	for (const char *p = l; *p && (!end || p < end - 2); p++) {
		if (quote) {
			if (*p == '\\' && p[1]) {
				p++;
			} else if (*p == quote) {
				quote = 0;
			}
		} else if (*p == '"' || *p == '\'') {
			quote = *p;
		} else if (*p == '{') {
			depth++;
		} else if (*p == '}') {
			depth--;
		}
	}
	return depth;
}

static void check_c(struct file *f, unsigned no, const char *l)
{
	const char *s = l;

	if (strstr(l, block_start)) {
		bad(f, no, "a block comment");
	}
	if (strstr(l, typedef_struct)) {
		bad(f, no, "a typedef of a struct");
	}
	while (*s == ' ' || *s == '\t') {
		s++;
	}
	if (f->depth > 0 && *s == '#' &&
	    (strncmp(s, "#if", 3) == 0 || strncmp(s, "# if", 4) == 0)) {
		bad(f, no, "a conditional inside a function body");
	}
}

// Removes `code` spans from Markdown line l into buf.
static void strip_code(const char *l, char *buf, size_t n)
{
	size_t k = 0;
	int in = 0;

	for (; *l && k + 1 < n; l++) {
		if (*l == '`') {
			in = !in;
		} else if (!in) {
			buf[k++] = *l;
		}
	}
	buf[k] = 0;
}

static void check_line(struct file *f, unsigned no, const char *l)
{
	char buf[CODE_LEN];
	const char *c = NULL;

	check_bytes(f, no, l);
	if (f->kind == K_C || f->kind == K_HASH) {
		if (no > f->shift) {
			check_header(f, no - f->shift, l);
		}
	}
	if (f->kind != K_C && f->kind != K_DATA && width(l) > COLUMNS &&
	    !long_ok(l)) {
		bad(f, no, "longer than 80 columns");
	}
	if (f->kind == K_C) {
		check_c(f, no, l);
		f->depth = brace_depth(l, f->depth);
		c = c_comment(l);
	} else if (f->kind == K_HASH || f->kind == K_BARE) {
		c = hash_comment(l);
	} else if (f->kind == K_MD) {
		if (strncmp(l, "```", 3) == 0) {
			f->fence = !f->fence;
		} else if (!f->fence) {
			strip_code(l, buf, sizeof(buf));
			c = buf;
		}
	}
	if (c && (f->kind == K_BARE || no != f->shift + 2)) {
		check_prose(f, no, c);
	}
}

// Checks the file at path; returns the number of problems.
static int check(const char *path)
{
	const char *slash = strrchr(path, '/');
	struct file f = { path, slash ? slash + 1 : path, K_DATA, 0, 0, 0, 0 };
	char *text = read_file(path);
	char *rest = text;
	const char *l = NULL;
	unsigned no = 0;
	size_t n = strlen(text);

	f.kind = kind_of(f.name);
	f.shift = strncmp(text, "#!", 2) == 0;
	if (n && text[n - 1] != '\n') {
		bad(&f, 0, "no newline at the end");
	}
	for (size_t i = 0, line = 1; i < n; i++) {
		if (text[i] == '\r') {
			bad(&f, (unsigned)line, "a carriage return");
			break;
		}
		line += text[i] == '\n';
	}
	while ((l = next_line(&rest)) != NULL) {
		check_line(&f, ++no, l);
	}
	if ((f.kind == K_C || f.kind == K_HASH) && no < HEAD_LINES + f.shift) {
		bad(&f, no, "the file is shorter than the header");
	}
	free(text);
	return f.problems;
}

int main(int argc, char **argv)
{
	int problems = 0;

	if (argc < 2) {
		die("usage: lint FILE...");
	}
	for (int i = 1; i < argc; i++) {
		problems += check(argv[i]);
	}
	close_stdout();
	return problems ? EXIT_FAILURE : EXIT_SUCCESS;
}
