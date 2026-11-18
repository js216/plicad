// SPDX-License-Identifier: GPL-3.0-only
// util.h -- Errors, memory, arrays, words and whole-file reading
// Copyright (c) 2026 Jakob Kastelic

#ifndef UTIL_H
#define UTIL_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

enum { WHERE_LEN = 4200 }; // Room for a path and a line number.

// NONE is the index of no face, or of no parent.
#define NONE SIZE_MAX

// A string of an array of strings.
struct str {
	char *s;
};

// Appends x to the array v of n items with room for cap.
#define PUSH(v, n, cap, x)                                         \
	do {                                                       \
		if ((n) == (cap)) {                                \
			(cap) = (cap) ? 2 * (cap) : 16;            \
			(v) = xrealloc((v), (cap) * sizeof(*(v))); \
		}                                                  \
		(v)[(n)++] = (x);                                  \
	} while (0)

// Prints a message and a newline to stderr and goes on.  The format is
// the literal at the call site, so the compiler checks it.  A failed
// write to stderr goes unchecked: the message has nowhere else to go.
#define warn(...)                                                  \
	((void)fflush(stdout), (void)fprintf(stderr, __VA_ARGS__), \
	 (void)fputc('\n', stderr))

// Prints a message as warn does and exits.  As a macro, the exit is
// visible in every caller and to the analyzer.
#define die(...) (warn(__VA_ARGS__), exit(EXIT_FAILURE))

// Formats into the array buf, or dies if the text does not fit.
#define xsnprintf(buf, ...)                                            \
	do {                                                           \
		int xsn_k = snprintf((buf), sizeof(buf), __VA_ARGS__); \
		if (xsn_k < 0 || (size_t)xsn_k >= sizeof(buf))         \
			die("a text is too long for its buffer");      \
	} while (0)

// Dies if a write to stdout failed.  Every program that writes stdout
// calls it before it returns.
void close_stdout(void);

void *xmalloc(size_t n);
void *xcalloc(size_t n, size_t size);
void *xrealloc(void *p, size_t n);
char *xstrdup(const char *s);

// Returns the file, or stdin for "-", NUL terminated.
char *read_file(const char *path);

// Splits s in place at newlines and returns the next line without its
// line end, or NULL.  A carriage return before the newline goes too.
char *next_line(char **s);

// Splits the first word off *s in place and moves *s past it; returns
// the word, or NULL if *s holds no more words.
char *split_first(char **s);
// Splits s in place into at most max words and returns their number, or
// max + 1 if there are more.  A word ends at a blank outside parentheses.
size_t split_words(char *s, char **words, size_t max);

// Returns the file name without directories and extension.
char *base_name(const char *path);

// The file name next to the file of: of with its last component
// replaced by name, unless name is absolute.
struct sibling {
	const char *of, *name;
};
char *sibling_path(struct sibling s);
// The name a/b, a new string.
char *join_names(const char *a, const char *b);
// The file of s, as sibling_path gives it, if there is one; else the
// file of that name in the directory dir of the library, plicad-lib(7):
// lib beside the directory of the running program, as build/ has
// beside it.  A new string.
char *lib_path(struct sibling s, const char *dir);
// The file of the name in the directory dir of the library, as lib_path
// has it.  A new string.
char *lib_file(const char *dir, const char *name);

// Sorts v[0 .. n - 1] up.
void sort_doubles(double *v, size_t n);

// The largest magnitude of any number in the input, in mm for a length:
// a kilometer, far past any part, and far from where the arithmetic of
// the programs loses its precision.
#define NUM_MAX 1e6
// Whether v is a finite number no larger than NUM_MAX either way.
int num_ok(double v);

// Reads *in by fmt as sscanf does, for the conversions %lf, %d and %Ns
// (each of them suppressed with a *, as %*f) and %n, and the blanks and
// other characters between them: each number by strtod or strtol, its
// end checked.  Returns the number of conversions assigned; *in is set
// past what it read.
int scan(const char **in, const char *fmt, ...);

#endif
