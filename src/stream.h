// SPDX-License-Identifier: GPL-3.0-only
// stream.h -- The text that runs down a pipe of tools: solids and source
// Copyright (c) 2026 Jakob Kastelic

// A part is one source file.  Its first tool reads the file; each tool
// writes a stream on stdout and the next one reads it.  A stream is
//   PliCad 1
//   source PATH
//   brep NAME ... end            each solid so far (shape.h)
//   @N TEXT                      each source line not yet consumed
// where N is the number of the line in PATH.  Each tool takes the lines
// of its own statements out and passes the others on.  The lines
// "NAME = EXPR" stay: every tool evaluates them, in order, so that each
// statement sees the names defined above it.  The last tool of a pipe
// refuses any line that is left.

#ifndef STREAM_H
#define STREAM_H

#include "expr.h"
#include "shape.h"
#include "util.h"
#include <stddef.h>

enum { ARGS_MAX = 64 }; // The most words of a statement.

// A line of the source: its number, its text without the comment, and
// whether a tool took it out.
struct sline {
	unsigned no;
	char *text;
	int gone;
};

struct stream {
	char *src;
	struct shape *sh;
	size_t nsh, capsh;
	struct sline *ln;
	size_t nln, capln;
	struct names nm;
	char where[WHERE_LEN];
	// The words of the line under work.
	char *w[ARGS_MAX + 1];
	size_t nw;
	char *buf;
};

// Whether the file at path, "-" for stdin, holds a stream.
int stream_file(const char *path);
// Reads path, "-" for stdin: a stream, or else a source file.
void stream_read(struct stream *st, const char *path);
void stream_write(const struct stream *st);
void stream_free(struct stream *st);

// Makes line i the line under work: sets where, splits its words into
// st->w, and evaluates it if it defines names.  Returns 1 if it holds a
// statement, 0 for a definition or an empty line.
int stream_line(struct stream *st, size_t i);
// The value of word i of the line under work.
double stream_num(struct stream *st, size_t i);
// Whether word i of the line under work is w.
int stream_is(const struct stream *st, size_t i, const char *w);
// Dies with msg at the line under work.
#define stream_fail(st, msg, near) expr_fail(&(st)->nm, (msg), (near))

// Adds a solid, which must have a new name; the stream owns it then.
void stream_add(struct stream *st, struct shape *sh);
// Takes the solid of that name out, into *out, or dies.
void stream_take(struct stream *st, const char *name, struct shape *out);
// Dies unless every line is gone but the definitions.
void stream_refuse_left(struct stream *st);

#endif
