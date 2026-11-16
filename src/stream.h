// SPDX-License-Identifier: GPL-3.0-only
// stream.h -- The text that runs down a pipe of tools: solids and source
// Copyright (c) 2026 Jakob Kastelic

// A stream holds parts (fold.h): the blanks that smfold makes, the
// solids of a pipe of tools, or the parts of an assembly in place.  One
// record a line:
//   PliCad
//   source PATH                  the source of a pipe
//   material NAME R G B ROUGHNESS DENSITY
//   part NAME ... end            each part
//   @N TEXT                      each source line not yet used
// where N is the number of the line in PATH.  A pipe begins with a
// source file: its first tool reads the file; each tool writes a stream
// on stdout and the next one reads it.  Each tool takes the lines of its
// own statements out and passes the others on.  The lines "NAME = EXPR"
// and "include FILE" (a parameter file next to the source) stay: every
// tool evaluates them, in order, so that each statement sees the names
// defined above it.  The last tool of a pipe refuses any line that is
// left.  The parts of a pipe are bare solids: no place, material or
// coat.

#ifndef STREAM_H
#define STREAM_H

#include "expr.h"
#include "fold.h"
#include "util.h"
#include <stddef.h>

struct shape; // An exact solid (shape.h).

enum { ARGS_MAX = 64 }; // The most words of a statement.

// A line of the source: its number, its text without the comment, and
// whether a tool took it out.
struct sline {
	unsigned no;
	char *text;
	int gone;
};

// A set of faces of a statement "faces NAME REF...": its name and its
// references, each a face reference (stream_face).
struct face_set {
	char *name;
	struct str *refs;
	size_t n;
};

struct stream {
	char *src;
	struct face_set *sets;
	size_t nsets, capsets;
	struct face_finish *fins; // The finishes of the faces of the solids.
	size_t nfins, capfins;
	struct face_thread *threads; // The threads of their holes.
	size_t nthreads, capthreads;
	struct part *ps; // The parts, as read.
	size_t nps, capps;
	struct material *mats;
	size_t nmats, capmats;
	struct shape *sh; // The solids of a pipe.
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

// Reads path, "-" for stdin: a stream of solids, or else a source
// file, for a tool of a pipe.
void stream_read(struct stream *st, const char *path);
void stream_write(const struct stream *st);
// Reads the parts of the stream at path, "-" for stdin, each with its
// materials: that of its base, which the stream must give, and those of
// its coats that it gives.  Dies on a malformed stream, or one with
// source lines left.
size_t read_parts(const char *path, struct part **out);
// Whether the file at path holds a stream, whose first statement is
// PliCad, and not some other source; a file that cannot be read counts as
// a stream, for read_parts to say so.
int stream_file(const char *path);
// Writes the first line of a stream.
void stream_head(void);
void stream_free(struct stream *st);

// Makes line i the line under work: sets where, splits its words into
// st->w, and evaluates it if it defines names or a set of faces.
// Returns 1 if it holds a statement, 0 for a definition or an empty
// line.
int stream_line(struct stream *st, size_t i);
// The value of word i of the line under work.
double stream_num(struct stream *st, size_t i);
// Whether word i of the line under work is w.
int stream_is(const struct stream *st, size_t i, const char *w);
// Dies with msg at the line under work.
#define stream_fail(st, msg, near) expr_fail(&(st)->nm, (msg), (near))

// A reference to faces in stream st.
struct face_ref {
	const struct stream *st;
	const char *ref;
};

// Whether a face of the name face matches the reference r.ref: any
// (every face), the name of a set of faces of r.st, NAME (every face of
// the solid that extrude made as NAME), NAME.start, NAME.end, NAME.side
// (every side) or NAME.SIDE (the faces of the corner named SIDE).  A
// face of a round, named after the faces it joins, matches what one of
// them matches.
int stream_face(struct face_ref r, const char *face);
// The material of the finish of the face of the name face, of the n
// finishes f: that of the last whose references match it, or NULL.
const char *finish_of(const struct face_finish *f, size_t n, const char *face);
// The thread of the face of the name face, of the n threads t: the last
// whose reference matches it, or NULL.
const struct face_thread *thread_of(const struct face_thread *t, size_t n,
				    const char *face);
// The material of the name in st, or NULL.
// The material of the name in the materials of the library, for a file
// read from path; dies if it has none.
struct material lib_material(const char *path, const char *name);
const struct material *stream_material(const struct stream *st,
				       const char *name);

// Adds a solid, which must have a new name; the stream owns it then.
void stream_add(struct stream *st, struct shape *sh);
// Takes the solid of that name out, into *out, or dies.
void stream_take(struct stream *st, const char *name, struct shape *out);
// Dies unless every line is gone but the definitions.
void stream_refuse_left(struct stream *st);

#endif
