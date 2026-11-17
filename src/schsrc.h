// SPDX-License-Identifier: GPL-3.0-only
// schsrc.h -- The schematic source and its symbols, read for schdraw
// Copyright (c) 2026 Jakob Kastelic

// The reader of schdraw and schnet: see sch(5) and schsym(5).

#ifndef SCHSRC_H
#define SCHSRC_H

#include <stddef.h>
#include <stdlib.h>

enum { SCH_R, SCH_U, SCH_L, SCH_D }; // The four ways a pin can point.

#define SCH_GRID 7.2 // Points to a unit of the grid at scale 1.
// The text is in the stroke font (stroke.h), its size the capital
// height.
#define SCH_TEXT 4.5 // The size of the text of the parts.
#define SCH_LEAD 7.6 // From one line of a label to the next.
#define SCH_DESCENT 1.8 // Under the baseline of the last line of a label.
#define SCH_LW 0.5 // The weight of every line but the bold ones.

// A pin of a shape: its name, where its end lies in grid units from the
// origin of the shape, and the way a wire leaves it.
struct sch_pin {
	char *name;
	int x, y, dir;
};

// A record of the drawing of a shape, in points from its origin: a line
// (open, or closed as a loop) of width w, a polygon filled dark or blank
// (the paper's color), a circle or a disk (center and radius); drawn
// always (when 0), only flipped (1) or only not (2).
enum { SCH_LINE, SCH_LOOP, SCH_FILL, SCH_BLANK, SCH_CIRCLE, SCH_DISK };
struct sch_draw {
	int kind, when;
	double w, *xy;
	size_t n;
};

// A shape: a symbol drawn by the records of its file, with its pins, the box its body takes (grid units), the letters of its
// designators and its class.  A flag (ground, supply, port, chassis) is a
// shape that names a net instead of being a part; swap lists the pins
// that may trade places (as a string of names, blank-separated); flip
// gives the two pins that trade places when the part is flipped, or -1.
struct sch_shape {
	char *name, *prefix, *flag, *swap;
	struct sch_pin *pin;
	size_t npin;
	struct sch_draw *draw;
	size_t ndraw;
	double box[4];
	int side; // The side for the text when the shape stands as drawn.
	int flip[2];
};

// A device: a shape with a table of pin numbers, or a generic shape whose
// pin names are the numbers.
struct sch_device {
	char *name, *prefix;
	struct sch_shape *shape;
	char **pnum; // The pin number of each pin of the shape.
	char *nc; // Pins of the package on no pin of the shape.
};

// A placed symbol: a part, or a flag.  Its place is computed from the
// statements of the source; its transform turns the shape by rot
// quarter turns counterclockwise after mirroring it in x, and a flipped
// part has the pins of its shape's flip pair traded.  The name is the
// path of its page and groups and its own name, as outputs.ch1.amp;
// placed tells whether it found a place.  A part without its value
// (novalue) is drawn with its designator alone: the most common ferrite
// of the page, whose value goes without saying.
struct sch_part {
	char *name, *ref, *value, *rating, *mpn;
	struct sch_device *dev;
	int x, y, rot, mirror, flip, side, line, novalue, placed;
	char *text; // The text of a flag: the name of its net.
	// Of each pin, whether an nc statement leaves it unconnected on
	// purpose; NULL for none.
	unsigned char *nc;
};

// A pin of a net: a part and the index of its pin, and the flag it has,
// if any.  The pins of one group of a net are wired together; the
// groups of a net join through their flags of one name.
struct sch_end {
	size_t part, pin;
	struct sch_shape *flag;
	int group;
};

// A net: its name and its pins.  A global net, one named by a flag or by
// a port of the sheet, is the same net wherever its name is used.
struct sch_net {
	char *name;
	struct sch_end *end;
	size_t nend;
	int global, line;
};

// A wire said in the source between two pins, with the side its path
// keeps to (SCH_R ... SCH_D), or -1.
struct sch_hint {
	size_t part[2], pin[2];
	int side;
};

// A page of the file: its number (its place in the file) and name, the line of its page
// statement, and its sheet ports (the ports outside any group) by name,
// each with the line it is first said on.
struct sch_page {
	int num, line;
	char *name, **port;
	int *port_line;
	size_t nport;
};

// A page of the source, read: its number and name, and the pages of the
// whole file.
struct sch {
	const char *file;
	int page;
	char *name;
	struct sch_page *pages;
	size_t npages;
	char *field[8][2]; // Name and text of the fields of the title block.
	size_t nfield;
	char *sheet;
	double scale;
	struct sch_part *part;
	size_t npart;
	struct sch_net *net;
	size_t nnet;
	struct sch_hint *hint;
	size_t nhint;
	char **lib;
	size_t nlib;
	// The room the drawing keeps, in points at scale 1, from the clear
	// records of the sheet template: between a wire and a body of
	// another net, or a body and a body (body); between text and
	// anything else (text); from a body to its own label (label); and
	// round a flag, for the labels (flag).  Wires of two nets keep a
	// pitch of the grid apart, as the grid gives them.  The placer, the
	// router and the label placer keep it, and schdraw measures it.
	double clear_body, clear_text, clear_label, clear_flag;
};

void *sch_alloc(size_t n);
char *sch_strdup(const char *s);
void sch_warn(const char *fmt, ...);
// Whether to tell each placement on stderr, with its line and rule.
extern int sch_verbose;
// Prints a message as sch_warn does and exits.  As a macro, the exit is
// visible in every caller and to the analyzer.
#define sch_die(...) (sch_warn(__VA_ARGS__), exit(EXIT_FAILURE))
// Reads page k (0 the first) of the source and the symbols it uses from
// the directories of lib, and places the parts and the flags; the pages
// of the file come with it.  Each fault of the page is told on stderr,
// by its line, and the reading goes on without what it touches (a part
// that cannot be placed is not placed); the faults of the file as a
// whole (a page name twice, a sheet port on one page alone) are told
// with page 0.  Returns the number of faults.
int sch_read(struct sch *s, const char *file, size_t k, char **lib,
	     size_t nlib);
// Where the pin p of part q stands, in grid units, and the way it points.
void sch_pin_at(const struct sch_part *q, size_t p, int *x, int *y, int *dir);
// The box of part q in grid units, turned as it is.
void sch_box(const struct sch_part *q, double b[4]);
// The pin number of pin p of the device.
const char *sch_pnum(const struct sch_device *d, size_t p);
// The width in points of a text in the stroke font of the size.
double sch_text_width(const char *s, double size);
// The size in points of the label of a part: its designator over its
// value and rating.  The designator is taken as its prefix and three
// digits until it is known.
void sch_label_size(const struct sch_part *q, double *w, double *h);
// The symbols of the library next to the program: ../lib/symbols from
// the directory of argv0.
char *sch_default_lib(const char *argv0);
// Finds the file name of the library, or NULL.
char *sch_find(char **lib, size_t nlib, const char *name);
// The file of the sheet template of the source, from the sheets next to
// each directory of symbols (LIB/symbols/../sheets), or NULL.
char *sch_sheet_path(const struct sch *s);

#endif
