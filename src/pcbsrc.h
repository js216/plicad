// SPDX-License-Identifier: GPL-3.0-only
// pcbsrc.h -- The board source, its netlist, footprints and lock
// Copyright (c) 2026 Jakob Kastelic

// The reader of pcbplace, pcbroute, pcbcheck and pcbgerber: see pcb(5)
// and pcbfp(5).  Lengths are in mm, y up; angles in degrees.

#ifndef PCBSRC_H
#define PCBSRC_H

#include <stdio.h>
#include <stdlib.h>

enum { PAD_RECT, PAD_RRECT, PAD_CIRCLE }; // The shapes of pads.
enum { LAYER_SIGNAL, LAYER_PLANE }; // The kinds of copper layers.

// A pad of a footprint: its name (the pin number), its shape and its
// box from the origin of the footprint, as seen from the top; a hole
// pad is drilled and plated, on every copper layer.  A pad without paste
// (nopaste) is a contact pad, never soldered.  A hole a screw goes
// through has a keep-out, a circle of diameter keep round it (0 for
// none), which keeps the parts of both sides off, and the vias.
struct fp_pad {
	char *name;
	int shape, hole, nopaste;
	double x, y, w, h, drill, keep;
};

// A line of the silkscreen of a footprint: its width and points.
struct fp_line {
	double w, *xy;
	size_t n;
};

struct fp {
	char *name;
	struct fp_pad *pad;
	size_t npad;
	double court[4]; // The courtyard, which keeps other parts off.
	struct fp_line *silk;
	size_t nsilk;
};

// A part: its name from the schematic (ch1.gain.rf), its designator,
// values and footprint, where it stands (rot a multiple of 90), and the
// statement that placed it, as its words, once known.
struct pcb_part {
	char *name, *ref, *type, *value, *mpn, *pins;
	struct fp *fp;
	double x, y;
	int rot, bottom, placed, line;
	int unplaced; // Of a group an unplaced statement names: no footprint.
	char *by;
};

// A pad of a part on a net: the part and the index of its pad.
struct pcb_end {
	size_t part, pad;
};

// A net: its name, the plane layer it is on (-1 for none), the width of
// its tracks and its pads; global, and its class (NULL for none), as the
// netlist says.
struct pcb_net {
	char *name, *klass;
	int global, plane;
	int waits; // It has pins on parts unplaced, and so is not whole.
	double width;
	struct pcb_end *end;
	size_t nend;
};

struct pcb_layer {
	char *name, *net;
	int kind;
};

// A connection of a net: the copper from one pad to another (a and b,
// each PART.PAD), or from a pad to its via (b "via"), of width w.  The
// lock keeps the copper by connection, so an edit makes anew only the
// connections it touches.
struct pcb_link {
	int net;
	char *a, *b;
	double w;
};

// The copper the router makes, each of a connection (link, -1 none).
struct pcb_track {
	int net, layer, link;
	double w, *xy;
	size_t n;
};

struct pcb_via {
	int net, link;
	double x, y, dia, drill;
};

// A statement of the source: its words and line.
struct pcb_stmt {
	char **w;
	int n, line;
};

// A block copied from another (block GROUP as GROUP): the copy, its
// source and the line; whether the two are of the same parts and nets
// (ok), and then the net of the copy for each net of the source (-1 for
// none); and the statements of the copy placing what one of its
// source's places, each replaced by the source's (by index), and the
// lines they were on.
struct pcb_copy {
	char *group, *src;
	int line, ok;
	long *net;
	size_t *dup, ndup;
	int *dup_line;
};

struct pcb {
	const char *file;
	double board[4]; // The outline, a rectangle.
	// The cutouts, rectangles through the board, which are edge like
	// the outline, and the radii of their corners.
	double cut[8][4], cutr[8];
	size_t ncut;
	double clearance, track, via_dia, via_drill, edge;
	// The classes of class CLASS width W: their names and widths.
	char **klass;
	double *class_width;
	size_t nclass;
	// From a pad's edge to the ring of a via of its net (the via joined
	// to it by a neck track), and the least web of solder mask between
	// two openings, or between an opening and a via, which is tented.
	double viagap, maskweb;
	char **edge_except;
	size_t nexcept;
	struct pcb_layer *layer;
	size_t nlayer;
	struct pcb_part *part;
	size_t npart;
	struct pcb_net *net;
	size_t nnet;
	struct pcb_track *track_;
	size_t ntrack;
	struct pcb_via *via;
	size_t nvia;
	struct pcb_link *link;
	size_t nlink;
	struct pcb_stmt *stmt;
	size_t nstmt;
	// The copies: the statements of each source's parts and nets, renamed,
	// are among the statements, so each program reads a copy's parts as
	// placed and routed like its source's.
	struct pcb_copy *copy;
	size_t ncopy;
	// For each net, the words its copper was made from, and its pads as
	// they stood then, a line each (pcb_net_pads).
	char **netby, ***netpad;
	size_t *nnetpad;
	int failed; // Set by pcb_error.
	// The section of the lock read and written (a group laid out alone,
	// or NULL for the board), and the other sections as they were read,
	// by name ("" the board's), passed on unchanged.
	const char *group;
	char **sec_name, **sec_text;
	size_t nsec;
};

// A pad as it stands on the board: its center, its size along x and y,
// the copper layer it is on (0 the top, nlayer - 1 the bottom; -1 every
// layer, for a hole), and its shape.
struct pcb_spot {
	double x, y, w, h, drill;
	int layer, shape;
};

void *pcb_alloc(size_t n);
void *pcb_grow(void *p, size_t n, size_t size);
char *pcb_strdup(const char *s);
void pcb_warn(const char *fmt, ...);
#define pcb_die(...) (pcb_warn(__VA_ARGS__), exit(EXIT_FAILURE))
// Prints a fault of the source and goes on; the exit status will be 1.
void pcb_error(struct pcb *b, int line, const char *fmt, ...);

// Reads the source, its netlist and footprints (from the directory
// footprints next to the source, then those of lib), and the lock from
// f (which may be NULL): its section of the group (group GROUP up to the
// next), or for a NULL group the board's (before any group line); the
// other sections are kept as they are.
void pcb_read(struct pcb *b, const char *file, char **lib, size_t nlib,
	      FILE *lock, const char *group);
// Frees what pcb_read and the programs made of the board, but the
// footprints, which every board read shares.
void pcb_free(struct pcb *b);
// Writes the lock: the section read made anew (the parts placed and the
// copper), the others as they were, the board's first and the groups'
// in the order of their names.
void pcb_write_lock(const struct pcb *b, FILE *f);
// A pass of a program over one section of the lock: the group's (NULL,
// the board's), read from in (which may be NULL: nothing locked yet),
// the whole lock written to out; returns non-zero on a fault.
typedef int pcb_pass(const char *file, char **lib, size_t nlib,
		     const char *group, FILE *in, FILE *out);
// Runs pass over every group of a block statement of the source, the
// deepest first (a group within another before it; of one depth, in the
// order of their names, the copies last), then over the board, each on
// the lock the one before wrote; or over the group alone, if one is
// given.  A copy is no pass: its sections become its source's, renamed
// part for part and net for net.  The lock is read from in and the last
// pass writes to out.  Returns non-zero if any pass failed.
int pcb_passes(const char *file, char **lib, size_t nlib, const char *group,
	       FILE *in, FILE *out, pcb_pass *pass);
// Tells the faults of the copies (block GROUP as GROUP): a source that
// is no block, or is or holds a copy; a copy and source of which one
// holds the other, or the source shallower; the two not of the same
// parts and nets; a statement of the copy placing what one of its
// source's places.
void pcb_tell_copies(struct pcb *b);
// The statement that places part k: a place or fix statement naming it
// (the last), else one placing a group of it whole (the first), or NULL.
const struct pcb_stmt *pcb_placed_by(const struct pcb *b, size_t k);
// Opens the lock of the command line: the file named, stdin for none; a
// file that is not there is nothing locked yet (NULL).
FILE *pcb_open_lock(const char *path);
// The text of a section of the lock as read, or NULL.
char *pcb_section(const struct pcb *b, const char *group);
// Where pad k of part q stands.
void pcb_spot(const struct pcb *b, const struct pcb_part *q, size_t k,
	      struct pcb_spot *s);
// The courtyard of a placed part, on the board.
void pcb_court(const struct pcb_part *q, double c[4]);
// The distance from a piece of copper to the edge, the outline and the
// cutouts with their round corners: from the segment s (x0 y0 x1 y1)
// grown by r, or with box set from the box s; 0 where it crosses into a
// cutout, less than 0 off the outline or into a round corner.  The
// point of the piece nearest the edge goes to at, unless NULL.
double pcb_edge_dist(const struct pcb *b, int box, const double s[4], double r,
		     double at[2]);
// The keep-out of pad k of part q, a hole a screw goes through: its
// center and radius on the board; returns 0 if it has none.
int pcb_keepout(const struct pcb *b, const struct pcb_part *q, size_t k,
		double *x, double *y, double *r);
// Whether placed parts q and o clash: their courtyards overlap on one
// side, or a keep-out of either overlaps the courtyard of the other
// (a keep-out is on both sides), whatever their nets.  The fault, in
// words that name both, goes to why, of room n.
int pcb_clash(const struct pcb *b, const struct pcb_part *q,
	      const struct pcb_part *o, char *why, size_t n);
// Whether a circle at (x, y) of radius rad (a via) overlaps the keep-out
// of a hole of placed part o; which goes to why, of room n.
int pcb_keeps(const struct pcb *b, const struct pcb_part *o, double x, double y,
	      double rad, char *why, size_t n);
// Whether such a circle overlaps the keep-out of a hole of any placed
// part of the board.
int pcb_in_keepout(const struct pcb *b, double x, double y, double rad,
		   char *why, size_t n);
// Finds a pin, PART.PIN (by number, or by name for a part with pin
// names): the part, and the first pad of the pin; returns 0 if none.
int pcb_pin(const struct pcb *b, const char *s, size_t *part, size_t *pad);
// The net of pad k of part q, or -1.
int pcb_net_of(const struct pcb *b, size_t q, size_t k);
// The words of a statement, joined.
char *pcb_words(char **w, int n);
// What part k is placed by, as the lock keeps it: the nets of its pads,
// pad by pad, and the statement's words ("nets 1 GND 2 OUT by place
// ..."), so the part is placed again when either changes (its turn
// depends on its nets).
char *pcb_by(const struct pcb *b, size_t k, const struct pcb_stmt *st);
// The pads of net k as they stand, a line each ("ch1.gain.rf.1 90.5500
// -126.5500 0.8000 0.9500 top": the pad by its part and name, its
// center, its size along x and y, and its layer, top, bottom or all),
// in the order of the net, those placed alone (a group laid out alone
// places only its own).  The count goes to *n.
char **pcb_net_pads(const struct pcb *b, size_t k, size_t *n);
// The statements about net k as a whole (path, fork, via), joined by " ; ":
// what its copper is made from.  The widths are each connection's.
char *pcb_net_words(const struct pcb *b, size_t k);
// The name of pad k of part q as the lock and the links name it:
// PART.PAD.
char *pcb_pad_key(const struct pcb *b, size_t q, size_t k);
// The width of the copper between pads a and b of net n (PART.PAD; b
// "via" for a pad's via): of a statement width PIN PIN W naming them,
// else the net's (width NET W), else its class's (class CLASS width W),
// else rule track.
double pcb_link_width(const struct pcb *b, int n, const char *a,
		      const char *bb);
// The width of the neck of a pad of part q on plane net n, to its via or
// its plane: the net's, but no wider than rule track on a small part of
// two pins, which takes heat alike at its two pads.
double pcb_neck_width(const struct pcb *b, int n, size_t q);
// Adds a connection; returns its index.
int pcb_add_link(struct pcb *b, int n, const char *a, const char *bb, double w);
// The designator of a part, or its name if it has none: for the lock,
// the messages and the traces, which name a part by both.
const char *pcb_ref(const struct pcb_part *q);
// The layer of a name, or -1.
int pcb_layer(const struct pcb *b, const char *name);
// A stroke of the silkscreen on the board: its width, its ends, the
// part it belongs to (of a text statement, the number of parts and the
// statement's index), and its side.
struct pcb_stroke {
	double w, x0, y0, x1, y1;
	size_t part;
	int ref; // A stroke of a text (a designator, a text statement); 2
		// of a designator that found no room.
	int bottom;
};
// The silkscreen of the board, in the stroke font (stroke.h): the lines
// of each placed part's footprint, its designator, and the text of each
// text statement (its baseline's left end at X Y, its capitals H high).
// A designator of a ref statement stands where it says.  Any other reads
// along its courtyard's long side (turned for a part taller than wide),
// on whichever long side has room, as large as fits from full size down
// to 0.5 mm, at that size as near the courtyard as fits; failing both
// long sides, a short side.  Room is clear of the mask openings and the
// courtyards of the other parts of its side, of the designators of its
// side placed before it, and inside the board by the edge rule; the
// designators with the fewest places clear choose first; with none
// clear, a designator takes its first place.  Nothing moves for them.
// On the bottom a text reads from below.  Returns the number of strokes,
// in *s.
size_t pcb_silk(const struct pcb *b, struct pcb_stroke **s);
// The box of the designator of part q among the strokes, if it has one.
int pcb_ref_box(const struct pcb_stroke *s, size_t n, size_t q, double box[4]);
// What would give the designator of part q room among the strokes, for
// a fault, into buf: "; REF is W x H mm at SIZE (tried SIDES)", then, of
// its four sides at 0.5 mm (in the middle, at the start or the end), the
// one that needs least: clear, "; try: ref PART SIDE [ALIGN] [rot 90]
// size 0.5"; else, if the parts past that side need to move out 2 mm at
// most, the part that needs most and the gap from it, to 0.1 mm, "; try:
// place PART WAY of PART GAP (E mm more SIDE)"; else "; nothing within
// 2 mm: move PART or a neighbor (NAME)", the part in the way.
void pcb_ref_hint(const struct pcb *b, const struct pcb_stroke *s, size_t n,
		  size_t q, char *buf, size_t len);
// The opening of the solder mask round pad k of part q, as a box; 0 if
// the pad is not on that side (bottom or not).
int pcb_opening(const struct pcb *b, size_t q, size_t k, int bottom,
		double box[4]);
// The distance from (x, y) to the copper of a pad, 0 inside it.
double pcb_spot_dist(const struct pcb_spot *s, double x, double y);
// The points of a pad a track joins it at: its center; but for a pad at
// least twice as long as wide (a long pad) its two ends, each a half
// width in from the end, and any point of the line between them.
// Returns the number of points given, 1 or 2.
int pcb_pad_ends(const struct pcb_spot *s, double p[2][2]);
// Whether pad k of part q is soldered: a pad with paste, or a plated
// hole; a contact pad (nopaste) is not.  The via gap keeps vias off the
// pads soldered, against solder wicking down them.
int pcb_soldered(const struct pcb_part *q, size_t k);
// Whether part q is a small part of two pins (each pad 1 mm2 or less: an
// 0402, an 0603), whose pads must take heat alike when soldered.
int pcb_small_two_pin(const struct pcb_part *q);
// Whether pad k of part q stands on a plane of its own net but keeps out
// of its pour, joined by a neck track instead (a thermal relief): a pad
// of a small part of two pins, so it heats as its other pad does.
int pcb_relieved(const struct pcb *b, size_t q, size_t k);
// The distance from (x, y) to the segment (ax, ay)-(bx, by).
double pcb_seg_dist(double x, double y, double ax, double ay, double bx,
		    double by);

#endif
