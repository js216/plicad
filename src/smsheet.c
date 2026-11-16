// SPDX-License-Identifier: GPL-3.0-only
// smsheet.c -- Lay the views of a drawing on sheets, as PostScript
// Copyright (c) 2026 Jakob Kastelic

// Usage: smsheet part.draw views|- > part.ps
//        smsheet -v part.draw > views
//
// A drawing is one pipe:
//   smsheet -v part.draw | smview part.st | smdim | smsheet part.draw -
// Reads the drawing file and the views of smdim, from a file or, for -,
// the standard input: one for each view record of the drawing file and
// in the same order, each at the scale its record gives.  The scale of
// a sheet is that of its first view.  The views of a sheet must have
// room in the template, and at a scale under 1 no ordinate may stand
// more than CROWD inches from its place; else smsheet dies and says
// how large the views are against the room of the template.  Writes
// one page for each sheet, in inches: the template, the views, and on
// sheet 1 the notes, with one for each texture record of the views
// (texture_note).  The lettering is the font named osifont, which
// Ghostscript finds with -sFONTPATH=DIR.  With -v it prints the views
// for smview: the units, the unmarked texture, and each view with its
// records for smdim, their lengths in inches (print_views).
//
// The drawing file is in the language of plicad(7): comments, a
// backslash that runs a line on into the next, and NAME = EXPR.  It has
// one record per line; the text of a record runs to the end of its
// line:
//   units in|mm [in|mm]    (the units of the sizes, the second one in
//                           brackets; needed)
//   template FILE          (the paper, next to the drawing file or in
//                           the templates of the library; needed)
//   field NAME TEXT        (a text of the template, which lists NAME)
//   material TEXT          (note 1, with a box round its number; %%t
//                           in a note is the thickness of the sheet
//                           and %%r its inside bend radius)
//   finish TEXT            (note 2, with a box)
//   boxed TEXT             (a further note with a box)
//   note TEXT              (a note without one)
//   revision REV DATE BY TEXT
//   inserts FORMAT         (the callout of a group of inserts, for smdim)
//   hatching GAP           (the gap between the section lines on the
//                           paper, in mm unless it says in, for smdim)
//   unmarked MATERIAL      (the texture of the surfaces no mark names,
//                           for smview)
//   unhatched MATERIAL...  (the materials whose coats get no hatch, for
//                           smdim)
//   view NAME scale S [hidden] [sheet N] [at X Y] [cut D]
//        [title TEXT]          (a view at scale S; the options in any
//                               order, the scale needed)
// The front, back, left, right, top and bottom views of a sheet stand
// together by third-angle projection, their datums in line; a view
// with at X Y has its datum there, in mm (or with in, inches) from the
// lower left corner of the sheet; the others stand apart.  Each goes to the first free
// place from the top left, and all move together toward the middle of
// the room above what stands at its foot.  The views stand .4 in apart,
// and .25 in off the notes and what the template keeps or takes.  Each
// page says in a comment what share of the free room its views cover.
// Lines that start with a blank belong to the view above them: they are
// records for smdim, and are not read here.  A note with a box needs a
// flag record under a view, which shows where it applies.  The drawing
// file is ASCII, with tabs, and a carriage return may stand before each
// newline.
//
// A template is PostScript that draws the border, the title block and
// the revision table of one paper, in inches from its lower left
// corner, and says in comments where things go:
//   %%PliCadPaper: W H              (the paper; needed)
//   %%PliCadRoom: X0 Y0 X1 Y1       (where the views go; needed)
//   %%PliCadKeep: X0 Y0 X1 Y1       (what they keep off; any number)
//   %%PliCadNotes: X0 Y0 X1 Y1      (the notes: wrapped to its width,
//                                    from its foot up, and taking no
//                                    more of it than they need)
//   %%PliCadRevisions: X0 X1 TOP ROW  (the revision table: two rows of
//                                    heads and one for each revision,
//                                    ROW high, down from TOP)
//   %%PliCadFields: NAME...         (the fields it reads)
// Before the template on each page, smsheet defines as strings each
// field of the drawing file; each listed field the file leaves out, as
// an empty string, but Filename, as the name of the drawing file, its
// directory left out;
// Paper, as "W x H"; Scale, Sheet and Sheets; MaterialNote and
// FinishNote, the numbers of those notes or empty; and Revisions, an
// array of [REV DATE BY TEXT] arrays, empty on the sheets but 1.  The
// template may use the procedures of the prolog: TX, LN and FLG.  It
// runs in a dictionary of its own.
//
// For example, a template with a title in the middle of a C sheet:
//   %%PliCadPaper: 22 17
//   %%PliCadRoom: 0.9 0.9 21.1 15
//   %%PliCadFields: Title
//   [ Title ] 11 16 0 0.2 1 TX
// and a drawing file for it:
//   template title_c.ps
//   field Title BRACKET (L), 2X
//   view top scale 1

#include "expr.h"
#include "font.h"
#include "geom.h"
#include "util.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ROOM 0.4 // Between the views, and round them.
#define TEXT_H 0.12 // The height of the text of the notes.
#define LEAD 0.2 // From one line of the notes to the next.
#define NOTES_HEAD "NOTES: UNLESS OTHERWISE SPECIFIED"
#define NOTE_IN 0.3 // From the number of a note to its text.
#define TITLE_ROOM 0.5 // Under a view, for its title.
#define TITLE_DROP 0.3 // From a view down to its title.
#define TITLE_H 0.16
#define MARGIN 0.25 // Round the notes and what the template keeps.
#define SLACK 1e-9 // Lets a view touch the room it keeps to.
#define FIT 1e-6 // A view this much larger than the room fits it.
#define HUNDRED 100.0 // Hundredths in one inch.
#define CROWD 0.75 // The farthest an ordinate may stand from its place.
#define DASH_SHORT 0.26 // A hidden line shorter than this, in inches,
#define DASH_PART 0.4 // has two dashes, each this share of it.
#define POINTS 72.0 // PostScript points in one inch.
#define THOU 1000.0 // Thousandths in one inch.

enum {
	NAME = 64,
	REAL = 330, // Room for the text of one real.
	KEEPS = 16, // The most keep regions of a template.
	GRID = 4, // The columns of the grid of views; it has fewer rows.
	NUMS = 5, // The most numbers of a record of a view.
	QUARTERS = 4, // The steps of the move toward the middle.
	OW_ROW = 12, // The widths of OW on a line of the prolog.
	DECIMAL = 10, // The base of the number of a note.
	NUM_TEXT = 64, // Room for the text of a number.
	POLY_ROW = 6 // The points of a poly on a line of the PostScript.
};

enum { U_NONE, U_IN, U_MM }; // The units of the dimensions.

struct rect {
	double x0, y0, x1, y1;
};

// A template, by its comments: the size of its paper, the room for the
// views, the regions they keep off, the region of the notes, and where
// the rows of the revision table go: x0 to x1, down from top, row high.
struct paper {
	const char *path;
	char *text;
	double w, h;
	struct rect room, notes;
	struct rect keep[KEEPS];
	size_t nkeep;
	char *fields; // The names of %%PliCadFields, a blank after each.
	int has_notes, has_revisions;
	double x0, x1, top, row;
};

// The records of a view, and the name of each in PostScript.
enum {
	O_FILL,
	O_M,
	O_L,
	O_A,
	O_Z,
	O_LINE,
	O_ARC,
	O_THIN,
	O_ARROW,
	O_DOT,
	O_FLAG,
	O_TEXT,
	O_HATCH,
	O_CLEAR,
	O_POLY,
	O_COUNT
};

static const struct {
	const char *name, *ps;
	size_t n; // Its numbers.
} ops[O_COUNT] = {
	{ "fill", "newpath", 0 }, { "m", "M", 2 },
	{ "l", "L", 2 },	  { "a", "A", NUMS },
	{ "z", "closepath", 0 },  { "line", "LN", 4 },
	{ "arc", "AR", NUMS },	  { "thin", "LN", 4 },
	{ "arrow", "ARROW", 4 },  { "dot", "DOT", 2 },
	{ "flag", "FLG", 2 },	  { "text", "TX", 4 },
	{ "hatch", "HATCH", 4 },  { "clear", "", 4 },
	{ "poly", "", 0 },
};

// The passes over the records of a view, from the gray of the material
// up to the text, and the line of each.
enum {
	P_FILL,
	P_HATCH,
	P_SEAM,
	P_HIDDEN,
	P_CENTER,
	P_PHANTOM,
	P_VISIBLE,
	P_THIN,
	P_MARK,
	P_COUNT
};

static const char *const pens[P_COUNT] = {
	"",
	"0.004 setlinewidth [] 0 setdash",
	"0.006 setlinewidth [] 0 setdash",
	"0.01 setlinewidth [0.06 0.04] 0 setdash",
	"0.006 setlinewidth [0.3 0.04 0.04 0.04] 0 setdash",
	"0.006 setlinewidth [0.3 0.04 0.04 0.04 0.04 0.04] 0 setdash",
	"0.02 setlinewidth [] 0 setdash",
	"0.006 setlinewidth",
	"",
};

// A record of a view: what it is, the pass that draws it, its numbers,
// and for a text its alignment and its string; for a poly its n points,
// x and y in turn in pt; for a fill, whether it is hatched, at the angle
// x[0] with the gap x[1].
struct rec {
	int op, pass;
	double x[NUMS];
	char align;
	const char *s;
	double *pt;
	size_t n;
	int hatched;
};

// A record for smdim under a view: its line and its text, its lengths
// worked out in inches.
struct rule {
	unsigned line;
	char *text;
};

struct view {
	const char *name; // In the drawing file.
	unsigned line; // Of its view record.
	int hidden; // Whether it shows the hidden lines.
	int section; // Whether it is a section, by the plane at cut,
	double cut; // in mm along the axis toward the eye.
	double own_scale; // The scale it gives.
	struct rule *rule;
	size_t nrule, caprule;
	const char *title;
	int sheet, fixed;
	double x, y; // Where its datum stands on the sheet.
	// The view of smdim: its scale, the farthest that an ordinate
	// stands from its place, its box and its records.
	double scale, crowd;
	struct rect box;
	struct rec *rec;
	size_t nrec, caprec;
};

struct note {
	char *text;
	int boxed;
};

struct revision {
	const char *rev, *date, *by, *text;
};

// A field record: the name of a field of the template, its text, and
// the line of the record.
struct field {
	const char *name;
	char *text;
	unsigned line;
};

struct drawing {
	const char *path;
	struct paper *size; // The template, or NULL before its record.
	struct paper paper;
	double *scale; // The scale of each sheet, from sheet 1 at [1].
	double thick, radius;
	struct field *field;
	size_t nfield, capfield;
	struct note *note;
	size_t nnote, capnote;
	struct revision *revision;
	size_t nrevision, caprevision;
	struct view *view;
	size_t nview, capview;
	int sheets;
	size_t material, finish; // The numbers of their notes, or 0.
	char *inserts; // The format of the callouts of inserts, or NULL.
	double hatching; // The gap between section lines, in inches, or 0.
	char *unmarked; // The texture of the surfaces not marked, or NULL.
	char *unhatched; // The materials of coats without a hatch, or NULL.
	char *other; // The note of the texture not marked, or NULL.
	struct names nm; // The names the drawing file defines.
	char where[WHERE_LEN];
	int units[2]; // The units, primary and secondary: U_IN, U_MM, U_NONE.
	struct src_line *ls; // The lines of the drawing file.
	size_t nls;
};

// A text under reading, word by word.
struct cursor {
	const char *what; // The name of the input, for messages.
	unsigned line;
	char *at;
	struct names *nm; // The names of the drawing file, for numbers.
};

// Cuts the next word off the text of c and returns it, or NULL at the
// end.
static char *next_word(struct cursor *c)
{
	char *w = c->at + strspn(c->at, " \t");

	c->at = w + strcspn(w, " \t");
	if (*c->at) {
		*c->at++ = 0;
	}
	return *w ? w : NULL;
}

// Whether a word is left in c.
static int next_word_at(const struct cursor *c)
{
	return c->at[strspn(c->at, " \t")] != 0;
}

static char *word(struct cursor *c)
{
	char *w = next_word(c);

	if (!w) {
		die("%s:%u: too few fields", c->what, c->line);
	}
	return w;
}

// The value of the next word of c, an expression (plicad(7)).
static double num(struct cursor *c)
{
	return eval(c->nm, word(c));
}

// The next word of c as a length, in inches; mm unless it says in.
static double length_in(struct cursor *c)
{
	return num(c) / INCH;
}

// Takes the rest of the text of c, without the blanks before it.
static char *rest(struct cursor *c)
{
	char *s = c->at + strspn(c->at, " \t");

	c->at = s + strlen(s);
	return s;
}

// Prints the first n characters of s as a PostScript string.
static void ps_string(const char *s, size_t n)
{
	printf("(");
	for (size_t i = 0; i < n && s[i]; i++) {
		if (s[i] == '(' || s[i] == ')' || s[i] == '\\') {
			printf("\\");
		}
		printf("%c", s[i]);
	}
	printf(")");
}

// A text: where, at what angle, how aligned and how high.
struct style {
	double x, y, angle;
	char align;
	double h;
};

// Prints the pieces of text s, with a name for each %% sign, in [ ].
static void text_pieces(const char *s)
{
	static const char signs[] = "cvdp";
	static const char *const names[] = { "/DIA", "/CSK", "/DEG", "/PM" };

	printf("[");
	while (*s) {
		const char *sign = NULL;
		size_t n = 1;

		if (s[0] == '%' && s[1] == '%' && s[2]) {
			sign = strchr(signs, s[2]);
		}
		if (sign) {
			printf("%s", names[sign - signs]);
			s += 3;
			continue;
		}
		while (s[n] && s[n] != '%') {
			n++;
		}
		ps_string(s, n);
		s += n;
	}
	printf("]");
}

// Prints the text s.
static void text(struct style st, const char *s)
{
	text_pieces(s);
	printf(" %.4f %.4f %.0f %.3f %d TX\n", st.x, st.y, st.angle, st.h,
	       (st.align != 'l') + (st.align == 'r'));
}

// The text a, a blank if sep is set, and the text b, as one new text.
static char *join(const char *a, int sep, const char *b)
{
	size_t n = strlen(a);
	size_t m = strlen(b);
	char *t = xmalloc(n + m + 2);
	size_t k = 0;

	for (size_t i = 0; i < n; i++) {
		t[k++] = a[i];
	}
	if (sep) {
		t[k++] = ' ';
	}
	for (size_t i = 0; i <= m; i++) {
		t[k++] = b[i];
	}
	return t;
}

static void add_note(struct drawing *d, const char *label, const char *s,
		     int boxed)
{
	struct note nt = { join(label, 0, s), boxed };

	PUSH(d->note, d->nnote, d->capnote, nt);
}

static void read_material(struct drawing *d, struct cursor *c)
{
	add_note(d, "MATERIAL: ", rest(c), 1);
	d->material = d->nnote;
}

static void read_finish(struct drawing *d, struct cursor *c)
{
	add_note(d, "FINISH: ", rest(c), 1);
	d->finish = d->nnote;
}

static void read_boxed(struct drawing *d, struct cursor *c)
{
	add_note(d, "", rest(c), 1);
}

static void read_note(struct drawing *d, struct cursor *c)
{
	add_note(d, "", rest(c), 0);
}

static void read_revision(struct drawing *d, struct cursor *c)
{
	struct revision r = { NULL, NULL, NULL, NULL };

	r.rev = word(c);
	r.date = word(c);
	r.by = word(c);
	r.text = rest(c);
	PUSH(d->revision, d->nrevision, d->caprevision, r);
}

// Reads the numbers of the comment %%NAME: of the template text into
// x; returns 0 if it has none.  With from set, it starts there and
// leaves *from past the comment, for comments that come more than once.
static int template_nums(const struct paper *t, const char *name, double *x,
			 size_t n, const char **from)
{
	char key[NAME];
	const char *at = from ? *from : t->text;

	xsnprintf(key, "%%%%%s:", name);
	for (at = strstr(at, key); at && at > t->text && at[-1] != '\n';
	     at = strstr(at + 1, key)) {
	}
	if (!at) {
		return 0;
	}
	at += strlen(key);
	for (size_t i = 0; i < n; i++) {
		char *end = NULL;

		x[i] = strtod(at, &end);
		if (end == at || !num_ok(x[i])) {
			die("%s: the numbers of %%%%%s are malformed", t->path,
			    name);
		}
		at = end;
	}
	if (*at && *at != '\n' && *at != '\r') {
		die("%s: %%%%%s has more than %zu numbers", t->path, name, n);
	}
	if (from) {
		*from = at;
	}
	return 1;
}

static struct rect as_rect(const double *x)
{
	struct rect r = { x[0], x[1], x[2], x[3] };

	return r;
}

// Dies unless region r of the template lies on its paper.
static void on_paper(const struct paper *t, const char *name, struct rect r)
{
	if (!(r.x0 >= 0 && r.y0 >= 0 && r.x1 <= t->w && r.y1 <= t->h &&
	      r.x0 < r.x1 && r.y0 < r.y1)) {
		die("%s: %%%%%s %g %g %g %g lies off the paper of %g x %g in",
		    t->path, name, r.x0, r.y0, r.x1, r.y1, t->w, t->h);
	}
}

// Reads the names of %%PliCadFields of template t into t->fields, each
// with a blank after it.
static void read_fields(struct paper *t)
{
	const char *at = strstr(t->text, "%%PliCadFields:");
	size_t n = 0;

	free(t->fields);
	t->fields = NULL;
	if (!at) {
		t->fields = join("", 0, "");
		return;
	}
	at += strlen("%%PliCadFields:");
	n = strcspn(at, "\r\n");
	t->fields = xmalloc(n + 2);
	for (size_t i = 0, k = 0; i <= n; i++) {
		char ch = ' ';

		if (i < n) {
			ch = at[i];
		}
		if (ch == ' ' || ch == '\t') {
			if (k && t->fields[k - 1] != ' ') {
				t->fields[k++] = ' ';
			}
		} else {
			t->fields[k++] = ch;
		}
		t->fields[k] = 0;
	}
}

// Whether the template lists the field name.
static int lists(const struct paper *t, const char *name)
{
	size_t n = strlen(name);

	for (const char *at = t->fields; *at; at += strcspn(at, " ") + 1) {
		if (strncmp(at, name, n) == 0 && at[n] == ' ') {
			return 1;
		}
	}
	return 0;
}

// Reads the template the record names, next to the drawing file or in
// the templates of the library, and its comments.
static void read_template(struct drawing *d, struct cursor *c)
{
	struct sibling sb = { d->path, rest(c) };
	struct paper *t = &d->paper;
	double x[4] = { 0 };
	const char *from = NULL;
	char *path = lib_path(sb, "templates");

	free(t->text);
	t->text = read_file(path);
	t->path = d->path;
	free(path);
	if (!template_nums(t, "PliCadPaper", x, 2, NULL) || !(x[0] > 0) ||
	    !(x[1] > 0)) {
		die("%s:%u: the template has no %%%%PliCadPaper: W H", c->what,
		    c->line);
	}
	t->w = x[0];
	t->h = x[1];
	if (!template_nums(t, "PliCadRoom", x, 4, NULL)) {
		die("%s:%u: the template has no %%%%PliCadRoom: X0 Y0 X1 Y1",
		    c->what, c->line);
	}
	t->room = as_rect(x);
	on_paper(t, "PliCadRoom", t->room);
	t->has_notes = template_nums(t, "PliCadNotes", x, 4, NULL);
	t->notes = as_rect(x);
	if (t->has_notes) {
		on_paper(t, "PliCadNotes", t->notes);
	}
	t->has_revisions = template_nums(t, "PliCadRevisions", x, 4, NULL);
	t->x0 = x[0];
	t->x1 = x[1];
	t->top = x[2];
	t->row = x[3];
	read_fields(t);
	from = t->text;
	for (t->nkeep = 0; template_nums(t, "PliCadKeep", x, 4, &from);) {
		if (t->nkeep == KEEPS) {
			die("%s: more than %d %%%%PliCadKeep comments", t->path,
			    KEEPS);
		}
		t->keep[t->nkeep] = as_rect(x);
		on_paper(t, "PliCadKeep", t->keep[t->nkeep++]);
	}
	d->size = t;
}

// The unit of the word w: U_IN or U_MM.
static int unit_of(const struct cursor *c, const char *w)
{
	if (strcmp(w, "in") != 0 && strcmp(w, "mm") != 0) {
		die("%s:%u: units in|mm [in|mm]", c->what, c->line);
	}
	return w[0] == 'i' ? U_IN : U_MM;
}

// Reads units in|mm [in|mm]: the unit of the dimensions, and of the
// second number in brackets after each.
static void read_units(struct drawing *d, struct cursor *c)
{
	const char *w = NULL;

	d->units[0] = unit_of(c, word(c));
	w = next_word(c);
	d->units[1] = w ? unit_of(c, w) : U_NONE;
	if (d->units[1] == d->units[0] || next_word(c)) {
		die("%s:%u: units in|mm [in|mm], two units apart", c->what,
		    c->line);
	}
}

// Whether s is a name of PostScript for a field: a letter, then
// letters, digits and underscores.
static int is_name(const char *s)
{
	if (!isalpha((unsigned char)*s)) {
		return 0;
	}
	for (; *s; s++) {
		if (!isalnum((unsigned char)*s) && *s != '_') {
			return 0;
		}
	}
	return 1;
}

// Reads a field record: its name and its text, which may be empty.
static void read_field(struct drawing *d, struct cursor *c)
{
	static const char *const reserved[] = { "Scale",     "Sheet",
						"Sheets",    "Paper",
						"Revisions", "MaterialNote",
						"FinishNote" };
	struct field f = { word(c), NULL, c->line };

	if (!is_name(f.name)) {
		die("%s:%u: %s is no name for a field: a letter, then letters, "
		    "digits and _",
		    c->what, c->line, f.name);
	}
	for (size_t i = 0; i < sizeof(reserved) / sizeof(*reserved); i++) {
		if (strcmp(f.name, reserved[i]) == 0) {
			die("%s:%u: smsheet gives the field %s itself", c->what,
			    c->line, f.name);
		}
	}
	f.text = join("", 0, rest(c));
	PUSH(d->field, d->nfield, d->capfield, f);
}

// Reads one option of a view record, o, with its values.
static void read_option(struct view *v, struct cursor *c, const char *o)
{
	if (strcmp(o, "scale") == 0) {
		v->own_scale = num(c);
		if (!(v->own_scale > 0)) {
			die("%s:%u: a scale is above 0", c->what, c->line);
		}
	} else if (strcmp(o, "sheet") == 0) {
		v->sheet = (int)lround(num(c));
	} else if (strcmp(o, "at") == 0) {
		v->fixed = 1;
		v->x = length_in(c);
		v->y = length_in(c);
	} else if (strcmp(o, "hidden") == 0) {
		v->hidden = 1;
	} else if (strcmp(o, "cut") == 0) {
		v->section = 1;
		v->cut = length_in(c) * INCH;
	} else if (strcmp(o, "title") == 0) {
		v->title = rest(c);
		if (!*v->title) {
			die("%s:%u: view %s: title without its text", c->what,
			    c->line, v->name);
		}
	} else {
		die("%s:%u: unknown view option %s", c->what, c->line, o);
	}
}

// Whether s names a view.
static int is_view(const char *s)
{
	static const char *const names[] = { "front", "back",	"left", "right",
					     "top",   "bottom", "flat" };

	for (size_t i = 0; i < sizeof(names) / sizeof(*names); i++) {
		if (strcmp(s, names[i]) == 0) {
			return 1;
		}
	}
	return 0;
}

static void read_view(struct drawing *d, struct cursor *c)
{
	struct view v;
	const char *o = NULL;

	memset(&v, 0, sizeof(v));
	v.sheet = 1;
	v.name = word(c);
	v.line = c->line;
	if (!is_view(v.name)) {
		die("%s:%u: the views are front, back, left, right, top, "
		    "bottom and flat",
		    c->what, c->line);
	}
	while ((o = next_word(c)) != NULL) {
		read_option(&v, c, o);
	}
	if (v.sheet < 1) {
		die("%s:%u: a sheet is above 0", c->what, c->line);
	}
	if (!(v.own_scale > 0)) {
		die("%s:%u: view %s: no scale; give it one, as scale 1",
		    c->what, c->line, v.name);
	}
	d->sheets = v.sheet > d->sheets ? v.sheet : d->sheets;
	PUSH(d->view, d->nview, d->capview, v);
}

// Reads a record inserts FORMAT: the text of the callout of each group of
// inserts of one type and one side, for smdim.
static void read_inserts(struct drawing *d, struct cursor *c)
{
	free(d->inserts);
	d->inserts = xstrdup(rest(c));
}

// Reads a record hatching GAP: the gap between the section lines, on the
// paper, for smdim.
static void read_hatching(struct drawing *d, struct cursor *c)
{
	d->hatching = length_in(c);
	if (!(d->hatching > 0)) {
		die("%s:%u: the gap of hatching is above 0", c->what, c->line);
	}
}

// Reads a record unmarked MATERIAL: the texture of the surfaces that no
// mark names, for smview.
static void read_unmarked(struct drawing *d, struct cursor *c)
{
	free(d->unmarked);
	d->unmarked = xstrdup(word(c));
}

// Reads a record unhatched MATERIAL...: the materials whose coats get no
// hatch, for smdim.
static void read_unhatched(struct drawing *d, struct cursor *c)
{
	free(d->unhatched);
	d->unhatched = xstrdup(rest(c));
}

// Reads the record of the drawing file that starts with the word k.
static void read_record(struct drawing *d, struct cursor *c, const char *k)
{
	static const struct {
		const char *name;
		void (*read)(struct drawing *d, struct cursor *c);
	} kinds[] = {
		{ "view", read_view },	       { "template", read_template },
		{ "material", read_material }, { "finish", read_finish },
		{ "boxed", read_boxed },       { "note", read_note },
		{ "units", read_units },       { "revision", read_revision },
		{ "field", read_field },       { "inserts", read_inserts },
		{ "hatching", read_hatching }, { "unmarked", read_unmarked },
		{ "unhatched", read_unhatched },
	};
	size_t i = 0;

	while (i < sizeof(kinds) / sizeof(*kinds) &&
	       strcmp(k, kinds[i].name) != 0) {
		i++;
	}
	if (i == sizeof(kinds) / sizeof(*kinds)) {
		die("%s:%u: unknown record %s", c->what, c->line, k);
	}
	if (!c->at[strspn(c->at, " \t")]) {
		die("%s:%u: the record %s is empty", c->what, c->line, k);
	}
	kinds[i].read(d, c);
}

// Reads the drawing file, whose text stays in memory.
// Dies unless each byte of the drawing file at path is printable
// ASCII, a tab or a newline; a carriage return may stand before a
// newline.
static void check_ascii(const char *path)
{
	FILE *f = fopen(path, "rb");
	unsigned line = 1;
	int b = 0;
	int prev = 0;

	if (!f) {
		die("cannot open %s", path);
	}
	while ((b = getc(f)) != EOF) {
		if (prev == '\r' && b != '\n') {
			break;
		}
		if (b == '\n') {
			line++;
		} else if (!(b >= ' ' && b <= '~') && b != '\t' && b != '\r') {
			break;
		}
		prev = b;
	}
	if (fclose(f) != 0) {
		die("cannot read %s", path);
	}
	if (prev == '\r' && b != '\n') {
		b = '\r';
	}
	if (b != EOF) {
		die("%s:%u: the byte 0x%02x is no printable ASCII, tab or "
		    "newline; a drawing file is ASCII",
		    path, line, (unsigned)b);
	}
}

// Appends the text of a number in inches to b, of room cap.
static void put_inches(char **b, size_t *n, size_t *cap, double x)
{
	char num_text[REAL];

	xsnprintf(num_text, " %s%.6f", signbit(x) ? "-" : "", fabs(x));
	for (const char *t = num_text; *t; t++) {
		PUSH(*b, *n, *cap, *t);
	}
}

// Appends the word w, after a blank, to b.
static void put_word(char **b, size_t *n, size_t *cap, const char *w)
{
	if (*n) {
		PUSH(*b, *n, *cap, ' ');
	}
	for (const char *t = w; *t; t++) {
		PUSH(*b, *n, *cap, *t);
	}
}

// Reads a record for smdim under view v from c, and keeps it with its
// lengths in inches, as smdim takes them:
//   drop FEATURE [x|y|d|r]
//   lin x|y A B [PLUS MINUS]
//   leader X Y TEXT
//   flag X Y N [TEXT]
//   hatch X0 Y0 X1 Y1
// What a rule of the word k takes: a word after k, the lengths, the
// last two of them optional, and a text at its end.
struct rule_form {
	int word;
	size_t lens;
	int num; // A note number after the lengths.
	int text;
};

static struct rule_form rule_form(const struct cursor *c, const char *k)
{
	static const struct {
		const char *k;
		struct rule_form f;
	} forms[] = {
		{ "drop", { 1, 0, 0, 1 } },   { "lin", { 1, 4, 0, 0 } },
		{ "leader", { 0, 2, 0, 1 } }, { "flag", { 0, 2, 1, 1 } },
		{ "hatch", { 0, 4, 0, 0 } },
	};

	for (size_t i = 0; i < sizeof(forms) / sizeof(forms[0]); i++) {
		if (strcmp(k, forms[i].k) == 0) {
			return forms[i].f;
		}
	}
	die("%s:%u: expected drop, lin, leader, flag or hatch", c->what,
	    c->line);
}

// Puts the end of a rule of form f at c into the text t of n characters
// and room cap: its note number, and its text.
static void rule_end(struct cursor *c, struct rule_form f, char **t, size_t *n,
		     size_t *cap)
{
	if (f.num) {
		char num_text[NUM_TEXT];

		xsnprintf(num_text, "%ld", lround(num(c)));
		put_word(t, n, cap, num_text);
	}
	if (f.text && c->at[strspn(c->at, " \t")]) {
		put_word(t, n, cap, rest(c));
	}
	if (c->at[strspn(c->at, " \t")]) {
		die("%s:%u: too much on the line", c->what, c->line);
	}
}

static void read_rule(struct view *v, struct cursor *c)
{
	const char *k = word(c);
	struct rule r = { c->line, NULL };
	size_t n = 0;
	size_t cap = 0;
	struct rule_form f = rule_form(c, k);
	size_t lens = f.lens;

	put_word(&r.text, &n, &cap, k);
	if (f.word) {
		put_word(&r.text, &n, &cap, word(c));
	}
	for (size_t i = 0; i < lens && (i < 2 || next_word_at(c)); i++) {
		put_inches(&r.text, &n, &cap, length_in(c));
	}
	rule_end(c, f, &r.text, &n, &cap);
	PUSH(r.text, n, cap, 0);
	PUSH(v->rule, v->nrule, v->caprule, r);
}

// Defines the name of a line NAME = EXPR, or reads the parameters of a
// line include FILE; returns 0 for any other line.
static int define_line(struct drawing *d, const char *l)
{
	const char *eq = strchr(l, '=');
	size_t a = strspn(l, " \t");
	size_t e = a;
	char name[NAME];

	if (strncmp(l + a, "include ", strlen("include ")) == 0) {
		struct sibling sb = { d->path, l + a + strlen("include ") };
		char *path = NULL;

		sb.name += strspn(sb.name, " \t");
		path = sibling_path(sb);
		read_params(&d->nm, path);
		free(path);
		return 1;
	}
	while (l[e] == '_' || isalnum((unsigned char)l[e])) {
		e++;
	}
	if (a || !eq || e == a || e - a >= sizeof(name) ||
	    eq != l + e + strspn(l + e, " \t")) {
		return 0;
	}
	memcpy(name, l + a, e - a);
	name[e - a] = 0;
	define(&d->nm, name, eval(&d->nm, eq + 1), 0);
	return 1;
}

// Reads the drawing file (plicad(7)): its records, and under each view
// the records for smdim, which begin with a blank.
static void read_drawing(struct drawing *d, char *file)
{
	struct cursor c = { d->path, 0, NULL, &d->nm };

	d->nls = source_lines(file, &d->ls);
	d->nm.where = d->where;
	for (size_t i = 0; i < d->nls; i++) {
		char *l = d->ls[i].text;
		const char *k = NULL;

		c.line = d->ls[i].no;
		c.at = l;
		xsnprintf(d->where, "%s:%u", d->path, c.line);
		if (define_line(d, l)) {
			continue;
		}
		if (*l == ' ' || *l == '\t') {
			if (!d->nview) {
				die("%s:%u: a record for a view before the "
				    "first view",
				    d->path, c.line);
			}
			read_rule(&d->view[d->nview - 1], &c);
			continue;
		}
		k = next_word(&c);
		if (k) {
			read_record(d, &c, k);
		}
	}
	if (!d->nview) {
		die("%s: no view", d->path);
	}
	if (!d->size) {
		die("%s: no template record; name the template of the paper",
		    d->path);
	}
	if (!d->units[0]) {
		die("%s: no units record; a drawing says its units: units "
		    "in|mm [in|mm]",
		    d->path);
	}
}

// The pass that draws a line or an arc, by the letters of its kind and
// its visibility in kv.
static int curve_pass(const char *kv)
{
	if (kv[1] == 'h') {
		return P_HIDDEN;
	}
	if (kv[0] == 's') {
		return P_SEAM;
	}
	if (kv[0] == 'p') {
		return P_PHANTOM;
	}
	return kv[0] == 'c' ? P_CENTER : P_VISIBLE;
}

// The length of the line, arc or poly of record r.
static double rec_length(const struct rec *r)
{
	double l = 0;

	if (r->op == O_ARC) {
		return r->x[2] * fabs(r->x[4] - r->x[3]) * DEG;
	}
	if (r->op != O_POLY) {
		return hypot(r->x[2] - r->x[0], r->x[3] - r->x[1]);
	}
	for (size_t i = 1; i < r->n; i++) {
		l += hypot(r->pt[2 * i] - r->pt[(2 * i) - 2],
			   r->pt[(2 * i) + 1] - r->pt[(2 * i) - 1]);
	}
	return l;
}

// Reads the points of poly record r from c: N, and N points.
static void read_points(struct rec *r, struct cursor *c)
{
	double n = num(c);
	size_t cap = 0;
	size_t k = 0;

	while (next_word_at(c)) {
		double x = num(c);

		PUSH(r->pt, k, cap, x);
	}
	r->n = k / 2;
	if (k % 2 || r->n < 2 || fabs(n - (double)r->n) > 0) {
		die("%s:%u: a poly has N of 2 or more, and N points", c->what,
		    c->line);
	}
}

// Reads a record of a view to draw: the word k, and the rest in c.
// Returns 0 for a word that names none.
static int read_rec(struct view *v, struct cursor *c, const char *k)
{
	struct rec r = { 0, P_MARK, { 0 }, 'l', NULL, NULL, 0, 0 };

	while (r.op < O_COUNT && strcmp(k, ops[r.op].name) != 0) {
		r.op++;
	}
	if (r.op == O_COUNT) {
		return 0;
	}
	if (r.op == O_FILL && next_word_at(c)) {
		if (strcmp(word(c), "hatch") != 0) {
			die("%s:%u: expected fill [hatch ANGLE GAP]", c->what,
			    c->line);
		}
		r.hatched = 1;
		r.x[0] = num(c);
		r.x[1] = num(c);
	}
	if (r.op <= O_Z) {
		r.pass = P_FILL;
	} else if (r.op <= O_ARC || r.op == O_POLY) {
		char kv[2] = { 0, 0 };

		kv[0] = word(c)[0];
		kv[1] = word(c)[0];
		r.pass = curve_pass(kv);
	} else if (r.op == O_HATCH) {
		r.pass = P_HATCH;
	} else if (r.op == O_CLEAR) {
		r.pass = P_COUNT;
	} else if (r.op == O_THIN) {
		r.pass = P_THIN;
	}
	for (size_t i = 0; i < ops[r.op].n; i++) {
		if (r.op == O_TEXT && i == 3) {
			r.align = word(c)[0];
		}
		r.x[i] = num(c);
	}
	if (r.op == O_POLY) {
		read_points(&r, c);
	}
	r.s = rest(c);
	if ((r.op == O_LINE || r.op == O_ARC || r.op == O_THIN ||
	     r.op == O_POLY) &&
	    !(rec_length(&r) > 0)) {
		die("%s:%u: a %s of no length", c->what, c->line, k);
	}
	PUSH(v->rec, v->nrec, v->caprec, r);
	return 1;
}

// Adds the note of a texture record of smdim, MARK CALLOUT, unless the
// drawing has it: TEXTURE: CALLOUT ON SURFACES MARKED "MARK"; for the
// mark -, the texture of the surfaces not marked, the text of the note
// that comes after all the others goes to d->other.
static void texture_note(struct drawing *d, struct cursor *c)
{
	const char *mark = word(c);
	char *a = join("TEXTURE: ", 0, rest(c));
	char *b = join(a, 1, "ON SURFACES MARKED \"");
	char *t = join(b, 0, mark);
	char *s = join(t, 0, "\"");

	if (strcmp(mark, "-") == 0) {
		free(s);
		s = NULL;
		if (!d->other) {
			d->other = join("TEXTURE: ALL OTHER SURFACES ", 0,
					a + strlen("TEXTURE: "));
		}
	}

	for (size_t i = 0; i < d->nnote && s; i++) {
		if (strcmp(d->note[i].text, s) == 0) {
			free(s);
			s = NULL;
		}
	}
	if (s) {
		struct note nt = { s, 0 };

		PUSH(d->note, d->nnote, d->capnote, nt);
	}
	free(a);
	free(b);
	free(t);
}

// Reads a record of a view that is not drawn: the word k, and the
// rest in c.
static void read_other(struct drawing *d, struct view *v, struct cursor *c,
		       const char *k)
{
	if (strcmp(k, "box") == 0) {
		v->box.x0 = num(c);
		v->box.y0 = num(c);
		v->box.x1 = num(c);
		v->box.y1 = num(c);
	} else if (strcmp(k, "thick") == 0) {
		d->thick = num(c);
	} else if (strcmp(k, "radius") == 0) {
		d->radius = num(c);
	} else if (strcmp(k, "crowd") == 0) {
		v->crowd = num(c);
	} else if (strcmp(k, "texture") == 0) {
		texture_note(d, c);
	} else if (strcmp(k, "dim") != 0 && strcmp(k, "far") != 0) {
		die("%s:%u: unknown record %s", c->what, c->line, k);
	}
}

// Reads the views of smdim into the views of the drawing, one for each
// and in their order.
static void read_views(struct drawing *d, const char *path, char *input)
{
	struct names none = { NULL, 0, 0, 0, NULL, NULL, 0, 0, NULL, NULL, 0 };
	struct cursor c = { path, 0, NULL, &none };
	char *more = input;
	struct view *v = NULL;
	size_t n = 0;

	while ((c.at = next_line(&more)) != NULL) {
		const char *k = next_word(&c);

		c.line++;
		if (!k) {
			continue;
		}
		if (strcmp(k, "view") == 0) {
			const char *name = word(&c);

			if (n == d->nview) {
				die("%s:%u: the view %s is one more than %s "
				    "names",
				    path, c.line, name, d->path);
			}
			v = &d->view[n++];
			if (strcmp(name, v->name) != 0) {
				die("%s:%u: the view %s is not the next of %s",
				    path, c.line, name, d->path);
			}
			word(&c);
			v->scale = num(&c);
		} else if (!v) {
			die("%s:%u: a record before the first view", path,
			    c.line);
		} else if (!read_rec(v, &c, k)) {
			read_other(d, v, &c, k);
		}
	}
	if (n < d->nview) {
		die("%s: no view %s, which %s names", path, d->view[n].name,
		    d->path);
	}
}

// Dies unless each flag of a view names a note with a box, and each
// note with a box has a flag: in a view or, for the material and the
// finish, in the title block.
static void check_flags(const struct drawing *d)
{
	int *seen = xcalloc(d->nnote + 1, sizeof(*seen));

	for (size_t i = 0; i < d->nview; i++) {
		const struct view *v = &d->view[i];

		for (size_t k = 0; k < v->nrec; k++) {
			const char *s = v->rec[k].s;
			char *end = NULL;
			unsigned long n = 0;

			if (v->rec[k].op != O_FLAG) {
				continue;
			}
			n = strtoul(s, &end, DECIMAL);
			if (end == s || *end || n < 1 || n > d->nnote ||
			    !d->note[n - 1].boxed) {
				die("%s: view %s: the flag %s names no note "
				    "with a box",
				    d->path, v->name, s);
			}
			seen[n - 1] = 1;
		}
	}
	for (size_t i = 0; i < d->nnote; i++) {
		if (d->note[i].boxed && !seen[i] && i + 1 != d->material &&
		    i + 1 != d->finish) {
			die("%s: note %zu has a box, but no view has a flag "
			    "for it",
			    d->path, i + 1);
		}
	}
	free(seen);
}

// Dies unless the template lists each field the drawing file sets.
static void check_fields(const struct drawing *d)
{
	for (size_t i = 0; i < d->nfield; i++) {
		if (!lists(d->size, d->field[i].name)) {
			die("%s:%u: the template has no field %s; it has: %s",
			    d->path, d->field[i].line, d->field[i].name,
			    d->size->fields);
		}
	}
}

// Puts the thickness of the sheet for each %%t of the notes, and the
// inside bend radius for each %%r, in inches.
static void fill_notes(struct drawing *d)
{
	size_t i = 0;

	while (i < d->nnote) {
		char *s = d->note[i].text;
		char *t = strstr(s, "%%t");
		char *at = t ? t : strstr(s, "%%r");
		long n = lround((t ? d->thick : d->radius) / INCH * THOU);
		long thou = lround(THOU);
		char whole[REAL] = "";
		char value[2 * REAL];

		if (!at) {
			i++;
			continue;
		}
		if (n <= 0) {
			die("%s: a note asks for a bend radius or a thickness "
			    "that the part has not",
			    d->path);
		}
		if (n >= thou) {
			xsnprintf(whole, "%ld", n / thou);
		}
		xsnprintf(value, "%s.%03ld", whole, n % thou);
		*at = 0;
		t = join(s, 0, value);
		d->note[i].text = join(t, 0, at + 3);
		free(t);
		free(s);
	}
}

// The length of the first line of the note text s: as many words as
// fit in wide inches, or as many characters of a longer word, one at
// the least.  Its width goes to w.
static size_t note_line(const char *s, double wide, double *w)
{
	size_t fit = 0;
	size_t i = 0;
	double at = 0;
	double fit_w = 0;

	while (s[i]) {
		double g = 0;
		size_t n = glyph(s + i, &g);

		if (i && at + (g * TEXT_H) > wide) {
			break;
		}
		at += g * TEXT_H;
		i += n;
		if (!s[i] || s[i] == ' ') {
			fit = i;
			fit_w = at;
		}
	}
	if (s[i] && fit) {
		*w = fit_w;
		return fit;
	}
	*w = at;
	return i;
}

// The width of the notes of the template less the room of their
// numbers, for their text.
static double note_room(const struct drawing *d)
{
	return d->size->notes.x1 - d->size->notes.x0 - NOTE_IN;
}

// The number of lines the notes take, with one blank line before each
// note; the width of the widest line, from the left of the region of
// the notes, goes to wide.
static double count_notes(const struct drawing *d, double *wide)
{
	double lines = 0;

	*wide = text_width(NOTES_HEAD, TEXT_H);
	for (size_t i = 0; i < d->nnote; i++) {
		const char *s = d->note[i].text;

		lines++;
		while (*s) {
			double w = 0;

			s += note_line(s, note_room(d), &w);
			s += *s == ' ';
			*wide = fmax(*wide, NOTE_IN + w);
			lines++;
		}
	}
	return lines;
}

// Draws the notes down from y: each with its number, in a box if the
// note says so.
static void draw_notes(const struct drawing *d, double y)
{
	struct style st = { d->size->notes.x0 + NOTE_IN, y, 0, 'l', TEXT_H };

	for (size_t i = 0; i < d->nnote; i++) {
		const char *s = d->note[i].text;

		st.y -= LEAD;
		printf("%.4f %.4f (%zu) %s\n", st.x - NOTE_IN + TEXT_H,
		       st.y - LEAD, i + 1, d->note[i].boxed ? "FLG" : "NUM");
		while (*s) {
			double w = 0;
			size_t n = note_line(s, note_room(d), &w);
			char *buf = xmalloc(n + 1);

			memcpy(buf, s, n);
			buf[n] = 0;
			s += n;
			s += *s == ' ';
			st.y -= LEAD;
			text(st, buf);
			free(buf);
		}
	}
}

// The region the notes take: at the foot of the region of the notes of
// the template, as wide as their widest line and as high as the head
// above them.
static struct rect notes_rect(const struct drawing *d)
{
	struct rect r = d->size->notes;
	double wide = 0;
	double lines = count_notes(d, &wide);

	r.x1 = r.x0 + wide;
	r.y1 = r.y0 + ((lines + 2) * LEAD) + TEXT_H;
	return r;
}

// The region the revision table takes, with its title and its heads.
static struct rect revisions_rect(const struct drawing *d)
{
	const struct paper *t = d->size;
	struct rect r = { t->x0, t->top - (((double)d->nrevision + 2) * t->row),
			  t->x1, t->top };

	return r;
}

// Rectangle r of the template shrunk by ROOM less MARGIN: the views,
// which keep ROOM off it, then keep MARGIN off r.  It may turn inside
// out, which the test of is_free bears.
static struct rect back(struct rect r)
{
	double in = ROOM - MARGIN;
	struct rect b = { r.x0 + in, r.y0 + in, r.x1 - in, r.y1 - in };

	return b;
}

// Lists into out what the views of a sheet must keep off: the keep
// regions of the template, the views with a fixed place, and on sheet
// 1 the notes and the revision table.  Returns their number.
static size_t taken(const struct drawing *d, const struct paper *sz, int sheet,
		    struct rect *out)
{
	size_t n = 0;

	for (size_t i = 0; i < sz->nkeep; i++) {
		out[n++] = back(sz->keep[i]);
	}
	if (sheet == 1 && d->nnote) {
		out[n++] = back(notes_rect(d));
	}
	if (sheet == 1 && d->nrevision && sz->has_revisions) {
		out[n++] = back(revisions_rect(d));
	}
	for (size_t i = 0; i < d->nview; i++) {
		const struct view *v = &d->view[i];
		struct rect r = { v->x + v->box.x0, v->y + v->box.y0,
				  v->x + v->box.x1, v->y + v->box.y1 };

		if (v->sheet == sheet && v->fixed) {
			out[n++] = r;
		}
	}
	return n;
}

// The column and the row of a view in the grid round the front view;
// the column is -1 for a view that stands apart.
struct cell {
	int col, row;
};

static struct cell grid_cell(const struct view *v)
{
	static const char *const names[] = { "left", "front", "right",
					     "back", "top",   "bottom" };
	static const struct cell cells[] = { { 0, 1 }, { 1, 1 }, { 2, 1 },
					     { 3, 1 }, { 1, 2 }, { 1, 0 } };
	static const struct cell apart = { -1, 0 };

	for (size_t i = 0; i < sizeof(names) / sizeof(*names); i++) {
		if (strcmp(v->name, names[i]) == 0) {
			return cells[i];
		}
	}
	return apart;
}

static void scale_text(double s, char *buf, size_t n);

// What a view takes on the paper, round its datum: its box, and the
// room for its title and its scale under it, a line each, centered
// under the box and as wide as their texts.
static struct rect extent(const struct drawing *d, const struct view *v)
{
	struct rect r = v->box;
	int own = fabs(v->scale - d->scale[v->sheet]) > 0;
	double mid = (r.x0 + r.x1) / 2;
	double half = 0;

	if (v->title || own) {
		r.y0 -= TITLE_ROOM;
	}
	if (v->title && own) {
		r.y0 -= LEAD + TEXT_H;
	}
	if (v->title) {
		half = text_width(v->title, TITLE_H) / 2;
	}
	if (own) {
		char buf[REAL] = "";

		scale_text(v->scale, buf, sizeof(buf));
		half = fmax(half, text_width(buf, TEXT_H) / 2);
	}
	r.x0 = fmin(r.x0, mid - half);
	r.x1 = fmax(r.x1, mid + half);
	return r;
}

// The grid of one sheet, columns under [0] and rows under [1].  The
// views of a column share the x of their datum and the views of a row
// its y: lo is the most that one of them takes before its datum, and
// hi after it.  The columns and the rows stand room apart.
struct grid {
	double lo[2][GRID], hi[2][GRID];
	double at[2][GRID]; // Where each column and row starts.
	double size[2]; // The width and the height of the grid.
};

static struct grid make_grid(double room, const struct drawing *d, int sheet)
{
	struct grid g;
	int seen[GRID][GRID] = { { 0 } };

	memset(&g, 0, sizeof(g));
	for (size_t i = 0; i < d->nview; i++) {
		const struct view *v = &d->view[i];
		struct rect e = extent(d, v);
		struct cell c = grid_cell(v);

		if (v->sheet != sheet || v->fixed || c.col < 0) {
			continue;
		}
		if (seen[c.col][c.row]++) {
			die("%s: two views %s on sheet %d; give one an at X Y",
			    d->path, v->name, sheet);
		}
		g.lo[0][c.col] = fmax(g.lo[0][c.col], -e.x0);
		g.hi[0][c.col] = fmax(g.hi[0][c.col], e.x1);
		g.lo[1][c.row] = fmax(g.lo[1][c.row], -e.y0);
		g.hi[1][c.row] = fmax(g.hi[1][c.row], e.y1);
	}
	for (int k = 0; k < 2; k++) {
		for (int i = 0; i < GRID; i++) {
			double w = g.lo[k][i] + g.hi[k][i];
			int gap = w > 0 && g.size[k] > 0;

			g.at[k][i] = g.size[k] + (gap ? room : 0);
			g.size[k] = w > 0 ? g.at[k][i] + w : g.size[k];
		}
	}
	return g;
}

// The room on a sheet for the views, by the template.
static struct rect room(const struct paper *sz)
{
	return sz->room;
}

// Whether r lies in the room in and off each of the n rectangles of
// v, by ROOM or more.
static int is_free(struct rect r, struct rect in, const struct rect *v,
		   size_t n)
{
	for (size_t i = 0; i < n; i++) {
		if (r.x0 < v[i].x1 + ROOM && v[i].x0 < r.x1 + ROOM &&
		    r.y0 < v[i].y1 + ROOM && v[i].y0 < r.y1 + ROOM) {
			return 0;
		}
	}
	return r.x0 >= in.x0 - FIT && r.y0 >= in.y0 - FIT &&
	       r.x1 <= in.x1 + FIT && r.y1 <= in.y1 + FIT;
}

// What is placed on one sheet: the n rectangles the views keep off,
// then the nb blocks of views, and whether each block found room.
struct layout {
	struct rect *v;
	size_t n, nb;
	int ok;
	struct rect in;
	double gap; // Between the columns and the rows of the grid.
};

static int falling(const void *lhs, const void *rhs)
{
	double x = *(const double *)lhs;
	double y = *(const double *)rhs;

	return (x < y) - (x > y);
}

// The places worth a try for the lower left corner of a block: n
// values of x and as many of y.
struct places {
	double *x, *y;
	size_t n;
};

// The places for a block as large as size: at either end of the room,
// and beside each rectangle placed, the x from the highest down and
// the y likewise.
static struct places edges(const struct layout *l, struct rect size)
{
	double w = size.x1 - size.x0;
	double h = size.y1 - size.y0;
	size_t most = (2 * (l->n + l->nb)) + 2;
	struct places p = { xmalloc(most * sizeof(double)),
			    xmalloc(most * sizeof(double)), 0 };

	p.x[p.n] = l->in.x0;
	p.y[p.n++] = l->in.y0;
	p.x[p.n] = l->in.x1 - w - SLACK;
	p.y[p.n++] = l->in.y1 - h - SLACK;
	for (size_t i = 0; i < l->n + l->nb; i++) {
		const struct rect *r = &l->v[i];

		p.x[p.n] = r->x1 + ROOM + SLACK;
		p.y[p.n++] = r->y1 + ROOM + SLACK;
		p.x[p.n] = r->x0 - ROOM - w - SLACK;
		p.y[p.n++] = r->y0 - ROOM - h - SLACK;
	}
	qsort(p.x, p.n, sizeof(*p.x), falling);
	qsort(p.y, p.n, sizeof(*p.y), falling);
	return p;
}

// Finds the first free place for a block as large as size: from the
// top of the room down, and from the left, at the places beside the
// ends of the room and the rectangles placed, where a block that has
// room anywhere has it too.  Adds it to the layout.
static void find_place(struct layout *l, struct rect size)
{
	double w = size.x1 - size.x0;
	double h = size.y1 - size.y0;
	struct places p = edges(l, size);
	int found = 0;

	for (size_t iy = 0; iy < p.n && !found; iy++) {
		for (size_t ix = p.n; ix-- > 0 && !found;) {
			struct rect r = { p.x[ix], p.y[iy], p.x[ix] + w,
					  p.y[iy] + h };

			if (is_free(r, l->in, l->v, l->n + l->nb)) {
				l->v[l->n + l->nb++] = r;
				found = 1;
			}
		}
	}
	free(p.x);
	free(p.y);
	l->ok = l->ok && found;
}

// Whether the blocks of the layout stay free if all move by the x0
// and y0 of by; with set, moves them.
static int shift(struct layout *l, struct rect by, int set)
{
	for (size_t i = l->n; i < l->n + l->nb; i++) {
		struct rect r = { l->v[i].x0 + by.x0, l->v[i].y0 + by.y0,
				  l->v[i].x1 + by.x0, l->v[i].y1 + by.y0 };

		if (!is_free(r, l->in, l->v, l->n)) {
			return 0;
		}
		if (set) {
			l->v[i] = r;
		}
	}
	return 1;
}

// Moves the blocks of the layout together toward the middle of the
// room above what stands at its foot, the notes and the title block:
// all the way if they stay free, or the largest part of it, in
// quarters, that leaves them so.
static void center(struct layout *l)
{
	struct rect all = l->v[l->n];
	double foot = l->in.y0;

	for (size_t i = 0; i < l->n; i++) {
		if (l->v[i].y0 < l->in.y0) {
			foot = fmax(foot, l->v[i].y1 + ROOM);
		}
	}
	for (size_t i = l->n; i < l->n + l->nb; i++) {
		all.x0 = fmin(all.x0, l->v[i].x0);
		all.y0 = fmin(all.y0, l->v[i].y0);
		all.x1 = fmax(all.x1, l->v[i].x1);
		all.y1 = fmax(all.y1, l->v[i].y1);
	}
	for (int k = QUARTERS; k > 0; k--) {
		double part = (double)k / (double)QUARTERS / 2;
		struct rect by = { (l->in.x0 + l->in.x1 - all.x0 - all.x1) *
					   part,
				   (foot + l->in.y1 - all.y0 - all.y1) * part,
				   0, 0 };

		if (shift(l, by, 0)) {
			shift(l, by, 1);
			return;
		}
	}
}

// Whether view v is one of the blocks of a sheet that stand apart.
static int stands_apart(const struct view *v, int sheet)
{
	return v->sheet == sheet && !v->fixed && grid_cell(v).col < 0;
}

// Places the views of a sheet of size sz, with the gap given between
// the columns and rows of the grid round the front view: the grid as
// one block, then each view that stands apart.
static struct layout try_sheet(const struct drawing *d, const struct paper *sz,
			       int sheet, double gap)
{
	struct layout l = {
		xmalloc((d->nview + 4) * sizeof(*l.v)), 0, 0, 1, room(sz), gap
	};
	struct grid g = make_grid(gap, d, sheet);
	struct rect all = { 0, 0, g.size[0], g.size[1] };

	l.n = taken(d, sz, sheet, l.v);
	if (g.size[0] > 0) {
		find_place(&l, all);
	}
	for (size_t i = 0; i < d->nview && l.ok; i++) {
		if (stands_apart(&d->view[i], sheet)) {
			find_place(&l, extent(d, &d->view[i]));
		}
	}
	if (l.ok && l.nb) {
		center(&l);
	}
	return l;
}

// Places the views of a sheet with the widest gap in the grid, of a
// few, that leaves room for them all.
static struct layout lay_sheet(const struct drawing *d, const struct paper *sz,
			       int sheet)
{
	struct layout l = { NULL, 0, 0, 0, { 0, 0, 0, 0 }, 0 };

	for (int k = QUARTERS; k > 0 && !l.ok; k--) {
		free(l.v);
		l = try_sheet(d, sz, sheet, ROOM * k);
	}
	return l;
}

// Moves the views of a sheet to their places in layout l.
static void move_views(struct drawing *d, int sheet, const struct layout *l)
{
	struct grid g = make_grid(l->gap, d, sheet);
	size_t k = l->n + (g.size[0] > 0);

	for (size_t i = 0; i < d->nview; i++) {
		struct view *v = &d->view[i];
		struct rect e = extent(d, v);
		struct cell c = grid_cell(v);

		if (stands_apart(v, sheet)) {
			v->x = l->v[k].x0 - e.x0;
			v->y = l->v[k++].y0 - e.y0;
		} else if (v->sheet == sheet && !v->fixed) {
			v->x = l->v[l->n].x0 + g.at[0][c.col] + g.lo[0][c.col];
			v->y = l->v[l->n].y0 + g.at[1][c.row] + g.lo[1][c.row];
		}
	}
}

// Gives a sheet the scale of its first view.  Returns whether a view of
// the sheet is crowded: at a scale under 1, an ordinate stands more
// than CROWD from its place.
static int sheet_scale(struct drawing *d, int sheet)
{
	int crowded = 0;
	int found = 0;

	for (size_t i = 0; i < d->nview; i++) {
		const struct view *v = &d->view[i];

		if (v->sheet != sheet) {
			continue;
		}
		if (!found) {
			d->scale[sheet] = v->scale;
			found = 1;
		}
		if (v->scale < 1 && v->crowd > CROWD) {
			crowded = 1;
		}
	}
	return crowded;
}

// Puts the views of a sheet in their places, if they have room on size
// sz and are not crowded.  Returns 0 if not.
static int place_sheet(struct drawing *d, const struct paper *sz, int sheet)
{
	int crowded = sheet_scale(d, sheet);
	struct layout l = lay_sheet(d, sz, sheet);
	int ok = l.ok && !crowded;

	if (ok) {
		move_views(d, sheet, &l);
	}
	free(l.v);
	return ok;
}

// The width and the height of rectangle r.
static struct rect dims(struct rect r)
{
	struct rect d = { r.x1 - r.x0, r.y1 - r.y0, 0, 0 };

	return d;
}

// How much more than room a length takes, to the next hundredth up;
// none for one that fits to within FIT.
static double excess(double length, double room)
{
	return length > room + FIT ? ceil((length - room) * HUNDRED) / HUNDRED :
				     0;
}

// Dies with what keeps the views of a sheet off the paper: a crowded
// view, a view larger than the room of the paper, or the views
// together.
static void explain(struct drawing *d, int sheet)
{
	const struct paper *sz = d->size;
	struct rect in = dims(room(sz));
	struct grid g;
	double most = CROWD;

	if (sheet == 1 && d->nnote && !sz->has_notes) {
		die("%s: the template has no %%%%PliCadNotes for the notes",
		    d->path);
	}
	if (sheet == 1 && d->nnote && notes_rect(d).y1 > sz->notes.y1) {
		die("%s: the notes take %.1f in, more than the %.1f in of the "
		    "region of the notes of the template",
		    d->path, notes_rect(d).y1 - sz->notes.y0,
		    sz->notes.y1 - sz->notes.y0);
	}

	sheet_scale(d, sheet);
	for (size_t i = 0; i < d->nview; i++) {
		const struct view *v = &d->view[i];
		struct rect e = dims(extent(d, v));

		if (v->sheet != sheet) {
			continue;
		}
		if (v->scale < 1 && v->crowd > CROWD) {
			die("%s:%u: sheet %d: view %s is crowded at scale %g: "
			    "an ordinate stands %.2f in from its place, more "
			    "than %.2f",
			    d->path, v->line, sheet, v->name, v->scale,
			    v->crowd, most);
		}
		if (e.x0 > in.x0 + FIT || e.y0 > in.y0 + FIT) {
			die("%s:%u: sheet %d: view %s at scale %g takes %.2f "
			    "x %.2f in with its dimensions, %.2f x %.2f in "
			    "more than the room of the template, %.2f x %.2f "
			    "in",
			    d->path, v->line, sheet, v->name, v->scale, e.x0,
			    e.y0, excess(e.x0, in.x0), excess(e.y0, in.y0),
			    in.x0, in.y0);
		}
	}
	g = make_grid(ROOM, d, sheet);
	die("%s: sheet %d: the views at scale %g take %.1f x %.1f in round "
	    "the front view, and the others beside them; the room of the "
	    "template has %.1f x %.1f in, less what the views keep off",
	    d->path, sheet, d->scale[sheet], g.size[0], g.size[1], in.x0,
	    in.y0);
}

// Puts the views of each sheet in their places on the paper of the
// drawing file; dies, with the reason, for a sheet without room.
static void fit_sheets(struct drawing *d)
{
	for (int s = 1; s <= d->sheets; s++) {
		if (!place_sheet(d, d->size, s)) {
			explain(d, s);
		}
	}
}

// Prints the numbers of record r and its name in PostScript.
static void print_rec(const struct rec *r)
{
	for (size_t k = 0; k < ops[r->op].n; k++) {
		printf("%.4f ", r->x[k]);
	}
	printf("%s\n", ops[r->op].ps);
}

// Draws the lines of hatch h of view v in the region of material of
// the path so far, but in the room of each clear record, which a text
// takes.
static void hatch_lines(const struct view *v, const struct rec *h)
{
	printf("eoclip\n");
	for (size_t i = 0; i < v->nrec; i++) {
		const struct rec *c = &v->rec[i];

		if (c->op == O_CLEAR) {
			printf("%.4f %.4f %.4f %.4f CLEAR\n", c->x[0], c->x[1],
			       c->x[2], c->x[3]);
		}
	}
	print_rec(h);
	printf("grestore\n");
}

// Draws the hatch h of view v: its lines, on each region of material
// in turn.  Where two regions lie one on the other, so do their lines.
static void draw_hatch(const struct view *v, const struct rec *h)
{
	int open = 0;

	for (size_t i = 0; i < v->nrec && v->rec[i].pass == P_FILL; i++) {
		if (v->rec[i].op == O_FILL && open) {
			hatch_lines(v, h);
		}
		if (v->rec[i].op == O_FILL) {
			printf("gsave ");
			open = 1;
		}
		print_rec(&v->rec[i]);
	}
	if (open) {
		hatch_lines(v, h);
	}
}

// Draws the section lines of the fill f of view v in the region of the
// path so far, but in the room of each clear record, which a text takes,
// and ends the path.
static void section_lines(const struct view *v, const struct rec *f)
{
	printf("gsave pathbbox eoclip newpath\n");
	for (size_t i = 0; i < v->nrec; i++) {
		const struct rec *c = &v->rec[i];

		if (c->op == O_CLEAR) {
			printf("%.4f %.4f %.4f %.4f CLEAR\n", c->x[0], c->x[1],
			       c->x[2], c->x[3]);
		}
	}
	printf("%.4f %.4f SECT grestore newpath\n", f->x[0], f->x[1]);
}

// Ends the region of material of fill f of view v: gray, or with its
// section lines if it has them.
static void end_fill(const struct view *v, const struct rec *f)
{
	if (f->hatched) {
		section_lines(v, f);
	} else {
		printf("FILL\n");
	}
}

// Draws poly record r: a line through its points.
static void draw_poly(const struct rec *r)
{
	printf("newpath %.4f %.4f M", r->pt[0], r->pt[1]);
	for (size_t i = 1; i < r->n; i++) {
		printf("%s%.4f %.4f L", i % POLY_ROW ? " " : "\n", r->pt[2 * i],
		       r->pt[(2 * i) + 1]);
	}
	printf(" stroke\n");
}

// Draws the records of view v that belong to pass p.  A hidden line
// too short for two dashes of the usual length gets two of its own.
static void draw_pass(const struct view *v, int p)
{
	const struct rec *open = NULL;

	printf("%s%s", pens[p], pens[p][0] ? "\n" : "");
	for (size_t i = 0; i < v->nrec; i++) {
		const struct rec *r = &v->rec[i];
		struct style st = { r->x[0], r->x[1], r->x[2], r->align,
				    r->x[3] };

		if (r->pass != p) {
			continue;
		}
		if (r->op == O_TEXT) {
			text(st, r->s);
			continue;
		}
		if (r->op == O_HATCH) {
			draw_hatch(v, r);
			continue;
		}
		if (r->op == O_FILL && open) {
			end_fill(v, open);
		}
		if (r->op == O_FILL) {
			open = r;
		}
		if (p == P_HIDDEN && rec_length(r) < DASH_SHORT) {
			printf("[%.4f %.4f] 0 setdash ",
			       rec_length(r) * DASH_PART,
			       rec_length(r) * (1 - (2 * DASH_PART)));
		}
		if (r->op == O_POLY) {
			draw_poly(r);
		} else {
			for (size_t k = 0; k < ops[r->op].n; k++) {
				printf("%.4f ", r->x[k]);
			}
			if (r->op == O_FLAG) {
				ps_string(r->s, strlen(r->s));
				printf(" ");
			}
			printf("%s\n", ops[r->op].ps);
		}
		if (p == P_HIDDEN && rec_length(r) < DASH_SHORT) {
			printf("%s\n", pens[p]);
		}
	}
	if (open) {
		end_fill(v, open);
	}
}

// Writes the scale s with two decimals, or with three if it has them.
static void scale_text(double s, char *buf, size_t n)
{
	int k = snprintf(buf, n, "SCALE %.3f", s);

	if (k < 0 || (size_t)k >= n) {
		die("a scale is too long for its buffer");
	}
	if (buf[k - 1] == '0') {
		buf[k - 1] = 0;
	}
}

// Draws a view: pass after pass, then its title and its scale, if it
// is not that of the sheet.
static void draw_view(const struct drawing *d, const struct view *v)
{
	struct style st = { (v->box.x0 + v->box.x1) / 2, v->box.y0 - TITLE_DROP,
			    0, 'c', TITLE_H };

	printf("gsave %.4f %.4f translate\n", v->x, v->y);
	for (int p = 0; p < P_COUNT; p++) {
		draw_pass(v, p);
	}
	if (v->title) {
		text(st, v->title);
		st.y -= LEAD + TEXT_H;
	}
	if (fabs(v->scale - d->scale[v->sheet]) > 0) {
		char buf[REAL] = "";

		scale_text(v->scale, buf, sizeof(buf));
		st.h = TEXT_H;
		text(st, buf);
	}
	printf("grestore\n");
}

// Defines the names the template fills its fields with: the texts of
// the drawing file and of the sheet, the numbers of the notes of the
// material and the finish, and the revisions on sheet 1.
static void fields(const struct drawing *d, int sheet)
{
	const struct paper *t = d->size;
	char buf[REAL] = "";
	const char *slash = strrchr(d->path, '/');
	const char *base = slash ? slash + 1 : d->path;
	const size_t notes[2] = { d->material, d->finish };
	static const char *const flags[2] = { "MaterialNote", "FinishNote" };

	for (size_t i = 0; i < d->nfield; i++) {
		printf("/%s ", d->field[i].name);
		ps_string(d->field[i].text, strlen(d->field[i].text));
		printf(" def\n");
	}
	if (lists(t, "Filename")) {
		printf("/Filename where { pop } { /Filename ");
		ps_string(base, strlen(base));
		printf(" def } ifelse\n");
	}
	for (const char *at = t->fields; *at; at += strcspn(at, " ") + 1) {
		printf("/%.*s where { pop } { /%.*s () def } ifelse\n",
		       (int)strcspn(at, " "), at, (int)strcspn(at, " "), at);
	}
	printf("/Paper (%g x %g) def\n", t->w, t->h);
	{
		static const char *const words[] = { "", "INCHES",
						     "MILLIMETERS" };

		printf("/Units (%s", words[d->units[0]]);
		if (d->units[1] != U_NONE) {
			printf(" [%s]", words[d->units[1]]);
		}
		printf(") def\n");
	}
	scale_text(d->scale[sheet], buf, sizeof(buf));
	printf("/Scale ");
	ps_string(buf + strlen("SCALE "), strlen(buf + strlen("SCALE ")));
	printf(" def\n/Sheet (%d) def\n/Sheets (%d) def\n", sheet, d->sheets);
	for (int i = 0; i < 2; i++) {
		printf("/%s (", flags[i]);
		if (notes[i]) {
			printf("%zu", notes[i]);
		}
		printf(") def\n");
	}
	printf("/Revisions [\n");
	for (size_t k = 0; sheet == 1 && k < d->nrevision; k++) {
		const struct revision *r = &d->revision[k];
		const char *const s[] = { r->rev, r->date, r->by, r->text };

		printf("  [");
		for (int i = 0; i < 4; i++) {
			printf(" ");
			ps_string(s[i], strlen(s[i]));
		}
		printf(" ]\n");
	}
	printf("] def\n");
}

// The width at height 1 of the sign that s starts with, as font.h has
// it.
static double sign_w(const char *s)
{
	double w = 0;

	(void)glyph(s, &w);
	return w;
}

// Whether a view of the drawing, as in use, has section lines.
static int sections(const struct drawing *d)
{
	for (size_t i = 0; i < d->nview; i++) {
		for (size_t k = 0; k < d->view[i].nrec; k++) {
			if (d->view[i].rec[k].hatched) {
				return 1;
			}
		}
	}
	return 0;
}

// Prints the procedure of the section lines: ANGLE GAP on the box X0
// Y0 X1 Y1 of the region clipped, SECT draws thin lines at the angle,
// GAP apart, through the box, each on a line of the origin rotated.
static void section_prolog(void)
{
	printf("/SP { /py exch def /px exch def\n"
	       "  px co mul py si mul add dup u0 lt { dup /u0 exch def } if\n"
	       "  dup u1 gt { /u1 exch def } { pop } ifelse\n"
	       "  py co mul px si mul sub dup w0 lt { dup /w0 exch def } if\n"
	       "  dup w1 gt { /w1 exch def } { pop } ifelse } bind def\n"
	       "/SECT { 16 dict begin /g exch def /a exch def /y1 exch def\n"
	       "  /x1 exch def /y0 exch def /x0 exch def /co a cos def\n"
	       "  /si a sin def /u0 1e9 def /u1 -1e9 def /w0 1e9 def\n"
	       "  /w1 -1e9 def x0 y0 SP x1 y0 SP x1 y1 SP x0 y1 SP\n"
	       "  a rotate 0.004 setlinewidth [] 0 setdash\n"
	       "  w0 g div floor g mul g w1 g add\n"
	       "  { dup u0 exch moveto u1 exch lineto stroke } for\n"
	       "  end } bind def\n");
}

// The prolog of the PostScript: the lettering, and the procedures of the
// drawing.  TX lays a text out by the widths of osifont, OW at height 1,
// as smsheet lays it out: a font that Ghostscript takes in its place,
// where it lacks osifont, is squeezed along the line to them if wider,
// so that no text runs out of its room.  The procedure of section lines
// comes only for a drawing that has them.
static void prolog(const struct drawing *d)
{
	printf("%%!PS-Adobe-3.0\n%%%%Creator: smsheet\n%%%%Pages: %d\n"
	       "%%%%DocumentMedia: paper %.0f %.0f 0 () ()\n%%%%EndComments\n",
	       d->sheets, d->size->w * POINTS, d->size->h * POINTS);
	printf("/osifont findfont dup length dict begin\n"
	       "{ 1 index /FID ne { def } { pop pop } ifelse } forall\n"
	       "/Encoding ISOLatin1Encoding dup length array copy\n"
	       "dup 45 /hyphen put def currentdict end\n"
	       "/Lettering exch definefont pop\n"
	       "/H 1 def\n"
	       "/LN { newpath moveto lineto stroke } bind def\n"
	       "/AR { newpath arc stroke } bind def\n"
	       "/L { lineto } bind def\n"
	       "/M { moveto } bind def\n"
	       "/A { 2 copy gt { arcn } { arc } ifelse } bind def\n"
	       "/FILL { 0.9 setgray eofill 0 setgray } bind def\n"
	       "/SYM { H 0.75 mul } bind def\n"
	       "/PEN { H 0.075 mul setlinewidth [] 0 setdash stroke } bind def\n"
	       "/DIA { currentpoint gsave translate newpath\n"
	       "  H 0.36 mul H 0.5 mul H 0.3 mul 0 360 arc\n"
	       "  H 0.1 mul H -0.05 mul moveto H 0.62 mul H 1.05 mul lineto\n"
	       "  PEN grestore SYM 0 rmoveto } bind def\n"
	       "/CSK { currentpoint gsave translate newpath\n"
	       "  H 0.08 mul H 0.8 mul moveto H 0.56 mul H 0.2 mul lineto\n"
	       "  H 1.04 mul H 0.8 mul lineto PEN grestore\n"
	       "  H 1.2 mul 0 rmoveto } bind def\n"
	       "/DEG { (\\260) show } bind def\n"
	       "/PM { (\\261) show } bind def\n"
	       "/WID { 0 exch { dup type /nametype eq\n"
	       "  { dup /DIA eq { pop SYM } { /CSK eq { H 1.2 mul }\n"
	       "    { (\\260) stringwidth pop } ifelse } ifelse }\n"
	       "  { stringwidth pop } ifelse add } forall } bind def\n"
	       "/OWID { 0 exch { dup type /nametype eq\n"
	       "  { dup /DIA eq { pop 0.75 } { /CSK eq { %.6f }\n"
	       "    { %.6f } ifelse } ifelse }\n"
	       "  { 0 exch { 32 sub dup 0 ge 1 index OW length lt and\n"
	       "    { OW exch get } { pop 0 } ifelse add } forall }\n"
	       "  ifelse add } forall H mul } bind def\n"
	       "/TX { gsave /al exch def /H exch def /rt exch def translate\n"
	       "  rt rotate /Lettering findfont H 1.3518 mul scalefont setfont\n"
	       "  dup OWID /W exch def dup WID dup W gt\n"
	       "  { W exch div 1 scale } { pop } ifelse\n"
	       "  W al mul -2 div H -2 div moveto\n"
	       "  { dup type /nametype eq { cvx exec } { show } ifelse }\n"
	       "  forall grestore } bind def\n"

	       "/ARROW { gsave 2 copy exch atan 5 3 roll translate rotate\n"
	       "  newpath 0 0 moveto -0.13 0.022 lineto -0.13 -0.022 lineto\n"
	       "  closepath fill grestore pop pop } bind def\n"
	       "/DOT { newpath 0.025 0 360 arc fill } bind def\n"
	       "/CLEAR { 4 dict begin /y1 exch def /x1 exch def /y0 exch def\n"
	       "  /x0 exch def newpath -1e3 -1e3 moveto 1e3 -1e3 lineto\n"
	       "  1e3 1e3 lineto -1e3 1e3 lineto closepath x0 y0 moveto\n"
	       "  x1 y0 lineto x1 y1 lineto x0 y1 lineto closepath eoclip\n"
	       "  end } bind def\n"
	       "/HATCH { /hy1 exch def /hx1 exch def /hy0 exch def\n"
	       "  /hx0 exch def hx0 hy0 hx1 hx0 sub hy1 hy0 sub rectclip\n"
	       "  hx0 hy1 sub 0.08 div ceiling 1 hx1 hy0 sub 0.08 div floor\n"
	       "  { 0.08 mul dup hy0 add hy0 moveto hy1 add hy1 lineto\n"
	       "    stroke } for } bind def\n"
	       "/FLG { 3 copy pop 0.12 sub exch 0.12 sub exch 0.24 0.24\n"
	       "  0.008 setlinewidth [] 0 setdash rectstroke NUM } bind def\n"
	       "/NUM { 3 1 roll [ 4 -1 roll ] 3 1 roll 0 %.3f 1 TX }"
	       " bind def\n",
	       sign_w("%%v"), sign_w("%%d"), TEXT_H);
	printf("/OW [");
	for (int c = ' '; c <= '~'; c++) {
		const char one[2] = { (char)c, 0 };
		double w = 0;

		(void)glyph(one, &w);
		printf("%s%.4f", (c - ' ') % OW_ROW ? " " : "\n", w);
	}
	printf(" ] def\n");
	if (sections(d)) {
		section_prolog();
	}
}

// The share of the free sheet that the views of a sheet cover with
// their dimensions: of the room of the template less what the views
// keep off.
static double cover(const struct drawing *d, int sheet)
{
	const struct paper *sz = d->size;
	struct rect *t = xmalloc((d->nview + 4) * sizeof(*t));
	size_t n = taken(d, sz, sheet, t);
	struct rect in = room(sz);
	double all = (in.x1 - in.x0) * (in.y1 - in.y0);
	double views = 0;

	for (size_t i = 0; i < d->nview; i++) {
		struct rect e = extent(d, &d->view[i]);

		if (d->view[i].sheet == sheet) {
			views += (e.x1 - e.x0) * (e.y1 - e.y0);
			n -= d->view[i].fixed != 0;
		}
	}
	for (size_t i = 0; i < n; i++) {
		all -= fmax(t[i].x1 - t[i].x0, 0) * fmax(t[i].y1 - t[i].y0, 0);
	}
	free(t);
	return views / all;
}

static void draw_sheet(struct drawing *d, int sheet)
{
	const struct paper *sz = d->size;
	struct style st = { sz->notes.x0, notes_rect(d).y1 - TEXT_H, 0, 'l',
			    TEXT_H };

	printf("%%%%Page: %d %d\n<< /PageSize [%.0f %.0f] >> setpagedevice\n"
	       "gsave %.0f dup scale 1 setlinecap 1 setlinejoin\n",
	       sheet, sheet, sz->w * POINTS, sz->h * POINTS, POINTS);
	printf("%% sheet %d: scale %g, the views cover %.2f of the free sheet\n",
	       sheet, d->scale[sheet], cover(d, sheet));
	fields(d, sheet);
	printf("gsave 100 dict begin\n%%%%BeginTemplate\n%s\n"
	       "%%%%EndTemplate\nend grestore\n",
	       sz->text);
	if (sheet == 1 && d->nnote) {
		text(st, NOTES_HEAD);
		draw_notes(d, st.y);
	}
	for (size_t i = 0; i < d->nview; i++) {
		if (d->view[i].sheet == sheet) {
			draw_view(d, &d->view[i]);
		}
	}
	printf("grestore showpage\n");
}

static const char *const unit_names[] = { "-", "in", "mm" };

// Prints the views of the drawing file for smview: its units, its
// unmarked texture or -, and each view after a line #line N FILE of its
// record, with its name, -h or - for its hidden lines, its scale, the
// depth of its cut in mm or -, and the records for smdim under it, each
// after a line #line N FILE.
static void print_views(const struct drawing *d)
{
	printf("units %s %s\n", unit_names[d->units[0]],
	       unit_names[d->units[1]]);
	printf("unmarked %s\n", d->unmarked ? d->unmarked : "-");
	for (size_t i = 0; i < d->nview; i++) {
		const struct view *v = &d->view[i];

		printf("#line %u %s\nview %s %s %.9g ", v->line, d->path,
		       v->name, v->hidden ? "-h" : "-", v->own_scale);
		if (v->section) {
			printf("%.9g\n", v->cut);
		} else {
			printf("-\n");
		}
		for (size_t k = 0; k < v->nrule; k++) {
			printf("#line %u %s\n %s\n", v->rule[k].line, d->path,
			       v->rule[k].text);
		}
		if (d->inserts) {
			printf(" inserts %s\n", d->inserts);
		}
		if (d->hatching > 0) {
			printf(" hatching %.6f\n", d->hatching);
		}
		if (d->unhatched) {
			printf(" unhatched %s\n", d->unhatched);
		}
	}
}

static void free_drawing(struct drawing *d)
{
	free(d->inserts);
	free(d->other);
	free(d->unmarked);
	free(d->unhatched);
	for (size_t i = 0; i < d->nview; i++) {
		for (size_t j = 0; j < d->view[i].nrec; j++) {
			free(d->view[i].rec[j].pt);
		}
		free(d->view[i].rec);
		for (size_t k = 0; k < d->view[i].nrule; k++) {
			free(d->view[i].rule[k].text);
		}
		free(d->view[i].rule);
	}
	free(d->scale);
	free_source_lines(d->ls, d->nls);
	free_names(&d->nm);
	for (size_t i = 0; i < d->nnote; i++) {
		free(d->note[i].text);
	}
	free(d->note);
	free(d->revision);
	free(d->view);
	free(d->paper.text);
	free(d->paper.fields);
	for (size_t i = 0; i < d->nfield; i++) {
		free(d->field[i].text);
	}
	free(d->field);
}

int main(int argc, char **argv)
{
	struct drawing d;
	char *file = NULL;
	char *views = NULL;

	memset(&d, 0, sizeof(d));
	d.sheets = 1;
	if (argc == 3 && strcmp(argv[1], "-v") == 0) {
		d.path = argv[2];
		check_ascii(argv[2]);
		file = read_file(argv[2]);
		read_drawing(&d, file);
		print_views(&d);
		free_drawing(&d);
		free(file);
		close_stdout();
		return 0;
	}
	if (argc != 3 || argv[1][0] == '-') {
		die("usage: smsheet part.draw views|- > part.ps\n"
		    "       smsheet -v part.draw > views");
	}
	d.path = argv[1];
	check_ascii(argv[1]);
	file = read_file(argv[1]);
	read_drawing(&d, file);
	views = read_file(argv[2]);
	read_views(&d, strcmp(argv[2], "-") == 0 ? "stdin" : argv[2], views);
	if (d.other) {
		struct note nt = { d.other, 0 };

		PUSH(d.note, d.nnote, d.capnote, nt);
		d.other = NULL;
	}
	d.scale = xcalloc((size_t)d.sheets + 1, sizeof(*d.scale));
	check_flags(&d);
	fill_notes(&d);
	check_fields(&d);
	if (d.nrevision && !d.size->has_revisions) {
		die("%s: the template has no %%%%PliCadRevisions for the "
		    "revisions",
		    d.path);
	}
	fit_sheets(&d);
	prolog(&d);
	for (int s = 1; s <= d.sheets; s++) {
		draw_sheet(&d, s);
	}
	printf("%%%%EOF\n");
	free_drawing(&d);
	free(file);
	free(views);
	close_stdout();
	return 0;
}
