// SPDX-License-Identifier: GPL-3.0-only
// smsheet.c -- Lay the views of a drawing on sheets, as PostScript
// Copyright (c) 2026 Jakob Kastelic

// Usage: smsheet part.draw [views] > part.ps
//        smsheet -r part.draw
//
// Reads the drawing file and the views of smdim, from a file or the
// standard input: one for each view record of the drawing file and in
// the same order, and then, if more than one scale is on offer, all of
// them again at each further scale, the largest scale first.  Each
// sheet takes the largest scale at which its views have room in the
// template and no ordinate stands more than CROWD inches from its
// place at a scale under 1.  If there is none, smsheet dies and says
// how large the views are at the smallest scale against the room of
// the template.  Writes one page for each sheet, in inches: the
// template, the views, and on sheet 1 the notes.  The lettering is the
// font named osifont, which Ghostscript finds with -sFONTPATH=DIR.
// With -r it prints the width and the height of the room for the views
// in the template, in inches, and reads no views.
//
// The drawing file has one record per line; the text of a record runs
// to the end of its line:
//   template FILE          (the paper, next to the drawing file; needed)
//   scale S                (the one scale of the views, for smdraw)
//   field NAME TEXT        (a text of the template, which lists NAME)
//   material TEXT          (note 1, with a box round its number; %%t
//                           in a note is the thickness of the sheet
//                           and %%r its inside bend radius)
//   finish TEXT            (note 2, with a box)
//   boxed TEXT             (a further note with a box)
//   note TEXT              (a note without one)
//   + TEXT                 (more of the field or the note above)
//   revision REV DATE BY TEXT
//   view NAME [hidden] [scale S] [sheet N] [at X Y] [title TEXT]
// The front, back, left, right, top and bottom views of a sheet stand
// together by third-angle projection, their datums in line; a view
// with at X Y has its datum there, in inches from the lower left corner
// of the sheet; the others stand apart.  Each goes to the first free
// place from the top left, and all move together toward the middle of
// the room above what stands at its foot.  The views stand .4 in apart,
// and .25 in off the notes and what the template keeps or takes.  Each
// page says in a comment what share of the free room its views cover.  Lines that start with
// a blank belong to the view above them: they are records for smdim,
// and are not read here.  A note with a box needs a flag record under a
// view, which shows where it applies.
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
// an empty string, but Filename, as the name of the drawing file;
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
//   view top

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
	DECIMAL = 10 // The base of the number of a note.
};

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
};

// The passes over the records of a view, from the grey of the material
// up to the text, and the line of each.
enum {
	P_FILL,
	P_HATCH,
	P_SEAM,
	P_HIDDEN,
	P_CENTRE,
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
	"0.02 setlinewidth [] 0 setdash",
	"0.006 setlinewidth",
	"",
};

// A record of a view: what it is, the pass that draws it, its numbers,
// and for a text its alignment and its string.
struct rec {
	int op, pass;
	double x[NUMS];
	char align;
	const char *s;
};

// A view at one scale: its records, its box, and the farthest that an
// ordinate stands from its place.
struct variant {
	double scale, crowd;
	struct rect box;
	struct rec *rec;
	size_t nrec;
};

// A view: what the drawing file says of it, the same view at each
// scale on offer, and the one in use, which the fields from scale on
// are a copy of.
struct view {
	const char *name; // In the drawing file.
	unsigned line; // Of its view record.
	const char *title;
	int sheet, fixed;
	int own; // Whether the drawing file gives its scale.
	double x, y; // Where its datum stands on the sheet.
	struct variant *var;
	size_t nvar, capvar;
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
	int last_field; // Whether the last record but + was a field.
	struct note *note;
	size_t nnote, capnote;
	struct revision *revision;
	size_t nrevision, caprevision;
	struct view *view;
	size_t nview, capview;
	int sheets;
	size_t material, finish; // The numbers of their notes, or 0.
};

// A text under reading, word by word.
struct cursor {
	const char *what; // The name of the input, for messages.
	unsigned line;
	char *at;
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

static char *word(struct cursor *c)
{
	char *w = next_word(c);

	if (!w) {
		die("%s:%u: too few fields", c->what, c->line);
	}
	return w;
}

static double num(struct cursor *c)
{
	const char *w = word(c);
	char *end = NULL;
	double x = strtod(w, &end);

	if (end == w || *end || !num_ok(x)) {
		die("%s:%u: %s is not a number", c->what, c->line, w);
	}
	return x;
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

// Prints the text s: its pieces, with a name for each %% sign.
static void text(struct style st, const char *s)
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
	printf("] %.4f %.4f %.0f %.3f %d TX\n", st.x, st.y, st.angle, st.h,
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

static void read_more(struct drawing *d, struct cursor *c)
{
	char *old = NULL;

	if (d->last_field) {
		old = d->field[d->nfield - 1].text;
		d->field[d->nfield - 1].text = join(old, 1, rest(c));
		free(old);
		return;
	}
	if (!d->nnote) {
		die("%s:%u: a + before the first note", c->what, c->line);
	}
	old = d->note[d->nnote - 1].text;
	d->note[d->nnote - 1].text = join(old, 1, rest(c));
	free(old);
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

// Reads the template the record names, next to the drawing file, and
// its comments.
static void read_template(struct drawing *d, struct cursor *c)
{
	struct sibling sb = { d->path, rest(c) };
	struct paper *t = &d->paper;
	double x[4] = { 0 };
	const char *from = NULL;
	char *path = sibling_path(sb);

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

// The scale record is for smdraw, which makes the views at that scale.
static void read_scale(struct drawing *d, struct cursor *c)
{
	(void)d;
	if (!(num(c) > 0)) {
		die("%s:%u: a scale is above 0", c->what, c->line);
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
		v->own = num(c) > 0;
	} else if (strcmp(o, "sheet") == 0) {
		v->sheet = (int)lround(num(c));
	} else if (strcmp(o, "at") == 0) {
		v->fixed = 1;
		v->x = num(c);
		v->y = num(c);
	} else if (strcmp(o, "title") == 0) {
		v->title = rest(c);
		if (!*v->title) {
			die("%s:%u: view %s: title without its text", c->what,
			    c->line, v->name);
		}
	} else if (strcmp(o, "hidden") != 0) {
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
	d->sheets = v.sheet > d->sheets ? v.sheet : d->sheets;
	PUSH(d->view, d->nview, d->capview, v);
}

// Reads the record of the drawing file that starts with the word k.
static void read_record(struct drawing *d, struct cursor *c, const char *k)
{
	static const struct {
		const char *name;
		void (*read)(struct drawing *d, struct cursor *c);
	} kinds[] = {
		{ "view", read_view },	       { "template", read_template },
		{ "scale", read_scale },       { "material", read_material },
		{ "finish", read_finish },     { "boxed", read_boxed },
		{ "note", read_note },	       { "+", read_more },
		{ "revision", read_revision }, { "field", read_field },
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
	if (strcmp(k, "+") != 0) {
		d->last_field = strcmp(k, "field") == 0;
	}
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

static void read_drawing(struct drawing *d, char *file)
{
	struct cursor c = { d->path, 0, NULL };
	char *more = file;
	char *l = NULL;

	while ((l = next_line(&more)) != NULL) {
		const char *k = NULL;

		c.line++;
		c.at = l;
		if (*l == ' ' || *l == '\t' || *l == '#') {
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
	return kv[0] == 'c' ? P_CENTRE : P_VISIBLE;
}

// The length of the line or arc of record r.
static double rec_length(const struct rec *r)
{
	if (r->op == O_ARC) {
		return r->x[2] * fabs(r->x[4] - r->x[3]) * DEG;
	}
	return hypot(r->x[2] - r->x[0], r->x[3] - r->x[1]);
}

// Reads a record of a view to draw: the word k, and the rest in c.
// Returns 0 for a word that names none.
static int read_rec(struct view *v, struct cursor *c, const char *k)
{
	struct rec r = { 0, P_MARK, { 0 }, 'l', NULL };

	while (r.op < O_COUNT && strcmp(k, ops[r.op].name) != 0) {
		r.op++;
	}
	if (r.op == O_COUNT) {
		return 0;
	}
	if (r.op <= O_Z) {
		r.pass = P_FILL;
	} else if (r.op <= O_ARC) {
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
	r.s = rest(c);
	if ((r.op == O_LINE || r.op == O_ARC || r.op == O_THIN) &&
	    !(rec_length(&r) > 0)) {
		die("%s:%u: a %s of no length", c->what, c->line, k);
	}
	PUSH(v->rec, v->nrec, v->caprec, r);
	return 1;
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
	} else if (strcmp(k, "dim") != 0 && strcmp(k, "far") != 0) {
		die("%s:%u: unknown record %s", c->what, c->line, k);
	}
}

// Keeps what was read into view v as one more of its variants.
static void keep_variant(struct view *v)
{
	struct variant x = { v->scale, v->crowd, v->box, v->rec, v->nrec };

	PUSH(v->var, v->nvar, v->capvar, x);
	v->rec = NULL;
	v->nrec = v->caprec = 0;
	v->crowd = 0;
}

// Takes variant r of view v into use.
static void use_variant(struct view *v, size_t r)
{
	v->scale = v->var[r].scale;
	v->crowd = v->var[r].crowd;
	v->box = v->var[r].box;
	v->rec = v->var[r].rec;
	v->nrec = v->var[r].nrec;
}

// Reads the views of smdim into the views of the drawing: a round of
// all the views in order for each scale on offer.
static void read_views(struct drawing *d, const char *path, char *input)
{
	struct cursor c = { path, 0, NULL };
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

			if (v) {
				keep_variant(v);
			}
			v = &d->view[n++ % d->nview];
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
	if (v) {
		keep_variant(v);
	}
	if (!n || n % d->nview) {
		die("%s: no view %s, which %s names", path,
		    d->view[n % d->nview].name, d->path);
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
		const struct variant *x = &v->var[0];

		for (size_t k = 0; k < x->nrec; k++) {
			const char *s = x->rec[k].s;
			char *end = NULL;
			unsigned long n = 0;

			if (x->rec[k].op != O_FLAG) {
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
// room for its title and its scale under it, a line each, centred
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
static void centre(struct layout *l)
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
		centre(&l);
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

// Takes round r into use for the views of a sheet; the scale of the
// sheet is that of its first view without a scale of its own, or of
// its first view.  Returns whether a view of the sheet is crowded: at
// a scale under 1, an ordinate stands more than CROWD from its place.
static int use_round(size_t r, struct drawing *d, int sheet)
{
	int crowded = 0;
	int found = 0;

	for (size_t i = 0; i < d->nview; i++) {
		struct view *v = &d->view[i];

		if (v->sheet != sheet) {
			continue;
		}
		use_variant(v, r);
		if (!found || (found < 2 && !v->own)) {
			d->scale[sheet] = v->scale;
			found = 1 + !v->own;
		}
		if (v->scale < 1 && v->crowd > CROWD) {
			crowded = 1;
		}
	}
	return crowded;
}

// Finds the first round, the largest scale, at which the views of a
// sheet have room on size sz and are not crowded; leaves it in use,
// and the views in their places.  Returns 0 if there is none.
static int best_round(struct drawing *d, const struct paper *sz, int sheet)
{
	for (size_t r = 0; r < d->view[0].nvar; r++) {
		int crowded = use_round(r, d, sheet);
		struct layout l = lay_sheet(d, sz, sheet);
		int ok = l.ok && !crowded;

		if (ok) {
			move_views(d, sheet, &l);
		}
		free(l.v);
		if (ok) {
			return 1;
		}
	}
	return 0;
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

// Dies with what keeps the views of a sheet off the paper at the
// smallest scale on offer: a crowded view, a view larger than the room
// of the paper, or the views together.
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

	use_round(d->view[0].nvar - 1, d, sheet);
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

// Gives each sheet the largest scale on offer at which its views have
// room on the paper of the drawing file, and puts them in their places
// there; dies, with the reason, for a sheet that has none.
static void fit_sheets(struct drawing *d)
{
	for (int s = 1; s <= d->sheets; s++) {
		if (!best_round(d, d->size, s)) {
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

// Draws the records of view v that belong to pass p.  A hidden line
// too short for two dashes of the usual length gets two of its own.
static void draw_pass(const struct view *v, int p)
{
	int open = 0;

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
			printf("FILL\n");
		}
		if (r->op == O_FILL) {
			open = 1;
		}
		if (p == P_HIDDEN && rec_length(r) < DASH_SHORT) {
			printf("[%.4f %.4f] 0 setdash ",
			       rec_length(r) * DASH_PART,
			       rec_length(r) * (1 - (2 * DASH_PART)));
		}
		for (size_t k = 0; k < ops[r->op].n; k++) {
			printf("%.4f ", r->x[k]);
		}
		if (r->op == O_FLAG) {
			ps_string(r->s, strlen(r->s));
			printf(" ");
		}
		printf("%s\n", ops[r->op].ps);
		if (p == P_HIDDEN && rec_length(r) < DASH_SHORT) {
			printf("%s\n", pens[p]);
		}
	}
	printf("%s", open ? "FILL\n" : "");
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
	char *base = base_name(d->path);
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
	free(base);
	for (const char *at = t->fields; *at; at += strcspn(at, " ") + 1) {
		printf("/%.*s where { pop } { /%.*s () def } ifelse\n",
		       (int)strcspn(at, " "), at, (int)strcspn(at, " "), at);
	}
	printf("/Paper (%g x %g) def\n", t->w, t->h);
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
	       "/TX { gsave /al exch def /H exch def /rt exch def translate\n"
	       "  rt rotate /Lettering findfont H 1.3518 mul scalefont setfont\n"
	       "  dup WID al mul -2 div H -2 div moveto\n"
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
	       TEXT_H);
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

static void free_drawing(struct drawing *d)
{
	for (size_t i = 0; i < d->nview; i++) {
		for (size_t k = 0; k < d->view[i].nvar; k++) {
			free(d->view[i].var[k].rec);
		}
		free(d->view[i].var);
	}
	free(d->scale);
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
	if (argc == 3 && strcmp(argv[1], "-r") == 0) {
		struct rect in;

		d.path = argv[2];
		check_ascii(argv[2]);
		file = read_file(argv[2]);
		read_drawing(&d, file);
		in = dims(room(d.size));
		printf("%.4f %.4f\n", in.x0, in.y0);
		free_drawing(&d);
		free(file);
		close_stdout();
		return 0;
	}
	if ((argc != 2 && argc != 3) || argv[1][0] == '-') {
		die("usage: smsheet part.draw [views] > part.ps\n"
		    "       smsheet -r part.draw");
	}
	d.path = argv[1];
	check_ascii(argv[1]);
	file = read_file(argv[1]);
	read_drawing(&d, file);
	views = read_file(argc == 3 ? argv[2] : "-");
	read_views(&d, argc == 3 ? argv[2] : "stdin", views);
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
