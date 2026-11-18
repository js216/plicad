// SPDX-License-Identifier: GPL-3.0-only
// schsrc.c -- Read a schematic source and the symbols of its library
// Copyright (c) 2026 Jakob Kastelic

// See sch(5) for the source and schsym(5) for the symbols.  Every error
// is fatal and names the file and the line.

#include "schsrc.h"
#include "stroke.h"
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BIG 1e9 // Beyond any page, in grid units.
#define EMPTY_BOX 1e300 // The corners of a box of nothing yet.
#define GAP_SLACK 1e-9 // What a gap may fall short of rounding.
enum {
	LINE = 4096,
	WORDS = 256,
	WHERE_SPARE = 64, // Room in where beyond a line, for the file.
	FIELDS = 8, // The most fields of a title block.
	DECIMAL = 10,
	INT_MOST = 100000, // The most size of a whole number of a source.
	NAME_LEN = 256, // Room for the name of a file of the library.
	PIN_WORDS = 5, // The words of pin NAME X Y DIR and box X0 Y0 X1 Y1.
	DRAW_KINDS = 6, // The kinds of drawing records.
	KIND_LEN = 64, // Room for the kind of a clear record.
	CLEARS = 4, // The kinds of clear records.
	TRACE_LEN = 256, // Room for the trace of a placement.
	RIGHT_ANGLE = 90, // A turn, in degrees.
	WIRE_WORDS = 5, // The words of wire PIN to PIN WAY.
	DEPTH_NONE = 1073741824, // Deeper than any port.
	PAGE_REFS = 100, // The designators of a prefix a page.
	REF_NUMBER_LEN = 16 // Room for the number of a designator.
};

void *sch_alloc(size_t n)
{
	void *p = calloc(1, n ? n : 1);

	if (!p) {
		sch_die("out of memory");
	}
	return p;
}

static void *grow(void *p, size_t n, size_t size)
{
	void *q = realloc(p, (n + 1) * size);

	if (!q) {
		sch_die("out of memory");
	}
	memset((char *)q + (n * size), 0, size);
	return q;
}

char *sch_strdup(const char *s)
{
	size_t n = strlen(s) + 1;
	char *d = sch_alloc(n);

	memcpy(d, s, n);
	return d;
}

void sch_warn(const char *fmt, ...)
{
	va_list ap;

	(void)fflush(stdout);
	va_start(ap, fmt);
	(void)vfprintf(stderr, fmt, ap);
	va_end(ap);
	(void)fputc('\n', stderr);
}

// Splits the line into words at blanks; a # outside a word starts a
// comment.  Returns the number of words.
static int split(char *s, char **w)
{
	int n = 0;

	for (;;) {
		while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
			s++;
		}
		if (!*s || *s == '#') {
			return n;
		}
		if (n == WORDS) {
			return -1;
		}
		w[n++] = s;
		while (*s && *s != ' ' && *s != '\t' && *s != '\r' &&
		       *s != '\n') {
			s++;
		}
		if (*s) {
			*s++ = 0;
		}
	}
}

// Joins the n words w with one blank between them.
static char *join(char **w, int n)
{
	size_t len = 1;
	char *s = NULL;

	for (int i = 0; i < n; i++) {
		len += strlen(w[i]) + 1;
	}
	s = sch_alloc(len);
	len = 0;
	for (int i = 0; i < n; i++) {
		size_t m = strlen(w[i]);

		if (i) {
			s[len++] = ' ';
		}
		memcpy(s + len, w[i], m);
		len += m;
	}
	return s;
}

static int integer(const char *s, int *v)
{
	char *end = NULL;
	long x = strtol(s, &end, DECIMAL);

	if (end == s || *end || x < -INT_MOST || x > INT_MOST) {
		return 0;
	}
	*v = (int)x;
	return 1;
}

char *sch_default_lib(const char *argv0)
{
	const char *slash = strrchr(argv0, '/');
	const char *tail = "/../lib/symbols";
	size_t n = slash ? (size_t)(slash - argv0) : 1;
	size_t m = strlen(tail) + 1;
	char *s = sch_alloc(n + m);

	memcpy(s, slash ? argv0 : ".", n);
	memcpy(s + n, tail, m);
	return s;
}

char *sch_sheet_path(const struct sch *s)
{
	static const char sheets[] = "/../sheets";
	char name[NAME_LEN];
	char **dirs = (char **)sch_alloc((s->nlib + 1) * sizeof *dirs);
	char *p = NULL;
	int len = snprintf(name, sizeof name, "%s.sheet", s->sheet);

	if (len < 0 || (size_t)len >= sizeof name) {
		sch_die("%s: the name of the sheet is too long", s->sheet);
	}
	for (size_t i = 0; i < s->nlib; i++) {
		size_t room = strlen(s->lib[i]) + sizeof(sheets);

		dirs[i] = sch_alloc(room);
		(void)snprintf(dirs[i], room, "%s%s", s->lib[i], sheets);
	}
	p = sch_find(dirs, s->nlib, name);
	for (size_t i = 0; i < s->nlib; i++) {
		free(dirs[i]);
	}
	free((void *)dirs);
	return p;
}

char *sch_find(char **lib, size_t nlib, const char *name)
{
	for (size_t i = 0; i < nlib; i++) {
		size_t room = strlen(lib[i]) + strlen(name) + 2;
		char *p = sch_alloc(room);
		FILE *f = NULL;

		(void)snprintf(p, room, "%s/%s", lib[i], name);
		f = fopen(p, "r");
		if (f) {
			(void)fclose(f);
			return p;
		}
		free(p);
	}
	return NULL;
}

static int side_of(const char *s)
{
	static const char *const sides[] = { "right", "above", "left",
					     "below" };

	for (int i = 0; i < 4; i++) {
		if (strcmp(s, sides[i]) == 0) {
			return i;
		}
	}
	return -1;
}

static int dir_of(const char *s)
{
	static const char *const dirs[] = { "r", "u", "l", "d" };

	for (int i = 0; i < 4; i++) {
		if (strcmp(s, dirs[i]) == 0) {
			return i;
		}
	}
	return -1;
}

// A page being read: the library; the page; its pins and the first
// pin of each part; which parts are placed, which beside another, and
// which a fault keeps from being placed; the pin each part hangs from
// (or -1) and how far along that pin's lead it hangs; the place of each
// part; the aliases and the statements for later; the page and groups
// open (outputs.ch1), their depth, the line and where it is; the faults
// told so far; and the traces of -v, kept until the designators are
// known: the part and what placed it.
struct reader {
	struct sch_lib *lib;
	struct sch *s;
	struct pinrec *pins;
	size_t npins;
	size_t *base;
	int *placed;
	int *besides;
	int *dead;
	long *anchor_of;
	int *along_of;
	struct place *places;
	struct alias *aliases;
	size_t naliases;
	struct later *laters;
	size_t nlaters;
	char group_path[LINE];
	int depth;
	int line_no;
	char where[LINE + WHERE_SPARE];
	int faults;
	size_t *trace_part;
	size_t ntraces;
	char **trace_text;
};

// Reads the shape NAME.sym: its records (schsym(5)), a line each, or
// more where a line ends in a backslash.
// A line of a file of the library: its path and its number.
struct lib_line {
	const char *path;
	size_t ln;
};

// A symbol being read: its shape, its file and line, and the width of
// its lines so far.
struct sym_read {
	struct sch_shape *s;
	const char *path;
	size_t ln;
	double width;
};

// The record pin NAME X Y DIR of a symbol.
static void sym_pin(struct sym_read *sr, char **w)
{
	struct sch_shape *s = sr->s;
	struct sch_pin p;

	p.name = sch_strdup(w[1]);
	p.dir = dir_of(w[4]);
	if (!integer(w[2], &p.x) || !integer(w[3], &p.y) || p.dir < 0) {
		sch_die("%s:%zu: bad pin", sr->path, sr->ln);
	}
	s->pin = grow(s->pin, s->npin, sizeof *s->pin);
	s->pin[s->npin++] = p;
}

// The record flip PIN PIN of a symbol: the two pins that trade places.
static void sym_flip(struct sym_read *sr, char **w)
{
	struct sch_shape *s = sr->s;

	for (int m = 0; m < 2; m++) {
		for (size_t i = 0; i < s->npin; i++) {
			if (!strcmp(s->pin[i].name, w[m + 1])) {
				s->flip[m] = (int)i;
			}
		}
		if (s->flip[m] < 0) {
			sch_die("%s:%zu: flip of no pin %s", sr->path, sr->ln,
				w[m + 1]);
		}
	}
}

// A drawing record of a symbol, its kind the word k of the n words w,
// drawn when when says (0 always, 1 flipped, 2 plain).
static void sym_draw(struct sym_read *sr, char **w, int n, const int kw[2])
{
	static const char *const kinds[] = { "line",  "loop",	"fill",
					     "blank", "circle", "disk" };
	struct sch_shape *s = sr->s;
	int k = kw[0];
	struct sch_draw d = { -1, kw[1], sr->width, NULL, 0 };

	for (int i = 0; i < DRAW_KINDS; i++) {
		d.kind = !strcmp(w[k], kinds[i]) ? i : d.kind;
	}
	d.n = (size_t)(n - k - 1);
	if (d.kind < 0 || (d.kind >= SCH_CIRCLE && d.n != 3) ||
	    (d.kind >= 0 && d.kind < SCH_CIRCLE && (d.n < 4 || d.n % 2))) {
		sch_die("%s:%zu: bad record %s", sr->path, sr->ln, w[k]);
	}
	d.xy = sch_alloc(d.n * sizeof *d.xy);
	for (size_t i = 0; i < d.n; i++) {
		d.xy[i] = strtod(w[k + 1 + (int)i], NULL);
	}
	s->draw = grow(s->draw, s->ndraw, sizeof *s->draw);
	s->draw[s->ndraw++] = d;
}

// A record of a symbol of n words w.
static void sym_record(struct sym_read *sr, char **w, int n)
{
	struct sch_shape *s = sr->s;
	int kw[2] = { 0, 0 };

	if (!strcmp(w[0], "flip") && n > 3) {
		kw[0] = 1;
		kw[1] = 1;
	} else if (!strcmp(w[0], "plain") && n > 1) {
		kw[0] = 1;
		kw[1] = 2;
	}
	if (strcmp(w[0], "pin") == 0 && n == PIN_WORDS) {
		sym_pin(sr, w);
	} else if (strcmp(w[0], "box") == 0 && n == PIN_WORDS) {
		for (int i = 0; i < 4; i++) {
			s->box[i] = strtod(w[i + 1], NULL);
		}
	} else if (strcmp(w[0], "prefix") == 0 && n == 2) {
		free(s->prefix);
		s->prefix = sch_strdup(w[1]);
	} else if (strcmp(w[0], "side") == 0 && n == 2) {
		s->side = side_of(w[1]);
		if (s->side < 0) {
			sch_die("%s:%zu: bad side", sr->path, sr->ln);
		}
	} else if (strcmp(w[0], "swap") == 0 && n >= 3) {
		s->swap = join(w + 1, n - 1);
	} else if (strcmp(w[0], "flip") == 0 && n == 3) {
		sym_flip(sr, w);
	} else if (strcmp(w[0], "flag") == 0 && n == 2) {
		s->flag = sch_strdup(w[1]);
	} else if (strcmp(w[0], "width") == 0 && n == 2) {
		sr->width = strtod(w[1], NULL);
	} else {
		sym_draw(sr, w, n, kw);
	}
}

// Reads a line of a symbol into buf, of room n, from f, with the lines
// that a backslash at the end goes on to; *ln counts them.  Returns 0 at
// the end.
static int sym_line(char *buf, size_t n, FILE *f, size_t *ln)
{
	size_t len = 0;

	if (!fgets(buf, (int)n, f)) {
		return 0;
	}
	len = strlen(buf);
	(*ln)++;
	// A line that ends in a backslash goes on on the next.
	while (len >= 2 && buf[len - 2] == '\\' && buf[len - 1] == '\n' &&
	       len < n - 1 && fgets(buf + len - 2, (int)(n - len + 2), f)) {
		(*ln)++;
		len = strlen(buf);
	}
	return 1;
}

static struct sch_shape *shape(struct reader *rd, char **lib, size_t nlib,
			       const char *name, const char *where)
{
	char buf[LINE];
	char *w[WORDS];
	char *path = NULL;
	char *file = NULL;
	size_t room = strlen(name) + sizeof(".sym");
	struct sym_read sr = { NULL, NULL, 0, SCH_LW };
	FILE *f = NULL;

	for (size_t i = 0; i < rd->lib->nshape; i++) {
		if (strcmp(rd->lib->shape[i]->name, name) == 0) {
			return rd->lib->shape[i];
		}
	}
	file = sch_alloc(room);
	(void)snprintf(file, room, "%s.sym", name);
	path = sch_find(lib, nlib, file);
	if (!path) {
		sch_die("%s: no symbol %s in the library", where, name);
	}
	f = fopen(path, "r");
	if (!f) {
		sch_die("%s: cannot read", path);
	}
	sr.path = path;
	sr.s = sch_alloc(sizeof *sr.s);
	sr.s->name = sch_strdup(name);
	sr.s->prefix = sch_strdup("X");
	sr.s->side = 0;
	sr.s->flip[0] = sr.s->flip[1] = -1;
	while (sym_line(buf, sizeof buf, f, &sr.ln)) {
		int n = split(buf, w);

		if (n >= 1 && w[0][0] != '#') {
			sym_record(&sr, w, n);
		}
	}
	(void)fclose(f);
	if (!sr.s->npin) {
		sch_die("%s: the symbol has no pins", path);
	}
	free(path);
	free(file);
	rd->lib->shape = (struct sch_shape **)grow((void *)rd->lib->shape,
						   rd->lib->nshape,
						   sizeof *rd->lib->shape);
	rd->lib->shape[rd->lib->nshape++] = sr.s;
	return sr.s;
}

// Reads the device NAME: a pin table if the library has a file of that
// name, else the generic shape NAME.sym.
// A record of the pin table of device d at the line ln of path.
static void table_record(struct reader *rd, struct sch_device *d,
			 const struct lib_line *at, char **w, int n)
{
	char **lib = rd->s->lib;
	size_t nlib = rd->s->nlib;

	if (strcmp(w[0], "shape") == 0 && n == 2 && !d->shape) {
		d->shape = shape(rd, lib, nlib, w[1], at->path);
		d->pnum = (char **)sch_alloc(d->shape->npin * sizeof *d->pnum);
	} else if (strcmp(w[0], "prefix") == 0 && n == 2) {
		d->prefix = sch_strdup(w[1]);
	} else if (strcmp(w[0], "pin") == 0 && n == 3 && d->shape) {
		size_t i = 0;

		while (i < d->shape->npin &&
		       strcmp(d->shape->pin[i].name, w[1]) != 0) {
			i++;
		}
		if (i == d->shape->npin) {
			sch_die("%s:%zu: shape %s has no pin %s", at->path,
				at->ln, d->shape->name, w[1]);
		}
		d->pnum[i] = sch_strdup(w[2]);
	} else if (strcmp(w[0], "nc") == 0 && n >= 2) {
		d->nc = join(w + 1, n - 1);
	} else {
		sch_die("%s:%zu: bad record", at->path, at->ln);
	}
}

// Reads the pin table of device d at path.
static void read_table(struct reader *rd, struct sch_device *d,
		       const char *path)
{
	char buf[LINE];
	char *w[WORDS];
	struct lib_line at = { path, 0 };
	FILE *f = fopen(path, "r");

	if (!f) {
		sch_die("%s: cannot read", path);
	}
	while (fgets(buf, sizeof buf, f)) {
		int n = split(buf, w);

		at.ln++;
		if (n) {
			table_record(rd, d, &at, w, n);
		}
	}
	(void)fclose(f);
	if (!d->shape) {
		sch_die("%s: no shape record", path);
	}
	for (size_t i = 0; i < d->shape->npin; i++) {
		if (!d->pnum[i]) {
			sch_die("%s: no number for pin %s", path,
				d->shape->pin[i].name);
		}
	}
	if (!d->prefix) {
		d->prefix = d->shape->prefix;
	}
}

static struct sch_device *device(struct reader *rd, char **lib, size_t nlib,
				 const char *name, const char *where)
{
	char *path = NULL;
	struct sch_device *d = NULL;

	for (size_t i = 0; i < rd->lib->ndev; i++) {
		if (strcmp(rd->lib->dev[i]->name, name) == 0) {
			return rd->lib->dev[i];
		}
	}
	d = sch_alloc(sizeof *d);
	d->name = sch_strdup(name);
	path = sch_find(lib, nlib, name);
	if (!path) {
		d->shape = shape(rd, lib, nlib, name, where);
		d->prefix = d->shape->prefix;
		d->pnum = (char **)sch_alloc(d->shape->npin * sizeof *d->pnum);
		for (size_t i = 0; i < d->shape->npin; i++) {
			d->pnum[i] = d->shape->pin[i].name;
		}
	} else {
		read_table(rd, d, path);
		free(path);
	}
	rd->lib->dev = (struct sch_device **)grow(
		(void *)rd->lib->dev, rd->lib->ndev, sizeof *rd->lib->dev);
	rd->lib->dev[rd->lib->ndev++] = d;
	return d;
}

const char *sch_pnum(const struct sch_device *d, size_t p)
{
	return d->pnum[p];
}

struct sch_at sch_pin_at(const struct sch_part *q, size_t p)
{
	struct sch_at at = { 0, 0, 0 };
	const struct sch_shape *sh = q->dev->shape;
	const struct sch_pin *pin = NULL;
	int px = 0;
	int py = 0;
	int d = 0;

	if (q->flip && sh->flip[0] >= 0) {
		if ((int)p == sh->flip[0]) {
			p = (size_t)sh->flip[1];
		} else if ((int)p == sh->flip[1]) {
			p = (size_t)sh->flip[0];
		}
	}
	pin = &sh->pin[p];
	px = pin->x;
	py = pin->y;
	d = pin->dir;
	if (q->mirror) {
		px = -px;
		if (d == SCH_R) {
			d = SCH_L;
		} else if (d == SCH_L) {
			d = SCH_R;
		}
	}
	for (int i = 0; i < q->rot; i++) {
		int t = px;

		px = -py;
		py = t;
		d = (d + 1) % 4;
	}
	at.x = q->x + px;
	at.y = q->y + py;
	at.dir = d;
	return at;
}

void sch_box(const struct sch_part *q, double b[4])
{
	const double *s = q->dev->shape->box;
	const double x[4] = { s[0], s[2], s[2], s[0] };
	const double y[4] = { s[1], s[1], s[3], s[3] };

	b[0] = b[1] = BIG;
	b[2] = b[3] = -BIG;
	for (int i = 0; i < 4; i++) {
		double px = q->mirror ? -x[i] : x[i];
		double py = y[i];

		for (int k = 0; k < q->rot; k++) {
			double t = px;

			px = -py;
			py = t;
		}
		px += q->x;
		py += q->y;
		b[0] = px < b[0] ? px : b[0];
		b[1] = py < b[1] ? py : b[1];
		b[2] = px > b[2] ? px : b[2];
		b[3] = py > b[3] ? py : b[3];
	}
}

double sch_text_width(const char *s, double size)
{
	return stroke_width(s, size);
}

struct sch_dims sch_label_size(const struct sch_part *q)
{
	int lines = 1;
	double v = 0;
	struct sch_dims d = { 0, 0 };
	double *w = &d.w;

	*w = sch_text_width(q->dev->prefix, SCH_TEXT) +
	     sch_text_width(q->ref ? q->ref + strlen(q->dev->prefix) : "400",
			    SCH_TEXT);
	if ((q->value || q->rating) && !q->novalue) {
		v = (q->value ? sch_text_width(q->value, SCH_TEXT) : 0) +
		    (q->rating ? sch_text_width(q->rating, SCH_TEXT) + 2 : 0);
		lines = 2;
	}
	*w = *w > v ? *w : v;
	d.h = SCH_DESCENT + SCH_TEXT + ((lines - 1) * SCH_LEAD);
	return d;
}

// The statements are read first and carried out after: names are
// looked up once all are known, and the parts are placed in the order
// their anchors allow, not the order they were written.
enum { ROOT, FROM, BESIDE, OF, BETWEEN }; // How a part is placed.

struct place {
	int kind, dir, line;
	int gap; // Of WAY of PART N: the least gap between the boxes, or -1.
	char *ref[2], *scope;
	size_t pid[2], part; // The pins or the part it is placed against.
	char *group; // Or the group, as its path and a dot ("ch1.").
};

// A pin: the root of its net (union-find), how far along its lead its
// branches reach, whether its line goes on, and its flag.
struct pinrec {
	size_t part, pin, up;
	int reach, series, used, line;
	struct sch_shape *flag;
	const char *global; // The global name the pin is on, if any.
	long flagpart; // The part that draws the flag, or -1.
};

struct alias {
	char *name, *ref, *scope; // The full name, what it stands for.
	int depth, line;
	long pid; // The pin, once known, or -1.
};

// A statement to be carried out once the names are known: a wire, a
// flag.
struct later {
	int kind, side, line;
	char *ref[2], *scope, *name;
	long pid; // Of a class, its pin once known, or -1.
};

enum { WIRE, FLAG, NC, CLASS };

static const int way_dx[4] = { 1, 0, -1, 0 };
static const int way_dy[4] = { 0, 1, 0, -1 };

// Tells a fault of the source and goes on, counted in n.
#define FLAG_PAD 0.6 // A flag's label beyond its text, in grid units,
#define FLAG_H 1.2 // its height,
#define FLAG_OUT 0.4 // and how far it stands out from the pin.
#define LABEL_GAP_X 0.3 // From a body to its label beside it,
#define LABEL_GAP_Y 0.4 // and above or below it.
#define CLEAR_PAD 0.5 // How far apart bodies keep.
#define SCALE_MIN 0.1 // The least scale of a page, past it,
#define SCALE_MAX 10.0 // and the most.

#define fault(n, ...) (sch_warn(__VA_ARGS__), (n)++)

// Room: from a pin to what continues it (FIRST) or to its first branch
// (HANG), and the least from a branch to the next or to what continues
// past them (STEP); the labels widen it.  FAR is as far as a part is
// pushed to find room.
enum { FIRST = 2, HANG = 3, STEP = 2, BESIDE_OFF = 3, OF_GAP = 2, FAR = 80 };
enum { PAIR_GAP = 2 }; // Between parts in parallel, in grid units.
enum { ROOT_GAP = 6 }; // Right of all placed, a part against nothing.

// Whether buf is a record clear KIND D: its KIND into kind, D into *d.
static int clear_record(const char *buf, char kind[KIND_LEN], double *d)
{
	int n = 0;
	char *end = NULL;

	if (sscanf(buf, "clear %63s%n", kind, &n) != 1) {
		return 0;
	}
	*d = strtod(buf + n, &end);
	return end != buf + n;
}

// Reads the room the drawing keeps from the clear records of the sheet
// template: clear body, text, label and flag, each in points.
static void read_clearances(struct reader *rd, struct sch *s)
{
	static const char *const what[CLEARS] = { "body", "text", "label",
						  "flag" };
	double *v[CLEARS] = { &s->clear_body, &s->clear_text, &s->clear_label,
			      &s->clear_flag };
	char *path = sch_sheet_path(s);
	char buf[LINE];
	char kind[KIND_LEN];
	int have[CLEARS] = { 0, 0, 0, 0 };
	FILE *f = path ? fopen(path, "r") : NULL;

	if (!f) {
		fault(rd->faults, "%s: no sheet %s in the library", s->file,
		      s->sheet);
		free(path);
		return;
	}
	while (fgets(buf, sizeof buf, f)) {
		double d = NAN;

		if (!clear_record(buf, kind, &d)) {
			continue;
		}
		for (int i = 0; i < CLEARS; i++) {
			if (!strcmp(kind, what[i]) && d >= 0) {
				*v[i] = d;
				have[i] = 1;
			}
		}
	}
	(void)fclose(f);
	for (int i = 0; i < CLEARS; i++) {
		if (!have[i]) {
			fault(rd->faults, "%s: no clear %s", path, what[i]);
		}
	}
	free(path);
}

static void at_line(struct reader *rd, int line)
{
	(void)snprintf(rd->where, sizeof rd->where, "%s:%d", rd->s->file, line);
}

static size_t root(struct reader *rd, size_t i)
{
	while (rd->pins[i].up != i) {
		rd->pins[i].up = rd->pins[rd->pins[i].up].up;
		i = rd->pins[i].up;
	}
	return i;
}

static void join_pins(struct reader *rd, size_t a, size_t b)
{
	rd->pins[a].used = rd->pins[b].used = 1;
	rd->pins[root(rd, a)].up = root(rd, b);
}

// The full name of name in the group open now.
static char *in_path(struct reader *rd, const char *name)
{
	size_t n = strlen(rd->group_path) + strlen(name) + 2;
	char *s = sch_alloc(n);

	(void)snprintf(s, n, "%s%s%s", rd->group_path,
		       *rd->group_path ? "." : "", name);
	return s;
}

static long part_named(struct reader *rd, const char *full)
{
	for (size_t i = 0; i < rd->s->npart; i++) {
		if (strcmp(rd->s->part[i].name, full) == 0) {
			return (long)i;
		}
	}
	return -1;
}

// A name and the group it is named in.
struct scoped {
	const char *name;
	const char *scope;
};

enum { NOT_HERE = -3 }; // A name not in the group looked in.

// The pin of the full name: an alias, or PART.PIN; -2 if the part has no
// such pin (the fault told), or NOT_HERE.
static long pin_named(struct reader *rd, const char *full)
{
	char obj[2 * LINE];
	char *dot = NULL;
	long q = 0;

	for (size_t i = 0; i < rd->naliases; i++) {
		if (strcmp(rd->aliases[i].name, full) == 0) {
			return rd->aliases[i].pid;
		}
	}
	memcpy(obj, full, strlen(full) + 1);
	dot = strrchr(obj, '.');
	if (!dot) {
		return NOT_HERE;
	}
	*dot++ = 0;
	q = part_named(rd, obj);
	if (q < 0) {
		return NOT_HERE;
	}
	{
		const struct sch_part *r = &rd->s->part[q];

		for (size_t p = 0; p < r->dev->shape->npin; p++) {
			if (!strcmp(r->dev->shape->pin[p].name, dot) ||
			    !strcmp(r->dev->pnum[p], dot)) {
				return (long)(rd->base[q] + p);
			}
		}
	}
	fault(rd->faults, "%s: %s has no pin %s", rd->where, obj, dot);
	return -2;
}

// Looks a name up in its group, then in each group round it, out to
// the page.  kind 0 finds a part, kind 1 a pin (PART.PIN or a port).
// Returns the part or the pin, -1 for a port not yet known, or -2 (the
// fault told).
static long lookup(struct reader *rd, struct scoped sn, int kind)
{
	char prefix[LINE];

	(void)snprintf(prefix, sizeof prefix, "%s", sn.scope);
	for (;;) {
		char full[2 * LINE];
		char *dot = NULL;
		long q = 0;

		(void)snprintf(full, sizeof full, "%s%s%s", prefix,
			       *prefix ? "." : "", sn.name);
		q = kind == 0 ? part_named(rd, full) : pin_named(rd, full);
		if (kind == 0 ? q >= 0 : q != NOT_HERE) {
			return q;
		}
		if (!*prefix) {
			break;
		}
		dot = strrchr(prefix, '.');
		if (dot) {
			*dot = 0;
		} else {
			*prefix = 0;
		}
	}
	fault(rd->faults, "%s: no %s %s here", rd->where, kind ? "pin" : "part",
	      sn.name);
	return -2;
}

// The pin of a name, or -1 (the fault told).
static long pin_of(struct reader *rd, struct scoped sn)
{
	long p = lookup(rd, sn, 1);

	if (p == -1) {
		fault(rd->faults, "%s: port %s leads in a circle", rd->where,
		      sn.name);
	}
	return p < 0 ? -1 : p;
}

// The path a/b, allocated.
static char *join_path(const char *a, const char *b)
{
	size_t n = strlen(a) + strlen(b) + 2;
	char *s = sch_alloc(n);

	(void)snprintf(s, n, "%s/%s", a, b);
	return s;
}

// Finds the pin table of a part number: a file of its name in lower
// case in a directory of the library (the maker's).  Returns its name
// in the library, or NULL.
static char *find_table(struct reader *rd, const char *value)
{
	char low[NAME_LEN];
	size_t i = 0;

	for (i = 0; i < sizeof low - 1 && value[i]; i++) {
		low[i] = (char)tolower((unsigned char)value[i]);
		if (low[i] == '/') {
			return NULL;
		}
	}
	low[i] = 0;
	for (size_t k = 0; k < rd->s->nlib; k++) {
		DIR *d = opendir(rd->s->lib[k]);
		const struct dirent *e = NULL;

		if (!d) {
			continue;
		}
		while ((e = readdir(d)) != NULL) {
			char *name = NULL;
			char *full = NULL;
			FILE *f = NULL;

			if (e->d_name[0] == '.') {
				continue;
			}
			name = join_path(e->d_name, low);
			full = join_path(rd->s->lib[k], name);
			f = fopen(full, "r");
			free(full);
			if (f) {
				(void)fclose(f);
				closedir(d);
				return name;
			}
			free(name);
		}
		closedir(d);
	}
	return NULL;
}

static int dir_word(const char *s)
{
	static const char *const dirs[] = { "right", "up", "left", "down" };

	for (int i = 0; i < 4; i++) {
		if (strcmp(s, dirs[i]) == 0) {
			return i;
		}
	}
	return -1;
}

// Whether s is a plain word: letters, digits, - and _ alone.
static int plain_word(const char *s)
{
	size_t n = strspn(s, "abcdefghijklmnopqrstuvwxyz"
			     "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_");

	return n && !s[n];
}

static size_t add_part(struct reader *rd, const struct sch_part *q,
		       const struct place *pl)
{
	size_t k = rd->s->npart;
	size_t n = q->dev->shape->npin;

	rd->s->part = grow(rd->s->part, k, sizeof *rd->s->part);
	rd->s->part[rd->s->npart++] = *q;
	rd->base = grow(rd->base, k, sizeof *rd->base);
	rd->placed = grow(rd->placed, k, sizeof *rd->placed);
	rd->dead = grow(rd->dead, k, sizeof *rd->dead);
	rd->besides = grow(rd->besides, k, sizeof *rd->besides);
	rd->anchor_of = grow(rd->anchor_of, k, sizeof *rd->anchor_of);
	rd->along_of = grow(rd->along_of, k, sizeof *rd->along_of);
	rd->places = grow(rd->places, k, sizeof *rd->places);
	rd->base[k] = rd->npins;
	rd->anchor_of[k] = -1;
	rd->places[k] = *pl;
	for (size_t p = 0; p < n; p++) {
		struct pinrec r;

		memset(&r, 0, sizeof r);
		r.part = k;
		r.pin = p;
		r.up = rd->npins;
		r.line = q->line;
		r.flagpart = -1;
		rd->pins = grow(rd->pins, rd->npins, sizeof *rd->pins);
		rd->pins[rd->npins++] = r;
	}
	return k;
}

static int in_box(const double b[4], double x, double y)
{
	return x > b[0] && x < b[2] && y > b[1] && y < b[3];
}

static int overlap(const double a[4], const double b[4])
{
	return a[0] < b[2] && b[0] < a[2] && a[1] < b[3] && b[1] < a[3];
}

// The box the label of a placed part takes, in grid units, on its own
// side; empty for a part without one.  A flag's text counts as its
// label.
// The box of the label of flag q, its text, out from its pin; 0 if it
// has none.
static int flag_label_box(const struct sch_part *q, double b[4])
{
	const char *flag = q->dev->shape->flag;
	struct sch_at at;
	double w = NAN;
	double h = FLAG_H;
	double ox = NAN;
	double oy = NAN;
	int out = 0;

	if (!q->text ||
	    (strcmp(flag, "supply") != 0 && strcmp(flag, "port") != 0 &&
	     strcmp(flag, "global") != 0)) {
		return 0;
	}
	at = sch_pin_at(q, 0);
	out = (at.dir + 2) % 4;
	w = (sch_text_width(q->text, SCH_TEXT) / SCH_GRID) + FLAG_PAD;
	ox = w / 2;
	if (out == SCH_L) {
		ox = w;
	} else if (out == SCH_R) {
		ox = 0;
	}
	oy = h / 2;
	if (out == SCH_D) {
		oy = h;
	} else if (out == SCH_U) {
		oy = 0;
	}
	b[0] = at.x + (way_dx[out] * FLAG_OUT) - ox;
	b[1] = at.y + (way_dy[out] * FLAG_OUT) - oy;
	b[2] = b[0] + w;
	b[3] = b[1] + h;
	return 1;
}

// The side of the label of q as its shape has it, turned with it.
static int own_side(const struct sch_part *q)
{
	int side = q->dev->shape->side;

	if (q->mirror && (side == 0 || side == 2)) {
		side = 2 - side;
	}
	return (side + q->rot) % 4;
}

static int label_box_at(const struct sch_part *q, int want, double b[4])
{
	struct sch_dims d = { NAN, NAN };
	double w = NAN;
	double h = NAN;
	double bx[4];
	double g = SCH_GRID;
	int side = want;

	if (q->dev->shape->flag) {
		return flag_label_box(q, b);
	}
	d = sch_label_size(q);
	w = d.w / g;
	h = d.h / g;
	sch_box(q, bx);
	if (side < 0) {
		side = own_side(q);
	}
	if (side == 0 || side == 2) {
		b[0] = side == 0 ? bx[2] + LABEL_GAP_X :
				   bx[0] - LABEL_GAP_X - w;
		b[1] = q->y - (h / 2);
	} else {
		b[0] = q->x - (w / 2);
		b[1] = side == 1 ? bx[3] + LABEL_GAP_Y :
				   bx[1] - LABEL_GAP_Y - h;
	}
	b[2] = b[0] + w;
	b[3] = b[1] + h;
	return 1;
}

static int label_box(const struct sch_part *q, double b[4])
{
	return label_box_at(q, q->side, b);
}

// A label tried: on the side, among the parts placed but skip.
struct label_try {
	int side;
	long skip;
};

// Whether the label of q, as lt tries it, keeps clear of the parts
// placed (but skip) and their labels.
static int label_clear(struct reader *rd, const struct sch_part *q,
		       const struct label_try *lt)
{
	double la[4];
	long skip = lt->skip;

	if (!label_box_at(q, lt->side, la)) {
		return 1;
	}
	for (size_t i = 0; i < rd->s->npart; i++) {
		const struct sch_part *o = &rd->s->part[i];
		double b[4];
		double lb[4];

		if (!rd->placed[i] || (long)i == skip || o == q) {
			continue;
		}
		sch_box(o, b);
		if (overlap(la, b) || (label_box(o, lb) && overlap(la, lb))) {
			return 0;
		}
		for (size_t p = 0; p < o->dev->shape->npin; p++) {
			int x = 0;
			int y = 0;

			{
				struct sch_at pin_at = sch_pin_at(o, p);
				x = pin_at.x;
				y = pin_at.y;
			}
			if (in_box(la, x, y)) {
				return 0;
			}
		}
	}
	return 1;
}

// The side the label of q takes: its own if it is clear, else the
// first clear of the other three, or -1 if none is.
static int label_side(struct reader *rd, const struct sch_part *q, long skip)
{
	int own = 0;

	if (q->side >= 0 || q->dev->shape->flag) {
		return q->side;
	}
	own = own_side(q);
	for (int i = 0; i < 4; i++) {
		struct label_try lt = { (own + i - 1) % 4, skip };

		if (i == 0) {
			lt.side = own;
		} else if (i == 1) {
			lt.side = (own + 2) % 4;
		}
		if (label_clear(rd, q, &lt)) {
			return lt.side;
		}
	}
	return -2;
}

// The boxes that part q keeps clear of a part placed: its own a, the
// part's b, and the part's label lb if it has one.
struct clear_boxes {
	double a[4];
	double b[4];
	double lb[4];
	int has_lb;
};

// Whether the pins of part q keep clear of the pin at of a part placed:
// none on it, in its body or label, or on the point just out from it,
// nor it on theirs; and its own point out of the box of q.
static int pins_clear(const struct sch_part *q, struct sch_at at,
		      const struct clear_boxes *cb)
{
	int x = at.x;
	int y = at.y;
	int d = at.dir;

	if (in_box(cb->a, x, y)) {
		return 0;
	}
	for (size_t r = 0; r < q->dev->shape->npin; r++) {
		struct sch_at pin_at = sch_pin_at(q, r);
		int x2 = pin_at.x;
		int y2 = pin_at.y;
		int d2 = pin_at.dir;

		// A pin keeps the point out from it free, for its wire or
		// its flag.
		if ((x2 == x && y2 == y) || in_box(cb->b, x2, y2) ||
		    (cb->has_lb && in_box(cb->lb, x2, y2)) ||
		    (x2 + way_dx[d2] == x && y2 + way_dy[d2] == y) ||
		    (x + way_dx[d] == x2 && y + way_dy[d] == y2) ||
		    (x + way_dx[d] == x2 + way_dx[d2] &&
		     y + way_dy[d] == y2 + way_dy[d2]) ||
		    in_box(cb->b, x2 + way_dx[d2], y2 + way_dy[d2]) ||
		    in_box(cb->a, x + way_dx[d], y + way_dy[d])) {
			return 0;
		}
	}
	return 1;
}

// The box of part q, CLEAR_PAD out, into b.
static void padded_box(const struct sch_part *q, double b[4])
{
	sch_box(q, b);
	b[0] -= CLEAR_PAD;
	b[1] -= CLEAR_PAD;
	b[2] += CLEAR_PAD;
	b[3] += CLEAR_PAD;
}

// Whether part q, where it stands, keeps clear of the parts placed, and
// of skip: no body or label on a body, a pin or a label, and no pin on a
// pin or on the point just out from one.
static int clear(struct reader *rd, const struct sch_part *q, long skip)
{
	struct clear_boxes cb;

	if (label_side(rd, q, skip) == -2) {
		return 0;
	}
	padded_box(q, cb.a);
	for (size_t i = 0; i < rd->s->npart; i++) {
		const struct sch_part *o = &rd->s->part[i];

		if (!rd->placed[i] || (long)i == skip) {
			continue;
		}
		padded_box(o, cb.b);
		cb.has_lb = label_box(o, cb.lb);
		if (overlap(cb.a, cb.b) ||
		    (cb.has_lb && overlap(cb.a, cb.lb))) {
			return 0;
		}
		for (size_t p = 0; p < o->dev->shape->npin; p++) {
			if (!pins_clear(q, sch_pin_at(o, p), &cb)) {
				return 0;
			}
		}
	}
	return 1;
}

// Where the ground (or chassis) flag of a pin stands: one unit under a
// pin that points down, one out and one down from a pin that points
// aside.
static void ground_spot(struct reader *rd, size_t pid, int *fx, int *fy)
{
	int x = 0;
	int y = 0;
	int d = 0;

	{
		struct sch_at pin_at = sch_pin_at(
			&rd->s->part[rd->pins[pid].part], rd->pins[pid].pin);
		x = pin_at.x;
		y = pin_at.y;
		d = pin_at.dir;
	}
	if (d == SCH_U) {
		at_line(rd, rd->pins[pid].line);
		fault(rd->faults, "%s: a %s on a pin that points up", rd->where,
		      rd->pins[pid].flag->flag);
	}
	*fx = x + (d == SCH_D ? 0 : way_dx[d]);
	*fy = y - 1;
}

// Whether part k stands clear where it is, with the ground flags of its
// pins where they would go.
static int clear_unit(struct reader *rd, size_t k)
{
	const struct sch_part *q = &rd->s->part[k];

	if (!clear(rd, q, -1)) {
		return 0;
	}
	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		const struct pinrec *r = &rd->pins[rd->base[k] + p];
		struct sch_part *f = NULL;

		if (r->flagpart < 0 ||
		    (strcmp(r->flag->flag, "ground") != 0 &&
		     strcmp(r->flag->flag, "chassis") != 0)) {
			continue;
		}
		f = &rd->s->part[r->flagpart];
		ground_spot(rd, rd->base[k] + p, &f->x, &f->y);
		if (!clear(rd, f, (long)k)) {
			return 0;
		}
	}
	return 1;
}

// Marks part k placed, and its ground flags with it.

static void trace(struct reader *rd, size_t k, const char *text)
{
	rd->trace_part =
		grow(rd->trace_part, rd->ntraces, sizeof *rd->trace_part);
	rd->trace_text = (char **)grow((void *)rd->trace_text, rd->ntraces,
				       sizeof *rd->trace_text);
	rd->trace_part[rd->ntraces] = k;
	rd->trace_text[rd->ntraces++] = sch_strdup(text);
}

// Tells each placement, by the part's name and designator.
static void tell_traces(struct reader *rd)
{
	for (size_t i = 0; i < rd->ntraces; i++) {
		const struct sch_part *q = &rd->s->part[rd->trace_part[i]];

		(void)fprintf(stderr, "%s:%d: %s (%s) %s\n", rd->s->file,
			      q->line, q->name, q->ref ? q->ref : "-",
			      rd->trace_text[i]);
		free(rd->trace_text[i]);
	}
	free((void *)rd->trace_text);
	free(rd->trace_part);
	rd->ntraces = 0;
}

static void commit(struct reader *rd, size_t k)
{
	int side = label_side(rd, &rd->s->part[k], -1);
	static const char *const how[] = {
		"first clear place to the right",
		"first clear place along the pin", "beside its twin",
		"next to its anchor", "past what stands between the pins"
	};

	if (rd->lib->verbose) {
		char t[TRACE_LEN];

		(void)snprintf(t, sizeof t, "at %d %d, turned %d: %s",
			       rd->s->part[k].x, rd->s->part[k].y,
			       rd->s->part[k].rot * RIGHT_ANGLE,
			       how[rd->places[k].kind]);
		trace(rd, k, t);
	}
	rd->s->part[k].side = side >= 0 ? side : rd->s->part[k].side;
	rd->placed[k] = 1;
	rd->s->part[k].placed = 1;
	for (size_t p = 0; p < rd->s->part[k].dev->shape->npin; p++) {
		const struct pinrec *r = &rd->pins[rd->base[k] + p];

		if (r->flagpart >= 0 && (strcmp(r->flag->flag, "ground") == 0 ||
					 !strcmp(r->flag->flag, "chassis"))) {
			ground_spot(rd, rd->base[k] + p,
				    &rd->s->part[r->flagpart].x,
				    &rd->s->part[r->flagpart].y);
			rd->placed[r->flagpart] = 1;
			rd->s->part[r->flagpart].placed = 1;
		}
	}
}

// The way a pin faces to take a lead going dir.
static const char *facing_name(int dir)
{
	switch (dir) {
	case SCH_R:
		return "left";
	case SCH_L:
		return "right";
	case SCH_U:
		return "down";
	default:
		return "up";
	}
}

// The pin of part q that faces back along dir: for two pins, the first,
// with the part turned to suit.
static size_t entry_pin(struct reader *rd, struct sch_part *q, int dir)
{
	const struct sch_shape *sh = q->dev->shape;
	int back = (dir + 2) % 4;
	int ed = 0;

	if (sh->npin == 2) {
		q->rot = (back - sh->pin[0].dir + 4) % 4;
		return 0;
	}
	q->x = q->y = 0;
	for (size_t p = 0; p < sh->npin; p++) {
		{
			struct sch_at pin_at = sch_pin_at(q, p);
			ed = pin_at.dir;
		}
		if (ed == back) {
			return p;
		}
	}
	fault(rd->faults, "%s: %s has no pin facing %s", rd->where, q->name,
	      facing_name(dir));
	return (size_t)-1;
}

// The pins on the net of pin pid past the two a line joins, up to 3:
// room for their junctions.
static int extra_pins(struct reader *rd, size_t pid)
{
	int n = 0;

	for (size_t i = 0; i < rd->npins; i++) {
		n += rd->pins[i].used && !rd->pins[i].flag &&
		     !rd->s->part[rd->pins[i].part].dev->shape->flag &&
		     root(rd, i) == root(rd, pid);
	}
	if (n - 2 < 0) {
		return 0;
	}
	return n - 2 > 3 ? 3 : n - 2;
}

// Places part k, dir from pin pid: past the branches of the pin if dir
// is the pin's own way, else as the next branch on its lead.
static void grow_box(double b[4], const double c[4])
{
	b[0] = fmin(b[0], c[0]);
	b[1] = fmin(b[1], c[1]);
	b[2] = fmax(b[2], c[2]);
	b[3] = fmax(b[3], c[3]);
}

// The lead of a part placed: the point it comes from, and whether the
// part hangs as a branch from it.
struct lead {
	int x, y;
	int branch;
};

// The room a part keeps from the loops: that of a body, and for a
// branch, as thick as the thickest of two pins too.
static double loop_margin(const struct reader *rd, int branch)
{
	double m = rd->s->clear_body / SCH_GRID;

	for (size_t i = 0; branch && i < rd->s->npart; i++) {
		const struct sch_shape *s = rd->s->part[i].dev->shape;

		if (s->npin == 2 && !s->flag) {
			m = fmax(m, (rd->s->clear_body / SCH_GRID) +
					    (fmin(s->box[2] - s->box[0],
						  s->box[3] - s->box[1]) /
					     2));
		}
	}
	return m;
}

// Whether a pin of part k is on net.
static int part_on_net(struct reader *rd, size_t k, size_t net)
{
	const struct sch_part *q = &rd->s->part[k];

	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		if (root(rd, rd->base[k] + p) == net) {
			return 1;
		}
	}
	return 0;
}

// The box of the loop of hint h, of net, into b: round its two pins, one
// unit out from each, and round the parts placed of its net and their
// labels.
static void loop_box(struct reader *rd, const struct sch_hint *h, size_t net,
		     double b[4])
{
	b[0] = b[1] = EMPTY_BOX;
	b[2] = b[3] = -EMPTY_BOX;
	for (int e = 0; e < 2; e++) {
		struct sch_at pin_at =
			sch_pin_at(&rd->s->part[h->part[e]], h->pin[e]);
		int x = pin_at.x;
		int y = pin_at.y;
		int d = pin_at.dir;

		b[0] = fmin(b[0], fmin(x, x + way_dx[d]));
		b[1] = fmin(b[1], fmin(y, y + way_dy[d]));
		b[2] = fmax(b[2], fmax(x, x + way_dx[d]));
		b[3] = fmax(b[3], fmax(y, y + way_dy[d]));
	}
	// The loop takes in the parts of its net, and their labels.
	for (size_t j = 0; j < rd->s->npart; j++) {
		const struct sch_part *o = &rd->s->part[j];
		double c[4];

		if (!rd->placed[j] || !part_on_net(rd, j, net) ||
		    o->dev->shape->flag) {
			continue;
		}
		sch_box(o, c);
		grow_box(b, c);
		if (label_box(o, c)) {
			grow_box(b, c);
		}
	}
}

// Whether part k, standing with its lead ld, keeps out of the loops by
// the room of a body (a branch, by more): each wire said in the source
// closes a loop through the parts it joins, and the box round its two
// pins, one unit out from each, and round the parts of its net and
// their labels, is kept for that net.  A part joined to the net of the
// wire may stand in it; a wire of another net may cross it.
static int out_of_loops(struct reader *rd, size_t k, struct lead ld)
{
	const struct sch_part *q = &rd->s->part[k];
	double a[4];
	double m = loop_margin(rd, ld.branch);

	sch_box(q, a);
	a[0] = fmin(a[0], ld.x) - m;
	a[1] = fmin(a[1], ld.y) - m;
	a[2] = fmax(a[2], ld.x) + m;
	a[3] = fmax(a[3], ld.y) + m;
	for (size_t i = 0; i < rd->s->nhint; i++) {
		const struct sch_hint *h = &rd->s->hint[i];
		size_t net = root(rd, rd->base[h->part[0]] + h->pin[0]);
		double b[4];

		if (!rd->placed[h->part[0]] || !rd->placed[h->part[1]] ||
		    part_on_net(rd, k, net)) {
			continue;
		}
		loop_box(rd, h, net, b);
		if (overlap(a, b)) {
			return 0;
		}
	}
	return 1;
}

static int place_from(struct reader *rd, size_t k)
{
	struct sch_part *q = &rd->s->part[k];
	const struct place *pl = &rd->places[k];
	struct pinrec *a = &rd->pins[pl->pid[0]];
	int px = 0;
	int py = 0;
	int pd = 0;
	int ex = 0;
	int ey = 0;
	int dir = pl->dir;
	int start = HANG;
	size_t entry = entry_pin(rd, q, dir);

	if (entry == (size_t)-1) {
		return 0;
	}
	{
		struct sch_at pin_at =
			sch_pin_at(&rd->s->part[a->part], a->pin);
		px = pin_at.x;
		py = pin_at.y;
		pd = pin_at.dir;
	}
	if (dir == (pd + 2) % 4) {
		fault(rd->faults,
		      "%s: a pin can lead only its own way or across",
		      rd->where);
		return 0;
	}
	if (dir == pd && a->series) {
		fault(rd->faults, "%s: the line past %s goes on already",
		      rd->where, rd->s->part[a->part].name);
		return 0;
	}
	q->x = q->y = 0;
	{
		struct sch_at pin_at = sch_pin_at(q, entry);
		ex = pin_at.x;
		ey = pin_at.y;
	}
	if (a->reach) {
		start = a->reach + STEP;
	} else if (dir == pd) {
		start = FIRST + extra_pins(rd, pl->pid[0]);
	}
	for (int off = start;; off++) {
		int tx = 0;
		int ty = 0;
		struct lead ld = { px + (way_dx[pd] * off),
				   py + (way_dy[pd] * off), dir != pd };

		if (off > a->reach + FAR) {
			fault(rd->faults, "%s: no room for %s", rd->where,
			      q->name);
			return 0;
		}
		if (dir == pd) {
			tx = px + way_dx[dir] * off;
			ty = py + way_dy[dir] * off;
		} else {
			tx = px + way_dx[pd] * off + way_dx[dir] * FIRST;
			ty = py + way_dy[pd] * off + way_dy[dir] * FIRST;
		}
		q->x = tx - ex;
		q->y = ty - ey;
		if (out_of_loops(rd, k, ld) && clear_unit(rd, k)) {
			a->reach = off;
			a->series = dir == pd;
			break;
		}
	}
	rd->anchor_of[k] = (long)pl->pid[0];
	rd->along_of[k] = a->reach;
	return 1;
}

// Places part k in parallel with another of its shape, turned as it is,
// each pin joined to the pin of the same name.
static int place_beside(struct reader *rd, size_t k)
{
	struct sch_part *q = &rd->s->part[k];
	size_t b = rd->places[k].part;
	const struct sch_part *o = &rd->s->part[b];
	int y0 = 0;
	int y1 = 0;
	int dir = rd->places[k].dir;
	int off = 0;

	int any = 0;

	{
		struct sch_at pin_at = sch_pin_at(o, 0);
		y0 = pin_at.y;
	}
	{
		struct sch_at pin_at = sch_pin_at(o, o->dev->shape->npin - 1);
		y1 = pin_at.y;
	}
	any = dir < 0;
	if (any) {
		dir = y0 == y1 ? SCH_D : SCH_R;
	}
	q->rot = o->rot;
	q->mirror = o->mirror;
	q->flip = o->flip;
	// The first clear place out the way said, or either way across.
	for (off = BESIDE_OFF * (rd->besides[b] + 1);; off++) {
		int found = 0;

		if (off > FAR) {
			fault(rd->faults, "%s: no room for %s", rd->where,
			      q->name);
			return 0;
		}
		// Its own side up to three units on, then either.
		for (int t = 0;
		     t < 1 + (any &&
			      off > BESIDE_OFF * (rd->besides[b] + 1) + 3) &&
		     !found;
		     t++) {
			int w = t ? (dir + 2) % 4 : dir;

			q->x = o->x + way_dx[w] * off;
			q->y = o->y + way_dy[w] * off;
			if (clear_unit(rd, k)) {
				dir = w;
				found = 1;
			}
		}
		if (found) {
			rd->besides[b] = off / BESIDE_OFF;
			break;
		}
	}
	// A branch beside a branch takes room on the lead it hangs from.
	if (rd->anchor_of[b] >= 0) {
		struct pinrec *a = &rd->pins[rd->anchor_of[b]];
		int ad = 0;

		{
			struct sch_at pin_at =
				sch_pin_at(&rd->s->part[a->part], a->pin);
			ad = pin_at.dir;
		}
		if (dir == ad && rd->along_of[b] + off > a->reach) {
			a->reach = rd->along_of[b] + off;
		}
		rd->anchor_of[k] = rd->anchor_of[b];
		rd->along_of[k] = rd->along_of[b] + (dir == ad ? off : 0);
	}
	return 1;
}

// Places part k next to another without joining them: past its box the
// way said.
static int up_int(double v);
static int down_int(double v);

// The group of the name, from the scope out, as its path and a dot, if
// some part is in it.
static char *group_of(struct reader *rd, const char *name, const char *scope)
{
	size_t n = strlen(scope);
	size_t size = n + strlen(name) + 3;
	char *g = sch_alloc(size);

	for (;;) {
		(void)snprintf(g, size, "%.*s%s%s.", (int)n, scope,
			       n ? "." : "", name);
		for (size_t i = 0; i < rd->s->npart; i++) {
			if (!strncmp(rd->s->part[i].name, g, strlen(g))) {
				return g;
			}
		}
		if (!n) {
			free(g);
			return NULL;
		}
		while (n && scope[n - 1] != '.') {
			n--;
		}
		n = n ? n - 1 : 0;
	}
}

// Places part k next to a part, or next to a group: past the box of all
// the group has placed (its labels too), as the part of the same name in
// that group stands in it, so a group copied from another stands as a
// whole beside it.
// The box of all that the group of pl has placed (its labels too), into
// b, empty if none; returns the part of the group of the same name as
// part q in its group, or NULL.
static const struct sch_part *group_box(struct reader *rd,
					const struct place *pl,
					const struct sch_part *q, double b[4])
{
	// The name past as many groups as the group's path holds.
	const char *rest = q->name;
	size_t n = strlen(pl->group) + strlen(q->name) + 1;
	char *same = sch_alloc(n);
	const struct sch_part *o = NULL;
	double c[4];

	b[0] = b[1] = EMPTY_BOX;
	b[2] = b[3] = -EMPTY_BOX;
	for (const char *g = pl->group; *g && rest; g++) {
		if (*g == '.') {
			rest = strchr(rest, '.');
			rest = rest ? rest + 1 : NULL;
		}
	}
	(void)snprintf(same, n, "%s%s", pl->group, rest ? rest : q->name);
	for (size_t i = 0; i < rd->s->npart; i++) {
		if (rd->placed[i] && !strncmp(rd->s->part[i].name, pl->group,
					      strlen(pl->group))) {
			sch_box(&rd->s->part[i], c);
			grow_box(b, c);
			if (label_box(&rd->s->part[i], c)) {
				grow_box(b, c);
			}
			if (strcmp(rd->s->part[i].name, same) == 0) {
				o = &rd->s->part[i];
			}
		}
	}
	free(same);
	return o;
}

// Where a part placed next to the box b stands, off past it the way
// dir: for a group, by how far the group reaches beyond its part at o.
struct of_place {
	int dir;
	int group;
	int ox, oy;
};

static void place_past(struct sch_part *q, const struct of_place *op,
		       const double b[4], int off)
{
	int dx = 0;
	int dy = 0;

	q->x = op->ox;
	q->y = op->oy;
	switch (op->dir) {
	case SCH_R:
		dx = op->group ? op->ox - down_int(b[0]) : 0;
		q->x = up_int(b[2]) + off + dx;
		break;
	case SCH_L:
		dx = op->group ? up_int(b[2]) - op->ox : 0;
		q->x = down_int(b[0]) - off - dx;
		break;
	case SCH_U:
		dy = op->group ? op->oy - down_int(b[1]) : 0;
		q->y = up_int(b[3]) + off + dy;
		break;
	case SCH_D:
		dy = op->group ? up_int(b[3]) - op->oy : 0;
		q->y = down_int(b[1]) - off - dy;
		break;
	default:
		break;
	}
}

// The gap from the box b to the box c past it the way dir.
static double gap_past(int dir, const double b[4], const double c[4])
{
	switch (dir) {
	case SCH_R:
		return c[0] - b[2];
	case SCH_L:
		return b[0] - c[2];
	case SCH_U:
		return c[1] - b[3];
	default:
		return b[1] - c[3];
	}
}

static int place_of(struct reader *rd, size_t k)
{
	struct sch_part *q = &rd->s->part[k];
	const struct place *pl = &rd->places[k];
	double b[4];
	double c[4];
	const struct sch_part *o = NULL;
	struct of_place op = { pl->dir, pl->group != NULL, 0, 0 };

	if (pl->group) {
		o = group_box(rd, pl, q, b);
		if (b[2] < b[0]) {
			fault(rd->faults, "%s: no part of %s is placed",
			      rd->where, pl->ref[0]);
			return 0;
		}
	} else {
		o = &rd->s->part[pl->part];
		sch_box(o, b);
	}
	op.ox = o ? o->x : (int)((b[0] + b[2]) / 2);
	op.oy = o ? o->y : (int)((b[1] + b[3]) / 2);
	for (int off = OF_GAP;; off++) {
		if (off > FAR) {
			fault(rd->faults, "%s: no room for %s", rd->where,
			      q->name);
			return 0;
		}
		// Past the box, by how far the group reaches beyond its part.
		place_past(q, &op, b, off);
		if (pl->gap >= 0) {
			sch_box(q, c);
			if (gap_past(op.dir, b, c) < pl->gap - GAP_SLACK) {
				continue;
			}
		}
		if (clear_unit(rd, k)) {
			return 1;
		}
	}
}

// The whole number at or over, and at or under, v.
static int up_int(double v)
{
	double c = ceil(v);

	return (int)c;
}

static int down_int(double v)
{
	double f = floor(v);

	return (int)f;
}

// Places a two-pin part k between two pins placed, lying along the line
// from one to the other and out the way said past all that stands
// between them; its pins join theirs, the nearer to the first.
// The span of a part placed between two pins: the way out from the
// line, the way along it, and the reach of the pins along it; and the
// pins.
struct between {
	int dir;
	int along;
	double lo, hi;
	int x[2], y[2];
};

// The span of the part placed by pl between two pins.
static struct between between_span(const struct reader *rd,
				   const struct place *pl)
{
	struct between bt = { pl->dir, SCH_U, NAN, NAN, { 0, 0 }, { 0, 0 } };
	const int *x = bt.x;
	const int *y = bt.y;

	for (int i = 0; i < 2; i++) {
		struct sch_at pin_at =
			sch_pin_at(&rd->s->part[rd->pins[pl->pid[i]].part],
				   rd->pins[pl->pid[i]].pin);

		bt.x[i] = pin_at.x;
		bt.y[i] = pin_at.y;
	}
	if (pl->dir == SCH_U || pl->dir == SCH_D) {
		bt.along = SCH_R;
		bt.lo = x[0] < x[1] ? x[0] : x[1];
		bt.hi = x[0] > x[1] ? x[0] : x[1];
	} else {
		bt.lo = y[0] < y[1] ? y[0] : y[1];
		bt.hi = y[0] > y[1] ? y[0] : y[1];
	}
	return bt;
}

// Moves *edge out the way dir to t, if t is further out.
static void further(int dir, double *edge, double t)
{
	if (dir == SCH_U || dir == SCH_R) {
		*edge = t > *edge ? t : *edge;
	} else {
		*edge = t < *edge ? t : *edge;
	}
}

// The side of the box e that faces the way dir.
static double box_side(int dir, const double e[4])
{
	switch (dir) {
	case SCH_U:
		return e[3];
	case SCH_D:
		return e[1];
	case SCH_R:
		return e[2];
	default:
		return e[0];
	}
}

// The farthest out of part o, and its label, that lies across the span
// bt, past edge.
static double part_edge(const struct sch_part *o, const struct between *bt,
			double edge)
{
	double b[4];
	double l[4];
	int has = 0;

	sch_box(o, b);
	has = label_box(o, l);
	for (int pass = 0; pass < 1 + has; pass++) {
		const double *e = pass ? l : b;
		double s0 = bt->along == SCH_R ? e[0] : e[1];
		double s1 = bt->along == SCH_R ? e[2] : e[3];

		if (s1 >= bt->lo - 1 && s0 <= bt->hi + 1) {
			further(bt->dir, &edge, box_side(bt->dir, e));
		}
	}
	for (size_t p = 0; p < o->dev->shape->npin; p++) {
		struct sch_at pin_at = sch_pin_at(o, p);
		double s = bt->along == SCH_R ? pin_at.x : pin_at.y;
		double t = bt->along == SCH_R ? pin_at.y : pin_at.x;

		if (s >= bt->lo - 1 && s <= bt->hi + 1) {
			further(bt->dir, &edge, t);
		}
	}
	return edge;
}

static int place_between(struct reader *rd, size_t k)
{
	struct sch_part *q = &rd->s->part[k];
	const struct place *pl = &rd->places[k];
	struct between bt = { 0, 0, NAN, NAN, { 0, 0 }, { 0, 0 } };
	int dir = pl->dir;
	double edge = dir == SCH_U || dir == SCH_R ? -BIG : BIG;
	size_t first = 0;

	if (q->dev->shape->npin != 2) {
		fault(rd->faults, "%s: only a part of two pins goes between",
		      rd->where);
		return 0;
	}
	bt = between_span(rd, pl);
	// The farthest out of what lies across the span, its labels too.
	for (size_t i = 0; i < rd->s->npart; i++) {
		if (rd->placed[i]) {
			edge = part_edge(&rd->s->part[i], &bt, edge);
		}
	}
	if (bt.along == SCH_R) {
		first = bt.x[0] <= bt.x[1] ? 0 : 1;
		q->rot = 1; // Pin 1 left,
	} else {
		first = bt.y[0] <= bt.y[1] ? 0 : 1;
		q->rot = 0; // or on top.
	}
	for (int off = 2;; off++) {
		int c = (int)((bt.lo + bt.hi) / 2);
		int e = 0;

		if (off > FAR) {
			fault(rd->faults, "%s: no room for %s", rd->where,
			      q->name);
			return 0;
		}
		e = dir == SCH_U || dir == SCH_R ? up_int(edge) + off :
						   down_int(edge) - off;
		q->x = bt.along == SCH_R ? c : e;
		q->y = bt.along == SCH_R ? e : c;
		if (clear_unit(rd, k)) {
			break;
		}
	}
	// Pin 1 joins the first pin; the part is turned to have it nearer.
	if (first) {
		q->rot = (q->rot + 2) % 4;
	}
	return 1;
}

// The right edge of what is placed, for a part placed against nothing.
static int right_edge(struct reader *rd)
{
	double x1 = -BIG;

	for (size_t i = 0; i < rd->s->npart; i++) {
		double b[4];

		if (rd->placed[i]) {
			sch_box(&rd->s->part[i], b);
			x1 = b[2] > x1 ? b[2] : x1;
		}
	}
	return up_int(x1);
}

// The order the placements wait in: branches and parallels first, then
// what continues a line, then parts placed next to others, parts placed
// between, and last parts placed against nothing.
static int rank(struct reader *rd, size_t k)
{
	const struct place *pl = &rd->places[k];

	if (pl->kind == FROM) {
		int d = 0;
		const struct pinrec *a = &rd->pins[pl->pid[0]];

		{
			struct sch_at pin_at =
				sch_pin_at(&rd->s->part[a->part], a->pin);
			d = pin_at.dir;
		}
		return pl->dir == d ? 1 : 0;
	}
	switch (pl->kind) {
	case BESIDE:
		return 0;
	case OF:
		return 2;
	case BETWEEN:
		return 3;
	default:
		return 4;
	}
}

static int ready(struct reader *rd, size_t k)
{
	const struct place *pl = &rd->places[k];

	switch (pl->kind) {
	case FROM:
		return rd->placed[rd->pins[pl->pid[0]].part];
	case OF:
		for (size_t i = 0; pl->group && i < rd->s->npart; i++) {
			if (i != k && !rd->placed[i] && !rd->dead[i] &&
			    !strncmp(rd->s->part[i].name, pl->group,
				     strlen(pl->group))) {
				return 0;
			}
		}
		return pl->group || rd->placed[pl->part];
	case BESIDE:
		return rd->placed[pl->part];
	case BETWEEN:
		return rd->placed[rd->pins[pl->pid[0]].part] &&
		       rd->placed[rd->pins[pl->pid[1]].part];
	default:
		return 1;
	}
}

// The best ranked part ready to place, of those placed between pins if
// between is set, else of the others; or -1.
static long best_ready(struct reader *rd, int between)
{
	long best = -1;

	for (size_t k = 0; k < rd->s->npart; k++) {
		if (rd->placed[k] || rd->dead[k] ||
		    rd->s->part[k].dev->shape->flag || !ready(rd, k) ||
		    (rd->places[k].kind == BETWEEN) != between) {
			continue;
		}
		if (best < 0 || rank(rd, k) < rank(rd, (size_t)best)) {
			best = (long)k;
		}
	}
	return best;
}

// Places part k against nothing: at the first clear place right of all
// that is placed.
static void place_root(struct reader *rd, size_t k)
{
	struct sch_part *q = &rd->s->part[k];
	int any = 0;

	for (size_t i = 0; i < rd->s->npart; i++) {
		any = any || rd->placed[i];
	}
	q->x = any ? right_edge(rd) + ROOT_GAP : 0;
	q->y = 0;
	while (!clear_unit(rd, k)) {
		q->x++;
	}
}

// Places the parts that are ready, the best ranked first, but not
// those placed between pins unless between is set.
static void place_ready(struct reader *rd, int between)
{
	for (;;) {
		long best = best_ready(rd, between);
		int ok = 1;

		if (best < 0) {
			return;
		}
		at_line(rd, rd->places[best].line);
		switch (rd->places[best].kind) {
		case FROM:
			ok = place_from(rd, (size_t)best);
			break;
		case BESIDE:
			ok = place_beside(rd, (size_t)best);
			break;
		case OF:
			ok = place_of(rd, (size_t)best);
			break;
		case BETWEEN:
			ok = place_between(rd, (size_t)best);
			break;
		default:
			place_root(rd, (size_t)best);
			break;
		}
		if (ok) {
			commit(rd, (size_t)best);
		} else {
			rd->dead[best] =
				1; // Told, and left out of the drawing.
		}
	}
}

// Places the supplies and the ports of the sheet, past the branches of
// their pins, on the parts placed so far.
static void place_flags(struct reader *rd)
{
	for (size_t i = 0; i < rd->npins; i++) {
		struct pinrec *r = &rd->pins[i];
		struct sch_part *f = NULL;
		int x = 0;
		int y = 0;
		int d = 0;
		int off = 0;

		if (r->flagpart < 0 || rd->placed[r->flagpart] ||
		    !rd->placed[r->part]) {
			continue;
		}
		f = &rd->s->part[r->flagpart];
		{
			struct sch_at pin_at =
				sch_pin_at(&rd->s->part[r->part], r->pin);
			x = pin_at.x;
			y = pin_at.y;
			d = pin_at.dir;
		}
		f->rot = ((d + 2) % 4 - f->dev->shape->pin[0].dir + 4) % 4;
		off = 1;
		if (r->reach) {
			off = r->reach + STEP;
		} else if (!strcmp(r->flag->flag, "port")) {
			off = 3;
		}
		for (;; off++) {
			f->x = x + way_dx[d] * off;
			f->y = y + way_dy[d] * off;
			if (clear(rd, f, (long)r->part) ||
			    off > r->reach + FAR) {
				break;
			}
		}
		rd->placed[r->flagpart] = 1;
		f->placed = 1;
	}
}

// Places all: the parts against others, the flags at the ends of
// lines, then the parts between pins, which keep out past them, and the
// flags of those.
// Moves part k, and the flags of its pins, by (dx, dy).
static void move_unit(struct reader *rd, size_t k, int dx, int dy)
{
	rd->s->part[k].x += dx;
	rd->s->part[k].y += dy;
	for (size_t p = 0; p < rd->s->part[k].dev->shape->npin; p++) {
		long f = rd->pins[rd->base[k] + p].flagpart;

		if (f >= 0) {
			rd->s->part[f].x += dx;
			rd->s->part[f].y += dy;
		}
	}
}

// Whether part k, placed and then moved, stands clear of the others,
// its flags too: as clear_unit, the part and its flags out of the
// placed while they are tried.
static int clear_moved(struct reader *rd, size_t k)
{
	int ok = 0;

	rd->placed[k] = 0;
	for (size_t p = 0; p < rd->s->part[k].dev->shape->npin; p++) {
		long f = rd->pins[rd->base[k] + p].flagpart;

		if (f >= 0) {
			rd->placed[f] = 0;
		}
	}
	ok = clear_unit(rd, k);
	rd->placed[k] = 1;
	for (size_t p = 0; p < rd->s->part[k].dev->shape->npin; p++) {
		long f = rd->pins[rd->base[k] + p].flagpart;

		if (f >= 0) {
			rd->placed[f] = 1;
		}
	}
	return ok;
}

// Lays each part and the parts beside it as an even row in parallel
// about the line the part was placed on: PAIR_GAP apart, the row's
// middle on the line, so no one of two stands on it; and joins their
// pins of a kind by wires, the bars, which the line meets in their
// middle.  If the row is not clear, it stays as it was placed.
// A row of parts in parallel: the parts, their number, and whether the
// row runs along x.
struct row {
	size_t *k;
	size_t n;
	int across;
};

// Sorts the row in the order its parts stand, from low to high across
// the line.
static void sort_row(const struct reader *rd, struct row *rw)
{
	size_t *row = rw->k;

	for (size_t i = 1; i < rw->n; i++) {
		for (size_t j = i; j > 0; j--) {
			const struct sch_part *p = &rd->s->part[row[j - 1]];
			const struct sch_part *q = &rd->s->part[row[j]];
			size_t t = row[j - 1];

			if ((rw->across ? q->x - p->x : q->y - p->y) >= 0) {
				break;
			}
			row[j - 1] = row[j];
			row[j] = t;
		}
	}
}

// Moves the parts of the row to stand PAIR_GAP apart about line, and
// back if they are not clear there; returns whether they are.
static int lay_row(struct reader *rd, const struct row *rw, int line)
{
	size_t n = rw->n;
	int *was = sch_alloc(2 * n * sizeof *was);
	int ok = 1;

	for (size_t i = 0; i < n; i++) {
		const struct sch_part *q = &rd->s->part[rw->k[i]];
		int to = line + ((2 * (int)i - (int)(n - 1)) * PAIR_GAP / 2);
		int d = to - (rw->across ? q->x : q->y);

		was[2 * i] = q->x;
		was[(2 * i) + 1] = q->y;
		move_unit(rd, rw->k[i], rw->across ? d : 0, rw->across ? 0 : d);
	}
	for (size_t i = 0; i < n && ok; i++) {
		ok = clear_moved(rd, rw->k[i]);
	}
	for (size_t i = 0; i < n && !ok; i++) {
		const struct sch_part *q = &rd->s->part[rw->k[i]];

		move_unit(rd, rw->k[i], was[2 * i] - q->x,
			  was[(2 * i) + 1] - q->y);
	}
	free(was);
	return ok;
}

// Joins the pins of a kind of the parts of the row, of the pins of part
// b, by the bars.
static void bar_hints(struct reader *rd, size_t b, const struct row *rw)
{
	for (size_t i = 0; i + 1 < rw->n; i++) {
		for (size_t p = 0; p < rd->s->part[b].dev->shape->npin; p++) {
			struct sch_hint h = { { rw->k[i], rw->k[i + 1] },
					      { p, p },
					      -1 };

			rd->s->hint = grow(rd->s->hint, rd->s->nhint,
					   sizeof *rd->s->hint);
			rd->s->hint[rd->s->nhint++] = h;
		}
	}
}

static void center_besides(struct reader *rd)
{
	size_t *row = sch_alloc((rd->s->npart + 1) * sizeof *row);

	for (size_t b = 0; b < rd->s->npart; b++) {
		struct row rw = { row, 1, 0 };
		int line = 0;

		row[0] = b;
		for (size_t k = 0; rd->placed[b] && k < rd->s->npart; k++) {
			if (rd->placed[k] && rd->places[k].kind == BESIDE &&
			    rd->places[k].part == b) {
				row[rw.n++] = k;
			}
		}
		if (rw.n < 2) {
			continue;
		}
		// Across the line: along x if the twin stands beside it so.
		rw.across = rd->s->part[row[1]].x != rd->s->part[b].x;
		sort_row(rd, &rw);
		line = rw.across ? rd->s->part[b].x : rd->s->part[b].y;
		if (lay_row(rd, &rw, line)) {
			bar_hints(rd, b, &rw);
			if (rd->lib->verbose) {
				trace(rd, b,
				      "its parts in parallel about its line");
			}
		}
	}
	free(row);
}

static void place_all(struct reader *rd)
{
	place_ready(rd, 0);
	place_flags(rd);
	place_ready(rd, 1);
	place_flags(rd);
	center_besides(rd);
	for (size_t k = 0; k < rd->s->npart; k++) {
		if (!rd->placed[k] && !rd->dead[k] &&
		    !rd->s->part[k].dev->shape->flag) {
			at_line(rd, rd->places[k].line);
			fault(rd->faults,
			      "%s: %s waits on parts that are never placed",
			      rd->where, rd->s->part[k].name);
		}
	}
}

// Reads a part statement: KIND NAME [VALUE] [rating R] [mpn M]
// [side WAY] [flip] [WAY from PIN | beside PART [WAY] | WAY of PART |
// between PIN and PIN WAY].
// Whether a value is as sch(5) has it for the kind: a resistor's, a
// capacitor's or an inductor's a number with its capital unit letter
// right after it (100R, 10.0K, 120P, 1.5U); any other kind's anything.
static int value_ok(const struct sch_shape *sh, const char *v)
{
	const char *units = NULL;
	size_t n = strspn(v, "0123456789.");

	if (!strcmp(sh->name, "resistor")) {
		units = "RKM";
	} else if (!strcmp(sh->name, "capacitor")) {
		units = "PNU";
	} else if (!strcmp(sh->name, "inductor")) {
		units = "NU";
	}

	return !units ||
	       (n && strspn(v, "0123456789") && v[n] && strchr(units, v[n]) &&
		!v[n + 1] && strchr(v, '.') == strrchr(v, '.'));
}

// The unit letters of a value of the kind of shape sh, and an example.
static const char *units_told(const struct sch_shape *sh)
{
	if (!strcmp(sh->name, "resistor")) {
		return "R, K or M: 100R";
	}
	if (!strcmp(sh->name, "capacitor")) {
		return "P, N or U: 120P";
	}
	return "N or U: 1.5U";
}

// Frees what a part statement read so far has made.
static void free_draft(struct sch_part *q, struct place *pl)
{
	free(q->name);
	free(q->value);
	free(q->rating);
	free(q->mpn);
	free(pl->scope);
	free(pl->ref[0]);
	free(pl->ref[1]);
}

// Whether word i of w[0] .. w[n-1] needs a word after it that it lacks.
static int lacks_word(char **w, int n, int i)
{
	return i + 1 >= n &&
	       (dir_word(w[i]) >= 0 || !strcmp(w[i], "rating") ||
		!strcmp(w[i], "mpn") || !strcmp(w[i], "side") ||
		!strcmp(w[i], "beside") || !strcmp(w[i], "between"));
}

// Reads the words of a placement of a part statement from w[*i] on,
// into pl: WAY from PIN, WAY of PART [N], beside PART [WAY] or between
// PIN and PIN WAY; *i is set to its last word.  Returns 1 if it is one,
// 0 if not, -1 on a fault.
static int read_placement(struct reader *rd, char **w, int n, int *i,
			  struct place *pl)
{
	int k = *i;
	int d = dir_word(w[k]);

	if (d >= 0 && k + 2 < n &&
	    (!strcmp(w[k + 1], "from") || !strcmp(w[k + 1], "of"))) {
		pl->kind = w[k + 1][0] == 'f' ? FROM : OF;
		pl->dir = d;
		free(pl->ref[0]);
		pl->ref[0] = sch_strdup(w[k + 2]);
		k += 2;
		// WAY of PART N: N grid steps between the boxes.
		if (pl->kind == OF && k + 1 < n &&
		    strspn(w[k + 1], "0123456789") == strlen(w[k + 1]) &&
		    integer(w[k + 1], &pl->gap)) {
			k++;
		}
	} else if (!strcmp(w[k], "beside")) {
		pl->kind = BESIDE;
		free(pl->ref[0]);
		pl->ref[0] = sch_strdup(w[++k]);
		if (k + 1 < n && dir_word(w[k + 1]) >= 0) {
			pl->dir = dir_word(w[++k]);
		}
	} else if (!strcmp(w[k], "between")) {
		if (k + 4 >= n || strcmp(w[k + 2], "and") != 0 ||
		    dir_word(w[k + 4]) < 0) {
			fault(rd->faults, "%s: between PIN and PIN WAY",
			      rd->where);
			return -1;
		}
		pl->kind = BETWEEN;
		free(pl->ref[0]);
		free(pl->ref[1]);
		pl->ref[0] = sch_strdup(w[k + 1]);
		pl->ref[1] = sch_strdup(w[k + 3]);
		pl->dir = dir_word(w[k + 4]);
		k += 4;
	} else {
		return 0;
	}
	*i = k;
	return 1;
}

// Reads the words of a part statement past its name, w[2] .. w[n-1],
// into q and pl, its value into *value; returns 0 on a fault.
static int read_part_words(struct reader *rd, char **w, int n,
			   struct sch_part *q, struct place *pl,
			   const char **value)
{
	for (int i = 2; i < n; i++) {
		int got = 0;

		if (!strcmp(w[i], "flip")) {
			q->flip = 1;
			continue;
		}
		if (lacks_word(w, n, i)) {
			fault(rd->faults, "%s: %s needs a word after it",
			      rd->where, w[i]);
			return 0;
		}
		got = read_placement(rd, w, n, &i, pl);
		if (got < 0) {
			return 0;
		}
		if (got) {
			continue;
		}
		if (!strcmp(w[i], "rating")) {
			free(q->rating);
			q->rating = sch_strdup(w[++i]);
		} else if (!strcmp(w[i], "mpn")) {
			free(q->mpn);
			q->mpn = sch_strdup(w[++i]);
		} else if (!strcmp(w[i], "side")) {
			q->side = side_of(w[++i]);
			if (q->side < 0) {
				fault(rd->faults,
				      "%s: side is right, above, left or below",
				      rd->where);
				return 0;
			}
		} else if (!*value && dir_word(w[i]) < 0) {
			*value = w[i];
		} else {
			fault(rd->faults, "%s: what is %s?", rd->where, w[i]);
			return 0;
		}
	}
	return 1;
}

// The device of part q, of shape sh and value value (or NULL): of its
// pin table, if it has one; returns NULL on a fault.
static struct sch_device *
part_device(struct reader *rd, const struct sch_shape *sh, const char *value)
{
	char *table = value ? find_table(rd, value) : NULL;
	struct sch_device *dev = NULL;

	if (table) {
		dev = device(rd, rd->s->lib, rd->s->nlib, table, rd->where);
		free(table);
		if (dev->shape != sh) {
			fault(rd->faults, "%s: %s is drawn as %s, not %s",
			      rd->where, value, dev->shape->name, sh->name);
			return NULL;
		}
		return dev;
	}
	dev = device(rd, rd->s->lib, rd->s->nlib, sh->name, rd->where);
	for (size_t p = 0; p < sh->npin; p++) {
		if (!isdigit((unsigned char)sh->pin[p].name[0])) {
			fault(rd->faults,
			      "%s: a %s needs a part number with a pin "
			      "table",
			      rd->where, sh->name);
			return NULL;
		}
	}
	return dev;
}

static void read_part(struct reader *rd, struct sch_shape *sh, char **w, int n)
{
	struct sch_part q;
	struct place pl;
	const char *value = NULL;

	memset(&q, 0, sizeof q);
	memset(&pl, 0, sizeof pl);
	q.side = -1;
	q.line = rd->line_no;
	pl.kind = ROOT;
	pl.dir = -1;
	pl.gap = -1;
	pl.line = rd->line_no;
	if (n < 2) {
		fault(rd->faults, "%s: %s NAME ...", rd->where, w[0]);
		return;
	}
	pl.scope = sch_strdup(rd->group_path);
	q.name = in_path(rd, w[1]);
	if (part_named(rd, q.name) >= 0) {
		fault(rd->faults, "%s: part %s again", rd->where, q.name);
		free_draft(&q, &pl);
		return;
	}
	if (!read_part_words(rd, w, n, &q, &pl, &value)) {
		free_draft(&q, &pl);
		return;
	}
	if (value && !value_ok(sh, value)) {
		fault(rd->faults,
		      "%s: a %s's value is a number and its unit letter, %s",
		      rd->where, sh->name, units_told(sh));
		free_draft(&q, &pl);
		return;
	}
	if (value) {
		q.value = sch_strdup(value);
	}
	q.dev = part_device(rd, sh, value);
	if (!q.dev) {
		free_draft(&q, &pl);
		return;
	}
	if (q.flip && sh->flip[0] < 0) {
		q.mirror = 1; // A shape without a flip pair is mirrored.
		q.flip = 0;
	}
	add_part(rd, &q, &pl);
}

// Whether the library has a symbol of the name.
static int is_symbol(struct reader *rd, const char *name)
{
	size_t n = strlen(name) + sizeof ".sym";
	char *file = sch_alloc(n);
	char *path = NULL;
	int found = 0;

	(void)snprintf(file, n, "%s.sym", name);

	path = sch_find(rd->s->lib, rd->s->nlib, file);
	found = path != NULL;
	free(file);
	free(path);
	return found;
}

// A statement of the kind to be carried out later, of the line and the
// group open now.
static struct later new_later(const struct reader *rd, int kind)
{
	struct later l = {
		kind, -1, rd->line_no, { NULL, NULL }, NULL, NULL, -1
	};

	l.scope = sch_strdup(rd->group_path);
	return l;
}

static void push_later(struct reader *rd, const struct later *l)
{
	rd->laters = grow(rd->laters, rd->nlaters, sizeof *rd->laters);
	rd->laters[rd->nlaters++] = *l;
}

// group NAME: opens a group in the one open now.
static void read_group(struct reader *rd, const char *name)
{
	size_t len = strlen(rd->group_path);

	if (strchr(name, '.')) {
		fault(rd->faults, "%s: a name has no dot", rd->where);
		return;
	}
	if (len + strlen(name) + 2 > sizeof rd->group_path) {
		fault(rd->faults, "%s: the groups nest too deep", rd->where);
		return;
	}
	if (len) {
		rd->group_path[len++] = '.';
	}
	memcpy(rd->group_path + len, name, strlen(name) + 1);
	rd->depth++;
}

// end: closes the group open now.
static void read_end(struct reader *rd)
{
	char *dot = strrchr(rd->group_path, '.');

	if (!rd->depth) {
		fault(rd->faults, "%s: end of no group", rd->where);
		return;
	}
	if (dot) {
		*dot = 0;
	} else {
		*rd->group_path = 0;
	}
	rd->depth--;
}

// note TEXT: for the people and the agents that read the source, and
// for the viewer, which shows it by its group.
static void read_note(struct reader *rd, char **w, int n)
{
	struct sch_note nt;

	nt.group = rd->depth ? sch_strdup(rd->group_path) : NULL;
	nt.text = join(w + 1, n - 1);
	nt.line = rd->line_no;
	rd->s->note = grow(rd->s->note, rd->s->nnote, sizeof *rd->s->note);
	rd->s->note[rd->s->nnote++] = nt;
}

// port NAME PIN.  A port of the sheet is named alike on every page, and
// may stand at several pins: each is drawn, and they are one net; a port
// of a group stands for one pin.
static void read_port(struct reader *rd, char **w)
{
	struct alias al;

	al.name = rd->depth ? in_path(rd, w[1]) : sch_strdup(w[1]);
	for (size_t i = 0; rd->depth && i < rd->naliases; i++) {
		if (!strcmp(rd->aliases[i].name, al.name)) {
			fault(rd->faults, "%s: port %s again", rd->where,
			      al.name);
			free(al.name);
			return;
		}
	}
	al.ref = sch_strdup(w[2]);
	al.scope = sch_strdup(rd->group_path);
	al.depth = rd->depth;
	al.line = rd->line_no;
	al.pid = -1;
	rd->aliases = grow(rd->aliases, rd->naliases, sizeof *rd->aliases);
	rd->aliases[rd->naliases++] = al;
}

// class NAME PIN...: the net of each pin is of the class.
static void read_class(struct reader *rd, char **w, int n)
{
	if (!plain_word(w[1])) {
		fault(rd->faults, "%s: a class is a plain word, not %s",
		      rd->where, w[1]);
		return;
	}
	for (int i = 2; i < n; i++) {
		struct later l = new_later(rd, CLASS);

		l.ref[0] = sch_strdup(w[i]);
		l.name = sch_strdup(w[1]);
		push_later(rd, &l);
	}
}

// wire PIN to PIN [WAY].
static void read_wire(struct reader *rd, char **w, int n)
{
	struct later l;
	int side = n == WIRE_WORDS ? dir_word(w[WIRE_WORDS - 1]) : -1;

	if (n == WIRE_WORDS && side < 0) {
		fault(rd->faults,
		      "%s: a wire keeps to the left, right, up or down",
		      rd->where);
		return;
	}
	l = new_later(rd, WIRE);
	l.side = side;
	l.ref[0] = sch_strdup(w[1]);
	l.ref[1] = sch_strdup(w[3]);
	push_later(rd, &l);
}

// Whether w[0] .. w[n-1] is a flag statement: ground at PIN, chassis at
// PIN, supply NAME at PIN or global NAME at PIN.
static int is_flag(char **w, int n)
{
	if (!strcmp(w[0], "ground") || !strcmp(w[0], "chassis")) {
		return n == 3 && !strcmp(w[1], "at");
	}
	if (!strcmp(w[0], "supply") || !strcmp(w[0], "global")) {
		return n == 4 && !strcmp(w[2], "at");
	}
	return 0;
}

static void read_flag(struct reader *rd, char **w, int n)
{
	struct later l = new_later(rd, FLAG);
	const char *name = w[1];

	if (n != 4) {
		name = !strcmp(w[0], "ground") ? "GND" : "CHASSIS";
	}
	l.ref[0] = sch_strdup(w[n - 1]);
	l.ref[1] = sch_strdup(w[0]);
	l.name = sch_strdup(name);
	push_later(rd, &l);
}

// A part statement, of a symbol of the library not a flag.
static void read_symbol(struct reader *rd, char **w, int n)
{
	struct sch_shape *sh = NULL;

	if (is_symbol(rd, w[0])) {
		sh = shape(rd, rd->s->lib, rd->s->nlib, w[0], rd->where);
	}
	if (sh && !sh->flag) {
		read_part(rd, sh, w, n);
	} else {
		fault(rd->faults, "%s: what is %s?", rd->where, w[0]);
	}
}

static void read_statement(struct reader *rd, char **w, int n)
{
	if (!strcmp(w[0], "group") && n == 2) {
		read_group(rd, w[1]);
	} else if (!strcmp(w[0], "end") && n == 1) {
		read_end(rd);
	} else if (!strcmp(w[0], "note")) {
		read_note(rd, w, n);
	} else if (!strcmp(w[0], "port") && n == 3) {
		read_port(rd, w);
	} else if (!strcmp(w[0], "nc") && n == 2) {
		// A pin left unconnected on purpose.
		struct later l = new_later(rd, NC);

		l.ref[0] = sch_strdup(w[1]);
		push_later(rd, &l);
	} else if (!strcmp(w[0], "class") && n >= 3) {
		read_class(rd, w, n);
	} else if (!strcmp(w[0], "wire") && (n == 4 || n == WIRE_WORDS) &&
		   !strcmp(w[2], "to")) {
		read_wire(rd, w, n);
	} else if (is_flag(w, n)) {
		read_flag(rd, w, n);
	} else {
		read_symbol(rd, w, n);
	}
}

// A flag: the name of its shape, and the global name it gives.
struct flag_of {
	const char *flag;
	const char *name;
};

// Puts a flag on a pin: a part of the flag's shape, and a global name.
static void flag_pin(struct reader *rd, size_t pid, struct flag_of fo)
{
	const char *flag = fo.flag;
	const char *name = fo.name;
	struct pinrec *r = &rd->pins[pid];
	struct sch_part f;
	struct place pl;
	size_t k = 0;

	if (r->flag) {
		fault(rd->faults, "%s: the pin has a flag already", rd->where);
		return;
	}
	memset(&f, 0, sizeof f);
	memset(&pl, 0, sizeof pl);
	r->flag = shape(rd, rd->s->lib, rd->s->nlib, flag, rd->where);
	f.dev = device(rd, rd->s->lib, rd->s->nlib, flag, rd->where);
	f.name = sch_strdup(name);
	f.text = f.name;
	f.side = -1;
	f.line = rd->line_no;
	pl.kind = ROOT;
	pl.gap = -1;
	k = add_part(rd, &f, &pl);
	r = &rd->pins[pid];
	r->flagpart = (long)k;
	r->global = f.name;
	r->used = 1;
	rd->pins[rd->base[k]].used = 1;
	join_pins(rd, rd->base[k], pid);
	for (size_t i = 0; i < rd->npins; i++) {
		if (i != pid && rd->pins[i].global &&
		    !strcmp(rd->pins[i].global, name) &&
		    rd->pins[i].flagpart >= 0) {
			join_pins(rd, pid, i);
			break;
		}
	}
}

// Looks up the ports, which may stand for one another: -2 marks one
// whose name is not found (told once).
static void resolve_ports(struct reader *rd)
{
	for (int more = 1; more;) {
		more = 0;
		for (size_t i = 0; i < rd->naliases; i++) {
			long p = 0;

			if (rd->aliases[i].pid != -1) {
				continue;
			}
			at_line(rd, rd->aliases[i].line);
			p = lookup(rd,
				   (struct scoped){ rd->aliases[i].ref,
						    rd->aliases[i].scope },
				   1);
			if (p != -1) {
				rd->aliases[i].pid = p;
				if (p >= 0) {
					rd->pins[p].used = 1;
				}
				more = 1;
			}
		}
	}
	for (size_t i = 0; i < rd->naliases; i++) {
		if (rd->aliases[i].pid == -1) {
			at_line(rd, rd->aliases[i].line);
			fault(rd->faults, "%s: port %s leads in a circle",
			      rd->where, rd->aliases[i].name);
		}
	}
}

// Looks up the anchor of part k, and makes the joins it makes; a part
// whose anchor is not found is left out.
static void resolve_anchor(struct reader *rd, size_t k)
{
	struct place *pl = &rd->places[k];
	long a = 0;
	long b = 0;

	at_line(rd, pl->line);
	if (pl->kind == FROM || pl->kind == BETWEEN) {
		a = pin_of(rd, (struct scoped){ pl->ref[0], pl->scope });
	}
	if (pl->kind == BETWEEN) {
		b = pin_of(rd, (struct scoped){ pl->ref[1], pl->scope });
	}
	if (pl->kind == OF) {
		pl->group = group_of(rd, pl->ref[0], pl->scope);
		if (pl->group) {
			return;
		}
	}
	if (pl->kind == BESIDE || pl->kind == OF) {
		a = lookup(rd, (struct scoped){ pl->ref[0], pl->scope }, 0);
	}
	if (a < 0 || b < 0) {
		rd->dead[k] = 1;
		return;
	}
	if (pl->kind == FROM) {
		size_t e = entry_pin(rd, &rd->s->part[k], pl->dir);

		pl->pid[0] = (size_t)a;
		if (e == (size_t)-1) {
			rd->dead[k] = 1;
			return;
		}
		join_pins(rd, rd->base[k] + e, pl->pid[0]);
	} else if (pl->kind == BESIDE || pl->kind == OF) {
		pl->part = (size_t)a;
		if (pl->kind == OF) {
			return;
		}
		if (rd->s->part[pl->part].dev->shape !=
		    rd->s->part[k].dev->shape) {
			fault(rd->faults,
			      "%s: a part beside another is of its shape",
			      rd->where);
			rd->dead[k] = 1;
			return;
		}
		for (size_t p = 0; p < rd->s->part[k].dev->shape->npin; p++) {
			join_pins(rd, rd->base[k] + p, rd->base[pl->part] + p);
		}
	} else if (pl->kind == BETWEEN) {
		pl->pid[0] = (size_t)a;
		pl->pid[1] = (size_t)b;
		if (rd->s->part[k].dev->shape->npin != 2) {
			fault(rd->faults,
			      "%s: only a part of two pins goes between",
			      rd->where);
			rd->dead[k] = 1;
			return;
		}
		join_pins(rd, rd->base[k], pl->pid[0]);
		join_pins(rd, rd->base[k] + 1, pl->pid[1]);
	}
}

// Carries out the wire or flag i; one whose pins are not found is left
// out.
static void resolve_later(struct reader *rd, size_t i)
{
	const struct later *l = &rd->laters[i];
	long a = 0;
	long b = 0;

	at_line(rd, l->line);
	rd->line_no = l->line;
	a = pin_of(rd, (struct scoped){ l->ref[0], l->scope });
	if (l->kind == WIRE) {
		b = pin_of(rd, (struct scoped){ l->ref[1], l->scope });
	}
	if (a < 0 || b < 0) {
		return;
	}
	if (l->kind == WIRE) {
		struct sch_hint h;

		join_pins(rd, (size_t)a, (size_t)b);
		h.part[0] = rd->pins[a].part;
		h.pin[0] = rd->pins[a].pin;
		h.part[1] = rd->pins[b].part;
		h.pin[1] = rd->pins[b].pin;
		h.side = l->side;
		rd->s->hint =
			grow(rd->s->hint, rd->s->nhint, sizeof *rd->s->hint);
		rd->s->hint[rd->s->nhint++] = h;
	} else if (l->kind == CLASS) {
		rd->laters[i].pid = a;
	} else if (l->kind == NC) {
		struct sch_part *q = &rd->s->part[rd->pins[a].part];

		if (!q->nc) {
			q->nc = sch_alloc(q->dev->shape->npin + 1);
		}
		q->nc[rd->pins[a].pin] = 1;
	} else {
		flag_pin(rd, (size_t)a, (struct flag_of){ l->ref[1], l->name });
	}
}

// Looks up every name, now that all are known: the ports (which may
// stand for one another), the anchors of the parts, the wires and the
// flags.
static void resolve_all(struct reader *rd)
{
	resolve_ports(rd);
	for (size_t k = 0; k < rd->s->npart; k++) {
		resolve_anchor(rd, k);
	}
	for (size_t i = 0; i < rd->nlaters; i++) {
		resolve_later(rd, i);
	}
	// The ports of the sheet are drawn, as flags.
	for (size_t i = 0; i < rd->naliases; i++) {
		if (!rd->aliases[i].depth && rd->aliases[i].pid >= 0) {
			at_line(rd, rd->aliases[i].line);
			rd->line_no = rd->aliases[i].line;
			flag_pin(rd, (size_t)rd->aliases[i].pid,
				 (struct flag_of){ "port",
						   rd->aliases[i].name });
		}
	}
}

// Leaves the value off each ferrite of the kind the page uses most; its
// designator stays.
// Whether part o is a ferrite of the value and part number of q.
static int same_ferrite(const struct sch_part *o, const struct sch_part *q)
{
	return !strcmp(o->dev->shape->name, "ferrite") &&
	       !strcmp(o->value ? o->value : "", q->value ? q->value : "") &&
	       !strcmp(o->mpn ? o->mpn : "", q->mpn ? q->mpn : "");
}

static void common_ferrite(struct reader *rd)
{
	size_t best = 0;
	size_t most = 0;

	for (size_t i = 0; i < rd->s->npart; i++) {
		size_t n = 0;
		const struct sch_part *q = &rd->s->part[i];

		if (strcmp(q->dev->shape->name, "ferrite") != 0) {
			continue;
		}
		for (size_t j = 0; j < rd->s->npart; j++) {
			if (same_ferrite(&rd->s->part[j], q)) {
				n++;
			}
		}
		if (n > most) {
			most = n;
			best = i;
		}
	}
	for (size_t j = 0; most && j < rd->s->npart; j++) {
		const struct sch_part *q = &rd->s->part[best];
		struct sch_part *o = &rd->s->part[j];

		if (same_ferrite(o, q)) {
			o->novalue = 1;
		}
	}
}

// Gives net t, of the pins of root r, its class: of its class
// statements, or power if a supply or ground flag is on it.  A second
// class is a fault, told once.
static void net_class(struct reader *rd, struct sch_net *t, size_t r,
		      const char *power)
{
	const char *c = power;
	int told = 0;

	for (size_t i = 0; i < rd->nlaters; i++) {
		const struct later *l = &rd->laters[i];

		if (l->kind != CLASS || l->pid < 0 ||
		    root(rd, (size_t)l->pid) != r) {
			continue;
		}
		if (c && strcmp(c, l->name) != 0) {
			if (!told) {
				fault(rd->faults,
				      "%s:%d: net %s is of class %s and of "
				      "class %s",
				      rd->s->file, l->line, t->name, c,
				      l->name);
			}
			told = t->clash = 1;
			continue;
		}
		c = l->name;
		if (!t->class_line) {
			t->class_line = l->line;
		}
	}
	t->klass = c ? sch_strdup(c) : NULL;
}

// Makes the nets: the pins joined, each set that has two pins or a flag.
// A net takes its global name, or the name of its outermost port, or the
// name of its first pin.  Each pin with a flag and the flag make a group
// of their own; the other pins make one group with the first of them,
// wherever it stands among them.
// A net being made: the net, its name so far, the depth of the port it
// is named after, its pins, whether a flag is on it and one of supply or
// ground, and the group of its pins without a flag, and whether a pin
// with a flag has joined that.
struct net_draft {
	struct sch_net t;
	const char *name;
	int best;
	size_t n;
	int flagged, power;
	int rest, rflag;
};

// Adds an end to the net t.
static void add_end(struct sch_net *t, struct sch_end e)
{
	t->end = grow(t->end, t->nend, sizeof *t->end);
	t->end[t->nend++] = e;
}

// The name pin pj of the net nd gives: its global name, or the name of
// its outermost port.
static void name_by_pin(struct reader *rd, struct net_draft *nd, size_t j)
{
	const struct pinrec *pj = &rd->pins[j];

	if (pj->global) {
		if (nd->t.global && strcmp(nd->name, pj->global) != 0) {
			fault(rd->faults, "%s: nets %s and %s are joined",
			      rd->s->file, nd->name, pj->global);
		}
		nd->name = pj->global;
		nd->t.global = 1;
	}
	for (size_t k = 0; !nd->t.global && k < rd->naliases; k++) {
		if (rd->aliases[k].pid == (long)j &&
		    rd->aliases[k].depth < nd->best) {
			nd->best = rd->aliases[k].depth;
			nd->name = rd->aliases[k].name;
		}
	}
}

// Adds pin j to the net nd; *group is the next group free.
static void add_net_pin(struct reader *rd, struct net_draft *nd, size_t j,
			int *group)
{
	const struct pinrec *pj = &rd->pins[j];
	int g = 0;

	nd->n++;
	nd->flagged = nd->flagged || pj->flag != NULL;
	nd->power = nd->power || (pj->flag && pj->flag->flag &&
				  (!strcmp(pj->flag->flag, "supply") ||
				   !strcmp(pj->flag->flag, "ground")));
	name_by_pin(rd, nd, j);
	if (rd->s->part[pj->part].dev->shape->flag) {
		return; // With the pin it flags, below.
	}
	if (pj->flag && nd->rflag) {
		g = (*group)++;
	} else {
		// The first pin with a flag joins the other pins, those
		// before it too.
		if (nd->rest < 0) {
			nd->rest = (*group)++;
		}
		g = nd->rest;
		nd->rflag = nd->rflag || pj->flag;
	}
	add_end(&nd->t, (struct sch_end){ pj->part, pj->pin, pj->flag, g });
	if (pj->flagpart >= 0) {
		add_end(&nd->t,
			(struct sch_end){ (size_t)pj->flagpart, 0, NULL, g });
	}
}

// The name of a net of no global name or port: that of pin i.
static char *pin_net_name(const struct reader *rd, size_t i)
{
	const struct sch_part *q = &rd->s->part[rd->pins[i].part];
	const char *pin = q->dev->shape->pin[rd->pins[i].pin].name;
	size_t n = strlen(q->name) + strlen(pin) + 2;
	char *s = sch_alloc(n);

	(void)snprintf(s, n, "%s.%s", q->name, pin);
	return s;
}

static void make_nets(struct reader *rd)
{
	size_t *done = sch_alloc((rd->npins + 1) * sizeof *done);
	int group = 0;

	for (size_t i = 0; i < rd->npins; i++) {
		size_t r = root(rd, i);
		struct net_draft nd;

		if (!rd->pins[i].used || done[r]) {
			continue;
		}
		done[r] = 1;
		memset(&nd, 0, sizeof nd);
		nd.best = DEPTH_NONE;
		nd.rest = -1;
		nd.t.line = rd->pins[i].line;
		for (size_t j = 0; j < rd->npins; j++) {
			if (rd->pins[j].used && root(rd, j) == r) {
				add_net_pin(rd, &nd, j, &group);
			}
		}
		if (nd.n < 2 && !nd.flagged) {
			free(nd.t.end);
			continue;
		}
		nd.t.name = nd.name ? sch_strdup(nd.name) : pin_net_name(rd, i);
		net_class(rd, &nd.t, r, nd.power ? "power" : NULL);
		rd->s->net = grow(rd->s->net, rd->s->nnet, sizeof *rd->s->net);
		rd->s->net[rd->s->nnet++] = nd.t;
	}
	free(done);
}

// Gives each part its designator: its prefix and the page number times
// 100 plus its count among the parts of that prefix.
static void designate(struct reader *rd, struct sch *s)
{
	for (size_t i = 0; i < s->npart; i++) {
		struct sch_part *q = &s->part[i];
		int k = 1;
		size_t n = 0;

		if (q->dev->shape->flag) {
			continue;
		}
		for (size_t j = 0; j < i; j++) {
			const struct sch_part *o = &s->part[j];

			k += !o->dev->shape->flag &&
			     !strcmp(o->dev->prefix, q->dev->prefix);
		}
		if (k > PAGE_REFS - 1) {
			fault(rd->faults, "%s: more than 99 parts of prefix %s",
			      s->file, q->dev->prefix);
		}
		n = strlen(q->dev->prefix) + REF_NUMBER_LEN;
		q->ref = sch_alloc(n);
		(void)snprintf(q->ref, n, "%s%d", q->dev->prefix,
			       (s->page * PAGE_REFS) + k);
	}
}

// Frees what the reading of a page keeps only for itself.
static void forget(struct reader *rd)
{
	for (size_t k = 0; k < rd->s->npart; k++) {
		free(rd->places[k].ref[0]);
		free(rd->places[k].ref[1]);
		free(rd->places[k].scope);
		free(rd->places[k].group);
	}
	for (size_t i = 0; i < rd->naliases; i++) {
		free(rd->aliases[i].name);
		free(rd->aliases[i].ref);
		free(rd->aliases[i].scope);
	}
	for (size_t i = 0; i < rd->nlaters; i++) {
		free(rd->laters[i].ref[0]);
		free(rd->laters[i].ref[1]);
		free(rd->laters[i].scope);
		free(rd->laters[i].name);
	}
	free(rd->pins);
	free(rd->base);
	free(rd->placed);
	free(rd->besides);
	free(rd->dead);
	free(rd->anchor_of);
	free(rd->along_of);
	free(rd->places);
	free(rd->aliases);
	free(rd->laters);
	rd->pins = NULL;
	rd->base = NULL;
	rd->placed = rd->besides = rd->dead = rd->along_of = NULL;
	rd->anchor_of = NULL;
	rd->places = NULL;
	rd->aliases = NULL;
	rd->laters = NULL;
	rd->npins = rd->naliases = rd->nlaters = 0;
}

// A page being read: its index k, whether a port statement is open, and
// the statements before the first page.
struct page_read {
	size_t k;
	int open;
	int before;
};

// Notes a page statement, page NAME, among the pages of the file,
// numbered by its place among them; the page read checks its words.
// The page read checks its words: the one that ps reads.
static void add_page(struct reader *rd, char **w, int n,
		     const struct page_read *ps)
{
	struct sch *s = rd->s;
	int read = s->npages == ps->k;
	struct sch_page pg;
	int num = 0;

	memset(&pg, 0, sizeof pg);
	pg.line = rd->line_no;
	pg.name = sch_strdup(n == 2 ? w[1] : "");
	pg.num = (int)s->npages + 1;
	if (n != 2 || integer(w[1], &num)) {
		if (read) {
			fault(rd->faults, "%s: page NAME, a name and no number",
			      rd->where);
		}
	} else if (strchr(w[1], '.') && read) {
		fault(rd->faults, "%s: a name has no dot", rd->where);
	} else if (pg.num > PAGE_REFS - 1 && read) {
		fault(rd->faults, "%s: more than 99 pages", rd->where);
	}
	s->pages = grow(s->pages, s->npages, sizeof *s->pages);
	s->pages[s->npages++] = pg;
}

// Notes a sheet port of the page pg, a port outside any group, by the
// words of a statement; open counts the groups open.
static void note_port(struct reader *rd, struct sch_page *pg, char **w, int n,
		      int *open)
{
	if (!strcmp(w[0], "group") && n == 2) {
		(*open)++;
	} else if (!strcmp(w[0], "end") && n == 1 && *open) {
		(*open)--;
	} else if (!strcmp(w[0], "port") && n == 3 && !*open) {
		for (size_t i = 0; i < pg->nport; i++) {
			if (!strcmp(pg->port[i], w[1])) {
				return;
			}
		}
		pg->port = (char **)grow((void *)pg->port, pg->nport,
					 sizeof *pg->port);
		pg->port_line =
			grow(pg->port_line, pg->nport, sizeof *pg->port_line);
		pg->port[pg->nport] = sch_strdup(w[1]);
		pg->port_line[pg->nport++] = rd->line_no;
	}
}

// Tells the faults of the pages of the file: a name of a page twice, and, as a warning, a sheet port on one page alone.
static void check_pages(struct reader *rd, const struct sch *s)
{
	if (!s->npages) {
		fault(rd->faults, "%s: no page statement", s->file);
	}
	for (size_t j = 0; j < s->npages; j++) {
		const struct sch_page *b = &s->pages[j];

		for (size_t i = 0; i < j; i++) {
			const struct sch_page *a = &s->pages[i];

			if (*b->name && !strcmp(a->name, b->name)) {
				fault(rd->faults, "%s:%d: page %s again",
				      s->file, b->line, b->name);
			}
		}
	}
	for (size_t j = 0; s->npages > 1 && j < s->npages; j++) {
		const struct sch_page *b = &s->pages[j];

		for (size_t p = 0; p < b->nport; p++) {
			int other = 0;

			for (size_t i = 0; i < s->npages; i++) {
				for (size_t q = 0;
				     i != j && q < s->pages[i].nport; q++) {
					other = other ||
						!strcmp(s->pages[i].port[q],
							b->port[p]);
				}
			}
			if (!other) {
				sch_warn(
					"%s:%d: warning: port %s is on page %d "
					"alone",
					s->file, b->port_line[p], b->port[p],
					b->num);
			}
		}
	}
}

// field NAME TEXT or title TEXT: a field of the title block.
static void read_field(struct reader *rd, char **w, int n)
{
	struct sch *s = rd->s;
	int t = w[0][0] == 't';

	if (s->nfield == FIELDS) {
		fault(rd->faults, "%s: too many fields", rd->where);
		return;
	}
	s->field[s->nfield][0] = sch_strdup(t ? "Title" : w[1]);
	s->field[s->nfield][1] = join(w + (t ? 1 : 2), n - (t ? 1 : 2));
	s->nfield++;
}

// Takes the statement w of n words on its page, of ps: a page, a field
// or title, a sheet, a scale, or one read_statement reads.
static void page_record(struct reader *rd, char **w, int n,
			struct page_read *ps)
{
	struct sch *s = rd->s;
	size_t k = ps->k;

	if (!strcmp(w[0], "page")) {
		add_page(rd, w, n, ps);
		ps->open = 0;
		if (s->npages == k + 1) {
			s->page = s->pages[k].num;
			free(s->name);
			s->name = sch_strdup(s->pages[k].name);
			(void)snprintf(rd->group_path, sizeof rd->group_path,
				       "%s", s->name);
		}
		return;
	}
	if (!s->npages) {
		if (!k && !ps->before++) {
			fault(rd->faults,
			      "%s: a statement before the first page",
			      rd->where);
		}
		return;
	}
	note_port(rd, &s->pages[s->npages - 1], w, n, &ps->open);
	if (s->npages != k + 1) {
		return;
	}
	if ((!strcmp(w[0], "field") && n >= 3) ||
	    (!strcmp(w[0], "title") && n >= 2)) {
		read_field(rd, w, n);
	} else if (!strcmp(w[0], "sheet") && n == 2) {
		s->sheet = sch_strdup(w[1]);
	} else if (!strcmp(w[0], "scale") && n == 2) {
		s->scale = strtod(w[1], NULL);
		if (!(s->scale > SCALE_MIN && s->scale < SCALE_MAX)) {
			fault(rd->faults, "%s: bad scale", rd->where);
			s->scale = 1;
		}
	} else {
		read_statement(rd, w, n);
	}
}

// Reads the lines of the source f for its page ps->k.
static void read_lines(struct reader *rd, FILE *f, struct page_read *ps)
{
	struct sch *s = rd->s;
	char buf[LINE];
	char *w[WORDS];

	while (fgets(buf, sizeof buf, f)) {
		int n = 0;
		int here = 0;

		rd->line_no++;
		at_line(rd, rd->line_no);
		if (!strchr(buf, '\n') && !feof(f)) {
			int c = 0;

			if (s->npages == ps->k + 1 || (!s->npages && !ps->k)) {
				fault(rd->faults, "%s: line too long",
				      rd->where);
			}
			while ((c = getc(f)) != EOF && c != '\n') {
			}
			if (c == EOF) {
				break;
			}
			continue;
		}
		n = split(buf, w);
		here = s->npages == ps->k + 1;
		if (n < 0) {
			if (here) {
				fault(rd->faults, "%s: too many words",
				      rd->where);
			}
			continue;
		}
		if (n > 0) {
			page_record(rd, w, n, ps);
		}
	}
}

// Makes the lists of the parts, of room for one each.
static void start_parts(struct reader *rd)
{
	rd->s->part = grow(NULL, 0, sizeof *rd->s->part);
	rd->base = grow(NULL, 0, sizeof *rd->base);
	rd->placed = grow(NULL, 0, sizeof *rd->placed);
	rd->dead = grow(NULL, 0, sizeof *rd->dead);
	rd->besides = grow(NULL, 0, sizeof *rd->besides);
	rd->anchor_of = grow(NULL, 0, sizeof *rd->anchor_of);
	rd->along_of = grow(NULL, 0, sizeof *rd->along_of);
	rd->places = grow(NULL, 0, sizeof *rd->places);
}

int sch_read(struct sch *s, const char *file, size_t k, struct sch_lib *lib)
{
	static const char own[] = "/symbols";
	struct reader *rd = sch_alloc(sizeof *rd);
	struct page_read ps = { k, 0, 0 };
	char **libs = NULL;
	const char *slash = strrchr(file, '/');
	size_t dn = slash ? (size_t)(slash - file) : 1;
	FILE *f = fopen(file, "r");
	int faults = 0;

	memset(s, 0, sizeof *s);
	rd->lib = lib;
	rd->s = s;
	start_parts(rd);
	s->file = file;
	s->page = 1;
	s->name = sch_strdup("");
	// The project's own symbols come first: those next to the source.
	libs = (char **)sch_alloc((lib->ndir + 1) * sizeof *libs);
	libs[0] = sch_alloc(dn + sizeof(own));
	memcpy(libs[0], slash ? file : ".", dn);
	memcpy(libs[0] + dn, own, sizeof(own));
	memcpy((void *)(libs + 1), (const void *)lib->dir,
	       lib->ndir * sizeof *lib->dir);
	s->lib = libs;
	s->nlib = lib->ndir + 1;
	s->scale = 1;
	s->sheet = sch_strdup("sch_a");
	if (!f) {
		sch_die("%s: cannot read", file);
	}
	read_lines(rd, f, &ps);
	(void)fclose(f);
	read_clearances(rd, s);
	if (rd->depth) {
		fault(rd->faults, "%s: group %s has no end", file,
		      rd->group_path);
	}
	if (!k) {
		check_pages(rd, s);
	}
	resolve_all(rd);
	common_ferrite(rd);
	place_all(rd);
	make_nets(rd);
	designate(rd, s);
	tell_traces(rd);
	forget(rd);
	faults = rd->faults;
	free(rd);
	return faults;
}
