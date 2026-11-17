// SPDX-License-Identifier: GPL-3.0-only
// schsrc.c -- Read a schematic source and the symbols of its library
// Copyright (c) 2026 Jakob Kastelic

// See sch(5) for the source and schsym(5) for the symbols.  Every error
// is fatal and names the file and the line.

#define _POSIX_C_SOURCE 200809L // For opendir.

#include "schsrc.h"
#include "stroke.h"
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LINE = 4096, WORDS = 256 };

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
	p = realloc(p, (n + 1) * size);
	if (!p) {
		sch_die("out of memory");
	}
	memset((char *)p + n * size, 0, size);
	return p;
}

char *sch_strdup(const char *s)
{
	char *d = sch_alloc(strlen(s) + 1);

	strcpy(d, s);
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

// Joins words a..n-1 with one blank between them.
static char *join(char **w, int a, int n)
{
	size_t len = 1;
	char *s;

	for (int i = a; i < n; i++) {
		len += strlen(w[i]) + 1;
	}
	s = sch_alloc(len);
	for (int i = a; i < n; i++) {
		if (i > a) {
			strcat(s, " ");
		}
		strcat(s, w[i]);
	}
	return s;
}

static int integer(const char *s, int *v)
{
	char *end;
	long x = strtol(s, &end, 10);

	if (end == s || *end || x < -100000 || x > 100000) {
		return 0;
	}
	*v = (int)x;
	return 1;
}

char *sch_default_lib(const char *argv0)
{
	const char *slash = strrchr(argv0, '/'), *tail = "/../lib/symbols";
	size_t n = slash ? (size_t)(slash - argv0) : 1;
	char *s = sch_alloc(n + strlen(tail) + 1);

	memcpy(s, slash ? argv0 : ".", n);
	strcpy(s + n, tail);
	return s;
}

char *sch_sheet_path(const struct sch *s)
{
	char name[256], **dirs = sch_alloc((s->nlib + 1) * sizeof *dirs), *p;

	for (size_t i = 0; i < s->nlib; i++) {
		dirs[i] = sch_alloc(strlen(s->lib[i]) + 16);
		sprintf(dirs[i], "%s/../sheets", s->lib[i]);
	}
	snprintf(name, sizeof name, "%s.sheet", s->sheet);
	p = sch_find(dirs, s->nlib, name);
	for (size_t i = 0; i < s->nlib; i++) {
		free(dirs[i]);
	}
	free(dirs);
	return p;
}

char *sch_find(char **lib, size_t nlib, const char *name)
{
	for (size_t i = 0; i < nlib; i++) {
		char *p = sch_alloc(strlen(lib[i]) + strlen(name) + 2);
		FILE *f;

		sprintf(p, "%s/%s", lib[i], name);
		f = fopen(p, "r");
		if (f) {
			fclose(f);
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

// The shapes and devices read so far.
static struct sch_shape **shapes;
static size_t nshapes;
static struct sch_device **devices;
static size_t ndevices;

// Reads the shape NAME.sym: its records (schsym(5)), a line each, or
// more where a line ends in a backslash.
static struct sch_shape *shape(char **lib, size_t nlib, const char *name,
			       const char *where)
{
	static const char *const kinds[] = { "line",  "loop",	"fill",
					     "blank", "circle", "disk" };
	char buf[LINE], *w[WORDS], *path, *file;
	struct sch_shape *s;
	size_t ln = 0;
	double width = SCH_LW;
	FILE *f;

	for (size_t i = 0; i < nshapes; i++) {
		if (strcmp(shapes[i]->name, name) == 0) {
			return shapes[i];
		}
	}
	file = sch_alloc(strlen(name) + 5);
	sprintf(file, "%s.sym", name);
	path = sch_find(lib, nlib, file);
	if (!path) {
		sch_die("%s: no symbol %s in the library", where, name);
	}
	f = fopen(path, "r");
	if (!f) {
		sch_die("%s: cannot read", path);
	}
	s = sch_alloc(sizeof *s);
	s->name = sch_strdup(name);
	s->prefix = sch_strdup("X");
	s->side = 0;
	s->flip[0] = s->flip[1] = -1;
	while (fgets(buf, sizeof buf, f)) {
		int n, k = 0, when = 0;
		size_t len = strlen(buf);

		ln++;
		// A line that ends in a backslash goes on on the next.
		while (len >= 2 && buf[len - 2] == '\\' &&
		       buf[len - 1] == '\n' && len < sizeof buf - 1 &&
		       fgets(buf + len - 2, (int)(sizeof buf - len + 2), f)) {
			ln++;
			len = strlen(buf);
		}
		n = split(buf, w);
		if (n < 1 || w[0][0] == '#') {
			continue;
		}
		if (!strcmp(w[0], "flip") && n > 3) {
			when = 1;
			k = 1;
		} else if (!strcmp(w[0], "plain") && n > 1) {
			when = 2;
			k = 1;
		}
		if (strcmp(w[0], "pin") == 0 && n == 5) {
			struct sch_pin p;

			p.name = sch_strdup(w[1]);
			p.dir = dir_of(w[4]);
			if (!integer(w[2], &p.x) || !integer(w[3], &p.y) ||
			    p.dir < 0) {
				sch_die("%s:%zu: bad pin", path, ln);
			}
			s->pin = grow(s->pin, s->npin, sizeof *s->pin);
			s->pin[s->npin++] = p;
		} else if (strcmp(w[0], "box") == 0 && n == 5) {
			for (int i = 0; i < 4; i++) {
				s->box[i] = atof(w[i + 1]);
			}
		} else if (strcmp(w[0], "prefix") == 0 && n == 2) {
			free(s->prefix);
			s->prefix = sch_strdup(w[1]);
		} else if (strcmp(w[0], "side") == 0 && n == 2) {
			s->side = side_of(w[1]);
			if (s->side < 0) {
				sch_die("%s:%zu: bad side", path, ln);
			}
		} else if (strcmp(w[0], "swap") == 0 && n >= 3) {
			s->swap = join(w, 1, n);
		} else if (strcmp(w[0], "flip") == 0 && n == 3) {
			for (int m = 0; m < 2; m++) {
				for (size_t i = 0; i < s->npin; i++) {
					if (!strcmp(s->pin[i].name, w[m + 1])) {
						s->flip[m] = (int)i;
					}
				}
				if (s->flip[m] < 0) {
					sch_die("%s:%zu: flip of no pin %s",
						path, ln, w[m + 1]);
				}
			}
		} else if (strcmp(w[0], "flag") == 0 && n == 2) {
			s->flag = sch_strdup(w[1]);
		} else if (strcmp(w[0], "width") == 0 && n == 2) {
			width = atof(w[1]);
		} else {
			struct sch_draw d = { -1, when, width, NULL, 0 };

			for (int i = 0; i < 6; i++) {
				d.kind = !strcmp(w[k], kinds[i]) ? i : d.kind;
			}
			d.n = (size_t)(n - k - 1);
			if (d.kind < 0 ||
			    (d.kind >= SCH_CIRCLE ? d.n != 3 :
						    d.n < 4 || d.n % 2)) {
				sch_die("%s:%zu: bad record %s", path, ln,
					w[k]);
			}
			d.xy = sch_alloc(d.n * sizeof *d.xy);
			for (size_t i = 0; i < d.n; i++) {
				d.xy[i] = atof(w[k + 1 + (int)i]);
			}
			s->draw = grow(s->draw, s->ndraw, sizeof *s->draw);
			s->draw[s->ndraw++] = d;
		}
	}
	fclose(f);
	if (!s->npin) {
		sch_die("%s: the symbol has no pins", path);
	}
	free(path);
	free(file);
	shapes = grow(shapes, nshapes, sizeof *shapes);
	shapes[nshapes++] = s;
	return s;
}

// Reads the device NAME: a pin table if the library has a file of that
// name, else the generic shape NAME.sym.
static struct sch_device *device(char **lib, size_t nlib, const char *name,
				 const char *where)
{
	char buf[LINE], *w[WORDS], *path;
	struct sch_device *d;
	size_t ln = 0;
	FILE *f;

	for (size_t i = 0; i < ndevices; i++) {
		if (strcmp(devices[i]->name, name) == 0) {
			return devices[i];
		}
	}
	d = sch_alloc(sizeof *d);
	d->name = sch_strdup(name);
	path = sch_find(lib, nlib, name);
	if (!path) {
		d->shape = shape(lib, nlib, name, where);
		d->prefix = d->shape->prefix;
		d->pnum = sch_alloc(d->shape->npin * sizeof *d->pnum);
		for (size_t i = 0; i < d->shape->npin; i++) {
			d->pnum[i] = d->shape->pin[i].name;
		}
		goto done;
	}
	f = fopen(path, "r");
	if (!f) {
		sch_die("%s: cannot read", path);
	}
	while (fgets(buf, sizeof buf, f)) {
		int n = split(buf, w);

		ln++;
		if (n == 0) {
			continue;
		}
		if (strcmp(w[0], "shape") == 0 && n == 2 && !d->shape) {
			d->shape = shape(lib, nlib, w[1], path);
			d->pnum = sch_alloc(d->shape->npin * sizeof *d->pnum);
		} else if (strcmp(w[0], "prefix") == 0 && n == 2) {
			d->prefix = sch_strdup(w[1]);
		} else if (strcmp(w[0], "pin") == 0 && n == 3 && d->shape) {
			size_t i = 0;

			while (i < d->shape->npin &&
			       strcmp(d->shape->pin[i].name, w[1]) != 0) {
				i++;
			}
			if (i == d->shape->npin) {
				sch_die("%s:%zu: shape %s has no pin %s", path,
					ln, d->shape->name, w[1]);
			}
			d->pnum[i] = sch_strdup(w[2]);
		} else if (strcmp(w[0], "nc") == 0 && n >= 2) {
			d->nc = join(w, 1, n);
		} else {
			sch_die("%s:%zu: bad record", path, ln);
		}
	}
	fclose(f);
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
	free(path);
done:
	devices = grow(devices, ndevices, sizeof *devices);
	devices[ndevices++] = d;
	return d;
}

const char *sch_pnum(const struct sch_device *d, size_t p)
{
	return d->pnum[p];
}

void sch_pin_at(const struct sch_part *q, size_t p, int *x, int *y, int *dir)
{
	const struct sch_shape *sh = q->dev->shape;
	const struct sch_pin *pin;
	int px, py, d;

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
		d = (d == SCH_R) ? SCH_L : (d == SCH_L) ? SCH_R : d;
	}
	for (int i = 0; i < q->rot; i++) {
		int t = px;

		px = -py;
		py = t;
		d = (d + 1) % 4;
	}
	*x = q->x + px;
	*y = q->y + py;
	*dir = d;
}

void sch_box(const struct sch_part *q, double b[4])
{
	const double *s = q->dev->shape->box;
	const double x[4] = { s[0], s[2], s[2], s[0] };
	const double y[4] = { s[1], s[1], s[3], s[3] };

	b[0] = b[1] = 1e9;
	b[2] = b[3] = -1e9;
	for (int i = 0; i < 4; i++) {
		double px = q->mirror ? -x[i] : x[i], py = y[i];

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

void sch_label_size(const struct sch_part *q, double *w, double *h)
{
	int lines = 1;
	double v = 0;

	*w = sch_text_width(q->dev->prefix, SCH_TEXT) +
	     sch_text_width(q->ref ? q->ref + strlen(q->dev->prefix) : "400",
			    SCH_TEXT);
	if ((q->value || q->rating) && !q->novalue) {
		v = (q->value ? sch_text_width(q->value, SCH_TEXT) : 0) +
		    (q->rating ? sch_text_width(q->rating, SCH_TEXT) + 2 : 0);
		lines = 2;
	}
	*w = *w > v ? *w : v;
	*h = SCH_DESCENT + SCH_TEXT + (lines - 1) * SCH_LEAD;
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
};

enum { WIRE, FLAG, NC };

static struct sch *R;
static struct pinrec *pins;
static size_t npins;
static size_t *base; // The first pin of each part.
static int *placed, *besides;
static int *dead; // Parts that a fault keeps from being placed.
static long *anchor_of; // The pin each part hangs from, or -1.
static int *along_of; // How far along that pin's lead it hangs.
static struct place *places; // One for each part.
static struct alias *aliases;
static size_t naliases;
static struct later *laters;
static size_t nlaters;
static char group_path[LINE]; // The page and groups open: outputs.ch1.
static int depth, line_no;
static char where[LINE + 64];

static const int DX[4] = { 1, 0, -1, 0 }, DY[4] = { 0, 1, 0, -1 };
static int faults; // The faults of the source told so far.

// Tells a fault of the source and goes on.
#define fault(...) (sch_warn(__VA_ARGS__), faults++)

// Room: from a pin to what continues it (FIRST) or to its first branch
// (HANG), and the least from a branch to the next or to what continues
// past them (STEP); the labels widen it.  FAR is as far as a part is
// pushed to find room.
enum { FIRST = 2, HANG = 3, STEP = 2, BESIDE_OFF = 3, OF_GAP = 2, FAR = 80 };
enum { PAIR_GAP = 2 }; // Between parts in parallel, in grid units.

// Reads the room the drawing keeps from the clear records of the sheet
// template: clear body, text, label and flag, each in points.
static void read_clearances(struct sch *s)
{
	static const char *const what[] = { "body", "text", "label", "flag" };
	double *v[] = { &s->clear_body, &s->clear_text, &s->clear_label,
			&s->clear_flag };
	char *path = sch_sheet_path(s), buf[LINE], kind[64];
	int have[4] = { 0, 0, 0, 0 };
	FILE *f = path ? fopen(path, "r") : NULL;

	if (!f) {
		fault("%s: no sheet %s in the library", s->file, s->sheet);
		free(path);
		return;
	}
	while (fgets(buf, sizeof buf, f)) {
		double d;

		if (sscanf(buf, "clear %63s %lf", kind, &d) != 2) {
			continue;
		}
		for (int i = 0; i < 4; i++) {
			if (!strcmp(kind, what[i]) && d >= 0) {
				*v[i] = d;
				have[i] = 1;
			}
		}
	}
	fclose(f);
	for (int i = 0; i < 4; i++) {
		if (!have[i]) {
			fault("%s: no clear %s", path, what[i]);
		}
	}
	free(path);
}

static void at_line(int line)
{
	snprintf(where, sizeof where, "%s:%d", R->file, line);
}

static size_t root(size_t i)
{
	while (pins[i].up != i) {
		pins[i].up = pins[pins[i].up].up;
		i = pins[i].up;
	}
	return i;
}

static void join_pins(size_t a, size_t b)
{
	pins[a].used = pins[b].used = 1;
	pins[root(a)].up = root(b);
}

// The full name of name in the group open now.
static char *in_path(const char *name)
{
	char *s = sch_alloc(strlen(group_path) + strlen(name) + 2);

	sprintf(s, "%s%s%s", group_path, *group_path ? "." : "", name);
	return s;
}

static long part_named(const char *full)
{
	for (size_t i = 0; i < R->npart; i++) {
		if (strcmp(R->part[i].name, full) == 0) {
			return (long)i;
		}
	}
	return -1;
}

// Looks a name up in the group scope, then in each group round it, out
// to the page.  kind 0 finds a part, kind 1 a pin (PART.PIN or a port).
// Returns the part or the pin, -1 for a port not yet known, or dies.
static long lookup(const char *name, const char *scope, int kind)
{
	char prefix[LINE];

	snprintf(prefix, sizeof prefix, "%s", scope);
	for (;;) {
		char full[2 * LINE], obj[2 * LINE], *dot;
		long q;

		snprintf(full, sizeof full, "%s%s%s", prefix,
			 *prefix ? "." : "", name);
		if (kind == 0) {
			q = part_named(full);
			if (q >= 0) {
				return q;
			}
		} else {
			for (size_t i = 0; i < naliases; i++) {
				if (strcmp(aliases[i].name, full) == 0) {
					return aliases[i].pid;
				}
			}
			memcpy(obj, full, strlen(full) + 1);
			dot = strrchr(obj, '.');
			if (dot) {
				*dot++ = 0;
				q = part_named(obj);
				if (q >= 0) {
					const struct sch_part *r = &R->part[q];

					for (size_t p = 0;
					     p < r->dev->shape->npin; p++) {
						if (!strcmp(r->dev->shape
								    ->pin[p]
								    .name,
							    dot) ||
						    !strcmp(r->dev->pnum[p],
							    dot)) {
							return (long)(base[q] +
								      p);
						}
					}
					fault("%s: %s has no pin %s", where,
					      obj, dot);
					return -2;
				}
			}
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
	fault("%s: no %s %s here", where, kind ? "pin" : "part", name);
	return -2;
}

// The pin of a name, or -1 (the fault told).
static long pin_of(const char *name, const char *scope)
{
	long p = lookup(name, scope, 1);

	if (p == -1) {
		fault("%s: port %s leads in a circle", where, name);
	}
	return p < 0 ? -1 : p;
}

// Finds the pin table of a part number: a file of its name in lower
// case in a directory of the library (the maker's).  Returns its name
// in the library, or NULL.
static char *find_table(const char *value)
{
	char low[256];
	size_t i;

	for (i = 0; i < sizeof low - 1 && value[i]; i++) {
		low[i] = (char)tolower((unsigned char)value[i]);
		if (low[i] == '/') {
			return NULL;
		}
	}
	low[i] = 0;
	for (size_t k = 0; k < R->nlib; k++) {
		DIR *d = opendir(R->lib[k]);
		const struct dirent *e;

		if (!d) {
			continue;
		}
		while ((e = readdir(d)) != NULL) {
			char *name, *full;
			FILE *f;

			if (e->d_name[0] == '.') {
				continue;
			}
			name = sch_alloc(strlen(e->d_name) + strlen(low) + 2);
			sprintf(name, "%s/%s", e->d_name, low);
			full = sch_alloc(strlen(R->lib[k]) + strlen(name) + 2);
			sprintf(full, "%s/%s", R->lib[k], name);
			f = fopen(full, "r");
			free(full);
			if (f) {
				fclose(f);
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

static size_t add_part(const struct sch_part *q, const struct place *pl)
{
	size_t k = R->npart, n = q->dev->shape->npin;

	R->part = grow(R->part, k, sizeof *R->part);
	R->part[R->npart++] = *q;
	base = grow(base, k, sizeof *base);
	placed = grow(placed, k, sizeof *placed);
	dead = grow(dead, k, sizeof *dead);
	besides = grow(besides, k, sizeof *besides);
	anchor_of = grow(anchor_of, k, sizeof *anchor_of);
	along_of = grow(along_of, k, sizeof *along_of);
	places = grow(places, k, sizeof *places);
	base[k] = npins;
	anchor_of[k] = -1;
	places[k] = *pl;
	for (size_t p = 0; p < n; p++) {
		struct pinrec r;

		memset(&r, 0, sizeof r);
		r.part = k;
		r.pin = p;
		r.up = npins;
		r.line = q->line;
		r.flagpart = -1;
		pins = grow(pins, npins, sizeof *pins);
		pins[npins++] = r;
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
static int label_box_at(const struct sch_part *q, int want, double b[4])
{
	double w, h, bx[4], g = SCH_GRID;
	int side;

	if (q->dev->shape->flag) {
		int x, y, d, out;

		if (!q->text || (strcmp(q->dev->shape->flag, "supply") &&
				 strcmp(q->dev->shape->flag, "port") &&
				 strcmp(q->dev->shape->flag, "global"))) {
			return 0;
		}
		sch_pin_at(q, 0, &x, &y, &d);
		out = (d + 2) % 4;
		w = sch_text_width(q->text, SCH_TEXT) / g + 0.6;
		h = 1.2;
		b[0] = x + DX[out] * 0.4 -
		       (out == SCH_L ? w :
			out == SCH_R ? 0 :
				       w / 2);
		b[1] = y + DY[out] * 0.4 -
		       (out == SCH_D ? h :
			out == SCH_U ? 0 :
				       h / 2);
		b[2] = b[0] + w;
		b[3] = b[1] + h;
		return 1;
	}
	sch_label_size(q, &w, &h);
	w /= g;
	h /= g;
	sch_box(q, bx);
	side = want;
	if (side < 0) {
		side = q->dev->shape->side;
		if (q->mirror && (side == 0 || side == 2)) {
			side = 2 - side;
		}
		side = (side + q->rot) % 4;
	}
	if (side == 0 || side == 2) {
		b[0] = side == 0 ? bx[2] + 0.3 : bx[0] - 0.3 - w;
		b[1] = q->y - h / 2;
	} else {
		b[0] = q->x - w / 2;
		b[1] = side == 1 ? bx[3] + 0.4 : bx[1] - 0.4 - h;
	}
	b[2] = b[0] + w;
	b[3] = b[1] + h;
	return 1;
}

static int label_box(const struct sch_part *q, double b[4])
{
	return label_box_at(q, q->side, b);
}

// Whether the label of q, on the side, keeps clear of the parts placed
// (but skip) and their labels.
static int label_clear(const struct sch_part *q, int side, long skip)
{
	double la[4];

	if (!label_box_at(q, side, la)) {
		return 1;
	}
	for (size_t i = 0; i < R->npart; i++) {
		const struct sch_part *o = &R->part[i];
		double b[4], lb[4];

		if (!placed[i] || (long)i == skip || o == q) {
			continue;
		}
		sch_box(o, b);
		if (overlap(la, b) || (label_box(o, lb) && overlap(la, lb))) {
			return 0;
		}
		for (size_t p = 0; p < o->dev->shape->npin; p++) {
			int x, y, d;

			sch_pin_at(o, p, &x, &y, &d);
			if (in_box(la, x, y)) {
				return 0;
			}
		}
	}
	return 1;
}

// The side the label of q takes: its own if it is clear, else the
// first clear of the other three, or -1 if none is.
static int label_side(const struct sch_part *q, long skip)
{
	int own;

	if (q->side >= 0 || q->dev->shape->flag) {
		return q->side;
	}
	own = q->dev->shape->side;
	if (q->mirror && (own == 0 || own == 2)) {
		own = 2 - own;
	}
	own = (own + q->rot) % 4;
	for (int i = 0; i < 4; i++) {
		int s = i == 0 ? own :
			i == 1 ? (own + 2) % 4 :
				 (own + i - 1) % 4;

		if (label_clear(q, s, skip)) {
			return s;
		}
	}
	return -2;
}

// Whether part q, where it stands, keeps clear of the parts placed, and
// of skip: no body or label on a body, a pin or a label, and no pin on a
// pin or on the point just out from one.
static int clear(const struct sch_part *q, long skip)
{
	double a[4];

	if (label_side(q, skip) == -2) {
		return 0;
	}
	sch_box(q, a);
	a[0] -= 0.5;
	a[1] -= 0.5;
	a[2] += 0.5;
	a[3] += 0.5;
	for (size_t i = 0; i < R->npart; i++) {
		const struct sch_part *o = &R->part[i];
		double b[4], lb[4];
		int has_lb;

		if (!placed[i] || (long)i == skip) {
			continue;
		}
		sch_box(o, b);
		b[0] -= 0.5;
		b[1] -= 0.5;
		b[2] += 0.5;
		b[3] += 0.5;
		has_lb = label_box(o, lb);
		if (overlap(a, b) || (has_lb && overlap(a, lb))) {
			return 0;
		}
		for (size_t p = 0; p < o->dev->shape->npin; p++) {
			int x, y, d;

			sch_pin_at(o, p, &x, &y, &d);
			if (in_box(a, x, y)) {
				return 0;
			}
			for (size_t r = 0; r < q->dev->shape->npin; r++) {
				int x2, y2, d2;

				sch_pin_at(q, r, &x2, &y2, &d2);
				// A pin keeps the point out from it free, for
				// its wire or its flag.
				if ((x2 == x && y2 == y) || in_box(b, x2, y2) ||
				    (has_lb && in_box(lb, x2, y2)) ||
				    (x2 + DX[d2] == x && y2 + DY[d2] == y) ||
				    (x + DX[d] == x2 && y + DY[d] == y2) ||
				    (x + DX[d] == x2 + DX[d2] &&
				     y + DY[d] == y2 + DY[d2]) ||
				    in_box(b, x2 + DX[d2], y2 + DY[d2]) ||
				    in_box(a, x + DX[d], y + DY[d])) {
					return 0;
				}
			}
		}
	}
	return 1;
}

// Where the ground (or chassis) flag of a pin stands: one unit under a
// pin that points down, one out and one down from a pin that points
// aside.
static void ground_spot(size_t pid, int *fx, int *fy)
{
	int x, y, d;

	sch_pin_at(&R->part[pins[pid].part], pins[pid].pin, &x, &y, &d);
	if (d == SCH_U) {
		at_line(pins[pid].line);
		fault("%s: a %s on a pin that points up", where,
		      pins[pid].flag->flag);
	}
	*fx = x + (d == SCH_D ? 0 : DX[d]);
	*fy = y - 1;
}

// Whether part k stands clear where it is, with the ground flags of its
// pins where they would go.
static int clear_unit(size_t k)
{
	const struct sch_part *q = &R->part[k];

	if (!clear(q, -1)) {
		return 0;
	}
	for (size_t p = 0; p < q->dev->shape->npin; p++) {
		const struct pinrec *r = &pins[base[k] + p];
		struct sch_part *f;

		if (r->flagpart < 0 || (strcmp(r->flag->flag, "ground") &&
					strcmp(r->flag->flag, "chassis"))) {
			continue;
		}
		f = &R->part[r->flagpart];
		ground_spot(base[k] + p, &f->x, &f->y);
		if (!clear(f, (long)k)) {
			return 0;
		}
	}
	return 1;
}

// Marks part k placed, and its ground flags with it.
int sch_verbose;

// The traces of -v, kept until the designators are known: the part and
// what placed it.
static size_t *trace_part, ntraces;
static char **trace_text;

static void trace(size_t k, const char *text)
{
	trace_part = grow(trace_part, ntraces, sizeof *trace_part);
	trace_text = grow(trace_text, ntraces, sizeof *trace_text);
	trace_part[ntraces] = k;
	trace_text[ntraces++] = sch_strdup(text);
}

// Tells each placement, by the part's name and designator.
static void tell_traces(void)
{
	for (size_t i = 0; i < ntraces; i++) {
		const struct sch_part *q = &R->part[trace_part[i]];

		fprintf(stderr, "%s:%d: %s (%s) %s\n", R->file, q->line,
			q->name, q->ref ? q->ref : "-", trace_text[i]);
		free(trace_text[i]);
	}
	free(trace_text);
	free(trace_part);
	ntraces = 0;
}

static void commit(size_t k)
{
	int side = label_side(&R->part[k], -1);
	static const char *const how[] = {
		"first clear place to the right",
		"first clear place along the pin", "beside its twin",
		"next to its anchor", "past what stands between the pins"
	};

	if (sch_verbose) {
		char t[256];

		snprintf(t, sizeof t, "at %d %d, turned %d: %s", R->part[k].x,
			 R->part[k].y, R->part[k].rot * 90,
			 how[places[k].kind]);
		trace(k, t);
	}
	R->part[k].side = side >= 0 ? side : R->part[k].side;
	placed[k] = 1;
	R->part[k].placed = 1;
	for (size_t p = 0; p < R->part[k].dev->shape->npin; p++) {
		const struct pinrec *r = &pins[base[k] + p];

		if (r->flagpart >= 0 && (!strcmp(r->flag->flag, "ground") ||
					 !strcmp(r->flag->flag, "chassis"))) {
			ground_spot(base[k] + p, &R->part[r->flagpart].x,
				    &R->part[r->flagpart].y);
			placed[r->flagpart] = 1;
			R->part[r->flagpart].placed = 1;
		}
	}
}

// The pin of part q that faces back along dir: for two pins, the first,
// with the part turned to suit.
static size_t entry_pin(struct sch_part *q, int dir)
{
	const struct sch_shape *sh = q->dev->shape;
	int back = (dir + 2) % 4, ex, ey, ed;

	if (sh->npin == 2) {
		q->rot = (back - sh->pin[0].dir + 4) % 4;
		return 0;
	}
	q->x = q->y = 0;
	for (size_t p = 0; p < sh->npin; p++) {
		sch_pin_at(q, p, &ex, &ey, &ed);
		if (ed == back) {
			return p;
		}
	}
	fault("%s: %s has no pin facing %s", where, q->name,
	      dir == SCH_R ? "left" :
	      dir == SCH_L ? "right" :
	      dir == SCH_U ? "down" :
			     "up");
	return (size_t)-1;
}

// The pins on the net of pin pid past the two a line joins, up to 3:
// room for their junctions.
static int extra_pins(size_t pid)
{
	int n = 0;

	for (size_t i = 0; i < npins; i++) {
		n += pins[i].used && !pins[i].flag &&
		     !R->part[pins[i].part].dev->shape->flag &&
		     root(i) == root(pid);
	}
	return n - 2 < 0 ? 0 : n - 2 > 3 ? 3 : n - 2;
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

// Whether part q, standing with its lead from (lx, ly), keeps out of the
// loops by the room of a body (a branch, by more): each wire said in the source closes a loop through the parts
// it joins, and the box round its two pins, one unit out from each, and
// round the parts of its net and their labels, is kept for that net.  A part joined to the net of the wire may stand
// in it; a wire of another net may cross it.
static int out_of_loops(const struct sch_part *q, size_t k, int lx, int ly,
			int branch)
{
	double a[4], m = R->clear_body / SCH_GRID;

	// A branch hung from a lead keeps its whole line clear: its parts,
	// as thick as the thickest of two pins, by the room of a body.
	for (size_t i = 0; branch && i < R->npart; i++) {
		const struct sch_shape *s = R->part[i].dev->shape;

		if (s->npin == 2 && !s->flag) {
			m = fmax(m, R->clear_body / SCH_GRID +
					    fmin(s->box[2] - s->box[0],
						 s->box[3] - s->box[1]) /
						    2);
		}
	}
	sch_box(q, a);
	a[0] = fmin(a[0], lx) - m;
	a[1] = fmin(a[1], ly) - m;
	a[2] = fmax(a[2], lx) + m;
	a[3] = fmax(a[3], ly) + m;
	for (size_t i = 0; i < R->nhint; i++) {
		const struct sch_hint *h = &R->hint[i];
		size_t net = root(base[h->part[0]] + h->pin[0]);
		double b[4] = { 1e300, 1e300, -1e300, -1e300 };
		int on = 0;

		if (!placed[h->part[0]] || !placed[h->part[1]]) {
			continue;
		}
		for (size_t p = 0; p < q->dev->shape->npin; p++) {
			on |= root(base[k] + p) == net;
		}
		for (int e = 0; e < 2 && !on; e++) {
			int x, y, d;

			sch_pin_at(&R->part[h->part[e]], h->pin[e], &x, &y, &d);
			b[0] = fmin(b[0], fmin(x, x + DX[d]));
			b[1] = fmin(b[1], fmin(y, y + DY[d]));
			b[2] = fmax(b[2], fmax(x, x + DX[d]));
			b[3] = fmax(b[3], fmax(y, y + DY[d]));
		}
		// The loop takes in the parts of its net, and their labels.
		for (size_t j = 0; j < R->npart && !on; j++) {
			const struct sch_part *o = &R->part[j];
			double c[4];
			int of = 0;

			for (size_t p = 0; placed[j] && p < o->dev->shape->npin;
			     p++) {
				of |= root(base[j] + p) == net;
			}
			if (!of || o->dev->shape->flag) {
				continue;
			}
			sch_box(o, c);
			grow_box(b, c);
			if (label_box(o, c)) {
				grow_box(b, c);
			}
		}
		if (!on && overlap(a, b)) {
			return 0;
		}
	}
	return 1;
}

static int place_from(size_t k)
{
	struct sch_part *q = &R->part[k];
	const struct place *pl = &places[k];
	struct pinrec *a = &pins[pl->pid[0]];
	int px, py, pd, ex, ey, ed, dir = pl->dir;
	size_t entry = entry_pin(q, dir);

	if (entry == (size_t)-1) {
		return 0;
	}
	sch_pin_at(&R->part[a->part], a->pin, &px, &py, &pd);
	if (dir == (pd + 2) % 4) {
		fault("%s: a pin can lead only its own way or across", where);
		return 0;
	}
	if (dir == pd && a->series) {
		fault("%s: the line past %s goes on already", where,
		      R->part[a->part].name);
		return 0;
	}
	q->x = q->y = 0;
	sch_pin_at(q, entry, &ex, &ey, &ed);
	for (int off = a->reach	 ? a->reach + STEP :
		       dir == pd ? FIRST + extra_pins(pl->pid[0]) :
				   HANG;
	     ; off++) {
		int tx, ty;

		if (off > a->reach + FAR) {
			fault("%s: no room for %s", where, q->name);
			return 0;
		}
		if (dir == pd) {
			tx = px + DX[dir] * off;
			ty = py + DY[dir] * off;
		} else {
			tx = px + DX[pd] * off + DX[dir] * FIRST;
			ty = py + DY[pd] * off + DY[dir] * FIRST;
		}
		q->x = tx - ex;
		q->y = ty - ey;
		if (out_of_loops(q, k, px + DX[pd] * off, py + DY[pd] * off,
				 dir != pd) &&
		    clear_unit(k)) {
			a->reach = off;
			a->series = dir == pd;
			break;
		}
	}
	anchor_of[k] = (long)pl->pid[0];
	along_of[k] = a->reach;
	return 1;
}

// Places part k in parallel with another of its shape, turned as it is,
// each pin joined to the pin of the same name.
static int place_beside(size_t k)
{
	struct sch_part *q = &R->part[k];
	size_t b = places[k].part;
	const struct sch_part *o = &R->part[b];
	int x0, y0, x1, y1, d, dir = places[k].dir, off;

	int any;

	sch_pin_at(o, 0, &x0, &y0, &d);
	sch_pin_at(o, o->dev->shape->npin - 1, &x1, &y1, &d);
	any = dir < 0;
	if (any) {
		dir = y0 == y1 ? SCH_D : SCH_R;
	}
	q->rot = o->rot;
	q->mirror = o->mirror;
	q->flip = o->flip;
	// The first clear place out the way said, or either way across.
	for (off = BESIDE_OFF * (besides[b] + 1);; off++) {
		int found = 0;

		if (off > FAR) {
			fault("%s: no room for %s", where, q->name);
			return 0;
		}
		// Its own side up to three units on, then either.
		for (int t = 0;
		     t < 1 + (any && off > BESIDE_OFF * (besides[b] + 1) + 3) &&
		     !found;
		     t++) {
			int w = t ? (dir + 2) % 4 : dir;

			q->x = o->x + DX[w] * off;
			q->y = o->y + DY[w] * off;
			if (clear_unit(k)) {
				dir = w;
				found = 1;
			}
		}
		if (found) {
			besides[b] = off / BESIDE_OFF;
			break;
		}
	}
	// A branch beside a branch takes room on the lead it hangs from.
	if (anchor_of[b] >= 0) {
		struct pinrec *a = &pins[anchor_of[b]];
		int ax, ay, ad;

		sch_pin_at(&R->part[a->part], a->pin, &ax, &ay, &ad);
		if (dir == ad && along_of[b] + off > a->reach) {
			a->reach = along_of[b] + off;
		}
		anchor_of[k] = anchor_of[b];
		along_of[k] = along_of[b] + (dir == ad ? off : 0);
	}
	return 1;
}

// Places part k next to another without joining them: past its box the
// way said.
static int up_int(double v);
static int down_int(double v);

// The group of the name, from the scope out, as its path and a dot, if
// some part is in it.
static char *group_of(const char *name, const char *scope)
{
	size_t n = strlen(scope), len = strlen(name);
	char *g = sch_alloc(n + len + 3);

	for (;;) {
		memcpy(g, scope, n);
		g[n] = n ? '.' : 0;
		strcpy(g + (n ? n + 1 : 0), name);
		strcat(g, ".");
		for (size_t i = 0; i < R->npart; i++) {
			if (!strncmp(R->part[i].name, g, strlen(g))) {
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
static int place_of(size_t k)
{
	struct sch_part *q = &R->part[k];
	const struct place *pl = &places[k];
	int dir = pl->dir;
	double b[4] = { 1e300, 1e300, -1e300, -1e300 }, c[4];
	const struct sch_part *o = pl->group ? NULL : &R->part[pl->part];
	int ox, oy;

	if (pl->group) {
		// The name past as many groups as the group's path holds.
		const char *rest = q->name;
		char *same = sch_alloc(strlen(pl->group) + strlen(q->name) + 1);

		for (const char *g = pl->group; *g && rest; g++) {
			if (*g == '.') {
				rest = strchr(rest, '.');
				rest = rest ? rest + 1 : NULL;
			}
		}
		sprintf(same, "%s%s", pl->group, rest ? rest : q->name);
		for (size_t i = 0; i < R->npart; i++) {
			if (placed[i] && !strncmp(R->part[i].name, pl->group,
						  strlen(pl->group))) {
				sch_box(&R->part[i], c);
				grow_box(b, c);
				if (label_box(&R->part[i], c)) {
					grow_box(b, c);
				}
				o = strcmp(R->part[i].name, same) ? o :
								    &R->part[i];
			}
		}
		free(same);
		if (b[2] < b[0]) {
			fault("%s: no part of %s is placed", where, pl->ref[0]);
			return 0;
		}
	} else {
		sch_box(o, b);
	}
	ox = o ? o->x : (int)((b[0] + b[2]) / 2);
	oy = o ? o->y : (int)((b[1] + b[3]) / 2);
	for (int off = OF_GAP;; off++) {
		if (off > FAR) {
			fault("%s: no room for %s", where, q->name);
			return 0;
		}
		// Past the box, by how far the group reaches beyond its part.
		q->x = dir == SCH_R ?
			       up_int(b[2]) + off +
				       (pl->group ? ox - down_int(b[0]) : 0) :
		       dir == SCH_L ?
			       down_int(b[0]) - off -
				       (pl->group ? up_int(b[2]) - ox : 0) :
			       ox;
		q->y = dir == SCH_U ?
			       up_int(b[3]) + off +
				       (pl->group ? oy - down_int(b[1]) : 0) :
		       dir == SCH_D ?
			       down_int(b[1]) - off -
				       (pl->group ? up_int(b[3]) - oy : 0) :
			       oy;
		if (pl->gap >= 0) {
			sch_box(q, c);
			if ((dir == SCH_R ? c[0] - b[2] :
			     dir == SCH_L ? b[0] - c[2] :
			     dir == SCH_U ? c[1] - b[3] :
					    b[1] - c[3]) < pl->gap - 1e-9) {
				continue;
			}
		}
		if (clear_unit(k)) {
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
static int place_between(size_t k)
{
	struct sch_part *q = &R->part[k];
	const struct place *pl = &places[k];
	int x[2], y[2], d, dir = pl->dir, along;
	double lo, hi, edge = dir == SCH_U || dir == SCH_R ? -1e9 : 1e9;
	size_t first;

	if (q->dev->shape->npin != 2) {
		fault("%s: only a part of two pins goes between", where);
		return 0;
	}
	for (int i = 0; i < 2; i++) {
		sch_pin_at(&R->part[pins[pl->pid[i]].part],
			   pins[pl->pid[i]].pin, &x[i], &y[i], &d);
	}
	along = dir == SCH_U || dir == SCH_D ? SCH_R : SCH_U;
	lo = along == SCH_R ? (x[0] < x[1] ? x[0] : x[1]) :
			      (y[0] < y[1] ? y[0] : y[1]);
	hi = along == SCH_R ? (x[0] > x[1] ? x[0] : x[1]) :
			      (y[0] > y[1] ? y[0] : y[1]);
	// The farthest out of what lies across the span, its labels too.
	for (size_t i = 0; i < R->npart; i++) {
		double b[4], l[4];
		int has;

		if (!placed[i]) {
			continue;
		}
		sch_box(&R->part[i], b);
		has = label_box(&R->part[i], l);
		for (int pass = 0; pass < 1 + has; pass++) {
			const double *e = pass ? l : b;
			double s0 = along == SCH_R ? e[0] : e[1];
			double s1 = along == SCH_R ? e[2] : e[3];

			if (s1 < lo - 1 || s0 > hi + 1) {
				continue;
			}
			if (dir == SCH_U) {
				edge = e[3] > edge ? e[3] : edge;
			} else if (dir == SCH_D) {
				edge = e[1] < edge ? e[1] : edge;
			} else if (dir == SCH_R) {
				edge = e[2] > edge ? e[2] : edge;
			} else {
				edge = e[0] < edge ? e[0] : edge;
			}
		}
		for (size_t p = 0; p < R->part[i].dev->shape->npin; p++) {
			int px, py, pd;
			double s, t;

			sch_pin_at(&R->part[i], p, &px, &py, &pd);
			s = along == SCH_R ? px : py;
			t = along == SCH_R ? py : px;
			if (s >= lo - 1 && s <= hi + 1) {
				edge = dir == SCH_U || dir == SCH_R ?
					       (t > edge ? t : edge) :
					       (t < edge ? t : edge);
			}
		}
	}
	first = (along == SCH_R ? x[0] <= x[1] : y[0] <= y[1]) ? 0 : 1;
	q->rot = along == SCH_R ? 1 : 0; // Pin 1 left, or on top.
	for (int off = 2;; off++) {
		int c = (int)((lo + hi) / 2), e;

		if (off > FAR) {
			fault("%s: no room for %s", where, q->name);
			return 0;
		}
		e = dir == SCH_U || dir == SCH_R ? up_int(edge) + off :
						   down_int(edge) - off;
		q->x = along == SCH_R ? c : e;
		q->y = along == SCH_R ? e : c;
		if (clear_unit(k)) {
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
static int right_edge(void)
{
	double x1 = -1e9;

	for (size_t i = 0; i < R->npart; i++) {
		double b[4];

		if (placed[i]) {
			sch_box(&R->part[i], b);
			x1 = b[2] > x1 ? b[2] : x1;
		}
	}
	return up_int(x1);
}

// The order the placements wait in: branches and parallels first, then
// what continues a line, then parts placed next to others, parts placed
// between, and last parts placed against nothing.
static int rank(size_t k)
{
	const struct place *pl = &places[k];

	if (pl->kind == FROM) {
		int x, y, d;
		const struct pinrec *a = &pins[pl->pid[0]];

		sch_pin_at(&R->part[a->part], a->pin, &x, &y, &d);
		return pl->dir == d ? 1 : 0;
	}
	return pl->kind == BESIDE  ? 0 :
	       pl->kind == OF	   ? 2 :
	       pl->kind == BETWEEN ? 3 :
				     4;
}

static int ready(size_t k)
{
	const struct place *pl = &places[k];

	switch (pl->kind) {
	case FROM:
		return placed[pins[pl->pid[0]].part];
	case OF:
		for (size_t i = 0; pl->group && i < R->npart; i++) {
			if (i != k && !placed[i] && !dead[i] &&
			    !strncmp(R->part[i].name, pl->group,
				     strlen(pl->group))) {
				return 0;
			}
		}
		return pl->group || placed[pl->part];
	case BESIDE:
		return placed[pl->part];
	case BETWEEN:
		return placed[pins[pl->pid[0]].part] &&
		       placed[pins[pl->pid[1]].part];
	default:
		return 1;
	}
}

// Places the parts that are ready, the best ranked first, but not
// those placed between pins unless between is set.
static void place_ready(int between)
{
	for (;;) {
		long best = -1;
		int ok = 1;

		for (size_t k = 0; k < R->npart; k++) {
			if (placed[k] || dead[k] ||
			    R->part[k].dev->shape->flag || !ready(k) ||
			    (places[k].kind == BETWEEN) != between) {
				continue;
			}
			if (best < 0 || rank(k) < rank((size_t)best)) {
				best = (long)k;
			}
		}
		if (best < 0) {
			return;
		}
		at_line(places[best].line);
		switch (places[best].kind) {
		case FROM:
			ok = place_from((size_t)best);
			break;
		case BESIDE:
			ok = place_beside((size_t)best);
			break;
		case OF:
			ok = place_of((size_t)best);
			break;
		case BETWEEN:
			ok = place_between((size_t)best);
			break;
		default: {
			struct sch_part *q = &R->part[best];
			int any = 0;

			for (size_t i = 0; i < R->npart; i++) {
				any |= placed[i];
			}
			q->x = any ? right_edge() + 6 : 0;
			q->y = 0;
			while (!clear_unit((size_t)best)) {
				q->x++;
			}
			break;
		}
		}
		if (ok) {
			commit((size_t)best);
		} else {
			dead[best] = 1; // Told, and left out of the drawing.
		}
	}
}

// Places the supplies and the ports of the sheet, past the branches of
// their pins, on the parts placed so far.
static void place_flags(void)
{
	for (size_t i = 0; i < npins; i++) {
		struct pinrec *r = &pins[i];
		struct sch_part *f;
		int x, y, d, off;

		if (r->flagpart < 0 || placed[r->flagpart] ||
		    !placed[r->part]) {
			continue;
		}
		f = &R->part[r->flagpart];
		sch_pin_at(&R->part[r->part], r->pin, &x, &y, &d);
		f->rot = ((d + 2) % 4 - f->dev->shape->pin[0].dir + 4) % 4;
		for (off = r->reach			  ? r->reach + STEP :
			   !strcmp(r->flag->flag, "port") ? 3 :
							    1;
		     ; off++) {
			f->x = x + DX[d] * off;
			f->y = y + DY[d] * off;
			if (clear(f, (long)r->part) || off > r->reach + FAR) {
				break;
			}
		}
		placed[r->flagpart] = 1;
		f->placed = 1;
	}
}

// Places all: the parts against others, the flags at the ends of
// lines, then the parts between pins, which keep out past them, and the
// flags of those.
// Moves part k, and the flags of its pins, by (dx, dy).
static void move_unit(size_t k, int dx, int dy)
{
	R->part[k].x += dx;
	R->part[k].y += dy;
	for (size_t p = 0; p < R->part[k].dev->shape->npin; p++) {
		long f = pins[base[k] + p].flagpart;

		if (f >= 0) {
			R->part[f].x += dx;
			R->part[f].y += dy;
		}
	}
}

// Whether part k, placed and then moved, stands clear of the others,
// its flags too: as clear_unit, the part and its flags out of the
// placed while they are tried.
static int clear_moved(size_t k)
{
	int ok;

	placed[k] = 0;
	for (size_t p = 0; p < R->part[k].dev->shape->npin; p++) {
		long f = pins[base[k] + p].flagpart;

		if (f >= 0) {
			placed[f] = 0;
		}
	}
	ok = clear_unit(k);
	placed[k] = 1;
	for (size_t p = 0; p < R->part[k].dev->shape->npin; p++) {
		long f = pins[base[k] + p].flagpart;

		if (f >= 0) {
			placed[f] = 1;
		}
	}
	return ok;
}

// Lays each part and the parts beside it as an even row in parallel
// about the line the part was placed on: PAIR_GAP apart, the row's
// middle on the line, so no one of two stands on it; and joins their
// pins of a kind by wires, the bars, which the line meets in their
// middle.  If the row is not clear, it stays as it was placed.
static void center_besides(void)
{
	size_t *row = sch_alloc((R->npart + 1) * sizeof *row);

	for (size_t b = 0; b < R->npart; b++) {
		size_t n = 1;
		int across = 0, ok = 1, line;
		int *was;

		row[0] = b;
		for (size_t k = 0; placed[b] && k < R->npart; k++) {
			if (placed[k] && places[k].kind == BESIDE &&
			    places[k].part == b) {
				row[n++] = k;
			}
		}
		if (n < 2) {
			continue;
		}
		// Across the line: along x if the twin stands beside it so.
		across = R->part[row[1]].x != R->part[b].x;
		// In the order they stand, from low to high across the line.
		for (size_t i = 1; i < n; i++) {
			for (size_t j = i; j > 0; j--) {
				const struct sch_part *p = &R->part[row[j - 1]];
				const struct sch_part *q = &R->part[row[j]];

				if ((across ? q->x - p->x : q->y - p->y) >= 0) {
					break;
				}
				size_t t = row[j - 1];

				row[j - 1] = row[j];
				row[j] = t;
			}
		}
		was = sch_alloc(2 * n * sizeof *was);
		line = across ? R->part[b].x : R->part[b].y;
		for (size_t i = 0; i < n; i++) {
			const struct sch_part *q = &R->part[row[i]];
			int to = line +
				 (2 * (int)i - (int)(n - 1)) * PAIR_GAP / 2;
			int d = to - (across ? q->x : q->y);

			was[2 * i] = q->x;
			was[2 * i + 1] = q->y;
			move_unit(row[i], across ? d : 0, across ? 0 : d);
		}
		for (size_t i = 0; i < n && ok; i++) {
			ok = clear_moved(row[i]);
		}
		for (size_t i = 0; i < n && !ok; i++) {
			const struct sch_part *q = &R->part[row[i]];

			move_unit(row[i], was[2 * i] - q->x,
				  was[2 * i + 1] - q->y);
		}
		for (size_t i = 0; ok && i + 1 < n; i++) {
			for (size_t p = 0; p < R->part[b].dev->shape->npin;
			     p++) {
				struct sch_hint h = { { row[i], row[i + 1] },
						      { p, p },
						      -1 };

				R->hint = grow(R->hint, R->nhint,
					       sizeof *R->hint);
				R->hint[R->nhint++] = h;
			}
		}
		if (ok && sch_verbose) {
			trace(b, "its parts in parallel about its line");
		}
		free(was);
	}
	free(row);
}

static void place_all(void)
{
	place_ready(0);
	place_flags();
	place_ready(1);
	place_flags();
	center_besides();
	for (size_t k = 0; k < R->npart; k++) {
		if (!placed[k] && !dead[k] && !R->part[k].dev->shape->flag) {
			at_line(places[k].line);
			fault("%s: %s waits on parts that are never placed",
			      where, R->part[k].name);
		}
	}
}

// Reads a part statement: KIND NAME [VALUE] [rating R] [mpn M]
// [side WAY] [flip] [WAY from PIN | beside PART [WAY] | WAY of PART |
// between PIN and PIN WAY].
// Whether a value is as sch(5) has it for the kind: a resistor's, a
// capacitor's or an inductor's a number with its capital unit letter
// right after it (100R, 10.0K, 120P, 1.5U); any other kind's anything.
static int value_ok(const char *kind, const char *v)
{
	const char *units = !strcmp(kind, "resistor")  ? "RKM" :
			    !strcmp(kind, "capacitor") ? "PNU" :
			    !strcmp(kind, "inductor")  ? "NU" :
							 NULL;
	size_t n = strspn(v, "0123456789.");

	return !units ||
	       (n && strspn(v, "0123456789") && v[n] && strchr(units, v[n]) &&
		!v[n + 1] && strchr(v, '.') == strrchr(v, '.'));
}

static void read_part(struct sch_shape *sh, char **w, int n)
{
	struct sch_part q;
	struct place pl;
	int i;
	const char *value = NULL;
	char *table = NULL;

	memset(&q, 0, sizeof q);
	memset(&pl, 0, sizeof pl);
	q.side = -1;
	q.line = line_no;
	pl.kind = ROOT;
	pl.dir = -1;
	pl.gap = -1;
	pl.line = line_no;
	pl.scope = sch_strdup(group_path);
	if (n < 2) {
		fault("%s: %s NAME ...", where, w[0]);
		return;
	}
	q.name = in_path(w[1]);
	if (part_named(q.name) >= 0) {
		fault("%s: part %s again", where, q.name);
		return;
	}
	for (i = 2; i < n; i++) {
		int d = dir_word(w[i]);

		if (!strcmp(w[i], "flip")) {
			q.flip = 1;
		} else if (i + 1 >= n &&
			   (d >= 0 || !strcmp(w[i], "rating") ||
			    !strcmp(w[i], "mpn") || !strcmp(w[i], "side") ||
			    !strcmp(w[i], "beside") ||
			    !strcmp(w[i], "between"))) {
			fault("%s: %s needs a word after it", where, w[i]);
			return;
		} else if (d >= 0 && i + 2 < n &&
			   (!strcmp(w[i + 1], "from") ||
			    !strcmp(w[i + 1], "of"))) {
			pl.kind = w[i + 1][0] == 'f' ? FROM : OF;
			pl.dir = d;
			pl.ref[0] = sch_strdup(w[i + 2]);
			i += 2;
			// WAY of PART N: N grid steps between the boxes.
			if (pl.kind == OF && i + 1 < n &&
			    strspn(w[i + 1], "0123456789") ==
				    strlen(w[i + 1]) &&
			    integer(w[i + 1], &pl.gap)) {
				i++;
			}
		} else if (!strcmp(w[i], "beside")) {
			pl.kind = BESIDE;
			pl.ref[0] = sch_strdup(w[++i]);
			if (i + 1 < n && dir_word(w[i + 1]) >= 0) {
				pl.dir = dir_word(w[++i]);
			}
		} else if (!strcmp(w[i], "between")) {
			if (i + 4 >= n || strcmp(w[i + 2], "and") ||
			    dir_word(w[i + 4]) < 0) {
				fault("%s: between PIN and PIN WAY", where);
				return;
			}
			pl.kind = BETWEEN;
			pl.ref[0] = sch_strdup(w[i + 1]);
			pl.ref[1] = sch_strdup(w[i + 3]);
			pl.dir = dir_word(w[i + 4]);
			i += 4;
		} else if (!strcmp(w[i], "rating")) {
			q.rating = sch_strdup(w[++i]);
		} else if (!strcmp(w[i], "mpn")) {
			q.mpn = sch_strdup(w[++i]);
		} else if (!strcmp(w[i], "side")) {
			q.side = side_of(w[++i]);
			if (q.side < 0) {
				fault("%s: side is right, above, left or below",
				      where);
				return;
			}
		} else if (!value && d < 0) {
			value = w[i];
		} else {
			fault("%s: what is %s?", where, w[i]);
			return;
		}
	}
	if (value && !value_ok(sh->name, value)) {
		fault("%s: a %s's value is a number and its unit letter, %s",
		      where, sh->name,
		      !strcmp(sh->name, "resistor")  ? "R, K or M: 100R" :
		      !strcmp(sh->name, "capacitor") ? "P, N or U: 120P" :
						       "N or U: 1.5U");
		return;
	}
	if (value) {
		q.value = sch_strdup(value);
		table = find_table(value);
	}
	if (table) {
		q.dev = device(R->lib, R->nlib, table, where);
		free(table);
		if (q.dev->shape != sh) {
			fault("%s: %s is drawn as %s, not %s", where, value,
			      q.dev->shape->name, sh->name);
			return;
		}
	} else {
		q.dev = device(R->lib, R->nlib, sh->name, where);
		for (size_t p = 0; p < sh->npin; p++) {
			if (!isdigit((unsigned char)sh->pin[p].name[0])) {
				fault("%s: a %s needs a part number with a pin "
				      "table",
				      where, sh->name);
				return;
			}
		}
	}
	if (q.flip && sh->flip[0] < 0) {
		q.mirror = 1; // A shape without a flip pair is mirrored.
		q.flip = 0;
	}
	add_part(&q, &pl);
}

// Whether the library has a symbol of the name.
static int is_symbol(const char *name)
{
	char *file = sch_alloc(strlen(name) + 5), *path;
	int found;

	sprintf(file, "%s.sym", name);

	path = sch_find(R->lib, R->nlib, file);
	found = path != NULL;
	free(file);
	free(path);
	return found;
}

static void read_statement(char **w, int n)
{
	struct sch_shape *sh;

	if (!strcmp(w[0], "group") && n == 2) {
		size_t len = strlen(group_path);

		if (strchr(w[1], '.')) {
			fault("%s: a name has no dot", where);
			return;
		}
		if (len + strlen(w[1]) + 2 > sizeof group_path) {
			fault("%s: the groups nest too deep", where);
			return;
		}
		if (len) {
			group_path[len++] = '.';
		}
		strcpy(group_path + len, w[1]);
		depth++;
	} else if (!strcmp(w[0], "end") && n == 1) {
		char *dot = strrchr(group_path, '.');

		if (!depth) {
			fault("%s: end of no group", where);
			return;
		}
		if (dot) {
			*dot = 0;
		} else {
			*group_path = 0;
		}
		depth--;
	} else if (!strcmp(w[0], "note")) {
		// For the people and the agents that read the source, and
		// for the viewer, which shows it by its group.
		struct sch_note nt;

		nt.group = depth ? sch_strdup(group_path) : NULL;
		nt.text = join(w, 1, n);
		nt.line = line_no;
		R->note = grow(R->note, R->nnote, sizeof *R->note);
		R->note[R->nnote++] = nt;
	} else if (!strcmp(w[0], "port") && n == 3) {
		struct alias al;

		// A port of the sheet is named alike on every page.
		al.name = depth ? in_path(w[1]) : sch_strdup(w[1]);
		al.ref = sch_strdup(w[2]);
		al.scope = sch_strdup(group_path);
		al.depth = depth;
		al.line = line_no;
		al.pid = -1;
		// A port of the sheet may stand at several pins: each is drawn,
		// and they are one net; a port of a group stands for one pin.
		for (size_t i = 0; depth && i < naliases; i++) {
			if (!strcmp(aliases[i].name, al.name)) {
				fault("%s: port %s again", where, al.name);
				return;
			}
		}
		aliases = grow(aliases, naliases, sizeof *aliases);
		aliases[naliases++] = al;
	} else if (!strcmp(w[0], "nc") && n == 2) {
		// A pin left unconnected on purpose.
		struct later l;

		l.kind = NC;
		l.side = -1;
		l.ref[0] = sch_strdup(w[1]);
		l.ref[1] = NULL;
		l.name = NULL;
		l.scope = sch_strdup(group_path);
		l.line = line_no;
		laters = grow(laters, nlaters, sizeof *laters);
		laters[nlaters++] = l;
	} else if (!strcmp(w[0], "wire") && (n == 4 || n == 5) &&
		   !strcmp(w[2], "to")) {
		struct later l;

		l.kind = WIRE;
		l.side = n == 5 ? dir_word(w[4]) : -1;
		if (n == 5 && l.side < 0) {
			fault("%s: a wire keeps to the left, right, up or down",
			      where);
			return;
		}
		l.ref[0] = sch_strdup(w[1]);
		l.ref[1] = sch_strdup(w[3]);
		l.scope = sch_strdup(group_path);
		l.name = NULL;
		l.line = line_no;
		laters = grow(laters, nlaters, sizeof *laters);
		laters[nlaters++] = l;
	} else if (((!strcmp(w[0], "ground") || !strcmp(w[0], "chassis")) &&
		    n == 3 && !strcmp(w[1], "at")) ||
		   ((!strcmp(w[0], "supply") || !strcmp(w[0], "global")) &&
		    n == 4 && !strcmp(w[2], "at"))) {
		struct later l;

		l.kind = FLAG;
		l.side = -1;
		l.ref[0] = sch_strdup(w[n - 1]);
		l.ref[1] = sch_strdup(w[0]);
		l.name = sch_strdup(n == 4		    ? w[1] :
				    !strcmp(w[0], "ground") ? "GND" :
							      "CHASSIS");
		l.scope = sch_strdup(group_path);
		l.line = line_no;
		laters = grow(laters, nlaters, sizeof *laters);
		laters[nlaters++] = l;
	} else if (is_symbol(w[0]) &&
		   (sh = shape(R->lib, R->nlib, w[0], where)) != NULL &&
		   !sh->flag) {
		read_part(sh, w, n);
	} else {
		fault("%s: what is %s?", where, w[0]);
	}
}

// Puts a flag on a pin: a part of the flag's shape, and a global name.
static void flag_pin(size_t pid, const char *flag, const char *name)
{
	struct pinrec *r = &pins[pid];
	struct sch_part f;
	struct place pl;
	size_t k;

	if (r->flag) {
		fault("%s: the pin has a flag already", where);
		return;
	}
	memset(&f, 0, sizeof f);
	memset(&pl, 0, sizeof pl);
	r->flag = shape(R->lib, R->nlib, flag, where);
	f.dev = device(R->lib, R->nlib, flag, where);
	f.name = sch_strdup(name);
	f.text = f.name;
	f.side = -1;
	f.line = line_no;
	pl.kind = ROOT;
	pl.gap = -1;
	k = add_part(&f, &pl);
	r = &pins[pid];
	r->flagpart = (long)k;
	r->global = f.name;
	r->used = 1;
	pins[base[k]].used = 1;
	join_pins(base[k], pid);
	for (size_t i = 0; i < npins; i++) {
		if (i != pid && pins[i].global &&
		    !strcmp(pins[i].global, name) && pins[i].flagpart >= 0) {
			join_pins(pid, i);
			break;
		}
	}
}

// Looks up every name, now that all are known: the ports (which may
// stand for one another), the anchors of the parts, the wires and the
// flags.
static void resolve_all(void)
{
	// The ports, which may stand for one another: -2 marks one whose
	// name is not found (told once).
	for (int more = 1; more;) {
		more = 0;
		for (size_t i = 0; i < naliases; i++) {
			long p;

			if (aliases[i].pid != -1) {
				continue;
			}
			at_line(aliases[i].line);
			p = lookup(aliases[i].ref, aliases[i].scope, 1);
			if (p != -1) {
				aliases[i].pid = p;
				if (p >= 0) {
					pins[p].used = 1;
				}
				more = 1;
			}
		}
	}
	for (size_t i = 0; i < naliases; i++) {
		if (aliases[i].pid == -1) {
			at_line(aliases[i].line);
			fault("%s: port %s leads in a circle", where,
			      aliases[i].name);
		}
	}
	// The anchors of the parts, and the joins they make; a part whose
	// anchor is not found is left out.
	for (size_t k = 0; k < R->npart; k++) {
		struct place *pl = &places[k];
		long a = 0, b = 0;

		at_line(pl->line);
		if (pl->kind == FROM || pl->kind == BETWEEN) {
			a = pin_of(pl->ref[0], pl->scope);
		}
		if (pl->kind == BETWEEN) {
			b = pin_of(pl->ref[1], pl->scope);
		}
		if (pl->kind == OF &&
		    (pl->group = group_of(pl->ref[0], pl->scope)) != NULL) {
			continue;
		}
		if (pl->kind == BESIDE || pl->kind == OF) {
			a = lookup(pl->ref[0], pl->scope, 0);
		}
		if (a < 0 || b < 0) {
			dead[k] = 1;
			continue;
		}
		if (pl->kind == FROM) {
			size_t e = entry_pin(&R->part[k], pl->dir);

			pl->pid[0] = (size_t)a;
			if (e == (size_t)-1) {
				dead[k] = 1;
				continue;
			}
			join_pins(base[k] + e, pl->pid[0]);
		} else if (pl->kind == BESIDE || pl->kind == OF) {
			pl->part = (size_t)a;
			if (pl->kind == OF) {
				continue;
			}
			if (R->part[pl->part].dev->shape !=
			    R->part[k].dev->shape) {
				fault("%s: a part beside another is of its shape",
				      where);
				dead[k] = 1;
				continue;
			}
			for (size_t p = 0; p < R->part[k].dev->shape->npin;
			     p++) {
				join_pins(base[k] + p, base[pl->part] + p);
			}
		} else if (pl->kind == BETWEEN) {
			pl->pid[0] = (size_t)a;
			pl->pid[1] = (size_t)b;
			if (R->part[k].dev->shape->npin != 2) {
				fault("%s: only a part of two pins goes between",
				      where);
				dead[k] = 1;
				continue;
			}
			join_pins(base[k], pl->pid[0]);
			join_pins(base[k] + 1, pl->pid[1]);
		}
	}
	// The wires and the flags; one whose pins are not found is left out.
	for (size_t i = 0; i < nlaters; i++) {
		const struct later *l = &laters[i];
		long a, b = 0;

		at_line(l->line);
		line_no = l->line;
		a = pin_of(l->ref[0], l->scope);
		if (l->kind == WIRE) {
			b = pin_of(l->ref[1], l->scope);
		}
		if (a < 0 || b < 0) {
			continue;
		}
		if (l->kind == WIRE) {
			struct sch_hint h;

			join_pins((size_t)a, (size_t)b);
			h.part[0] = pins[a].part;
			h.pin[0] = pins[a].pin;
			h.part[1] = pins[b].part;
			h.pin[1] = pins[b].pin;
			h.side = l->side;
			R->hint = grow(R->hint, R->nhint, sizeof *R->hint);
			R->hint[R->nhint++] = h;
		} else if (l->kind == NC) {
			struct sch_part *q = &R->part[pins[a].part];

			if (!q->nc) {
				q->nc = sch_alloc(q->dev->shape->npin + 1);
			}
			q->nc[pins[a].pin] = 1;
		} else {
			flag_pin((size_t)a, l->ref[1], l->name);
		}
	}
	// The ports of the sheet are drawn, as flags.
	for (size_t i = 0; i < naliases; i++) {
		if (!aliases[i].depth && aliases[i].pid >= 0) {
			at_line(aliases[i].line);
			line_no = aliases[i].line;
			flag_pin((size_t)aliases[i].pid, "port",
				 aliases[i].name);
		}
	}
}

// Leaves the value off each ferrite of the kind the page uses most; its
// designator stays.
static void common_ferrite(void)
{
	size_t best = 0, most = 0;

	for (size_t i = 0; i < R->npart; i++) {
		size_t n = 0;
		const struct sch_part *q = &R->part[i];

		if (strcmp(q->dev->shape->name, "ferrite")) {
			continue;
		}
		for (size_t j = 0; j < R->npart; j++) {
			const struct sch_part *o = &R->part[j];

			n += !strcmp(o->dev->shape->name, "ferrite") &&
			     !strcmp(o->value ? o->value : "",
				     q->value ? q->value : "") &&
			     !strcmp(o->mpn ? o->mpn : "",
				     q->mpn ? q->mpn : "");
		}
		if (n > most) {
			most = n;
			best = i;
		}
	}
	for (size_t j = 0; most && j < R->npart; j++) {
		const struct sch_part *q = &R->part[best];
		struct sch_part *o = &R->part[j];

		if (!strcmp(o->dev->shape->name, "ferrite") &&
		    !strcmp(o->value ? o->value : "",
			    q->value ? q->value : "") &&
		    !strcmp(o->mpn ? o->mpn : "", q->mpn ? q->mpn : "")) {
			o->novalue = 1;
		}
	}
}

// Makes the nets: the pins joined, each set that has two pins or a flag.
// A net takes its global name, or the name of its outermost port, or the
// name of its first pin.  Each pin with a flag and the flag make a group
// of their own; the other pins make one group with the first of them,
// wherever it stands among them.
static void make_nets(void)
{
	size_t *done = sch_alloc((npins + 1) * sizeof *done);
	int group = 0;

	for (size_t i = 0; i < npins; i++) {
		size_t r = root(i), n = 0;
		struct sch_net t;
		const char *name = NULL;
		int best = 1 << 30, flagged = 0, rest = -1, rflag = 0;

		if (!pins[i].used || done[r]) {
			continue;
		}
		done[r] = 1;
		memset(&t, 0, sizeof t);
		t.line = pins[i].line;
		for (size_t j = 0; j < npins; j++) {
			const struct pinrec *pj = &pins[j];
			int g;

			if (!pj->used || root(j) != r) {
				continue;
			}
			n++;
			flagged |= pj->flag != NULL;
			if (pj->global) {
				if (t.global && strcmp(name, pj->global)) {
					fault("%s: nets %s and %s are joined",
					      R->file, name, pj->global);
				}
				name = pj->global;
				t.global = 1;
			}
			for (size_t k = 0; !t.global && k < naliases; k++) {
				if (aliases[k].pid == (long)j &&
				    aliases[k].depth < best) {
					best = aliases[k].depth;
					name = aliases[k].name;
				}
			}
			if (R->part[pj->part].dev->shape->flag) {
				continue; // With the pin it flags, below.
			}
			if (pj->flag && rflag) {
				g = group++;
			} else if (pj->flag) {
				// The first pin with a flag joins the other
				// pins, those before it too.
				if (rest < 0) {
					rest = group++;
				}
				g = rest;
				rflag = 1;
			} else {
				if (rest < 0) {
					rest = group++;
				}
				g = rest;
			}
			t.end = grow(t.end, t.nend, sizeof *t.end);
			t.end[t.nend].part = pj->part;
			t.end[t.nend].pin = pj->pin;
			t.end[t.nend].flag = pj->flag;
			t.end[t.nend].group = g;
			t.nend++;
			if (pj->flagpart >= 0) {
				t.end = grow(t.end, t.nend, sizeof *t.end);
				t.end[t.nend].part = (size_t)pj->flagpart;
				t.end[t.nend].pin = 0;
				t.end[t.nend].flag = NULL;
				t.end[t.nend].group = g;
				t.nend++;
			}
		}
		if (n < 2 && !flagged) {
			free(t.end);
			continue;
		}
		if (!name) {
			const struct sch_part *q = &R->part[pins[i].part];
			char *s = sch_alloc(strlen(q->name) + 64);

			sprintf(s, "%s.%s", q->name,
				q->dev->shape->pin[pins[i].pin].name);
			t.name = s;
		} else {
			t.name = sch_strdup(name);
		}
		R->net = grow(R->net, R->nnet, sizeof *R->net);
		R->net[R->nnet++] = t;
	}
	free(done);
}

// Gives each part its designator: its prefix and the page number times
// 100 plus its count among the parts of that prefix.
static void designate(struct sch *s)
{
	for (size_t i = 0; i < s->npart; i++) {
		struct sch_part *q = &s->part[i];
		int k = 1;

		if (q->dev->shape->flag) {
			continue;
		}
		for (size_t j = 0; j < i; j++) {
			const struct sch_part *o = &s->part[j];

			k += !o->dev->shape->flag &&
			     !strcmp(o->dev->prefix, q->dev->prefix);
		}
		if (k > 99) {
			fault("%s: more than 99 parts of prefix %s", s->file,
			      q->dev->prefix);
		}
		q->ref = sch_alloc(strlen(q->dev->prefix) + 16);
		sprintf(q->ref, "%s%d", q->dev->prefix, s->page * 100 + k);
	}
}

// Frees what the reading of a page keeps only for itself.
static void forget(void)
{
	for (size_t k = 0; k < R->npart; k++) {
		free(places[k].ref[0]);
		free(places[k].ref[1]);
		free(places[k].scope);
		free(places[k].group);
	}
	for (size_t i = 0; i < naliases; i++) {
		free(aliases[i].name);
		free(aliases[i].ref);
		free(aliases[i].scope);
	}
	for (size_t i = 0; i < nlaters; i++) {
		free(laters[i].ref[0]);
		free(laters[i].ref[1]);
		free(laters[i].scope);
		free(laters[i].name);
	}
	free(pins);
	free(base);
	free(placed);
	free(besides);
	free(dead);
	free(anchor_of);
	free(along_of);
	free(places);
	free(aliases);
	free(laters);
	pins = NULL;
	base = NULL;
	placed = besides = dead = along_of = NULL;
	anchor_of = NULL;
	places = NULL;
	aliases = NULL;
	laters = NULL;
	npins = naliases = nlaters = 0;
}

// Notes a page statement, page NAME, among the pages of the file,
// numbered by its place among them; the page read checks its words.
static void add_page(struct sch *s, char **w, int n, int read)
{
	struct sch_page pg;
	int num;

	memset(&pg, 0, sizeof pg);
	pg.line = line_no;
	pg.name = sch_strdup(n == 2 ? w[1] : "");
	pg.num = (int)s->npages + 1;
	if (n != 2 || integer(w[1], &num)) {
		if (read) {
			fault("%s: page NAME, a name and no number", where);
		}
	} else if (strchr(w[1], '.') && read) {
		fault("%s: a name has no dot", where);
	} else if (pg.num > 99 && read) {
		fault("%s: more than 99 pages", where);
	}
	s->pages = grow(s->pages, s->npages, sizeof *s->pages);
	s->pages[s->npages++] = pg;
}

// Notes a sheet port of the page pg, a port outside any group, by the
// words of a statement; open counts the groups open.
static void note_port(struct sch_page *pg, char **w, int n, int *open)
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
		pg->port = grow(pg->port, pg->nport, sizeof *pg->port);
		pg->port_line =
			grow(pg->port_line, pg->nport, sizeof *pg->port_line);
		pg->port[pg->nport] = sch_strdup(w[1]);
		pg->port_line[pg->nport++] = line_no;
	}
}

// Tells the faults of the pages of the file: a name of a page twice, and, as a warning, a sheet port on one page alone.
static void check_pages(const struct sch *s)
{
	if (!s->npages) {
		fault("%s: no page statement", s->file);
	}
	for (size_t j = 0; j < s->npages; j++) {
		const struct sch_page *b = &s->pages[j];

		for (size_t i = 0; i < j; i++) {
			const struct sch_page *a = &s->pages[i];

			if (*b->name && !strcmp(a->name, b->name)) {
				fault("%s:%d: page %s again", s->file, b->line,
				      b->name);
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
					other |= !strcmp(s->pages[i].port[q],
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

int sch_read(struct sch *s, const char *file, size_t k, char **lib, size_t nlib)
{
	char buf[LINE], *w[WORDS], **libs;
	const char *slash = strrchr(file, '/');
	size_t dn = slash ? (size_t)(slash - file) : 1;
	FILE *f = fopen(file, "r");
	int open = 0, before = 0;

	memset(s, 0, sizeof *s);
	R = s;
	s->file = file;
	s->page = 1;
	s->name = sch_strdup("");
	faults = 0;
	line_no = 0;
	depth = 0;
	*group_path = 0;
	// The project's own symbols come first: those next to the source.
	libs = sch_alloc((nlib + 1) * sizeof *libs);
	libs[0] = sch_alloc(dn + 16);
	memcpy(libs[0], slash ? file : ".", dn);
	strcpy(libs[0] + dn, "/symbols");
	memcpy(libs + 1, lib, nlib * sizeof *lib);
	s->lib = libs;
	s->nlib = nlib + 1;
	s->scale = 1;
	s->sheet = sch_strdup("sch_a");
	if (!f) {
		sch_die("%s: cannot read", file);
	}
	while (fgets(buf, sizeof buf, f)) {
		int n, here;

		line_no++;
		at_line(line_no);
		if (!strchr(buf, '\n') && !feof(f)) {
			int c;

			if (s->npages == k + 1 || (!s->npages && !k)) {
				fault("%s: line too long", where);
			}
			while ((c = getc(f)) != EOF && c != '\n') {
			}
			continue;
		}
		n = split(buf, w);
		here = s->npages == k + 1;
		if (n < 0) {
			if (here) {
				fault("%s: too many words", where);
			}
			continue;
		}
		if (n == 0) {
			continue;
		}
		if (!strcmp(w[0], "page")) {
			add_page(s, w, n, s->npages == k);
			open = 0;
			if (s->npages == k + 1) {
				s->page = s->pages[k].num;
				free(s->name);
				s->name = sch_strdup(s->pages[k].name);
				snprintf(group_path, sizeof group_path, "%s",
					 s->name);
			}
			continue;
		}
		if (!s->npages) {
			if (!k && !before++) {
				fault("%s: a statement before the first page",
				      where);
			}
			continue;
		}
		note_port(&s->pages[s->npages - 1], w, n, &open);
		if (!here) {
			continue;
		}
		if ((!strcmp(w[0], "field") && n >= 3) ||
		    (!strcmp(w[0], "title") && n >= 2)) {
			int t = w[0][0] == 't';

			if (s->nfield == 8) {
				fault("%s: too many fields", where);
				continue;
			}
			s->field[s->nfield][0] = sch_strdup(t ? "Title" : w[1]);
			s->field[s->nfield][1] = join(w, t ? 1 : 2, n);
			s->nfield++;
		} else if (!strcmp(w[0], "sheet") && n == 2) {
			s->sheet = sch_strdup(w[1]);
		} else if (!strcmp(w[0], "scale") && n == 2) {
			s->scale = atof(w[1]);
			if (!(s->scale > 0.1 && s->scale < 10)) {
				fault("%s: bad scale", where);
				s->scale = 1;
			}
		} else {
			read_statement(w, n);
		}
	}
	fclose(f);
	read_clearances(s);
	if (depth) {
		fault("%s: group %s has no end", file, group_path);
	}
	if (!k) {
		check_pages(s);
	}
	resolve_all();
	common_ferrite();
	place_all();
	make_nets();
	designate(s);
	tell_traces();
	forget();
	return faults;
}
