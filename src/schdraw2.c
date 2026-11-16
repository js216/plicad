// SPDX-License-Identifier: GPL-3.0-only
// schdraw2.c -- Draw a schematic sheet, as schdraw(1) says
// Copyright (c) 2026 Jakob Kastelic

// The program reads the source and its library, joins the pins into
// nets, places the parts cluster by cluster, routes each group of each
// net as a tree on the grid, puts each label where it covers least,
// measures the room between all things and writes the page.  All
// geometry is in units of the grid, 0.1 in at scale 1, until the page
// is written in points.
//
// A cluster is a part placed by no other part (a root: free, or WAY of
// something) and all that hangs from it.  A cluster is built in a frame
// of its own; once nothing more can join it, it moves as a whole out
// its way until it keeps its room from all the clusters before it.

#define _POSIX_C_SOURCE 200809L // For opendir.

#include "util.h"

#include <dirent.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum { R, U, L, D }; // The ways, counterclockwise.
static const int DX[4] = { 1, 0, -1, 0 }; // A step each way.
static const int DY[4] = { 0, 1, 0, -1 };
#define OPP(d) (((d) + 2) & 3)

enum { MAXW = 1024 }; // The most words of a record.
enum { LIM = 80 }; // The most steps a placement tries.
static const double EPS = 1e-6;
static const double PT = 7.2; // Points to the grid at scale 1.
static const double PI = 3.14159265358979323846;

// Grows array v of n items with room for cap to hold one more, and
// zeroes that one.
static void *grow(void *v, int n, int *cap, size_t size)
{
	char *p = v;
	if (n >= *cap) {
		*cap = *cap ? 2 * *cap : 16;
		p = xrealloc(p, (size_t)*cap * size);
	}
	memset(p + (size_t)n * size, 0, size);
	return p;
}
// Appends a zeroed item to v and gives its address.
#define ADD(v, n, cap) ((v) = grow((v), (n), &(cap), sizeof(*(v))), &(v)[(n)++])

static const char *src; // The path of the source.
static int faults;

static void complain(const char *file, int line, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));

// Tells a fault as FILE:LINE: (or FILE: for no line) and counts it.
static void complain(const char *file, int line, const char *fmt, ...)
{
	va_list ap;
	faults++;
	(void)fflush(stdout);
	if (line > 0) {
		(void)fprintf(stderr, "%s:%d: ", file, line);
	} else {
		(void)fprintf(stderr, "%s: ", file);
	}
	va_start(ap, fmt);
	(void)vfprintf(stderr, fmt, ap);
	va_end(ap);
	(void)fputc('\n', stderr);
}

static char *cat3(const char *a, const char *b, const char *c)
{
	size_t n = strlen(a) + strlen(b) + strlen(c) + 1;
	char *s = xmalloc(n);
	(void)snprintf(s, n, "%s%s%s", a, b, c);
	return s;
}

// The name b within scope a: a.b, or b at the top.
static char *within(const char *a, const char *b)
{
	return *a ? cat3(a, ".", b) : xstrdup(b);
}

// The integers next below and above v.
static int down_to(double v)
{
	double f = floor(v + EPS);
	return (int)f;
}
static int up_to(double v)
{
	double f = ceil(v - EPS);
	return (int)f;
}

static int num(const char *w, double *v)
{
	char *e = NULL;
	*v = strtod(w, &e);
	return e != w && !*e && num_ok(*v);
}

// The way a word names (left, right, up or down; r u l d), or -1.
static int way_of(const char *w)
{
	static const char *const names[] = { "right", "up", "left", "down",
					     "r",     "u",  "l",    "d" };
	for (int i = 0; i < 8; i++) {
		if (!strcmp(w, names[i])) {
			return i & 3;
		}
	}
	return -1;
}
static const char *const WAYNAME[] = { "right", "up", "left", "down" };

// ---- Files: logical lines, the library

struct line {
	char *s;
	int no;
};

// The lines of a file, comments cut off, a backslash at the end of a
// line joining the next; NULL if the file cannot be opened.
static struct line *read_lines(const char *path, int *n)
{
	FILE *f = fopen(path, "rb");
	struct line *v = NULL;
	char *buf = NULL, *p = NULL, *l = NULL;
	int cap = 0, no = 1;
	*n = 0;
	if (!f) {
		return NULL;
	}
	(void)fclose(f);
	buf = read_file(path);
	for (p = buf; *p; p++) {
		if (p[0] == '\\' && p[1] == '\r' && p[2] == '\n') {
			p[0] = p[1] = ' ';
			p[2] = '\v';
		} else if (p[0] == '\\' && p[1] == '\n') {
			p[0] = ' ';
			p[1] = '\v';
		}
	}
	p = buf;
	while ((l = next_line(&p)) != NULL) {
		struct line *e = ADD(v, *n, cap);
		int extra = 0;
		char *hash = NULL;
		for (char *q = l; *q; q++) {
			if (*q == '\v') {
				*q = ' ';
				extra++;
			}
		}
		hash = strchr(l, '#');
		if (hash) {
			*hash = 0;
		}
		e->s = l;
		e->no = no;
		no += 1 + extra;
	}
	return v;
}

// Splits a line into words; 0 for too many.
static int words(char *s, char **w)
{
	size_t n = split_words(s, w, MAXW);
	return n > MAXW ? 0 : (int)n;
}

// The text of s after its first k words, trimmed.
static char *rest(const char *s, int k)
{
	char *e = NULL;
	char *t = NULL;
	for (int i = 0; i < k; i++) {
		s += strspn(s, " \t");
		s += strcspn(s, " \t");
	}
	s += strspn(s, " \t");
	t = xstrdup(s);
	e = t + strlen(t);
	while (e > t && (e[-1] == ' ' || e[-1] == '\t')) {
		*--e = 0;
	}
	return t;
}

static char **symdir, **sheetdir;
static int nsymdir, csymdir, nsheetdir, csheetdir;

// A drawing record of a shape.  cond: 0 always, 1 only plain, 2 only
// flipped.
enum { K_LINE, K_LOOP, K_FILL, K_BLANK, K_CIRCLE, K_DISK };
struct rec {
	int kind, cond, n;
	double w;
	double *v;
};
struct spin {
	char *name;
	int x, y, dir;
};
enum { NOFLAG, GROUND, CHASSIS, SUPPLY, PORT };
struct shape {
	char *kind, *prefix;
	struct spin *pin;
	struct rec *rec;
	int npin, cpin, nrec, crec, side, flag, flip0, flip1;
	double box[4];
};
static struct shape **shapes;
static int nshapes, cshapes;

static int name_in(const char *w, const char *const *names, int n)
{
	for (int i = 0; i < n; i++) {
		if (!strcmp(w, names[i])) {
			return i;
		}
	}
	return -1;
}

static const char *const KINDS[] = { "line",  "loop",	"fill",
				     "blank", "circle", "disk" };
static const char *const FLAGS[] = { "", "ground", "chassis", "supply",
				     "port" };
static const char *const SIDES[] = { "right", "above", "left", "below" };

static struct shape *read_shape(const char *path, const char *kind)
{
	int n = 0;
	struct line *ln = read_lines(path, &n);
	struct shape *s = NULL;
	double width = 0.5;
	char **w = NULL;
	if (!ln) {
		return NULL;
	}
	w = xmalloc(MAXW * sizeof(*w));
	s = xcalloc(1, sizeof(*s));
	s->kind = xstrdup(kind);
	s->prefix = xstrdup("X");
	s->side = R;
	s->flip0 = s->flip1 = -1;
	for (int i = 0; i < n; i++) {
		int k = words(ln[i].s, w), c = 0, j = 0, bad = 0, rk = 0;
		if (k == 0) {
			continue;
		}
		if (k > 1 &&
		    (!strcmp(w[0], "plain") || !strcmp(w[0], "flip")) &&
		    name_in(w[1], KINDS, 6) >= 0) {
			c = w[0][0] == 'p' ? 1 : 2;
			j = 1;
		}
		rk = name_in(w[j], KINDS, 6);
		if (rk >= 0) {
			struct rec *r = ADD(s->rec, s->nrec, s->crec);
			r->kind = rk;
			r->cond = c;
			r->w = width;
			r->n = k - j - 1;
			r->v = xmalloc((size_t)(r->n + 1) * sizeof(double));
			for (int m = 0; m < r->n; m++) {
				bad |= !num(w[j + 1 + m], &r->v[m]);
			}
			bad |= r->n < (rk >= K_CIRCLE ? 3 : 2) ||
			       (rk < K_CIRCLE && r->n % 2);
		} else if (!strcmp(w[0], "prefix") && k == 2) {
			s->prefix = xstrdup(w[1]);
		} else if (!strcmp(w[0], "pin") && k == 5) {
			struct spin *p = ADD(s->pin, s->npin, s->cpin);
			double x = 0, y = 0;
			p->name = xstrdup(w[1]);
			bad = !num(w[2], &x) || !num(w[3], &y) ||
			      way_of(w[4]) < 0;
			p->x = (int)lround(x);
			p->y = (int)lround(y);
			p->dir = way_of(w[4]) & 3;
		} else if (!strcmp(w[0], "box") && k == 5) {
			for (int m = 0; m < 4; m++) {
				bad |= !num(w[1 + m], &s->box[m]);
			}
		} else if (!strcmp(w[0], "side") && k == 2) {
			s->side = name_in(w[1], SIDES, 4) & 3;
			bad = name_in(w[1], SIDES, 4) < 0;
		} else if (!strcmp(w[0], "flip") && k == 3) {
			for (int m = 0; m < s->npin; m++) {
				s->flip0 = strcmp(s->pin[m].name, w[1]) ?
						   s->flip0 :
						   m;
				s->flip1 = strcmp(s->pin[m].name, w[2]) ?
						   s->flip1 :
						   m;
			}
			bad = s->flip0 < 0 || s->flip1 < 0;
		} else if (!strcmp(w[0], "flag") && k == 2) {
			s->flag = name_in(w[1], FLAGS, 5);
			bad = s->flag <= 0;
		} else if (!strcmp(w[0], "width") && k == 2) {
			bad = !num(w[1], &width);
		} else if (strcmp(w[0], "swap")) {
			bad = 1;
		}
		if (bad) {
			complain(path, ln[i].no, "what is %s?", w[0]);
		}
	}
	free(w);
	return s;
}

// The shape of a kind, from the first directory that has it, or NULL.
static struct shape *shape_of(const char *kind)
{
	for (int i = 0; i < nshapes; i++) {
		if (!strcmp(shapes[i]->kind, kind)) {
			return shapes[i];
		}
	}
	for (int i = 0; i < nsymdir; i++) {
		char *path = cat3(symdir[i], "/", kind);
		char *full = cat3(path, ".sym", "");
		struct shape *s = read_shape(full, kind);
		free(path);
		free(full);
		if (s) {
			*ADD(shapes, nshapes, cshapes) = s;
			return s;
		}
	}
	return NULL;
}

// The sheet template and the room the drawing keeps, in grid units.
enum { C_BODY, C_TEXT, C_LABEL, C_FLAG };
static const char *const CLEARS[] = { "body", "text", "label", "flag" };
static double paper[2], room[4], clr[4];
static struct line *sheetln;
static int nsheetln;

static int read_sheet(const char *name)
{
	char *w[16];
	int have[4] = { 0, 0, 0, 0 };
	for (int i = 0; i < nsheetdir; i++) {
		char *path = cat3(sheetdir[i], "/", name);
		char *full = cat3(path, ".sheet", "");
		sheetln = read_lines(full, &nsheetln);
		if (!sheetln) {
			continue;
		}
		for (int j = 0; j < nsheetln; j++) {
			char *copy = xstrdup(sheetln[j].s);
			size_t k = split_words(copy, w, 16);
			double v = 0;
			if (k == 3 && !strcmp(w[0], "paper")) {
				(void)num(w[1], &paper[0]);
				(void)num(w[2], &paper[1]);
			} else if (k == 5 && !strcmp(w[0], "room")) {
				for (int m = 0; m < 4; m++) {
					(void)num(w[1 + m], &room[m]);
				}
			} else if (k == 3 && !strcmp(w[0], "clear") &&
				   name_in(w[1], CLEARS, 4) >= 0 &&
				   num(w[2], &v)) {
				clr[name_in(w[1], CLEARS, 4)] = v / PT;
				have[name_in(w[1], CLEARS, 4)] = 1;
			}
			free(copy);
		}
		for (int m = 0; m < 4; m++) {
			if (!have[m]) {
				complain(full, 0, "no clear %s", CLEARS[m]);
				return -1;
			}
		}
		return 1;
	}
	return 0;
}

// The advances of Nimbus Sans, regular and italic, at a size of 1.
enum { NCODE = 0x3000 };
static double adv[2][NCODE];
static int have_font;

static void read_font(const char *path)
{
	int n = 0, f = 0;
	double units = 1000;
	struct line *ln = read_lines(path, &n);
	for (int i = 0; ln && i < n; i++) {
		char *s = ln[i].s;
		if (!strncmp(s, "font ", 5)) {
			f = s[5] == 'I';
			units = strtod(s + 7, NULL);
			have_font = 1;
		} else if (!strncmp(s, "glyph ", 6) && units > 0) {
			char *e = NULL;
			long c = strtol(s + 6, &e, 10);
			if (c >= 0 && c < NCODE) {
				adv[f][c] = strtod(e, NULL) / units;
			}
		}
	}
}

// The width of UTF-8 text s in font italic (0 or 1) at a size.
static double text_w(int italic, const char *s, double size)
{
	const unsigned char *p = (const unsigned char *)s;
	double w = 0;
	while (*p) {
		unsigned c = *p++;
		if (c >= 0xc0) {
			int more = c >= 0xf0 ? 3 : c >= 0xe0 ? 2 : 1;
			c &= 0x3fu >> more;
			while (more-- && (*p & 0xc0) == 0x80) {
				c = (c << 6) | (*p++ & 0x3fu);
			}
		}
		w += have_font && c < NCODE && adv[italic][c] > 0 ?
			     adv[italic][c] :
			     0.556;
	}
	return w * size;
}

// ---- The source

enum { FREE, FROM, OF, BESIDE, BETWEEN };
struct part {
	char *name, *scope, *value, *rating, *type, *prefix, *vtext, *a, *b;
	char **num; // The number of each pin of the shape.
	struct shape *sh;
	int line, side, flip, how, way, ok, pa, pb, group, placed, gave;
	int x, y, rot, mir, swp, clu, base, nlab, lside;
	double lw, lh, slide;
	char ref[32];
};
// A pin of a part, or the pin of a flag (part < 0).
struct gpin {
	int part, k, net, x, y, dir, reach;
};
struct flag {
	char *net, *ref, *scope;
	struct shape *sh;
	int kind, line, gp, at, ok, placed, rot, clu;
	double r[4], tx, ty; // Its room, and where its text starts.
};
struct wire {
	char *a, *b, *scope;
	int line, way, pa, pb;
};
struct port {
	char *scope, *name, *target;
	int line, pin;
};
struct field {
	char *name, *text;
};
struct net {
	char *name;
	int size;
};

static struct part *part;
static struct gpin *pin;
static struct flag *flag;
static struct wire *wire;
static struct port *port;
static struct field *field;
static struct net *net;
static char **group;
static int npart, cpart, npin, cpin, nflag, cflag, nwire, cwire, nport, cport;
static int nfield, cfield, nnet, cnet, ngroup, cgroup;
static int page = -1;
static double scale = 1, G = 7.2; // G: points to the grid.
static char *sheetname;

static void set_field(const char *name, char *text)
{
	struct field *f = ADD(field, nfield, cfield);
	f->name = xstrdup(name);
	f->text = text;
}

static int find_part(const char *full)
{
	for (int i = 0; i < npart; i++) {
		if (!strcmp(part[i].name, full)) {
			return i;
		}
	}
	return -1;
}

static int find_group(const char *full)
{
	for (int i = 0; i < ngroup; i++) {
		if (!strcmp(group[i], full)) {
			return i;
		}
	}
	return -1;
}

// The pin table of a part number: symbols/MAKER/PART in lower case.
static void find_table(struct part *p)
{
	char low[256];
	size_t n = strlen(p->value);
	if (n >= sizeof(low) || strchr(p->value, '/')) {
		return;
	}
	for (size_t i = 0; i <= n; i++) {
		unsigned char c = (unsigned char)p->value[i];
		low[i] = c >= 'A' && c <= 'Z' ? (char)(c - 'A' + 'a') : (char)c;
	}
	for (int i = 0; i < nsymdir; i++) {
		DIR *dp = opendir(symdir[i]);
		struct dirent *e = NULL;
		while (dp && (e = readdir(dp)) != NULL) {
			char *maker = cat3(symdir[i], "/", e->d_name);
			char *path = cat3(maker, "/", low);
			int k = 0;
			struct line *ln = e->d_name[0] == '.' ?
						  NULL :
						  read_lines(path, &k);
			char *w[8];
			free(maker);
			free(path);
			if (!ln) {
				continue;
			}
			p->type = cat3(e->d_name, "/", low);
			for (int j = 0; j < k; j++) {
				size_t m = split_words(ln[j].s, w, 8);
				struct shape *s =
					m == 2 && !strcmp(w[0], "shape") ?
						shape_of(w[1]) :
						NULL;
				p->sh = s ? s : p->sh;
				if (m == 2 && !strcmp(w[0], "prefix")) {
					p->prefix = xstrdup(w[1]);
				}
				for (int q = 0;
				     m == 3 && !strcmp(w[0], "pin") &&
				     q < p->sh->npin;
				     q++) {
					p->num[q] = strcmp(p->sh->pin[q].name,
							   w[1]) ?
							    p->num[q] :
							    xstrdup(w[2]);
				}
			}
			(void)closedir(dp);
			return;
		}
		if (dp) {
			(void)closedir(dp);
		}
	}
}

// Reads a part statement: KIND NAME [VALUE] [rating R] [mpn M] [side
// WAY] [flip] [PLACE].
static void read_part(char **w, int n, const char *scope, int line)
{
	struct shape *sh = shape_of(w[0]);
	struct part *p = NULL;
	char *full = NULL;
	if (!sh || sh->flag || n < 2) {
		complain(src, line, "what is %s?", w[0]);
		return;
	}
	full = within(scope, w[1]);
	if (find_part(full) >= 0) {
		complain(src, line, "%s is named twice", full);
		return;
	}
	p = ADD(part, npart, cpart);
	p->name = full;
	p->scope = xstrdup(scope);
	p->line = line;
	p->sh = sh;
	p->side = -1;
	p->ok = 1;
	p->pa = p->pb = -1;
	for (int i = 2; i < n; i++) {
		int wy = way_of(w[i]);
		if (i + 1 < n && !strcmp(w[i], "rating")) {
			p->rating = xstrdup(w[++i]);
		} else if (i + 1 < n && !strcmp(w[i], "mpn")) {
			i++;
		} else if (i + 1 < n && !strcmp(w[i], "side") &&
			   way_of(w[i + 1]) >= 0) {
			p->side = way_of(w[++i]);
		} else if (!strcmp(w[i], "flip")) {
			p->flip = 1;
		} else if (wy >= 0 && i + 2 < n && !p->how &&
			   (!strcmp(w[i + 1], "from") ||
			    !strcmp(w[i + 1], "of"))) {
			p->how = w[i + 1][0] == 'f' ? FROM : OF;
			p->way = wy;
			p->a = xstrdup(w[i + 2]);
			i += 2;
		} else if (!strcmp(w[i], "beside") && i + 1 < n && !p->how) {
			p->how = BESIDE;
			p->a = xstrdup(w[++i]);
			p->way = i + 1 < n && way_of(w[i + 1]) >= 0 ?
					 way_of(w[++i]) :
					 -1;
		} else if (!strcmp(w[i], "between") && i + 4 < n && !p->how &&
			   !strcmp(w[i + 2], "and") && way_of(w[i + 4]) >= 0) {
			p->how = BETWEEN;
			p->a = xstrdup(w[i + 1]);
			p->b = xstrdup(w[i + 3]);
			p->way = way_of(w[i + 4]);
			i += 4;
		} else if (!p->value) {
			p->value = xstrdup(w[i]);
		} else {
			complain(src, line, "what is %s?", w[i]);
			p->ok = 0;
		}
	}
	p->prefix = sh->prefix;
	p->num = xcalloc((size_t)sh->npin + 1, sizeof(char *));
	if (p->value) {
		find_table(p);
	}
	for (int k = 0; k < p->sh->npin; k++) {
		p->num[k] = p->num[k] ? p->num[k] : p->sh->pin[k].name;
	}
	p->vtext = !p->value ? NULL :
		   p->rating ? cat3(p->value, " ", p->rating) :
			       xstrdup(p->value);
}

static void add_flag(int kind, const char *name, const char *at,
		     const char *scope, int line)
{
	static const char *const NAMES[] = { "", "GND", "CHASSIS" };
	struct flag *f = ADD(flag, nflag, cflag);
	f->kind = kind;
	f->net =
		xstrdup(kind == GROUND || kind == CHASSIS ? NAMES[kind] : name);
	f->ref = xstrdup(at);
	f->scope = xstrdup(scope);
	f->line = line;
	f->sh = shape_of(FLAGS[kind]);
	f->ok = 1;
	if (!f->sh || f->sh->flag != kind || f->sh->npin < 1) {
		complain(src, line, "no symbol %s in the library", FLAGS[kind]);
		f->ok = 0;
	}
}

static void read_source(void)
{
	int n = 0, nopen = 0, copen = 0;
	struct line *ln = read_lines(src, &n);
	char **open = NULL, *scope = xstrdup(""), **w = NULL;
	if (!ln) {
		die("cannot open %s", src);
	}
	w = xmalloc(MAXW * sizeof(*w));
	for (int i = 0; i < n; i++) {
		char *copy = xstrdup(ln[i].s);
		int k = words(ln[i].s, w), no = ln[i].no;
		if (k == 0) {
			free(copy);
			continue;
		}
		if (!strcmp(w[0], "page") && k == 2) {
			double v = 0;
			page = num(w[1], &v) && v >= 1 && v <= 99 ? (int)v : -1;
			if (page < 0) {
				complain(src, no, "the page must be 1 to 99");
			}
		} else if (!strcmp(w[0], "title")) {
			set_field("Title", rest(copy, 1));
		} else if (!strcmp(w[0], "field") && k >= 2) {
			set_field(w[1], rest(copy, 2));
		} else if (!strcmp(w[0], "sheet") && k == 2) {
			sheetname = xstrdup(w[1]);
		} else if (!strcmp(w[0], "scale") && k == 2) {
			if (!num(w[1], &scale) || scale <= 0) {
				complain(src, no, "what is scale %s?", w[1]);
				scale = 1;
			}
		} else if (!strcmp(w[0], "group") && k == 2) {
			char *full = within(scope, w[1]);
			*ADD(open, nopen, copen) = xstrdup(w[1]);
			free(scope);
			scope = full;
			*ADD(group, ngroup, cgroup) = xstrdup(full);
		} else if (!strcmp(w[0], "end") && k == 1) {
			if (!nopen) {
				complain(src, no, "end of no group");
			} else {
				char *dot = strrchr(scope, '.');
				nopen--;
				*(dot ? dot : scope) = 0;
			}
		} else if (!strcmp(w[0], "port") && k == 3) {
			if (!*scope) {
				add_flag(PORT, w[1], w[2], scope, no);
			} else {
				struct port *p = ADD(port, nport, cport);
				p->scope = xstrdup(scope);
				p->name = xstrdup(w[1]);
				p->target = xstrdup(w[2]);
				p->line = no;
				p->pin = -1;
			}
		} else if (!strcmp(w[0], "note")) {
			// For the reader.
		} else if (!strcmp(w[0], "wire") && (k == 4 || k == 5) &&
			   !strcmp(w[2], "to") &&
			   (k == 4 || way_of(w[4]) >= 0)) {
			struct wire *e = ADD(wire, nwire, cwire);
			e->a = xstrdup(w[1]);
			e->b = xstrdup(w[3]);
			e->scope = xstrdup(scope);
			e->line = no;
			e->way = k == 5 ? way_of(w[4]) : -1;
		} else if ((!strcmp(w[0], "ground") ||
			    !strcmp(w[0], "chassis")) &&
			   k == 3 && !strcmp(w[1], "at")) {
			add_flag(w[0][0] == 'g' ? GROUND : CHASSIS, "", w[2],
				 scope, no);
		} else if (!strcmp(w[0], "supply") && k == 4 &&
			   !strcmp(w[2], "at")) {
			add_flag(SUPPLY, w[1], w[3], scope, no);
		} else {
			read_part(w, k, scope, no);
		}
		free(copy);
	}
	for (int i = 0; i < nopen; i++) {
		complain(src, 0, "group %s has no end", open[i]);
	}
	if (page < 0) {
		complain(src, 0, "no page");
		page = 1;
	}
	free(w);
}

// The pin a name gives within a scope, as PART.PIN or GROUP.PORT, or -1
// after a fault.
static int resolve_pin(const char *scope, const char *ref, int line, int depth)
{
	const char *dot = strrchr(ref, '.');
	char *owner = NULL, *s = xstrdup(scope);
	int got = -1, done = 0;
	if (!dot || depth > 20) {
		complain(src, line, "no pin %s here", ref);
		free(s);
		return -1;
	}
	owner = xstrdup(ref);
	owner[dot - ref] = 0;
	while (!done) {
		char *full = within(s, owner);
		int i = find_part(full);
		char *cut = strrchr(s, '.');
		if (i >= 0) {
			for (int k = 0; k < part[i].sh->npin; k++) {
				if (!strcmp(part[i].sh->pin[k].name, dot + 1) ||
				    !strcmp(part[i].num[k], dot + 1)) {
					got = part[i].base + k;
				}
			}
			if (got < 0) {
				complain(src, line, "%s has no pin %s", owner,
					 dot + 1);
			}
			done = 1;
		} else if (find_group(full) >= 0) {
			for (int k = 0; k < nport && !done; k++) {
				if (!strcmp(port[k].scope, full) &&
				    !strcmp(port[k].name, dot + 1)) {
					got = resolve_pin(full, port[k].target,
							  port[k].line,
							  depth + 1);
					done = 1;
				}
			}
		}
		if (!done && !*s) {
			complain(src, line, "no pin %s here", ref);
			done = 1;
		}
		*(cut ? cut : s) = 0;
		free(full);
	}
	free(s);
	free(owner);
	return got;
}

// The part, or the group (*isgroup set), a name gives within a scope.
static int resolve_part(const char *scope, const char *name, int line,
			int *isgroup)
{
	char *s = xstrdup(scope);
	for (;;) {
		char *full = within(s, name);
		int i = find_part(full), g = find_group(full);
		char *cut = strrchr(s, '.');
		free(full);
		if (i >= 0 || g >= 0) {
			free(s);
			*isgroup = i < 0;
			return i >= 0 ? i : g;
		}
		if (!*s) {
			break;
		}
		*(cut ? cut : s) = 0;
	}
	free(s);
	complain(src, line, "no part %s here", name);
	return -1;
}

static int *up;
static int find(int a)
{
	while (up[a] != a) {
		up[a] = up[up[a]];
		a = up[a];
	}
	return a;
}
static void join(int a, int b)
{
	if (a >= 0 && b >= 0) {
		up[find(a)] = find(b);
	}
}

// Makes the pins, resolves every name, and joins the pins into nets.
static void make_nets(void)
{
	char **name = NULL, *told = NULL;
	for (int i = 0; i < npart; i++) {
		part[i].base = npin;
		for (int k = 0; k < part[i].sh->npin; k++) {
			struct gpin *g = ADD(pin, npin, cpin);
			g->part = i;
			g->k = k;
		}
	}
	for (int f = 0; f < nflag; f++) {
		struct gpin *g = ADD(pin, npin, cpin);
		g->part = -1 - f;
		flag[f].gp = npin - 1;
	}
	for (int i = 0; i < nport; i++) {
		port[i].pin = resolve_pin(port[i].scope, port[i].target,
					  port[i].line, 0);
	}
	up = xmalloc((size_t)npin * sizeof(int));
	for (int i = 0; i < npin; i++) {
		up[i] = i;
	}
	for (int i = 0; i < npart; i++) {
		struct part *p = &part[i];
		int isg = 0;
		if (p->how == FROM || p->how == BETWEEN) {
			p->pa = resolve_pin(p->scope, p->a, p->line, 0);
			p->ok &= p->pa >= 0;
		}
		if (p->how == BETWEEN) {
			p->pb = resolve_pin(p->scope, p->b, p->line, 0);
			p->ok &= p->pb >= 0 && p->sh->npin >= 2;
		}
		if (p->how == OF || p->how == BESIDE) {
			p->pa = resolve_part(p->scope, p->a, p->line, &isg);
			p->group = isg;
			p->ok &= p->pa >= 0 && !(isg && p->how == BESIDE);
		}
		if (p->how == FROM && p->ok) {
			join(p->base, p->pa);
		} else if (p->how == BETWEEN && p->ok) {
			join(p->base, p->pa);
			join(p->base + 1, p->pb);
		} else if (p->how == BESIDE && p->ok) {
			struct part *x = &part[p->pa];
			for (int k = 0; k < p->sh->npin && k < x->sh->npin;
			     k++) {
				join(p->base + k, x->base + k);
			}
		}
	}
	for (int i = 0; i < nwire; i++) {
		wire[i].pa =
			resolve_pin(wire[i].scope, wire[i].a, wire[i].line, 0);
		wire[i].pb =
			resolve_pin(wire[i].scope, wire[i].b, wire[i].line, 0);
		if (wire[i].pa >= 0 && wire[i].pb >= 0) {
			join(wire[i].pa, wire[i].pb);
		}
	}
	for (int f = 0; f < nflag; f++) {
		flag[f].at = flag[f].ok ?
				     resolve_pin(flag[f].scope, flag[f].ref,
						 flag[f].line, 0) :
				     -1;
		flag[f].ok &= flag[f].at >= 0;
		if (flag[f].ok) {
			join(flag[f].gp, flag[f].at);
			for (int g = 0; g < f; g++) {
				if (flag[g].ok &&
				    !strcmp(flag[g].net, flag[f].net)) {
					join(flag[f].gp, flag[g].gp);
					break;
				}
			}
		}
	}
	// The name of each net: a flag's, then a port's, then a pin's.
	name = xcalloc((size_t)npin, sizeof(char *));
	told = xcalloc((size_t)npin, 1);
	for (int f = 0; f < nflag; f++) {
		int r = find(flag[f].gp);
		if (!flag[f].ok) {
			continue;
		}
		if (!name[r]) {
			name[r] = flag[f].net;
		} else if (strcmp(name[r], flag[f].net) && !told[r]) {
			complain(src, 0, "nets %s and %s are joined", name[r],
				 flag[f].net);
			told[r] = 1;
		}
	}
	for (int i = 0; i < nport; i++) {
		int r = port[i].pin >= 0 ? find(port[i].pin) : -1;
		if (r >= 0 && !name[r]) {
			name[r] = within(port[i].scope, port[i].name);
		}
	}
	for (int i = 0; i < npin; i++) {
		pin[i].net = -1;
	}
	for (int i = 0; i < npin; i++) {
		int r = find(i), size = 0;
		if (pin[r].net >= 0 || pin[i].part < 0) {
			continue;
		}
		for (int j = 0; j < npin; j++) {
			size += find(j) == r;
		}
		if (size > 1) {
			struct net *e = ADD(net, nnet, cnet);
			struct part *p = &part[pin[i].part];
			e->name = name[r] ? name[r] :
					    within(p->name,
						   p->sh->pin[pin[i].k].name);
			e->size = size;
			pin[r].net = nnet - 1;
		}
	}
	for (int i = 0; i < npin; i++) {
		pin[i].net = pin[find(i)].net;
	}
	free(name);
	free(told);
}

// ---- Geometry of parts and flags

// Mirrors (x to -x) and then turns rot quarters counterclockwise.
static void turn(int rot, int mir, double x, double y, double *ox, double *oy)
{
	double t = 0;
	x = mir ? -x : x;
	for (int k = 0; k < rot; k++) {
		t = x;
		x = -y;
		y = t;
	}
	*ox = x;
	*oy = y;
}
static int tdir(int d, int rot, int mir)
{
	return ((mir && (d == R || d == L) ? OPP(d) : d) + rot) & 3;
}

// The pin of the shape that stands where pin k does: its flip pair
// trades places.
static int seat(const struct part *p, int k)
{
	if (p->swp && k == p->sh->flip0) {
		return p->sh->flip1;
	}
	return p->swp && k == p->sh->flip1 ? p->sh->flip0 : k;
}

// Sets where the pins of part i are, as it stands.
static void set_pins(int i)
{
	struct part *p = &part[i];
	for (int k = 0; k < p->sh->npin; k++) {
		struct spin *s = &p->sh->pin[seat(p, k)];
		struct gpin *g = &pin[p->base + k];
		double x = 0, y = 0;
		turn(p->rot, p->mir, s->x, s->y, &x, &y);
		g->x = p->x + (int)lround(x);
		g->y = p->y + (int)lround(y);
		g->dir = tdir(s->dir, p->rot, p->mir);
	}
}

static void rect_of(const double *box, int rot, int mir, double x, double y,
		    double *r)
{
	double ax = 0, ay = 0, bx = 0, by = 0;
	turn(rot, mir, box[0], box[1], &ax, &ay);
	turn(rot, mir, box[2], box[3], &bx, &by);
	r[0] = x + fmin(ax, bx);
	r[1] = y + fmin(ay, by);
	r[2] = x + fmax(ax, bx);
	r[3] = y + fmax(ay, by);
}

static void body_rect(int i, double *r)
{
	struct part *p = &part[i];
	rect_of(p->sh->box, p->rot, p->mir, p->x, p->y, r);
}

// The label of part i on a side, slid along it.
static void label_rect(int i, int side, double slide, double *r)
{
	struct part *p = &part[i];
	double b[4], g = clr[C_LABEL];
	double cx = 0, cy = 0;
	body_rect(i, b);
	cx = (b[0] + b[2]) / 2 + slide;
	cy = (b[1] + b[3]) / 2 + slide;
	if (side == R || side == L) {
		r[0] = side == R ? b[2] + g : b[0] - g - p->lw;
		r[1] = cy - p->lh / 2;
	} else {
		r[0] = cx - p->lw / 2;
		r[1] = side == U ? b[3] + g : b[1] - g - p->lh;
	}
	r[2] = r[0] + p->lw;
	r[3] = r[1] + p->lh;
}

// The number of places of a label along a side, half a step apart.
static int slides(int i, int side)
{
	struct part *p = &part[i];
	double b[4], span = 0;
	body_rect(i, b);
	span = (side == R || side == L ? b[3] - b[1] + p->lh :
					 b[2] - b[0] + p->lw) /
	       2;
	return 2 * (int)(span * 2) + 1;
}

static void label_size(int i)
{
	struct part *p = &part[i];
	double s = scale, w0 = 0, w1 = 0;
	w0 = text_w(1, p->prefix, 7 * s) +
	     text_w(0, p->ref + strlen(p->prefix), 6 * s);
	p->nlab = p->vtext && !p->gave ? 2 : 1;
	w1 = p->nlab > 1 ? text_w(0, p->vtext, 7 * s) : 0;
	p->lw = fmax(w0, w1) / G;
	p->lh = (0.72 * 7 * s + (p->nlab - 1) * 8.5 * s + 0.22 * 7 * s +
		 (p->nlab == 1) * 1.6 * s) /
		G;
}

static int on_net(int i, int n)
{
	for (int k = 0; k < part[i].sh->npin; k++) {
		if (n >= 0 && pin[part[i].base + k].net == n) {
			return 1;
		}
	}
	return 0;
}

// Puts flag f t steps out from its pin: a ground hangs under it, a
// supply or a port stands at the end of its line.
static void put_flag(int f, int t)
{
	struct flag *fl = &flag[f];
	struct gpin *a = &pin[fl->at], *q = &pin[fl->gp];
	int d = a->dir;
	double tw = 0, th = 0.72 * 7 * scale / G, gap = 1.5 * scale / G;
	if (fl->kind == GROUND || fl->kind == CHASSIS) {
		q->x = a->x + (d == U ? t : d == D ? 0 : DX[d] * t);
		q->y = a->y + (d == D ? -t : d == U ? 0 : -1);
		q->dir = U;
		fl->rot = 0;
		rect_of(fl->sh->box, 0, 0, q->x, q->y, fl->r);
		return;
	}
	q->x = a->x + DX[d] * t;
	q->y = a->y + DY[d] * t;
	q->dir = OPP(d);
	fl->rot = (OPP(d) - fl->sh->pin[0].dir) & 3;
	tw = text_w(0, fl->net, 7 * scale) / G;
	gap += fl->kind == PORT ? 1.7 * scale / G : 0;
	fl->tx = d == R ? q->x + gap : d == L ? q->x - gap - tw : q->x - tw / 2;
	fl->ty = d == U ? q->y + gap : d == D ? q->y - gap - th : q->y - th / 2;
	fl->r[0] = fmin(fl->tx, q->x - gap);
	fl->r[1] = fmin(fl->ty, q->y - (fl->kind == PORT) * gap);
	fl->r[2] = fmax(fl->tx + tw, q->x + (fl->kind == PORT) * gap);
	fl->r[3] = fmax(fl->ty + th, q->y + (fl->kind == PORT) * gap);
	if (fl->kind == SUPPLY && (d == U || d == D)) {
		fl->r[0] = fl->tx;
		fl->r[2] = fl->tx + tw;
		fl->r[1] = d == U ? q->y + EPS : fl->ty;
		fl->r[3] = d == U ? fl->ty + th : q->y - EPS;
	}
}

// ---- Placement

enum { BODY, LABEL, FLAGBOX, LEAD };
struct item {
	double r[4];
	int kind, owner, clu, net; // net: a lead's.
};
static struct item *item;
static int nitem, citem, cur = -1, nclu;

static void set_item(struct item *t, const double *r, int kind, int owner,
		     int nt)
{
	memcpy(t->r, r, sizeof(t->r));
	t->kind = kind;
	t->owner = owner;
	t->clu = cur;
	t->net = nt;
}
static void add_item(const double *r, int kind, int owner, int nt)
{
	set_item(ADD(item, nitem, citem), r, kind, owner, nt);
}

// Whether rectangles a and b keep g apart.
static int apart(const double *a, const double *b, double g)
{
	return a[0] >= b[2] + g - EPS || b[0] >= a[2] + g - EPS ||
	       a[1] >= b[3] + g - EPS || b[1] >= a[3] + g - EPS;
}

static void seg(double *r, int x0, int y0, int x1, int y1)
{
	r[0] = x0 < x1 ? x0 : x1;
	r[1] = y0 < y1 ? y0 : y1;
	r[2] = x0 > x1 ? x0 : x1;
	r[3] = y0 > y1 ? y0 : y1;
}

static void grow_rect(double *r, const double *s)
{
	r[0] = fmin(r[0], s[0]);
	r[1] = fmin(r[1], s[1]);
	r[2] = fmax(r[2], s[2]);
	r[3] = fmax(r[3], s[3]);
}

// The box a wire's loop keeps free: round its two pins, a step out for
// the wire, and the parts of its net in cluster c with their labels.
static int loop_box(int w, int c, double *r)
{
	struct gpin *a = &pin[wire[w].pa], *b = &pin[wire[w].pb];
	if (a->part < 0 || b->part < 0 || !part[a->part].placed ||
	    !part[b->part].placed || part[a->part].clu != c ||
	    part[b->part].clu != c) {
		return 0;
	}
	seg(r, a->x, a->y, b->x, b->y);
	r[0] -= 1;
	r[1] -= 1;
	r[2] += 1;
	r[3] += 1;
	for (int j = 0; j < npart; j++) {
		double s[4];
		if (part[j].placed && part[j].clu == c && on_net(j, a->net)) {
			body_rect(j, s);
			grow_rect(r, s);
			label_rect(j, part[j].lside, part[j].slide, s);
			grow_rect(r, s);
		}
	}
	return 1;
}

static int inside(const double *r, double g, int x, int y)
{
	return x > r[0] - g + EPS && x < r[2] + g - EPS && y > r[1] - g + EPS &&
	       y < r[3] + g - EPS;
}

// Whether the pins of placed things in cluster c lead out clear of rect
// r with room g, and the pins of part i (if any) clear of bodies.
static int exits_clear(const double *r, double g, int i, int nt, int c)
{
	for (int k = 0; k < npin; k++) {
		struct gpin *q = &pin[k];
		int placed = q->part >= 0 ? part[q->part].placed &&
						    part[q->part].clu == c :
					    flag[-1 - q->part].placed &&
						    flag[-1 - q->part].clu == c;
		int mine = i >= 0 ? on_net(i, q->net) : q->net == nt;
		if (q->net >= 0 && placed &&
		    inside(r, mine ? 0 : g, q->x + DX[q->dir],
			   q->y + DY[q->dir])) {
			return 0;
		}
	}
	for (int k = 0; i >= 0 && k < part[i].sh->npin; k++) {
		struct gpin *q = &pin[part[i].base + k];
		for (int t = 0; t < nitem && q->net >= 0; t++) {
			if (item[t].clu == c && item[t].kind == BODY &&
			    inside(item[t].r,
				   on_net(item[t].owner, q->net) ? 0 :
								   clr[C_BODY],
				   q->x + DX[q->dir], q->y + DY[q->dir])) {
				return 0;
			}
		}
	}
	return 1;
}

// Widens the body r of part i across its pins to the thickness of the
// parts hung from them, and down to the grounds they hang: a bead keeps
// out of a loop by the room of the capacitor it carries.
static void thicken(int i, double *r)
{
	double t = 0, down = 0;
	int across = pin[part[i].base].dir == R || pin[part[i].base].dir == L;
	for (int j = 0; j < npart; j++) {
		const double *b = part[j].sh->box;
		if (part[j].ok && part[j].how == FROM && part[j].pa >= 0 &&
		    pin[part[j].pa].part == i) {
			t = fmax(t, fmax(fabs(b[0]), fabs(b[2])));
			for (int f = 0; f < nflag; f++) {
				if (flag[f].ok && flag[f].kind <= CHASSIS &&
				    pin[flag[f].at].part == j) {
					down = fmax(down,
						    1 - flag[f].sh->box[1]);
				}
			}
		}
	}
	if (across) {
		double c = (r[1] + r[3]) / 2;
		r[1] = fmin(r[1], c - fmax(t, down));
		r[3] = fmax(r[3], c + t);
	} else {
		double c = (r[0] + r[2]) / 2;
		r[0] = fmin(r[0], c - t);
		r[2] = fmax(r[2], c + t);
	}
}

// Whether item t is a body, a flag or a lead of net n.
static int item_on(const struct item *t, int n)
{
	if (t->kind == BODY) {
		return on_net(t->owner, n);
	}
	return t->kind == FLAGBOX ? pin[flag[-1 - t->owner].gp].net == n :
				    t->kind == LEAD && t->net == n;
}

// Whether candidate a keeps its room from a placed item t: a lead keeps
// the room of a body only from those of other nets.
static int clear_of(const struct item *a, const struct item *t)
{
	static const int room_of[4][4] = {
		// item: BODY, LABEL, FLAGBOX, LEAD
		{ C_BODY, C_TEXT, C_BODY, C_BODY }, // a body
		{ C_TEXT, C_TEXT, C_FLAG, -1 }, // a label
		{ C_BODY, C_FLAG, C_TEXT, C_BODY }, // a flag
		{ C_BODY, -1, C_BODY, -1 }, // a lead
	};
	int k = room_of[a->kind][t->kind];
	if (k == C_BODY && ((a->kind == LEAD && item_on(t, a->net)) ||
			    (t->kind == LEAD && item_on(a, t->net)))) {
		return 1;
	}
	return k < 0 || apart(a->r, t->r, clr[k]);
}

// Whether the things of a candidate keep their room in cluster c from
// all that is placed, the loops, and the flags still to come.
static int clear_all(const struct item *r, int n, int i, int nets_of,
		     int anchor)
{
	int c = cur;
	for (int t = 0; t < nitem; t++) {
		for (int j = 0; j < n && item[t].clu == c; j++) {
			if (!clear_of(&r[j], &item[t])) {
				return 0;
			}
		}
	}
	for (int j = 0; j < n; j++) {
		if (r[j].kind != LEAD &&
		    !exits_clear(r[j].r, r[j].kind == BODY ? clr[C_BODY] : 0, i,
				 nets_of, c)) {
			return 0;
		}
	}
	for (int w = 0; w < nwire; w++) {
		double b[4];
		int wn = wire[w].pa >= 0 && wire[w].pb >= 0 ?
				 pin[wire[w].pa].net :
				 -1;
		if (wn < 0 || (i >= 0 ? on_net(i, wn) : nets_of == wn) ||
		    !loop_box(w, c, b)) {
			continue;
		}
		for (int j = 0; j < n; j++) {
			double o[4];
			memcpy(o, r[j].r, sizeof(o));
			if (r[j].kind == BODY && i >= 0) {
				thicken(i, o);
			}
			if (r[j].kind != LEAD && !apart(o, b, clr[C_BODY])) {
				return 0;
			}
		}
	}
	// A supply or a port to come keeps the end of its pin's line.
	for (int f = 0; f < nflag && i >= 0; f++) {
		struct flag *fl = &flag[f];
		struct gpin *a = &pin[fl->at];
		double l[4];
		if (!fl->ok || fl->placed || fl->kind < SUPPLY ||
		    fl->at == anchor || a->part < 0 || !part[a->part].placed ||
		    part[a->part].clu != c) {
			continue;
		}
		put_flag(f, a->reach + 1);
		seg(l, a->x, a->y, pin[fl->gp].x, pin[fl->gp].y);
		for (int j = 0; j < n; j++) {
			if (r[j].kind == BODY &&
			    (!apart(r[j].r, fl->r, clr[C_BODY]) ||
			     (!on_net(i, a->net) &&
			      !apart(r[j].r, l, clr[C_BODY])))) {
				return 0;
			}
		}
	}
	return 1;
}

// The grounds hung at the pins of part i, and with all, its other flags.
static int grounds_of(int i, int *f, int max, int all)
{
	int n = 0;
	for (int g = 0; g < nflag && n < max; g++) {
		if (flag[g].ok && (all || flag[g].kind <= CHASSIS) &&
		    pin[flag[g].at].part == i) {
			f[n++] = g;
		}
	}
	return n;
}

// Whether a label of part i keeps off the leads its pins will take.
static int lanes_clear(int i, const double *r)
{
	for (int k = 0; k < part[i].sh->npin; k++) {
		struct gpin *q = &pin[part[i].base + k];
		double l[4];
		seg(l, q->x, q->y, q->x + 2 * DX[q->dir],
		    q->y + 2 * DY[q->dir]);
		if (!apart(r, l, clr[C_TEXT])) {
			return 0;
		}
	}
	return 1;
}

// Whether part i fits where it stands, with leads from its anchor.
static int fits(int i, double (*lead)[4], int nlead, int anchor)
{
	struct item r[24];
	double b[4];
	int n = 0, gf[8], ng = 0, own = part[i].lside,
	    nt = anchor >= 0 ? pin[anchor].net : -1;
	set_pins(i);
	body_rect(i, b);
	set_item(&r[n++], b, BODY, i, -1);
	set_item(&r[n++], b, LABEL, i, -1);
	ng = grounds_of(i, gf, 8, 1);
	for (int g = 0; g < ng; g++) {
		struct gpin *a = &pin[flag[gf[g]].at],
			    *q = &pin[flag[gf[g]].gp];
		put_flag(gf[g], 1);
		set_item(&r[n++], flag[gf[g]].r, FLAGBOX, -1 - gf[g], -1);
		seg(b, a->x, a->y, q->x, q->y);
		set_item(&r[n++], b, LEAD, -1 - gf[g], a->net);
	}
	for (int l = 0; l < nlead && n < 24; l++) {
		set_item(&r[n++], lead[l], LEAD, i, nt);
	}
	// The label takes the first clear place: its own side, the other,
	// then the two across, the middle of each first, and off the leads
	// of its own pins.
	for (int k = 0; k < 4; k++) {
		int side = (own + (k == 1 ? 2 : k == 2 ? 1 : k)) & 3;
		for (int j = 0; j < slides(i, side); j++) {
			double slide = (j + 1) / 2 * 0.5 * (j % 2 ? 1 : -1);
			label_rect(i, side, slide, r[1].r);
			if (lanes_clear(i, r[1].r) &&
			    clear_all(r, n, i, -1, anchor)) {
				part[i].lside = side;
				part[i].slide = slide;
				return 1;
			}
		}
	}
	part[i].lside = own;
	part[i].slide = 0;
	return 0;
}

// Lays part i where it stands, with its grounds and leads.
static void commit(int i, double (*lead)[4], int nlead)
{
	struct part *p = &part[i];
	double r[4];
	int gf[8], ng = grounds_of(i, gf, 8, 0);
	int anchor_net = p->how == FROM ? pin[p->pa].net : -1;
	p->placed = 1;
	p->clu = cur;
	set_pins(i);
	body_rect(i, r);
	add_item(r, BODY, i, -1);
	label_rect(i, p->lside, p->slide, r);
	add_item(r, LABEL, i, -1);
	for (int l = 0; l < nlead; l++) {
		add_item(lead[l], LEAD, i, anchor_net);
	}
	for (int g = 0; g < ng; g++) {
		struct flag *fl = &flag[gf[g]];
		struct gpin *a = &pin[fl->at];
		put_flag(gf[g], 1);
		fl->placed = 1;
		fl->clu = cur;
		add_item(fl->r, FLAGBOX, -1 - gf[g], -1);
		seg(r, a->x, a->y, pin[fl->gp].x, pin[fl->gp].y);
		add_item(r, LEAD, -1 - gf[g], a->net);
	}
}

// Turns part i so that its pin k points way d.
static void aim(int i, int k, int d)
{
	struct part *p = &part[i];
	p->mir = p->flip && p->sh->flip0 < 0;
	p->swp = p->flip && p->sh->flip0 >= 0;
	p->rot = 0;
	while (tdir(p->sh->pin[seat(p, k)].dir, p->rot, p->mir) != d) {
		p->rot++;
	}
	p->lside = p->side >= 0 ? p->side : tdir(p->sh->side, p->rot, p->mir);
}

// Stands part i with its pin k at (x, y).
static void stand(int i, int k, int x, int y)
{
	struct part *p = &part[i];
	p->x = p->y = 0;
	set_pins(i);
	p->x = x - pin[p->base + k].x;
	p->y = y - pin[p->base + k].y;
	set_pins(i);
}

static int place_from(int i)
{
	struct part *p = &part[i];
	struct gpin *a = &pin[p->pa];
	int d = a->dir, w = p->way;
	double l[2][4];
	if (w == OPP(d)) {
		complain(src, p->line,
			 "a pin can lead only its own way or across");
		p->ok = 0;
		return 1;
	}
	aim(i, 0, OPP(w));
	if (w == d) {
		int t0 = a->reach +
			 (a->net >= 0 && net[a->net].size > 2 ? 2 : 1);
		for (int t = t0; t < t0 + LIM; t++) {
			stand(i, 0, a->x + DX[d] * t, a->y + DY[d] * t);
			seg(l[0], a->x, a->y, a->x + DX[d] * t,
			    a->y + DY[d] * t);
			if (fits(i, l, 1, p->pa)) {
				commit(i, l, 1);
				a->reach = t;
				return 1;
			}
		}
		return 0;
	}
	for (int k = 0; k < LIM; k++) {
		for (int s = 1; s <= k + 1; s++) {
			int t = a->reach + 1 + k - (s - 1);
			int jx = a->x + DX[d] * t, jy = a->y + DY[d] * t;
			stand(i, 0, jx + DX[w] * s, jy + DY[w] * s);
			seg(l[0], a->x, a->y, jx, jy);
			seg(l[1], jx, jy, jx + DX[w] * s, jy + DY[w] * s);
			if (fits(i, l, 2, p->pa)) {
				commit(i, l, 2);
				a->reach = t;
				return 1;
			}
		}
	}
	return 0;
}

static int place_beside(int i)
{
	struct part *p = &part[i], *x = &part[p->pa];
	int axis = pin[x->base].dir, ways[2];
	int across = axis == R || axis == L;
	ways[0] = p->way >= 0 ? p->way : across ? D : R;
	ways[1] = p->way >= 0 ? p->way : OPP(ways[0]);
	if ((ways[0] == axis || ways[0] == OPP(axis)) || p->sh != x->sh) {
		return 0;
	}
	p->rot = x->rot;
	p->mir = x->mir;
	p->swp = x->swp;
	p->lside = p->side >= 0 ? p->side : tdir(p->sh->side, p->rot, p->mir);
	for (int k = 1; k < LIM; k++) {
		for (int j = 0; j < 2; j++) {
			p->x = x->x + DX[ways[j]] * k;
			p->y = x->y + DY[ways[j]] * k;
			if (fits(i, NULL, 0, -1)) {
				commit(i, NULL, 0);
				return 1;
			}
		}
	}
	return 0;
}

static int place_between(int i)
{
	struct part *p = &part[i];
	struct gpin *a = &pin[p->pa], *b = &pin[p->pb];
	int dx = b->x - a->x, dy = b->y - a->y, w = p->way;
	int axis = abs(dx) >= abs(dy) ? (dx >= 0 ? R : L) : (dy >= 0 ? U : D);
	struct spin *s0 = &p->sh->pin[0], *s1 = &p->sh->pin[1];
	int sx = s1->x - s0->x, sy = s1->y - s0->y;
	int saxis = abs(sx) >= abs(sy) ? (sx >= 0 ? R : L) : (sy >= 0 ? U : D);
	int base = 0;
	if (w == axis || w == OPP(axis)) {
		complain(src, p->line,
			 "between needs a way across the line of its pins");
		p->ok = 0;
		return 1;
	}
	aim(i, 0, OPP(axis));
	p->rot = (axis - tdir(saxis, 0, p->mir) + 4) &
		 3; // The pins run the way of the line.
	p->lside = p->side >= 0 ? p->side : tdir(p->sh->side, p->rot, p->mir);
	base = w == U ? (a->y > b->y ? a->y : b->y) :
	       w == D ? (a->y < b->y ? a->y : b->y) :
	       w == R ? (a->x > b->x ? a->x : b->x) :
			(a->x < b->x ? a->x : b->x);
	for (int s = 1; s < LIM; s++) {
		int mx = w == U || w == D ? (a->x + b->x) / 2 :
					    base + DX[w] * s;
		int my = w == U || w == D ? base + DY[w] * s :
					    (a->y + b->y) / 2;
		p->x = p->y = 0;
		set_pins(i);
		p->x = mx - (pin[p->base].x + pin[p->base + 1].x) / 2;
		p->y = my - (pin[p->base].y + pin[p->base + 1].y) / 2;
		if (fits(i, NULL, 0, -1)) {
			commit(i, NULL, 0);
			return 1;
		}
	}
	return 0;
}

// Places a supply or a port at the end of its pin's line.
static int place_flag(int f)
{
	struct flag *fl = &flag[f];
	struct gpin *a = &pin[fl->at];
	struct item r[2];
	double b[4];
	int t0 = a->reach + 1;
	cur = part[a->part].clu;
	// Past every pin of its net placed so far, so each joins its line.
	for (int k = 0; k < npin; k++) {
		struct gpin *q = &pin[k];
		int ahead = (q->x - a->x) * DX[a->dir] +
			    (q->y - a->y) * DY[a->dir] + 1;
		if (q->net == a->net && q->part >= 0 && part[q->part].placed &&
		    part[q->part].clu == cur) {
			t0 = ahead > t0 ? ahead : t0;
		}
	}
	for (int t = t0; t < t0 + LIM; t++) {
		put_flag(f, t);
		set_item(&r[0], fl->r, FLAGBOX, -1 - f, -1);
		seg(b, a->x, a->y, pin[fl->gp].x, pin[fl->gp].y);
		set_item(&r[1], b, LEAD, -1 - f, a->net);
		if (clear_all(r, 2, -1, a->net, -1)) {
			fl->placed = 1;
			fl->clu = cur;
			add_item(fl->r, FLAGBOX, -1 - f, -1);
			add_item(b, LEAD, -1 - f, a->net);
			a->reach = t;
			return 1;
		}
	}
	return 0;
}

// Whether a pin's part is placed and no branch across its line waits.
static int line_done(int g, int along)
{
	struct gpin *a = &pin[g];
	if (a->part < 0 || !part[a->part].placed) {
		return 0;
	}
	for (int j = 0; j < npart; j++) {
		struct part *q = &part[j];
		if (q->ok && !q->placed && q->how == FROM && q->pa == g &&
		    (!along || q->way != a->dir)) {
			return 0;
		}
	}
	return 1;
}

// Whether the target of an "of" is placed whole, in clusters closed.
static int target_done(const struct part *p)
{
	int any = 0;
	if (!p->group) {
		return part[p->pa].placed;
	}
	for (int j = 0; j < npart; j++) {
		size_t n = strlen(group[p->pa]);
		if (!strncmp(part[j].name, group[p->pa], n) &&
		    part[j].name[n] == '.' && part[j].ok) {
			if (!part[j].placed) {
				return 0;
			}
			any = 1;
		}
	}
	return any;
}

// The box of an item's owner within a group or a part.
static int owned_by(const struct item *t, const struct part *p)
{
	int o = t->owner >= 0 ? t->owner : pin[flag[-1 - t->owner].at].part;
	size_t n = p->group ? strlen(group[p->pa]) : 0;
	if (!p->group) {
		return o == p->pa;
	}
	return !strncmp(part[o].name, group[p->pa], n) &&
	       part[o].name[n] == '.';
}

static void shift(int c, int dx, int dy)
{
	for (int i = 0; i < npart; i++) {
		if (part[i].placed && part[i].clu == c) {
			part[i].x += dx;
			part[i].y += dy;
			set_pins(i);
		}
	}
	for (int f = 0; f < nflag; f++) {
		struct flag *fl = &flag[f];
		if (fl->placed && fl->clu == c) {
			pin[fl->gp].x += dx;
			pin[fl->gp].y += dy;
			fl->r[0] += dx;
			fl->r[2] += dx;
			fl->r[1] += dy;
			fl->r[3] += dy;
			fl->tx += dx;
			fl->ty += dy;
		}
	}
	for (int t = 0; t < nitem; t++) {
		if (item[t].clu == c) {
			item[t].r[0] += dx;
			item[t].r[2] += dx;
			item[t].r[1] += dy;
			item[t].r[3] += dy;
		}
	}
}

static int box_of(int c, int with, double *r)
{
	int any = 0;
	r[0] = r[1] = 1e9;
	r[2] = r[3] = -1e9;
	for (int t = 0; t < nitem; t++) {
		if (with ? item[t].clu == c : item[t].clu != c) {
			grow_rect(r, item[t].r);
			any = 1;
		}
	}
	return any;
}

// Moves cluster c, whose root is part i, out its way until it keeps two
// steps from all the clusters before it.
static void settle(int c, int i)
{
	struct part *p = &part[i];
	double cb[4], t[4], room2 = 2;
	int w = p->how == OF ? p->way : R, dx = 0, dy = 0, ok = 0;
	(void)box_of(c, 1, cb);
	if (!box_of(c, 0, t)) {
		return;
	}
	if (p->how == OF) {
		struct part *x = p->group ? NULL : &part[p->pa];
		t[0] = t[1] = 1e9;
		t[2] = t[3] = -1e9;
		for (int k = 0; k < nitem; k++) {
			if (item[k].clu != c && owned_by(&item[k], p)) {
				grow_rect(t, item[k].r);
			}
		}
		if (w == U || w == D) {
			dx = x ? x->x - p->x : (int)lround(t[0] - cb[0]);
		} else {
			dy = x ? x->y - p->y : (int)lround(t[3] - cb[3]);
		}
	}
	if (w == R) {
		dx = up_to(t[2] + room2 - cb[0]);
	} else if (w == L) {
		dx = down_to(t[0] - room2 - cb[2]);
	} else if (w == U) {
		dy = up_to(t[3] + room2 - cb[1]);
	} else {
		dy = down_to(t[1] - room2 - cb[3]);
	}
	for (int k = 0; k < 4 * LIM && !ok; k++) {
		ok = 1;
		for (int a = 0; a < nitem && ok; a++) {
			for (int b = 0; b < nitem && ok && item[a].clu == c;
			     b++) {
				double r[4] = { item[a].r[0] + dx,
						item[a].r[1] + dy,
						item[a].r[2] + dx,
						item[a].r[3] + dy };
				ok = item[b].clu == c ||
				     apart(r, item[b].r, room2);
			}
		}
		if (!ok) {
			dx += DX[w];
			dy += DY[w];
		}
	}
	shift(c, dx, dy);
}

static void place(void)
{
	int root = -1;
	for (;;) {
		int done = 0;
		for (int i = 0; i < npart && !done && cur >= 0; i++) {
			struct part *p = &part[i];
			int ready = 0;
			if (!p->ok || p->placed) {
				continue;
			}
			if (p->how == FROM) {
				struct gpin *a = &pin[p->pa];
				ready = a->part >= 0 && part[a->part].placed &&
					(p->way != a->dir ||
					 line_done(p->pa, 1));
			} else if (p->how == BESIDE) {
				ready = part[p->pa].placed;
			} else if (p->how == BETWEEN) {
				ready = pin[p->pa].part >= 0 &&
					pin[p->pb].part >= 0 &&
					part[pin[p->pa].part].placed &&
					part[pin[p->pb].part].placed;
			}
			if (!ready ||
			    (p->how == BESIDE ?
				     part[p->pa].clu :
				     part[pin[p->pa].part].clu) != cur) {
				continue;
			}
			done = 1;
			if (p->how == FROM   ? !place_from(i) :
			    p->how == BESIDE ? !place_beside(i) :
					       !place_between(i)) {
				complain(src, p->line, "%s finds no place",
					 p->name);
				p->ok = 0;
			}
		}
		for (int f = 0; f < nflag && !done && cur >= 0; f++) {
			if (flag[f].ok && !flag[f].placed &&
			    flag[f].kind >= SUPPLY &&
			    line_done(flag[f].at, 0) &&
			    part[pin[flag[f].at].part].clu == cur) {
				done = 1;
				if (!place_flag(f)) {
					complain(src, flag[f].line,
						 "the %s flag finds no place",
						 flag[f].net);
					flag[f].ok = 0;
				}
			}
		}
		if (done) {
			continue;
		}
		if (cur >= 0) {
			settle(cur, root);
			cur = -1;
			continue;
		}
		for (int i = 0; i < npart && !done; i++) {
			struct part *p = &part[i];
			if (p->ok && !p->placed &&
			    (p->how == FREE ||
			     (p->how == OF && target_done(p)))) {
				cur = nclu++;
				root = i;
				aim(i, 0, p->sh->npin ? p->sh->pin[0].dir : R);
				p->rot = 0;
				p->lside = p->side >= 0 ?
						   p->side :
						   tdir(p->sh->side, 0, p->mir);
				p->x = p->y = 0;
				(void)fits(i, NULL, 0, -1);
				commit(i, NULL, 0);
				done = 1;
			}
		}
		if (!done) {
			break;
		}
	}
	for (int i = 0; i < npart; i++) {
		if (part[i].ok && !part[i].placed) {
			complain(src, part[i].line, "%s finds no place",
				 part[i].name);
			part[i].ok = 0;
		}
	}
	for (int f = 0; f < nflag; f++) {
		flag[f].ok &= flag[f].placed;
	}
}

// A random number from xorshift64, seeded once from /dev/urandom, or
// from the clock if it cannot be read.
static uint64_t rnd(void)
{
	static uint64_t x;
	if (!x) {
		FILE *f = fopen("/dev/urandom", "rb");
		if (!f || fread(&x, sizeof(x), 1, f) != 1) {
			x = (uint64_t)time(NULL) ^ ((uint64_t)clock() << 20);
		}
		if (f) {
			(void)fclose(f);
		}
		x |= 1;
	}
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	return x;
}

// Gives each part a random designator, a new one at every run: its
// prefix and the page, as R4xx, and a number unique among the parts of
// its prefix, shuffled.  Nothing hangs on designators: the parts are
// known by their names (the path of their groups and their own)
// everywhere, the checks of the drawing among them, so a designator is
// for the reader alone.
static void designate(void)
{
	int *order = xmalloc((size_t)npart * sizeof(int)), n = 0;
	for (int i = 0; i < npart; i++) {
		if (part[i].placed) {
			order[n++] = i;
		}
	}
	// Each prefix together, then the parts of each in random order.
	for (int a = 1; a < n; a++) {
		for (int b = a; b > 0 && strcmp(part[order[b - 1]].prefix,
						part[order[b]].prefix) > 0;
		     b--) {
			int t = order[b];
			order[b] = order[b - 1];
			order[b - 1] = t;
		}
	}
	for (int a = 0, e = 0; a < n; a = e) {
		for (e = a; e < n && !strcmp(part[order[e]].prefix,
					     part[order[a]].prefix);
		     e++) {
		}
		for (int k = e - 1; k > a; k--) {
			int j = a + (int)(rnd() % (uint64_t)(k - a + 1));
			int t = order[k];
			order[k] = order[j];
			order[j] = t;
		}
		for (int k = a; k < e; k++) {
			struct part *p = &part[order[k]];
			(void)snprintf(p->ref, sizeof(p->ref), "%s%d",
				       p->prefix, page * 100 + k - a + 1);
			label_size(order[k]);
		}
	}
	free(order);
}

// ---- Routing on the grid

// A group of a net routed as one tree; own: the pin whose flag it
// joins, or -1.
struct grp {
	int net, own, n, np, cp, cost, *t, *path;
};
static struct grp *grp;
static int ngrp, cgrp;
static int W, H, X0, Y0;
static int *hw, *vw,
	*pinat; // Group on the edge right of and above each node; pin + 1 at it.
static unsigned char **mask; // For each net: 0 free, 1 near a body, 2 shut.
static int *dist, *prev, *heap, nheap, cheap;

static const int BEND = 2, NEAR = 1, CROSS = 2, JNEAR = 3, OFFSIDE = 3;

static int node(int x, int y)
{
	return (y - Y0) * W + (x - X0);
}

static int *edge(int n, int e)
{
	int x = n % W, y = n / W;
	if (e == R) {
		return x + 1 < W ? &hw[n] : NULL;
	}
	if (e == L) {
		return x > 0 ? &hw[n - 1] : NULL;
	}
	if (e == U) {
		return y + 1 < H ? &vw[n] : NULL;
	}
	return y > 0 ? &vw[n - W] : NULL;
}

static int step(int n, int e)
{
	int x = n % W + DX[e], y = n / W + DY[e];
	return x < 0 || y < 0 || x >= W || y >= H ? -1 : y * W + x;
}

// The edges of group g at node n, and of other groups.
static int deg(int n, int g, int *other)
{
	int k = 0;
	*other = 0;
	for (int e = 0; e < 4; e++) {
		int *p = edge(n, e);
		k += p && *p == g;
		*other += p && *p >= 0 && *p != g;
	}
	return k;
}

static unsigned char *net_mask(int nt)
{
	unsigned char *m = mask[nt];
	if (m) {
		return m;
	}
	m = mask[nt] = xcalloc((size_t)(W * H), 1);
	for (int t = 0; t < nitem; t++) {
		struct item *it = &item[t];
		int own = 0;
		double g = clr[C_BODY];
		if (it->kind == BODY) {
			own = on_net(it->owner, nt);
		} else if (it->kind == FLAGBOX) {
			own = pin[flag[-1 - it->owner].gp].net == nt;
		} else {
			continue;
		}
		for (int y = down_to(it->r[1] - g); y <= up_to(it->r[3] + g);
		     y++) {
			for (int x = down_to(it->r[0] - g);
			     x <= up_to(it->r[2] + g); x++) {
				int n = node(x, y), v = 0;
				if (x < X0 || y < Y0 || x >= X0 + W ||
				    y >= Y0 + H) {
					continue;
				}
				v = inside(it->r, 0, x, y) && it->kind == BODY ?
					    2 :
				    inside(it->r, g, x, y) ? (own ? 1 : 2) :
							     0;
				m[n] = (unsigned char)(m[n] > v ? m[n] : v);
			}
		}
	}
	return m;
}

static void push(int s)
{
	int i = 0;
	*ADD(heap, nheap, cheap) = s;
	i = nheap - 1;
	while (i > 0 && dist[heap[(i - 1) / 2]] > dist[heap[i]]) {
		int t = heap[i];
		heap[i] = heap[(i - 1) / 2];
		heap[(i - 1) / 2] = t;
		i = (i - 1) / 2;
	}
}

static int pop(void)
{
	int top = heap[0], i = 0;
	heap[0] = heap[--nheap];
	for (;;) {
		int a = 2 * i + 1, b = a + 1, m = i, t = 0;
		m = a < nheap && dist[heap[a]] < dist[heap[m]] ? a : m;
		m = b < nheap && dist[heap[b]] < dist[heap[m]] ? b : m;
		if (m == i) {
			return top;
		}
		t = heap[i];
		heap[i] = heap[m];
		heap[m] = t;
		i = m;
	}
}

// Whether node n lies past the side way said of the pins a and b.
static int offside(int n, int way, const struct gpin *a, const struct gpin *b)
{
	int x = n % W + X0, y = n / W + Y0;
	if (way < 0 || !b) {
		return 0;
	}
	if (way == D) {
		return y > (a->y > b->y ? a->y : b->y);
	}
	if (way == U) {
		return y < (a->y < b->y ? a->y : b->y);
	}
	if (way == L) {
		return x > (a->x > b->x ? a->x : b->x);
	}
	return x < (a->x < b->x ? a->x : b->x);
}

// Lays the cheapest path of group g from pin s to pin t, or to the
// tree of g if t < 0; its cost, or -1 if there is no way.
static int search(int g, int s, int t, int way, const struct gpin *wa,
		  const struct gpin *wb)
{
	struct gpin *ps = &pin[s];
	unsigned char *m = net_mask(grp[g].net);
	int n0 = node(ps->x, ps->y), d0 = ps->dir, goal = -1;
	for (int i = 0; i < W * H * 4; i++) {
		dist[i] = INT32_MAX;
	}
	nheap = 0;
	dist[n0 * 4 + d0] = 0;
	prev[n0 * 4 + d0] = -1;
	push(n0 * 4 + d0);
	while (nheap > 0) {
		int st = pop(), n = st / 4, din = st % 4, other = 0;
		int own = deg(n, g, &other);
		if (n != n0 && (own > 0 || (t >= 0 && pinat[n] == t + 1))) {
			goal = st;
			break;
		}
		for (int e = 0; e < 4; e++) {
			int k = step(n, e), c = dist[st] + 1, *ep = edge(n, e);
			int kown = 0, kother = 0;
			if (k < 0 || !ep || *ep >= 0 || (n == n0 && e != d0) ||
			    (n != n0 && e == OPP(din)) || (other && e != din)) {
				continue;
			}
			c += n != n0 && e != din ? BEND : 0;
			if (pinat[k]) {
				if (t < 0 || pinat[k] != t + 1 ||
				    e != OPP(pin[t].dir)) {
					continue;
				}
			} else {
				if (m[k] == 2) {
					continue;
				}
				c += m[k] == 1 ? NEAR : 0;
				kown = deg(k, g, &kother);
				if (kother) {
					int *a = edge(k, (e + 1) & 3),
					    *b = edge(k, (e + 3) & 3);
					int *f = edge(k, e);
					if (kown || kother != 2 || !a || !b ||
					    *a != *b || (f && *f >= 0)) {
						continue;
					}
					c += CROSS;
				}
				if (kown && (t >= 0 || kown != 2)) {
					continue;
				}
				for (int j = 0; kown && j < 4; j++) {
					int q = step(k, j), qo = 0;
					c += q >= 0 && deg(q, g, &qo) >= 3 ?
						     JNEAR :
						     0;
				}
				for (int j = 1; j < 4; j += 2) {
					int q = step(k, (e + j) & 3);
					int *a = q >= 0 ? edge(q, e) : NULL,
					    *b = q >= 0 ? edge(q, OPP(e)) :
							  NULL;
					c += (a && *a >= 0 && *a != g) ||
							     (b && *b >= 0 &&
							      *b != g) ?
						     NEAR :
						     0;
				}
			}
			c += offside(k, way, wa, wb) ? OFFSIDE : 0;
			if (c < dist[k * 4 + e]) {
				dist[k * 4 + e] = c;
				prev[k * 4 + e] = st;
				push(k * 4 + e);
			}
		}
	}
	if (goal < 0) {
		return -1;
	}
	{
		struct grp *gr = &grp[g];
		int cost = dist[goal];
		for (int st = goal; st >= 0; st = prev[st]) {
			int pv = prev[st];
			int *ep = pv >= 0 ? edge(pv / 4, st % 4) : NULL;
			*ADD(gr->path, gr->np, gr->cp) = st / 4;
			if (ep) {
				*ep = g;
			}
		}
		*ADD(gr->path, gr->np, gr->cp) = -1;
		return cost;
	}
}

static void unlay(int g)
{
	for (int i = 0; i < W * H; i++) {
		hw[i] = hw[i] == g ? -1 : hw[i];
		vw[i] = vw[i] == g ? -1 : vw[i];
	}
	grp[g].np = 0;
	grp[g].cost = 0;
}

// The wire said between pins a and b, or -1.
static int said(int a, int b)
{
	for (int w = 0; w < nwire; w++) {
		if ((wire[w].pa == a && wire[w].pb == b) ||
		    (wire[w].pa == b && wire[w].pb == a)) {
			return w;
		}
	}
	return -1;
}

static int route_group(int g)
{
	struct grp *gr = &grp[g];
	int a = -1, b = -1, best = -1, w = -1, c = 0, *in = NULL, found = 0;
	if (gr->n < 2) {
		return 1;
	}
	// First a wire the source names; else the two pins that face each
	// other on one line farthest apart; else the nearest two.
	for (int i = 0; i < gr->n && w < 0; i++) {
		for (int j = 0; j < gr->n && w < 0; j++) {
			w = said(gr->t[i], gr->t[j]);
			a = w >= 0 ? gr->t[i] : a;
			b = w >= 0 ? gr->t[j] : b;
		}
	}
	found = w >= 0;
	for (int i = 0; i < gr->n && !found; i++) {
		for (int j = 0; j < gr->n; j++) {
			struct gpin *p = &pin[gr->t[i]], *q = &pin[gr->t[j]];
			int far = abs(q->x - p->x) + abs(q->y - p->y);
			if (((p->dir == R && q->dir == L && p->y == q->y &&
			      p->x < q->x) ||
			     (p->dir == U && q->dir == D && p->x == q->x &&
			      p->y < q->y)) &&
			    far > best) {
				best = far;
				a = gr->t[i];
				b = gr->t[j];
			}
		}
	}
	found |= best >= 0;
	best = INT32_MAX;
	for (int j = 1; j < gr->n && !found; j++) {
		struct gpin *p = &pin[gr->t[0]], *q = &pin[gr->t[j]];
		int far = abs(q->x - p->x) + abs(q->y - p->y);
		if (far < best) {
			best = far;
			a = gr->t[0];
			b = gr->t[j];
		}
	}
	c = search(g, a, b, w >= 0 ? wire[w].way : -1, &pin[a], &pin[b]);
	if (c < 0) {
		return 0;
	}
	gr->cost = c;
	in = xcalloc((size_t)gr->n, sizeof(int));
	for (int i = 0; i < gr->n; i++) {
		in[i] = gr->t[i] == a || gr->t[i] == b;
	}
	for (;;) {
		int pick = -1, near = INT32_MAX, way = -1, o = -1;
		for (int i = 0; i < gr->n; i++) {
			for (int k = 0; !in[i] && k < gr->np; k++) {
				int nd = gr->path[k], dd = 0;
				if (nd < 0) {
					continue;
				}
				dd = abs(nd % W + X0 - pin[gr->t[i]].x) +
				     abs(nd / W + Y0 - pin[gr->t[i]].y);
				if (dd < near) {
					near = dd;
					pick = i;
				}
			}
		}
		if (pick < 0) {
			break;
		}
		for (int j = 0; j < gr->n && way < 0; j++) {
			int sw = in[j] ? said(gr->t[pick], gr->t[j]) : -1;
			way = sw >= 0 ? wire[sw].way : -1;
			o = sw >= 0 ? gr->t[j] : -1;
		}
		c = search(g, gr->t[pick], -1, way, &pin[gr->t[pick]],
			   o >= 0 ? &pin[o] : NULL);
		if (c < 0) {
			free(in);
			return 0;
		}
		gr->cost += c;
		in[pick] = 1;
	}
	free(in);
	return 1;
}

static int has_flag(int gp)
{
	for (int f = 0; f < nflag; f++) {
		if (flag[f].ok && flag[f].at == gp) {
			return 1;
		}
	}
	return 0;
}

// Makes the groups of each net: a pin with a flag of its own and the
// flag; the other pins of the net with the first such pin and its flag.
static void make_groups(void)
{
	for (int nt = 0; nt < nnet; nt++) {
		int gm = ngrp, first = -1;
		for (int i = 0; i < npin && first < 0; i++) {
			first = pin[i].net == nt && pin[i].part >= 0 &&
						part[pin[i].part].placed &&
						has_flag(i) ?
					i :
					-1;
		}
		ADD(grp, ngrp, cgrp)->net = nt;
		grp[gm].own = first;
		for (int i = 0; i < npin; i++) {
			struct gpin *q = &pin[i];
			int live = q->part >= 0 ? part[q->part].placed :
						  flag[-1 - q->part].ok;
			int owner = q->part >= 0 ? (has_flag(i) ? i : first) :
						   flag[-1 - q->part].at;
			int g = gm;
			if (q->net != nt || !live) {
				continue;
			}
			while (g < ngrp && grp[g].own != owner) {
				g++;
			}
			if (g == ngrp) {
				struct grp *e = ADD(grp, ngrp, cgrp);
				e->net = nt;
				e->own = owner;
			}
			grp[g].t = xrealloc(grp[g].t, (size_t)(grp[g].n + 1) *
							      sizeof(int));
			grp[g].t[grp[g].n++] = i;
		}
	}
}

static void route(void)
{
	int *order = NULL, lo[2] = { 1 << 30, 1 << 30 },
	    hi[2] = { -(1 << 30), -(1 << 30) };
	for (int t = 0; t < nitem; t++) {
		lo[0] = lo[0] < down_to(item[t].r[0]) ? lo[0] :
							down_to(item[t].r[0]);
		lo[1] = lo[1] < down_to(item[t].r[1]) ? lo[1] :
							down_to(item[t].r[1]);
		hi[0] = hi[0] > up_to(item[t].r[2]) ? hi[0] :
						      up_to(item[t].r[2]);
		hi[1] = hi[1] > up_to(item[t].r[3]) ? hi[1] :
						      up_to(item[t].r[3]);
	}
	if (!nitem) {
		return;
	}
	X0 = lo[0] - 6;
	Y0 = lo[1] - 6;
	W = hi[0] - lo[0] + 13;
	H = hi[1] - lo[1] + 13;
	hw = xmalloc((size_t)(W * H) * sizeof(int));
	vw = xmalloc((size_t)(W * H) * sizeof(int));
	pinat = xcalloc((size_t)(W * H), sizeof(int));
	dist = xmalloc((size_t)(W * H * 4) * sizeof(int));
	prev = xmalloc((size_t)(W * H * 4) * sizeof(int));
	mask = xcalloc((size_t)nnet + 1, sizeof(*mask));
	make_groups();
	for (int g = 0; g < ngrp; g++) {
		for (int k = 0; k < grp[g].n; k++) {
			pinat[node(pin[grp[g].t[k]].x, pin[grp[g].t[k]].y)] =
				grp[g].t[k] + 1;
		}
	}
	order = xmalloc((size_t)(ngrp + 1) * sizeof(int));
	for (int g = 0; g < ngrp; g++) {
		order[g] = g;
	}
	// A group that finds no way goes first, and all are routed again.
	for (int pass = 0; pass <= ngrp + 1; pass++) {
		int failed = -1, last = pass == ngrp + 1;
		for (int i = 0; i < W * H; i++) {
			hw[i] = vw[i] = -1;
		}
		for (int g = 0; g < ngrp; g++) {
			grp[g].np = 0;
		}
		for (int k = 0; k < ngrp; k++) {
			if (!route_group(order[k])) {
				unlay(order[k]);
				failed = failed < 0 ? k : failed;
				if (!last) {
					break;
				}
				complain(src, 0, "net %s cannot be routed",
					 net[grp[order[k]].net].name);
			}
		}
		if (failed < 0 || last) {
			break;
		}
		if (failed == 0) {
			pass = ngrp;
			continue;
		}
		for (int k = failed; k > 0; k--) {
			int t = order[k];
			order[k] = order[k - 1];
			order[k - 1] = t;
		}
	}
	// Each group again among the others: the cheaper way stays.
	for (int g = 0; g < ngrp; g++) {
		struct grp *gr = &grp[g];
		int old = gr->cost, n = gr->np, *keep = NULL;
		if (gr->n < 2 || !n) {
			continue;
		}
		keep = xmalloc((size_t)n * sizeof(int));
		memcpy(keep, gr->path, (size_t)n * sizeof(int));
		unlay(g);
		if (!route_group(g) || gr->cost >= old) {
			unlay(g);
			for (int k = 0; k + 1 < n; k++) {
				if (keep[k] >= 0 && keep[k + 1] >= 0) {
					for (int e = 0; e < 4; e++) {
						int *ep =
							step(keep[k],
							     e) == keep[k + 1] ?
								edge(keep[k],
								     e) :
								NULL;
						if (ep) {
							*ep = g;
						}
					}
				}
			}
			memcpy(gr->path, keep, (size_t)n * sizeof(int));
			gr->np = n;
			gr->cost = old;
		}
		free(keep);
	}
	free(order);
}

// ---- Labels last, then the check

// How much of the drawing rect r covers, besides part i's own body.
static double cover(const double *r, int i)
{
	double s = 0, g = clr[C_TEXT];
	for (int t = 0; t < nitem; t++) {
		const double *o = item[t].r;
		double k = item[t].kind == FLAGBOX ? clr[C_FLAG] : g;
		double w = fmin(r[2] + k, o[2]) - fmax(r[0] - k, o[0]);
		double h = fmin(r[3] + k, o[3]) - fmax(r[1] - k, o[1]);
		if (item[t].kind == LEAD ||
		    (item[t].kind == BODY && item[t].owner == i) ||
		    (item[t].kind == LABEL && item[t].owner == i)) {
			continue;
		}
		if (w > -EPS && h > -EPS) {
			s += fmax(w, 0.1) * fmax(h, 0.1);
		}
	}
	for (int w = 0; w < nwire; w++) {
		double b[4];
		int wn = wire[w].pa >= 0 && wire[w].pb >= 0 ?
				 pin[wire[w].pa].net :
				 -1;
		if (wn >= 0 && !on_net(i, wn) && loop_box(w, part[i].clu, b) &&
		    !apart(r, b, 0)) {
			s += 10;
		}
	}
	for (int g2 = 0; g2 < ngrp; g2++) {
		for (int k = 0; k + 1 < grp[g2].np; k++) {
			int a = grp[g2].path[k], b = grp[g2].path[k + 1];
			double l[4];
			if (a < 0 || b < 0) {
				continue;
			}
			seg(l, a % W + X0, a / W + Y0, b % W + X0, b / W + Y0);
			if (!apart(r, l, g)) {
				s += 1;
			}
		}
	}
	return s;
}

static void place_labels(void)
{
	for (int i = 0; i < npart; i++) {
		struct part *p = &part[i];
		double best = 1e18, r[4];
		int lab = -1, home = 0;
		if (!p->placed) {
			continue;
		}
		for (int t = 0; t < nitem; t++) {
			lab = item[t].kind == LABEL && item[t].owner == i ? t :
									    lab;
		}
		home = p->side >= 0 ? p->side :
				      tdir(p->sh->side, p->rot, p->mir);
		for (int k = 0; k < 4 && best > EPS; k++) {
			int side = (home + (k == 1 ? 2 : k == 2 ? 1 : k)) & 3;
			for (int j = 0; j < slides(i, side) && best > EPS;
			     j++) {
				double slide = (j + 1) / 2 * 0.5 *
					       (j % 2 ? 1 : -1),
				       c = 0;
				label_rect(i, side, slide, r);
				c = cover(r, i);
				if (c < best) {
					best = c;
					p->lside = side;
					p->slide = slide;
				}
			}
		}
		label_rect(i, p->lside, p->slide, item[lab].r);
	}
}

static char *who(int i)
{
	char buf[256];
	(void)snprintf(buf, sizeof(buf), "%s (%s)", part[i].name, part[i].ref);
	return xstrdup(buf);
}

// Measures every pair that must keep room, and tells each too close.
static void check(void)
{
	for (int a = 0; a < nitem; a++) {
		struct item *x = &item[a];
		int xi = x->owner >= 0 ? x->owner : -1;
		if (x->kind == LEAD) {
			continue;
		}
		for (int b = a + 1; b < nitem; b++) {
			struct item *y = &item[b];
			int yi = y->owner >= 0 ? y->owner : -1, k = -1;
			if (y->kind == LEAD) {
				continue;
			}
			if (x->kind == BODY && y->kind == BODY) {
				k = C_BODY;
			} else if (x->kind == LABEL || y->kind == LABEL) {
				int same = x->owner == y->owner;
				k = same ? C_LABEL :
				    x->kind == FLAGBOX || y->kind == FLAGBOX ?
					   C_FLAG :
					   C_TEXT;
			}
			if (k >= 0 && !apart(x->r, y->r, clr[k] - 1e-3)) {
				const struct item *p = x->owner >= 0 ? x : y;
				char *n1 =
					xi >= 0 ?
						who(xi) :
						xstrdup(flag[-1 - x->owner].net);
				char *n2 =
					yi >= 0 ?
						who(yi) :
						xstrdup(flag[-1 - y->owner].net);
				complain(
					src,
					p->owner >= 0 ? part[p->owner].line : 0,
					"%s%s and %s%s are too close",
					x->kind == LABEL ? "the label of " : "",
					n1,
					y->kind == LABEL ? "the label of " : "",
					n2);
				free(n1);
				free(n2);
			}
		}
	}
	for (int g = 0; g < ngrp; g++) {
		for (int k = 0; k + 1 < grp[g].np; k++) {
			int a = grp[g].path[k], b = grp[g].path[k + 1];
			double l[4];
			if (a < 0 || b < 0) {
				continue;
			}
			seg(l, a % W + X0, a / W + Y0, b % W + X0, b / W + Y0);
			for (int t = 0; t < nitem; t++) {
				struct item *it = &item[t];
				int bad = it->kind == BODY &&
					  !on_net(it->owner, grp[g].net) &&
					  !apart(l, it->r, clr[C_BODY] - 1e-3);
				bad |= it->kind == LABEL &&
				       !apart(l, it->r, clr[C_TEXT] - 1e-3);
				if (bad) {
					char *n = who(it->owner);
					complain(
						src, part[it->owner].line,
						"%s%s and net %s are too close",
						it->kind == LABEL ?
							"the label of " :
							"",
						n, net[grp[g].net].name);
					free(n);
				}
			}
		}
	}
}

// ---- The page

static double OX, OY; // The page point of grid point (0, 0).

static void xy(double x, double y)
{
	printf(" %.3f %.3f", OX + x * G, OY + y * G);
}

static void text(int italic, double size, double x, double y, const char *s)
{
	printf("text %c %.3f 0 0 %.3f %.3f %s\n", italic ? 'I' : 'R', size, x,
	       y, s);
}

// Draws a shape at grid point (x, y), as it stands.
static void draw_shape(const struct shape *sh, int rot, int mir, int flipped,
		       int x, int y)
{
	for (int i = 0; i < sh->nrec; i++) {
		struct rec *r = &sh->rec[i];
		double px = 0, py = 0;
		if ((r->cond == 1 && flipped) || (r->cond == 2 && !flipped)) {
			continue;
		}
		if (r->kind == K_CIRCLE || r->kind == K_DISK) {
			turn(rot, mir, r->v[0], r->v[1], &px, &py);
			px = OX + x * G + px * scale;
			py = OY + y * G + py * scale;
			if (r->kind == K_DISK) {
				printf("disk %.3f %.3f %.3f\n", px, py,
				       r->v[2] * scale);
				continue;
			}
			printf("line %.3f", r->w * scale);
			for (int k = 0; k <= 48; k++) {
				printf(" %.3f %.3f",
				       px + r->v[2] * scale * cos(k * PI / 24),
				       py + r->v[2] * scale * sin(k * PI / 24));
			}
			printf("\n");
			continue;
		}
		if (r->kind == K_BLANK) {
			printf("clear\n");
		}
		if (r->kind == K_FILL || r->kind == K_BLANK) {
			printf("poly");
		} else {
			printf("line %.3f", r->w * scale);
		}
		for (int k = 0; k <= r->n / 2 - (r->kind != K_LOOP); k++) {
			int j = k % (r->n / 2);
			turn(rot, mir, r->v[2 * j], r->v[2 * j + 1], &px, &py);
			printf(" %.3f %.3f", OX + x * G + px * scale,
			       OY + y * G + py * scale);
		}
		printf("\n%s", r->kind == K_BLANK ? "dark\n" : "");
	}
}

static const char *field_of(const char *name)
{
	for (int i = nfield - 1; i >= 0; i--) {
		if (!strcmp(field[i].name, name)) {
			return field[i].text;
		}
	}
	return "";
}

static void draw_sheet(void)
{
	char *w[MAXW];
	for (int i = 0; i < nsheetln; i++) {
		char *copy = xstrdup(sheetln[i].s);
		size_t k = split_words(copy, w, MAXW);
		if (k >= 5 && k <= MAXW && !strcmp(w[0], "line")) {
			printf("line 0.50");
			for (size_t j = 1; j < k; j++) {
				printf(" %s", w[j]);
			}
			printf("\n");
		} else if (k >= 8 && !strcmp(w[0], "text")) {
			char *t = rest(sheetln[i].s, 7),
			     *dollar = strchr(t, '$');
			const char *f = dollar ? field_of(dollar + 1) : "";
			if (dollar) {
				*dollar = 0;
			}
			if (!dollar || *f) {
				printf("text %s %s %s %s %s %s %s%s\n", w[1],
				       w[2], w[3], w[4], w[5], w[6], t, f);
			}
			free(t);
		}
		free(copy);
	}
}

static void draw_part(int i)
{
	struct part *p = &part[i];
	double b[4], r[4], s = scale, top = 0, base = 0;
	const char *digits = p->ref + strlen(p->prefix);
	double w0 = text_w(1, p->prefix, 7 * s) + text_w(0, digits, 6 * s);
	double w1 = p->nlab > 1 ? text_w(0, p->vtext, 7 * s) : 0;
	printf("part %s %s\n", p->name, p->ref);
	body_rect(i, b);
	printf("name %s", p->name);
	xy(b[0], b[1]);
	xy(b[2], b[3]);
	printf("\n");
	draw_shape(p->sh, p->rot, p->mir, p->flip, p->x, p->y);
	for (int k = 0; k < p->sh->npin; k++) {
		struct gpin *g = &pin[p->base + k];
		if (g->net >= 0) {
			printf("pin %s.%s", p->ref, p->num[k]);
			xy(g->x, g->y);
			printf("\n");
		}
	}
	label_rect(i, p->lside, p->slide, r);
	top = OY + r[3] * G;
	base = top - 0.72 * 7 * s;
	for (int l = 0; l < p->nlab; l++) {
		double w = l ? w1 : w0, x = OX + r[0] * G;
		x += p->lside == L ? (r[2] - r[0]) * G - w :
		     p->lside == R ? 0 :
				     ((r[2] - r[0]) * G - w) / 2;
		if (l == 0) {
			text(1, 7 * s, x, base, p->prefix);
			text(0, 6 * s, x + text_w(1, p->prefix, 7 * s),
			     base - 1.6 * s, digits);
		} else {
			text(0, 7 * s, x, base - 8.5 * s, p->vtext);
		}
	}
	for (int k = 0; p->type && k < p->sh->npin; k++) {
		struct gpin *g = &pin[p->base + k];
		double px = OX + g->x * G, py = OY + g->y * G;
		double w = text_w(0, p->num[k], 5 * s);
		px += g->dir == L ? 2 * s : g->dir == R ? -2 * s - w : 1.5 * s;
		py += g->dir == L || g->dir == R ? 1.2 * s :
		      g->dir == U		 ? -2 * s - 0.72 * 5 * s :
						   2 * s;
		text(0, 5 * s, px, py, p->num[k]);
	}
	printf("part\n");
}

static void draw_flag(int f)
{
	struct flag *fl = &flag[f];
	struct gpin *q = &pin[fl->gp];
	draw_shape(fl->sh, fl->rot, 0, 0, q->x, q->y);
	if (fl->kind >= SUPPLY) {
		text(0, 7 * scale, OX + fl->tx * G, OY + fl->ty * G, fl->net);
	}
	printf("flag %s", fl->net);
	xy(q->x, q->y);
	printf("\n");
}

static void draw_nets(void)
{
	for (int nt = 0; nt < nnet; nt++) {
		int any = 0;
		for (int g = 0; g < ngrp; g++) {
			struct grp *gr = &grp[g];
			if (gr->net != nt || !gr->np) {
				continue;
			}
			if (!any++) {
				printf("net %s\n", net[nt].name);
			}
			for (int k = 0; k < gr->np; k++) {
				int a = gr->path[k],
				    b = k + 1 < gr->np ? gr->path[k + 1] : -1;
				int c = k > 0 ? gr->path[k - 1] : -1;
				if (a < 0) {
					continue;
				}
				// A point starts a line, ends it, or turns it.
				if (c < 0) {
					printf("line %.3f", 0.5 * scale);
					xy(a % W + X0, a / W + Y0);
				} else if (b < 0 || (b - a) != (a - c)) {
					xy(a % W + X0, a / W + Y0);
				}
				if (b < 0) {
					printf("\n");
				}
			}
			for (int n = 0; n < W * H; n++) {
				int o = 0;
				if (deg(n, g, &o) >= 3) {
					printf("disk");
					xy(n % W + X0, n / W + Y0);
					printf(" %.3f\n", 1.6 * scale);
				}
			}
		}
		if (any) {
			printf("net\n");
		}
	}
}

static void draw(void)
{
	double b[4] = { 1e9, 1e9, -1e9, -1e9 };
	for (int t = 0; t < nitem; t++) {
		grow_rect(b, item[t].r);
	}
	if (nitem) {
		OX = (room[0] + room[2]) / 2 - (b[0] + b[2]) / 2 * G;
		OY = (room[1] + room[3]) / 2 - (b[1] + b[3]) / 2 * G;
	}
	printf("# vec of %s, by schdraw2\n", src);
	printf("page %g %g\n", paper[0], paper[1]);
	draw_sheet();
	draw_nets();
	for (int i = 0; i < npart; i++) {
		if (part[i].placed) {
			draw_part(i);
		}
	}
	for (int f = 0; f < nflag; f++) {
		if (flag[f].ok) {
			draw_flag(f);
		}
	}
}

// The ferrites of the value the sheet uses most go without it.
static void common_ferrite(void)
{
	int best = 0;
	const char *v = NULL;
	for (int i = 0; i < npart; i++) {
		int n = 0;
		for (int j = 0;
		     part[i].value && !strcmp(part[i].sh->kind, "ferrite") &&
		     j < npart;
		     j++) {
			n += part[j].value &&
			     !strcmp(part[j].sh->kind, "ferrite") &&
			     !strcmp(part[j].value, part[i].value);
		}
		if (n > best) {
			best = n;
			v = part[i].value;
		}
	}
	for (int i = 0; v && i < npart; i++) {
		part[i].gave = part[i].value &&
			       !strcmp(part[i].sh->kind, "ferrite") &&
			       !strcmp(part[i].value, v);
	}
}

static void add_dir(char ***v, int *n, int *cap, const char *a, const char *b)
{
	*ADD(*v, *n, *cap) = cat3(a, "/", b);
}

int main(int argc, char **argv)
{
	int verbose = 0, nlib = 0;
	char *dir = NULL, *slash = NULL, *font = NULL;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) {
			verbose = 1;
		} else if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			i++;
		} else if (argv[i][0] == '-' || src) {
			die("usage: schdraw2 [-v] [-L lib] ... sheet.sch");
		} else {
			src = argv[i];
		}
	}
	if (!src) {
		die("usage: schdraw2 [-v] [-L lib] ... sheet.sch");
	}
	dir = xstrdup(src);
	slash = strrchr(dir, '/');
	if (slash) {
		slash[0] = 0;
	} else {
		free(dir);
		dir = xstrdup(".");
	}
	add_dir(&symdir, &nsymdir, &csymdir, dir, "symbols");
	add_dir(&sheetdir, &nsheetdir, &csheetdir, dir, "sheets");
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-L") && i + 1 < argc) {
			add_dir(&symdir, &nsymdir, &csymdir, argv[++i],
				"symbols");
			add_dir(&sheetdir, &nsheetdir, &csheetdir, argv[i],
				"sheets");
			font = font ? font :
				      cat3(argv[i], "/fonts/nimbussans.txt",
					   "");
			nlib++;
		}
	}
	if (!nlib) {
		char *prog = xstrdup(argv[0]), *cut = strrchr(prog, '/');
		char *lib = NULL;
		if (cut) {
			*cut = 0;
		}
		lib = cat3(cut ? prog : ".", "/../lib", "");
		add_dir(&symdir, &nsymdir, &csymdir, lib, "symbols");
		add_dir(&sheetdir, &nsheetdir, &csheetdir, lib, "sheets");
		font = cat3(lib, "/fonts/nimbussans.txt", "");
	}
	read_font(font);
	read_source();
	G = PT * scale;
	{
		const char *base = strrchr(src, '/');
		char pg[16];
		(void)snprintf(pg, sizeof(pg), "%d", page);
		set_field("File", xstrdup(base ? base + 1 : src));
		set_field("Page", xstrdup(pg));
	}
	{
		int got = read_sheet(sheetname ? sheetname : "sch_a");
		if (!got) {
			complain(src, 0, "no sheet %s in the library",
				 sheetname ? sheetname : "sch_a");
		}
		if (got <= 0) {
			return EXIT_FAILURE;
		}
	}
	make_nets();
	common_ferrite();
	// A designator of the right length holds the room of its label
	// until the parts are numbered.
	for (int i = 0; i < npart; i++) {
		int n = 0;
		for (int j = 0; j < npart; j++) {
			n += !strcmp(part[j].prefix, part[i].prefix);
		}
		(void)snprintf(part[i].ref, sizeof(part[i].ref), "%s%d",
			       part[i].prefix, page * 100 + n);
		label_size(i);
	}
	place();
	designate();
	route();
	place_labels();
	check();
	for (int i = 0; verbose && i < npart; i++) {
		if (part[i].placed) {
			static const char *const HOW[] = { "free", "from", "of",
							   "beside",
							   "between" };
			struct part *p = &part[i];
			(void)fprintf(
				stderr,
				"%s:%d: %s (%s) at %d %d, %s%s%s%s%s%s%s\n",
				src, p->line, p->name, p->ref, p->x, p->y,
				p->how == FROM || p->how == OF ?
					WAYNAME[p->way] :
					"",
				p->how == FROM || p->how == OF ? " " : "",
				HOW[p->how], p->a ? " " : "", p->a ? p->a : "",
				p->b ? " and " : "", p->b ? p->b : "");
		}
	}
	draw();
	close_stdout();
	return faults ? EXIT_FAILURE : EXIT_SUCCESS;
}
