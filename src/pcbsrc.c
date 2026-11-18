// SPDX-License-Identifier: GPL-3.0-only
// pcbsrc.c -- Read a board source, its netlist, footprints and lock
// Copyright (c) 2026 Jakob Kastelic

// See pcb(5) for the source and the lock, pcbfp(5) for the footprints
// and netlist(5) for the netlist.  Each fault names its file and line
// and the reading goes on, so one run lists them all.

#include "pcbsrc.h"
#include "stroke.h"
#include "util.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	LINE = 1U << 16U, // Room for a line.
	WORDS = 4096, // The most words of a line.
	NAME_LEN = 1024, // Room for a pin, PART.PIN.
	PIN_NAME_LEN = 64, // Room for the number of a pin of a name.
	TURN = 360, // A whole turn, and a quarter, in degrees.
	QUARTER = 90,
	KEY_SPARE = 32, // Room in a key beyond its names.
	RECT_WORDS = 6, // The words of board rect and cutout rect,
	ROUND_WORDS = 8, // and of cutout rect with round R.
	PART_WORDS = 7, // The words of a part record of the lock, past them
	// the words of what placed it;
	PAD_WORDS = 6, // of a pad record kept, of the REF left out;
	LINK_WORDS = 5, // of a link record;
	TRACK_WORDS = 8, // the least of a track record;
	VIA_WORDS = 6, // of a via record.
	PLACE_OF_WORDS = 5, // The words of place PART WAY of PART.
	WHY_LEN = 2048, // Room for a fault of a copy.
	PAD_REC_WORDS = 7, // The words of a pad record of a footprint,
	HOLE_REC_WORDS = 6, // of a hole record,
	SILK_REC_WORDS = 6, // the least of a silk record,
	COURT_REC_WORDS = 5, // and of a court record.
	BY_SPARE = 16, // Room in the words of what placed a part beyond them.
	BOX_PTS = 4, // The corners of a box.
	// The word of a part record of the lock where its place starts
	// (X Y ROT), and its side.
	LOCK_X = 3,
	LOCK_SIDE = 6,
	VIA_X = 2 // The word of a via record where its center starts.
};
#define EPS 1e-9 // Lengths closer than this are one, in mm.
#define REF_MIN 0.5 // The least size of a designator, in mm,
#define REF_MAX 0.8 // and the most.
#define MASK_GROW 0.05 // The mask's growth round a pad.
#define FAR 1e300 // Beyond any box, for a box of nothing yet.

void *pcb_alloc(size_t n)
{
	void *p = calloc(1, n ? n : 1);

	if (!p) {
		pcb_die("out of memory");
	}
	return p;
}

void *pcb_grow(void *p, size_t n, size_t size)
{
	void *q = realloc(p, (n + 1) * size);

	if (!q) {
		pcb_die("out of memory");
	}
	memset((char *)q + (n * size), 0, size);
	return q;
}

char *pcb_strdup(const char *s)
{
	size_t n = strlen(s) + 1;
	char *d = pcb_alloc(n);

	memcpy(d, s, n);
	return d;
}

void pcb_warn(const char *fmt, ...)
{
	va_list ap;

	(void)fflush(stdout);
	va_start(ap, fmt);
	(void)vfprintf(stderr, fmt, ap);
	va_end(ap);
	(void)fputc('\n', stderr);
}

void pcb_error(struct pcb *b, int line, const char *fmt, ...)
{
	va_list ap;

	(void)fflush(stdout);
	if (line) {
		(void)fprintf(stderr, "%s:%d: ", b->file, line);
	} else {
		(void)fprintf(stderr, "%s: ", b->file);
	}
	va_start(ap, fmt);
	(void)vfprintf(stderr, fmt, ap);
	va_end(ap);
	(void)fputc('\n', stderr);
	b->failed = 1;
}

// Splits a line into words at blanks, up to a #.
static int split(char *s, char **w)
{
	int n = 0;

	for (;;) {
		while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
			s++;
		}
		if (!*s || *s == '#' || n == WORDS) {
			return n;
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

char *pcb_words(char **w, int n)
{
	size_t len = 1;
	char *s = NULL;

	for (int i = 0; i < n; i++) {
		len += strlen(w[i]) + 1;
	}
	s = pcb_alloc(len);
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

static int number(const char *s, double *v)
{
	char *end = NULL;

	*v = strtod(s, &end);
	return end != s && !*end && isfinite(*v);
}

// Reads the n numbers of the words w into v, up to the first that is
// none; returns whether all are.
static int numbers(char **w, int n, double *v)
{
	for (int i = 0; i < n; i++) {
		if (!number(w[i], &v[i])) {
			return 0;
		}
	}
	return 1;
}

int pcb_layer(const struct pcb *b, const char *name)
{
	for (size_t i = 0; i < b->nlayer; i++) {
		if (!strcmp(b->layer[i].name, name)) {
			return (int)i;
		}
	}
	return -1;
}

static long part_named(const struct pcb *b, const char *name)
{
	for (size_t i = 0; i < b->npart; i++) {
		if (!strcmp(b->part[i].name, name)) {
			return (long)i;
		}
	}
	return -1;
}

static long net_named(const struct pcb *b, const char *name)
{
	for (size_t i = 0; i < b->nnet; i++) {
		if (!strcmp(b->net[i].name, name)) {
			return (long)i;
		}
	}
	return -1;
}

void pcb_add_lib(struct pcb_lib *l, const char *path)
{
	static const char fps[] = "/footprints";
	size_t room = strlen(path) + sizeof(fps);

	l->dir = (char **)pcb_grow((void *)l->dir, l->ndir, sizeof *l->dir);
	l->dir[l->ndir] = pcb_alloc(room);
	(void)snprintf(l->dir[l->ndir++], room, "%s%s", path, fps);
}

void pcb_free_lib(struct pcb_lib *l)
{
	for (size_t k = 0; k < l->ndir; k++) {
		free(l->dir[k]);
	}
	free((void *)l->dir);
	l->dir = NULL;
	l->ndir = 0;
}

// Where the footprints of a source are read from: the directories, the
// project's own next to the source first, and the library, which keeps
// those read.
struct fp_search {
	char **dir;
	size_t ndir;
	struct pcb_lib *lib;
};

static FILE *open_in(const struct fp_search *fs, const char *name)
{
	for (size_t i = 0; i < fs->ndir; i++) {
		size_t room = strlen(fs->dir[i]) + strlen(name) + 2;
		char *p = pcb_alloc(room);
		FILE *f = NULL;

		(void)snprintf(p, room, "%s/%s", fs->dir[i], name);
		f = fopen(p, "r");
		free(p);
		if (f) {
			return f;
		}
	}
	return NULL;
}

// The shape of a pad of the word w.
static int pad_shape(const char *w)
{
	if (!strcmp(w, "rect")) {
		return PAD_RECT;
	}
	return !strcmp(w, "rrect") ? PAD_RRECT : PAD_CIRCLE;
}

// Reads a record of a pad (pad NAME SHAPE X Y W H [nopaste]) or a hole
// (hole NAME X Y D DRILL [keep K]) of the n words w into p; returns 0
// if it is neither.
static int pad_record(char **w, int n, struct fp_pad *p)
{
	double v[4];

	if (!strcmp(w[0], "pad") &&
	    (n == PAD_REC_WORDS || (n == PAD_REC_WORDS + 1 &&
				    !strcmp(w[PAD_REC_WORDS], "nopaste"))) &&
	    numbers(w + 3, 4, v)) {
		p->nopaste = n == PAD_REC_WORDS + 1;
		p->shape = pad_shape(w[2]);
		p->x = v[0];
		p->y = v[1];
		p->w = v[2];
		p->h = v[3];
		return 1;
	}
	if (!strcmp(w[0], "hole") &&
	    (n == HOLE_REC_WORDS ||
	     (n == HOLE_REC_WORDS + 2 && !strcmp(w[HOLE_REC_WORDS], "keep") &&
	      number(w[HOLE_REC_WORDS + 1], &p->keep) && p->keep > 0)) &&
	    numbers(w + 2, 4, v)) {
		p->shape = PAD_CIRCLE;
		p->hole = 1;
		p->x = v[0];
		p->y = v[1];
		p->w = p->h = v[2];
		p->drill = v[3];
		return 1;
	}
	return 0;
}

// Reads a silk record (silk W X Y X Y...) of the n words w into f;
// returns 0 if it is bad.
static int silk_record(struct fp *f, char **w, int n)
{
	struct fp_line ln = { 0, NULL, 0 };
	int ok = number(w[1], &ln.w);

	ln.xy = pcb_alloc((size_t)(n - 2) * sizeof *ln.xy);
	for (int i = 2; ok && i < n; i++) {
		ok = number(w[i], &ln.xy[i - 2]);
	}
	if (!ok) {
		free(ln.xy);
		return 0;
	}
	ln.n = (size_t)(n - 2) / 2;
	f->silk = pcb_grow(f->silk, f->nsilk, sizeof *f->silk);
	f->silk[f->nsilk++] = ln;
	return 1;
}

// Reads the footprint of the name from in into f, the faults told at
// the line of b.
static void read_fp(struct pcb *b, int line, FILE *in, struct fp *f)
{
	char *buf = pcb_alloc(LINE);
	char **w = (char **)pcb_alloc(WORDS * sizeof *w);

	while (fgets(buf, LINE, in)) {
		int n = split(buf, w);
		struct fp_pad p;

		if (!n) {
			continue;
		}
		memset(&p, 0, sizeof p);
		if (pad_record(w, n, &p)) {
			p.name = pcb_strdup(w[1]);
			f->pad = pcb_grow(f->pad, f->npad, sizeof *f->pad);
			f->pad[f->npad++] = p;
		} else if (!strcmp(w[0], "silk") && n >= SILK_REC_WORDS &&
			   n % 2 == 0) {
			if (!silk_record(f, w, n)) {
				pcb_error(b, line, "%s: a bad silk line",
					  f->name);
			}
		} else if (!strcmp(w[0], "court") && n == COURT_REC_WORDS) {
			for (int i = 0; i < 4; i++) {
				if (!number(w[i + 1], &f->court[i])) {
					pcb_error(b, line,
						  "%s: a bad courtyard",
						  f->name);
				}
			}
		} else {
			pcb_error(b, line, "%s: a bad record %s", f->name,
				  w[0]);
		}
	}
	free(buf);
	free((void *)w);
}

static struct fp *footprint(struct pcb *b, const struct fp_search *fs,
			    const char *name, int line)
{
	struct pcb_lib *lib = fs->lib;
	struct fp *f = NULL;
	FILE *in = NULL;

	for (size_t i = 0; i < lib->nfp; i++) {
		if (!strcmp(lib->fp[i]->name, name)) {
			return lib->fp[i];
		}
	}
	in = open_in(fs, name);
	if (!in) {
		pcb_error(b, line, "no footprint %s", name);
		return NULL;
	}
	f = pcb_alloc(sizeof *f);
	f->name = pcb_strdup(name);
	read_fp(b, line, in, f);
	(void)fclose(in);
	lib->fp = (struct fp **)pcb_grow((void *)lib->fp, lib->nfp,
					 sizeof *lib->fp);
	lib->fp[lib->nfp++] = f;
	return f;
}

// The pins of each net as the netlist gives them: part and number,
// until the footprints say which pads they are.
struct pend_net {
	size_t *part;
	char **num;
	size_t n;
};

// The pins pending, net by net.
struct pending {
	struct pend_net *net;
};

// Sets the word *field to v, a new string, the old one freed.
static void set_word(char **field, char *v)
{
	free(*field);
	*field = v;
}

// Reads a part record of the netlist, of the n words w.
static void netlist_part(struct pcb *b, char **w, int n)
{
	struct pcb_part q;

	memset(&q, 0, sizeof q);
	q.ref = pcb_strdup(w[1]);
	q.type = pcb_strdup(w[2]);
	q.name = q.ref;
	for (int i = 3; i + 1 < n; i += 2) {
		char *v = pcb_strdup(w[i + 1]);

		if (!strcmp(w[i], "name")) {
			if (q.name != q.ref) {
				free(q.name);
			}
			q.name = v;
		} else if (!strcmp(w[i], "value")) {
			set_word(&q.value, v);
		} else if (!strcmp(w[i], "mpn")) {
			set_word(&q.mpn, v);
		} else if (!strcmp(w[i], "pins")) {
			set_word(&q.pins, v);
		} else {
			free(v);
		}
	}
	b->part = pcb_grow(b->part, b->npart, sizeof *b->part);
	b->part[b->npart++] = q;
}

// Reads the net record r of the netlist of path: the net, and its pins
// pending in pd; a pin of no part is a fault of the line of b.
static void netlist_net(struct pcb *b, const struct pcb_stmt *r,
			const char *path, struct pending *pd)
{
	char **w = r->w;
	int n = r->n;
	struct pcb_net t;
	struct pend_net *pn = NULL;
	size_t k = b->nnet;
	int first = 2;

	memset(&t, 0, sizeof t);
	t.name = pcb_strdup(w[1]);
	t.plane = -1;
	if (first < n && !strcmp(w[first], "global")) {
		t.global = 1;
		first++;
	}
	if (first + 1 < n && !strcmp(w[first], "class")) {
		t.klass = pcb_strdup(w[first + 1]);
		first += 2;
	}
	b->net = pcb_grow(b->net, k, sizeof *b->net);
	b->net[b->nnet++] = t;
	pd->net = pcb_grow(pd->net, k, sizeof *pd->net);
	pn = &pd->net[k];
	pn->part = pcb_alloc((size_t)n * sizeof *pn->part);
	pn->num = (char **)pcb_alloc((size_t)n * sizeof *pn->num);
	for (int i = first; i < n; i++) {
		char *dot = strrchr(w[i], '.');
		size_t q = 0;

		if (!dot) {
			continue;
		}
		*dot = 0;
		while (q < b->npart && strcmp(b->part[q].ref, w[i]) != 0) {
			q++;
		}
		if (q == b->npart) {
			pcb_error(b, r->line, "%s: a pin of no part, %s", path,
				  w[i]);
			continue;
		}
		pn->part[pn->n] = q;
		pn->num[pn->n++] = pcb_strdup(dot + 1);
	}
}

// Reads the netlist: the parts and their nets, the pins pending in pd.
static void read_netlist(struct pcb *b, const char *path, int line,
			 struct pending *pd)
{
	char *buf = NULL;
	char **w = NULL;
	FILE *f = fopen(path, "r");

	if (!f) {
		pcb_error(b, line, "cannot read %s", path);
		return;
	}
	buf = pcb_alloc(LINE);
	w = (char **)pcb_alloc(WORDS * sizeof *w);
	while (fgets(buf, LINE, f)) {
		struct pcb_stmt r = { w, split(buf, w), line };

		if (r.n >= 3 && !strcmp(w[0], "part")) {
			netlist_part(b, w, r.n);
		} else if (r.n >= 2 && !strcmp(w[0], "net")) {
			netlist_net(b, &r, path, pd);
		}
	}
	(void)fclose(f);
	free(buf);
	free((void *)w);
}

// Gives each net the pads of its pins: every pad of the footprint that
// bears the pin number.
// The parts of each group an unplaced statement names (by its name at a
// dot) are not on the board: no footprint, so no pads, so no copper; a
// net that had pins on them waits for them.
static void leave_unplaced(struct pcb *b, const struct pending *pd)
{
	for (size_t s = 0; s < b->nstmt; s++) {
		const struct pcb_stmt *st = &b->stmt[s];
		size_t n = st->n == 2 ? strlen(st->w[1]) : 0;
		size_t hit = 0;

		if (strcmp(st->w[0], "unplaced") != 0) {
			continue;
		}
		for (size_t i = 0; n && i < b->npart; i++) {
			if (!strncmp(b->part[i].name, st->w[1], n) &&
			    b->part[i].name[n] == '.') {
				b->part[i].fp = NULL;
				b->part[i].unplaced = 1;
				hit++;
			}
		}
		if (!hit) {
			pcb_error(b, st->line, "unplaced GROUP, of parts");
		}
	}
	// The nets and their pins pending come of the netlist together.
	for (size_t k = 0; pd->net && k < b->nnet; k++) {
		const struct pend_net *pn = &pd->net[k];

		for (size_t j = 0; j < pn->n; j++) {
			b->net[k].waits = b->net[k].waits ||
					  b->part[pn->part[j]].unplaced;
		}
	}
}

static void give_pads(struct pcb *b, struct pending *pd)
{
	for (size_t k = 0; pd->net && k < b->nnet; k++) {
		struct pcb_net *t = &b->net[k];
		struct pend_net *pn = &pd->net[k];

		for (size_t j = 0; j < pn->n; j++) {
			const struct pcb_part *q = &b->part[pn->part[j]];

			for (size_t p = 0; q->fp && p < q->fp->npad; p++) {
				if (!strcmp(q->fp->pad[p].name, pn->num[j])) {
					t->end = pcb_grow(t->end, t->nend,
							  sizeof *t->end);
					t->end[t->nend].part = pn->part[j];
					t->end[t->nend++].pad = p;
				}
			}
			free(pn->num[j]);
		}
		free((void *)pn->num);
		free(pn->part);
	}
	free(pd->net);
	pd->net = NULL;
	b->netby = (char **)pcb_alloc((b->nnet + 1) * sizeof *b->netby);
	b->netpad = (char ***)pcb_alloc((b->nnet + 1) * sizeof *b->netpad);
	b->nnetpad = pcb_alloc((b->nnet + 1) * sizeof *b->nnetpad);
}

int pcb_net_of(const struct pcb *b, size_t q, size_t k)
{
	for (size_t i = 0; i < b->nnet; i++) {
		for (size_t j = 0; j < b->net[i].nend; j++) {
			if (b->net[i].end[j].part == q &&
			    b->net[i].end[j].pad == k) {
				return (int)i;
			}
		}
	}
	return -1;
}

int pcb_pin(const struct pcb *b, const char *s, struct pcb_end *at)
{
	char name[NAME_LEN];
	char *dot = NULL;
	long q = 0;
	const char *num = NULL;

	if (strlen(s) >= sizeof name) {
		return 0;
	}
	(void)snprintf(name, sizeof name, "%s", s);
	dot = strrchr(name, '.');
	if (!dot) {
		return 0;
	}
	*dot++ = 0;
	q = part_named(b, name);
	if (q < 0 || !b->part[q].fp) {
		return 0;
	}
	num = dot;
	// A pin name, from the pins of the netlist (in-:2,out:6).
	if (b->part[q].pins) {
		const char *p = b->part[q].pins;
		size_t len = strlen(dot);
		static char buf[PIN_NAME_LEN];

		while (*p) {
			const char *c = strchr(p, ':');
			const char *e = strchr(p, ',');

			if (!e) {
				e = p + strlen(p);
			}
			if (c && (size_t)(c - p) == len &&
			    !strncmp(p, dot, len) &&
			    (size_t)(e - c - 1) < sizeof buf) {
				memcpy(buf, c + 1, (size_t)(e - c - 1));
				buf[e - c - 1] = 0;
				num = buf;
				break;
			}
			p = *e ? e + 1 : e;
		}
	}
	for (size_t k = 0; k < b->part[q].fp->npad; k++) {
		if (!strcmp(b->part[q].fp->pad[k].name, num)) {
			at->part = (size_t)q;
			at->pad = k;
			return 1;
		}
	}
	return 0;
}

// The point p turned by rot degrees, a multiple of 90.
static struct pcb_pt turn(int rot, struct pcb_pt p)
{
	switch (((rot % TURN) + TURN) % TURN) {
	case QUARTER:
		return (struct pcb_pt){ -p.y, p.x };
	case 2 * QUARTER:
		return (struct pcb_pt){ -p.x, -p.y };
	case 3 * QUARTER:
		return (struct pcb_pt){ p.y, -p.x };
	default:
		return p;
	}
}

void pcb_spot(const struct pcb *b, const struct pcb_part *q, size_t k,
	      struct pcb_spot *s)
{
	const struct fp_pad *p = &q->fp->pad[k];
	struct pcb_pt d = turn(q->rot, (struct pcb_pt){ p->x, p->y });
	int quarter = (q->rot / QUARTER) % 2;

	s->x = q->x + d.x;
	s->y = q->y + d.y;
	s->w = quarter ? p->h : p->w;
	s->h = quarter ? p->w : p->h;
	s->drill = p->drill;
	s->shape = p->shape;
	s->layer = q->bottom ? (int)b->nlayer - 1 : 0;
	if (p->hole) {
		s->layer = -1;
	}
}

int pcb_keepout(const struct pcb *b, const struct pcb_part *q, size_t k,
		struct pcb_disk *d)
{
	struct pcb_spot s;

	if (!q->fp->pad[k].hole || !(q->fp->pad[k].keep > 0)) {
		return 0;
	}
	pcb_spot(b, q, k, &s);
	d->x = s.x;
	d->y = s.y;
	d->r = q->fp->pad[k].keep / 2;
	return 1;
}

// The distance from (x, y) to the box c, 0 inside it.
static double box_dist(const double c[4], double x, double y)
{
	return hypot(fmax(fmax(c[0] - x, x - c[2]), 0),
		     fmax(fmax(c[1] - y, y - c[3]), 0));
}

static char *pad_key(const struct pcb_part *p, size_t k);

// Whether a keep-out of part h, a circle, overlaps the courtyard c of
// part o; what, for the message, goes to why.
static int keep_in(const struct pcb *b, const struct pcb_part *h,
		   const struct pcb_part *o, const double c[4], char *why,
		   size_t n)
{
	for (size_t k = 0; k < h->fp->npad; k++) {
		struct pcb_disk d = { NAN, NAN, NAN };

		if (pcb_keepout(b, h, k, &d) &&
		    box_dist(c, d.x, d.y) < d.r - EPS) {
			char *key = pad_key(h, k);

			(void)snprintf(
				why, n,
				"the courtyard of %s (%s) in the keep-out of "
				"hole %s (%s)",
				o->name, pcb_ref(o), key, pcb_ref(h));
			free(key);
			return 1;
		}
	}
	return 0;
}

int pcb_clash(const struct pcb *b, const struct pcb_part *q,
	      const struct pcb_part *o, char *why, size_t n)
{
	double c[4];
	double d[4];

	if (!q->placed || !o->placed || !q->fp || !o->fp || q == o) {
		return 0;
	}
	pcb_court(q, c);
	pcb_court(o, d);
	if (q->bottom == o->bottom && c[0] < d[2] - EPS && d[0] < c[2] - EPS &&
	    c[1] < d[3] - EPS && d[1] < c[3] - EPS) {
		(void)snprintf(why, n,
			       "the courtyards of %s (%s) and %s (%s) overlap",
			       q->name, pcb_ref(q), o->name, pcb_ref(o));
		return 1;
	}
	return keep_in(b, o, q, c, why, n) || keep_in(b, q, o, d, why, n);
}

int pcb_keeps(const struct pcb *b, const struct pcb_part *o, double x, double y,
	      double rad, char *why, size_t n)
{
	for (size_t k = 0; o->placed && o->fp && k < o->fp->npad; k++) {
		struct pcb_disk d = { NAN, NAN, NAN };

		if (pcb_keepout(b, o, k, &d) &&
		    hypot(x - d.x, y - d.y) < d.r + rad - EPS) {
			char *key = pad_key(o, k);

			(void)snprintf(why, n, "the keep-out of hole %s (%s)",
				       key, pcb_ref(o));
			free(key);
			return 1;
		}
	}
	return 0;
}

int pcb_in_keepout(const struct pcb *b, double x, double y, double rad,
		   char *why, size_t n)
{
	for (size_t i = 0; i < b->npart; i++) {
		if (pcb_keeps(b, &b->part[i], x, y, rad, why, n)) {
			return 1;
		}
	}
	return 0;
}

// The distance from (x, y) to the segment a-b; its point nearest goes
// to at.
static double seg_near(double x, double y, const double *a, const double *b,
		       double at[2])
{
	double dx = b[0] - a[0];
	double dy = b[1] - a[1];
	double l = (dx * dx) + (dy * dy);
	double t = NAN;

	t = l > 0 ? ((x - a[0]) * dx + (y - a[1]) * dy) / l : 0;
	if (t < 0) {
		t = 0;
	} else if (t > 1) {
		t = 1;
	}
	at[0] = a[0] + t * dx;
	at[1] = a[1] + t * dy;
	return hypot(x - at[0], y - at[1]);
}

// A shape: a segment of its two ends, or a box of its four corners in
// turn (filled).  The distance from p to it, 0 inside; its point
// nearest goes to at.
static double shape_near(const double *v, int n, struct pcb_pt p, double at[2])
{
	double x = p.x;
	double y = p.y;

	if (n == 4) {
		const double *far = v + 4; // The corner across from v.

		at[0] = fmin(fmax(x, fmin(v[0], far[0])), fmax(v[0], far[0]));
		at[1] = fmin(fmax(y, fmin(v[1], far[1])), fmax(v[1], far[1]));
		return hypot(x - at[0], y - at[1]);
	}
	return seg_near(x, y, v, v + 2, at);
}

// Whether segments a-b and c-d cross.
static int crosses(const double *a, const double *b, const double *c,
		   const double *d)
{
	double d1 = ((b[0] - a[0]) * (c[1] - a[1])) -
		    ((b[1] - a[1]) * (c[0] - a[0]));
	double d2 = ((b[0] - a[0]) * (d[1] - a[1])) -
		    ((b[1] - a[1]) * (d[0] - a[0]));
	double d3 = ((d[0] - c[0]) * (a[1] - c[1])) -
		    ((d[1] - c[1]) * (a[0] - c[0]));
	double d4 = ((d[0] - c[0]) * (b[1] - c[1])) -
		    ((d[1] - c[1]) * (b[0] - c[0]));

	return d1 * d2 < 0 && d3 * d4 < 0;
}

// The distance between shapes s (n corners) and t (m corners), 0 where
// they meet: the least from a corner of one to the other, as both are
// convex; the point of s nearest t goes to at.
static double shapes_dist(const double *s, int n, const double *t, int m,
			  double at[2])
{
	double best = HUGE_VAL;
	double p[2];
	size_t ns = (size_t)n;
	size_t ms = (size_t)m;

	for (size_t i = 0; i < ns; i++) {
		struct pcb_pt c = { s[2 * i], s[(2 * i) + 1] };
		double d = shape_near(t, m, c, p);

		if (d < best) {
			best = d;
			at[0] = s[2 * i];
			at[1] = s[(2 * i) + 1];
		}
	}
	for (size_t j = 0; j < ms; j++) {
		struct pcb_pt c = { t[2 * j], t[(2 * j) + 1] };
		double d = shape_near(s, n, c, p);

		if (d < best) {
			best = d;
			at[0] = p[0];
			at[1] = p[1];
		}
	}
	for (size_t i = 0; i < (ns == 4 ? 4 : 1); i++) {
		for (size_t j = 0; j < (ms == 4 ? 4 : 1); j++) {
			if (crosses(s + (2 * i), s + (2 * ((i + 1) % ns)),
				    t + (2 * j), t + (2 * ((j + 1) % ms)))) {
				return 0;
			}
		}
	}
	return best;
}

double pcb_edge_dist(const struct pcb *b, int box, const double s[4], double r,
		     double at[2])
{
	double v[2 * BOX_PTS];
	double p[2];
	double here[2];
	double best = HUGE_VAL;
	int n = box ? BOX_PTS : 2;

	at = at ? at : here;
	if (box) {
		const double c[2 * BOX_PTS] = { s[0], s[1], s[2], s[1],
						s[2], s[3], s[0], s[3] };

		memcpy(v, c, sizeof c);
	} else {
		memcpy(v, s, 4 * sizeof *v);
	}
	// The outline, a rectangle round the copper: from its side nearest a
	// corner or end.
	for (size_t i = 0; i < (size_t)n; i++) {
		double x = v[2 * i];
		double y = v[(2 * i) + 1];
		double d = fmin(fmin(x - b->board[0], b->board[2] - x),
				fmin(y - b->board[1], b->board[3] - y)) -
			   r;

		if (d < best) {
			best = d;
			at[0] = x;
			at[1] = y;
		}
	}
	// A cutout: the box of its corners' centers, grown by their radius.
	for (size_t k = 0; k < b->ncut; k++) {
		const double *c = b->cut[k];
		double cr = b->cutr[k];
		double d = NAN;
		const double in[2 * BOX_PTS] = { c[0] + cr, c[1] + cr,
						 c[2] - cr, c[1] + cr,
						 c[2] - cr, c[3] - cr,
						 c[0] + cr, c[3] - cr };

		d = shapes_dist(v, n, in, 4, p) - r - cr;
		if (d < best) {
			best = d;
			at[0] = p[0];
			at[1] = p[1];
		}
	}
	return best;
}

void pcb_court(const struct pcb_part *q, double c[4])
{
	const double *k = q->fp->court;
	struct pcb_pt a = turn(q->rot, (struct pcb_pt){ k[0], k[1] });
	struct pcb_pt e = turn(q->rot, (struct pcb_pt){ k[2], k[3] });

	c[0] = q->x + fmin(a.x, e.x);
	c[1] = q->y + fmin(a.y, e.y);
	c[2] = q->x + fmax(a.x, e.x);
	c[3] = q->y + fmax(a.y, e.y);
}

char *pcb_net_words(const struct pcb *b, size_t k)
{
	char *s = pcb_strdup("");

	for (size_t i = 0; i < b->nstmt; i++) {
		const struct pcb_stmt *st = &b->stmt[i];
		int about = 0;

		if ((!strcmp(st->w[0], "path") || !strcmp(st->w[0], "fork")) &&
		    st->n >= 2) {
			about = !strcmp(st->w[1], b->net[k].name);
		} else if (!strcmp(st->w[0], "via") && st->n == 3) {
			struct pcb_end at = { 0, 0 };

			about = pcb_pin(b, st->w[1], &at) &&
				pcb_net_of(b, at.part, at.pad) == (int)k;
		}
		if (about) {
			char *w = pcb_words(st->w, st->n);
			size_t room = strlen(s) + strlen(w) + 4;
			char *t = pcb_alloc(room);

			(void)snprintf(t, room, "%s%s%s", s, *s ? " ; " : "",
				       w);
			free(s);
			free(w);
			s = t;
		}
	}
	return s;
}

// The key of pad k of part p, as pcb_pad_key gives it.
static char *pad_key(const struct pcb_part *p, size_t k)
{
	size_t room = strlen(p->name) + strlen(p->fp->pad[k].name) + KEY_SPARE;
	char *s = pcb_alloc(room);
	size_t same = 0;
	size_t nth = 0;

	// A pin of several pads: each by its place among them, j.2/1.
	for (size_t i = 0; i < p->fp->npad; i++) {
		if (!strcmp(p->fp->pad[i].name, p->fp->pad[k].name)) {
			same++;
			nth += i <= k;
		}
	}
	if (same > 1) {
		(void)snprintf(s, room, "%s.%s/%zu", p->name,
			       p->fp->pad[k].name, nth);
	} else {
		(void)snprintf(s, room, "%s.%s", p->name, p->fp->pad[k].name);
	}
	return s;
}

char *pcb_pad_key(const struct pcb *b, size_t q, size_t k)
{
	return pad_key(&b->part[q], k);
}

// The key (PART.PAD) of the pad of pin s (PART.PIN), or NULL; a new
// string.
static char *pin_key(const struct pcb *b, const char *s)
{
	struct pcb_end at = { 0, 0 };

	return pcb_pin(b, s, &at) ? pcb_pad_key(b, at.part, at.pad) : NULL;
}

// Whether the keys of a statement's two pins, k1 and k2, are a and bb,
// either way.
static int keys_are(const char *const k[2], const char *a, const char *bb)
{
	return k[0] && k[1] &&
	       ((!strcmp(k[0], a) && !strcmp(k[1], bb)) ||
		(!strcmp(k[0], bb) && !strcmp(k[1], a)));
}

double pcb_link_width(const struct pcb *b, int n, const char *a, const char *bb)
{
	for (size_t i = 0; i < b->nstmt; i++) {
		const struct pcb_stmt *st = &b->stmt[i];
		char *k[2] = { NULL, NULL };
		int hit = 0;

		if (strcmp(st->w[0], "width") != 0 || st->n != 4) {
			continue;
		}
		k[0] = pin_key(b, st->w[1]);
		k[1] = pin_key(b, st->w[2]);
		hit = keys_are((const char *const *)k, a, bb);
		free(k[0]);
		free(k[1]);
		if (hit) {
			return strtod(st->w[3], NULL);
		}
	}
	return b->net[n].width;
}

double pcb_neck_width(const struct pcb *b, const struct pcb_part *q, int n)
{
	double w = b->net[n].width;

	return pcb_small_two_pin(q) ? fmin(w, b->track) : w;
}

int pcb_add_link(struct pcb *b, int n, const char *a, const char *bb, double w)
{
	struct pcb_link l;

	l.net = n;
	l.a = pcb_strdup(a);
	l.b = pcb_strdup(bb);
	l.w = w;
	b->link = pcb_grow(b->link, b->nlink, sizeof *b->link);
	b->link[b->nlink] = l;
	return (int)b->nlink++;
}

const char *pcb_ref(const struct pcb_part *q)
{
	return q->ref && *q->ref ? q->ref : q->name;
}

char **pcb_net_pads(const struct pcb *b, size_t k, size_t *n)
{
	char **p = (char **)pcb_alloc((b->net[k].nend + 1) * sizeof *p);

	*n = 0;
	for (size_t j = 0; j < b->net[k].nend; j++) {
		const struct pcb_part *q = &b->part[b->net[k].end[j].part];
		size_t pad = b->net[k].end[j].pad;
		struct pcb_spot s;
		char *key = NULL;
		const char *side = "bottom";
		int len = 0;

		if (!q->placed) {
			continue;
		}
		pcb_spot(b, q, pad, &s);
		key = pcb_pad_key(b, b->net[k].end[j].part, pad);
		if (s.layer < 0) {
			side = "all";
		} else if (s.layer == 0) {
			side = "top";
		}
		len = snprintf(NULL, 0, "%s %.4f %.4f %.4f %.4f %s", key, s.x,
			       s.y, s.w, s.h, side);
		if (len < 0) {
			pcb_die("%s: a pad that cannot be told", key);
		}
		p[*n] = pcb_alloc((size_t)len + 1);
		(void)snprintf(p[(*n)++], (size_t)len + 1,
			       "%s %.4f %.4f %.4f %.4f %s", key, s.x, s.y, s.w,
			       s.h, side);
		free(key);
	}
	return p;
}

double pcb_spot_dist(const struct pcb_spot *s, double x, double y)
{
	if (s->shape == PAD_CIRCLE) {
		return fmax(hypot(x - s->x, y - s->y) - (s->w / 2), 0);
	}
	return hypot(fmax(fabs(x - s->x) - (s->w / 2), 0),
		     fmax(fabs(y - s->y) - (s->h / 2), 0));
}

int pcb_pad_ends(const struct pcb_spot *s, double p[2][2])
{
	double lo = fmin(s->w, s->h);
	double hi = fmax(s->w, s->h);
	double u = (hi - lo) / 2;

	if (s->shape == PAD_CIRCLE || hi < 2 * lo) {
		p[0][0] = s->x;
		p[0][1] = s->y;
		return 1;
	}
	p[0][0] = s->x - (s->w > s->h ? u : 0);
	p[0][1] = s->y - (s->w > s->h ? 0 : u);
	p[1][0] = s->x + (s->w > s->h ? u : 0);
	p[1][1] = s->y + (s->w > s->h ? 0 : u);
	return 2;
}

int pcb_soldered(const struct pcb_part *q, size_t k)
{
	return q->fp->pad[k].hole || !q->fp->pad[k].nopaste;
}

int pcb_small_two_pin(const struct pcb_part *q)
{
	if (!q->fp || q->fp->npad != 2) {
		return 0;
	}
	for (size_t k = 0; k < 2; k++) {
		const struct fp_pad *p = &q->fp->pad[k];

		if (p->hole || p->w * p->h > 1.0 + EPS) {
			return 0;
		}
	}
	return 1;
}

int pcb_relieved(const struct pcb *b, size_t q, size_t k)
{
	struct pcb_spot s;
	int n = pcb_net_of(b, q, k);

	if (n < 0 || b->net[n].plane < 0 || !pcb_small_two_pin(&b->part[q])) {
		return 0;
	}
	pcb_spot(b, &b->part[q], k, &s);
	return s.layer == b->net[n].plane;
}

double pcb_seg_dist(struct pcb_pt p, struct pcb_seg s)
{
	double x = p.x;
	double y = p.y;
	double ax = s.a.x;
	double ay = s.a.y;
	double bx = s.b.x;
	double by = s.b.y;
	double dx = bx - ax;
	double dy = by - ay;
	double l = (dx * dx) + (dy * dy);
	double t = NAN;

	t = l > 0 ? ((x - ax) * dx + (y - ay) * dy) / l : 0;
	if (t < 0) {
		t = 0;
	} else if (t > 1) {
		t = 1;
	}
	return hypot(x - (ax + t * dx), y - (ay + t * dy));
}

// The words of a ref statement: whether they are as pcb(5) says.
static int ref_words(char **w, int n)
{
	int k = 3;

	if (n < 3 ||
	    (strcmp(w[2], "above") != 0 && strcmp(w[2], "below") != 0 &&
	     strcmp(w[2], "left") != 0 && strcmp(w[2], "right") != 0)) {
		return 0;
	}
	if (k < n && (!strcmp(w[k], "start") || !strcmp(w[k], "middle") ||
		      !strcmp(w[k], "end"))) {
		k++;
	}
	if (k + 1 < n && !strcmp(w[k], "rot") && !strcmp(w[k + 1], "90")) {
		k += 2;
	}
	if (k + 1 < n && !strcmp(w[k], "size")) {
		char *e = NULL;
		double h = strtod(w[k + 1], &e);

		if (*e || h < REF_MIN || h > REF_MAX) {
			return 0;
		}
		k += 2;
	}
	return k == n;
}

// A place of a text on the silkscreen: the lower left of its box, the
// box's size, the capital height of its text, and whether it is turned
// to read upward.
struct place {
	double x, y, w, h, size;
	int rot;
};

static char *free_text(const struct pcb_stmt *st, struct place *at,
		       int *bottom);

// Whether a text statement is as pcb(5) says.
static int text_ok(const struct pcb_stmt *st)
{
	struct place at;
	int bottom = 0;
	char *t = free_text(st, &at, &bottom);

	free(t);
	return t != NULL;
}

// Reads one statement of the source that the reader itself carries out:
// the netlist, the board, layers, rules, footprints and widths.  The
// rest are kept for the programs.
// The statement board rect X0 Y0 X1 Y1.
static void st_board(struct pcb *b, const struct pcb_stmt *st)
{
	for (int i = 0; i < 4; i++) {
		if (!number(st->w[i + 2], &b->board[i])) {
			pcb_error(b, st->line, "board rect X0 Y0 X1 Y1");
		}
	}
}

// The statement cutout rect X0 Y0 X1 Y1 [round R].
static void st_cutout(struct pcb *b, const struct pcb_stmt *st)
{
	char **w = st->w;
	size_t k = b->ncut < PCB_CUTS ? b->ncut : PCB_CUTS - 1;
	double *c = b->cut[k];

	b->cutr[k] = 0;
	for (int i = 0; i < 4; i++) {
		if (!number(w[i + 2], &c[i])) {
			pcb_error(b, st->line,
				  "cutout rect X0 Y0 X1 Y1 [round R]");
		}
	}
	if (st->n == ROUND_WORDS &&
	    (strcmp(w[RECT_WORDS], "round") != 0 ||
	     !number(w[RECT_WORDS + 1], &b->cutr[k]) || b->cutr[k] < 0 ||
	     2 * b->cutr[k] > fmin(c[2] - c[0], c[3] - c[1]))) {
		pcb_error(b, st->line, "cutout rect X0 Y0 X1 Y1 [round R]");
	}
	if (b->ncut == PCB_CUTS) {
		pcb_error(b, st->line, "more than 8 cutouts");
	}
	b->ncut += b->ncut < PCB_CUTS;
}

// The statement layer NAME signal|plane [NET].
static void st_layer(struct pcb *b, const struct pcb_stmt *st)
{
	char **w = st->w;
	struct pcb_layer l;

	l.name = pcb_strdup(w[1]);
	l.kind = !strcmp(w[2], "plane") ? LAYER_PLANE : LAYER_SIGNAL;
	l.net = st->n > 3 ? pcb_strdup(w[3]) : NULL;
	if (strcmp(w[2], "plane") != 0 && strcmp(w[2], "signal") != 0) {
		pcb_error(b, st->line, "a layer is signal or plane NET");
	}
	b->layer = pcb_grow(b->layer, b->nlayer, sizeof *b->layer);
	b->layer[b->nlayer++] = l;
}

// The statement rule edge D [except NET]...: the nets the edge rule
// leaves alone.
static void rule_except(struct pcb *b, const struct pcb_stmt *st)
{
	for (int i = 3; i < st->n; i++) {
		if (!strcmp(st->w[i], "except")) {
			continue;
		}
		b->edge_except = (char **)pcb_grow((void *)b->edge_except,
						   b->nexcept,
						   sizeof *b->edge_except);
		b->edge_except[b->nexcept++] = pcb_strdup(st->w[i]);
	}
}

// The statement rule NAME V [V].
static void st_rule(struct pcb *b, const struct pcb_stmt *st)
{
	char **w = st->w;
	int n = st->n;
	double v[2] = { NAN, NAN };
	int ok = 1;

	for (int i = 2; i < n && i < 4; i++) {
		ok = number(w[i], &v[i - 2]) && ok;
	}
	if (!ok && strcmp(w[1], "edge") != 0) {
		pcb_error(b, st->line, "a rule takes numbers");
	} else if (!strcmp(w[1], "clearance")) {
		b->clearance = v[0];
	} else if (!strcmp(w[1], "track")) {
		b->track = v[0];
	} else if (!strcmp(w[1], "via") && n == 4) {
		b->via_dia = v[0];
		b->via_drill = v[1];
	} else if (!strcmp(w[1], "viagap") && n == 3) {
		b->viagap = v[0];
	} else if (!strcmp(w[1], "maskweb") && n == 3) {
		b->maskweb = v[0];
	} else if (!strcmp(w[1], "edge") && number(w[2], &b->edge)) {
		rule_except(b, st);
	} else {
		pcb_error(b, st->line,
			  "rule clearance|track|via|viagap|maskweb|edge ...");
	}
}

// The statement class CLASS width W: the width of the tracks of the
// nets of a class.
static void st_class(struct pcb *b, const struct pcb_stmt *st)
{
	char **w = st->w;
	size_t k = b->nclass;
	double v = NAN;

	if (st->n != 4 || strcmp(w[2], "width") != 0 || !number(w[3], &v) ||
	    v <= 0) {
		pcb_error(b, st->line, "class CLASS width W");
		return;
	}
	for (size_t i = 0; i < k; i++) {
		if (!strcmp(b->klass[i], w[1])) {
			pcb_error(b, st->line, "class %s again", w[1]);
			return;
		}
	}
	b->klass = (char **)pcb_grow((void *)b->klass, k, sizeof *b->klass);
	b->class_width = pcb_grow(b->class_width, k, sizeof *b->class_width);
	b->klass[k] = pcb_strdup(w[1]);
	b->class_width[k] = v;
	b->nclass++;
}

// The statement netlist FILE, the file next to the source unless it is
// absolute.
static void st_netlist(struct pcb *b, const struct pcb_stmt *st,
		       struct pending *pd)
{
	const char *name = st->w[1];
	size_t room = strlen(b->file) + strlen(name) + 2;
	char *path = pcb_alloc(room);
	const char *slash = strrchr(b->file, '/');

	if (name[0] != '/' && slash) {
		(void)snprintf(path, room, "%.*s/%s", (int)(slash - b->file),
			       b->file, name);
	} else {
		(void)snprintf(path, room, "%s", name);
	}
	read_netlist(b, path, st->line, pd);
	free(path);
}

// The statement footprint TYPE|MPN NAME: the footprint of the parts of
// the type or part number.
static void st_footprint(struct pcb *b, const struct pcb_stmt *st,
			 const struct fp_search *fs)
{
	char **w = st->w;
	struct fp *f = footprint(b, fs, w[2], st->line);
	int used = 0;

	for (size_t i = 0; i < b->npart; i++) {
		struct pcb_part *q = &b->part[i];

		if ((q->mpn && !strcmp(q->mpn, w[1])) ||
		    (!q->mpn && !strcmp(q->type, w[1]))) {
			q->fp = f;
			used = 1;
		}
	}
	if (!used) {
		pcb_error(b, st->line, "no part is a %s", w[1]);
	}
}

// Keeps a statement for the programs: a copy of its words.
static void keep_stmt(struct pcb *b, const struct pcb_stmt *st)
{
	struct pcb_stmt k = { NULL, st->n, st->line };

	k.w = (char **)pcb_alloc((size_t)st->n * sizeof *k.w);
	for (int i = 0; i < st->n; i++) {
		k.w[i] = pcb_strdup(st->w[i]);
	}
	b->stmt = pcb_grow(b->stmt, b->nstmt, sizeof *b->stmt);
	b->stmt[b->nstmt++] = k;
}

// Reads one statement of the source that the reader itself carries out:
// the netlist, the board, layers, rules, footprints and widths.  The
// rest are kept for the programs, the footprints read from fs and the
// pins of the netlist pending in pd.
static void statement(struct pcb *b, const struct pcb_stmt *st,
		      const struct fp_search *fs, struct pending *pd)
{
	char **w = st->w;
	int n = st->n;

	if (!strcmp(w[0], "board") && n == RECT_WORDS &&
	    !strcmp(w[1], "rect")) {
		st_board(b, st);
	} else if (!strcmp(w[0], "cutout") &&
		   (n == RECT_WORDS || n == ROUND_WORDS) &&
		   !strcmp(w[1], "rect")) {
		st_cutout(b, st);
	} else if (!strcmp(w[0], "layer") && n >= 3) {
		st_layer(b, st);
	} else if (!strcmp(w[0], "rule") && n >= 3) {
		st_rule(b, st);
	} else if (!strcmp(w[0], "class")) {
		st_class(b, st);
	} else if (!strcmp(w[0], "netlist") && n == 2) {
		st_netlist(b, st, pd);
	} else if (!strcmp(w[0], "footprint") && n == 3) {
		st_footprint(b, st, fs);
	} else if (!strcmp(w[0], "note")) {
		return;
	} else if (!strcmp(w[0], "text") && !text_ok(st)) {
		pcb_error(b, st->line, "text TEXT at X Y [size H] [bottom]");
	} else if (!strcmp(w[0], "ref") && !ref_words(w, n)) {
		pcb_error(b, st->line,
			  "ref PART above|below|left|right "
			  "[start|middle|end] [rot 90] [size 0.5 to 0.8]");
	} else {
		keep_stmt(b, st);
	}
}

// The record part NAME REF X Y ROT SIDE [BY...] of the lock: where the
// part was placed.
static void lock_part(struct pcb *b, const struct pcb_stmt *r)
{
	char **w = r->w;
	int n = r->n;
	double v[3] = { NAN, NAN, NAN };
	long q = 0;

	// The designator (w[2]) is for the reader; the name keys.
	if (!numbers(w + LOCK_X, 3, v)) {
		return;
	}
	q = part_named(b, w[1]);
	if (q < 0 || !b->part[q].fp) {
		return; // Stale: told by pcbcheck.
	}
	b->part[q].x = v[0];
	b->part[q].y = v[1];
	b->part[q].rot = (int)v[2];
	b->part[q].bottom = !strcmp(w[LOCK_SIDE], "bottom");
	b->part[q].placed = 1;
	b->part[q].by = n > PART_WORDS ?
				pcb_words(w + PART_WORDS, n - PART_WORDS) :
				pcb_strdup("");
}

// The record pad NET PART.PIN REF X Y W H LAYER of the lock, the REF
// left out.
static void lock_pad(struct pcb *b, const struct pcb_stmt *r)
{
	char **w = r->w;
	long k = net_named(b, w[1]);

	if (k >= 0) {
		char *t[PAD_WORDS];

		// The pad, then its center, size and layer.
		t[0] = w[2];
		for (int i = 1; i < PAD_WORDS; i++) {
			t[i] = w[3 + i];
		}

		b->netpad[k] = (char **)pcb_grow((void *)b->netpad[k],
						 b->nnetpad[k],
						 sizeof *b->netpad[k]);
		b->netpad[k][b->nnetpad[k]++] = pcb_words(t, PAD_WORDS);
	}
}

// The record track NET LAYER W X Y X Y... of the lock, of the connection
// link (or -1).
static void lock_track(struct pcb *b, const struct pcb_stmt *r, int link)
{
	char **w = r->w;
	struct pcb_track t;
	long k = net_named(b, w[1]);
	double v = NAN;

	if (k < 0 || pcb_layer(b, w[2]) < 0 || !number(w[3], &v)) {
		return;
	}
	t.net = (int)k;
	t.link = link >= 0 && b->link[link].net == k ? link : -1;
	t.layer = pcb_layer(b, w[2]);
	t.w = v;
	t.n = (size_t)(r->n - 4) / 2;
	t.xy = pcb_alloc(2 * t.n * sizeof *t.xy);
	for (size_t i = 0; i < 2 * t.n; i++) {
		t.xy[i] = strtod(w[4 + i], NULL);
	}
	b->track_ = pcb_grow(b->track_, b->ntrack, sizeof *b->track_);
	b->track_[b->ntrack++] = t;
}

// The record via NET X Y DIA DRILL of the lock, of the connection link
// (or -1).
static void lock_via(struct pcb *b, const struct pcb_stmt *r, int link)
{
	char **w = r->w;
	struct pcb_via vi;
	long k = net_named(b, w[1]);

	if (k < 0) {
		return;
	}
	vi.net = (int)k;
	vi.link = link >= 0 && b->link[link].net == k ? link : -1;
	vi.x = strtod(w[VIA_X], NULL);
	vi.y = strtod(w[VIA_X + 1], NULL);
	vi.dia = strtod(w[VIA_X + 2], NULL);
	vi.drill = strtod(w[VIA_X + 3], NULL);
	b->via = pcb_grow(b->via, b->nvia, sizeof *b->via);
	b->via[b->nvia++] = vi;
}

// Reads the record r of the lock; returns the connection the copper
// that follows is of, link before it.
static int lock_record(struct pcb *b, const struct pcb_stmt *r, int link)
{
	char **w = r->w;
	int n = r->n;
	double v = NAN;

	if (!strcmp(w[0], "part") && n >= PART_WORDS) {
		lock_part(b, r);
	} else if (!strcmp(w[0], "net") && n >= 3 && !strcmp(w[2], "by")) {
		long k = net_named(b, w[1]);

		if (k >= 0) {
			b->netby[k] = pcb_words(w + 3, n - 3);
		}
		return -1;
	} else if (!strcmp(w[0], "link") && n == LINK_WORDS &&
		   number(w[4], &v)) {
		long k = net_named(b, w[1]);

		return k >= 0 ? pcb_add_link(b, (int)k, w[2], w[3], v) : -1;
	} else if (!strcmp(w[0], "pad") && n == PAD_WORDS + 3) {
		lock_pad(b, r);
	} else if (!strcmp(w[0], "track") && n >= TRACK_WORDS &&
		   (n - 4) % 2 == 0) {
		lock_track(b, r, link);
	} else if (!strcmp(w[0], "via") && n == VIA_WORDS) {
		lock_via(b, r, link);
	}
	return link;
}

// Reads the lock: parts where they were placed, and copper.
static void read_lock(struct pcb *b, FILE *f)
{
	char *buf = pcb_alloc(LINE);
	char **w = (char **)pcb_alloc(WORDS * sizeof *w);
	int ln = 0;
	int link = -1; // The connection the copper read is of.

	while (fgets(buf, LINE, f)) {
		struct pcb_stmt r = { w, split(buf, w), 0 };

		ln++;
		r.line = ln;
		if (r.n) {
			link = lock_record(b, &r, link);
		}
	}
	free(buf);
	free((void *)w);
}

// The store of the section of a name, made if new.
static size_t store_of(struct pcb *b, const char *name)
{
	for (size_t i = 0; i < b->nsec; i++) {
		if (!strcmp(b->sec_name[i], name)) {
			return i;
		}
	}
	b->sec_name = (char **)pcb_grow((void *)b->sec_name, b->nsec,
					sizeof *b->sec_name);
	b->sec_text = (char **)pcb_grow((void *)b->sec_text, b->nsec,
					sizeof *b->sec_text);
	b->sec_name[b->nsec] = pcb_strdup(name);
	b->sec_text[b->nsec] = pcb_strdup("");
	return b->nsec++;
}

// Splits the lock into its sections: the group's (the board's, "",
// for none) goes to the stream returned, the rest to the stores.
static FILE *sections(struct pcb *b, FILE *lock, const char *group)
{
	static const char head[] = "# lock of ";
	char *line = pcb_alloc(LINE);
	char name[NAME_LEN] = "";
	FILE *mine = tmpfile();

	if (!mine) {
		pcb_die("cannot write a temporary file");
	}
	while (fgets(line, LINE, lock)) {
		size_t k = 0;
		size_t room = 0;
		char *t = NULL;

		if (sscanf(line, "group %1023s", name) == 1 ||
		    !strncmp(line, head, sizeof(head) - 1)) {
			continue; // A section begins; the header is written anew.
		}
		if (!strcmp(name, group ? group : "")) {
			(void)fputs(line, mine);
			continue;
		}
		k = store_of(b, name);
		room = strlen(b->sec_text[k]) + strlen(line) + 1;
		t = pcb_alloc(room);
		(void)snprintf(t, room, "%s%s", b->sec_text[k], line);
		free(b->sec_text[k]);
		b->sec_text[k] = t;
	}
	free(line);
	if (!stream_rewind(mine)) {
		pcb_die("cannot write a temporary file");
	}
	return mine;
}

// The depth of a group: its dots.
static size_t depth(const char *g)
{
	size_t n = 0;

	for (; *g; g++) {
		n += *g == '.';
	}
	return n;
}

// Whether a name is of a group: GROUP.REST.
static int of_group(const char *name, const char *group)
{
	size_t n = strlen(group);

	return !strncmp(name, group, n) && name[n] == '.';
}

// A name of the source in the copy: the copy's group for the source's,
// GROUP.REST of the copy for GROUP.REST of the source; any other name
// as it is.
static char *moved(const struct pcb_copy *cp, const char *w)
{
	char *s = NULL;
	size_t room = 0;

	if (strcmp(w, cp->src) != 0 && !of_group(w, cp->src)) {
		return pcb_strdup(w);
	}
	room = strlen(cp->group) + strlen(w) + 1;
	s = pcb_alloc(room);
	(void)snprintf(s, room, "%s%s", cp->group, w + strlen(cp->src));
	return s;
}

// A word of the source in the copy: a net of the source, the copy's
// net; else as moved.
static char *in_copy(const struct pcb *b, const struct pcb_copy *cp,
		     const char *w)
{
	long k = net_named(b, w);

	if (k >= 0 && cp->net[k] >= 0) {
		return pcb_strdup(b->net[cp->net[k]].name);
	}
	return moved(cp, w);
}

// The name of the net of pad p of part q, "-" for none.
static const char *net_word(const struct pcb *b, size_t q, size_t p)
{
	int n = pcb_net_of(b, q, p);

	return n >= 0 ? b->net[n].name : "-";
}

// A check of the groups of a copy cp of board b: the map back from the
// nets of the copy to the source's, and the fault found ("" for none).
struct alike {
	struct pcb *b;
	struct pcb_copy *cp;
	long *back;
	char why[WHY_LEN];
};

// The fault of the groups of the copy themselves, if any, to why: one
// holds the other, the source is or holds a copy, is no block, or is
// shallower.
static void groups_fault(struct alike *al)
{
	const struct pcb *b = al->b;
	const char *c = al->cp->group;
	const char *s = al->cp->src;
	int block = 0;
	int held = 0;

	for (size_t i = 0; i < b->nstmt; i++) {
		block = block ||
			(!strcmp(b->stmt[i].w[0], "block") &&
			 b->stmt[i].n == 2 && !strcmp(b->stmt[i].w[1], s));
	}
	for (size_t i = 0; i < b->ncopy; i++) {
		held = held || !strcmp(b->copy[i].group, s) ||
		       of_group(b->copy[i].group, s);
	}
	if (!strcmp(c, s) || of_group(c, s) || of_group(s, c)) {
		(void)snprintf(al->why, sizeof al->why,
			       "the one holds the other");
	} else if (held) {
		(void)snprintf(al->why, sizeof al->why,
			       "%s is or holds a copy; copy a block that is "
			       "none",
			       s);
	} else if (!block) {
		(void)snprintf(al->why, sizeof al->why, "%s is no block", s);
	} else if (depth(s) < depth(c)) {
		(void)snprintf(al->why, sizeof al->why,
			       "%s is shallower than %s, so laid out after it",
			       s, c);
	}
}

// Maps the nets of the pads of part tw[0] of the source to those of its
// twin tw[1] of the copy, or tells to why where they differ.
static void twin_nets(struct alike *al, const size_t tw[2])
{
	const struct pcb *b = al->b;
	long *net = al->cp->net;
	const struct fp *f = b->part[tw[0]].fp;

	for (size_t p = 0; !*al->why && f && p < f->npad; p++) {
		int ns = pcb_net_of(b, tw[0], p);
		int nc = pcb_net_of(b, tw[1], p);

		if ((ns < 0) != (nc < 0) ||
		    (ns >= 0 && ((net[ns] >= 0 && net[ns] != nc) ||
				 (al->back[nc] >= 0 && al->back[nc] != ns)))) {
			char *ks = pcb_pad_key(b, tw[0], p);
			char *kc = pcb_pad_key(b, tw[1], p);

			(void)snprintf(al->why, sizeof al->why,
				       "not of the same nets (%s on %s, %s "
				       "on %s)",
				       ks, net_word(b, tw[0], p), kc,
				       net_word(b, tw[1], p));
			free(ks);
			free(kc);
		} else if (ns >= 0) {
			net[ns] = nc;
			al->back[nc] = ns;
		}
	}
}

// Checks part i, of either group of the copy, against its twin in the
// other: there, of the same footprint, and of the source, its pads on
// nets that go with the twin's.
static void part_twin(struct alike *al, size_t i)
{
	const struct pcb *b = al->b;
	const char *c = al->cp->group;
	const char *s = al->cp->src;
	const char *name = b->part[i].name;
	int mine = of_group(name, c);
	size_t room = strlen(name) + strlen(c) + strlen(s);
	char *o = NULL;
	long j = 0;

	if (!mine && !of_group(name, s)) {
		return;
	}
	o = pcb_alloc(room);
	(void)snprintf(o, room, "%s%s", mine ? s : c,
		       name + strlen(mine ? c : s));
	j = part_named(b, o);
	if (j < 0 || b->part[j].fp != b->part[i].fp) {
		(void)snprintf(al->why, sizeof al->why,
			       "not of the same parts (%s %s %s)", name,
			       j < 0 ? "and no" : "of another footprint than",
			       o);
	}
	free(o);
	if (!*al->why && !mine) {
		const size_t tw[2] = { i, (size_t)j };

		twin_nets(al, tw);
	}
}

// Whether the two groups of a copy are of the same parts (by name past
// the group, of one footprint) and nets, pad for pad, making the map of
// the nets; the fault, if any, told if tell is set.
static int copy_alike(struct pcb *b, struct pcb_copy *cp, int tell)
{
	struct alike al;

	al.b = b;
	al.cp = cp;
	al.back = pcb_alloc((b->nnet + 1) * sizeof *al.back);
	al.why[0] = 0;
	free(cp->net);
	cp->net = pcb_alloc((b->nnet + 1) * sizeof *cp->net);
	for (size_t k = 0; k < b->nnet; k++) {
		cp->net[k] = al.back[k] = -1;
	}
	groups_fault(&al);
	for (size_t i = 0; !*al.why && i < b->npart; i++) {
		part_twin(&al, i);
	}
	if (*al.why && tell) {
		pcb_error(b, cp->line, "block %s as %s: %s", cp->group, cp->src,
			  al.why);
	}
	free(al.back);
	return !*al.why;
}

// Whether statement st of the source is the copy's too, renamed: a
// block statement of a group in the source; a place statement of a
// part or group of the source against one of it; a ref, width, path,
// fork or via statement naming a part, pin or net of the source (a net
// the copy has another of) and no part or pin outside it.
static int copied(const struct pcb *b, const struct pcb_copy *cp,
		  const struct pcb_stmt *st)
{
	const char *k = st->w[0];
	int in = 0;

	if (!strcmp(k, "block")) {
		return st->n == 2 && of_group(st->w[1], cp->src);
	}
	if (!strcmp(k, "place")) {
		const char *to = st->n >= PLACE_OF_WORDS ? st->w[4] : "";

		if (st->n < 4 || !of_group(st->w[1], cp->src)) {
			return 0;
		}
		if (!strcmp(st->w[2], "near")) {
			to = st->w[3];
		}
		return of_group(to, cp->src);
	}
	if (strcmp(k, "ref") != 0 && strcmp(k, "width") != 0 &&
	    strcmp(k, "path") != 0 && strcmp(k, "fork") != 0 &&
	    strcmp(k, "via") != 0) {
		return 0;
	}
	for (int i = 1; i < st->n; i++) {
		long n = net_named(b, st->w[i]);
		struct pcb_end at = { 0, 0 };

		if (of_group(st->w[i], cp->src) ||
		    (n >= 0 && cp->net[n] >= 0 && cp->net[n] != n)) {
			in = 1;
		} else if (part_named(b, st->w[i]) >= 0 ||
			   pcb_pin(b, st->w[i], &at)) {
			return 0;
		}
	}
	return in;
}

// Whether two statements place the same thing (place or fix), or its
// designator (ref), or make the same block.
static int same_place(const struct pcb_stmt *a, const struct pcb_stmt *o)
{
	int pa = !strcmp(a->w[0], "place") || !strcmp(a->w[0], "fix");
	int po = !strcmp(o->w[0], "place") || !strcmp(o->w[0], "fix");

	int same = 0;

	if (pa) {
		same = po;
	} else {
		same = (!strcmp(a->w[0], "ref") || !strcmp(a->w[0], "block")) &&
		       !strcmp(a->w[0], o->w[0]);
	}
	return a->n >= 2 && o->n >= 2 && !strcmp(a->w[1], o->w[1]) && same;
}

// Reads the copies (block GROUP as GROUP), each its statement made
// block GROUP; then gives each copy of the same parts and nets as its
// source the source's statements, renamed, on the copy's line, each in
// place of the copy's own statement placing the same (a fault).
static void read_copies(struct pcb *b)
{
	size_t n0 = b->nstmt;

	for (size_t i = 0; i < n0; i++) {
		struct pcb_stmt *st = &b->stmt[i];
		struct pcb_copy cp;

		if (strcmp(st->w[0], "block") != 0 || st->n != 4 ||
		    strcmp(st->w[2], "as") != 0) {
			continue;
		}
		memset(&cp, 0, sizeof cp);
		cp.group = pcb_strdup(st->w[1]);
		cp.src = st->w[3];
		cp.line = st->line;
		free(st->w[2]);
		st->n = 2;
		b->copy = pcb_grow(b->copy, b->ncopy, sizeof *b->copy);
		b->copy[b->ncopy++] = cp;
	}
	for (size_t c = 0; c < b->ncopy; c++) {
		struct pcb_copy *cp = &b->copy[c];

		cp->ok = copy_alike(b, cp, 0);
		for (size_t i = 0; cp->ok && i < n0; i++) {
			struct pcb_stmt st;
			size_t u = 0;

			if (!copied(b, cp, &b->stmt[i])) {
				continue;
			}
			st.n = b->stmt[i].n;
			st.line = cp->line;
			st.w = (char **)pcb_alloc((size_t)st.n * sizeof *st.w);
			for (int j = 0; j < st.n; j++) {
				st.w[j] = in_copy(b, cp, b->stmt[i].w[j]);
			}
			while (u < n0 && !same_place(&b->stmt[u], &st)) {
				u++;
			}
			if (u < n0) {
				cp->dup = pcb_grow(cp->dup, cp->ndup,
						   sizeof *cp->dup);
				cp->dup_line = pcb_grow(cp->dup_line, cp->ndup,
							sizeof *cp->dup_line);
				cp->dup[cp->ndup] = u;
				cp->dup_line[cp->ndup++] = b->stmt[u].line;
				for (int j = 0; j < b->stmt[u].n; j++) {
					free(b->stmt[u].w[j]);
				}
				free((void *)b->stmt[u].w);
				b->stmt[u] = st;
				continue;
			}
			b->stmt = pcb_grow(b->stmt, b->nstmt, sizeof *b->stmt);
			b->stmt[b->nstmt++] = st;
		}
	}
}

void pcb_tell_copies(struct pcb *b)
{
	for (size_t c = 0; c < b->ncopy; c++) {
		struct pcb_copy *cp = &b->copy[c];

		if (!copy_alike(b, cp, 1)) {
			continue;
		}
		for (size_t i = 0; i < cp->ndup; i++) {
			pcb_error(b, cp->dup_line[i],
				  "%s is of the copy %s, laid out as in its "
				  "source %s",
				  b->stmt[cp->dup[i]].w[1], cp->group, cp->src);
		}
	}
}

const struct pcb_stmt *pcb_placed_by(const struct pcb *b, size_t k)
{
	const char *name = b->part[k].name;
	const struct pcb_stmt *by = NULL;

	for (size_t s = 0; s < b->nstmt; s++) {
		const struct pcb_stmt *st = &b->stmt[s];

		if ((!strcmp(st->w[0], "place") || !strcmp(st->w[0], "fix")) &&
		    st->n >= 2 && !strcmp(st->w[1], name)) {
			by = st;
		}
	}
	for (size_t s = 0; !by && s < b->nstmt; s++) {
		const struct pcb_stmt *st = &b->stmt[s];

		if (!strcmp(st->w[0], "place") &&
		    ((st->n >= PLACE_OF_WORDS && !strcmp(st->w[3], "from")) ||
		     (st->n >= 4 && !strcmp(st->w[2], "near"))) &&
		    of_group(name, st->w[1])) {
			by = st;
		}
	}
	return by;
}

// A part record of a section of the source's lock, in the copy, to f:
// the part renamed, by its statement now.
static void copy_part(const struct pcb *b, const struct pcb_copy *cp, char **w,
		      FILE *f)
{
	char *name = moved(cp, w[1]);
	long k = part_named(b, name);
	const struct pcb_stmt *st = k >= 0 ? pcb_placed_by(b, (size_t)k) : NULL;

	if (st) {
		char *by = pcb_by(b, (size_t)k, st);

		(void)fprintf(f, "part %s %s %s %s %s %s %s\n", name,
			      pcb_ref(&b->part[k]), w[LOCK_X], w[LOCK_X + 1],
			      w[LOCK_X + 2], w[LOCK_SIDE], by);
		free(by);
	}
	free(name);
}

// A net record of a section of the source's lock, in the copy, to f: the
// net renamed, by its words now.
static void copy_net(const struct pcb *b, const struct pcb_copy *cp, char **w,
		     FILE *f)
{
	char *name = in_copy(b, cp, w[1]);
	long k = net_named(b, name);
	char *by = k >= 0 ? pcb_net_words(b, (size_t)k) : pcb_strdup("");

	(void)fprintf(f, "net %s by%s%s\n", name, *by ? " " : "", by);
	free(name);
	free(by);
}

// The designator, in the copy, of the part of the pad PART.PAD of the
// source: a new string.
static char *copy_ref(const struct pcb *b, const struct pcb_copy *cp,
		      const char *pad)
{
	char *part = in_copy(b, cp, pad);
	char *dot = strrchr(part, '.');
	long k = 0;
	char *ref = NULL;

	if (dot) {
		*dot = 0;
	}
	k = part_named(b, part);
	ref = pcb_strdup(k >= 0 ? pcb_ref(&b->part[k]) : "-");
	free(part);
	return ref;
}

// A line of a section of the source's lock, in the copy: its parts and
// nets renamed; a part by its statement now and a net by its words now.
static void copy_line(const struct pcb *b, const struct pcb_copy *cp,
		      const struct pcb_stmt *r, FILE *f)
{
	char **w = r->w;
	int n = r->n;
	int last = 1;

	if (!strcmp(w[0], "part") && n >= PART_WORDS) {
		copy_part(b, cp, w, f);
		return;
	}
	if (!strcmp(w[0], "net") && n >= 3) {
		copy_net(b, cp, w, f);
		return;
	}
	if (!strcmp(w[0], "link")) {
		last = 3;
	} else if (!strcmp(w[0], "pad")) {
		last = 2;
	}
	for (int i = 0; i < n; i++) {
		char *t = NULL;

		// A pad's designator: its part's in the copy.
		if (!strcmp(w[0], "pad") && i == 3 && n == PAD_WORDS + 3) {
			t = copy_ref(b, cp, w[2]);
		} else if (i && i <= last) {
			t = in_copy(b, cp, w[i]);
		} else {
			t = pcb_strdup(w[i]);
		}
		(void)fprintf(f, "%s%s", i ? " " : "", t);
		free(t);
	}
	(void)fputc('\n', f);
}

// The text of a section of the source's lock, in the copy.
static char *copy_text(const struct pcb *b, const struct pcb_copy *cp,
		       const char *text)
{
	char *out = NULL;
	char *line = pcb_alloc(LINE);
	char **w = (char **)pcb_alloc(WORDS * sizeof *w);
	FILE *f = tmpfile();

	if (!f) {
		pcb_die("cannot write a temporary file");
	}
	while (*text) {
		size_t l = strcspn(text, "\n");
		struct pcb_stmt r = { w, 0, 0 };

		if (l >= LINE) {
			pcb_die("%s: a line of the lock is too long", b->file);
		}
		memcpy(line, text, l);
		line[l] = 0;
		text += l + (text[l] == '\n');
		r.n = split(line, w);
		if (r.n) {
			copy_line(b, cp, &r, f);
		}
	}
	out = stream_text(f);
	if (!out) {
		pcb_die("cannot write a temporary file");
	}
	free(line);
	free((void *)w);
	return out;
}

// The copy of a group, or NULL.
static const struct pcb_copy *copy_of(const struct pcb *b, const char *group)
{
	for (size_t i = 0; i < b->ncopy; i++) {
		if (!strcmp(b->copy[i].group, group)) {
			return &b->copy[i];
		}
	}
	return NULL;
}

// The pass of a copy: the lock with the copy's sections (its group's and
// those of groups in it) the source's, in the copy, in place of what
// they were; nothing is placed or routed.
static int copy_lock(const struct pcb_run *r)
{
	struct pcb b;
	const struct pcb_copy *cp = NULL;
	const char *group = r->group;
	size_t n = 0;

	pcb_read(&b, r->file, r->lib, r->in, NULL);
	cp = copy_of(&b, group);
	for (size_t i = 0; i < b.nsec; i++) {
		if (!strcmp(b.sec_name[i], group) ||
		    of_group(b.sec_name[i], group)) {
			free(b.sec_name[i]);
			free(b.sec_text[i]);
			continue;
		}
		b.sec_name[n] = b.sec_name[i];
		b.sec_text[n++] = b.sec_text[i];
	}
	b.nsec = n;
	for (size_t i = 0; cp && cp->ok && i < n; i++) {
		if (!strcmp(b.sec_name[i], cp->src) ||
		    of_group(b.sec_name[i], cp->src)) {
			char *name = moved(cp, b.sec_name[i]);
			char *text = copy_text(&b, cp, b.sec_text[i]);
			size_t k = store_of(&b, name);

			free(b.sec_text[k]);
			b.sec_text[k] = text;
			free(name);
		}
	}
	pcb_write_lock(&b, r->out);
	pcb_free(&b);
	return 0;
}

// A group of a block statement, to order the passes: its name, its
// depth, and whether it is a copy.
struct pass_group {
	const char *name;
	size_t depth;
	int copy;
};

static int deeper_first(const void *lhs, const void *rhs)
{
	const struct pass_group *a = lhs;
	const struct pass_group *b = rhs;

	if (a->depth != b->depth) {
		return (a->depth < b->depth) - (a->depth > b->depth);
	}
	if (a->copy != b->copy) {
		return a->copy - b->copy;
	}
	return strcmp(a->name, b->name);
}

// Whether a group is laid out by no pass: a copy not alike its source,
// or a group in a copy.
static int no_pass(const struct pcb *b, const char *group)
{
	for (size_t i = 0; i < b->ncopy; i++) {
		if (of_group(group, b->copy[i].group) ||
		    (!b->copy[i].ok && !strcmp(group, b->copy[i].group))) {
			return 1;
		}
	}
	return 0;
}

// The groups of the block statements of src that a pass lays out, in
// the order of the passes; their number goes to *n.
static struct pass_group *pass_groups(const struct pcb *src, size_t *n)
{
	struct pass_group *g = NULL;

	*n = 0;
	for (size_t i = 0; i < src->nstmt; i++) {
		const struct pcb_stmt *st = &src->stmt[i];

		if (!strcmp(st->w[0], "block") && st->n == 2 &&
		    !no_pass(src, st->w[1])) {
			g = pcb_grow(g, *n, sizeof *g);
			g[*n].name = st->w[1];
			g[*n].depth = depth(st->w[1]);
			g[(*n)++].copy = copy_of(src, st->w[1]) != NULL;
		}
	}
	if (*n) {
		qsort(g, *n, sizeof *g, deeper_first);
	}
	return g;
}

// The lock the pass before wrote into the temporary file f, to read
// from its start; NULL for none, f closed.  Dies if it is not all
// written.
static FILE *lock_in(FILE *f)
{
	long n = ftell(f);

	if (n < 0 || !stream_rewind(f)) {
		pcb_die("cannot write a temporary file");
	}
	if (!n) {
		(void)fclose(f);
		return NULL;
	}
	return f;
}

int pcb_passes(const struct pcb_run *r, pcb_pass *pass)
{
	struct pcb src;
	struct pass_group *groups = NULL;
	FILE *prev = NULL; // The lock the pass before wrote, to read.
	size_t ng = 0;
	int failed = 0;

	pcb_read(&src, r->file, r->lib, NULL, NULL);
	if (r->group) {
		const struct pcb_copy *cp = copy_of(&src, r->group);

		failed = cp && cp->ok ? copy_lock(r) : pass(r);
		pcb_free(&src);
		return failed;
	}
	groups = pass_groups(&src, &ng);
	for (size_t i = 0; i <= ng; i++) {
		struct pcb_run step = *r;

		step.group = i < ng ? groups[i].name : NULL;
		step.out = i < ng ? tmpfile() : r->out;
		step.in = i ? prev : r->in;
		if (!step.out) {
			pcb_die("cannot write a temporary file");
		}
		if (i < ng && groups[i].copy) {
			failed = copy_lock(&step) || failed;
		} else {
			failed = pass(&step) || failed;
		}
		if (i && step.in) {
			(void)fclose(step.in);
		}
		prev = i < ng ? lock_in(step.out) : NULL;
	}
	free(groups);
	pcb_free(&src);
	return failed;
}

FILE *pcb_open_lock(const char *path)
{
	return path ? fopen(path, "r") : stdin;
}

char *pcb_section(const struct pcb *b, const char *group)
{
	for (size_t i = 0; i < b->nsec; i++) {
		if (!strcmp(b->sec_name[i], group ? group : "")) {
			return b->sec_text[i];
		}
	}
	return NULL;
}

// The defaults of the rules, in mm.
#define CLEARANCE 0.2
#define TRACK 0.2
#define VIA_DIA 0.6
#define VIA_DRILL 0.3
#define EDGE 0.5
#define VIAGAP 0.3
#define MASKWEB 0.1

// Reads the statements of the source f, the footprints from fs and the
// pins of the netlist pending in pd.
static void read_source(struct pcb *b, FILE *f, const struct fp_search *fs,
			struct pending *pd)
{
	char *buf = pcb_alloc(LINE);
	char **w = (char **)pcb_alloc(WORDS * sizeof *w);
	int ln = 0;

	while (fgets(buf, LINE, f)) {
		struct pcb_stmt st = { w, split(buf, w), 0 };

		ln++;
		st.line = ln;
		if (st.n) {
			statement(b, &st, fs, pd);
		}
	}
	free(buf);
	free((void *)w);
}

// Gives each net its width, its class's or rule track, and its plane.
static void net_widths(struct pcb *b)
{
	for (size_t k = 0; k < b->nnet; k++) {
		b->net[k].width = b->track;
		for (size_t i = 0; b->net[k].klass && i < b->nclass; i++) {
			if (!strcmp(b->klass[i], b->net[k].klass)) {
				b->net[k].width = b->class_width[i];
			}
		}
		for (size_t i = 0; i < b->nlayer; i++) {
			if (b->layer[i].kind == LAYER_PLANE &&
			    b->layer[i].net &&
			    !strcmp(b->layer[i].net, b->net[k].name)) {
				b->net[k].plane = (int)i;
			}
		}
	}
}

// Whether the width statement st, width PIN PIN W, is of two pins of one
// net.
static int pins_width_ok(const struct pcb *b, const struct pcb_stmt *st)
{
	struct pcb_end p[2] = { { 0, 0 }, { 0, 0 } };
	double v = NAN;

	return pcb_pin(b, st->w[1], &p[0]) && pcb_pin(b, st->w[2], &p[1]) &&
	       pcb_net_of(b, p[0].part, p[0].pad) >= 0 &&
	       pcb_net_of(b, p[0].part, p[0].pad) ==
		       pcb_net_of(b, p[1].part, p[1].pad) &&
	       number(st->w[3], &v);
}

// Carries out the width statements: width NET W sets the width of the
// net; width PIN PIN W, of a connection, is checked.
static void read_widths(struct pcb *b)
{
	for (size_t i = 0; i < b->nstmt; i++) {
		const struct pcb_stmt *st = &b->stmt[i];
		double v = NAN;
		long k = 0;

		if (strcmp(st->w[0], "width") != 0) {
			continue;
		}
		if (st->n == 4) {
			// The copper between two pins of one net.
			if (!pins_width_ok(b, st)) {
				pcb_error(
					b, st->line,
					"width PIN PIN W, of two pins of one net");
			}
			continue;
		}
		k = st->n == 3 ? net_named(b, st->w[1]) : -1;
		if (k < 0 || !number(st->w[2], &v)) {
			pcb_error(
				b, st->line,
				"width NET W, of a net of the netlist; or width "
				"PIN PIN W");
			continue;
		}
		b->net[k].width = v;
	}
}

// Reads the section of the group of the lock into b.
static void read_section(struct pcb *b, FILE *lock, const char *group)
{
	FILE *mine = sections(b, lock, group);

	read_lock(b, mine);
	(void)fclose(mine);
}

void pcb_read(struct pcb *b, const char *file, struct pcb_lib *lib, FILE *lock,
	      const char *group)
{
	static const char own[] = "/footprints";
	struct fp_search fs = { NULL, lib->ndir + 1, lib };
	struct pending pd = { NULL };
	const char *slash = strrchr(file, '/');
	size_t dn = slash ? (size_t)(slash - file) : 1;
	FILE *f = fopen(file, "r");

	memset(b, 0, sizeof *b);
	b->file = file;
	b->group = group;
	b->clearance = CLEARANCE;
	b->track = TRACK;
	b->via_dia = VIA_DIA;
	b->via_drill = VIA_DRILL;
	b->edge = EDGE;
	b->viagap = VIAGAP;
	b->maskweb = MASKWEB;
	// The project's own footprints come first: those next to the source.
	fs.dir = (char **)pcb_alloc(fs.ndir * sizeof *fs.dir);
	fs.dir[0] = pcb_alloc(dn + sizeof(own));
	memcpy(fs.dir[0], slash ? file : ".", dn);
	memcpy(fs.dir[0] + dn, own, sizeof(own));
	if (lib->ndir) {
		memcpy((void *)(fs.dir + 1), (const void *)lib->dir,
		       lib->ndir * sizeof *lib->dir);
	}
	if (!f) {
		pcb_die("%s: cannot read", file);
	}
	read_source(b, f, &fs, &pd);
	(void)fclose(f);
	leave_unplaced(b, &pd);
	for (size_t i = 0; i < b->npart; i++) {
		if (!b->part[i].fp && !b->part[i].unplaced) {
			pcb_error(b, 0, "%s (%s, %s) has no footprint",
				  b->part[i].name, pcb_ref(&b->part[i]),
				  b->part[i].mpn ? b->part[i].mpn :
						   b->part[i].type);
		}
	}
	give_pads(b, &pd);
	read_copies(b);
	net_widths(b);
	read_widths(b);
	if (lock) {
		read_section(b, lock, group);
	}
	free(fs.dir[0]);
	free((void *)fs.dir);
}

void pcb_free(struct pcb *b)
{
	for (size_t i = 0; i < b->nexcept; i++) {
		free(b->edge_except[i]);
	}
	free((void *)b->edge_except);
	for (size_t i = 0; i < b->nclass; i++) {
		free(b->klass[i]);
	}
	free((void *)b->klass);
	free(b->class_width);
	for (size_t i = 0; i < b->nlayer; i++) {
		free(b->layer[i].name);
		free(b->layer[i].net);
	}
	free(b->layer);
	for (size_t i = 0; i < b->npart; i++) {
		struct pcb_part *q = &b->part[i];

		if (q->name != q->ref) {
			free(q->name);
		}
		free(q->ref);
		free(q->type);
		free(q->value);
		free(q->mpn);
		free(q->pins);
		free(q->by);
	}
	free(b->part);
	for (size_t k = 0; k < b->nnet; k++) {
		free(b->net[k].name);
		free(b->net[k].klass);
		free(b->net[k].end);
		if (b->netby) {
			free(b->netby[k]);
		}
		for (size_t j = 0; b->netpad && j < b->nnetpad[k]; j++) {
			free(b->netpad[k][j]);
		}
		if (b->netpad) {
			free((void *)b->netpad[k]);
		}
	}
	free(b->net);
	free((void *)b->netby);
	free((void *)b->netpad);
	free(b->nnetpad);
	for (size_t i = 0; i < b->ntrack; i++) {
		free(b->track_[i].xy);
	}
	free(b->track_);
	free(b->via);
	for (size_t i = 0; i < b->nlink; i++) {
		free(b->link[i].a);
		free(b->link[i].b);
	}
	free(b->link);
	for (size_t i = 0; i < b->nstmt; i++) {
		for (int j = 0; j < b->stmt[i].n; j++) {
			free(b->stmt[i].w[j]);
		}
		free((void *)b->stmt[i].w);
	}
	free(b->stmt);
	for (size_t i = 0; i < b->ncopy; i++) {
		free(b->copy[i].group);
		free(b->copy[i].src);
		free(b->copy[i].net);
		free(b->copy[i].dup);
		free(b->copy[i].dup_line);
	}
	free(b->copy);
	for (size_t i = 0; i < b->nsec; i++) {
		free(b->sec_name[i]);
		free(b->sec_text[i]);
	}
	free((void *)b->sec_name);
	free((void *)b->sec_text);
	memset(b, 0, sizeof *b);
}

// A part or a net by its name, to sort.
struct named {
	const char *name;
	size_t i;
};

static int by_name(const void *lhs, const void *rhs)
{
	const struct named *x = lhs;
	const struct named *y = rhs;
	int c = strcmp(x->name, y->name);

	if (c) {
		return c;
	}
	return (x->i > y->i) - (x->i < y->i);
}

// The parts or the nets in the order of their names, so a lock lists
// them the same way whatever the netlist's order.
static size_t *sorted(const struct pcb *b, int parts)
{
	size_t n = parts ? b->npart : b->nnet;
	struct named *v = pcb_alloc((n + 1) * sizeof *v);
	size_t *o = pcb_alloc((n + 1) * sizeof *o);

	for (size_t i = 0; i < n; i++) {
		v[i].name = parts ? b->part[i].name : b->net[i].name;
		v[i].i = i;
	}
	qsort(v, n, sizeof *v, by_name);
	for (size_t i = 0; i < n; i++) {
		o[i] = v[i].i;
	}
	free(v);
	return o;
}

// Writes the tracks and vias of net k of connection l (-1: of none).
static void write_copper(const struct pcb *b, FILE *f, int k, int l)
{
	for (size_t i = 0; i < b->ntrack; i++) {
		const struct pcb_track *t = &b->track_[i];

		if (t->net != k || t->link != l) {
			continue;
		}
		(void)fprintf(f, "track %s %s %.4f", b->net[k].name,
			      b->layer[t->layer].name, t->w);
		for (size_t j = 0; j < t->n; j++) {
			(void)fprintf(f, " %.4f %.4f", t->xy[2 * j],
				      t->xy[(2 * j) + 1]);
		}
		(void)fprintf(f, "\n");
	}
	for (size_t i = 0; i < b->nvia; i++) {
		const struct pcb_via *v = &b->via[i];

		if (v->net == k && v->link == l) {
			(void)fprintf(f, "via %s %.4f %.4f %.4f %.4f\n",
				      b->net[k].name, v->x, v->y, v->dia,
				      v->drill);
		}
	}
}

static int by_text(const void *lhs, const void *rhs)
{
	return strcmp(*(char *const *)lhs, *(char *const *)rhs);
}

// Writes the section read, made anew.
static void write_section(const struct pcb *b, FILE *f);

void pcb_write_lock(const struct pcb *b, FILE *f)
{
	const char **names =
		(const char **)pcb_alloc((b->nsec + 2) * sizeof *names);
	size_t n = 0;
	int mine_done = 0;

	(void)fprintf(
		f, "# lock of %s: made by the programs; edit the source\n",
		strrchr(b->file, '/') ? strrchr(b->file, '/') + 1 : b->file);
	for (size_t i = 0; i < b->nsec; i++) {
		names[n++] = b->sec_name[i];
	}
	names[n++] = b->group ? b->group : "";
	qsort((void *)names, n, sizeof *names, by_text);
	for (size_t i = 0; i < n; i++) {
		int mine = !strcmp(names[i], b->group ? b->group : "");
		const char *t = pcb_section(b, names[i]);

		if (i && !strcmp(names[i], names[i - 1])) {
			continue;
		}
		if (*names[i]) {
			(void)fprintf(f, "group %s\n", names[i]);
		}
		if (mine && !mine_done) {
			write_section(b, f);
			mine_done = 1;
		} else if (t) {
			(void)fputs(t, f);
		}
	}
	free((void *)names);
}

// Writes the pads of net k as they stood when its copper was made, each
// by its part and name, then its part's designator.
static void write_pads(const struct pcb *b, FILE *f, size_t k)
{
	for (size_t i = 0; i < b->nnetpad[k]; i++) {
		const char *p = b->netpad[k][i];
		const char *sp = strchr(p, ' ');
		char *part = NULL;
		char *dot = NULL;
		long q = 0;

		if (!sp) {
			continue;
		}
		part = pcb_strdup(p);
		part[sp - p] = 0;
		dot = strrchr(part, '.');
		if (dot) {
			*dot = 0;
		}
		q = part_named(b, part);
		free(part);
		(void)fprintf(f, "pad %s %.*s %s%s\n", b->net[k].name,
			      (int)(sp - p), p,
			      q >= 0 ? pcb_ref(&b->part[q]) : "-", sp);
	}
}

// Writes the copper of net k: of no connection, then of each of its
// connections, after the connection.
static void write_links(const struct pcb *b, FILE *f, size_t k)
{
	for (long l = -1; l < (long)b->nlink; l++) {
		if (l >= 0 && b->link[l].net != (int)k) {
			continue;
		}
		if (l >= 0) {
			(void)fprintf(f, "link %s %s %s %.4f\n", b->net[k].name,
				      b->link[l].a, b->link[l].b, b->link[l].w);
		}
		write_copper(b, f, (int)k, (int)l);
	}
}

static void write_section(const struct pcb *b, FILE *f)
{
	size_t *po = sorted(b, 1);
	size_t *no = sorted(b, 0);

	for (size_t o = 0; o < b->npart; o++) {
		const struct pcb_part *q = &b->part[po[o]];

		if (q->placed) {
			(void)fprintf(f, "part %s %s %.4f %.4f %d %s %s\n",
				      q->name, pcb_ref(q), q->x, q->y, q->rot,
				      q->bottom ? "bottom" : "top",
				      q->by ? q->by : "by");
		}
	}
	for (size_t o = 0; o < b->nnet; o++) {
		size_t k = no[o];

		if (!b->netby[k]) {
			continue;
		}
		(void)fprintf(f, "net %s by%s%s\n", b->net[k].name,
			      *b->netby[k] ? " " : "", b->netby[k]);
		write_pads(b, f, k);
		write_links(b, f, k);
	}
	free(po);
	free(no);
}

int pcb_opening(const struct pcb *b, struct pcb_end at, int bottom,
		double box[4])
{
	struct pcb_spot s;

	pcb_spot(b, &b->part[at.part], at.pad, &s);
	if (s.layer >= 0 && (s.layer != 0) != bottom) {
		return 0;
	}
	box[0] = s.x - (s.w / 2) - MASK_GROW;
	box[1] = s.y - (s.h / 2) - MASK_GROW;
	box[2] = s.x + (s.w / 2) + MASK_GROW;
	box[3] = s.y + (s.h / 2) + MASK_GROW;
	return 1;
}

#define SILK_H 0.8 // The height of a designator.
#define SILK_GAP 0.15 // From the courtyard to the designator.
#define SILK_STEP 0.1 // The step of the sizes and gaps a designator tries.
#define NEAR_PADS 1.0 // How near a box of text a part's openings matter.
#define HINT_MOST 2.0 // The most a part is told to move for a designator.
#define TENTHS 10.0 // A hint's lengths are in tenths of a mm.
#define ROUND_SLACK 1e-9 // What rounding up to tenths lets go.
#define NO_NEED 1e-10 // A need below this is none.

static int overlap(const double a[4], const double c[4])
{
	return a[0] < c[2] && c[0] < a[2] && a[1] < c[3] && c[1] < a[3];
}

// The side of a courtyard c a box of text is out from, to tell how far
// what is in its way must move: 0 above, 1 below, 2 left, 3 right; -1
// for none.
struct out_side {
	const double *c;
	int sd;
};

// The most need so far, and the part of it.
struct need {
	double need;
	size_t who;
};

// The boxes of text placed: their boxes, sides, and parts (owner, NULL
// for none), n of them.
struct texts {
	double (*box)[4];
	int *side;
	size_t *owner;
	size_t n;
};

// The need of box t of text for room against box r of part p, when they
// overlap: how far r must move out from the side of os to clear t,
// HUGE_VAL if r is not past that side (or there is none); the most need
// so far in nd.  Returns whether the need is past help.
static int need_of(const double t[4], const double r[4],
		   const struct out_side *os, size_t p, struct need *nd)
{
	const double *c = os->c;
	double e = HUGE_VAL;

	if (!overlap(t, r)) {
		return 0;
	}
	if (os->sd == 0 && r[1] >= c[3] - EPS) {
		e = t[3] - r[1];
	} else if (os->sd == 1 && r[3] <= c[1] + EPS) {
		e = r[3] - t[1];
	} else if (os->sd == 2 && r[2] <= c[0] + EPS) {
		e = r[2] - t[0];
	} else if (os->sd == 3 && r[0] >= c[2] - EPS) {
		e = t[2] - r[0];
	}
	if (e > nd->need) {
		nd->need = e;
		nd->who = p;
	}
	return nd->need == HUGE_VAL;
}

// Whether box t of text is outside the board less the edge rule.
static int off_board(const struct pcb *b, const double t[4])
{
	return t[0] < b->board[0] + b->edge || t[1] < b->board[1] + b->edge ||
	       t[2] > b->board[2] - b->edge || t[3] > b->board[3] - b->edge;
}

// The need of box t of text on a side (bottom or not) for room against
// part p: its courtyard, but for part q's own, and its openings.
// Returns whether the need is past help.
static int part_need(const struct pcb *b, size_t q, int bottom,
		     const double t[4], const struct out_side *os, size_t p,
		     struct need *nd)
{
	const struct pcb_part *o = &b->part[p];
	double oc[4];
	double g[4];

	if (!o->placed || !o->fp) {
		return 0;
	}
	pcb_court(o, oc);
	if (p != q && o->bottom == bottom && need_of(t, oc, os, p, nd)) {
		return 1;
	}
	// The openings of a part far from the box are no matter.
	g[0] = oc[0] - NEAR_PADS;
	g[1] = oc[1] - NEAR_PADS;
	g[2] = oc[2] + NEAR_PADS;
	g[3] = oc[3] + NEAR_PADS;
	for (size_t k = 0; overlap(t, g) && k < o->fp->npad; k++) {
		struct pcb_end at = { p, k };

		if (pcb_opening(b, at, bottom, oc) &&
		    need_of(t, oc, os, p, nd)) {
			return 1;
		}
	}
	return 0;
}

// How far the things in the way of a box of text on a side (bottom or
// not) must move out for it to be clear (text_need below, with no side
// in os: 0 if clear, else HUGE_VAL): the openings, the courtyards of
// the parts of its side but part q (its own; b->npart for none), the
// boxes of text of its side placed (done), and the board inside the
// edge rule, which never moves.  The part in the way most goes to who
// (b->npart for the edge), unless NULL.
static double text_need(const struct pcb *b, size_t q, int bottom,
			const double t[4], const struct texts *done,
			const struct out_side *os, size_t *who)
{
	struct need nd = { 0, b->npart };
	double need = 0;

	if (off_board(b, t)) {
		need = HUGE_VAL;
	}
	for (size_t i = 0; need == 0 && i < done->n; i++) {
		if (done->side[i] == bottom &&
		    need_of(t, done->box[i], os,
			    done->owner ? done->owner[i] : b->npart, &nd)) {
			need = -1;
		}
	}
	for (size_t p = 0; need == 0 && p < b->npart; p++) {
		if (part_need(b, q, bottom, t, os, p, &nd)) {
			need = -1;
		}
	}
	if (who) {
		*who = nd.who;
	}
	return need > 0 ? need : nd.need;
}

// Whether a box of text on a side is clear (text_need).
static int text_clear(const struct pcb *b, size_t q, int bottom,
		      const double t[4], const struct texts *done)
{
	static const struct out_side none = { NULL, -1 };

	return text_need(b, q, bottom, t, done, &none, NULL) == 0;
}

// The strokes of a silkscreen under way.
struct strokes {
	struct pcb_stroke *s;
	size_t n;
};

static void stroke(struct strokes *ss, struct pcb_stroke st)
{
	ss->s = pcb_grow(ss->s, ss->n, sizeof *ss->s);
	ss->s[ss->n++] = st;
}

// The ref statement of the part of the name, or NULL.
static const struct pcb_stmt *ref_of(const struct pcb *b, const char *name)
{
	for (size_t i = 0; i < b->nstmt; i++) {
		if (!strcmp(b->stmt[i].w[0], "ref") &&
		    !strcmp(b->stmt[i].w[1], name)) {
			return &b->stmt[i];
		}
	}
	return NULL;
}

// The height of the designator of a ref statement: its size, or else
// SILK_H.
static double ref_size(const struct pcb_stmt *rs)
{
	for (int k = 3; k + 1 < rs->n; k++) {
		if (!strcmp(rs->w[k], "size")) {
			return strtod(rs->w[k + 1], NULL);
		}
	}
	return SILK_H;
}

// Whether a ref statement turns its designator.
static int ref_turned(const struct pcb_stmt *rs)
{
	for (int k = 3; k < rs->n; k++) {
		if (!strcmp(rs->w[k], "rot")) {
			return 1;
		}
	}
	return 0;
}

// Where a ref statement aligns its designator: 0 at the start of its
// side, 1 in the middle, 2 at the end.
static int ref_align(const struct pcb_stmt *rs)
{
	const char *al = rs->n > 3 ? rs->w[3] : "";

	if (!strcmp(al, "start")) {
		return 0;
	}
	return !strcmp(al, "end") ? 2 : 1;
}

// The size of a box.
struct dims {
	double w, h;
};

// Of a box of length len along the side from lo to hi, where it starts:
// at the start (a 0), the end (a 2) or in the middle.
static double aligned(int a, double lo, double hi, double len)
{
	if (a == 0) {
		return lo;
	}
	return a == 2 ? hi - len : (lo + hi - len) / 2;
}

// The lower left corner of a designator's box, of size d, by the
// courtyard c: on the side sd, at the gap, from the start of the side
// (its left or top end), its middle or its end (a, as ref_align).
static void ref_at(const char *sd, int a, const double c[4], struct dims d,
		   double at[2])
{
	int across = !strcmp(sd, "above") || !strcmp(sd, "below");

	if (across) {
		at[0] = aligned(a, c[0], c[2], d.w);
		at[1] = !strcmp(sd, "above") ? c[3] + SILK_GAP :
					       c[1] - SILK_GAP - d.h;
	} else {
		at[0] = !strcmp(sd, "left") ? c[0] - SILK_GAP - d.w :
					      c[2] + SILK_GAP;
		at[1] = aligned(2 - a, c[1], c[3], d.h);
	}
}

enum {
	RULE_SIZES = 4, // The sizes a designator by rule tries: 0.8 to 0.5.
	RULE_GAPS = 9, // The gaps it tries: SILK_GAP, out by 0.1 mm.
	// The places at a size: 4 sides, 2 ways, each gap; at all sizes.
	SIZE_PLACES = 8 * RULE_GAPS,
	RULE_PLACES = SIZE_PLACES * RULE_SIZES,
	// The places a hint tries on a side, 2 ways, 3 alignments; and at
	// a size, on the 4 sides.
	SIDE_TRIES = 6,
	HINT_TRIES = 4 * SIDE_TRIES,
	TRIED_LEN = 40 // Room for the sides a hint tried.
};

// The box of the text t at place q, of its capital height, turned or
// not.
static struct dims ref_dims(const char *t, const struct place *q)
{
	double h = q->size;
	double tw = stroke_width(t, h);
	double dn = stroke_descends(t) ? h / 3 : 0;
	struct dims d = { tw, h + dn };

	if (q->rot) {
		d.w = h + dn;
		d.h = tw;
	}
	return d;
}

// Sets the box of place q of the text t from its size and turn.
static void place_dims(struct place *q, const char *t)
{
	struct dims d = ref_dims(t, q);

	q->w = d.w;
	q->h = d.h;
}

// The places of a designator with no ref statement, in the order it
// takes them: from full size down to 0.5 mm, at each size on the long
// sides first, then on the short ones; on either of a pair, its text
// along the courtyard's long side (turned for a part taller than wide),
// then the other way, and so as near the courtyard as fits, centered on
// the side.  Returns their number.
static size_t ref_places(const struct pcb_part *p, const char *ref,
			 struct place *o)
{
	double c[4];
	int tall = 0;

	pcb_court(p, c);
	tall = c[3] - c[1] > c[2] - c[0];
	for (int k = 0; k < RULE_PLACES; k++) {
		struct place *q = &o[k];
		int side = k % 2;
		int r = (k / (2 * RULE_GAPS)) % 2;
		double gap = SILK_GAP + (SILK_STEP * ((k / 2) % RULE_GAPS));
		// Across the long axis (the long sides) or along it.
		int across = tall == ((k / (4 * RULE_GAPS)) % 2);

		int z = k / SIZE_PLACES;

		q->size = SILK_H - (SILK_STEP * z);
		q->rot = tall != r;
		place_dims(q, ref);
		if (across) {
			q->x = (c[0] + c[2] - q->w) / 2;
			q->y = side ? c[1] - gap - q->h : c[3] + gap;
		} else {
			q->y = (c[1] + c[3] - q->h) / 2;
			q->x = side ? c[2] + gap : c[0] - gap - q->w;
		}
	}
	return RULE_PLACES;
}

// The box of a place, its pen round it.
static void place_box(const struct place *q, double t[4])
{
	double pen = STROKE_PEN * q->size;

	t[0] = q->x - pen;
	t[1] = q->y - pen;
	t[2] = q->x + q->w + pen;
	t[3] = q->y + q->h + pen;
}

// Writes the strokes of a text in its place: from the box's lower left,
// its baseline a descender up; turned, reading upward; on the bottom,
// read from below, mirrored about the box's middle; of the part, side
// and kind (ref) of pen.
static void text_strokes(struct strokes *ss, const struct pcb_stroke *pen,
			 const char *text, const struct place *at)
{
	double *sg = NULL;
	double dn = stroke_descends(text) ? at->size / 3 : 0;
	size_t ns = stroke_text(text, at->size, &sg);

	for (size_t k = 0; k < ns; k++) {
		struct pcb_stroke st = *pen;
		double xy[4];

		for (size_t e = 0; e < 4; e += 2) {
			double lx = sg[(4 * k) + e];
			double ly = sg[(4 * k) + e + 1] + dn;

			xy[e] = at->x + (at->rot ? at->w - ly : lx);
			xy[e + 1] = at->y + (at->rot ? lx : ly);
			if (pen->bottom) {
				xy[e] = (2 * at->x) + at->w - xy[e];
			}
		}
		st.w = STROKE_PEN * at->size;
		st.x0 = xy[0];
		st.y0 = xy[1];
		st.x1 = xy[2];
		st.y1 = xy[3];
		stroke(ss, st);
	}
	free(sg);
}

// The words of a text statement: text TEXT at X Y [size H] [bottom];
// its place goes to at, its side to bottom, and its text to the caller
// to free.  Returns NULL if they are not so.
static char *free_text(const struct pcb_stmt *st, struct place *at, int *bottom)
{
	int k = st->n - 1;
	int a = 0;
	double v[3];
	char *t = NULL;

	*bottom = k >= 0 && !strcmp(st->w[k], "bottom");
	k -= *bottom;
	at->size = SILK_H;
	if (k >= 1 && !strcmp(st->w[k - 1], "size")) {
		if (!number(st->w[k], &v[2]) || !(v[2] > 0)) {
			return NULL;
		}
		at->size = v[2];
		k -= 2;
	}
	a = k - 2;
	if (a < 2 || strcmp(st->w[a], "at") != 0 ||
	    !number(st->w[a + 1], &v[0]) || !number(st->w[a + 2], &v[1])) {
		return NULL;
	}
	t = pcb_words(st->w + 1, a - 1);
	at->x = v[0];
	at->rot = 0;
	place_dims(at, t);
	at->y = v[1] - (stroke_descends(t) ? at->size / 3 : 0);
	return t;
}

// The silkscreen under way: of board b, the boxes of the designators
// placed, the place of each part's (at), whether it has one (has: 1, 2
// for one that found no room), and the strokes.
struct silk {
	const struct pcb *b;
	struct texts done;
	struct place *at;
	char *has;
	struct strokes out;
};

// Takes the place at of a designator, of a side, among those done.
static void take_place(struct silk *sk, const struct place *at, int bottom)
{
	struct texts *d = &sk->done;

	place_box(at, d->box[d->n]);
	d->side[d->n++] = bottom;
}

// Places the designators that a ref statement says where; returns the
// number of the rest, in order.
static size_t refs_by_statement(struct silk *sk, size_t *order)
{
	const struct pcb *b = sk->b;
	size_t nrule = 0;

	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *p = &b->part[q];
		const char *ref = p->ref ? p->ref : "";
		const struct pcb_stmt *rs = ref_of(b, p->name);
		struct place *at = &sk->at[q];
		double c[4];
		double ra[2];

		if (!p->placed || !p->fp || !*ref) {
			continue;
		}
		if (!rs) {
			order[nrule++] = q;
			continue;
		}
		pcb_court(p, c);
		at->size = ref_size(rs);
		at->rot = ref_turned(rs);
		place_dims(at, ref);
		ref_at(rs->w[2], ref_align(rs), c,
		       (struct dims){ at->w, at->h }, ra);
		at->x = ra[0];
		at->y = ra[1];
		take_place(sk, at, p->bottom);
		sk->has[q] = 1;
	}
	return nrule;
}

// Orders the nrule parts of order, whose designators go by rule, the
// most cramped first: by the places each has clear of all but the
// designators, room for each.
static void rank_by_room(const struct pcb *b, size_t *order, size_t nrule,
			 struct place *o)
{
	static const struct texts none = { NULL, NULL, NULL, 0 };
	size_t *room = pcb_alloc((b->npart + 1) * sizeof *room);

	for (size_t i = 0; i < nrule; i++) {
		const struct pcb_part *p = &b->part[order[i]];
		size_t np = ref_places(p, p->ref, o);

		room[order[i]] = 0;
		for (size_t k = 0; k < np; k++) {
			double t[4];

			place_box(&o[k], t);
			room[order[i]] += (size_t)text_clear(
				b, order[i], p->bottom, t, &none);
		}
	}
	for (size_t i = 1; i < nrule; i++) {
		for (size_t j = i; j > 0 && room[order[j]] < room[order[j - 1]];
		     j--) {
			size_t t = order[j];

			order[j] = order[j - 1];
			order[j - 1] = t;
		}
	}
	free(room);
}

// Places the designator of part q by rule: the first place clear of
// those done, or with none the first, which pcbcheck tells.
static void ref_by_rule(struct silk *sk, size_t q, struct place *o)
{
	const struct pcb_part *p = &sk->b->part[q];
	size_t np = ref_places(p, p->ref, o);
	size_t pick = np;

	for (size_t k = 0; k < np; k++) {
		double t[4];

		place_box(&o[k], t);
		if (text_clear(sk->b, q, p->bottom, t, &sk->done)) {
			pick = k;
			break;
		}
	}
	sk->has[q] = pick == np ? 2 : 1;
	sk->at[q] = o[pick == np ? 0 : pick];
	take_place(sk, &sk->at[q], p->bottom);
}

// The strokes of the silkscreen lines of the footprint of part q.
static void fp_strokes(struct silk *sk, size_t q)
{
	const struct pcb_part *p = &sk->b->part[q];

	for (size_t i = 0; p->placed && p->fp && i < p->fp->nsilk; i++) {
		const struct fp_line *ln = &p->fp->silk[i];

		for (size_t j = 0; j + 1 < ln->n; j++) {
			const double *xy = ln->xy + (2 * j);
			struct pcb_pt a =
				turn(p->rot, (struct pcb_pt){ xy[0], xy[1] });
			struct pcb_pt e =
				turn(p->rot, (struct pcb_pt){ xy[2], xy[3] });
			struct pcb_stroke st = { ln->w,	     a.x + p->x,
						 a.y + p->y, e.x + p->x,
						 e.y + p->y, q,
						 0,	     p->bottom };

			stroke(&sk->out, st);
		}
	}
}

// The strokes of the text statements of the source, of no part.
static void free_texts(struct silk *sk)
{
	const struct pcb *b = sk->b;

	for (size_t i = 0; i < b->nstmt; i++) {
		struct place ta;
		int bottom = 0;
		char *t = NULL;

		if (strcmp(b->stmt[i].w[0], "text") != 0) {
			continue;
		}
		t = free_text(&b->stmt[i], &ta, &bottom);
		if (t) {
			struct pcb_stroke pen = { 0, 0,	    0,
						  0, 0,	    b->npart + i,
						  1, bottom };

			text_strokes(&sk->out, &pen, t, &ta);
			free(t);
		}
	}
}

struct pcb_strokes pcb_silk(const struct pcb *b)
{
	size_t nt = b->npart + b->nstmt + 1;
	double(*box)[4] = pcb_alloc(nt * sizeof *box);
	int *side = pcb_alloc(nt * sizeof *side);
	struct place *at = pcb_alloc((b->npart + 1) * sizeof *at);
	char *has = pcb_alloc(b->npart + 1);
	struct silk sk = { b, { box, side, NULL, 0 }, at, has, { NULL, 0 } };
	struct place *o = pcb_alloc((size_t)RULE_PLACES * sizeof *o);
	size_t *order = pcb_alloc((b->npart + 1) * sizeof *order);
	size_t nrule = 0;
	struct pcb_strokes out = { NULL, 0 };

	// A ref statement says where; those first take their places.
	nrule = refs_by_statement(&sk, order);
	// The rest by rule: the most cramped first, the places each has
	// clear of all but the designators counted, each taking its room.
	rank_by_room(b, order, nrule, o);
	for (size_t i = 0; i < nrule; i++) {
		ref_by_rule(&sk, order[i], o);
	}
	for (size_t q = 0; q < b->npart; q++) {
		fp_strokes(&sk, q);
		if (sk.has[q]) {
			struct pcb_stroke pen = {
				0, 0, 0, 0, 0, q, sk.has[q], b->part[q].bottom
			};

			text_strokes(&sk.out, &pen, b->part[q].ref, &sk.at[q]);
		}
	}
	free_texts(&sk);
	free((void *)box);
	free(side);
	free(at);
	free(o);
	free(order);
	free(has);
	out.s = sk.out.s;
	out.n = sk.out.n;
	return out;
}

int pcb_ref_box(const struct pcb_strokes *ss, size_t q, double box[4])
{
	int any = 0;

	for (size_t i = 0; i < ss->n; i++) {
		const struct pcb_stroke *s = &ss->s[i];
		double h = s->w / 2;

		if (s->part != q || !s->ref) {
			continue;
		}
		if (!any) {
			box[0] = box[1] = FAR;
			box[2] = box[3] = -FAR;
		}
		any = 1;
		box[0] = fmin(box[0], fmin(s->x0, s->x1) - h);
		box[1] = fmin(box[1], fmin(s->y0, s->y1) - h);
		box[2] = fmax(box[2], fmax(s->x0, s->x1) + h);
		box[3] = fmax(box[3], fmax(s->y0, s->y1) + h);
	}
	return any;
}

static const char *const sides[] = { "above", "below", "left", "right" };

// The best place a hint found: what it needs, its side, alignment, turn
// and size, and the part most in the way.
struct hint {
	double best;
	int bs, ba, br;
	double bz;
	size_t who;
};

// Searches the places of the designator of part q by its courtyard c,
// as the rule does, for a hint: from 0.8 mm down, the first size with
// room; at each, on each side in order, either way (rot the first),
// in the middle, at the start and the end; with none, what needs least
// at 0.5 mm.  The designators placed are done.
static struct hint hint_search(const struct pcb *b, size_t q,
			       const struct texts *done, const double c[4],
			       const int order[4], int rot)
{
	static const int am[] = { 1, 0, 2 };
	const struct pcb_part *p = &b->part[q];
	struct hint h = { HUGE_VAL, -1, 1, 0, REF_MIN, b->npart };

	for (int z = 0; z < RULE_SIZES && h.best > NO_NEED; z++) {
		h.bs = -1;
		for (int i = 0; i < HINT_TRIES; i++) {
			struct place pl;
			double at[2];
			double t[4];
			double e = NAN;
			size_t wh = 0;
			struct out_side os = { c, order[i / SIDE_TRIES] };

			pl.size = SILK_H - (SILK_STEP * z);
			pl.rot = rot != (i / 3) % 2;
			place_dims(&pl, p->ref);
			ref_at(sides[order[i / SIDE_TRIES]], am[i % 3], c,
			       (struct dims){ pl.w, pl.h }, at);
			pl.x = at[0];
			pl.y = at[1];
			place_box(&pl, t);
			e = text_need(b, q, p->bottom, t, done, &os, &wh);
			if (h.bs < 0 || e < h.best) {
				h.best = e;
				h.bs = order[i / SIDE_TRIES];
				h.ba = am[i % 3];
				h.br = pl.rot;
				h.bz = pl.size;
				h.who = wh;
			}
		}
	}
	return h;
}

// How far courtyard oc is out from courtyard c on side sd.
static double out_by(const double c[4], const double oc[4], int sd)
{
	switch (sd) {
	case 0:
		return oc[1] - c[3];
	case 1:
		return c[1] - oc[3];
	case 2:
		return c[0] - oc[2];
	default:
		return oc[0] - c[2];
	}
}

// The name of what is most in the way of the designator of part q: the
// board edge, its own pads, or a neighbor.
static const char *in_way(const struct pcb *b, size_t q, size_t who)
{
	if (who == b->npart) {
		return "the board edge";
	}
	return who == q ? "its own pads" : b->part[who].name;
}

// Tells the hint h for the designator of part q, by its courtyard c,
// into buf of room len.
static void tell_hint(const struct pcb *b, size_t q, const struct hint *h,
		      const double c[4], char *buf, size_t len)
{
	static const char *const ways[] = { "up", "down", "left", "right" };
	static const char *const aligns[] = { " start", "", " end" };
	const struct pcb_part *p = &b->part[q];
	double best = ceil((h->best * TENTHS) - ROUND_SLACK) / TENTHS;

	if (best == 0) {
		(void)snprintf(buf, len, "; try: ref %s %s%s%s size %.1f",
			       p->name, sides[h->bs], aligns[h->ba],
			       h->br ? " rot 90" : "", h->bz);
	} else if (best <= HINT_MOST) {
		double oc[4];
		double d = NAN;

		pcb_court(&b->part[h->who], oc);
		d = out_by(c, oc, h->bs);
		(void)snprintf(
			buf, len,
			"; try: place %s %s of %s %.1f (%.1f mm more "
			"%s)",
			b->part[h->who].name, ways[h->bs], p->name,
			ceil(((fmax(d, 0) + best) * TENTHS) - ROUND_SLACK) /
				TENTHS,
			best, sides[h->bs]);
	} else {
		(void)snprintf(buf, len,
			       "; nothing within 2 mm: move %s or a neighbor "
			       "(%s)",
			       p->name, in_way(b, q, h->who));
	}
}

void pcb_ref_hint(const struct pcb *b, const struct pcb_strokes *ss, size_t q,
		  char *buf, size_t len)
{
	const struct pcb_part *p = &b->part[q];
	const struct pcb_stmt *rs = ref_of(b, p->name);
	struct texts done = { NULL, NULL, NULL, 0 };
	double c[4];
	double size = rs ? ref_size(rs) : REF_MIN;
	struct dims d = { NAN, NAN };
	struct place pz;
	struct hint h;
	int rot = 0;
	int order[4];
	int k = 0;
	char tried[TRIED_LEN] = "";

	done.box = pcb_alloc((b->npart + 1) * sizeof *done.box);
	done.side = pcb_alloc((b->npart + 1) * sizeof *done.side);
	done.owner = pcb_alloc((b->npart + 1) * sizeof *done.owner);
	pcb_court(p, c);
	rot = rs ? ref_turned(rs) : c[3] - c[1] > c[2] - c[0];
	for (size_t o = 0; o < b->npart; o++) {
		if (o != q && pcb_ref_box(ss, o, done.box[done.n])) {
			done.side[done.n] = b->part[o].bottom;
			done.owner[done.n++] = o;
		}
	}
	// The sides in the order tried: a ref statement's alone, else the
	// long ones first; the search then takes all four.
	for (int i = 0; i < 4; i++) {
		order[i] = rot ? (i + 2) % 4 : i;
	}
	for (int i = 0; rs && i < 4; i++) {
		if (!strcmp(rs->w[2], sides[order[i]])) {
			int t = order[0];

			order[0] = order[i];
			order[i] = t;
		}
	}
	for (int i = 0; i < (rs ? 1 : 4); i++) {
		size_t used = strlen(tried);

		(void)snprintf(tried + used, sizeof tried - used, "%s%s",
			       i ? ", " : "", sides[order[i]]);
	}
	h = hint_search(b, q, &done, c, order, rot);
	pz.size = rs ? size : REF_MIN;
	pz.rot = rot;
	d = ref_dims(p->ref, &pz);
	k = snprintf(buf, len, "; %s is %.2f x %.2f mm at %g (%s %s)", p->ref,
		     d.w, d.h, rs ? size : REF_MIN,
		     rs ? "tried" : "tried 0.8 to 0.5", tried);
	k = k < 0 || (size_t)k >= len ? (int)len - 1 : k;
	tell_hint(b, q, &h, c, buf + k, len - (size_t)k);
	free((void *)done.box);
	free(done.side);
	free(done.owner);
}

// Appends t to s at *k.
static void append(char *s, size_t *k, const char *t)
{
	size_t m = strlen(t);

	memcpy(s + *k, t, m + 1);
	*k += m;
}

char *pcb_by(const struct pcb *b, size_t k, const struct pcb_stmt *st)
{
	char *w = pcb_words(st->w, st->n);
	char *s = NULL;
	size_t len = strlen(w) + BY_SPARE;
	size_t at = 0;
	const struct fp *f = b->part[k].fp;

	for (size_t p = 0; f && p < f->npad; p++) {
		int n = pcb_net_of(b, k, p);

		len += strlen(f->pad[p].name) +
		       strlen(n >= 0 ? b->net[n].name : "-") + 2;
	}
	s = pcb_alloc(len);
	append(s, &at, "nets");
	for (size_t p = 0; f && p < f->npad; p++) {
		int n = pcb_net_of(b, k, p);

		append(s, &at, " ");
		append(s, &at, f->pad[p].name);
		append(s, &at, " ");
		append(s, &at, n >= 0 ? b->net[n].name : "-");
	}
	append(s, &at, " by ");
	append(s, &at, w);
	free(w);
	return s;
}
