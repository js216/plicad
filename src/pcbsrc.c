// SPDX-License-Identifier: GPL-3.0-only
// pcbsrc.c -- Read a board source, its netlist, footprints and lock
// Copyright (c) 2026 Jakob Kastelic

// See pcb(5) for the source and the lock, pcbfp(5) for the footprints
// and netlist(5) for the netlist.  Each fault names its file and line
// and the reading goes on, so one run lists them all.

#define _POSIX_C_SOURCE 200809L // For open_memstream and fmemopen.

#include "pcbsrc.h"
#include "stroke.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>

enum { LINE = 1 << 16, WORDS = 4096 };

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
	p = realloc(p, (n + 1) * size);
	if (!p) {
		pcb_die("out of memory");
	}
	memset((char *)p + n * size, 0, size);
	return p;
}

char *pcb_strdup(const char *s)
{
	char *d = pcb_alloc(strlen(s) + 1);

	strcpy(d, s);
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
		fprintf(stderr, "%s:%d: ", b->file, line);
	} else {
		fprintf(stderr, "%s: ", b->file);
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
	char *s;

	for (int i = 0; i < n; i++) {
		len += strlen(w[i]) + 1;
	}
	s = pcb_alloc(len);
	for (int i = 0; i < n; i++) {
		if (i) {
			strcat(s, " ");
		}
		strcat(s, w[i]);
	}
	return s;
}

static int number(const char *s, double *v)
{
	char *end;

	*v = strtod(s, &end);
	return end != s && !*end && isfinite(*v);
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

// The footprints read so far, by name.
static struct fp **fps;
static size_t nfps;

static FILE *open_in(char **dirs, size_t ndirs, const char *name)
{
	for (size_t i = 0; i < ndirs; i++) {
		char *p = pcb_alloc(strlen(dirs[i]) + strlen(name) + 2);
		FILE *f;

		sprintf(p, "%s/%s", dirs[i], name);
		f = fopen(p, "r");
		free(p);
		if (f) {
			return f;
		}
	}
	return NULL;
}

static struct fp *footprint(struct pcb *b, char **dirs, size_t ndirs,
			    const char *name, int line)
{
	char *buf, **w;
	struct fp *f;
	FILE *in;

	for (size_t i = 0; i < nfps; i++) {
		if (!strcmp(fps[i]->name, name)) {
			return fps[i];
		}
	}
	in = open_in(dirs, ndirs, name);
	buf = pcb_alloc(LINE);
	w = pcb_alloc(WORDS * sizeof *w);
	if (!in) {
		pcb_error(b, line, "no footprint %s", name);
		return NULL;
	}
	f = pcb_alloc(sizeof *f);
	f->name = pcb_strdup(name);
	while (fgets(buf, LINE, in)) {
		int n = split(buf, w);
		struct fp_pad p;
		double v[6];

		if (!n) {
			continue;
		}
		memset(&p, 0, sizeof p);
		if (!strcmp(w[0], "pad") &&
		    (n == 7 || (n == 8 && !strcmp(w[7], "nopaste"))) &&
		    number(w[3], &v[0]) && number(w[4], &v[1]) &&
		    number(w[5], &v[2]) && number(w[6], &v[3])) {
			p.nopaste = n == 8;
			p.shape = !strcmp(w[2], "rect")	 ? PAD_RECT :
				  !strcmp(w[2], "rrect") ? PAD_RRECT :
							   PAD_CIRCLE;
			p.x = v[0];
			p.y = v[1];
			p.w = v[2];
			p.h = v[3];
		} else if (!strcmp(w[0], "hole") &&
			   (n == 6 || (n == 8 && !strcmp(w[6], "keep") &&
				       number(w[7], &p.keep) && p.keep > 0)) &&
			   number(w[2], &v[0]) && number(w[3], &v[1]) &&
			   number(w[4], &v[2]) && number(w[5], &v[3])) {
			p.shape = PAD_CIRCLE;
			p.hole = 1;
			p.x = v[0];
			p.y = v[1];
			p.w = p.h = v[2];
			p.drill = v[3];
		} else if (!strcmp(w[0], "silk") && n >= 6 && n % 2 == 0) {
			struct fp_line ln = { 0, NULL, 0 };
			int ok = number(w[1], &ln.w);

			ln.xy = pcb_alloc((size_t)(n - 2) * sizeof *ln.xy);
			for (int i = 2; ok && i < n; i++) {
				ok = number(w[i], &ln.xy[i - 2]);
			}
			if (!ok) {
				pcb_error(b, line, "%s: a bad silk line", name);
				free(ln.xy);
				continue;
			}
			ln.n = (size_t)(n - 2) / 2;
			f->silk = pcb_grow(f->silk, f->nsilk, sizeof *f->silk);
			f->silk[f->nsilk++] = ln;
			continue;
		} else if (!strcmp(w[0], "court") && n == 5) {
			for (int i = 0; i < 4; i++) {
				if (!number(w[i + 1], &f->court[i])) {
					pcb_error(b, line,
						  "%s: a bad courtyard", name);
				}
			}
			continue;
		} else {
			pcb_error(b, line, "%s: a bad record %s", name, w[0]);
			continue;
		}
		p.name = pcb_strdup(w[1]);
		f->pad = pcb_grow(f->pad, f->npad, sizeof *f->pad);
		f->pad[f->npad++] = p;
	}
	fclose(in);
	free(buf);
	free(w);
	fps = pcb_grow(fps, nfps, sizeof *fps);
	fps[nfps++] = f;
	return f;
}

// The pins of each net as the netlist gives them: part and number,
// until the footprints say which pads they are.
static size_t **pend_part;
static char ***pend_num;
static size_t *pend_n;

// Reads the netlist: the parts and their nets.
static void read_netlist(struct pcb *b, const char *path, int line)
{
	char *buf = pcb_alloc(LINE), **w = pcb_alloc(WORDS * sizeof *w);
	FILE *f = fopen(path, "r");

	if (!f) {
		pcb_error(b, line, "cannot read %s", path);
		free(buf);
		free(w);
		return;
	}
	while (fgets(buf, LINE, f)) {
		int n = split(buf, w);

		if (n >= 3 && !strcmp(w[0], "part")) {
			struct pcb_part q;

			memset(&q, 0, sizeof q);
			q.ref = pcb_strdup(w[1]);
			q.type = pcb_strdup(w[2]);
			q.name = q.ref;
			for (int i = 3; i + 1 < n; i += 2) {
				char *v = pcb_strdup(w[i + 1]);

				if (!strcmp(w[i], "name")) {
					q.name = v;
				} else if (!strcmp(w[i], "value")) {
					q.value = v;
				} else if (!strcmp(w[i], "mpn")) {
					q.mpn = v;
				} else if (!strcmp(w[i], "pins")) {
					q.pins = v;
				} else {
					free(v);
				}
			}
			b->part = pcb_grow(b->part, b->npart, sizeof *b->part);
			b->part[b->npart++] = q;
		} else if (n >= 2 && !strcmp(w[0], "net")) {
			struct pcb_net t;
			size_t k = b->nnet;
			int first = n > 2 && !strcmp(w[2], "global") ? 3 : 2;

			memset(&t, 0, sizeof t);
			t.name = pcb_strdup(w[1]);
			t.plane = -1;
			t.global = first == 3;
			b->net = pcb_grow(b->net, k, sizeof *b->net);
			b->net[b->nnet++] = t;
			pend_part = pcb_grow(pend_part, k, sizeof *pend_part);
			pend_num = pcb_grow(pend_num, k, sizeof *pend_num);
			pend_n = pcb_grow(pend_n, k, sizeof *pend_n);
			pend_part[k] =
				pcb_alloc((size_t)n * sizeof **pend_part);
			pend_num[k] = pcb_alloc((size_t)n * sizeof **pend_num);
			for (int i = first; i < n; i++) {
				char *dot = strrchr(w[i], '.');
				size_t q;

				if (!dot) {
					continue;
				}
				*dot = 0;
				for (q = 0; q < b->npart &&
					    strcmp(b->part[q].ref, w[i]);
				     q++) {
				}
				if (q == b->npart) {
					pcb_error(b, line,
						  "%s: a pin of no part, %s",
						  path, w[i]);
					continue;
				}
				pend_part[k][pend_n[k]] = q;
				pend_num[k][pend_n[k]++] = pcb_strdup(dot + 1);
			}
		}
	}
	fclose(f);
	free(buf);
	free(w);
}

// Gives each net the pads of its pins: every pad of the footprint that
// bears the pin number.
// The parts of each group an unplaced statement names (by its name at a
// dot) are not on the board: no footprint, so no pads, so no copper; a
// net that had pins on them waits for them.
static void leave_unplaced(struct pcb *b)
{
	for (size_t s = 0; s < b->nstmt; s++) {
		const struct pcb_stmt *st = &b->stmt[s];
		size_t n = st->n == 2 ? strlen(st->w[1]) : 0, hit = 0;

		if (strcmp(st->w[0], "unplaced")) {
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
	for (size_t k = 0; k < b->nnet; k++) {
		for (size_t j = 0; j < pend_n[k]; j++) {
			b->net[k].waits |= b->part[pend_part[k][j]].unplaced;
		}
	}
}

static void give_pads(struct pcb *b)
{
	for (size_t k = 0; k < b->nnet; k++) {
		struct pcb_net *t = &b->net[k];

		for (size_t j = 0; j < pend_n[k]; j++) {
			const struct pcb_part *q = &b->part[pend_part[k][j]];

			for (size_t p = 0; q->fp && p < q->fp->npad; p++) {
				if (!strcmp(q->fp->pad[p].name,
					    pend_num[k][j])) {
					t->end = pcb_grow(t->end, t->nend,
							  sizeof *t->end);
					t->end[t->nend].part = pend_part[k][j];
					t->end[t->nend++].pad = p;
				}
			}
			free(pend_num[k][j]);
		}
		free(pend_num[k]);
		free(pend_part[k]);
	}
	free(pend_num);
	free(pend_part);
	free(pend_n);
	// Read again for another lock of the board (a block's).
	pend_num = NULL;
	pend_part = NULL;
	pend_n = NULL;
	b->netby = pcb_alloc((b->nnet + 1) * sizeof *b->netby);
	b->netpad = pcb_alloc((b->nnet + 1) * sizeof *b->netpad);
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

int pcb_pin(const struct pcb *b, const char *s, size_t *part, size_t *pad)
{
	char name[1024], *dot;
	long q;
	const char *num;

	snprintf(name, sizeof name, "%s", s);
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
		static char buf[64];

		while (*p) {
			const char *c = strchr(p, ':'), *e = strchr(p, ',');

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
			*part = (size_t)q;
			*pad = k;
			return 1;
		}
	}
	return 0;
}

static void turn(int rot, double x, double y, double *ox, double *oy)
{
	switch ((rot % 360 + 360) % 360) {
	case 90:
		*ox = -y;
		*oy = x;
		break;
	case 180:
		*ox = -x;
		*oy = -y;
		break;
	case 270:
		*ox = y;
		*oy = -x;
		break;
	default:
		*ox = x;
		*oy = y;
		break;
	}
}

void pcb_spot(const struct pcb *b, const struct pcb_part *q, size_t k,
	      struct pcb_spot *s)
{
	const struct fp_pad *p = &q->fp->pad[k];
	double dx, dy;
	int quarter = (q->rot / 90) % 2;

	turn(q->rot, p->x, p->y, &dx, &dy);
	s->x = q->x + dx;
	s->y = q->y + dy;
	s->w = quarter ? p->h : p->w;
	s->h = quarter ? p->w : p->h;
	s->drill = p->drill;
	s->shape = p->shape;
	s->layer = p->hole ? -1 : q->bottom ? (int)b->nlayer - 1 : 0;
}

int pcb_keepout(const struct pcb *b, const struct pcb_part *q, size_t k,
		double *x, double *y, double *r)
{
	struct pcb_spot s;

	if (!q->fp->pad[k].hole || !(q->fp->pad[k].keep > 0)) {
		return 0;
	}
	pcb_spot(b, q, k, &s);
	*x = s.x;
	*y = s.y;
	*r = q->fp->pad[k].keep / 2;
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
		double x, y, r;

		if (pcb_keepout(b, h, k, &x, &y, &r) &&
		    box_dist(c, x, y) < r - 1e-9) {
			char *key = pad_key(h, k);

			snprintf(why, n,
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
	double c[4], d[4];

	if (!q->placed || !o->placed || !q->fp || !o->fp || q == o) {
		return 0;
	}
	pcb_court(q, c);
	pcb_court(o, d);
	if (q->bottom == o->bottom && c[0] < d[2] - 1e-9 &&
	    d[0] < c[2] - 1e-9 && c[1] < d[3] - 1e-9 && d[1] < c[3] - 1e-9) {
		snprintf(why, n,
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
		double cx, cy, r;

		if (pcb_keepout(b, o, k, &cx, &cy, &r) &&
		    hypot(x - cx, y - cy) < r + rad - 1e-9) {
			char *key = pad_key(o, k);

			snprintf(why, n, "the keep-out of hole %s (%s)", key,
				 pcb_ref(o));
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
	double dx = b[0] - a[0], dy = b[1] - a[1], l = dx * dx + dy * dy, t;

	t = l > 0 ? ((x - a[0]) * dx + (y - a[1]) * dy) / l : 0;
	t = t < 0 ? 0 : t > 1 ? 1 : t;
	at[0] = a[0] + t * dx;
	at[1] = a[1] + t * dy;
	return hypot(x - at[0], y - at[1]);
}

// A shape: a segment of its two ends, or a box of its four corners in
// turn (filled).  The distance from (x, y) to it, 0 inside; its point
// nearest goes to at.
static double shape_near(const double *v, int n, double x, double y,
			 double at[2])
{
	if (n == 4) {
		at[0] = fmin(fmax(x, fmin(v[0], v[4])), fmax(v[0], v[4]));
		at[1] = fmin(fmax(y, fmin(v[1], v[5])), fmax(v[1], v[5]));
		return hypot(x - at[0], y - at[1]);
	}
	return seg_near(x, y, v, v + 2, at);
}

// Whether segments a-b and c-d cross.
static int crosses(const double *a, const double *b, const double *c,
		   const double *d)
{
	double d1 = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
	double d2 = (b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0]);
	double d3 = (d[0] - c[0]) * (a[1] - c[1]) - (d[1] - c[1]) * (a[0] - c[0]);
	double d4 = (d[0] - c[0]) * (b[1] - c[1]) - (d[1] - c[1]) * (b[0] - c[0]);

	return d1 * d2 < 0 && d3 * d4 < 0;
}

// The distance between shapes s (n corners) and t (m corners), 0 where
// they meet: the least from a corner of one to the other, as both are
// convex; the point of s nearest t goes to at.
static double shapes_dist(const double *s, int n, const double *t, int m,
			  double at[2])
{
	double best = HUGE_VAL, p[2];

	for (int i = 0; i < n; i++) {
		double d = shape_near(t, m, s[2 * i], s[2 * i + 1], p);

		if (d < best) {
			best = d;
			at[0] = s[2 * i];
			at[1] = s[2 * i + 1];
		}
	}
	for (int j = 0; j < m; j++) {
		double d = shape_near(s, n, t[2 * j], t[2 * j + 1], p);

		if (d < best) {
			best = d;
			at[0] = p[0];
			at[1] = p[1];
		}
	}
	for (int i = 0; i < (n == 4 ? 4 : 1); i++) {
		for (int j = 0; j < (m == 4 ? 4 : 1); j++) {
			if (crosses(s + 2 * i, s + 2 * ((i + 1) % n), t + 2 * j,
				    t + 2 * ((j + 1) % m))) {
				return 0;
			}
		}
	}
	return best;
}

double pcb_edge_dist(const struct pcb *b, int box, const double s[4], double r,
		     double at[2])
{
	double v[8], p[2], here[2], best = HUGE_VAL;
	int n = box ? 4 : 2;

	at = at ? at : here;
	if (box) {
		const double c[8] = { s[0], s[1], s[2], s[1],
				      s[2], s[3], s[0], s[3] };

		memcpy(v, c, sizeof c);
	} else {
		memcpy(v, s, 4 * sizeof *v);
	}
	// The outline, a rectangle round the copper: from its side nearest a
	// corner or end.
	for (int i = 0; i < n; i++) {
		double x = v[2 * i], y = v[2 * i + 1];
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
		double cr = b->cutr[k], d;
		const double in[8] = { c[0] + cr, c[1] + cr, c[2] - cr,
				       c[1] + cr, c[2] - cr, c[3] - cr,
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
	double x[2], y[2];

	turn(q->rot, q->fp->court[0], q->fp->court[1], &x[0], &y[0]);
	turn(q->rot, q->fp->court[2], q->fp->court[3], &x[1], &y[1]);
	c[0] = q->x + fmin(x[0], x[1]);
	c[1] = q->y + fmin(y[0], y[1]);
	c[2] = q->x + fmax(x[0], x[1]);
	c[3] = q->y + fmax(y[0], y[1]);
}

char *pcb_net_words(const struct pcb *b, size_t k)
{
	char *s = pcb_strdup("");

	for (size_t i = 0; i < b->nstmt; i++) {
		const struct pcb_stmt *st = &b->stmt[i];
		int about = 0;

		if (!strcmp(st->w[0], "path") && st->n >= 2) {
			about = !strcmp(st->w[1], b->net[k].name);
		} else if (!strcmp(st->w[0], "via") && st->n == 3) {
			size_t q, p;

			about = pcb_pin(b, st->w[1], &q, &p) &&
				pcb_net_of(b, q, p) == (int)k;
		}
		if (about) {
			char *w = pcb_words(st->w, st->n), *t;

			t = pcb_alloc(strlen(s) + strlen(w) + 4);
			sprintf(t, "%s%s%s", s, *s ? " ; " : "", w);
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
	char *s = pcb_alloc(strlen(p->name) + strlen(p->fp->pad[k].name) + 32);
	size_t same = 0, nth = 0;

	// A pin of several pads: each by its place among them, j.2/1.
	for (size_t i = 0; i < p->fp->npad; i++) {
		if (!strcmp(p->fp->pad[i].name, p->fp->pad[k].name)) {
			same++;
			nth += i <= k;
		}
	}
	if (same > 1) {
		sprintf(s, "%s.%s/%zu", p->name, p->fp->pad[k].name, nth);
	} else {
		sprintf(s, "%s.%s", p->name, p->fp->pad[k].name);
	}
	return s;
}

char *pcb_pad_key(const struct pcb *b, size_t q, size_t k)
{
	return pad_key(&b->part[q], k);
}

// Whether pin s (PART.PIN) is the pad of key (PART.PAD).
static int pin_is(const struct pcb *b, const char *s, const char *key)
{
	size_t q, p;
	char *k;
	int same;

	if (!pcb_pin(b, s, &q, &p)) {
		return 0;
	}
	k = pcb_pad_key(b, q, p);
	same = !strcmp(k, key);
	free(k);
	return same;
}

double pcb_link_width(const struct pcb *b, int n, const char *a, const char *bb)
{
	for (size_t i = 0; i < b->nstmt; i++) {
		const struct pcb_stmt *st = &b->stmt[i];

		if (!strcmp(st->w[0], "width") && st->n == 4 &&
		    ((pin_is(b, st->w[1], a) && pin_is(b, st->w[2], bb)) ||
		     (pin_is(b, st->w[1], bb) && pin_is(b, st->w[2], a)))) {
			return atof(st->w[3]);
		}
	}
	return b->net[n].width;
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
	char **p = pcb_alloc((b->net[k].nend + 1) * sizeof *p);

	*n = 0;
	for (size_t j = 0; j < b->net[k].nend; j++) {
		const struct pcb_part *q = &b->part[b->net[k].end[j].part];
		size_t pad = b->net[k].end[j].pad;
		struct pcb_spot s;
		char buf[1400], *key;

		if (!q->placed) {
			continue;
		}
		pcb_spot(b, q, pad, &s);
		key = pcb_pad_key(b, b->net[k].end[j].part, pad);
		snprintf(buf, sizeof buf, "%s %.4f %.4f %.4f %.4f %s", key, s.x,
			 s.y, s.w, s.h,
			 s.layer < 0  ? "all" :
			 s.layer == 0 ? "top" :
					"bottom");
		free(key);
		p[(*n)++] = pcb_strdup(buf);
	}
	return p;
}

double pcb_spot_dist(const struct pcb_spot *s, double x, double y)
{
	if (s->shape == PAD_CIRCLE) {
		return fmax(hypot(x - s->x, y - s->y) - s->w / 2, 0);
	}
	return hypot(fmax(fabs(x - s->x) - s->w / 2, 0),
		     fmax(fabs(y - s->y) - s->h / 2, 0));
}

int pcb_pad_ends(const struct pcb_spot *s, double p[2][2])
{
	double lo = fmin(s->w, s->h), hi = fmax(s->w, s->h), u = (hi - lo) / 2;

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

		if (p->hole || p->w * p->h > 1.0 + 1e-9) {
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

double pcb_seg_dist(double x, double y, double ax, double ay, double bx,
		    double by)
{
	double dx = bx - ax, dy = by - ay, l = dx * dx + dy * dy, t;

	t = l > 0 ? ((x - ax) * dx + (y - ay) * dy) / l : 0;
	t = t < 0 ? 0 : t > 1 ? 1 : t;
	return hypot(x - (ax + t * dx), y - (ay + t * dy));
}

// The words of a ref statement: whether they are as pcb(5) says.
static int ref_words(char **w, int n)
{
	int k = 3;

	if (n < 3 || (strcmp(w[2], "above") && strcmp(w[2], "below") &&
		      strcmp(w[2], "left") && strcmp(w[2], "right"))) {
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

		if (*e || h < 0.5 || h > 0.8) {
			return 0;
		}
		k += 2;
	}
	return k == n;
}

// Reads one statement of the source that the reader itself carries out:
// the netlist, the board, layers, rules, footprints and widths.  The
// rest are kept for the programs.
static void statement(struct pcb *b, char **w, int n, int line, char **dirs,
		      size_t ndirs)
{
	double v[4];
	struct pcb_stmt st;

	if (!strcmp(w[0], "board") && n == 6 && !strcmp(w[1], "rect")) {
		for (int i = 0; i < 4; i++) {
			if (!number(w[i + 2], &b->board[i])) {
				pcb_error(b, line, "board rect X0 Y0 X1 Y1");
			}
		}
		return;
	}
	if (!strcmp(w[0], "cutout") && (n == 6 || n == 8) &&
	    !strcmp(w[1], "rect")) {
		size_t k = b->ncut < 8 ? b->ncut : 7;
		double *c = b->cut[k];

		b->cutr[k] = 0;
		for (int i = 0; i < 4; i++) {
			if (!number(w[i + 2], &c[i])) {
				pcb_error(b, line,
					  "cutout rect X0 Y0 X1 Y1 [round R]");
			}
		}
		if (n == 8 &&
		    (strcmp(w[6], "round") || !number(w[7], &b->cutr[k]) ||
		     b->cutr[k] < 0 ||
		     2 * b->cutr[k] > fmin(c[2] - c[0], c[3] - c[1]))) {
			pcb_error(b, line, "cutout rect X0 Y0 X1 Y1 [round R]");
		}
		if (b->ncut == 8) {
			pcb_error(b, line, "more than 8 cutouts");
		}
		b->ncut += b->ncut < 8;
		return;
	}
	if (!strcmp(w[0], "layer") && n >= 3) {
		struct pcb_layer l;

		l.name = pcb_strdup(w[1]);
		l.kind = !strcmp(w[2], "plane") ? LAYER_PLANE : LAYER_SIGNAL;
		l.net = n > 3 ? pcb_strdup(w[3]) : NULL;
		if (strcmp(w[2], "plane") && strcmp(w[2], "signal")) {
			pcb_error(b, line, "a layer is signal or plane NET");
		}
		b->layer = pcb_grow(b->layer, b->nlayer, sizeof *b->layer);
		b->layer[b->nlayer++] = l;
		return;
	}
	if (!strcmp(w[0], "rule") && n >= 3) {
		int ok = 1;

		for (int i = 2; i < n && i < 4; i++) {
			ok &= number(w[i], &v[i - 2]);
		}
		if (!ok && strcmp(w[1], "edge")) {
			pcb_error(b, line, "a rule takes numbers");
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
			for (int i = 3; i < n; i++) {
				if (!strcmp(w[i], "except")) {
					continue;
				}
				b->edge_except =
					pcb_grow(b->edge_except, b->nexcept,
						 sizeof *b->edge_except);
				b->edge_except[b->nexcept++] = pcb_strdup(w[i]);
			}
		} else {
			pcb_error(
				b, line,
				"rule clearance|track|via|viagap|maskweb|edge ...");
		}
		return;
	}
	if (!strcmp(w[0], "netlist") && n == 2) {
		char *path = pcb_alloc(strlen(b->file) + strlen(w[1]) + 2);
		const char *slash = strrchr(b->file, '/');

		if (w[1][0] != '/' && slash) {
			size_t len = (size_t)(slash - b->file);

			memcpy(path, b->file, len);
			path[len] = '/';
			strcpy(path + len + 1, w[1]);
		} else {
			strcpy(path, w[1]);
		}
		read_netlist(b, path, line);
		free(path);
		return;
	}
	if (!strcmp(w[0], "footprint") && n == 3) {
		struct fp *f = footprint(b, dirs, ndirs, w[2], line);
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
			pcb_error(b, line, "no part is a %s", w[1]);
		}
		return;
	}
	if (!strcmp(w[0], "note")) {
		return;
	}
	if (!strcmp(w[0], "ref") && !ref_words(w, n)) {
		pcb_error(b, line,
			  "ref PART above|below|left|right "
			  "[start|middle|end] [rot 90] [size 0.5 to 0.8]");
		return;
	}
	st.w = pcb_alloc((size_t)n * sizeof *st.w);
	for (int i = 0; i < n; i++) {
		st.w[i] = pcb_strdup(w[i]);
	}
	st.n = n;
	st.line = line;
	b->stmt = pcb_grow(b->stmt, b->nstmt, sizeof *b->stmt);
	b->stmt[b->nstmt++] = st;
}

// Reads the lock: parts where they were placed, and copper.
static void read_lock(struct pcb *b, FILE *f)
{
	char *buf = pcb_alloc(LINE), **w = pcb_alloc(WORDS * sizeof *w);
	int ln = 0, link = -1; // The connection the copper read is of.

	while (fgets(buf, LINE, f)) {
		int n = split(buf, w);
		double v[4];
		long k;

		ln++;
		if (!n) {
			continue;
		}
		// The designator (w[2]) is for the reader; the name keys.
		if (!strcmp(w[0], "part") && n >= 7 && number(w[3], &v[0]) &&
		    number(w[4], &v[1]) && number(w[5], &v[2])) {
			long q = part_named(b, w[1]);

			if (q < 0 || !b->part[q].fp) {
				continue; // Stale: told by pcbcheck.
			}
			b->part[q].x = v[0];
			b->part[q].y = v[1];
			b->part[q].rot = (int)v[2];
			b->part[q].bottom = !strcmp(w[6], "bottom");
			b->part[q].placed = 1;
			b->part[q].by = n > 7 ? pcb_words(w + 7, n - 7) :
						pcb_strdup("");
		} else if (!strcmp(w[0], "net") && n >= 3 &&
			   !strcmp(w[2], "by")) {
			k = net_named(b, w[1]);
			link = -1;
			if (k >= 0) {
				b->netby[k] = pcb_words(w + 3, n - 3);
			}
		} else if (!strcmp(w[0], "link") && n == 5 &&
			   number(w[4], &v[0])) {
			k = net_named(b, w[1]);
			link = k >= 0 ? pcb_add_link(b, (int)k, w[2], w[3],
						     v[0]) :
					-1;
		} else if (!strcmp(w[0], "pad") && n == 9) {
			// pad NET PART.PIN REF X Y W H LAYER, the REF left out.
			k = net_named(b, w[1]);
			if (k >= 0) {
				char *t[6] = { w[2], w[4], w[5],
					       w[6], w[7], w[8] };

				b->netpad[k] = pcb_grow(b->netpad[k],
							b->nnetpad[k],
							sizeof *b->netpad[k]);
				b->netpad[k][b->nnetpad[k]++] = pcb_words(t, 6);
			}
		} else if (!strcmp(w[0], "track") && n >= 8 &&
			   (n - 4) % 2 == 0) {
			struct pcb_track t;

			k = net_named(b, w[1]);
			if (k < 0 || pcb_layer(b, w[2]) < 0 ||
			    !number(w[3], &v[0])) {
				continue;
			}
			t.net = (int)k;
			t.link = link >= 0 && b->link[link].net == k ? link :
								       -1;
			t.layer = pcb_layer(b, w[2]);
			t.w = v[0];
			t.n = (size_t)(n - 4) / 2;
			t.xy = pcb_alloc(2 * t.n * sizeof *t.xy);
			for (size_t i = 0; i < 2 * t.n; i++) {
				t.xy[i] = atof(w[4 + i]);
			}
			b->track_ = pcb_grow(b->track_, b->ntrack,
					     sizeof *b->track_);
			b->track_[b->ntrack++] = t;
		} else if (!strcmp(w[0], "via") && n == 6) {
			struct pcb_via vi;

			k = net_named(b, w[1]);
			if (k < 0) {
				continue;
			}
			vi.net = (int)k;
			vi.link = link >= 0 && b->link[link].net == k ? link :
									-1;
			vi.x = atof(w[2]);
			vi.y = atof(w[3]);
			vi.dia = atof(w[4]);
			vi.drill = atof(w[5]);
			b->via = pcb_grow(b->via, b->nvia, sizeof *b->via);
			b->via[b->nvia++] = vi;
		}
	}
	free(buf);
	free(w);
}

// The store of the section of a name, made if new.
static size_t store_of(struct pcb *b, const char *name)
{
	for (size_t i = 0; i < b->nsec; i++) {
		if (!strcmp(b->sec_name[i], name)) {
			return i;
		}
	}
	b->sec_name = pcb_grow(b->sec_name, b->nsec, sizeof *b->sec_name);
	b->sec_text = pcb_grow(b->sec_text, b->nsec, sizeof *b->sec_text);
	b->sec_name[b->nsec] = pcb_strdup(name);
	b->sec_text[b->nsec] = pcb_strdup("");
	return b->nsec++;
}

// Splits the lock into its sections: the group's (the board's, "",
// for none) goes to the stream returned, the rest to the stores.
static FILE *sections(struct pcb *b, FILE *lock, const char *group, char **text,
		      size_t *len)
{
	char *line = pcb_alloc(LINE), name[1024] = "";
	FILE *mine = open_memstream(text, len);

	if (!mine) {
		pcb_die("out of memory");
	}
	while (fgets(line, LINE, lock)) {
		size_t k;
		char *t;

		if (sscanf(line, "group %1023s", name) == 1 ||
		    !strncmp(line, "# lock of ", 10)) {
			continue; // A section begins; the header is written anew.
		}
		if (!strcmp(name, group ? group : "")) {
			fputs(line, mine);
			continue;
		}
		k = store_of(b, name);
		t = pcb_alloc(strlen(b->sec_text[k]) + strlen(line) + 1);
		strcpy(t, b->sec_text[k]);
		strcat(t, line);
		free(b->sec_text[k]);
		b->sec_text[k] = t;
	}
	free(line);
	fclose(mine);
	return fmemopen(*text, *len, "r");
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

static int deeper_first(const void *x, const void *y)
{
	const char *a = *(char *const *)x, *b = *(char *const *)y;
	size_t da = depth(a), db = depth(b);

	return da != db ? (da < db) - (da > db) : strcmp(a, b);
}

int pcb_passes(const char *file, char **lib, size_t nlib, const char *group,
	       FILE *in, FILE *out, pcb_pass *pass)
{
	struct pcb src;
	char **groups = NULL, *text = NULL;
	size_t ng = 0, len = 0;
	int failed = 0;

	if (group) {
		return pass(file, lib, nlib, group, in, out);
	}
	pcb_read(&src, file, lib, nlib, NULL, NULL);
	for (size_t i = 0; i < src.nstmt; i++) {
		if (!strcmp(src.stmt[i].w[0], "block") && src.stmt[i].n == 2) {
			groups = pcb_grow(groups, ng, sizeof *groups);
			groups[ng++] = src.stmt[i].w[1];
		}
	}
	if (ng) {
		qsort(groups, ng, sizeof *groups, deeper_first);
	}
	for (size_t i = 0; i <= ng; i++) {
		char *next = NULL;
		size_t nlen = 0;
		FILE *mid = i < ng ? open_memstream(&next, &nlen) : out;
		FILE *prev = !i ? in : len ? fmemopen(text, len, "r") : NULL;

		if (!mid || (i && len && !prev)) {
			pcb_die("out of memory");
		}
		failed |= pass(file, lib, nlib, i < ng ? groups[i] : NULL, prev,
			       mid);
		if (i && prev) {
			fclose(prev);
		}
		if (i < ng) {
			fclose(mid);
		}
		free(text);
		text = next;
		len = nlen;
	}
	free(text);
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

void pcb_read(struct pcb *b, const char *file, char **lib, size_t nlib,
	      FILE *lock, const char *group)
{
	char *buf = pcb_alloc(LINE), **w = pcb_alloc(WORDS * sizeof *w);
	char **dirs = pcb_alloc((nlib + 1) * sizeof *dirs);
	const char *slash = strrchr(file, '/');
	FILE *f = fopen(file, "r");
	int ln = 0;

	memset(b, 0, sizeof *b);
	b->file = file;
	b->group = group;
	b->clearance = 0.2;
	b->track = 0.2;
	b->via_dia = 0.6;
	b->via_drill = 0.3;
	b->edge = 0.5;
	b->viagap = 0.3;
	b->maskweb = 0.1;
	// The project's own footprints come first: those next to the source.
	dirs[0] = pcb_alloc(strlen(file) + 16);
	memcpy(dirs[0], slash ? file : ".", slash ? (size_t)(slash - file) : 1);
	strcat(dirs[0], "/footprints");
	if (nlib) {
		memcpy(dirs + 1, lib, nlib * sizeof *lib);
	}
	if (!f) {
		pcb_die("%s: cannot read", file);
	}
	while (fgets(buf, LINE, f)) {
		int n = split(buf, w);

		ln++;
		if (n) {
			statement(b, w, n, ln, dirs, nlib + 1);
		}
	}
	fclose(f);
	leave_unplaced(b);
	for (size_t i = 0; i < b->npart; i++) {
		if (!b->part[i].fp && !b->part[i].unplaced) {
			pcb_error(b, 0, "%s (%s, %s) has no footprint",
				  b->part[i].name, pcb_ref(&b->part[i]),
				  b->part[i].mpn ? b->part[i].mpn :
						   b->part[i].type);
		}
	}
	give_pads(b);
	for (size_t k = 0; k < b->nnet; k++) {
		b->net[k].width = b->track;
		for (size_t i = 0; i < b->nlayer; i++) {
			if (b->layer[i].kind == LAYER_PLANE &&
			    b->layer[i].net &&
			    !strcmp(b->layer[i].net, b->net[k].name)) {
				b->net[k].plane = (int)i;
			}
		}
	}
	for (size_t i = 0; i < b->nstmt; i++) {
		const struct pcb_stmt *st = &b->stmt[i];
		double v;
		long k;

		if (strcmp(st->w[0], "width")) {
			continue;
		}
		if (st->n == 4) {
			size_t q[2], p[2];

			// The copper between two pins of one net.
			if (!pcb_pin(b, st->w[1], &q[0], &p[0]) ||
			    !pcb_pin(b, st->w[2], &q[1], &p[1]) ||
			    pcb_net_of(b, q[0], p[0]) < 0 ||
			    pcb_net_of(b, q[0], p[0]) !=
				    pcb_net_of(b, q[1], p[1]) ||
			    !number(st->w[3], &v)) {
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
	if (lock) {
		char *text = NULL;
		size_t len = 0;
		FILE *mine = sections(b, lock, group, &text, &len);

		if (mine) {
			read_lock(b, mine);
			fclose(mine);
		}
		free(text);
	}
	free(buf);
	free(w);
	free(dirs[0]);
	free(dirs);
}

void pcb_free(struct pcb *b)
{
	for (size_t i = 0; i < b->nexcept; i++) {
		free(b->edge_except[i]);
	}
	free(b->edge_except);
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
		free(b->net[k].end);
		if (b->netby) {
			free(b->netby[k]);
		}
		for (size_t j = 0; b->netpad && j < b->nnetpad[k]; j++) {
			free(b->netpad[k][j]);
		}
		if (b->netpad) {
			free(b->netpad[k]);
		}
	}
	free(b->net);
	free(b->netby);
	free(b->netpad);
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
		free(b->stmt[i].w);
	}
	free(b->stmt);
	for (size_t i = 0; i < b->nsec; i++) {
		free(b->sec_name[i]);
		free(b->sec_text[i]);
	}
	free(b->sec_name);
	free(b->sec_text);
	memset(b, 0, sizeof *b);
}

// The parts or the nets in the order of their names, so a lock lists
// them the same way whatever the netlist's order.
static const struct pcb *order_b;
static int order_parts;

static int by_name(const void *x, const void *y)
{
	size_t i = *(const size_t *)x, j = *(const size_t *)y;

	if (order_parts) {
		return strcmp(order_b->part[i].name, order_b->part[j].name);
	}
	return strcmp(order_b->net[i].name, order_b->net[j].name);
}

static size_t *sorted(const struct pcb *b, int parts)
{
	size_t n = parts ? b->npart : b->nnet,
	       *o = pcb_alloc((n + 1) * sizeof *o);

	for (size_t i = 0; i < n; i++) {
		o[i] = i;
	}
	order_b = b;
	order_parts = parts;
	qsort(o, n, sizeof *o, by_name);
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
		fprintf(f, "track %s %s %.4f", b->net[k].name,
			b->layer[t->layer].name, t->w);
		for (size_t j = 0; j < t->n; j++) {
			fprintf(f, " %.4f %.4f", t->xy[2 * j],
				t->xy[2 * j + 1]);
		}
		fprintf(f, "\n");
	}
	for (size_t i = 0; i < b->nvia; i++) {
		const struct pcb_via *v = &b->via[i];

		if (v->net == k && v->link == l) {
			fprintf(f, "via %s %.4f %.4f %.4f %.4f\n",
				b->net[k].name, v->x, v->y, v->dia, v->drill);
		}
	}
}

static int by_text(const void *x, const void *y)
{
	return strcmp(*(char *const *)x, *(char *const *)y);
}

// Writes the section read, made anew.
static void write_section(const struct pcb *b, FILE *f);

void pcb_write_lock(const struct pcb *b, FILE *f)
{
	const char **names = pcb_alloc((b->nsec + 2) * sizeof *names);
	size_t n = 0;
	int mine_done = 0;

	fprintf(f, "# lock of %s: made by the programs; edit the source\n",
		strrchr(b->file, '/') ? strrchr(b->file, '/') + 1 : b->file);
	for (size_t i = 0; i < b->nsec; i++) {
		names[n++] = b->sec_name[i];
	}
	names[n++] = b->group ? b->group : "";
	qsort(names, n, sizeof *names, by_text);
	for (size_t i = 0; i < n; i++) {
		int mine = !strcmp(names[i], b->group ? b->group : "");
		const char *t = pcb_section(b, names[i]);

		if (i && !strcmp(names[i], names[i - 1])) {
			continue;
		}
		if (*names[i]) {
			fprintf(f, "group %s\n", names[i]);
		}
		if (mine && !mine_done) {
			write_section(b, f);
			mine_done = 1;
		} else if (t) {
			fputs(t, f);
		}
	}
	free(names);
}

static void write_section(const struct pcb *b, FILE *f)
{
	size_t *po = sorted(b, 1), *no = sorted(b, 0);

	for (size_t o = 0; o < b->npart; o++) {
		const struct pcb_part *q = &b->part[po[o]];

		if (q->placed) {
			fprintf(f, "part %s %s %.4f %.4f %d %s %s\n", q->name,
				pcb_ref(q), q->x, q->y, q->rot,
				q->bottom ? "bottom" : "top",
				q->by ? q->by : "by");
		}
	}
	for (size_t o = 0; o < b->nnet; o++) {
		size_t k = no[o];

		if (!b->netby[k]) {
			continue;
		}
		fprintf(f, "net %s by%s%s\n", b->net[k].name,
			*b->netby[k] ? " " : "", b->netby[k]);
		for (size_t i = 0; i < b->nnetpad[k]; i++) {
			const char *p = b->netpad[k][i], *sp = strchr(p, ' ');
			char *part;
			long q;

			if (!sp) {
				continue;
			}
			part = pcb_strdup(p);
			// The pad by its part and name, then its part's designator.
			part[sp - p] = 0;
			if (strrchr(part, '.')) {
				*strrchr(part, '.') = 0;
			}
			q = part_named(b, part);
			free(part);
			fprintf(f, "pad %s %.*s %s%s\n", b->net[k].name,
				(int)(sp - p), p,
				q >= 0 ? pcb_ref(&b->part[q]) : "-", sp);
		}
		for (long l = -1; l < (long)b->nlink; l++) {
			if (l >= 0 && b->link[l].net != (int)k) {
				continue;
			}
			if (l >= 0) {
				fprintf(f, "link %s %s %s %.4f\n",
					b->net[k].name, b->link[l].a,
					b->link[l].b, b->link[l].w);
			}
			write_copper(b, f, (int)k, (int)l);
		}
	}
	free(po);
	free(no);
}

int pcb_opening(const struct pcb *b, size_t q, size_t k, int bottom,
		double box[4])
{
	struct pcb_spot s;
	double g = 0.05; // The mask's growth round a pad.

	pcb_spot(b, &b->part[q], k, &s);
	if (s.layer >= 0 && (s.layer != 0) != bottom) {
		return 0;
	}
	box[0] = s.x - s.w / 2 - g;
	box[1] = s.y - s.h / 2 - g;
	box[2] = s.x + s.w / 2 + g;
	box[3] = s.y + s.h / 2 + g;
	return 1;
}

#define SILK_H 0.8 // The height of a designator.
#define SILK_GAP 0.15 // From the courtyard to the designator.

static int overlap(const double a[4], const double c[4])
{
	return a[0] < c[2] && c[0] < a[2] && a[1] < c[3] && c[1] < a[3];
}

// Whether a designator's box is clear: of the openings and of the
// courtyards of the other parts of its side, of the designators of its
// side before it, and inside the board.
static int text_clear(const struct pcb *b, size_t q, const double t[4],
		      double (*done)[4], const int *side, size_t ndone)
{
	int bottom = b->part[q].bottom;

	if (t[0] < b->board[0] + b->edge || t[1] < b->board[1] + b->edge ||
	    t[2] > b->board[2] - b->edge || t[3] > b->board[3] - b->edge) {
		return 0;
	}
	for (size_t i = 0; i < ndone; i++) {
		if (side[i] == bottom && overlap(t, done[i])) {
			return 0;
		}
	}
	for (size_t p = 0; p < b->npart; p++) {
		const struct pcb_part *o = &b->part[p];
		double c[4];

		if (!o->placed || !o->fp) {
			continue;
		}
		pcb_court(o, c);
		if (p != q && o->bottom == bottom && overlap(t, c)) {
			return 0;
		}
		for (size_t k = 0; k < o->fp->npad; k++) {
			if (pcb_opening(b, p, k, bottom, c) && overlap(t, c)) {
				return 0;
			}
		}
	}
	return 1;
}

static void stroke(struct pcb_stroke **s, size_t *n, size_t q, double w,
		   const double xy[4], int ref)
{
	*s = pcb_grow(*s, *n, sizeof **s);
	(*s)[*n].ref = ref;
	(*s)[*n].w = w;
	(*s)[*n].x0 = xy[0];
	(*s)[*n].y0 = xy[1];
	(*s)[*n].x1 = xy[2];
	(*s)[*n].y1 = xy[3];
	(*s)[(*n)++].part = q;
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

// The lower left corner of a designator's box, bw by bh, where its ref
// statement puts it by the courtyard c: on the side, at the gap, from
// the start of the side (its left or top end), its middle or its end.
static void ref_at(const struct pcb_stmt *rs, const double c[4], double bw,
		   double bh, double at[2])
{
	const char *sd = rs->w[2];
	const char *al = rs->n > 3 && strcmp(rs->w[3], "rot") &&
					 strcmp(rs->w[3], "size") ?
				 rs->w[3] :
				 "middle";
	int across = !strcmp(sd, "above") || !strcmp(sd, "below");
	int a = !strcmp(al, "start") ? 0 : !strcmp(al, "end") ? 2 : 1;

	if (across) {
		at[0] = a == 0 ? c[0] :
			a == 2 ? c[2] - bw :
				 (c[0] + c[2] - bw) / 2;
		at[1] = !strcmp(sd, "above") ? c[3] + SILK_GAP :
					       c[1] - SILK_GAP - bh;
	} else {
		at[0] = !strcmp(sd, "left") ? c[0] - SILK_GAP - bw :
					      c[2] + SILK_GAP;
		at[1] = a == 0 ? c[3] - bh :
			a == 2 ? c[1] :
				 (c[1] + c[3] - bh) / 2;
	}
}

size_t pcb_silk(const struct pcb *b, struct pcb_stroke **s)
{
	double(*done)[4] = pcb_alloc((b->npart + 1) * sizeof *done);
	int *side = pcb_alloc((b->npart + 1) * sizeof *side);
	size_t n = 0, ndone = 0;

	*s = NULL;
	for (size_t q = 0; q < b->npart; q++) {
		const struct pcb_part *p = &b->part[q];
		const char *ref = p->ref ? p->ref : "";
		const struct pcb_stmt *rs = ref_of(b, p->name);
		double h = rs ? ref_size(rs) : SILK_H, pen = STROKE_PEN * h;
		double c[4], t[4], tw = stroke_width(ref, h);
		double dn = stroke_descends(ref) ? h / 3 : 0; // The descender.
		double at[12][2], bw = tw, bh = h + dn;
		int pick = 0, rot = 0;

		if (!p->placed || !p->fp) {
			continue;
		}
		for (size_t i = 0; i < p->fp->nsilk; i++) {
			const struct fp_line *ln = &p->fp->silk[i];

			for (size_t j = 0; j + 1 < ln->n; j++) {
				double xy[4];

				turn(p->rot, ln->xy[2 * j], ln->xy[2 * j + 1],
				     &xy[0], &xy[1]);
				turn(p->rot, ln->xy[2 * j + 2],
				     ln->xy[2 * j + 3], &xy[2], &xy[3]);
				xy[0] += p->x;
				xy[1] += p->y;
				xy[2] += p->x;
				xy[3] += p->y;
				stroke(s, &n, q, ln->w, xy, 0);
			}
		}
		if (!*ref) {
			continue;
		}
		// The lower left corners of the places: the four sides, then
		// the four 1 mm and 2 mm farther out.
		pcb_court(p, c);
		if (rs) {
			rot = ref_turned(rs);
			bw = rot ? h + dn : tw;
			bh = rot ? tw : h + dn;
			ref_at(rs, c, bw, bh, at[0]);
		}
		for (int i = 0; !rs && i < 12; i++) {
			double g = SILK_GAP + i / 4;

			at[i][0] = i % 4 < 2  ? (c[0] + c[2] - tw) / 2 :
				   i % 4 == 2 ? c[0] - g - tw :
						c[2] + g;
			at[i][1] = i % 4 == 0 ? c[3] + g :
				   i % 4 == 1 ? c[1] - g - bh :
						(c[1] + c[3] - bh) / 2;
		}
		for (int i = 11; !rs && i >= 0; i--) {
			t[0] = at[i][0] - pen;
			t[1] = at[i][1] - pen;
			t[2] = at[i][0] + tw + pen;
			t[3] = at[i][1] + bh + pen;
			if (text_clear(b, q, t, done, side, ndone)) {
				pick = i;
			}
		}
		t[0] = at[pick][0] - pen;
		t[1] = at[pick][1] - pen;
		t[2] = at[pick][0] + bw + pen;
		t[3] = at[pick][1] + bh + pen;
		memcpy(done[ndone], t, sizeof t);
		side[ndone++] = p->bottom;
		{
			double *sg;
			size_t ns = stroke_text(ref, h, &sg);

			for (size_t k = 0; k < ns; k++) {
				double xy[4];

				// From the box's lower left, the baseline a
				// descender up.
				for (size_t e = 0; e < 4; e += 2) {
					double lx = sg[4 * k + e];
					double ly = sg[4 * k + e + 1] + dn;

					// Turned, it reads upward in its box.
					xy[e] = at[pick][0] + (rot ? bw - ly : lx);
					xy[e + 1] = at[pick][1] + (rot ? lx : ly);
					if (p->bottom) {
						xy[e] = 2 * at[pick][0] + bw - xy[e];
					}
				}
				stroke(s, &n, q, pen, xy, 1);
			}
			free(sg);
		}
	}
	free(done);
	free(side);
	return n;
}

char *pcb_by(const struct pcb *b, size_t k, const struct pcb_stmt *st)
{
	char *w = pcb_words(st->w, st->n), *s;
	size_t len = strlen(w) + 16;
	const struct fp *f = b->part[k].fp;

	for (size_t p = 0; f && p < f->npad; p++) {
		int n = pcb_net_of(b, k, p);

		len += strlen(f->pad[p].name) +
		       strlen(n >= 0 ? b->net[n].name : "-") + 2;
	}
	s = pcb_alloc(len);
	strcpy(s, "nets");
	for (size_t p = 0; f && p < f->npad; p++) {
		int n = pcb_net_of(b, k, p);

		strcat(s, " ");
		strcat(s, f->pad[p].name);
		strcat(s, " ");
		strcat(s, n >= 0 ? b->net[n].name : "-");
	}
	strcat(s, " by ");
	strcat(s, w);
	free(w);
	return s;
}
