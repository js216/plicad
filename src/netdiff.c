// SPDX-License-Identifier: GPL-3.0-only
// netdiff.c -- Compare two netlists by their circuits, not their names
// Copyright (c) 2026 Jakob Kastelic

// Usage: netdiff a.nl b.nl
//
// Reads two netlists (netlist(5)) and matches the parts of one to those
// of the other by their type, value, rating and part number and by how
// they are connected; designators and the names of nets play no part,
// but a global net matches only the global net of its name, and the
// class of a net plays no part.  Then
// prints each part and each pin of a net that differs, one per line; no
// output means the circuits are the same.  A net of one pin, unless it
// is global, is no connection.
//
// The match refines labels, as in the method of Weisfeiler and Lehman.
// A part starts with a hash of its type and values; a net takes the sum
// of a hash of each of its pins (the label of the part and the class of
// the pin: the pins of a swap list are one class); a part takes its own
// label and the sum for its pins of a hash of the class and the label of
// the net.  Sums make the labels of a set the same in any order.  The
// passes go on while the labels split.  Then a label that one unmatched
// part holds on each side pairs them; if there is none, one pair is taken
// from the label most held on both sides (alike parts, as in parallel).
// A pair gets a label of its own and the refining goes on from there.
// Where the circuits differ, the labels differ all the way out from the
// difference; then the labels are made again from the values and the
// pairs with 3, 2, 1 and no passes, which see less far, and a label
// unique on each side at the deepest of these pairs its parts.  Parts
// left unmatched are paired by their values where most of their
// pins lie on nets that go with each other, or reported.  Each pass is
// linear in the pins, so a board of thousands of parts takes a moment.

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct pin {
	char *num;
	uint64_t cls; // The class of the pin: its number, or one swap list.
	int net, swap;
};

struct part {
	char *ref, *show; // The designator; the record, without its name.
	uint64_t sig, label, own; // own: the label of its pair, once matched.
	struct pin *pin; // The pins on nets.
	size_t npin;
	int mate;
	char *swap; // The swap list, as ",1,2,".
};

struct net {
	char *name;
	int global, *part, *pin; // The part and the index of its pin.
	size_t n;
	uint64_t label;
};

struct list {
	const char *file;
	struct part *part;
	size_t npart;
	struct net *net;
	size_t nnet;
};

enum {
	NUM_LEN = 64, // Room for the number of a value.
	LINE_START = 256, // The first room for a line,
	WORDS_START = 64, // and for its words.
	SIG_LEN = 1024, // Room for the values of a part, joined.
	VALUE_LEN = 256, // Room for one value, normalized.
	SHOW_LEN = 4096, // Room for a part record as shown.
	PASSES_MAX = 100, // The most passes of refining until it settles.
	SCORE_NUMBER = 8 // The score of a pin of the same number.
};
#define MICRO 1e-6 // The micro sign as a prefix.

// The constants of the finalizer of SplitMix64,
static const unsigned mix_shift1 = 30;
static const uint64_t mix_mul1 = 0xbf58476d1ce4e5b9U;
static const unsigned mix_shift2 = 27;
static const uint64_t mix_mul2 = 0x94d049bb133111ebU;
static const unsigned mix_shift3 = 31;
// of the 64-bit hash of Fowler, Noll and Vo,
static const uint64_t fnv_offset = 1469598103934665603U;
static const uint64_t fnv_prime = 1099511628211U;
// and of the labels: the factor of a label beside the class of a pin,
// of a class beside the label of a net, and the salt of a net.
static const uint64_t label_mul = 31;
static const uint64_t class_mul = 17;
static const uint64_t net_salt = 0x5bd1e995U;

static void die(const char *msg, const char *file, unsigned line)
{
	(void)fflush(stdout);
	if (line) {
		(void)fprintf(stderr, "%s:%u: %s\n", file, line, msg);
	} else {
		(void)fprintf(stderr, "%s: %s\n", file, msg);
	}
	exit(2);
}

static void *grow(void *p, size_t n, size_t size)
{
	void *q = realloc(p, (n + 1) * size);

	if (!q) {
		die("out of memory", "netdiff", 0);
	}
	return q;
}

static void *zalloc(size_t n)
{
	void *p = calloc(1, n ? n : 1);

	if (!p) {
		die("out of memory", "netdiff", 0);
	}
	return p;
}

static char *dup(const char *s)
{
	size_t n = strlen(s) + 1;
	char *d = zalloc(n);

	memcpy(d, s, n);
	return d;
}

// A strong mix of 64 bits (the finalizer of SplitMix64).
static uint64_t mix(uint64_t x)
{
	x ^= x >> mix_shift1;
	x *= mix_mul1;
	x ^= x >> mix_shift2;
	x *= mix_mul2;
	x ^= x >> mix_shift3;
	return x;
}

static uint64_t hash(const char *s)
{
	uint64_t h = fnv_offset;

	for (; *s; s++) {
		h = (h ^ (unsigned char)*s) * fnv_prime;
	}
	return mix(h);
}

// A value in one form: the number in %g with its unit, so that 499R,
// 499 and 499 with the ohm sign agree, 1.00K and 1k, 0.1uF (with u or
// mu) and 0.1U, 100nF, 100n and 100N, 120pF and 120P; ohms, farads and
// henries go without saying.  A
// text that is no number is kept, in lower case.  Returns 0 if the
// value does not fit the n bytes of out.
static int norm_value(const char *s, char *out, size_t n)
{
	static const char pre[] = "pnumkKMGPNU";
	static const double mul[] = { 1e-12, 1e-9, 1e-6,  1e-3, 1e3, 1e3,
				      1e6,   1e9,  1e-12, 1e-9, 1e-6 };
	char num[NUM_LEN];
	size_t k = 0;
	double m = 1;
	const char *p = s;
	const char *unit = NULL;
	int len = 0;

	while (*p && k < sizeof num - 1) {
		const char *q = strchr(pre, *p);

		if (isdigit((unsigned char)*p) || *p == '.') {
			num[k++] = *p++;
			continue;
		}
		if (!strncmp(p, "\xce\xbc", 2) || !strncmp(p, "\xc2\xb5", 2)) {
			m = MICRO;
			p += 2;
		} else if (q != NULL) {
			m = mul[q - pre];
			p++;
		} else if (*p == 'R' || *p == 'r') {
			p++;
		} else {
			break;
		}
		// A prefix in the middle stands for the point, as in 4k7.
		if (isdigit((unsigned char)*p) && !strchr(num, '.')) {
			num[k++] = '.';
		} else {
			break;
		}
	}
	num[k] = 0;
	unit = p;
	if (!k || !strpbrk(num, "0123456789")) {
		size_t i = 0;

		for (; i < n - 1 && s[i]; i++) {
			out[i] = (char)tolower((unsigned char)s[i]);
		}
		out[i] = 0;
		return 1;
	}
	if (!strcmp(unit, "\xce\xa9") || !strcmp(unit, "ohm") ||
	    !strcmp(unit, "R") || !strcmp(unit, "F") || !strcmp(unit, "H")) {
		unit = "";
	}
	len = snprintf(out, n, "%.12g%s", strtod(num, NULL) * m, unit);
	return len >= 0 && (size_t)len < n;
}

// Reads a line of any length into *buf; returns 0 at the end.
static int read_line(FILE *f, char **buf, size_t *cap)
{
	size_t n = 0;
	int c = 0;

	if (!*buf) {
		*cap = LINE_START;
		*buf = zalloc(*cap);
	}
	while ((c = getc(f)) != EOF && c != '\n') {
		if (n + 2 > *cap) {
			char *more = NULL;

			*cap *= 2;
			more = realloc(*buf, *cap);
			if (!more) {
				die("out of memory", "netdiff", 0);
			}
			*buf = more;
		}
		(*buf)[n++] = (char)c;
	}
	(*buf)[n] = 0;
	return c != EOF || n;
}

// Splits s into words at blanks, up to a #; returns their number.
static size_t split(char *s, char ***w, size_t *cap)
{
	size_t n = 0;

	for (;;) {
		while (*s && isspace((unsigned char)*s)) {
			s++;
		}
		if (!*s || *s == '#') {
			return n;
		}
		if (n + 1 > *cap) {
			char **more = NULL;

			*cap = *cap ? 2 * *cap : WORDS_START;
			more = (char **)realloc((void *)*w, *cap * sizeof **w);
			if (!more) {
				die("out of memory", "netdiff", 0);
			}
			*w = more;
		}
		(*w)[n++] = s;
		while (*s && !isspace((unsigned char)*s)) {
			s++;
		}
		if (*s) {
			*s++ = 0;
		}
	}
}

static int by_ref(const void *lhs, const void *rhs)
{
	return strcmp(((const struct part *)lhs)->ref,
		      ((const struct part *)rhs)->ref);
}

// The part of the designator, by bisection of the sorted parts, or -1.
static long find_part(const struct list *l, const char *ref)
{
	size_t lo = 0;
	size_t hi = l->npart;

	while (lo < hi) {
		size_t mid = (lo + hi) / 2;
		int c = strcmp(l->part[mid].ref, ref);

		if (!c) {
			return (long)mid;
		}
		if (c < 0) {
			lo = mid + 1;
		} else {
			hi = mid;
		}
	}
	return -1;
}

static int live(const struct net *t)
{
	return t->n >= 2 || (t->global && t->n >= 1);
}

// Reads a part record into p.
static void read_part(struct part *p, char **w, size_t n, const char *file,
		      unsigned ln)
{
	char sig[SIG_LEN];
	char v[VALUE_LEN] = "";
	char r[VALUE_LEN] = "";
	char mpn[VALUE_LEN] = "";
	char show[SHOW_LEN];
	size_t len = 0;
	int fits = 1;

	memset(p, 0, sizeof *p);
	p->ref = dup(w[1]);
	p->mate = -1;
	p->swap = dup("");
	show[0] = 0;
	for (size_t i = 2; i < n; i++) {
		if (i % 2 == 1) {
			const char *k = w[i];
			const char *x = w[i + 1];

			if (!strcmp(k, "name") || !strcmp(k, "pins")) {
				i++;
				continue;
			}
			if (!strcmp(k, "value")) {
				fits = fits && norm_value(x, v, sizeof v);
			} else if (!strcmp(k, "rating")) {
				fits = fits && norm_value(x, r, sizeof r);
			} else if (!strcmp(k, "mpn")) {
				fits = fits && strlen(x) < sizeof mpn;
				(void)snprintf(mpn, sizeof mpn, "%s", x);
			} else if (!strcmp(k, "swap")) {
				size_t room = strlen(x) + 3;

				free(p->swap);
				p->swap = zalloc(room);
				(void)snprintf(p->swap, room, ",%s,", x);
			} else {
				die("unknown word in a part record", file, ln);
			}
		}
		if (len + strlen(w[i]) + 2 < sizeof show) {
			len += (size_t)sprintf(show + len, "%s%s",
					       i > 2 ? " " : "", w[i]);
		}
	}
	len = (size_t)snprintf(sig, sizeof sig, "%s|%s|%s|%s", w[2], v, r, mpn);
	if (!fits || len >= sizeof sig) {
		die("a value of a part record is too long", file, ln);
	}
	p->sig = hash(sig);
	p->show = dup(show);
}

// Gives the pins of a net that connects anything to their parts.
static void take_pins(struct list *l, struct net *t, char **num)
{
	if (!live(t)) {
		return;
	}
	for (size_t j = 0; j < t->n; j++) {
		struct part *p = &l->part[t->part[j]];
		struct pin pn;
		size_t room = strlen(num[j]) + 3;
		char *key = zalloc(room);

		(void)snprintf(key, room, ",%s,", num[j]);
		pn.num = dup(num[j]);
		pn.swap = strstr(p->swap, key) != NULL;
		free(key);
		pn.cls = pn.swap ? hash("~swap") : hash(num[j]);
		pn.net = (int)l->nnet;
		p->pin = grow(p->pin, p->npin, sizeof *p->pin);
		p->pin[p->npin] = pn;
		t->pin[j] = (int)p->npin++;
	}
}

// A net record: its words past "net", their number, and its line.
struct net_rec {
	char **w;
	size_t n;
	unsigned line;
};

// Reads the records of the netlist file f into l: the parts, and the
// net records into *recs, their number returned.
static size_t read_records(struct list *l, FILE *f, struct net_rec **recs)
{
	char *buf = NULL;
	char **w = NULL;
	size_t cap = 0;
	size_t wcap = 0;
	size_t nrecs = 0;
	unsigned ln = 0;

	while (read_line(f, &buf, &cap)) {
		size_t n = split(buf, &w, &wcap);

		ln++;
		if (!n) {
			continue;
		}
		if (!strcmp(w[0], "part") && n >= 3 && n % 2 == 1) {
			l->part = grow(l->part, l->npart, sizeof *l->part);
			read_part(&l->part[l->npart++], w, n, l->file, ln);
		} else if (!strcmp(w[0], "net") && n >= 2) {
			struct net_rec r = { NULL, n - 1, ln };

			r.w = (char **)zalloc(n * sizeof *r.w);
			for (size_t i = 1; i < n; i++) {
				r.w[i - 1] = dup(w[i]);
			}
			*recs = grow(*recs, nrecs, sizeof **recs);
			(*recs)[nrecs++] = r;
		} else if (!strcmp(w[0], "pin") && n == 3) {
			// A pin on no net: schcheck's, no difference here.
		} else {
			die("bad record", l->file, ln);
		}
	}
	free(buf);
	free((void *)w);
	return nrecs;
}

// Adds the net of record r to l, and its pins to their parts.
static void add_net(struct list *l, struct net_rec *r)
{
	struct net t;
	size_t first = r->n > 1 && !strcmp(r->w[1], "global") ? 2 : 1;

	memset(&t, 0, sizeof t);
	t.name = r->w[0];
	t.global = first == 2;
	// The class of the net plays no part.
	if (first + 1 < r->n && !strcmp(r->w[first], "class")) {
		first += 2;
	}
	t.part = zalloc(r->n * sizeof *t.part);
	t.pin = zalloc(r->n * sizeof *t.pin);
	for (size_t j = first; j < r->n; j++) {
		char *dot = strrchr(r->w[j], '.');
		long k = 0;

		if (!dot) {
			die("a pin is PART.PIN", l->file, r->line);
		}
		*dot = 0;
		k = find_part(l, r->w[j]);
		if (k < 0) {
			die("a pin of no part", l->file, r->line);
		}
		t.part[t.n] = (int)k;
		t.pin[t.n++] = -1;
		r->w[j] = dot + 1;
	}
	take_pins(l, &t, r->w + first);
	l->net = grow(l->net, l->nnet, sizeof *l->net);
	l->net[l->nnet++] = t;
}

// Reads a netlist: the parts, sorted by designator, then the nets.
static void load(struct list *l, const char *file)
{
	struct net_rec *recs = NULL;
	size_t nrecs = 0;
	FILE *f = fopen(file, "r");

	memset(l, 0, sizeof *l);
	l->file = file;
	if (!f) {
		die("cannot read", file, 0);
	}
	nrecs = read_records(l, f, &recs);
	(void)fclose(f);
	if (l->npart) {
		qsort(l->part, l->npart, sizeof *l->part, by_ref);
	}
	for (size_t i = 1; i < l->npart; i++) {
		if (!strcmp(l->part[i].ref, l->part[i - 1].ref)) {
			die("a designator twice", file, 0);
		}
	}
	for (size_t i = 0; i < nrecs; i++) {
		add_net(l, &recs[i]);
	}
	free(recs);
}

// One pass: the nets from their parts, then the parts from their nets.
static void refine(struct list *l)
{
	for (size_t i = 0; i < l->nnet; i++) {
		struct net *t = &l->net[i];
		uint64_t sum = t->global ? hash(t->name) : 0;

		if (!live(t)) {
			continue;
		}
		for (size_t j = 0; j < t->n; j++) {
			const struct part *p = &l->part[t->part[j]];

			sum += mix((p->label * label_mul) +
				   p->pin[t->pin[j]].cls);
		}
		t->label = mix(sum ^ net_salt);
	}
	for (size_t k = 0; k < l->npart; k++) {
		struct part *p = &l->part[k];
		uint64_t sum = 0;

		for (size_t q = 0; q < p->npin; q++) {
			sum += mix((p->pin[q].cls * class_mul) +
				   l->net[p->pin[q].net].label);
		}
		p->label = mix(p->label ^ mix(sum));
	}
}

// A label held by an unmatched part: its side and index, to sort.
struct held {
	uint64_t label;
	int side, k;
};

static int by_label(const void *lhs, const void *rhs)
{
	const struct held *x = lhs;
	const struct held *y = rhs;

	if (x->label != y->label) {
		return x->label < y->label ? -1 : 1;
	}
	if (x->side != y->side) {
		return x->side - y->side;
	}
	return x->k - y->k;
}

// The labels the unmatched parts hold, sorted; returns how many.
static size_t gather(const struct list *a, const struct list *b, struct held *h)
{
	size_t n = 0;

	for (int s = 0; s < 2; s++) {
		const struct list *l = s ? b : a;

		for (size_t k = 0; k < l->npart; k++) {
			if (l->part[k].mate < 0) {
				h[n].label = l->part[k].label;
				h[n].side = s;
				h[n++].k = (int)k;
			}
		}
	}
	qsort(h, n, sizeof *h, by_label);
	return n;
}

static size_t distinct(const struct held *h, size_t n)
{
	size_t d = 0;

	for (size_t i = 0; i < n; i++) {
		d += i == 0 || h[i].label != h[i - 1].label;
	}
	return d;
}

static void pair(struct list *a, struct list *b, int i, int j, uint64_t id)
{
	a->part[i].mate = j;
	b->part[j].mate = i;
	a->part[i].label = b->part[j].label = mix((id * 2) + 1);
	a->part[i].own = b->part[j].own = a->part[i].label;
}

// Starts the labels afresh from the values and the pairs, and refines
// them depth passes: labels that see only that far round each part, so
// a difference elsewhere leaves them alone.
static void shallow(struct list *a, struct list *b, int depth)
{
	for (int s = 0; s < 2; s++) {
		struct list *l = s ? b : a;

		for (size_t k = 0; k < l->npart; k++) {
			l->part[k].label = l->part[k].mate >= 0 ?
						   l->part[k].own :
						   l->part[k].sig;
		}
		for (int d = 0; d < depth; d++) {
			refine(l);
		}
	}
}

// The run of the label held most on both sides: its start in the
// sorted labels, and its count.
struct run {
	size_t best;
	size_t most;
};

// Pairs each label one unmatched part holds on each side; returns
// whether it paired any, and the run of the label held most on both
// sides in *r.
static int pair_unique(struct list *a, struct list *b, struct held *h,
		       uint64_t *id, struct run *r)
{
	size_t n = gather(a, b, h);
	int paired = 0;

	r->most = 0;
	for (size_t i = 0; i < n;) {
		size_t j = i;
		size_t na = 0;
		size_t nb = 0;

		while (j < n && h[j].label == h[i].label) {
			j++;
		}
		while (i + na < j && h[i + na].side == 0) {
			na++;
		}
		nb = j - i - na;
		if (na == 1 && nb == 1) {
			pair(a, b, h[i].k, h[i + 1].k, (*id)++);
			paired = 1;
		} else if (na && nb && (na < nb ? na : nb) > r->most) {
			r->most = na < nb ? na : nb;
			r->best = i;
		}
		i = j;
	}
	return paired;
}

// Refines both lists until the labels of the unmatched parts split no
// further.
static void settle(struct list *a, struct list *b, struct held *h)
{
	size_t d = 0;

	for (int pass = 0; pass < PASSES_MAX; pass++) {
		size_t e = 0;

		refine(a);
		refine(b);
		e = distinct(h, gather(a, b, h));
		if (e == d) {
			return;
		}
		d = e;
	}
}

// Matches the parts of a and b by their labels.
static void match(struct list *a, struct list *b)
{
	struct held *h = zalloc((a->npart + b->npart + 1) * sizeof *h);
	uint64_t id = 0;

	for (size_t k = 0; k < a->npart; k++) {
		a->part[k].label = a->part[k].sig;
	}
	for (size_t k = 0; k < b->npart; k++) {
		b->part[k].label = b->part[k].sig;
	}
	for (;;) {
		struct run r = { 0, 0 };
		int paired = 0;

		settle(a, b, h);
		paired = pair_unique(a, b, h, &id, &r);
		if (!paired && r.most) {
			size_t j = r.best;

			while (h[j].side == 0) {
				j++;
			}
			pair(a, b, h[r.best].k, h[j].k, id++);
			paired = 1;
		}
		// Where the circuits differ, the labels near the difference
		// differ all the way out; labels that see less still match.
		for (int depth = 3; !paired && depth >= 0; depth--) {
			shallow(a, b, depth);
			paired = pair_unique(a, b, h, &id, &r);
		}
		if (!paired) {
			break;
		}
	}
	free(h);
}

static int by_int(const void *lhs, const void *rhs)
{
	int x = *(const int *)lhs;
	int y = *(const int *)rhs;

	return (x > y) - (x < y);
}

// The lists matched, a and b: the pin of b that each pin of a stands
// for (or -1), part by part, and the net of b that each net of a goes
// with (or -1).
struct maps {
	struct list *a;
	struct list *b;
	int **map;
	int *corr;
};

// The net of b that the net t of a goes with: the global net of its
// name, or the net that holds most of its pins as map gives them.
static int net_mate(const struct maps *m, const struct net *t, int *tmp)
{
	const struct list *a = m->a;
	const struct list *b = m->b;
	int *const *map = m->map;
	size_t n = 0;
	int best = -1;
	int run = 0;
	int bestrun = 0;

	if (t->global) {
		for (size_t j = 0; j < b->nnet; j++) {
			if (b->net[j].global &&
			    !strcmp(b->net[j].name, t->name)) {
				return (int)j;
			}
		}
	}
	for (size_t j = 0; live(t) && j < t->n; j++) {
		int k = t->part[j];
		int v = map[k] ? map[k][t->pin[j]] : -1;

		if (v >= 0) {
			tmp[n++] = b->part[a->part[k].mate].pin[v].net;
		}
	}
	qsort(tmp, n, sizeof *tmp, by_int);
	for (size_t i = 0; i < n; i++) {
		run = i && tmp[i] == tmp[i - 1] ? run + 1 : 1;
		if (run > bestrun) {
			bestrun = run;
			best = tmp[i];
		}
	}
	return best;
}

// How well pin pv of a part of b stands for pin pu of its mate in a: a
// pin of one number SCORE_NUMBER, of another -1; a pin of a swap list
// 4 if its net goes with that of pu, 2 more if the labels of their
// nets agree, 1 more if of one number.
static int pin_score(const struct maps *m, const struct pin *pu,
		     const struct pin *pv)
{
	if (!pu->swap) {
		return strcmp(pv->num, pu->num) ? -1 : SCORE_NUMBER;
	}
	return (4 * (m->corr[pu->net] == pv->net)) +
	       (2 * (m->b->net[pv->net].label == m->a->net[pu->net].label)) +
	       !strcmp(pv->num, pu->num);
}

// The pin of the mate of part p of a that stands best for its pin u, of
// those of its class not taken by the pins before u in row, or -1.
static int best_pin(const struct maps *m, const struct part *p, const int *row,
		    size_t u)
{
	const struct part *q = p->mate >= 0 ? &m->b->part[p->mate] : NULL;
	int best = -1;
	int bs = -1;

	for (size_t v = 0; q && v < q->npin; v++) {
		int sc = 0;
		int taken = 0;

		for (size_t x = 0; x < u; x++) {
			taken = taken || row[x] == (int)v;
		}
		if (taken || q->pin[v].cls != p->pin[u].cls) {
			continue;
		}
		sc = pin_score(m, &p->pin[u], &q->pin[v]);
		if (sc > bs) {
			bs = sc;
			best = (int)v;
		}
	}
	return best;
}

// Fills the map and corr of m.  Pins of one number stand for each other;
// a pin of a swap list stands for the pin of the list on the net that
// goes with its own, if there is one.  The two are refined in turn.
static void build_map(struct maps *m)
{
	const struct list *a = m->a;
	size_t most = 1;
	int *tmp = NULL;

	for (size_t i = 0; i < a->nnet; i++) {
		most = a->net[i].n > most ? a->net[i].n : most;
		m->corr[i] = -1;
	}
	tmp = zalloc(most * sizeof *tmp);
	for (int round = 0; round < 4; round++) {
		for (size_t k = 0; k < a->npart; k++) {
			const struct part *p = &a->part[k];

			for (size_t u = 0; m->map[k] && u < p->npin; u++) {
				m->map[k][u] = best_pin(m, p, m->map[k], u);
			}
		}
		for (size_t i = 0; i < a->nnet; i++) {
			m->corr[i] = net_mate(m, &a->net[i], tmp);
		}
	}
	free(tmp);
}

// Pairs the parts left unmatched: the same values, and most pins on nets
// that go with each other.
static void pair_rest(struct maps *m)
{
	struct list *a = m->a;
	struct list *b = m->b;

	for (size_t i = 0; i < a->npart; i++) {
		struct part *p = &a->part[i];
		int best = -1;
		int bs = -1;

		if (p->mate >= 0) {
			continue;
		}
		for (size_t j = 0; j < b->npart; j++) {
			const struct part *q = &b->part[j];
			int sc = 0;

			if (q->mate >= 0 || q->sig != p->sig) {
				continue;
			}
			for (size_t u = 0; u < p->npin; u++) {
				for (size_t v = 0; v < q->npin; v++) {
					sc += m->corr[p->pin[u].net] ==
						      q->pin[v].net &&
					      q->pin[v].cls == p->pin[u].cls;
				}
			}
			if (sc > bs) {
				bs = sc;
				best = (int)j;
			}
		}
		if (best >= 0) {
			p->mate = best;
			b->part[best].mate = (int)i;
			m->map[i] = zalloc((p->npin + 1) * sizeof **m->map);
		}
	}
	build_map(m);
}

static void pin_only(const char *an, const char *bn, const struct list *l,
		     const struct part *p, const char *num)
{
	printf("net %s / %s: pin only in %s: %s.%s\n", an, bn, l->file, p->ref,
	       num);
}

static void net_only(const struct list *l, const struct net *t)
{
	printf("net only in %s: %s", l->file, t->name);
	for (size_t j = 0; j < t->n; j++) {
		const struct part *p = &l->part[t->part[j]];

		printf(" %s.%s", p->ref, p->pin[t->pin[j]].num);
	}
	printf("\n");
}

// Nets that go with each other: net i of a, net mi of b.
struct net_pair {
	size_t i;
	int mi;
};

// Prints how net i of a differs from the net mi of b it goes with;
// returns whether it does.
static int compare_net(const struct maps *m, struct net_pair np)
{
	const struct list *a = m->a;
	const struct list *b = m->b;
	int *const *map = m->map;
	const struct net *t = &a->net[np.i];
	const struct net *s = &b->net[np.mi];
	int differ = 0;

	for (size_t j = 0; j < t->n; j++) {
		int k = t->part[j];
		int v = map[k] ? map[k][t->pin[j]] : -1;

		if (v < 0 || b->part[a->part[k].mate].pin[v].net != np.mi) {
			pin_only(t->name, s->name, a, &a->part[k],
				 a->part[k].pin[t->pin[j]].num);
			differ = 1;
		}
	}
	for (size_t j = 0; j < s->n; j++) {
		int k = s->part[j];
		int v = s->pin[j];
		int pa = b->part[k].mate;
		int hit = 0;

		for (size_t u = 0; pa >= 0 && map[pa] && u < a->part[pa].npin;
		     u++) {
			hit = hit || (map[pa][u] == v &&
				      a->part[pa].pin[u].net == (int)np.i);
		}
		if (!hit) {
			pin_only(t->name, s->name, b, &b->part[k],
				 b->part[k].pin[v].num);
			differ = 1;
		}
	}
	return differ;
}

int main(int argc, char **argv)
{
	struct list a;
	struct list b;
	struct maps m = { &a, &b, NULL, NULL };
	int *used = NULL;
	int differ = 0; // Whether anything was printed.

	if (argc != 3) {
		(void)fprintf(stderr, "usage: netdiff a.nl b.nl\n");
		return 2;
	}
	load(&a, argv[1]);
	load(&b, argv[2]);
	match(&a, &b);
	m.map = (int **)zalloc((a.npart + 1) * sizeof *m.map);
	m.corr = zalloc((a.nnet + 1) * sizeof *m.corr);
	used = zalloc((b.nnet + 1) * sizeof *used);
	for (size_t k = 0; k < a.npart; k++) {
		if (a.part[k].mate >= 0) {
			m.map[k] =
				zalloc((a.part[k].npin + 1) * sizeof **m.map);
		}
	}
	build_map(&m);
	pair_rest(&m);
	for (int s = 0; s < 2; s++) {
		const struct list *l = s ? &b : &a;

		for (size_t k = 0; k < l->npart; k++) {
			if (l->part[k].mate < 0) {
				differ = 1;
				printf("part only in %s: %s %s\n", l->file,
				       l->part[k].ref, l->part[k].show);
			}
		}
	}
	for (size_t i = 0; i < a.nnet; i++) {
		int mi = m.corr[i];

		if (!live(&a.net[i])) {
			continue;
		}
		if (mi < 0) {
			net_only(&a, &a.net[i]);
			differ = 1;
			continue;
		}
		if (used[mi]) {
			differ = 1;
			printf("nets %s and another of %s are one net %s of %s\n",
			       a.net[i].name, a.file, b.net[mi].name, b.file);
		}
		used[mi] = 1;
		differ = compare_net(&m, (struct net_pair){ i, mi }) || differ;
	}
	for (size_t j = 0; j < b.nnet; j++) {
		if (live(&b.net[j]) && !used[j]) {
			net_only(&b, &b.net[j]);
			differ = 1;
		}
	}
	free(used);
	(void)fflush(stdout);
	if (ferror(stdout)) {
		return 2;
	}
	return differ;
}
