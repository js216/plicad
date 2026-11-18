// SPDX-License-Identifier: GPL-3.0-only
// schnet.c -- Write the netlist of a schematic sheet
// Copyright (c) 2026 Jakob Kastelic

// Usage: schnet [-L LIB]... sheet.sch > sheet.nl
//
// Reads the source (sch(5)), every page of it, and its symbols, as
// schdraw does, and writes the flat netlist (netlist(5)): a part record
// for each part, by its designator, and a net record for each net, with
// its pins by their numbers and its class (power for the net of a
// supply or ground flag); a global net of several pages is one net.  A
// net of two classes is a fault, and then no netlist is written.
// Flags are not parts; each pin on no net is a pin record of its own,
// after the nets, nc if the source leaves it unconnected on purpose, -
// if not.  The parts and the pins of a net are sorted by designator and
// pin, the nets by name, so the output of a source is always the same.

#include "schsrc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { DECIMAL = 10 }; // The base of decimal numbers.

#define USAGE "usage: schnet [-L LIB]... sheet.sch > sheet.nl"

// A part or a pin of a page.
struct at {
	const struct sch *s;
	size_t part, pin;
};

// A net of a page.
struct net_at {
	const struct sch *s;
	const struct sch_net *t;
};

// Compares designators as a person would: letters, then the number.
static int ref_cmp(const char *a, const char *b)
{
	size_t na = 0;
	size_t nb = 0;
	int c = 0;

	if (!a || !b) {
		return !a - !b; // A flag, with none, goes last.
	}
	na = strcspn(a, "0123456789");
	nb = strcspn(b, "0123456789");
	c = strncmp(a, b, na < nb ? na : nb);
	if (c) {
		return c;
	}
	if (na != nb) {
		return na < nb ? -1 : 1;
	}
	if (strtol(a + na, NULL, DECIMAL) != strtol(b + nb, NULL, DECIMAL)) {
		return strtol(a + na, NULL, DECIMAL) <
				       strtol(b + nb, NULL, DECIMAL) ?
			       -1 :
			       1;
	}
	return strcmp(a, b);
}

static int by_ref(const void *lhs, const void *rhs)
{
	const struct at *e = lhs;
	const struct at *f = rhs;

	return ref_cmp(e->s->part[e->part].ref, f->s->part[f->part].ref);
}

static int by_pin(const void *lhs, const void *rhs)
{
	const struct at *e = lhs;
	const struct at *f = rhs;
	const struct sch_part *p = &e->s->part[e->part];
	const struct sch_part *q = &f->s->part[f->part];
	int c = ref_cmp(p->ref, q->ref);

	if (c) {
		return c;
	}
	return ref_cmp(sch_pnum(p->dev, e->pin), sch_pnum(q->dev, f->pin));
}

static int by_name(const void *lhs, const void *rhs)
{
	const struct net_at *n = lhs;
	const struct net_at *m = rhs;

	return strcmp(n->t->name, m->t->name);
}

// Writes the part record of a part.
static void write_part(const struct sch_part *q)
{
	const char *swap = q->dev->shape->swap;

	printf("part %s %s", q->ref, q->dev->name);
	if (q->value) {
		printf(" value %s", q->value);
	}
	if (q->rating) {
		printf(" rating %s", q->rating);
	}
	if (q->mpn) {
		printf(" mpn %s", q->mpn);
	}
	if (swap) {
		// The numbers of the pins that may trade places.
		char *buf = sch_strdup(swap);
		char *w = NULL;
		int first = 1;

		printf(" swap ");
		for (w = strtok(buf, " "); w; w = strtok(NULL, " ")) {
			for (size_t p = 0; p < q->dev->shape->npin; p++) {
				if (!strcmp(q->dev->shape->pin[p].name, w)) {
					printf("%s%s", first ? "" : ",",
					       sch_pnum(q->dev, p));
					first = 0;
				}
			}
		}
		free(buf);
	}
	// The names of the pins of a part with a pin table.
	if (q->dev->pnum[0] != q->dev->shape->pin[0].name) {
		printf(" pins ");
		for (size_t p = 0; p < q->dev->shape->npin; p++) {
			printf("%s%s:%s", p ? "," : "",
			       q->dev->shape->pin[p].name, sch_pnum(q->dev, p));
		}
	}
	printf(" name %s\n", q->name);
}

// The class of the nets n[0] .. n[m-1], of one name, or NULL; *clash
// is set if they are of two classes, told once, by the line of a class
// statement.
static const char *net_class(const struct net_at *n, size_t m, int *clash)
{
	const struct net_at *c = NULL;

	for (size_t i = 0; i < m; i++) {
		const struct net_at *o = &n[i];

		*clash = *clash || o->t->clash;
		if (!o->t->klass) {
			continue;
		}
		if (!c) {
			c = o;
		} else if (strcmp(c->t->klass, o->t->klass) != 0) {
			const struct net_at *at = o->t->class_line ? o : c;

			sch_warn("%s:%d: net %s is of class %s and of class %s",
				 at->s->file, at->t->class_line, o->t->name,
				 c->t->klass, o->t->klass);
			*clash = 1;
			break;
		}
	}
	return c ? c->t->klass : NULL;
}

// Writes the net record of the nets n[0] .. n[m-1], of one name: their
// pins, those of the flags left out, if any are left.
static void write_net(const struct net_at *n, size_t m)
{
	struct at *pin = NULL;
	size_t np = 0;
	size_t k = 0;
	int global = 0;
	int clash = 0;
	const char *c = net_class(n, m, &clash);

	for (size_t i = 0; i < m; i++) {
		np += n[i].t->nend;
		global = global || n[i].t->global;
	}
	pin = sch_alloc(np * sizeof *pin);
	for (size_t i = 0; i < m; i++) {
		for (size_t j = 0; j < n[i].t->nend; j++) {
			const struct sch_end *e = &n[i].t->end[j];

			if (!n[i].s->part[e->part].dev->shape->flag) {
				pin[k++] =
					(struct at){ n[i].s, e->part, e->pin };
			}
		}
	}
	qsort(pin, k, sizeof *pin, by_pin);
	if (k) {
		printf("net %s%s%s%s", n[0].t->name, global ? " global" : "",
		       c ? " class " : "", c ? c : "");
		for (size_t j = 0; j < k; j++) {
			const struct sch_part *q = &pin[j].s->part[pin[j].part];

			printf(" %s.%s", q->ref, sch_pnum(q->dev, pin[j].pin));
		}
		printf("\n");
	}
	free(pin);
}

// The symbol directories of the -L options, from argv[1], or the
// default one; *argi is set to the first argument past the options.
static struct sch_lib read_options(int argc, char **argv, int *argi)
{
	struct sch_lib sl = { NULL, 0, NULL, 0, NULL, 0, 0 };
	int i = 1;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (strcmp(argv[i], "-L") == 0 && i + 1 < argc) {
			size_t n = strlen(argv[i + 1]) + sizeof "/symbols";
			char **more = (char **)realloc(
				(void *)sl.dir, (sl.ndir + 1) * sizeof *sl.dir);

			if (!more) {
				sch_die("out of memory");
			}
			sl.dir = more;
			sl.dir[sl.ndir] = sch_alloc(n);
			(void)snprintf(sl.dir[sl.ndir++], n, "%s/symbols",
				       argv[++i]);
		} else {
			sch_die(USAGE);
		}
	}
	if (i != argc - 1) {
		sch_die(USAGE);
	}
	if (!sl.ndir) {
		sl.dir = (char **)sch_alloc(sizeof *sl.dir);
		sl.dir[0] = sch_default_lib(argv[0]);
		sl.ndir = 1;
	}
	*argi = i;
	return sl;
}

// Reads every page of file into *page, which is set; *npage is set to
// their number.  Returns the faults.
static int read_pages(const char *file, struct sch_lib *sl, struct sch **page,
		      size_t *npage)
{
	struct sch *s = sch_alloc(sizeof *s);
	struct sch *more = NULL;
	int faults = sch_read(&s[0], file, 0, sl);
	size_t n = s[0].npages ? s[0].npages : 1;

	more = realloc(s, n * sizeof *s);
	if (!more) {
		sch_die("out of memory");
	}
	s = more;
	for (size_t k = 1; k < n; k++) {
		faults += sch_read(&s[k], file, k, sl);
	}
	*page = s;
	*npage = n;
	return faults;
}

// The number of the nets from net[k] on of the name of net[k].
static size_t same_name(const struct net_at *net, size_t nnet, size_t k)
{
	size_t m = 1;

	while (k + m < nnet && !strcmp(net[k].t->name, net[k + m].t->name)) {
		m++;
	}
	return m;
}

// Whether pin p of part j of page s is on a net.
static int on_net(const struct sch *s, size_t j, size_t p)
{
	for (size_t t = 0; t < s->nnet; t++) {
		for (size_t e = 0; e < s->net[t].nend; e++) {
			if (s->net[t].end[e].part == j &&
			    s->net[t].end[e].pin == p) {
				return 1;
			}
		}
	}
	return 0;
}

// Writes the pin records of the pins on no net: of each part, in the
// order of its pins.
static void write_lone_pins(const struct at *part, size_t npart)
{
	for (size_t k = 0; k < npart; k++) {
		const struct sch *s = part[k].s;
		const struct sch_part *q = &s->part[part[k].part];

		for (size_t p = 0; p < q->dev->shape->npin; p++) {
			if (!on_net(s, part[k].part, p)) {
				printf("pin %s.%s %s\n", q->ref,
				       sch_pnum(q->dev, p),
				       q->nc && q->nc[p] ? "nc" : "-");
			}
		}
	}
}

static void free_lib(struct sch_lib *sl)
{
	for (size_t k = 0; k < sl->ndir; k++) {
		free(sl->dir[k]);
	}
	free((void *)sl->dir);
}

int main(int argc, char **argv)
{
	int i = 0;
	struct sch_lib sl = read_options(argc, argv, &i);
	struct sch *page = NULL;
	size_t npage = 0;
	size_t npart = 0;
	size_t nnet = 0;
	struct at *part = NULL;
	struct net_at *net = NULL;
	int faults = read_pages(argv[i], &sl, &page, &npage);
	int clash = 0;

	for (size_t k = 0; k < npage; k++) {
		npart += page[k].npart;
		nnet += page[k].nnet;
	}
	part = sch_alloc(npart * sizeof *part);
	net = sch_alloc(nnet * sizeof *net);
	npart = nnet = 0;
	for (size_t k = 0; k < npage; k++) {
		for (size_t j = 0; j < page[k].npart; j++) {
			if (!page[k].part[j].dev->shape->flag) {
				part[npart++] = (struct at){ &page[k], j, 0 };
			}
		}
		for (size_t j = 0; j < page[k].nnet; j++) {
			net[nnet++] =
				(struct net_at){ &page[k], &page[k].net[j] };
		}
	}
	qsort(part, npart, sizeof *part, by_ref);
	qsort(net, nnet, sizeof *net, by_name);
	// A net of two classes: no netlist.
	for (size_t k = 0, m = 0; k < nnet; k += m) {
		m = same_name(net, nnet, k);
		(void)net_class(&net[k], m, &clash);
	}
	if (clash) {
		free(part);
		free(net);
		free_lib(&sl);
		free(page);
		return 1;
	}
	printf("# netlist of %s, from schnet\n",
	       strrchr(argv[i], '/') ? strrchr(argv[i], '/') + 1 : argv[i]);
	for (size_t k = 0; k < npart; k++) {
		write_part(&part[k].s->part[part[k].part]);
	}
	for (size_t k = 0, m = 0; k < nnet; k += m) {
		m = same_name(net, nnet, k);
		write_net(&net[k], m);
	}
	write_lone_pins(part, npart);
	free(part);
	free(net);
	free_lib(&sl);
	free(page);
	return faults > 0;
}
