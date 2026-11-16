// SPDX-License-Identifier: GPL-3.0-only
// schnet.c -- Write the netlist of a schematic sheet
// Copyright (c) 2026 Jakob Kastelic

// Usage: schnet [-L LIB]... sheet.sch > sheet.nl
//
// Reads the source (sch(5)), every page of it, and its symbols, as
// schdraw does, and writes the flat netlist (netlist(5)): a part record
// for each part, by its designator, and a net record for each net, with
// its pins by their numbers; a global net of several pages is one net.
// Flags are not parts; a pin on no net is left out.  The parts and the
// pins of a net are sorted by designator and pin, the nets by name, so
// the output of a source is always the same.

#include "schsrc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The pages read.
static struct sch *S;

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
	size_t na, nb;
	int c;

	if (!a || !b) {
		return !a - !b; // A flag, with none, goes last.
	}
	na = strcspn(a, "0123456789");
	nb = strcspn(b, "0123456789");
	c = strncmp(a, b, na < nb ? na : nb);
	if (c || na != nb) {
		return c ? c : (na < nb ? -1 : 1);
	}
	if (atol(a + na) != atol(b + nb)) {
		return atol(a + na) < atol(b + nb) ? -1 : 1;
	}
	return strcmp(a, b);
}

static int by_ref(const void *a, const void *b)
{
	const struct at *e = a, *f = b;

	return ref_cmp(e->s->part[e->part].ref, f->s->part[f->part].ref);
}

static int by_pin(const void *a, const void *b)
{
	const struct at *e = a, *f = b;
	const struct sch_part *p = &e->s->part[e->part];
	const struct sch_part *q = &f->s->part[f->part];
	int c = ref_cmp(p->ref, q->ref);

	if (c) {
		return c;
	}
	return ref_cmp(sch_pnum(p->dev, e->pin), sch_pnum(q->dev, f->pin));
}

static int by_name(const void *a, const void *b)
{
	const struct net_at *n = a, *m = b;

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
		char buf[256], *w;
		int first = 1;

		snprintf(buf, sizeof buf, "%s", swap);
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

// Writes the net record of the nets n[0] .. n[m-1], of one name: their
// pins, those of the flags left out, if any are left.
static void write_net(const struct net_at *n, size_t m)
{
	struct at *pin;
	size_t np = 0, k = 0;
	int global = 0;

	for (size_t i = 0; i < m; i++) {
		np += n[i].t->nend;
		global |= n[i].t->global;
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
		printf("net %s%s", n[0].t->name, global ? " global" : "");
		for (size_t j = 0; j < k; j++) {
			const struct sch_part *q = &pin[j].s->part[pin[j].part];

			printf(" %s.%s", q->ref, sch_pnum(q->dev, pin[j].pin));
		}
		printf("\n");
	}
	free(pin);
}

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0, npage, npart = 0, nnet = 0;
	struct at *part;
	struct net_at *net;
	int i = 1, faults;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (strcmp(argv[i], "-L") == 0 && i + 1 < argc) {
			char **more = realloc(lib, (nlib + 1) * sizeof *lib);

			if (!more) {
				sch_die("out of memory");
			}
			lib = more;
			lib[nlib] = sch_alloc(strlen(argv[i + 1]) + 16);
			sprintf(lib[nlib++], "%s/symbols", argv[++i]);
		} else {
			sch_die("usage: schnet [-L LIB]... sheet.sch > sheet.nl");
		}
	}
	if (i != argc - 1) {
		sch_die("usage: schnet [-L LIB]... sheet.sch > sheet.nl");
	}
	if (!nlib) {
		lib = sch_alloc(sizeof *lib);
		lib[0] = sch_default_lib(argv[0]);
		nlib = 1;
	}
	S = sch_alloc(sizeof *S);
	faults = sch_read(&S[0], argv[i], 0, lib, nlib);
	npage = S[0].npages ? S[0].npages : 1;
	S = realloc(S, npage * sizeof *S);
	if (!S) {
		sch_die("out of memory");
	}
	for (size_t k = 1; k < npage; k++) {
		faults += sch_read(&S[k], argv[i], k, lib, nlib);
	}
	for (size_t k = 0; k < npage; k++) {
		npart += S[k].npart;
		nnet += S[k].nnet;
	}
	part = sch_alloc(npart * sizeof *part);
	net = sch_alloc(nnet * sizeof *net);
	npart = nnet = 0;
	for (size_t k = 0; k < npage; k++) {
		for (size_t j = 0; j < S[k].npart; j++) {
			if (!S[k].part[j].dev->shape->flag) {
				part[npart++] = (struct at){ &S[k], j, 0 };
			}
		}
		for (size_t j = 0; j < S[k].nnet; j++) {
			net[nnet++] = (struct net_at){ &S[k], &S[k].net[j] };
		}
	}
	qsort(part, npart, sizeof *part, by_ref);
	printf("# netlist of %s, from schnet\n",
	       strrchr(argv[i], '/') ? strrchr(argv[i], '/') + 1 : argv[i]);
	for (size_t k = 0; k < npart; k++) {
		write_part(&part[k].s->part[part[k].part]);
	}
	qsort(net, nnet, sizeof *net, by_name);
	for (size_t k = 0, m; k < nnet; k += m) {
		for (m = 1; k + m < nnet &&
			    !strcmp(net[k].t->name, net[k + m].t->name);
		     m++) {
		}
		write_net(&net[k], m);
	}
	free(part);
	free(net);
	for (size_t k = 0; k < nlib; k++) {
		free(lib[k]);
	}
	free(lib);
	return faults > 0;
}
