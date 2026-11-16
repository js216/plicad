// SPDX-License-Identifier: GPL-3.0-only
// schnet.c -- Write the netlist of a schematic sheet
// Copyright (c) 2026 Jakob Kastelic

// Usage: schnet [-L LIB]... sheet.sch > sheet.nl
//
// Reads the source (sch(5)) and its symbols, as schdraw does, and writes
// the flat netlist (netlist(5)): a part record for each part, by its
// designator, and a net record for each net, with its pins by their
// numbers.  Flags are not parts; a pin on no net is left out.  The parts
// and the pins of a net are sorted by designator and pin, the nets by
// name, so the output of a source is always the same.

#include "schsrc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct sch S;

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
	const struct sch_part *p = &S.part[*(const size_t *)a];
	const struct sch_part *q = &S.part[*(const size_t *)b];

	return ref_cmp(p->ref, q->ref);
}

static int by_pin(const void *a, const void *b)
{
	const struct sch_end *e = a, *f = b;
	const struct sch_part *p = &S.part[e->part], *q = &S.part[f->part];
	int c = ref_cmp(p->ref, q->ref);

	if (c) {
		return c;
	}
	return ref_cmp(sch_pnum(p->dev, e->pin), sch_pnum(q->dev, f->pin));
}

static int by_name(const void *a, const void *b)
{
	const struct sch_net *n = a, *m = b;

	return strcmp(n->name, m->name);
}

int main(int argc, char **argv)
{
	char **lib = NULL;
	size_t nlib = 0, *order;
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
	faults = sch_read(&S, argv[i], lib, nlib);
	order = sch_alloc(S.npart * sizeof *order);
	for (size_t k = 0; k < S.npart; k++) {
		order[k] = k;
	}
	qsort(order, S.npart, sizeof *order, by_ref);
	printf("# netlist of %s, from schnet\n",
	       strrchr(S.file, '/') ? strrchr(S.file, '/') + 1 : S.file);
	for (size_t k = 0; k < S.npart; k++) {
		const struct sch_part *q = &S.part[order[k]];
		const char *swap = q->dev->shape->swap;

		if (q->dev->shape->flag) {
			continue;
		}
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
				for (size_t p = 0; p < q->dev->shape->npin;
				     p++) {
					if (!strcmp(q->dev->shape->pin[p].name,
						    w)) {
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
				       q->dev->shape->pin[p].name,
				       sch_pnum(q->dev, p));
			}
		}
		printf(" name %s\n", q->name);
	}
	qsort(S.net, S.nnet, sizeof *S.net, by_name);
	for (size_t k = 0; k < S.nnet; k++) {
		struct sch_net *t = &S.net[k];
		size_t n = 0;

		qsort(t->end, t->nend, sizeof *t->end, by_pin);
		for (size_t j = 0; j < t->nend; j++) {
			n += !S.part[t->end[j].part].dev->shape->flag;
		}
		if (!n) {
			continue;
		}
		printf("net %s%s", t->name, t->global ? " global" : "");
		for (size_t j = 0; j < t->nend; j++) {
			const struct sch_part *q = &S.part[t->end[j].part];

			if (!q->dev->shape->flag) {
				printf(" %s.%s", q->ref,
				       sch_pnum(q->dev, t->end[j].pin));
			}
		}
		printf("\n");
	}
	return faults > 0;
}
