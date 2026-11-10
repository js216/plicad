// SPDX-License-Identifier: GPL-3.0-only
// bsolid.c -- The shells of a boundary representation
// Copyright (c) 2026 Jakob Kastelic

#include "brep.h"
#include "bsolid.h"
#include "util.h"
#include <stdlib.h>

// The two faces of each edge, edge e at uses[2 e] and uses[2 e + 1].
static size_t *edge_uses(const struct brep *b)
{
	size_t *uses = xmalloc((2 * b->ne + 1) * sizeof(*uses));
	size_t *count = xcalloc(b->ne + 1, sizeof(*count));

	for (size_t fi = 0; fi < b->nf; fi++) {
		for (size_t l = 0; l < b->f[fi].nloops; l++) {
			const struct bloop *lp = &b->l[b->f[fi].loop0 + l];

			for (size_t k = 0; k < lp->n; k++) {
				size_t e = b->u[lp->first + k].e;

				if (count[e] == 2) {
					die("internal error: edge %zu has "
					    "more than two uses",
					    e);
				}
				uses[(2 * e) + count[e]++] = fi;
			}
		}
	}
	for (size_t e = 0; e < b->ne; e++) {
		if (count[e] != 2) {
			die("internal error: edge %zu has %zu uses", e,
			    count[e]);
		}
	}
	free(count);
	return uses;
}

// Gives the faces of the shell of face f the number id.
static void mark_shell(const struct brep *b, const size_t *uses, size_t *shell,
		       size_t f, size_t id)
{
	size_t *stack = xmalloc((b->nf + 1) * sizeof(*stack));
	size_t sp = 0;

	shell[f] = id;
	stack[sp++] = f;
	while (sp) {
		size_t fi = stack[--sp];

		for (size_t e = 0; e < b->ne; e++) {
			size_t f0 = uses[2 * e];
			size_t f1 = uses[(2 * e) + 1];
			size_t g = f0 == fi ? f1 : f0;

			if ((f0 == fi || f1 == fi) && shell[g] == NONE) {
				shell[g] = id;
				stack[sp++] = g;
			}
		}
	}
	free(stack);
}

size_t brep_shells(const struct brep *b, size_t *shell)
{
	size_t *uses = edge_uses(b);
	size_t n = 0;

	for (size_t f = 0; f < b->nf; f++) {
		shell[f] = NONE;
	}
	for (size_t f = 0; f < b->nf; f++) {
		if (shell[f] == NONE) {
			mark_shell(b, uses, shell, f, n++);
		}
	}
	free(uses);
	return n;
}
