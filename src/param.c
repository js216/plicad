// SPDX-License-Identifier: GPL-3.0-only
// param.c -- Print the values of the names of a source file
// Copyright (c) 2026 Jakob Kastelic

// Usage: param [-t] FILE NAME...
//        param [-t] -e EXPR FILE
//
// Reads the definitions of a source file in the language of plicad(7):
// its lines NAME = EXPR, and the parameters of its lines include FILE,
// in order, with the lines tolerance LEN ANGLE of a file of parameters;
// the other lines it leaves alone.  Then it prints the value
// of each NAME, or of the expression EXPR, one a line, in mm for a
// length and in degrees for an angle, to 12 places; with -t, the low
// and the high end of its range after it.

#include "expr.h"
#include "util.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { NAME_LEN = 64 }; // Room for a name.

// The reading of the source file: its path and its names.
struct reading {
	const char *path;
	struct names nm;
};

// Takes a line NAME = EXPR or include FILE of the source; leaves others.
static void line(char *l, void *ctx)
{
	struct reading *rd = ctx;
	size_t a = strspn(l, " \t");
	size_t e = a;
	const char *eq = strchr(l, '=');
	char name[NAME_LEN];

	if (strncmp(l + a, "include", strlen("include")) == 0 &&
	    isspace((unsigned char)l[a + strlen("include")])) {
		struct sibling sb = { rd->path, l + a + strlen("include") };
		char *path = NULL;

		sb.name += strspn(sb.name, " \t");
		path = sibling_path(sb);
		read_params(&rd->nm, path);
		free(path);
		return;
	}
	if (tolerance_line(&rd->nm, l)) {
		return;
	}
	while (l[e] == '_' || isalnum((unsigned char)l[e])) {
		e++;
	}
	if (!eq || e == a || e - a >= sizeof(name) ||
	    eq != l + e + strspn(l + e, " \t")) {
		return;
	}
	memcpy(name, l + a, e - a);
	name[e - a] = 0;
	define(&rd->nm, name, eq + 1, 0);
}

// Prints the value of the expression s, and with range its range.
static void print_value(struct reading *rd, const char *s, int range)
{
	struct ival v = eval_range(&rd->nm, s);

	if (range) {
		printf("%.12g %.12g %.12g\n", v.v, v.lo, v.hi);
	} else {
		printf("%.12g\n", v.v);
	}
}

int main(int argc, char **argv)
{
	struct reading rd;
	int range = argc > 1 && strcmp(argv[1], "-t") == 0;
	int a = range ? 2 : 1;
	int expr = argc == a + 3 && strcmp(argv[a], "-e") == 0;

	memset(&rd, 0, sizeof(rd));
	if (!expr && (argc < a + 2 || argv[a][0] == '-')) {
		die("usage: param [-t] FILE NAME...\n"
		    "       param [-t] -e EXPR FILE");
	}
	rd.path = argv[expr ? a + 2 : a];
	each_line(&rd.nm, rd.path, line, &rd);
	if (expr) {
		print_value(&rd, argv[a + 1], range);
	}
	for (int i = a + 1; !expr && i < argc; i++) {
		print_value(&rd, argv[i], range);
	}
	free_names(&rd.nm);
	close_stdout();
	return 0;
}
