// SPDX-License-Identifier: GPL-3.0-only
// schcheck.c -- Check a netlist for what a schematic leaves undone
// Copyright (c) 2026 Jakob Kastelic

// Usage: schcheck [FILE.nl]
//
// Reads a netlist (netlist(5)), from FILE or stdin, and tells each
// fault on a line, FILE: and the fault (stdin for none):
//   a pin on no net, but one the source marks unconnected (pin REF.PIN
//   nc), as schnet lists them (pin REF.PIN -);
//   a net of one pin alone;
//   a part with two pins or more, all of them on one net (a part
//   shorted);
//   a net of two classes, told by its line (FILE:LINE:).
// The exit status is 1 on a fault, 2 if the file cannot be read.

#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LINE = 1U << 16U, WORDS = 4096, MSG_LEN = 2100 };

// A pin of a part: its part's designator, and its net (NULL for none).
struct pin {
	char *ref;
	const char *net;
};

// The check under way: the pins and the nets so far, the file and its
// line, and whether a fault has come.
struct check {
	struct pin *pins;
	size_t npins;
	char **nets;
	size_t nnets;
	const char *file;
	int line;
	int faults;
};

// Splits a line into words at blanks, up to a #.
static size_t split(char *s, char **w)
{
	size_t n = 0;

	for (;;) {
		s += strspn(s, " \t\r\n");
		if (!*s || *s == '#' || n == WORDS) {
			return n;
		}
		w[n++] = s;
		s += strcspn(s, " \t\r\n");
		if (*s) {
			*s++ = 0;
		}
	}
}

// Notes a pin of a part, REF.PIN, on no net; returns it, for its net.
static struct pin *add_pin(struct check *ck, const char *refpin)
{
	const char *dot = strrchr(refpin, '.');
	size_t n = dot ? (size_t)(dot - refpin) : strlen(refpin);
	struct pin *p = NULL;

	ck->pins = xrealloc(ck->pins, (ck->npins + 1) * sizeof *ck->pins);
	p = &ck->pins[ck->npins++];
	p->ref = xmalloc(n + 1);
	memcpy(p->ref, refpin, n);
	p->ref[n] = 0;
	p->net = NULL;
	return p;
}

static void fault(struct check *ck, const char *msg)
{
	printf("%s: %s\n", ck->file, msg);
	ck->faults = 1;
}

// The first pin of the net record w[0] .. w[n-1], past its words global
// and class CLASS; a second class is a fault of the line.
static size_t first_pin(struct check *ck, char **w, size_t n)
{
	size_t first = 2;
	const char *c = NULL;

	if (first < n && !strcmp(w[first], "global")) {
		first++;
	}
	for (; first + 1 < n && !strcmp(w[first], "class"); first += 2) {
		if (c && strcmp(c, w[first + 1]) != 0) {
			printf("%s:%d: net %.1000s is of class %.1000s and of "
			       "class %.1000s\n",
			       ck->file, ck->line, w[1], c, w[first + 1]);
			ck->faults = 1;
		}
		c = w[first + 1];
	}
	return first;
}

// Takes the net record w[0] .. w[n-1]: the net, and its pins.
static void read_net(struct check *ck, char **w, size_t n)
{
	size_t first = first_pin(ck, w, n);
	char *name = xstrdup(w[1]);

	ck->nets = (char **)xrealloc((void *)ck->nets,
				     (ck->nnets + 1) * sizeof *ck->nets);
	ck->nets[ck->nnets++] = name;
	if (n - first == 1) {
		char msg[MSG_LEN];

		xsnprintf(msg, "net %.1000s has one pin alone, %.1000s", name,
			  w[first]);
		fault(ck, msg);
	}
	for (size_t i = first; i < n; i++) {
		add_pin(ck, w[i])->net = name;
	}
}

// Reads the netlist: its nets and their pins, and the pins on none.
static void read_netlist(struct check *ck, FILE *in)
{
	char *buf = xmalloc(LINE);
	char **w = (char **)xmalloc(WORDS * sizeof *w);

	while (fgets(buf, LINE, in)) {
		size_t n = split(buf, w);

		ck->line++;
		if (n >= 2 && !strcmp(w[0], "net")) {
			read_net(ck, w, n);
		} else if (n == 3 && !strcmp(w[0], "pin")) {
			if (strcmp(w[2], "nc") != 0) {
				char msg[MSG_LEN];

				xsnprintf(msg,
					  "%.1000s is on no net, and not "
					  "marked nc",
					  w[1]);
				fault(ck, msg);
			}
			add_pin(ck, w[1]);
		}
	}
	free(buf);
	free((void *)w);
}

// Each part of two pins or more, all of them on one net.
static void check_parts(struct check *ck)
{
	const struct pin *pins = ck->pins;
	char *done = xmalloc(ck->npins + 1);

	memset(done, 0, ck->npins + 1);
	for (size_t i = 0; i < ck->npins; i++) {
		size_t count = 0;
		int one = 1;

		if (done[i]) {
			continue;
		}
		for (size_t j = i; j < ck->npins; j++) {
			if (strcmp(pins[j].ref, pins[i].ref) != 0) {
				continue;
			}
			done[j] = 1;
			count++;
			one = one && pins[i].net && pins[j].net &&
			      !strcmp(pins[j].net, pins[i].net);
		}
		if (count >= 2 && one) {
			char msg[MSG_LEN];

			xsnprintf(msg,
				  "%.1000s has all its pins on net %.1000s",
				  pins[i].ref, pins[i].net);
			fault(ck, msg);
		}
	}
	free(done);
}

int main(int argc, char **argv)
{
	struct check ck = { NULL, 0, NULL, 0, "stdin", 0, 0 };

	if (argc > 2 || (argc == 2 && argv[1][0] == '-')) {
		(void)fprintf(stderr, "usage: schcheck [FILE.nl]\n");
		return 2;
	}
	if (argc == 2) {
		FILE *in = fopen(argv[1], "r");

		if (!in) {
			(void)fprintf(stderr, "%s: cannot read\n", argv[1]);
			return 2;
		}
		ck.file = argv[1];
		read_netlist(&ck, in);
		(void)fclose(in);
	} else {
		read_netlist(&ck, stdin);
	}
	check_parts(&ck);
	for (size_t i = 0; i < ck.npins; i++) {
		free(ck.pins[i].ref);
	}
	free(ck.pins);
	for (size_t i = 0; i < ck.nnets; i++) {
		free(ck.nets[i]);
	}
	free((void *)ck.nets);
	return ck.faults;
}
