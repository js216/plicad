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
//   shorted).
// The exit status is 1 on a fault, 2 if the file cannot be read.

#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LINE = 1 << 16, WORDS = 4096 };

// A pin of a part: its part's designator, and its net (NULL for none).
struct pin {
	char *ref;
	const char *net;
};

static struct pin *pins;
static size_t npins;
static char **nets;
static size_t nnets;
static const char *file;
static int faults;

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

// Notes a pin of a part, REF.PIN, on a net or none.
static void add_pin(const char *refpin, const char *net)
{
	const char *dot = strrchr(refpin, '.');
	size_t n = dot ? (size_t)(dot - refpin) : strlen(refpin);

	pins = xrealloc(pins, (npins + 1) * sizeof *pins);
	pins[npins].ref = xmalloc(n + 1);
	memcpy(pins[npins].ref, refpin, n);
	pins[npins].ref[n] = 0;
	pins[npins++].net = net;
}

static void fault(const char *msg)
{
	printf("%s: %s\n", file, msg);
	faults = 1;
}

// Reads the netlist: its nets and their pins, and the pins on none.
static void read_netlist(FILE *in)
{
	char *buf = xmalloc(LINE), **w = xmalloc(WORDS * sizeof *w), msg[2100];

	while (fgets(buf, LINE, in)) {
		size_t n = split(buf, w);

		if (n >= 2 && !strcmp(w[0], "net")) {
			size_t first = n > 2 && !strcmp(w[2], "global") ? 3 : 2;
			char *name = xstrdup(w[1]);

			nets = xrealloc(nets, (nnets + 1) * sizeof *nets);
			nets[nnets++] = name;
			if (n - first == 1) {
				snprintf(
					msg, sizeof msg,
					"net %.1000s has one pin alone, %.1000s",
					name, w[first]);
				fault(msg);
			}
			for (size_t i = first; i < n; i++) {
				add_pin(w[i], name);
			}
		} else if (n == 3 && !strcmp(w[0], "pin")) {
			if (strcmp(w[2], "nc")) {
				snprintf(
					msg, sizeof msg,
					"%.1000s is on no net, and not marked nc",
					w[1]);
				fault(msg);
			}
			add_pin(w[1], NULL);
		}
	}
	free(buf);
	free(w);
}

// Each part of two pins or more, all of them on one net.
static void check_parts(void)
{
	char *done = xmalloc(npins + 1);

	memset(done, 0, npins + 1);
	for (size_t i = 0; i < npins; i++) {
		size_t count = 0;
		int one = 1;

		if (done[i]) {
			continue;
		}
		for (size_t j = i; j < npins; j++) {
			if (strcmp(pins[j].ref, pins[i].ref)) {
				continue;
			}
			done[j] = 1;
			count++;
			one &= pins[i].net && pins[j].net &&
			       !strcmp(pins[j].net, pins[i].net);
		}
		if (count >= 2 && one) {
			char msg[2100];

			snprintf(msg, sizeof msg,
				 "%.1000s has all its pins on net %.1000s",
				 pins[i].ref, pins[i].net);
			fault(msg);
		}
	}
	free(done);
}

int main(int argc, char **argv)
{
	FILE *in = stdin;

	if (argc > 2 || (argc == 2 && argv[1][0] == '-')) {
		fprintf(stderr, "usage: schcheck [FILE.nl]\n");
		return 2;
	}
	file = argc == 2 ? argv[1] : "stdin";
	if (argc == 2 && !(in = fopen(argv[1], "r"))) {
		fprintf(stderr, "%s: cannot read\n", argv[1]);
		return 2;
	}
	read_netlist(in);
	if (in != stdin) {
		fclose(in);
	}
	check_parts();
	for (size_t i = 0; i < npins; i++) {
		free(pins[i].ref);
	}
	free(pins);
	for (size_t i = 0; i < nnets; i++) {
		free(nets[i]);
	}
	free(nets);
	return faults;
}
