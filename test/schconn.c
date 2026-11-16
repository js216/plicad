// SPDX-License-Identifier: GPL-3.0-only
// schconn.c -- Check that a drawn sheet joins what its netlist joins
// Copyright (c) 2026 Jakob Kastelic

// Usage: schconn SHEET.vec SHEET.nl
//
// A test of schdraw, from its output alone: the sheet as vec(5) and
// the netlist of schnet (netlist(5)).  It sees only what is drawn: the
// wires (the lines of the nets), the ends of the pins (pin records) and
// of the flags (flag records).  A pin is on every wire that runs through
// or ends at it; two wires join where the end of one lies on the other;
// pins at one point join; flags of one name join.  Then each net of the
// netlist must be one such group, no group may hold pins of two nets (or
// a pin of none with a pin of one), a flag joins only the net of its
// name, and no four wires may meet at a point.  Parts are known by their
// names (the path of their groups and their own, as ch1.gain.rf), never
// by designator: a pin of the drawing is named by the part bracket it
// stands in, and a pin of the netlist by the name of its part record,
// so the two may number their parts as they please.  Each fault is a line on
// stdout; the exit status is 1 if there is any, 2 if a file cannot be
// read.

#define _POSIX_C_SOURCE 200809L // For getline and strdup.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A polyline of a wire, in hundredths of a point.
struct wire {
	long *xy;
	size_t n;
};

// A pin or a flag: its name (PART.PIN by the part's name, or the net of
// a flag) and point.
struct end {
	char *name;
	long x, y;
	int flag;
};

static struct wire *wires;
static struct end *ends;
static size_t nwires, nends, *up;

static void *more(void *p, size_t n)
{
	p = realloc(p, n ? n : 1);
	if (!p) {
		fprintf(stderr, "schconn: out of memory\n");
		exit(2);
	}
	return p;
}

static long q(double v)
{
	return lround(v * 100);
}

static size_t find(size_t i)
{
	while (up[i] != i) {
		up[i] = up[up[i]];
		i = up[i];
	}
	return i;
}

static void unite(size_t a, size_t b)
{
	up[find(a)] = find(b);
}

static int on_seg(long x, long y, const long *a, const long *b)
{
	if (a[0] == b[0]) {
		return x == a[0] && y >= (a[1] < b[1] ? a[1] : b[1]) &&
		       y <= (a[1] > b[1] ? a[1] : b[1]);
	}
	if (a[1] == b[1]) {
		return y == a[1] && x >= (a[0] < b[0] ? a[0] : b[0]) &&
		       x <= (a[0] > b[0] ? a[0] : b[0]);
	}
	return 0; // Wires run along the grid.
}

static int on_wire(const struct wire *w, long x, long y)
{
	for (size_t j = 1; j < w->n; j++) {
		if (on_seg(x, y, &w->xy[2 * j - 2], &w->xy[2 * j])) {
			return 1;
		}
	}
	return w->n == 1 && w->xy[0] == x && w->xy[1] == y;
}

// The wire arms at a point: one for each wire ending there, two for each
// running through.
static int arms(long x, long y)
{
	int n = 0;

	for (size_t i = 0; i < nwires; i++) {
		const struct wire *w = &wires[i];
		int e = (w->xy[0] == x && w->xy[1] == y) +
			(w->xy[2 * w->n - 2] == x && w->xy[2 * w->n - 1] == y);

		n += e ? e : on_wire(w, x, y) ? 2 : 0;
	}
	return n;
}

// The pin of the part named name: name and the .PIN after a designator.
static char *by_name(const char *name, const char *dotpin)
{
	char *s = more(NULL, strlen(name) + strlen(dotpin) + 1);

	strcpy(s, name);
	strcat(s, dotpin);
	return s;
}

static void read_vec(const char *path)
{
	FILE *f = fopen(path, "r");
	char *line = NULL;
	size_t cap = 0;
	int in_net = 0;
	char part[1024] = "";

	if (!f) {
		fprintf(stderr, "%s: cannot read\n", path);
		exit(2);
	}
	while (getline(&line, &cap, f) > 0) {
		char name[1024], *s, *end;
		double x, y;

		line[strcspn(line, "\n")] = 0;
		if (!strncmp(line, "part", 4) && (!line[4] || line[4] == ' ')) {
			if (sscanf(line, "part %1023s", part) != 1) {
				part[0] = 0;
			}
		} else if (!strncmp(line, "net", 3) && (!line[3] || line[3] == ' ')) {
			in_net = line[3] == ' ';
		} else if (in_net && !strncmp(line, "line ", 5)) {
			struct wire w = { NULL, 0 };

			s = line + 5;
			(void)strtod(s, &end); // The width.
			s = end;
			for (;;) {
				x = strtod(s, &end);
				if (end == s) {
					break;
				}
				s = end;
				y = strtod(s, &end);
				s = end;
				w.xy = more(w.xy, 2 * (w.n + 1) * sizeof *w.xy);
				w.xy[2 * w.n] = q(x);
				w.xy[2 * w.n + 1] = q(y);
				w.n++;
			}
			if (w.n) {
				wires = more(wires, (nwires + 1) * sizeof *wires);
				wires[nwires++] = w;
			}
		} else if ((sscanf(line, "pin %1023s %lf %lf", name, &x, &y) == 3 ||
			    sscanf(line, "flag %1023s %lf %lf", name, &x, &y) ==
				    3)) {
			char *dot = strchr(name, '.');

			ends = more(ends, (nends + 1) * sizeof *ends);
			ends[nends].name = line[0] == 'p' && *part && dot ?
					   by_name(part, dot) : strdup(name);
			ends[nends].x = q(x);
			ends[nends].y = q(y);
			ends[nends++].flag = line[0] == 'f';
		}
	}
	free(line);
	fclose(f);
}

// The netlist: the net of each pin, by PART.PIN; the nets' names; the
// name of the part of each designator.
static char **pin_name, **pin_net, **nets, **refs, **names;
static size_t npins, nnets, nrefs;

static void read_netlist(const char *path)
{
	FILE *f = fopen(path, "r");
	char *line = NULL;
	size_t cap = 0;

	if (!f) {
		fprintf(stderr, "%s: cannot read\n", path);
		exit(2);
	}
	while (getline(&line, &cap, f) > 0) {
		char *save = NULL, *w = strtok_r(line, " \t\r\n", &save), *net;

		if (w && !strcmp(w, "part")) {
			char *ref = strtok_r(NULL, " \t\r\n", &save), *k;

			while (ref && (k = strtok_r(NULL, " \t\r\n", &save))) {
				if (!strcmp(k, "name") &&
				    (k = strtok_r(NULL, " \t\r\n", &save))) {
					refs = more(refs, (nrefs + 1) * sizeof *refs);
					names = more(names, (nrefs + 1) * sizeof *names);
					refs[nrefs] = strdup(ref);
					names[nrefs++] = strdup(k);
				}
			}
			continue;
		}
		if (!w || strcmp(w, "net")) {
			continue;
		}
		net = strtok_r(NULL, " \t\r\n", &save);
		if (!net) {
			continue;
		}
		nets = more(nets, (nnets + 1) * sizeof *nets);
		nets[nnets++] = strdup(net);
		while ((w = strtok_r(NULL, " \t\r\n", &save)) != NULL) {
			if (!strcmp(w, "global")) {
				continue;
			}
			pin_name = more(pin_name, (npins + 1) * sizeof *pin_name);
			pin_net = more(pin_net, (npins + 1) * sizeof *pin_net);
			pin_name[npins] = strdup(w);
			pin_net[npins++] = nets[nnets - 1];
		}
	}
	free(line);
	fclose(f);
	// Each REF.PIN becomes PART.PIN, by the part record of REF.
	for (size_t p = 0; p < npins; p++) {
		char *dot = strchr(pin_name[p], '.');
		size_t n = dot ? (size_t)(dot - pin_name[p]) : 0;

		for (size_t r = 0; dot && r < nrefs; r++) {
			if (strlen(refs[r]) == n &&
			    !strncmp(refs[r], pin_name[p], n)) {
				char *s = by_name(names[r], dot);

				free(pin_name[p]);
				pin_name[p] = s;
				break;
			}
		}
	}
}

// The net of an end: of its pin in the netlist, or of its flag; NULL
// for a pin on no net.
static const char *net_of(const struct end *e)
{
	if (e->flag) {
		return e->name;
	}
	for (size_t i = 0; i < npins; i++) {
		if (!strcmp(pin_name[i], e->name)) {
			return pin_net[i];
		}
	}
	return NULL;
}

int main(int argc, char **argv)
{
	int bad = 0;

	if (argc != 3) {
		fprintf(stderr, "usage: schconn SHEET.vec SHEET.nl\n");
		return 2;
	}
	read_vec(argv[1]);
	read_netlist(argv[2]);
	up = more(NULL, (nwires + nends + 1) * sizeof *up);
	for (size_t i = 0; i < nwires + nends; i++) {
		up[i] = i;
	}
	for (size_t i = 0; i < nwires; i++) {
		const struct wire *w = &wires[i];

		for (size_t j = 0; j < nwires; j++) {
			if (i != j && (on_wire(&wires[j], w->xy[0], w->xy[1]) ||
				       on_wire(&wires[j], w->xy[2 * w->n - 2],
					       w->xy[2 * w->n - 1]))) {
				unite(i, j);
			}
		}
	}
	for (size_t i = 0; i < nends; i++) {
		const struct end *e = &ends[i];

		for (size_t j = 0; j < nwires; j++) {
			if (on_wire(&wires[j], e->x, e->y)) {
				unite(nwires + i, j);
			}
		}
		for (size_t k = 0; k < i; k++) {
			if ((ends[k].x == e->x && ends[k].y == e->y) ||
			    (e->flag && ends[k].flag &&
			     !strcmp(ends[k].name, e->name))) {
				unite(nwires + i, nwires + k);
			}
		}
	}
	// Each pin of the netlist drawn, and each net one group.
	for (size_t n = 0; n < nnets; n++) {
		size_t g = (size_t)-1;
		const char *first = NULL;

		for (size_t p = 0; p < npins; p++) {
			size_t i;

			if (pin_net[p] != nets[n]) {
				continue;
			}
			for (i = 0; i < nends; i++) {
				if (!ends[i].flag &&
				    !strcmp(ends[i].name, pin_name[p])) {
					break;
				}
			}
			if (i == nends) {
				printf("net %s: %s is not drawn\n", nets[n],
				       pin_name[p]);
				bad = 1;
			} else if (g == (size_t)-1) {
				g = find(nwires + i);
				first = pin_name[p];
			} else if (find(nwires + i) != g) {
				printf("net %s: %s is not drawn joined to %s\n",
				       nets[n], pin_name[p], first);
				bad = 1;
			}
		}
	}
	// No group of two nets, or of a net and a pin of none.
	for (size_t i = 0; i < nends; i++) {
		for (size_t k = i + 1; k < nends; k++) {
			const char *a = net_of(&ends[i]), *b = net_of(&ends[k]);

			if ((a || b) && (!a || !b || strcmp(a, b)) &&
			    find(nwires + i) == find(nwires + k)) {
				printf("%s%s (net %s) is drawn joined to %s%s "
				       "(net %s)\n",
				       ends[i].flag ? "the flag " : "",
				       ends[i].name, a ? a : "none",
				       ends[k].flag ? "the flag " : "",
				       ends[k].name, b ? b : "none");
				bad = 1;
			}
		}
	}
	for (size_t i = 0; i < nwires; i++) {
		const struct wire *w = &wires[i];

		for (size_t j = 0; j < w->n; j += w->n > 1 ? w->n - 1 : 1) {
			if (arms(w->xy[2 * j], w->xy[2 * j + 1]) >= 4) {
				printf("four wires meet at %.2f %.2f\n",
				       (double)w->xy[2 * j] / 100,
				       (double)w->xy[2 * j + 1] / 100);
				bad = 1;
			}
		}
	}
	return bad;
}
