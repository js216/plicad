// SPDX-License-Identifier: GPL-3.0-only
// budget.c -- Run a case of a test and hold it to its budget
// Copyright (c) 2026 Jakob Kastelic

// Usage: budget FILE COMMAND [ARG...]
//        budget -v FILE NAME KEY VALUE [KEY VALUE...]
//
// Runs COMMAND, a case of a test whose name is in the environment as
// name, and measures it as it ends: the peak resident set of it and its
// children (rss, in KB) and the time they took on the processor, user
// and system (cpu, in s), by getrusage.  With -v it takes the values
// given instead, as the lines and the bytes of a program.  FILE holds
// the budgets, one case a line, the values and then the name:
//   KEY VALUE [KEY VALUE...] -- NAME...
// and the margins of all of them, which it states:
//   margin KEY FACTOR [KEY FACTOR...] [slack SECONDS]
// A value over its budget times its factor, and the time over that and
// the slack too, fails the case, with a line on the standard error; a
// case FILE has no line of is new, and its values go to the end of FILE
// as its budget.  Budgets are never changed but by hand.  The exit
// status is that of COMMAND, or 1 for a value over its budget.  Of a
// name, its first NAME_LEN characters count, each blank a space.

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

enum {
	LINE_LEN = 1024, // The longest line of a budget file.
	NAME_LEN = 400, // The most of a name that counts.
	KEYS = 8 // The most values of a case.
};

// The values of a case, or the margins: KEYS at most, by their keys.
struct values {
	char key[KEYS][32];
	double v[KEYS];
	size_t n;
	double slack;
};

static void die(const char *what, const char *arg)
{
	(void)fprintf(stderr, "budget: %s%s%s\n", what, arg ? ": " : "",
		      arg ? arg : "");
	exit(2);
}

// Reads the pairs KEY VALUE of the words of line s into x, and returns
// the rest after --, the name, or NULL if there is none.
static char *pairs(char *s, struct values *x)
{
	char *end = NULL;

	x->n = 0;
	for (;;) {
		char key[32];
		int used = 0;
		double v = 0;

		while (*s == ' ' || *s == '\t') {
			s++;
		}
		if (!strncmp(s, "-- ", strlen("-- "))) {
			s += strlen("-- ");
			break;
		}
		if (sscanf(s, "%31s %lf%n", key, &v, &used) != 2 ||
		    x->n == KEYS) {
			break;
		}
		if (!strcmp(key, "slack")) {
			x->slack = v;
		} else {
			memcpy(x->key[x->n], key, sizeof(key));
			x->v[x->n++] = v;
		}
		s += used;
	}
	end = s + strlen(s);
	while (end > s && (end[-1] == '\n' || end[-1] == ' ')) {
		*--end = 0;
	}
	return *s ? s : NULL;
}

// The factor of the key in the margins m, or 0 for none.
static double factor(const struct values *m, const char *key)
{
	for (size_t i = 0; i < m->n; i++) {
		if (!strcmp(m->key[i], key)) {
			return m->v[i];
		}
	}
	return 0;
}

// Holds the values got of the case name to its budget in file path, or
// records them as its budget if it has none; returns 0, or 1 if a value
// is over its budget.
static int hold(const char *path, const char *name, const struct values *got)
{
	FILE *f = fopen(path, "r");
	char line[LINE_LEN];
	struct values m;
	int found = 0;
	int over = 0;

	memset(&m, 0, sizeof(m));
	if (!f) {
		die("cannot read the budgets", path);
	}
	while (fgets(line, sizeof(line), f)) {
		struct values b;
		char *rest = NULL;

		memset(&b, 0, sizeof(b));
		if (line[0] == '#' || line[0] == '\n') {
			continue;
		}
		if (!strncmp(line, "margin ", strlen("margin "))) {
			(void)pairs(line + strlen("margin "), &m);
			continue;
		}
		rest = pairs(line, &b);
		if (!rest || strcmp(rest, name) != 0) {
			continue;
		}
		found = 1;
		for (size_t i = 0; i < b.n; i++) {
			double k = factor(&m, b.key[i]);
			double lim = b.v[i] * k;

			if (!strcmp(b.key[i], "cpu")) {
				lim += m.slack;
			}
			for (size_t j = 0; k > 0 && j < got->n; j++) {
				if (!strcmp(got->key[j], b.key[i]) &&
				    got->v[j] > lim) {
					(void)fprintf(stderr,
						      "budget: %s: %s %g over "
						      "%g, the budget %g "
						      "with its margin\n",
						      name, b.key[i], got->v[j],
						      lim, b.v[i]);
					over = 1;
				}
			}
		}
	}
	(void)fclose(f);
	if (!found) {
		f = fopen(path, "a");
		if (!f) {
			die("cannot add to the budgets", path);
		}
		for (size_t j = 0; j < got->n; j++) {
			(void)fprintf(f, "%s %.6g ", got->key[j], got->v[j]);
		}
		(void)fprintf(f, "-- %s\n", name);
		if (fclose(f) != 0) {
			die("cannot add to the budgets", path);
		}
	}
	return over;
}

// Runs the command argv and measures it into got; returns its status.
static int run(char **argv, struct values *got)
{
	pid_t pid = fork();
	int st = 0;
	struct rusage ru;

	if (pid < 0) {
		die("cannot fork", NULL);
	}
	if (!pid) {
		(void)execvp(argv[0], argv);
		(void)fprintf(stderr, "budget: cannot run %s: %s\n", argv[0],
			      strerror(errno));
		_exit(127);
	}
	while (waitpid(pid, &st, 0) < 0) {
		if (errno != EINTR) {
			die("cannot wait", NULL);
		}
	}
	(void)getrusage(RUSAGE_CHILDREN, &ru);
	memset(got, 0, sizeof(*got));
	memcpy(got->key[0], "rss", sizeof("rss"));
	got->v[0] = (double)ru.ru_maxrss;
	memcpy(got->key[1], "cpu", sizeof("cpu"));
	got->v[1] = (double)ru.ru_utime.tv_sec +
		    ((double)ru.ru_utime.tv_usec / 1e6) +
		    (double)ru.ru_stime.tv_sec +
		    ((double)ru.ru_stime.tv_usec / 1e6);
	got->n = 2;
	if (WIFEXITED(st)) {
		return WEXITSTATUS(st);
	}
	return 1;
}

int main(int argc, char **argv)
{
	struct values got;
	const char *env = getenv("name");
	char name[NAME_LEN + 1];
	int st = 0;
	size_t n = 0;

	memset(&got, 0, sizeof(got));
	if (argc >= 6 && !strcmp(argv[1], "-v") && argc % 2 == 0) {
		for (int i = 4; i + 1 < argc && got.n < KEYS; i += 2) {
			char *end = NULL;

			if (strlen(argv[i]) >= sizeof(got.key[0])) {
				die("a key is too long", argv[i]);
			}
			memcpy(got.key[got.n], argv[i], strlen(argv[i]) + 1);
			got.v[got.n++] = strtod(argv[i + 1], &end);
			if (*end || end == argv[i + 1]) {
				die("a value is no number", argv[i + 1]);
			}
		}
		return hold(argv[2], argv[3], &got);
	}
	if (argc < 3 || argv[1][0] == '-') {
		die("usage: budget FILE COMMAND [ARG...] or budget -v FILE "
		    "NAME KEY VALUE...",
		    NULL);
	}
	if (!env || !*env) {
		die("no name of the case in the environment", NULL);
	}
	for (; env[n] && n < NAME_LEN; n++) {
		name[n] = env[n] == '\n' || env[n] == '\t' || env[n] == '\r' ?
				  ' ' :
				  env[n];
	}
	while (n && name[n - 1] == ' ') {
		n--;
	}
	name[n] = 0;
	st = run(argv + 2, &got);
	return hold(argv[1], name, &got) ? 1 : st;
}
