// SPDX-License-Identifier: GPL-3.0-only
// stream.c -- The text that runs down a pipe of tools: solids and source
// Copyright (c) 2026 Jakob Kastelic

#include "stream.h"
#include "expr.h"
#include "fold.h"
#include "geom.h"
#include "insert.h"
#include "shape.h"
#include "util.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char magic[] = "PliCad";

enum {
	HEAD_WORDS = 2, // The words of a record outside a part.
	DECIMAL = 10, // The base of the line numbers.
	HEAD_LINE = 256 // Room for a line before the first statement.
};

int stream_file(const char *path)
{
	FILE *f = fopen(path, "r");
	char l[HEAD_LINE];
	int stream = 1;

	while (f && fgets(l, sizeof(l), f)) {
		const char *w = l + strspn(l, " \t\r\n");

		if (*w && *w != '#') {
			size_t n = strlen(magic);

			stream = strncmp(w, magic, n) == 0 &&
				 (w[n] == 0 || strchr(" \t\r\n#", w[n]));
			break;
		}
	}
	if (f) {
		(void)fclose(f);
	}
	return stream;
}

static void add_line(struct stream *st, unsigned no, const char *text)
{
	struct sline l = { no, xstrdup(text), 0 };

	PUSH(st->ln, st->nln, st->capln, l);
}

// Reads the lines of a source file (source_lines).
static void read_source(struct stream *st, char *text)
{
	struct src_line *ls = NULL;
	size_t n = source_lines(text, &ls);

	for (size_t i = 0; i < n; i++) {
		add_line(st, ls[i].no, ls[i].text);
	}
	free_source_lines(ls, n);
}

// Reads a source line @N TEXT of a stream.
static void source_record(struct stream *st, char *l, const char *where)
{
	char *end = NULL;
	unsigned long k = strtoul(l + 1, &end, DECIMAL);

	if (end == l + 1 || *end != ' ') {
		die("%s: bad source line", where);
	}
	add_line(st, (unsigned)k, end + 1);
}

// Reads a material record of a stream.
static void material_record(struct stream *st, char *l, const char *where)
{
	struct material m;

	read_material(&m, l, where);
	for (size_t i = 0; i < st->nmats; i++) {
		if (strcmp(st->mats[i].name, m.name) == 0) {
			die("%s: a second material %s", where, m.name);
		}
	}
	PUSH(st->mats, st->nmats, st->capmats, m);
}

// Opens part name of a stream.
static void part_record(struct stream *st, const char *name, const char *where)
{
	for (size_t i = 0; i < st->nps; i++) {
		if (strcmp(st->ps[i].name, name) == 0) {
			die("%s: a second part %s", where, name);
		}
	}
	{
		struct part p = new_part(name);

		PUSH(st->ps, st->nps, st->capps, p);
	}
}

// Reads one line of a stream outside a part: a header, the head of a
// part, or a source line; returns whether a part opens.
static int stream_record(struct stream *st, char *l, const char *where)
{
	char *w[HEAD_WORDS + 1];
	char *copy = NULL;
	size_t n = 0;

	if (l[0] == '@') {
		source_record(st, l, where);
		return 0;
	}
	if (strncmp(l, "material ", strlen("material ")) == 0) {
		material_record(st, l, where);
		return 0;
	}
	copy = xstrdup(l);
	n = split_words(copy, w, HEAD_WORDS);
	if (n == 2 && strcmp(w[0], "source") == 0) {
		free(st->src);
		st->src = xstrdup(w[1]);
	} else if (n == 2 && strcmp(w[0], "part") == 0) {
		part_record(st, w[1], where);
		free(copy);
		return 1;
	} else if (n) {
		die("%s: unknown record %s", where, w[0]);
	}
	free(copy);
	return 0;
}

static void read_stream(struct stream *st, char *text, const char *path)
{
	char *rest = text;
	char *l = next_line(&rest);
	char where[WHERE_LEN];
	int in_part = 0;
	unsigned no = 1;

	if (strcmp(l, magic) != 0) {
		die("%s:1: a stream begins with the line %s", path, magic);
	}
	while ((l = next_line(&rest)) != NULL) {
		no++;
		xsnprintf(where, "%s:%u", path, no);
		if (in_part) {
			in_part = part_line(&st->ps[st->nps - 1], l, where);
		} else if (*l) {
			in_part = stream_record(st, l, where);
		}
	}
	if (in_part) {
		die("%s: part %s has no end", path, st->ps[st->nps - 1].name);
	}
}

static char *read_all(const char *path)
{
	char *buf = NULL;
	size_t n = 0;
	size_t cap = 0;
	int c = 0;

	if (strcmp(path, "-") != 0) {
		return read_file(path);
	}
	while ((c = getchar()) != EOF) {
		PUSH(buf, n, cap, (char)c);
	}
	PUSH(buf, n, cap, 0);
	return buf;
}

// Whether text begins with the line of the magic.
static int is_stream(const char *text)
{
	size_t n = strlen(magic);

	return strncmp(text, magic, n) == 0 &&
	       (text[n] == 0 || text[n] == '\n' || text[n] == '\r');
}

// Reads the stream or source file at path into st.
static void read_any(struct stream *st, const char *path, int source_ok)
{
	char *text = read_all(path);
	const char *name = strcmp(path, "-") == 0 ? "stdin" : path;

	memset(st, 0, sizeof(*st));
	if (is_stream(text)) {
		read_stream(st, text, name);
	} else if (source_ok) {
		st->src = xstrdup(path);
		read_source(st, text);
	} else {
		die("%s: not a stream (stream(5))", name);
	}
	free(text);
}

// Whether m leaves every point where it is.
static int at_home(const struct rigid *m)
{
	double d = fabs(m->t.x) + fabs(m->t.y) + fabs(m->t.z);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			d += fabs(m->r[i][j] - (i == j ? 1 : 0));
		}
	}
	return !(d > 0);
}

void stream_read(struct stream *st, const char *path)
{
	read_any(st, path, 1);
	if (!st->src) {
		die("%s: a stream with no source", path);
	}
	if (st->nmats) {
		die("%s: a pipe takes no material", path);
	}
	for (size_t i = 0; i < st->nps; i++) {
		struct part *p = &st->ps[i];

		if (!p->solid || p->base || p->ncoats || !at_home(&p->place)) {
			die("%s: part %s of a pipe is not a bare solid", path,
			    p->name);
		}
		PUSH(st->sh, st->nsh, st->capsh, *p->solid);
		free(p->solid);
		p->solid = NULL;
		clear_part(p);
	}
	free(st->ps);
	st->ps = NULL;
	st->nps = st->capps = 0;
}

void stream_head(void)
{
	printf("%s\n", magic);
}

void stream_write(const struct stream *st)
{
	stream_head();
	printf("source %s\n", st->src);
	for (size_t i = 0; i < st->nsh; i++) {
		printf("part %s\n", st->sh[i].name);
		shape_write(stdout, &st->sh[i]);
		printf("end\n");
	}
	for (size_t i = 0; i < st->nln; i++) {
		if (!st->ln[i].gone) {
			printf("@%u %s\n", st->ln[i].no, st->ln[i].text);
		}
	}
}

// Adds the material of the name of stream st to part p, unless it has
// it; dies if the stream has none of that name and need is set.
static void part_takes(struct part *p, const struct stream *st,
		       const char *name, const char *path, int need)
{
	struct material m;
	const struct material *f = NULL;

	if (part_material(p, name)) {
		return;
	}
	for (size_t i = 0; i < st->nmats && !f; i++) {
		if (strcmp(st->mats[i].name, name) == 0) {
			f = &st->mats[i];
		}
	}
	if (!f && !need) {
		return;
	}
	if (!f) {
		die("%s: part %s: no material %s", path, p->name, name);
	}
	m = *f;
	m.name = xstrdup(f->name);
	PUSH(p->mats, p->nmats, p->capmats, m);
}

// Gives part p a copy of finish f.
static void add_finish(struct part *p, const struct face_finish *f)
{
	struct face_finish c = { xstrdup(f->material), NULL, f->n };

	c.refs = xmalloc((f->n + 1) * sizeof(*c.refs));
	for (size_t k = 0; k < f->n; k++) {
		c.refs[k].s = xstrdup(f->refs[k].s);
	}
	PUSH(p->fins, p->nfins, p->capfins, c);
}

const char *finish_of(const struct face_finish *f, size_t n, const char *face)
{
	const char *m = NULL;

	for (size_t i = 0; i < n; i++) {
		for (size_t k = 0; k < f[i].n; k++) {
			struct face_ref r = { NULL, f[i].refs[k].s };

			if (stream_face(r, face)) {
				m = f[i].material;
			}
		}
	}
	return m;
}

const struct face_thread *thread_of(const struct face_thread *t, size_t n,
				    const char *face)
{
	const struct face_thread *found = NULL;

	for (size_t i = 0; i < n; i++) {
		struct face_ref r = { NULL, t[i].ref };

		if (stream_face(r, face)) {
			found = &t[i];
		}
	}
	return found;
}

// Gives part p a copy of thread t.
static void add_thread(struct part *p, const struct face_thread *t)
{
	struct face_thread c = { xstrdup(t->ref), xstrdup(t->spec), t->deep };

	PUSH(p->threads, p->nthreads, p->capthreads, c);
}

const struct material *stream_material(const struct stream *st,
				       const char *name)
{
	for (size_t i = 0; name && i < st->nmats; i++) {
		if (strcmp(st->mats[i].name, name) == 0) {
			return &st->mats[i];
		}
	}
	return NULL;
}

struct material lib_material(const char *path, const char *name)
{
	struct sibling sb = { path, "materials" };
	char *file = lib_path(sb, "");
	char *text = read_file(file);
	struct src_line *ls = NULL;
	size_t n = source_lines(text, &ls);
	struct material m;
	int found = 0;

	memset(&m, 0, sizeof(m));
	for (size_t i = 0; i < n && !found; i++) {
		char where[WHERE_LEN];
		struct material x;

		xsnprintf(where, "%s:%u", file, ls[i].no);
		read_material(&x, ls[i].text, where);
		found = strcmp(x.name, name) == 0;
		if (found) {
			m = x;
		} else {
			free(x.name);
		}
	}
	free_source_lines(ls, n);
	free(text);
	if (!found) {
		die("%s: no material %s in %s", path, name, file);
	}
	free(file);
	return m;
}

// The body of insert in of folded part p, read from path, as its record
// gives it: a round bar that stands on its side of the sheet over its
// hole, and fills the hole too if it says so, through the sheet or to
// the floor of a blind hole; of its material, which the stream gives.
static struct part insert_body(const struct stream *st, const struct part *p,
			       const struct part_insert *in, const char *path)
{
	const struct hole *h = &p->holes[in->hole];
	char *name = join_names(p->name, in->name);
	struct part b = new_part(name);
	struct bar c;
	double sunk = h->blind > 0 ? h->blind : p->t;

	free(name);
	memset(&c, 0, sizeof(c));
	c.o = fmap(p, h->face, h->cx, h->cy, in->side > 0 ? 0 : -p->t);
	c.n = mul(fnormal(p, h->face), in->side);
	if (in->fills) {
		c.r[c.ns] = h->r;
		c.z[c.ns] = -sunk;
		c.z[++c.ns] = 0;
	}
	if (in->body_h > 0) {
		c.r[c.ns] = in->body_d / 2;
		c.z[c.ns] = 0;
		c.z[++c.ns] = in->body_h;
	}
	if (c.ns == 2 && !(fabs(c.r[0] - c.r[1]) > SH_EPS)) {
		c.ns = 1;
		c.z[1] = in->body_h;
	}
	b.solid = xmalloc(sizeof(*b.solid));
	bar_shape(b.solid, c, b.name);
	b.place = p->place;
	b.body = 1;
	if (in->material) {
		part_takes(&b, st, in->material, path, 1);
		set_base(&b, part_material(&b, in->material));
	}
	return b;
}

// Puts the bodies of the inserts of each part of st, read from path,
// after it.
// A list of parts under way.
struct part_list {
	struct part *ps;
	size_t n, cap;
};

static void add_part(struct part_list *l, struct part p)
{
	PUSH(l->ps, l->n, l->cap, p);
}

static void insert_bodies(struct stream *st, const char *path)
{
	struct part_list l = { NULL, 0, 0 };

	for (size_t i = 0; i < st->nps; i++) {
		struct part *p = &st->ps[i];
		size_t at = l.n;
		size_t nb = p->bodied ? 0 : p->nins;

		add_part(&l, *p);
		for (size_t k = 0; k < nb; k++) {
			add_part(&l, insert_body(st, p, &p->ins[k], path));
		}
		l.ps[at].bodied = 1;
	}
	free(st->ps);
	st->ps = l.ps;
	st->nps = l.n;
	st->capps = l.cap;
}

size_t read_parts(const char *path, struct part **out)
{
	struct stream st;
	size_t n = 0;

	read_any(&st, path, 0);
	if (st.nln) {
		if (!st.src) {
			die("%s: source lines with no source", path);
		}
		stream_refuse_left(&st);
	}
	for (size_t i = 0; i < st.nps; i++) {
		struct part *p = &st.ps[i];

		if (p->base) {
			char *b = xstrdup(p->base);

			part_takes(p, &st, b, path, 1);
			set_base(p, part_material(p, b));
			free(b);
		}
		for (size_t k = 0; k < p->ncoats; k++) {
			part_takes(p, &st, p->coats[k].material, path, 0);
		}
		for (size_t k = 0; p->solid && k < st.nfins; k++) {
			add_finish(p, &st.fins[k]);
		}
		for (size_t k = 0; k < p->nfins; k++) {
			part_takes(p, &st, p->fins[k].material, path, 0);
		}
		for (size_t k = 0; p->solid && k < st.nthreads; k++) {
			add_thread(p, &st.threads[k]);
		}
	}
	insert_bodies(&st, path);
	*out = st.ps;
	n = st.nps;
	st.ps = NULL;
	st.nps = 0;
	stream_free(&st);
	return n;
}

void stream_free(struct stream *st)
{
	for (size_t i = 0; i < st->nsh; i++) {
		shape_free(&st->sh[i]);
	}
	for (size_t i = 0; i < st->nln; i++) {
		free(st->ln[i].text);
	}
	free(st->sh);
	free_parts(st->ps, st->nps);
	for (size_t i = 0; i < st->nsets; i++) {
		for (size_t k = 0; k < st->sets[i].n; k++) {
			free(st->sets[i].refs[k].s);
		}
		free(st->sets[i].refs);
		free(st->sets[i].name);
	}
	free(st->sets);
	for (size_t i = 0; i < st->nfins; i++) {
		free_finish(&st->fins[i]);
	}
	free(st->fins);
	for (size_t i = 0; i < st->nthreads; i++) {
		free_thread(&st->threads[i]);
	}
	free(st->threads);
	for (size_t i = 0; i < st->nmats; i++) {
		free(st->mats[i].name);
	}
	free(st->mats);
	free(st->ln);
	free(st->src);
	free(st->buf);
	free_names(&st->nm);
	memset(st, 0, sizeof(*st));
}

// Reads the parameter file of a line include FILE, next to the source,
// as smfold does; returns 0 if the line is none.
static int include_line(struct stream *st)
{
	char *path = NULL;

	if (st->nw < 1 || strcmp(st->w[0], "include") != 0) {
		return 0;
	}
	if (st->nw != 2) {
		stream_fail(st, "include FILE", NULL);
	}
	path = sibling_path((struct sibling){ st->src, st->w[1] });
	read_params(&st->nm, path);
	free(path);
	return 1;
}

// Evaluates a line NAME = EXPR; returns 0 if it is none.
static int define_line(struct stream *st, const char *text)
{
	const char *eq = strchr(text, '=');
	size_t a = strspn(text, " \t");
	size_t e = a;
	char name[ARGS_MAX];

	while (text[e] == '_' || isalnum((unsigned char)text[e])) {
		e++;
	}
	if (!eq || e == a || e - a >= sizeof(name) ||
	    eq != text + e + strspn(text + e, " \t")) {
		return 0;
	}
	memcpy(name, text + a, e - a);
	name[e - a] = 0;
	define(&st->nm, name, eval(&st->nm, eq + 1), 0);
	return 1;
}

static const struct face_set *find_set(const struct stream *st,
				       const char *name)
{
	for (size_t i = 0; i < st->nsets; i++) {
		if (strcmp(st->sets[i].name, name) == 0) {
			return &st->sets[i];
		}
	}
	return NULL;
}

// Adds the reference ref to the set fs, of room cap.
static void add_ref(struct face_set *fs, size_t *cap, const char *ref)
{
	struct str r = { xstrdup(ref) };

	PUSH(fs->refs, fs->n, *cap, r);
}

// Reads a line faces NAME REF...; returns 0 if the line is none.  The
// references that name sets go in as the references of those.
static int faces_line(struct stream *st)
{
	struct face_set fs = { NULL, NULL, 0 };
	size_t cap = 0;

	if (st->nw < 1 || strcmp(st->w[0], "faces") != 0) {
		return 0;
	}
	if (st->nw < 3) {
		stream_fail(st, "faces NAME REF...", NULL);
	}
	if (find_set(st, st->w[1]) || strcmp(st->w[1], "any") == 0) {
		stream_fail(st, "a set of faces of that name exists", st->w[1]);
	}
	for (size_t i = 2; i < st->nw; i++) {
		const struct face_set *in = find_set(st, st->w[i]);

		for (size_t k = 0; in && k < in->n; k++) {
			add_ref(&fs, &cap, in->refs[k].s);
		}
		if (!in) {
			add_ref(&fs, &cap, st->w[i]);
		}
	}
	fs.name = xstrdup(st->w[1]);
	PUSH(st->sets, st->nsets, st->capsets, fs);
	return 1;
}

// Reads a line finish MATERIAL REF...; returns 0 if the line is none.
// The references that name sets go in as the references of those.
static int finish_line(struct stream *st)
{
	struct face_set fs = { NULL, NULL, 0 };
	struct face_finish f = { NULL, NULL, 0 };
	size_t cap = 0;

	if (st->nw < 1 || strcmp(st->w[0], "finish") != 0) {
		return 0;
	}
	if (st->nw < 3) {
		stream_fail(st, "finish MATERIAL REF...", NULL);
	}
	for (size_t i = 2; i < st->nw; i++) {
		const struct face_set *in = find_set(st, st->w[i]);

		for (size_t k = 0; in && k < in->n; k++) {
			add_ref(&fs, &cap, in->refs[k].s);
		}
		if (!in) {
			add_ref(&fs, &cap, st->w[i]);
		}
	}
	f.material = xstrdup(st->w[1]);
	f.refs = fs.refs;
	f.n = fs.n;
	PUSH(st->fins, st->nfins, st->capfins, f);
	return 1;
}

// Reads a line thread REF [deep D] DESIGNATION...; returns 0 if the
// line is none.  A reference that names a set gives a thread for each
// of its references.
static int thread_line(struct stream *st)
{
	struct face_thread t = { NULL, NULL, 0 };
	const struct face_set *in = NULL;
	size_t i = 2;
	size_t n = 1;

	if (st->nw < 1 || strcmp(st->w[0], "thread") != 0) {
		return 0;
	}
	if (st->nw > 3 && strcmp(st->w[2], "deep") == 0) {
		t.deep = stream_num(st, 3);
		i = 4;
		if (!(t.deep > 0)) {
			stream_fail(st, "the depth of a thread is above 0",
				    st->w[3]);
		}
	}
	if (st->nw <= i) {
		stream_fail(st, "thread REF [deep D] DESIGNATION...", NULL);
	}
	for (size_t k = i; k < st->nw; k++) {
		n += strlen(st->w[k]) + 1;
	}
	in = find_set(st, st->w[1]);
	for (size_t r = 0; r < (in ? in->n : 1); r++) {
		size_t at = 0;

		t.ref = xstrdup(in ? in->refs[r].s : st->w[1]);
		t.spec = xmalloc(n);
		for (size_t k = i; k < st->nw; k++) {
			size_t m = strlen(st->w[k]);

			if (at) {
				t.spec[at++] = ' ';
			}
			memcpy(t.spec + at, st->w[k], m);
			at += m;
		}
		t.spec[at] = 0;
		PUSH(st->threads, st->nthreads, st->capthreads, t);
	}
	return 1;
}

// Adds material m to st, unless st has one of that name, which must be
// the same.
static void add_material(struct stream *st, struct material m)
{
	for (size_t i = 0; i < st->nmats; i++) {
		const struct material *e = &st->mats[i];

		if (strcmp(e->name, m.name) == 0) {
			if (materials_differ(e, &m)) {
				stream_fail(st,
					    "a second material of that name",
					    m.name);
			}
			free(m.name);
			return;
		}
	}
	PUSH(st->mats, st->nmats, st->capmats, m);
}

// Reads a line material NAME R G B ROUGHNESS DENSITY [FINISH], or
// materials FILE of such lines; returns 0 if the line is none.
static int material_line(struct stream *st, const char *text)
{
	if (st->nw < 1 || (strcmp(st->w[0], "material") != 0 &&
			   strcmp(st->w[0], "materials") != 0)) {
		return 0;
	}
	if (st->w[0][strlen("material")] == 's') {
		char *path = NULL;
		char *file = NULL;
		struct src_line *ls = NULL;
		size_t n = 0;

		if (st->nw != 2) {
			stream_fail(st, "materials FILE", NULL);
		}
		path = lib_path((struct sibling){ st->src, st->w[1] }, "");
		file = read_file(path);
		n = source_lines(file, &ls);
		for (size_t i = 0; i < n; i++) {
			struct material m;
			char where[WHERE_LEN];

			xsnprintf(where, "%s:%u", path, ls[i].no);
			read_material(&m, ls[i].text, where);
			add_material(st, m);
		}
		free_source_lines(ls, n);
		free(file);
		free(path);
		return 1;
	}
	{
		struct material m;
		char *copy = xstrdup(text);

		read_material(&m, copy, st->where);
		free(copy);
		add_material(st, m);
	}
	return 1;
}

// Whether the part c, n long, of the name of a face, matches ref.
static int part_face(const char *ref, const char *c, size_t n)
{
	const char *dot_at = strchr(ref, '.');
	size_t k = strlen(ref);
	size_t a = 0;
	size_t side = strlen(".side");

	if (!dot_at) {
		return (n == k || (n > k && c[k] == '.')) &&
		       strncmp(c, ref, k) == 0;
	}
	if (!strcmp(dot_at, ".start") || !strcmp(dot_at, ".end")) {
		return n == k && strncmp(c, ref, k) == 0;
	}
	if (!strcmp(dot_at, ".side")) {
		return (n == k || (n > k && c[k] == '.')) &&
		       strncmp(c, ref, k) == 0;
	}
	a = (size_t)(dot_at - ref);
	return n == k + side && strncmp(c, ref, a) == 0 &&
	       strncmp(c + a, ".side", side) == 0 &&
	       strncmp(c + a + side, dot_at, k - a) == 0;
}

// Whether r.ref, any or a name of the source, matches face: one of its
// parts between + signs.
static int name_face(struct face_ref r, const char *face)
{
	if (strcmp(r.ref, "any") == 0) {
		return 1;
	}
	for (const char *c = face; *c;) {
		size_t n = strcspn(c, "+");

		if (part_face(r.ref, c, n)) {
			return 1;
		}
		c += n + (c[n] == '+');
	}
	return 0;
}

int stream_face(struct face_ref r, const char *face)
{
	const struct face_set *fs = r.st ? find_set(r.st, r.ref) : NULL;

	for (size_t i = 0; fs && i < fs->n; i++) {
		struct face_ref in = { r.st, fs->refs[i].s };

		if (name_face(in, face)) {
			return 1;
		}
	}
	return !fs && name_face(r, face);
}

int stream_line(struct stream *st, size_t i)
{
	const struct sline *l = &st->ln[i];

	xsnprintf(st->where, "%s:%u", st->src, l->no);
	st->nm.where = st->where;
	free(st->buf);
	st->buf = xstrdup(l->text);
	st->nw = split_words(st->buf, st->w, ARGS_MAX);
	if (st->nw > ARGS_MAX) {
		stream_fail(st, "too many words", NULL);
	}
	if (!st->nw || include_line(st) || define_line(st, l->text) ||
	    faces_line(st) || finish_line(st) || thread_line(st) ||
	    material_line(st, l->text)) {
		return 0;
	}
	return 1;
}

double stream_num(struct stream *st, size_t i)
{
	if (i >= st->nw) {
		stream_fail(st, "a number is missing", NULL);
	}
	return eval(&st->nm, st->w[i]);
}

int stream_is(const struct stream *st, size_t i, const char *w)
{
	return i < st->nw && strcmp(st->w[i], w) == 0;
}

// The solid of that name, or NULL.
static struct shape *stream_find(struct stream *st, const char *name)
{
	for (size_t i = 0; i < st->nsh; i++) {
		if (strcmp(st->sh[i].name, name) == 0) {
			return &st->sh[i];
		}
	}
	return NULL;
}

void stream_add(struct stream *st, struct shape *sh)
{
	if (stream_find(st, sh->name)) {
		stream_fail(st, "a solid of that name exists", sh->name);
	}
	PUSH(st->sh, st->nsh, st->capsh, *sh);
	memset(sh, 0, sizeof(*sh));
}

void stream_take(struct stream *st, const char *name, struct shape *out)
{
	for (size_t i = 0; i < st->nsh; i++) {
		if (strcmp(st->sh[i].name, name) == 0) {
			*out = st->sh[i];
			for (size_t k = i + 1; k < st->nsh; k++) {
				st->sh[k - 1] = st->sh[k];
			}
			st->nsh--;
			return;
		}
	}
	stream_fail(st, "no solid of that name", name);
}

void stream_refuse_left(struct stream *st)
{
	for (size_t i = 0; i < st->nln; i++) {
		if (!st->ln[i].gone && stream_line(st, i)) {
			stream_fail(st, "no tool took this statement",
				    st->w[0]);
		}
	}
}
