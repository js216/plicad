// SPDX-License-Identifier: GPL-3.0-only
// asm.c -- The lines of an assembly file and the parts they place
// Copyright (c) 2026 Jakob Kastelic

#include "asm.h"
#include "expr.h"
#include "fold.h"
#include "geom.h"
#include "stream.h"
#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	MAXWORDS = 128,
	DEPTH_MAX = 64, // The deepest nesting of subassemblies.
	MATERIAL_WORDS = 7, // material NAME R G B ROUGHNESS DENSITY
	IMAGE_WORDS = 5 + ASM_CORNERS // image NAME FILE on PART ...
};

// The reading of one file: its names, its path, the reading of the file
// it is a subassembly of and its depth there, its unit, its materials,
// the directory of its meshes, its lines so far, and its step and its
// clearance.
struct reading {
	struct names nm;
	const char *path;
	const struct reading *up;
	int depth;
	size_t unit;
	struct material *mats;
	size_t nmats, capmats;
	const char *meshdir;
	struct assembly *a;
	size_t first; // Its lines are a->ln[first ..].
	double step[2];
	int has_step;
	double clearance;
};

// The words of a line.
struct words {
	char **w;
	size_t n;
};

static double num(struct reading *r, const struct words *ws, size_t i)
{
	if (i >= ws->n) {
		expr_fail(&r->nm, "a number is missing", NULL);
	}
	return eval(&r->nm, ws->w[i]);
}

static int is(const struct words *ws, size_t i, const char *w)
{
	return i < ws->n && strcmp(ws->w[i], w) == 0;
}

static void add_material(struct reading *r, struct material m)
{
	for (size_t i = 0; i < r->nmats; i++) {
		if (strcmp(r->mats[i].name, m.name) == 0) {
			expr_fail(&r->nm, "a material of that name exists",
				  m.name);
		}
	}
	check_material(&m, expr_where(&r->nm));
	PUSH(r->mats, r->nmats, r->capmats, m);
}

// A number of the reading ctx, the expression w.
static double expr_num(void *ctx, const char *w)
{
	struct reading *r = ctx;

	return eval(&r->nm, w);
}

// Reads a statement material NAME R G B ROUGHNESS DENSITY [FINISH].
static void material_line(struct reading *r, const struct words *ws)
{
	struct material m;

	if (ws->n < MATERIAL_WORDS) {
		expr_fail(&r->nm,
			  "material NAME R G B ROUGHNESS DENSITY [FINISH]",
			  NULL);
	}
	m.name = xstrdup(ws->w[1]);
	for (int i = 0; i < 3; i++) {
		m.rgb[i] = num(r, ws, 2 + (size_t)i);
	}
	m.rough = num(r, ws, MATERIAL_WORDS - 2);
	m.density = num(r, ws, MATERIAL_WORDS - 1);
	{
		struct finish_words fw = { ws->w + MATERIAL_WORDS,
					   ws->n - MATERIAL_WORDS,
					   expr_where(&r->nm), expr_num, r };

		material_finish(&m, &fw);
	}
	add_material(r, m);
}

static const struct material *find_material(struct reading *r, const char *name)
{
	for (size_t i = 0; i < r->nmats; i++) {
		if (strcmp(r->mats[i].name, name) == 0) {
			return &r->mats[i];
		}
	}
	expr_fail(&r->nm, "no such material", name);
}

// Reads a line of a file of materials, a material statement.
static void materials_line(char *l, void *ctx)
{
	struct reading *r = ctx;
	char *w[MAXWORDS + 1];
	struct words ws = { w, split_words(l, w, MAXWORDS) };

	if (!ws.n) {
		return;
	}
	if (ws.n > MAXWORDS || strcmp(w[0], "material") != 0) {
		expr_fail(&r->nm,
			  "expected material NAME R G B ROUGHNESS DENSITY",
			  w[0]);
	}
	material_line(r, &ws);
}

// Reads a step turn x|y|z DEG [about X Y Z] at word *i, and moves *i
// past it.
static struct asm_step read_turn(struct reading *r, const struct words *ws,
				 size_t *i)
{
	const char *ax = *i + 1 < ws->n ? ws->w[*i + 1] : "";
	struct asm_step s;

	memset(&s, 0, sizeof(s));
	if (strlen(ax) != 1 || !strchr("xyz", ax[0])) {
		expr_fail(&r->nm, "turn x|y|z DEG [about X Y Z]", ax);
	}
	s.turn = 1;
	s.axis = ax[0] - 'x';
	s.deg = num(r, ws, *i + 2);
	*i += 3;
	if (is(ws, *i, "about")) {
		s.about = 1;
		s.at = v3(num(r, ws, *i + 1), num(r, ws, *i + 2),
			  num(r, ws, *i + 3));
		*i += 4;
	}
	return s;
}

// Reads the steps of a place or a path from word *i on, up to the first
// word that begins none.
static void read_steps(struct reading *r, const struct words *ws, size_t *i,
		       struct asm_steps *out)
{
	while (*i < ws->n) {
		struct asm_step s;

		memset(&s, 0, sizeof(s));
		if (is(ws, *i, "move")) {
			s.d = v3(num(r, ws, *i + 1), num(r, ws, *i + 2),
				 num(r, ws, *i + 3));
			*i += 4;
		} else if (is(ws, *i, "turn")) {
			s = read_turn(r, ws, i);
		} else {
			return;
		}
		PUSH(out->s, out->n, out->cap, s);
	}
}

// Reads the words DIST DEG of a step at word i into x; dies unless both
// are above 0.
static void read_step_size(struct reading *r, const struct words *ws, size_t i,
			   double *x)
{
	x[0] = num(r, ws, i);
	x[1] = num(r, ws, i + 1);
	if (!(x[0] > 0) || !(x[1] > 0)) {
		expr_fail(&r->nm, "step DIST DEG, both above 0", NULL);
	}
}

struct rigid asm_step_rig(const struct asm_line *l, const struct asm_step *s)
{
	struct rigid m = rig_id();

	if (!s->turn) {
		m.t = s->d;
		return m;
	}
	{
		struct line3 ax = { s->about ? s->at : l->m.t,
				    axis_vec(s->axis, 1) };

		return rig_rot(ax, s->deg);
	}
}

// The motion of the steps of a place, applied in order, the turns about
// the origin unless they name a point.
static struct rigid place_rig(const struct asm_steps *st)
{
	struct rigid m = rig_id();

	for (size_t i = 0; i < st->n; i++) {
		const struct asm_step *s = &st->s[i];
		struct rigid k = rig_id();

		if (s->turn) {
			struct line3 ax = { s->about ? s->at : v3(0, 0, 0),
					    axis_vec(s->axis, 1) };

			k = rig_rot(ax, s->deg);
		} else {
			k.t = s->d;
		}
		m = rig_mul(k, m);
	}
	return m;
}

// The line of the name in the unit of r so far, or NONE.
static size_t line_named(const struct reading *r, const char *name)
{
	const struct assembly *a = r->a;

	for (size_t i = r->first; i < a->nln; i++) {
		const struct asm_line *l = &a->ln[i];

		if (l->unit == r->unit && l->kind != ASM_END &&
		    l->kind != ASM_IMAGE && strcmp(l->name, name) == 0) {
			return i;
		}
	}
	return NONE;
}

// Reads what follows the file of part line ws into l: its place, its
// material, and its path, with its step, or the line it travels with.
static void read_tail(struct reading *r, const struct words *ws,
		      struct asm_line *l)
{
	size_t i = 3;

	read_steps(r, ws, &i, &l->place);
	if (is(ws, i, "material")) {
		const struct material *m = NULL;

		if (i + 1 >= ws->n) {
			expr_fail(&r->nm, "material MATERIAL", NULL);
		}
		m = find_material(r, ws->w[i + 1]);
		l->has_rgb = 1;
		memcpy(l->rgb, m->rgb, sizeof(l->rgb));
		i += 2;
	}
	if (is(ws, i, "path")) {
		i++;
		l->has_path = 1;
		read_steps(r, ws, &i, &l->path);
		if (!l->path.n) {
			expr_fail(&r->nm, "path STEP...", NULL);
		}
		if (is(ws, i, "step")) {
			read_step_size(r, ws, i + 1, l->step);
			l->has_step = 1;
			i += 3;
		}
	} else if (is(ws, i, "with")) {
		if (i + 2 != ws->n || line_named(r, ws->w[i + 1]) == NONE) {
			expr_fail(&r->nm, "with NAME, of a part above", NULL);
		}
		l->with = xstrdup(ws->w[i + 1]);
		i += 2;
	}
	if (i < ws->n) {
		expr_fail(&r->nm, "expected move, turn, material, path or with",
			  ws->w[i]);
	}
}

// Gives part p the material of the name, or none, and the materials of
// its coats and of the finishes of its faces.
static void set_materials(struct reading *r, struct part *p, const char *name)
{
	if (name) {
		const struct material *m = find_material(r, name);
		struct material c = *m;

		c.name = xstrdup(m->name);
		PUSH(p->mats, p->nmats, p->capmats, c);
		set_base(p, m);
	}
	for (size_t i = 0; i < p->ncoats + p->nfins; i++) {
		const char *f = i < p->ncoats ? p->coats[i].material :
						p->fins[i - p->ncoats].material;
		struct material c;

		if (part_material(p, f)) {
			continue;
		}
		c = *find_material(r, f);
		c.name = xstrdup(c.name);
		PUSH(p->mats, p->nmats, p->capmats, c);
	}
}

// The name of part p of the np parts of the file of part line name, of
// which own are not the bodies of inserts: name for the one part of its
// own, and NAME/INSERT for the bodies of its inserts, as for a stream of
// parts NAME/SUB.
static char *part_name(const char *name, const struct part *p, size_t np,
		       size_t own)
{
	const char *slash = strchr(p->name, '/');

	if (own == 1 && p->body && slash) {
		return join_names(name, slash + 1);
	}
	return np == 1 || (own == 1 && !p->body) ? xstrdup(name) :
						   join_names(name, p->name);
}

// The name of the material of part line ws, or NULL.
static const char *material_of(const struct words *ws)
{
	for (size_t i = 3; i + 1 < ws->n; i++) {
		if (is(ws, i, "material")) {
			return ws->w[i + 1];
		}
	}
	return NULL;
}

// Reads the parts of the stream of line l, part line ws, into a.
static void stream_parts(struct reading *r, const struct words *ws,
			 struct asm_line *l)
{
	struct assembly *a = r->a;
	struct part *ps = NULL;
	size_t np = read_parts(l->file, &ps);
	size_t own = 0;

	if (!np) {
		expr_fail(&r->nm, "the file holds no parts", ws->w[2]);
	}
	for (size_t i = 0; i < np; i++) {
		own += !ps[i].body;
	}
	l->first = a->nout;
	l->n = np;
	for (size_t i = 0; i < np; i++) {
		char *name = part_name(l->full, &ps[i], np, own);

		free(ps[i].name);
		ps[i].name = name;
		ps[i].place = rig_mul(l->world, ps[i].place);
		set_materials(r, &ps[i], material_of(ws));
		PUSH(a->out, a->nout, a->capout, ps[i]);
	}
	free(ps);
}

// The path p with its . and DIR/.. parts taken out, a new string, so that
// two paths of one file from one directory compare equal.
static char *plain_path(const char *p)
{
	size_t n = strlen(p);
	char *out = xmalloc(n + 2);
	size_t k = 0;
	size_t base = 0;

	if (p[0] == '/') {
		out[k++] = '/';
		base = 1;
	}
	for (const char *c = p; *c;) {
		size_t m = strcspn(c, "/");

		if (m == 2 && c[0] == '.' && c[1] == '.' && k > base &&
		    !(k - base >= 2 && out[k - 1] == '.' && out[k - 2] == '.' &&
		      (k - base == 2 || out[k - 3] == '/'))) {
			while (k > base && out[k - 1] != '/') {
				k--;
			}
			k -= k > base;
		} else if (m && !(m == 1 && c[0] == '.')) {
			if (k > base) {
				out[k++] = '/';
			}
			memcpy(out + k, c, m);
			k += m;
		}
		c += m + (c[m] == '/');
	}
	out[k] = 0;
	return out;
}

// Refuses the subassembly file of a part line of r, if r or a reading
// above it is of that file, with the chain of the files from the top.
static void check_cycle(struct reading *r, const char *file)
{
	char *f = plain_path(file);
	int cycle = 0;

	for (const struct reading *u = r; u && !cycle; u = u->up) {
		char *g = plain_path(u->path);

		cycle = strcmp(f, g) == 0;
		free(g);
	}
	free(f);
	if (!cycle && r->depth + 1 < DEPTH_MAX) {
		return;
	}
	{
		size_t n = strlen(file) + 1;
		char *chain = NULL;
		size_t k = 0;

		for (const struct reading *u = r; u; u = u->up) {
			n += strlen(u->path) + 2;
		}
		chain = xmalloc(n);
		k = n - strlen(file) - 1;
		memcpy(chain + k, file, strlen(file) + 1);
		for (const struct reading *u = r; u; u = u->up) {
			size_t m = strlen(u->path);

			k -= m + 2;
			memcpy(chain + k, u->path, m);
			chain[k + m] = ',';
			chain[k + m + 1] = ' ';
		}
		expr_fail(&r->nm,
			  cycle ? "a cycle of subassemblies" :
				  "subassemblies nest too deep",
			  chain);
	}
}

static void read_file_lines(struct reading *r);

// Reads the subassembly of unit line u, part line ws, into a: its lines
// after that of the unit, and an end line.
static void sub_lines(struct reading *r, const struct words *ws, size_t u)
{
	struct assembly *a = r->a;
	struct reading b;
	struct asm_line end;
	char *dir = NULL;

	if (material_of(ws)) {
		expr_fail(&r->nm, "a subassembly takes no material", ws->w[1]);
	}
	check_cycle(r, a->ln[u].file);
	memset(&b, 0, sizeof(b));
	b.path = a->ln[u].file;
	b.up = r;
	b.depth = r->depth + 1;
	b.unit = u;
	b.a = a;
	b.first = a->nln;
	if (r->meshdir) {
		dir = join_names(r->meshdir, ws->w[1]);
		b.meshdir = dir;
	}
	a->ln[u].first = a->nout;
	read_file_lines(&b);
	a->ln[u].n = a->nout - a->ln[u].first;
	if (!a->ln[u].n) {
		expr_fail(&r->nm, "the subassembly holds no parts", ws->w[2]);
	}
	memset(&end, 0, sizeof(end));
	end.kind = ASM_END;
	end.depth = r->depth;
	end.unit = r->unit;
	end.name = xstrdup(a->ln[u].name);
	end.where = xstrdup(expr_where(&r->nm));
	PUSH(a->ln, a->nln, a->capln, end);
	for (size_t i = 0; i < b.nmats; i++) {
		free(b.mats[i].name);
	}
	free(b.mats);
	free_names(&b.nm);
	free(dir);
}

// A new line of the kind for r, named name.
static struct asm_line new_line(struct reading *r, int kind, const char *name)
{
	struct asm_line l;
	const struct assembly *a = r->a;

	memset(&l, 0, sizeof(l));
	l.kind = kind;
	l.depth = r->depth;
	l.unit = r->unit;
	l.name = xstrdup(name);
	l.full = r->unit == NONE ? xstrdup(name) :
				   join_names(a->ln[r->unit].full, name);
	l.where = xstrdup(expr_where(&r->nm));
	l.m = rig_id();
	l.world = rig_id();
	return l;
}

// Reads a statement part NAME FILE ...
static void part_statement(struct reading *r, const struct words *ws)
{
	struct assembly *a = r->a;
	struct sibling sib = { r->path, ws->n > 2 ? ws->w[2] : "" };
	struct asm_line l;
	size_t k = 0;

	if (ws->n < 3) {
		expr_fail(&r->nm, "part NAME FILE ...", NULL);
	}
	if (line_named(r, ws->w[1]) != NONE) {
		expr_fail(&r->nm, "a part of that name exists", ws->w[1]);
	}
	l = new_line(r, ASM_PART, ws->w[1]);
	read_tail(r, ws, &l);
	l.m = place_rig(&l.place);
	l.world = r->unit == NONE ? l.m : rig_mul(a->ln[r->unit].world, l.m);
	l.file = sibling_path(sib);
	if (r->meshdir) {
		static const char obj[] = ".obj";
		char *dir = join_names(r->meshdir, ws->w[1]);
		size_t n = strlen(dir);

		l.mesh = xmalloc(n + sizeof(obj));
		memcpy(l.mesh, dir, n);
		memcpy(l.mesh + n, obj, sizeof(obj));
		free(dir);
	}
	if (!stream_file(l.file)) {
		l.kind = ASM_UNIT;
	}
	k = a->nln;
	PUSH(a->ln, a->nln, a->capln, l);
	if (a->ln[k].kind == ASM_UNIT) {
		sub_lines(r, ws, k);
	} else {
		stream_parts(r, ws, &a->ln[k]);
	}
}

// Reads a statement image NAME FILE on PART X0 Y0 Z0 X1 Y1 Z1 X2 Y2 Z2,
// its file relative to the assembly file.
static void image_line(struct reading *r, const struct words *ws)
{
	struct asm_line l;
	struct sibling sib = { r->path, ws->n > 2 ? ws->w[2] : "" };

	if (ws->n != IMAGE_WORDS || !is(ws, 3, "on")) {
		expr_fail(&r->nm,
			  "image NAME FILE on PART X0 Y0 Z0 X1 Y1 Z1 X2 Y2 "
			  "Z2",
			  NULL);
	}
	if (line_named(r, ws->w[4]) == NONE) {
		expr_fail(&r->nm, "on PART, a part above", ws->w[4]);
	}
	l = new_line(r, ASM_IMAGE, ws->w[1]);
	l.file = sibling_path(sib);
	l.on = xstrdup(ws->w[4]);
	for (size_t i = 0; i < ASM_CORNERS; i++) {
		l.corner[i] = num(r, ws, IMAGE_WORDS - ASM_CORNERS + i);
	}
	PUSH(r->a->ln, r->a->nln, r->a->capln, l);
}

// Reads a statement include FILE or materials FILE.
static void file_line(struct reading *r, const struct words *ws)
{
	struct sibling sib = { r->path, ws->n == 2 ? ws->w[1] : "" };
	int inc = ws->w[0][0] == 'i';
	char *file = NULL;

	if (ws->n != 2) {
		expr_fail(&r->nm, "include FILE or materials FILE", NULL);
	}
	file = inc ? sibling_path(sib) : lib_path(sib, "");
	if (inc) {
		read_params(&r->nm, file);
	} else {
		each_line(&r->nm, file, materials_line, r);
	}
	free(file);
}

static void statement(char *l, void *ctx)
{
	struct reading *r = ctx;
	char *w[MAXWORDS + 1];
	struct words ws = { w, split_words(l, w, MAXWORDS) };

	if (!ws.n) {
		return;
	}
	if (ws.n > MAXWORDS) {
		expr_fail(&r->nm, "too many words", NULL);
	}
	if (!strcmp(w[0], "include") || !strcmp(w[0], "materials")) {
		file_line(r, &ws);
	} else if (!strcmp(w[0], "material")) {
		material_line(r, &ws);
	} else if (!strcmp(w[0], "part")) {
		part_statement(r, &ws);
	} else if (!strcmp(w[0], "image")) {
		image_line(r, &ws);
	} else if (!strcmp(w[0], "step") && ws.n == 3 && !r->has_step) {
		read_step_size(r, &ws, 1, r->step);
		r->has_step = 1;
	} else if (!strcmp(w[0], "clearance") && ws.n == 2 &&
		   !(r->clearance > 0)) {
		r->clearance = num(r, &ws, 1);
		if (!(r->clearance > 0)) {
			expr_fail(&r->nm, "clearance MM, above 0", NULL);
		}
	} else {
		expr_fail(&r->nm,
			  "expected part, image, material, materials, include, "
			  "step DIST DEG or clearance MM, once each",
			  w[0]);
	}
}

// Defines the name of a line NAME = EXPR, or reads the statement.
static void define_line(char *l, void *ctx)
{
	struct reading *r = ctx;
	const char *eq = strchr(l, '=');
	size_t s = strspn(l, " \t");
	size_t e = s;

	while (l[e] == '_' || (l[e] >= 'a' && l[e] <= 'z') ||
	       (l[e] >= 'A' && l[e] <= 'Z') || (l[e] >= '0' && l[e] <= '9')) {
		e++;
	}
	if (!eq || e == s || eq != l + e + strspn(l + e, " \t")) {
		statement(l, ctx);
		return;
	}
	l[e] = 0;
	define(&r->nm, l + s, eq + 1, 0);
}

// Reads the lines of the file of r, and gives those of its paths that
// name no step its step, and each its clearance.
static void read_file_lines(struct reading *r)
{
	struct assembly *a = r->a;

	each_line(&r->nm, r->path, define_line, r);
	for (size_t i = r->first; i < a->nln; i++) {
		struct asm_line *l = &a->ln[i];

		if (l->unit != r->unit) {
			continue;
		}
		if (!l->has_step && r->has_step) {
			memcpy(l->step, r->step, sizeof(l->step));
			l->has_step = 1;
		}
		l->clearance = r->clearance;
	}
}

void asm_read(struct assembly *a, struct asm_file f)
{
	struct reading r;

	memset(a, 0, sizeof(*a));
	memset(&r, 0, sizeof(r));
	r.path = f.path;
	r.unit = NONE;
	r.meshdir = f.meshdir;
	r.a = a;
	read_file_lines(&r);
	for (size_t i = 0; i < r.nmats; i++) {
		free(r.mats[i].name);
	}
	free(r.mats);
	free_names(&r.nm);
}

void asm_free(struct assembly *a)
{
	for (size_t i = 0; i < a->nln; i++) {
		struct asm_line *l = &a->ln[i];

		free(l->name);
		free(l->full);
		free(l->file);
		free(l->mesh);
		free(l->place.s);
		free(l->path.s);
		free(l->with);
		free(l->where);
		free(l->on);
	}
	free(a->ln);
	free_parts(a->out, a->nout);
	memset(a, 0, sizeof(*a));
}
