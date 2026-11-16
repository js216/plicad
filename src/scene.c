// SPDX-License-Identifier: GPL-3.0-only
// scene.c -- The parts, the camera and the commands of view(1)
// Copyright (c) 2026 Jakob Kastelic

#include "scene.h"
#include "bvh.h"
#include "expr.h"
#include "geom.h"
#include "obj.h"
#include "util.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIT_MARGIN 1.1 // The room round the parts in a fitted view.
#define ISO_UP 2.0 // The z of the up (-1, 1, ISO_UP) of the iso view.
#define HALF_MIN 1e-6 // The least half height of a view, in mm.
#define HALF_MAX 1e7 // The most half height of a view, in mm.
#define BACK_GRAY \
	(81.0 / 255) // The gray of a background by default, as FreeCAD's.
#define PART_GRAY 0.6F // The gray of a part of no color.
#define PICK_EPS 1e-12 // The least determinant of a ray and a triangle.

enum {
	WORDS_MAX = 128, // The most words on a line of a scene file.
	PPM_MAX = 255, // The largest value of a PPM picture.
	DECIMAL = 10,
	PIC_MAX = 16384, // The widest and tallest picture, in pixels.
	BYTE = 255,
	CORNER_WORDS = 9, // The numbers of the corners of a picture.
	IMAGE_WORDS = 5 + CORNER_WORDS, // The words of an image line.
	MOVE_WORDS = 4, // The words of a color or a move.
	TURN_WORDS = 3, // turn AXIS DEG, and about X Y Z after it.
	MSG_LEN = 4096, // Room for the message of an error.
	PICK_DEPTH = 128, // Room for the nodes of a tree a pick has yet to see.
	WIN_W = 640, // The size of a new window, in pixels.
	WIN_H = 480
};

#define FRAMES_MAX 100000.0 // The most frames of a spin.
#define LEVEL_MAX 64.0 // The most units up for H, O, S and a drag.
#define CENTER_ZOOM 2.0 // The zoom of a center again.

static void free_part(struct part *p)
{
	free(p->name);
	free(p->pos);
	free(p->nrm);
	free(p->uv);
	free(p->col);
	free(p->grade);
	free(p->pix);
	bvh_free(&p->bvh);
	memset(p, 0, sizeof(*p));
}

static void free_item(struct item *it)
{
	free(it->name);
	free(it->path);
	free(it->steps);
	free(it->with);
}

void scene_free(struct scene *s)
{
	for (size_t i = 0; i < s->np; i++) {
		free_part(&s->p[i]);
	}
	for (size_t i = 0; i < s->nit; i++) {
		free_item(&s->it[i]);
	}
	for (size_t i = 0; i < s->nhist; i++) {
		free(s->hist[i].shown);
		free(s->hist[i].drag);
	}
	free(s->hist);
	free(s->drag);
	free(s->seq);
	free(s->p);
	free(s->it);
	free(s->view);
	s->hist = NULL;
	s->drag = NULL;
	s->seq = NULL;
	s->nseq = s->capseq = 0;
	s->nhist = s->caphist = s->at = 0;
	s->p = NULL;
	s->it = NULL;
	s->view = NULL;
	s->np = s->capp = s->nit = s->capit = 0;
}

const char *const view_help[] = {
	"left drag       turn",
	"middle, right   move",
	"wheel           zoom",
	"click           pick a part, print its name",
	"shift drag      move the unit of the part in the view plane",
	"space           fit all",
	"c               center on the picked part; again, zoom in",
	"i f b l r t u   iso front back left right top under",
	"h               hide the part under the cursor",
	"o               show only the part under the cursor, or all again",
	"s               see through the part under the cursor, or not",
	"H O S           the same for its unit, the level up",
	"[ ]             the level of the unit down, up",
	"a               show all, solid, all in place, no moves",
	"+ -             the next part in along its path, the last out",
	"A               all out, at the starts of their paths",
	"ctrl-z ctrl-y   undo, redo",
	"?               this help, on and off",
	"q               quit",
	NULL,
};

// Reads n numbers of a command from words w into x.
static int command_numbers(char **w, double *x, size_t n)
{
	for (size_t k = 0; k < n; k++) {
		char *end = NULL;

		x[k] = strtod(w[k], &end);
		if (end == w[k] || *end || !num_ok(x[k])) {
			return -1;
		}
	}
	return 0;
}

// The reading of a scene file: its names, the trap that an error of an
// expression jumps to, with its message, the item being read, and the
// unit open, or NONE.
struct reading {
	struct names nm;
	jmp_buf trap;
	char msg[MSG_LEN];
	struct item *it;
	size_t unit;
};

// Reads n numbers, expressions, from words w into x.
static void numbers(struct reading *rd, char **w, double *x, size_t n)
{
	for (size_t k = 0; k < n; k++) {
		x[k] = eval(&rd->nm, w[k]);
	}
}

// Reads a turn x|y|z DEG [about X Y Z] or a move DX DY DZ from words w,
// n of them, into st; returns the words it takes, or 0 for neither.
static size_t read_step(struct reading *rd, char **w, size_t n,
			struct pstep *st)
{
	double x[3];

	memset(st, 0, sizeof(*st));
	if (!strcmp(w[0], "move") && n >= MOVE_WORDS) {
		numbers(rd, w + 1, x, 3);
		st->d = v3(x[0], x[1], x[2]);
		return MOVE_WORDS;
	}
	if (strcmp(w[0], "turn") != 0 || n < TURN_WORDS || strlen(w[1]) != 1 ||
	    !strchr("xyz", w[1][0])) {
		return 0;
	}
	st->turn = 1;
	st->ax = axis_vec(w[1][0] - 'x', 1);
	numbers(rd, w + 2, &st->deg, 1);
	if (n >= TURN_WORDS + MOVE_WORDS && !strcmp(w[TURN_WORDS], "about")) {
		numbers(rd, w + TURN_WORDS + 1, x, 3);
		st->about = 1;
		st->at = v3(x[0], x[1], x[2]);
		return TURN_WORDS + MOVE_WORDS;
	}
	return TURN_WORDS;
}

// The motion of step st, a turn about at if it names no point.
static struct rigid pstep_rig(const struct pstep *st, struct vec at)
{
	struct rigid m = rig_id();
	struct line3 ax = { st->about ? st->at : at, st->ax };

	if (!st->turn) {
		m.t = st->d;
		return m;
	}
	return rig_rot(ax, st->deg);
}

// Reads the steps of an insertion path from words w[0 .. n - 1] into it;
// a turn that names no point turns about the origin of the part at its
// place.
static int path_steps(struct reading *rd, struct item *it, char **w, size_t n)
{
	for (size_t i = 0; i < n;) {
		struct pstep st;
		size_t k = read_step(rd, w + i, n - i, &st);

		if (!k) {
			return -1;
		}
		if (st.turn && !st.about) {
			st.about = 1;
			st.at = it->m.t;
		}
		PUSH(it->steps, it->nsteps, it->capsteps, st);
		i += k;
	}
	return 0;
}

// Reads the color, moves and turns of a mesh, from words w[0 .. n - 1].
static int mesh_options(struct reading *rd, struct item *it, char **w, size_t n)
{
	for (size_t i = 0; i < n;) {
		struct pstep st;
		size_t k = 0;

		if (!strcmp(w[i], "color") && i + 3 < n) {
			double x[3];

			numbers(rd, w + i + 1, x, 3);
			it->has_rgb = 1;
			for (int c = 0; c < 3; c++) {
				it->rgb[c] = (float)fmin(fmax(x[c], 0), 1);
			}
			i += MOVE_WORDS;
			continue;
		}
		k = read_step(rd, w + i, n - i, &st);
		if (k) {
			it->m = rig_mul(pstep_rig(&st, v3(0, 0, 0)), it->m);
			it->moved = 1;
			i += k;
		} else if (!strcmp(w[i], "path") && !it->with) {
			return path_steps(rd, it, w + i + 1, n - i - 1);
		} else if (!strcmp(w[i], "with") && i + 2 == n && !it->nsteps) {
			it->with = xstrdup(w[i + 1]);
			return 0;
		} else {
			return -1;
		}
	}
	return 0;
}

// The name of the name n of a line in the unit open, a new string.
static char *full_name(const struct reading *rd, const struct scene *s,
		       const char *n)
{
	return rd->unit == NONE ? xstrdup(n) :
				  join_names(s->it[rd->unit].name, n);
}

// The mesh or unit of the name n of a line, among the items of s so
// far, or NONE.
static size_t find_item(const struct reading *rd, const struct scene *s,
			const char *n)
{
	char *name = full_name(rd, s, n);
	size_t k = NONE;

	for (size_t i = 0; i < s->nit && k == NONE; i++) {
		if (!s->it[i].image && !strcmp(s->it[i].name, name)) {
			k = i;
		}
	}
	free(name);
	return k;
}

// Puts the mesh or unit it, read in the frame of the unit open, into the
// world: its place, its path and the item it travels with.
static void into_unit(const struct reading *rd, const struct scene *s,
		      struct item *it)
{
	const struct item *u = NULL;

	it->up = rd->unit;
	if (rd->unit == NONE) {
		return;
	}
	u = &s->it[rd->unit];
	it->m = rig_mul(u->m, it->m);
	it->moved = it->moved || u->moved;
	for (size_t i = 0; i < it->nsteps; i++) {
		struct pstep *st = &it->steps[i];
		struct rigid r = u->m;

		r.t = v3(0, 0, 0);
		st->d = rig_pt(r, st->d);
		st->ax = rig_pt(r, st->ax);
		st->at = rig_pt(u->m, st->at);
	}
	if (it->with) {
		char *w = join_names(u->name, it->with);

		free(it->with);
		it->with = w;
	}
}

// Reads a line image NAME FILE on PART X0 Y0 Z0 X1 Y1 Z1 X2 Y2 Z2 into
// it: the corners in the frame of the mesh PART, put into the world.
static int image_line(struct reading *rd, const struct scene *s, char **w,
		      size_t n, struct item *it)
{
	double x[CORNER_WORDS];

	if (n != IMAGE_WORDS || strcmp(w[3], "on") != 0) {
		return -1;
	}
	it->on = find_item(rd, s, w[4]);
	if (it->on == NONE) {
		return -1;
	}
	numbers(rd, w + IMAGE_WORDS - CORNER_WORDS, x, CORNER_WORDS);
	for (size_t k = 0; k < 3; k++) {
		it->corner[k] =
			rig_pt(s->it[it->on].m,
			       v3(x[3 * k], x[(3 * k) + 1], x[(3 * k) + 2]));
	}
	return 0;
}

// Reads one line of words w[0 .. n - 1] into an item of s; returns 0 for
// an item, 2 for a unit, 1 for a line of neither, or -1.
static int scene_line(struct reading *rd, struct scene *s, char **w, size_t n,
		      struct item *it)
{
	memset(it, 0, sizeof(*it));
	it->m = rig_id();
	it->on = NONE;
	it->up = NONE;
	if (!strcmp(w[0], "end") && n == 1) {
		if (rd->unit == NONE) {
			return -1;
		}
		rd->unit = s->it[rd->unit].up;
		return 1;
	}
	if (!strcmp(w[0], "unit") && n >= 2) {
		it->unit = 1;
		if (mesh_options(rd, it, w + 2, n - 2) < 0 || it->has_rgb) {
			return -1;
		}
		it->name = full_name(rd, s, w[1]);
		into_unit(rd, s, it);
		return 2;
	}
	if (!strcmp(w[0], "view") && n == 2) {
		struct cam c;

		if (cam_view(&c, w[1]) < 0) {
			return -1;
		}
		free(s->view);
		s->view = xstrdup(w[1]);
		return 1;
	}
	if (!strcmp(w[0], "background") && n == 4) {
		double x[3];

		numbers(rd, w + 1, x, 3);
		for (int k = 0; k < 3; k++) {
			s->back[k] = (float)fmin(fmax(x[k], 0), 1);
		}
		return 1;
	}
	if (n < 3 ||
	    (strcmp(w[0], "mesh") != 0 && strcmp(w[0], "image") != 0)) {
		return -1;
	}
	it->image = !strcmp(w[0], "image");
	if (it->image ? image_line(rd, s, w, n, it) < 0 :
			mesh_options(rd, it, w + 3, n - 3) < 0) {
		return -1;
	}
	it->name = full_name(rd, s, w[1]);
	it->path = sibling_path((struct sibling){ s->path, w[2] });
	if (!it->image) {
		into_unit(rd, s, it);
	} else {
		it->up = rd->unit;
	}
	return 0;
}

// Reads the header number of a PPM file at *s.
static long ppm_number(char **s)
{
	char *end = NULL;
	long v = 0;

	for (;;) {
		while (**s == ' ' || **s == '\t' || **s == '\n' ||
		       **s == '\r') {
			(*s)++;
		}
		if (**s != '#') {
			break;
		}
		while (**s && **s != '\n') {
			(*s)++;
		}
	}
	if (**s < '0' || **s > '9') {
		return -1;
	}
	v = strtol(*s, &end, DECIMAL);
	*s = end;
	return v;
}

// The header of a picture: its size, its channels, its largest value
// and the place of its first pixel, or 0 for a bad header.
struct pic_head {
	long w, h, depth, max;
	size_t at;
};

// Reads the header of a PAM file (P7) in text, after its magic number,
// of the tuple type RGB or RGB_ALPHA.
static void pam_head(char *text, struct pic_head *hd)
{
	char type[WORDS_MAX] = "";
	char *s = text + 2;

	for (;;) {
		char *line = s;
		char *end = strchr(s, '\n');
		char *v = NULL;

		if (!end) {
			return;
		}
		*end = 0;
		s = end + 1;
		v = line + strcspn(line, " \t");
		v += strspn(v, " \t");
		if (!strcmp(line, "ENDHDR")) {
			break;
		}
		if (!strncmp(line, "TUPLTYPE ", strlen("TUPLTYPE ")) &&
		    strlen(type) + strlen(v) < sizeof(type)) {
			memcpy(type + strlen(type), v, strlen(v) + 1);
		} else if (!strncmp(line, "WIDTH ", strlen("WIDTH "))) {
			hd->w = strtol(v, NULL, DECIMAL);
		} else if (!strncmp(line, "HEIGHT ", strlen("HEIGHT "))) {
			hd->h = strtol(v, NULL, DECIMAL);
		} else if (!strncmp(line, "DEPTH ", strlen("DEPTH "))) {
			hd->depth = strtol(v, NULL, DECIMAL);
		} else if (!strncmp(line, "MAXVAL ", strlen("MAXVAL "))) {
			hd->max = strtol(v, NULL, DECIMAL);
		} else if (line[0] != '#' && line[0] != 0) {
			return;
		}
	}
	if ((hd->depth == 3 && !strcmp(type, "RGB")) ||
	    (hd->depth == 4 && !strcmp(type, "RGB_ALPHA"))) {
		hd->at = (size_t)(s - text);
	}
}

// Reads the header of a PPM file (P6) in text.
static void ppm_head(char *text, struct pic_head *hd)
{
	char *s = text + 2;

	hd->w = ppm_number(&s);
	hd->h = ppm_number(&s);
	hd->max = ppm_number(&s);
	hd->depth = 3;
	hd->at = (size_t)(s - text) + 1;
}

// Reads the picture of a PPM (P6) or PAM (P7) file into p, four bytes
// a pixel, red, green, blue and opacity, the top row first.
static int read_ppm(const char *path, struct part *p, struct fault *e)
{
	size_t n = 0;
	char *text = slurp(path, &n, e);
	struct pic_head hd = { 0, 0, 0, 0, 0 };
	size_t np = 0;

	if (!text) {
		return -1;
	}
	if (n > 2 && text[0] == 'P' && text[1] == '6') {
		ppm_head(text, &hd);
	} else if (n > 2 && text[0] == 'P' && text[1] == '7') {
		pam_head(text, &hd);
	}
	if (hd.w < 1 || hd.h < 1 || hd.w > PIC_MAX || hd.h > PIC_MAX ||
	    hd.max < 1 || hd.max > PPM_MAX || !hd.at ||
	    hd.at + ((size_t)hd.w * (size_t)hd.h * (size_t)hd.depth) > n) {
		free(text);
		fault_at(e,
			 "%s: not a picture of bytes (PPM P6, or PAM P7 of "
			 "RGB or RGB_ALPHA)",
			 path);
		return -1;
	}
	p->pw = (int)hd.w;
	p->ph = (int)hd.h;
	np = (size_t)hd.w * (size_t)hd.h;
	p->pix = xmalloc(np * 4);
	for (size_t i = 0; i < np; i++) {
		const char *q = &text[hd.at + (i * (size_t)hd.depth)];

		for (size_t k = 0; k < 4; k++) {
			long v = k < (size_t)hd.depth ? (unsigned char)q[k] :
							hd.max;

			p->pix[(4 * i) + k] =
				(unsigned char)(v * BYTE / hd.max);
		}
		p->clear = p->clear || p->pix[(4 * i) + 3] < BYTE;
	}
	free(text);
	return 0;
}

static void put3(float *a, struct vec v)
{
	a[0] = (float)v.x;
	a[1] = (float)v.y;
	a[2] = (float)v.z;
}

static struct vec get3(const float *a)
{
	return v3((double)a[0], (double)a[1], (double)a[2]);
}

// Makes the two triangles of the picture of item it in p.
static void image_part(const struct item *it, struct part *p)
{
	const struct vec *c = it->corner;
	struct vec d = add(c[1], sub(c[2], c[0]));
	const struct vec q[PIC_CORNERS] = { c[0], c[1], d, c[0], d, c[2] };
	static const float uv[2 * PIC_CORNERS] = { 0, 1, 1, 1, 1, 0,
						   0, 1, 1, 0, 0, 0 };
	struct vec n = cross(sub(c[1], c[0]), sub(c[2], c[0]));

	n = len(n) > 0 ? unit(n) : v3(0, 0, 1);
	p->nt = 2;
	p->pos = xmalloc(sizeof(float) * TRI_FLOATS * 2);
	p->nrm = xmalloc(sizeof(float) * TRI_FLOATS * 2);
	p->uv = xmalloc(sizeof(uv));
	memcpy(p->uv, uv, sizeof(uv));
	for (size_t k = 0; k < PIC_CORNERS; k++) {
		put3(&p->pos[3 * k], q[k]);
		put3(&p->nrm[3 * k], n);
	}
}

static void part_box(struct part *p)
{
	p->box = box_empty();
	for (size_t i = 0; i < 3 * p->nt; i++) {
		box_add(&p->box, get3(&p->pos[3 * i]));
	}
	bvh_build(&p->bvh, p->pos, p->nt);
}

// Moves the obj part q by the place of it into p, named name.
static void mesh_part(const struct item *it, struct obj_part *q, struct part *p,
		      char *name)
{
	p->name = name;
	p->nt = q->nt;
	p->pos = q->pos;
	p->nrm = q->nrm;
	q->pos = q->nrm = NULL;
	for (size_t i = 0; i < 3 * p->nt && it->moved; i++) {
		put3(&p->pos[3 * i], rig_pt(it->m, get3(&p->pos[3 * i])));
		put3(&p->nrm[3 * i], rig_dir(it->m, get3(&p->nrm[3 * i])));
	}
	for (int k = 0; k < 3; k++) {
		p->rgb[k] = PART_GRAY;
		if (it->has_rgb) {
			p->rgb[k] = it->rgb[k];
		} else if (q->has_rgb) {
			p->rgb[k] = q->rgb[k];
		}
	}
	p->col = q->col;
	q->col = NULL;
	p->grade = q->grade;
	q->grade = NULL;
	for (size_t i = 0; p->col && i < 3 * p->nt; i++) {
		if (p->col[i] < 0) {
			p->col[i] = p->rgb[i % 3];
		}
	}
}

// The part of the picture of item i into out.
static int load_image(const struct scene *s, size_t i, struct part **out,
		      struct fault *e)
{
	const struct item *it = &s->it[i];
	struct part *p = xcalloc(1, sizeof(*p));

	*out = p;
	if (read_ppm(it->path, p, e) < 0) {
		free(p);
		*out = NULL;
		return -1;
	}
	image_part(it, p);
	p->name = xstrdup(it->name);
	p->item = i;
	p->shown = 1;
	p->rgb[0] = p->rgb[1] = p->rgb[2] = 1;
	part_box(p);
	return 0;
}

// The name of part k of an OBJ file of n parts for item it.
static char *part_name(const struct item *it, const struct obj_file *f,
		       size_t k)
{
	size_t a = strlen(it->name);
	size_t b = f->n == 1 ? 0 : strlen(f->p[k].name) + 1;
	char *name = xmalloc(a + b + 1);

	memcpy(name, it->name, a);
	if (b) {
		name[a] = '/';
		memcpy(name + a + 1, f->p[k].name, b - 1);
	}
	name[a + b] = 0;
	return name;
}

// The parts of item i into out.
static int load_item(const struct scene *s, size_t i, struct part **out,
		     size_t *n, struct fault *e)
{
	const struct item *it = &s->it[i];
	struct obj_file f;
	size_t cap = 0;

	*out = NULL;
	*n = 0;
	if (it->unit) {
		return 0;
	}
	if (it->image) {
		*n = 1;
		return load_image(s, i, out, e);
	}
	if (obj_read(it->path, &f, e) < 0) {
		return -1;
	}
	for (size_t k = 0; k < f.n; k++) {
		struct part p;

		memset(&p, 0, sizeof(p));
		mesh_part(it, &f.p[k], &p, part_name(it, &f, k));
		p.item = i;
		p.shown = 1;
		part_box(&p);
		PUSH(*out, *n, cap, p);
	}
	obj_free(&f);
	return 0;
}

static void add_parts(struct scene *s, struct part *p, size_t n)
{
	for (size_t k = 0; k < n; k++) {
		PUSH(s->p, s->np, s->capp, p[k]);
	}
	free(p);
}

// Defines the name of a line NAME = EXPR of words w; returns 0 if the
// line is none.
static int define_words(struct reading *rd, char **w, size_t n)
{
	size_t k = 0;

	if (n < 3 || strcmp(w[1], "=") != 0) {
		return 0;
	}
	for (const char *c = w[0]; *c; c++) {
		k += !(*c == '_' || (*c >= 'a' && *c <= 'z') ||
		       (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9'));
	}
	if (k || (w[0][0] >= '0' && w[0][0] <= '9')) {
		return 0;
	}
	{
		double v = 0;

		numbers(rd, w + 2, &v, 1);
		if (n != 3) {
			expr_fail(&rd->nm,
				  "NAME = EXPR, the expression without "
				  "blanks",
				  NULL);
		}
		define(&rd->nm, w[0], v, 0);
	}
	return 1;
}

// Reads the line l, numbered no, of the scene file into s.
static int read_line(struct reading *rd, struct scene *s, char *l, unsigned no,
		     struct fault *e)
{
	char *w[WORDS_MAX + 1];
	size_t n = split_words(l, w, WORDS_MAX);
	struct item it;
	int r = n > WORDS_MAX ? -1 : 1;

	memset(&it, 0, sizeof(it));
	if (n && n <= WORDS_MAX && !define_words(rd, w, n)) {
		rd->it = &it;
		r = scene_line(rd, s, w, n, &it);
		rd->it = NULL;
	}
	if (r < 0) {
		free_item(&it);
		fault_at(e, "%s:%u: bad line", s->path, no);
		return -1;
	}
	if (r == 0 || r == 2) {
		it.line = no;
		PUSH(s->it, s->nit, s->capit, it);
	}
	if (r == 2) {
		rd->unit = s->nit - 1;
	}
	return 0;
}

// Reads the lines of the scene file text into s.
static int scene_lines(struct scene *s, char *text, struct fault *e)
{
	struct reading *rd = xcalloc(1, sizeof(*rd));
	struct src_line *ls = NULL;
	size_t n = source_lines(text, &ls);
	char where[WHERE_LEN];
	int r = 0;

	rd->nm.trap = &rd->trap;
	rd->nm.msg = rd->msg;
	rd->nm.nmsg = sizeof(rd->msg);
	rd->nm.where = where;
	rd->unit = NONE;
	for (size_t i = 0; i < n && r == 0; i++) {
		xsnprintf(where, "%s:%u", s->path, ls[i].no);
		if (setjmp(rd->trap)) {
			if (rd->it) {
				free_item(rd->it);
			}
			fault_at(e, "%s", rd->msg);
			r = -1;
			break;
		}
		r = read_line(rd, s, ls[i].text, ls[i].no, e);
	}
	if (r == 0 && rd->unit != NONE) {
		fault_at(e, "%s: unit %s has no end", s->path,
			 s->it[rd->unit].name);
		r = -1;
	}
	free_source_lines(ls, n);
	free_names(&rd->nm);
	free(rd);
	return r;
}

int scene_read(struct scene *s, const char *path, struct fault *e)
{
	size_t size = 0;
	char *text = NULL;

	memset(s, 0, sizeof(*s));
	s->path = path;
	s->level = 1;
	s->picked = NONE;
	s->cam.sz = (struct size){ WIN_W, WIN_H };
	s->back[0] = s->back[1] = s->back[2] = (float)BACK_GRAY;
	cam_view(&s->cam, "iso");
	text = slurp(path, &size, e);
	if (!text) {
		return -1;
	}
	if (scene_lines(s, text, e) < 0) {
		free(text);
		scene_free(s);
		return -1;
	}
	free(text);
	for (size_t i = 0; i < s->nit; i++) {
		struct part *p = NULL;
		size_t n = 0;

		if (load_item(s, i, &p, &n, e) < 0) {
			scene_free(s);
			return -1;
		}
		add_parts(s, p, n);
	}
	s->drag = xcalloc(s->nit + 1, sizeof(*s->drag));
	scene_sequence(s);
	scene_place(s);
	scene_history_reset(s);
	if (s->view) {
		cam_view(&s->cam, s->view);
	}
	cam_fit(&s->cam, s);
	return 0;
}

int scene_reload(struct scene *s, size_t i, struct fault *e)
{
	struct part *p = NULL;
	size_t n = 0;
	size_t keep = 0;

	if (load_item(s, i, &p, &n, e) < 0) {
		return -1;
	}
	for (size_t k = 0; k < n; k++) {
		for (size_t j = 0; j < s->np; j++) {
			if (s->p[j].item == i &&
			    !strcmp(s->p[j].name, p[k].name)) {
				p[k].shown = s->p[j].shown;
			}
		}
	}
	for (size_t j = 0; j < s->np; j++) {
		if (s->p[j].item == i) {
			free_part(&s->p[j]);
		} else {
			s->p[keep++] = s->p[j];
		}
	}
	s->np = keep;
	add_parts(s, p, n);
	scene_place(s);
	scene_history_reset(s);
	return 0;
}

// The box of the parts that show.
static struct box scene_box(const struct scene *s)
{
	struct box b = box_empty();

	for (size_t i = 0; i < s->np; i++) {
		if (s->p[i].shown && s->p[i].nt) {
			struct box q = part_box_at(&s->p[i]);

			box_add(&b, q.lo);
			box_add(&b, q.hi);
		}
	}
	return b;
}

// A standard view: its name, and its right and up.
struct std_view {
	const char *name;
	double right[3], up[3];
};

static const struct std_view views[] = {
	{ "front", { 1, 0, 0 }, { 0, 0, 1 } },
	{ "back", { -1, 0, 0 }, { 0, 0, 1 } },
	{ "right", { 0, 1, 0 }, { 0, 0, 1 } },
	{ "left", { 0, -1, 0 }, { 0, 0, 1 } },
	{ "top", { 1, 0, 0 }, { 0, 1, 0 } },
	{ "bottom", { 1, 0, 0 }, { 0, -1, 0 } },
	{ "iso", { 1, 1, 0 }, { -1, 1, ISO_UP } },
};

int cam_view(struct cam *c, const char *name)
{
	for (size_t i = 0; i < sizeof(views) / sizeof(views[0]); i++) {
		const struct std_view *v = &views[i];

		if (!strcmp(v->name, name)) {
			c->r[0] =
				unit(v3(v->right[0], v->right[1], v->right[2]));
			c->r[1] = unit(v3(v->up[0], v->up[1], v->up[2]));
			c->r[2] = cross(c->r[0], c->r[1]);
			return 0;
		}
	}
	return -1;
}

void cam_fit(struct cam *c, const struct scene *s)
{
	struct box b = scene_box(s);
	double aspect = (double)c->sz.w / (double)c->sz.h;
	double rx = 0;
	double ry = 0;

	if (b.lo.x > b.hi.x) {
		c->c = v3(0, 0, 0);
		c->half = 1;
		return;
	}
	c->c = mul(add(b.lo, b.hi), 1.0 / 2);
	for (unsigned k = 0; k < BOX_CORNERS; k++) {
		struct vec d = sub(box_corner(b, k), c->c);

		rx = fmax(rx, fabs(dot(d, c->r[0])));
		ry = fmax(ry, fabs(dot(d, c->r[1])));
	}
	c->half = fmax(fmax(ry, rx / aspect) * FIT_MARGIN, HALF_MIN);
}

// v turned about the unit axis k by a radians.
static struct vec turn(struct vec v, struct vec k, double a)
{
	return add(add(mul(v, cos(a)), mul(cross(k, v), sin(a))),
		   mul(k, dot(k, v) * (1 - cos(a))));
}

void cam_turn(struct cam *c, struct px d)
{
	struct vec z = v3(0, 0, 1);
	struct vec right = c->r[0];

	for (int k = 0; k < 3; k++) {
		c->r[k] = turn(turn(c->r[k], right, -d.y * DEG), z, -d.x * DEG);
	}
	c->r[0] = unit(c->r[0]);
	c->r[1] = unit(sub(c->r[1], mul(c->r[0], dot(c->r[0], c->r[1]))));
	c->r[2] = cross(c->r[0], c->r[1]);
}

// The size of a pixel, in mm.
static double pixel(const struct cam *c)
{
	return 2 * c->half / (double)c->sz.h;
}

struct vec cam_shift(const struct cam *c, struct px d)
{
	double k = pixel(c);

	return add(mul(c->r[0], d.x * k), mul(c->r[1], -d.y * k));
}

void cam_pan(struct cam *c, struct px d)
{
	double k = pixel(c);

	c->c = add(c->c, add(mul(c->r[0], -d.x * k), mul(c->r[1], d.y * k)));
}

// The point of the world at pixel at, on the plane through the middle.
static struct vec cam_point(const struct cam *c, struct px at)
{
	double k = pixel(c);
	double x = (at.x - ((double)c->sz.w / 2)) * k;
	double y = (((double)c->sz.h / 2) - at.y) * k;

	return add(c->c, add(mul(c->r[0], x), mul(c->r[1], y)));
}

void cam_zoom(struct cam *c, double f, struct px at)
{
	struct vec p = cam_point(c, at);
	double half = fmin(fmax(c->half * f, HALF_MIN), HALF_MAX);

	f = half / c->half;
	c->c = add(p, mul(sub(c->c, p), f));
	c->half = half;
}

double cam_depth(const struct cam *c, const struct scene *s)
{
	struct box b = scene_box(s);
	double d = 1;

	for (unsigned k = 0; k < BOX_CORNERS && b.lo.x <= b.hi.x; k++) {
		d = fmax(d, fabs(dot(sub(box_corner(b, k), c->c), c->r[2])));
	}
	return 2 * d;
}

// The whole number at or below v, within 0 to hi.
static int clamp(double v, int hi)
{
	if (!(v > 0)) {
		return 0;
	}
	if (v > hi) {
		return hi;
	}
	return (int)v;
}

// The pixel of picture p at the point of weights b of the corners of
// its triangle of texture coordinates uv.
static const unsigned char *pic_at(const struct part *p, const float *uv,
				   const double *b)
{
	double u = 0;
	double v = 0;
	int i = 0;
	int j = 0;

	for (size_t k = 0; k < 3; k++) {
		u += b[k] * (double)uv[2 * k];
		v += b[k] * (double)uv[(2 * k) + 1];
	}
	i = clamp(u * p->pw, p->pw - 1);
	j = clamp(v * p->ph, p->ph - 1);
	return &p->pix[4 * (((size_t)j * (size_t)p->pw) + (size_t)i)];
}

// The distance along the ray r (from r.o along r.d) to the triangle t,
// or HUGE_VAL if the ray misses it; the weights of its corners at the
// hit in b.
static double hit(struct line3 r, const float *t, double *b)
{
	struct vec o = r.o;
	struct vec dir = r.d;
	struct vec a = get3(t);
	struct vec e1 = sub(get3(t + 3), a);
	struct vec e2 = sub(get3(t + (size_t)(2 * 3)), a);
	struct vec p = cross(dir, e2);
	double det = dot(e1, p);
	struct vec s = sub(o, a);
	struct vec q = cross(s, e1);
	double u = 0;
	double v = 0;

	if (fabs(det) < PICK_EPS) {
		return HUGE_VAL;
	}
	u = dot(s, p) / det;
	v = dot(dir, q) / det;
	if (u < 0 || v < 0 || u + v > 1) {
		return HUGE_VAL;
	}
	b[0] = 1 - u - v;
	b[1] = u;
	b[2] = v;
	return dot(e2, q) / det;
}

// Whether the ray r meets triangle t of p nearer than *best, which it
// then sets; a clear pixel of a picture does not count.
static int pick_tri(const struct part *p, struct line3 r, size_t t,
		    double *best)
{
	double b[3] = { 0, 0, 0 };
	double d = hit(r, &p->pos[TRI_FLOATS * t], b);

	if (d < HUGE_VAL && p->uv &&
	    !pic_at(p, &p->uv[PIC_CORNERS * t], b)[3]) {
		return 0;
	}
	if (d >= 0 && d < *best) {
		*best = d;
		return 1;
	}
	return 0;
}

// Whether the ray r meets part p nearer than *best, which it then sets:
// the ray goes down the tree of the triangles of p, into the boxes it
// meets nearer than *best.
static int pick_part(const struct part *p, struct line3 r, double *best)
{
	size_t stack[PICK_DEPTH];
	size_t top = 0;
	int found = 0;

	stack[top++] = 0;
	while (top) {
		const struct bvh_node *n = &p->bvh.n[stack[--top]];

		if (!(ray_box(r, n->b, *best) < HUGE_VAL)) {
			continue;
		}
		if (!n->left || top + 2 > PICK_DEPTH) {
			for (size_t i = n->lo; i < n->hi; i++) {
				found = pick_tri(p, r, p->bvh.idx[i], best) ||
					found;
			}
			continue;
		}
		stack[top++] = n->left;
		stack[top++] = n->left + 1;
	}
	return found;
}

size_t scene_pick(const struct scene *s, struct px at)
{
	const struct cam *c = &s->cam;
	struct line3 ray = { add(cam_point(c, at),
				 mul(c->r[2], cam_depth(c, s))),
			     mul(c->r[2], -1) };
	double best = HUGE_VAL;
	size_t pick = NONE;

	for (size_t i = 0; i < s->np; i++) {
		const struct part *p = &s->p[i];
		struct rigid back = rig_inv(p->mot);
		struct line3 home;

		home.o = rig_pt(back, ray.o);
		home.d = rig_dir(back, ray.d);

		if (p->shown && p->nt && pick_part(p, home, &best)) {
			pick = i;
		}
	}
	return pick;
}

// Shows the parts that name pat, or all if pat is NULL, as how says:
// 1 show, 0 hide, 2 show only them, 3 solid, 4 see-through.
static size_t show(struct scene *s, const char *pat, int how)
{
	size_t n = 0;

	for (size_t i = 0; i < s->np; i++) {
		int match = !pat || scene_in_unit(pat, s->p[i].name);

		n += (size_t)match;
		if (how == 2) {
			s->p[i].shown = match;
		} else if (match && how > 2) {
			s->p[i].see = how == 4;
		} else if (match) {
			s->p[i].shown = how;
		}
	}
	return n;
}

// Reads the numbers of words w[1 .. n - 1] into x and checks them.
static int args(char **w, size_t n, double *x, size_t want)
{
	return n == want + 1 && command_numbers(w + 1, x, want) == 0 ? 0 : -1;
}

// Runs a command of the camera into r; returns 1 if it is none.
static int cam_command(struct scene *s, char **w, size_t n, struct cmd *r)
{
	double x[2];
	struct cam *c = &s->cam;

	r->ask = CMD_DRAW;
	if (!strcmp(w[0], "center") && n == 1) {
		scene_center(s);
		return 0;
	}
	if ((!strcmp(w[0], "view") && n == 2 && cam_view(c, w[1]) == 0) ||
	    (!strcmp(w[0], "fit") && n == 1)) {
		cam_fit(c, s);
	} else if (!strcmp(w[0], "turn") && !args(w, n, x, 2)) {
		cam_turn(c, (struct px){ x[0], x[1] });
	} else if (!strcmp(w[0], "zoom") && !args(w, n, x, 1) && x[0] > 0) {
		cam_zoom(c, x[0],
			 (struct px){ (double)c->sz.w / 2,
				      (double)c->sz.h / 2 });
	} else if (!strcmp(w[0], "select") && n <= 2) {
		scene_select_name(s, n == 2 ? w[1] : NULL);
		return 0;
	} else {
		r->ask = 0;
		return 1;
	}
	scene_cam_moved(s);
	return 0;
}

void scene_select(struct scene *s, size_t i)
{
	s->picked = i < s->np ? i : NONE;
	s->centered = 0;
}

void scene_select_name(struct scene *s, const char *name)
{
	size_t k = NONE;

	for (size_t i = 0; name && i < s->np && k == NONE; i++) {
		if (!strcmp(s->p[i].name, name)) {
			k = i;
		}
	}
	for (size_t i = 0; name && i < s->np && k == NONE; i++) {
		if (scene_in_unit(name, s->p[i].name)) {
			k = i;
		}
	}
	scene_select(s, k);
}

void scene_center(struct scene *s)
{
	if (s->picked >= s->np) {
		return;
	}
	if (s->centered) {
		cam_zoom(&s->cam, 1 / CENTER_ZOOM,
			 (struct px){ (double)s->cam.sz.w / 2,
				      (double)s->cam.sz.h / 2 });
	} else {
		struct box b = part_box_at(&s->p[s->picked]);

		s->cam.c = mul(add(b.lo, b.hi), 1.0 / 2);
		s->centered = 1;
	}
	s->touched = 1;
}

void scene_cam_moved(struct scene *s)
{
	s->centered = 0;
	s->touched = 1;
}

void scene_resize(struct scene *s, struct size sz)
{
	s->cam.sz = sz;
	if (!s->touched) {
		cam_fit(&s->cam, s);
	}
}

// Prints each part, its triangles, whether it shows, whether
// see-through, and the middle of its box where it stands.
static void print_list(const struct scene *s)
{
	for (size_t i = 0; i < s->np; i++) {
		struct box b = part_box_at(&s->p[i]);
		struct vec m = mul(add(b.lo, b.hi), 1.0 / 2);

		printf("%s %zu %s %s at %.3f %.3f %.3f\n", s->p[i].name,
		       s->p[i].nt, s->p[i].shown ? "shown" : "hidden",
		       s->p[i].see ? "see" : "solid", m.x + 0.0, m.y + 0.0,
		       m.z + 0.0);
	}
}

// Prints the name of the part at the pixel at, with its unit in
// brackets if label, or - for none.
static void print_pick(const struct scene *s, struct px at, int label)
{
	size_t i = scene_pick(s, at);
	char *l = NULL;

	if (i == NONE) {
		printf("-\n");
		return;
	}
	l = label ? scene_label(s, i) : xstrdup(s->p[i].name);
	printf("%s\n", l);
	free(l);
}

// Reads a command that asks the program that draws into r; returns 1 if
// it is none.
static int ask_command(char **w, size_t n, struct cmd *r)
{
	double x[1];

	if (!strcmp(w[0], "shot") && n == 2) {
		r->file = w[1];
		r->ask = CMD_SHOT;
	} else if (!strcmp(w[0], "spin") && !args(w, n, x, 1) && x[0] >= 1 &&
		   x[0] <= FRAMES_MAX) {
		r->frames = lround(x[0]);
		r->ask = CMD_SPIN;
	} else if (!strcmp(w[0], "quit") && n == 1) {
		r->ask = CMD_QUIT;
	} else {
		return 1;
	}
	return 0;
}

// Runs a command that prints or asks for frames into r; returns 1 if it
// is none.
static int out_command(const struct scene *s, char **w, size_t n, struct cmd *r)
{
	double x[2];

	if ((!strcmp(w[0], "pick") || !strcmp(w[0], "label")) &&
	    !args(w, n, x, 2)) {
		print_pick(s, (struct px){ x[0], x[1] }, w[0][0] == 'l');
	} else if (!strcmp(w[0], "help") && n == 1) {
		for (size_t i = 0; view_help[i]; i++) {
			printf("%s\n", view_help[i]);
		}
	} else if (!strcmp(w[0], "list") && n == 1) {
		print_list(s);
	} else if (ask_command(w, n, r)) {
		return 1;
	}
	(void)fflush(stdout);
	return 0;
}

// Runs a command that shows or hides parts into r; returns 1 if it is
// none, -1 if it matches no part.
static int show_command(struct scene *s, char **w, size_t n, struct cmd *r)
{
	static const char *const how[] = { "hide", "show", "only", "solid",
					   "see" };

	r->ask = CMD_DRAW;
	if (n == 1 && !strcmp(w[0], "all")) {
		(void)show(s, NULL, 1);
		scene_commit(s);
		return 0;
	}
	for (int k = 0; k < (int)(sizeof(how) / sizeof(how[0])) && n == 2;
	     k++) {
		if (!strcmp(w[0], how[k])) {
			size_t m = show(s, w[1], k);

			scene_commit(s);
			return m ? 0 : -1;
		}
	}
	r->ask = 0;
	return 1;
}

char *scene_unit(const struct scene *s, size_t i)
{
	const char *name = s->p[i].name;
	size_t n = strlen(name);
	const char *first = strchr(name, '/');

	for (int k = 0; k < s->level && first; k++) {
		const char *sl = name + n - 1;

		while (sl > first && *sl != '/') {
			sl--;
		}
		n = (size_t)(sl - name);
		if (sl == first) {
			break;
		}
	}
	{
		char *u = xmalloc(n + 1);

		memcpy(u, name, n);
		u[n] = 0;
		return u;
	}
}

char *scene_label(const struct scene *s, size_t i)
{
	const char *name = s->p[i].name;
	char *u = scene_unit(s, i);
	size_t n = strlen(u);
	size_t a = n;
	char *l = xmalloc(strlen(name) + 3);

	while (a > 0 && u[a - 1] != '/') {
		a--;
	}
	memcpy(l, name, a);
	l[a] = '[';
	memcpy(l + a + 1, name + a, n - a);
	l[n + 1] = ']';
	memcpy(l + n + 2, name + n, strlen(name + n) + 1);
	free(u);
	return l;
}

// Whether the parts that show are those of unit u, all of them.
static int alone(const struct scene *s, const char *u)
{
	for (size_t i = 0; i < s->np; i++) {
		if (!s->p[i].shown != !scene_in_unit(u, s->p[i].name)) {
			return 0;
		}
	}
	return 1;
}

// Shows every part again, as they are, if one part alone shows, or with
// up, one unit alone; returns whether it did.
static int all_again(struct scene *s, int up)
{
	size_t shown = 0;
	size_t first = NONE;
	char *v = NULL;
	int all = 0;

	for (size_t i = 0; i < s->np; i++) {
		shown += (size_t)(s->p[i].shown != 0);
		if (first == NONE && s->p[i].shown) {
			first = i;
		}
	}
	if (first == NONE) {
		return 0;
	}
	v = up ? scene_unit(s, first) : NULL;
	all = up ? alone(s, v) : shown == 1;
	free(v);
	for (size_t i = 0; all && i < s->np; i++) {
		s->p[i].shown = 1;
	}
	return all;
}

int scene_show_at(struct scene *s, int key, struct px at)
{
	size_t k = NONE;
	int up = key == 'H' || key == 'O' || key == 'S';
	int low = up ? key - 'A' + 'a' : key;
	char *u = NULL;
	int see = 0;

	// Out of the view of one part or unit, o shows all again.
	if (low == 'o' && all_again(s, up)) {
		scene_commit(s);
		return 0;
	}
	k = scene_pick(s, at);
	if (k == NONE) {
		return -1;
	}
	u = up ? scene_unit(s, k) : xstrdup(s->p[k].name);
	see = !s->p[k].see;
	for (size_t i = 0; i < s->np; i++) {
		int in = up ? scene_in_unit(u, s->p[i].name) : i == k;

		if (low == 's' && in) {
			s->p[i].see = see;
		} else if (low == 'o' || (low == 'h' && in)) {
			s->p[i].shown = low == 'o' && in;
		}
	}
	free(u);
	scene_commit(s);
	return 0;
}

// Runs a command of the sequence, the moves or the history into r;
// returns 1 if it is none.
static int arrange_command(struct scene *s, char **w, size_t n, struct cmd *r)
{
	double x[1];
	double y[2];
	double d[3];

	r->ask = CMD_DRAW;
	if (!strcmp(w[0], "play") && !args(w, n, x, 1) && x[0] >= 0 &&
	    x[0] <= (double)s->nseq) {
		scene_play(s, x[0]);
		scene_commit(s);
	} else if (!strcmp(w[0], "move") && n == MOVE_WORDS + 1 &&
		   !command_numbers(w + 2, d, 3) &&
		   scene_move(s, w[1], v3(d[0], d[1], d[2]))) {
		scene_commit(s);
	} else if (!strcmp(w[0], "level") && !args(w, n, x, 1) && x[0] >= 1 &&
		   x[0] <= LEVEL_MAX) {
		s->level = (int)lround(x[0]);
	} else if (!strcmp(w[0], "home") && n == 1) {
		scene_home(s);
		scene_commit(s);
	} else if (!strcmp(w[0], "key") && n == 4 && w[1][0] && !w[1][1] &&
		   strchr("hosHOS", w[1][0]) && !command_numbers(w + 2, y, 2)) {
		(void)scene_show_at(s, w[1][0], (struct px){ y[0], y[1] });
	} else if (!strcmp(w[0], "reset") && n == 1) {
		scene_reset(s);
		scene_commit(s);
	} else if (!strcmp(w[0], "undo") && n == 1) {
		(void)scene_undo(s);
	} else if (!strcmp(w[0], "redo") && n == 1) {
		(void)scene_redo(s);
	} else {
		r->ask = 0;
		return 1;
	}
	return 0;
}

struct cmd scene_command(struct scene *s, char *line)
{
	struct cmd r = { 0, NULL, 0 };
	char *w[WORDS_MAX + 1];
	size_t n = split_words(line, w, WORDS_MAX);
	int bad = 0;

	if (!n) {
		return r;
	}
	if (n > WORDS_MAX) {
		warn("view: bad command");
		return r;
	}
	bad = show_command(s, w, n, &r);
	if (bad > 0) {
		bad = cam_command(s, w, n, &r);
	}
	if (bad > 0) {
		bad = arrange_command(s, w, n, &r);
	}
	if (bad > 0) {
		bad = out_command(s, w, n, &r);
	}
	if (bad < 0) {
		warn("view: no part matches %s", w[1]);
	} else if (bad) {
		warn("view: bad command %s", w[0]);
	}
	return r;
}

// The shade of a face of normal n in the view, seen from either side.
static float shade(struct vec n)
{
	struct vec l = unit(v3(LIGHT_X, LIGHT_Y, LIGHT_Z));

	if (n.z < 0) {
		n = mul(n, -1);
	}
	return AMBIENT + ((1 - AMBIENT) * (float)fmax(dot(unit(n), l), 0));
}

// A triangle on the screen: its corners in pixels and depth, and its
// color, or its picture and the texture coordinates of its corners.
struct stri {
	double x[3], y[3], z[3];
	unsigned char rgb[3];
	unsigned alpha; // The opacity of a color, 0 to BYTE.
	const struct part *p;
	const float *uv;
};

// The frame of a render: the pixels and their depths.
struct frame {
	unsigned char *rgb;
	double *z;
	struct size sz;
};

// Lays the color rgb over out by its opacity a, 0 to BYTE.
static void mix(const unsigned char *rgb, unsigned a, unsigned char *out)
{
	for (size_t k = 0; k < 3; k++) {
		out[k] = (unsigned char)(((rgb[k] * a) + (out[k] * (BYTE - a)) +
					  (BYTE / 2)) /
					 BYTE);
	}
}

// Lays the pixel of the picture of t at weights b over out, by its
// opacity.
static void blend(const struct stri *t, const double *b, unsigned char *out)
{
	const unsigned char *px = pic_at(t->p, t->uv, b);

	mix(px, px[3], out);
}

// Fills the pixels of t nearer than those there.
static void fill(struct frame *f, const struct stri *t)
{
	double area = ((t->x[1] - t->x[0]) * (t->y[2] - t->y[0])) -
		      ((t->x[2] - t->x[0]) * (t->y[1] - t->y[0]));
	int x0 = clamp(fmin(fmin(t->x[0], t->x[1]), t->x[2]), f->sz.w - 1);
	int x1 = clamp(fmax(fmax(t->x[0], t->x[1]), t->x[2]), f->sz.w - 1);
	int y0 = clamp(fmin(fmin(t->y[0], t->y[1]), t->y[2]), f->sz.h - 1);
	int y1 = clamp(fmax(fmax(t->y[0], t->y[1]), t->y[2]), f->sz.h - 1);

	if (fabs(area) < PICK_EPS) {
		return;
	}
	for (int y = y0; y <= y1; y++) {
		for (int x = x0; x <= x1; x++) {
			double px = x + (1.0 / 2);
			double py = y + (1.0 / 2);
			double b[3];
			size_t at = ((size_t)y * (size_t)f->sz.w) + (size_t)x;
			double z = 0;

			for (int k = 0; k < 3; k++) {
				int i = (k + 1) % 3;
				int j = (k + 2) % 3;

				b[k] = (((t->x[j] - t->x[i]) * (py - t->y[i])) -
					((px - t->x[i]) *
					 (t->y[j] - t->y[i]))) /
				       area;
			}
			if (b[0] < 0 || b[1] < 0 || b[2] < 0) {
				continue;
			}
			z = (b[0] * t->z[0]) + (b[1] * t->z[1]) +
			    (b[2] * t->z[2]);
			if (z <= f->z[at]) {
				continue;
			}
			if (t->uv) {
				blend(t, b, &f->rgb[3 * at]);
			} else if (t->alpha < BYTE) {
				mix(t->rgb, t->alpha, &f->rgb[3 * at]);
			} else {
				f->z[at] = z;
				memcpy(&f->rgb[3 * at], t->rgb, 3);
			}
		}
	}
}

// The triangle k of part p on the screen of camera c.
static struct stri on_screen(const struct cam *c, const struct part *p,
			     size_t k)
{
	struct stri t;
	double px = 1 / pixel(c);
	struct vec n = v3(0, 0, 0);
	float light = 0;

	memset(&t, 0, sizeof(t));
	for (size_t i = 0; i < 3; i++) {
		struct vec d =
			sub(rig_pt(p->mot,
				   get3(&p->pos[(TRI_FLOATS * k) + (3 * i)])),
			    c->c);

		t.x[i] = ((double)c->sz.w / 2) + (dot(d, c->r[0]) * px);
		t.y[i] = ((double)c->sz.h / 2) - (dot(d, c->r[1]) * px);
		t.z[i] = dot(d, c->r[2]);
		n = add(n, get3(&p->nrm[(TRI_FLOATS * k) + (3 * i)]));
	}
	n = rig_dir(p->mot, n);
	light = len(n) > 0 ? shade(v3(dot(n, c->r[0]), dot(n, c->r[1]),
				      dot(n, c->r[2]))) :
			     AMBIENT;
	for (int i = 0; i < 3; i++) {
		float base = p->col ? p->col[(3 * k) + (size_t)i] : p->rgb[i];
		double v = (double)(base * (p->uv ? 1 : light));

		t.rgb[i] = (unsigned char)lround(fmin(fmax(v, 0), 1) *
						 (double)BYTE);
	}
	t.p = p;
	t.uv = p->uv ? &p->uv[PIC_CORNERS * k] : NULL;
	t.alpha = p->see ? (unsigned)lround((double)SEE_ALPHA * (double)BYTE) :
			   BYTE;
	return t;
}

void scene_render(const struct scene *s, unsigned char *rgb)
{
	struct frame f = { rgb, NULL, s->cam.sz };
	size_t n = (size_t)f.sz.w * (size_t)f.sz.h;
	size_t *order = NULL;

	f.z = xmalloc(n * sizeof(double));
	for (size_t i = 0; i < n; i++) {
		f.z[i] = -HUGE_VAL;
	}
	for (size_t i = 0; i < 3 * n; i++) {
		rgb[i] = (unsigned char)lround((double)s->back[i % 3] *
					       (double)BYTE);
	}
	for (size_t i = 0; i < s->np; i++) {
		const struct part *p = &s->p[i];

		for (size_t k = 0; p->shown && !p->uv && !p->see && k < p->nt;
		     k++) {
			struct stri t = on_screen(&s->cam, p, k);

			fill(&f, &t);
		}
	}
	n = scene_pictures(s, &order);
	for (size_t i = 0; i < n; i++) {
		const struct part *p = &s->p[order[i]];

		for (size_t k = 0; k < p->nt; k++) {
			struct stri t = on_screen(&s->cam, p, k);

			fill(&f, &t);
		}
	}
	free(order);
	free(f.z);
}

size_t scene_pictures(const struct scene *s, size_t **out)
{
	size_t *v = xmalloc((s->np + 1) * sizeof(size_t));
	double *d = xmalloc((s->np + 1) * sizeof(double));
	size_t n = 0;

	for (size_t i = 0; i < s->np; i++) {
		const struct part *p = &s->p[i];
		struct box q = part_box_at(p);
		struct vec c = mul(add(q.lo, q.hi), 1.0 / 2);
		double z = dot(sub(c, s->cam.c), s->cam.r[2]);
		size_t j = n;

		if (!p->shown || (!p->uv && !p->see)) {
			continue;
		}
		for (; j > 0 && d[j - 1] > z; j--) {
			v[j] = v[j - 1];
			d[j] = d[j - 1];
		}
		v[j] = i;
		d[j] = z;
		n++;
	}
	free(d);
	*out = v;
	return n;
}

int write_ppm(const char *path, const unsigned char *rgb, struct size sz)
{
	FILE *f = fopen(path, "wb");
	size_t n = (size_t)sz.w * (size_t)sz.h * 3;
	int bad = 0;

	if (!f) {
		return -1;
	}
	bad = fprintf(f, "P6\n%d %d\n%d\n", sz.w, sz.h, BYTE) < 0 ||
	      fwrite(rgb, 1, n, f) != n;
	if (fclose(f) != 0) {
		bad = 1;
	}
	return bad ? -1 : 0;
}
