// SPDX-License-Identifier: GPL-3.0-only
// scene.c -- The parts, the camera and the commands of view(1)
// Copyright (c) 2026 Jakob Kastelic

#include "scene.h"
#include "geom.h"
#include "obj.h"
#include "util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIT_MARGIN 1.1 // The room round the parts in a fitted view.
#define ISO_UP 2.0 // The z of the up (-1, 1, ISO_UP) of the iso view.
#define HALF_MIN 1e-6 // The least half height of a view, in mm.
#define HALF_MAX 1e7 // The most half height of a view, in mm.
#define BACK_GREY \
	(81.0 / 255) // The grey of a background by default, as FreeCAD's.
#define PART_GREY 0.6F // The grey of a part of no colour.
#define PICK_EPS 1e-12 // The least determinant of a ray and a triangle.

enum {
	WORDS_MAX = 128, // The most words on a line of a scene file.
	PPM_MAX = 255, // The largest value of a PPM picture.
	DECIMAL = 10,
	PIC_MAX = 16384, // The widest and tallest picture, in pixels.
	BYTE = 255,
	IMAGE_WORDS = 12, // The words of an image line.
	MOVE_WORDS = 4, // The words of a colour or a move.
	TURN_WORDS = 5,
	WIN_W = 640, // The size of a new window, in pixels.
	WIN_H = 480
};

#define EXPLODE_MAX 100.0 // The largest factor of an explode.
#define WIN_MAX 8192.0 // The widest and tallest window, in pixels.
#define FRAMES_MAX 100000.0 // The most frames of a spin.

static void free_part(struct part *p)
{
	free(p->name);
	free(p->pos);
	free(p->nrm);
	free(p->uv);
	free(p->pix);
	memset(p, 0, sizeof(*p));
}

static void free_item(struct item *it)
{
	free(it->name);
	free(it->path);
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
	free(s->p);
	free(s->it);
	free(s->view);
	s->hist = NULL;
	s->drag = NULL;
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
	"shift drag      drag the part out",
	"space           fit all",
	"i f b l r t u   iso front back left right top under",
	"h               hide the part under the cursor",
	"o               show only the part under the cursor",
	"s               see through the part under the cursor, or not",
	"a               show all, solid, all home",
	"+ -             explode out, back in",
	"ctrl-z ctrl-y   undo, redo",
	"?               this help, on and off",
	"q               quit",
	NULL,
};

// Reads n numbers from words w into x.
static int numbers(char **w, double *x, size_t n)
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

// Reads the colour, moves and turns of a mesh, from words w[0 .. n - 1].
static int mesh_options(struct item *it, char **w, size_t n)
{
	double x[4];

	for (size_t i = 0; i < n;) {
		if (!strcmp(w[i], "colour") && i + 3 < n &&
		    !numbers(w + i + 1, x, 3)) {
			it->has_rgb = 1;
			for (int k = 0; k < 3; k++) {
				it->rgb[k] = (float)fmin(fmax(x[k], 0), 1);
			}
			i += MOVE_WORDS;
		} else if (!strcmp(w[i], "move") && i + 3 < n &&
			   !numbers(w + i + 1, x, 3)) {
			it->m.t = add(it->m.t, v3(x[0], x[1], x[2]));
			it->moved = 1;
			i += MOVE_WORDS;
		} else if (!strcmp(w[i], "turn") && i + 4 < n &&
			   !numbers(w + i + 1, x, 4) &&
			   len(v3(x[0], x[1], x[2])) > 0) {
			struct line3 ax = { v3(0, 0, 0),
					    unit(v3(x[0], x[1], x[2])) };

			it->m = rig_mul(rig_rot(ax, x[3]), it->m);
			it->moved = 1;
			i += TURN_WORDS;
		} else {
			return -1;
		}
	}
	return 0;
}

// Reads one line of words w[0 .. n - 1] into an item of s.
static int scene_line(struct scene *s, char **w, size_t n, struct item *it)
{
	memset(it, 0, sizeof(*it));
	it->m = rig_id();
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

		if (numbers(w + 1, x, 3) < 0) {
			return -1;
		}
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
	if (it->image) {
		double x[IMAGE_WORDS - 3];

		if (n != IMAGE_WORDS || numbers(w + 3, x, n - 3) < 0) {
			return -1;
		}
		for (size_t k = 0; k < 3; k++) {
			it->corner[k] =
				v3(x[3 * k], x[(3 * k) + 1], x[(3 * k) + 2]);
		}
	} else if (mesh_options(it, w + 3, n - 3) < 0) {
		return -1;
	}
	it->name = xstrdup(w[1]);
	it->path = sibling_path((struct sibling){ s->path, w[2] });
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
		p->rgb[k] = PART_GREY;
		if (it->has_rgb) {
			p->rgb[k] = it->rgb[k];
		} else if (q->has_rgb) {
			p->rgb[k] = q->rgb[k];
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

// Reads the lines of the scene file text into s.
static int scene_lines(struct scene *s, char *text, struct fault *e)
{
	char *at = text;
	char *l = NULL;
	size_t line = 0;

	while ((l = next_line(&at)) != NULL) {
		char *w[WORDS_MAX + 1];
		char *hash = strchr(l, '#');
		size_t n = 0;
		struct item it;
		int r = 0;

		line++;
		if (hash) {
			*hash = 0;
		}
		n = split_words(l, w, WORDS_MAX);
		r = n > WORDS_MAX ? -1 : 1;
		if (n && n <= WORDS_MAX) {
			r = scene_line(s, w, n, &it);
		}
		if (r < 0) {
			fault_at(e, "%s:%zu: bad line", s->path, line);
			return -1;
		}
		if (r == 0) {
			it.line = line;
			PUSH(s->it, s->nit, s->capit, it);
		}
	}
	return 0;
}

int scene_read(struct scene *s, const char *path, struct fault *e)
{
	size_t size = 0;
	char *text = NULL;

	memset(s, 0, sizeof(*s));
	s->path = path;
	s->cam.sz = (struct size){ WIN_W, WIN_H };
	s->back[0] = s->back[1] = s->back[2] = (float)BACK_GREY;
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
	s->drag = xcalloc(s->nit + 1, sizeof(double));
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
		struct line3 home = { sub(ray.o, p->off), ray.d };

		for (size_t t = 0; p->shown && t < p->nt; t++) {
			double b[3] = { 0, 0, 0 };
			double d = hit(home, &p->pos[TRI_FLOATS * t], b);

			if (d < HUGE_VAL && p->uv &&
			    !pic_at(p, &p->uv[PIC_CORNERS * t], b)[3]) {
				continue;
			}
			if (d >= 0 && d < best) {
				best = d;
				pick = i;
			}
		}
	}
	return pick;
}

// Shows the parts that match pat, or all if pat is NULL, as how says:
// 1 show, 0 hide, 2 show only them, 3 solid, 4 see-through.
static size_t show(struct scene *s, const char *pat, int how)
{
	size_t n = 0;

	for (size_t i = 0; i < s->np; i++) {
		int match = !pat || glob(pat, s->p[i].name);

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
	return n == want + 1 && numbers(w + 1, x, want) == 0 ? 0 : -1;
}

// Runs a command of the camera into r; returns 1 if it is none.
static int cam_command(struct scene *s, char **w, size_t n, struct cmd *r)
{
	double x[2];
	struct cam *c = &s->cam;

	r->ask = CMD_DRAW;
	if ((!strcmp(w[0], "view") && n == 2 && cam_view(c, w[1]) == 0) ||
	    (!strcmp(w[0], "fit") && n == 1)) {
		cam_fit(c, s);
	} else if (!strcmp(w[0], "turn") && !args(w, n, x, 2)) {
		cam_turn(c, (struct px){ x[0], x[1] });
	} else if (!strcmp(w[0], "zoom") && !args(w, n, x, 1) && x[0] > 0) {
		cam_zoom(c, x[0],
			 (struct px){ (double)c->sz.w / 2,
				      (double)c->sz.h / 2 });
	} else if (!strcmp(w[0], "size") && !args(w, n, x, 2) && x[0] >= 1 &&
		   x[1] >= 1 && x[0] <= WIN_MAX && x[1] <= WIN_MAX) {
		c->sz = (struct size){ (int)x[0], (int)x[1] };
		r->ask |= CMD_SIZE;
	} else {
		r->ask = 0;
		return 1;
	}
	return 0;
}

// Runs a command that prints or asks for frames into r; returns 1 if it
// is none.
static int out_command(const struct scene *s, char **w, size_t n, struct cmd *r)
{
	double x[2];

	if (!strcmp(w[0], "pick") && !args(w, n, x, 2)) {
		size_t i = scene_pick(s, (struct px){ x[0], x[1] });

		printf("%s\n", i == NONE ? "-" : s->p[i].name);
	} else if (!strcmp(w[0], "help") && n == 1) {
		for (size_t i = 0; view_help[i]; i++) {
			printf("%s\n", view_help[i]);
		}
	} else if (!strcmp(w[0], "list") && n == 1) {
		for (size_t i = 0; i < s->np; i++) {
			struct box b = part_box_at(&s->p[i]);
			struct vec m = mul(add(b.lo, b.hi), 1.0 / 2);

			printf("%s %zu %s %s at %.3f %.3f %.3f\n", s->p[i].name,
			       s->p[i].nt, s->p[i].shown ? "shown" : "hidden",
			       s->p[i].see ? "see" : "solid", m.x + 0.0,
			       m.y + 0.0, m.z + 0.0);
		}
	} else if (!strcmp(w[0], "shot") && n == 2) {
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

int scene_show_at(struct scene *s, int key, struct px at)
{
	size_t k = scene_pick(s, at);

	if (k == NONE) {
		return -1;
	}
	if (key == 's') {
		s->p[k].see = !s->p[k].see;
	}
	for (size_t i = 0; i < s->np && key != 's'; i++) {
		if (key == 'o' || i == k) {
			s->p[i].shown = key == 'o' && i == k;
		}
	}
	scene_commit(s);
	return 0;
}

// The item of the part or item named name, or NONE.
static size_t item_named(const struct scene *s, const char *name)
{
	for (size_t i = 0; i < s->nit; i++) {
		if (!strcmp(s->it[i].name, name)) {
			return i;
		}
	}
	for (size_t i = 0; i < s->np; i++) {
		if (!strcmp(s->p[i].name, name)) {
			return s->p[i].item;
		}
	}
	return NONE;
}

// Runs a command of the explode, the drags or the history into r;
// returns 1 if it is none.
static int arrange_command(struct scene *s, char **w, size_t n, struct cmd *r)
{
	double x[1];
	double y[2];

	r->ask = CMD_DRAW;
	if (!strcmp(w[0], "explode") && !args(w, n, x, 1) && x[0] >= 0 &&
	    x[0] <= EXPLODE_MAX) {
		scene_explode(s, x[0]);
		scene_commit(s);
	} else if (!strcmp(w[0], "drag") && n == 3 && !numbers(w + 2, x, 1) &&
		   item_named(s, w[1]) != NONE) {
		scene_drag(s, item_named(s, w[1]), x[0]);
		scene_commit(s);
	} else if (!strcmp(w[0], "home") && n == 1) {
		scene_home(s);
		scene_commit(s);
	} else if (!strcmp(w[0], "key") && n == 4 && w[1][0] && !w[1][1] &&
		   strchr("hos", w[1][0]) && !numbers(w + 2, y, 2)) {
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
// colour, or its picture and the texture coordinates of its corners.
struct stri {
	double x[3], y[3], z[3];
	unsigned char rgb[3];
	unsigned alpha; // The opacity of a colour, 0 to BYTE.
	const struct part *p;
	const float *uv;
};

// The frame of a render: the pixels and their depths.
struct frame {
	unsigned char *rgb;
	double *z;
	struct size sz;
};

// Lays the colour rgb over out by its opacity a, 0 to BYTE.
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
		struct vec d = sub(
			add(get3(&p->pos[(TRI_FLOATS * k) + (3 * i)]), p->off),
			c->c);

		t.x[i] = ((double)c->sz.w / 2) + (dot(d, c->r[0]) * px);
		t.y[i] = ((double)c->sz.h / 2) - (dot(d, c->r[1]) * px);
		t.z[i] = dot(d, c->r[2]);
		n = add(n, get3(&p->nrm[(TRI_FLOATS * k) + (3 * i)]));
	}
	light = len(n) > 0 ? shade(v3(dot(n, c->r[0]), dot(n, c->r[1]),
				      dot(n, c->r[2]))) :
			     AMBIENT;
	for (int i = 0; i < 3; i++) {
		double v = (double)(p->rgb[i] * (p->uv ? 1 : light));

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
