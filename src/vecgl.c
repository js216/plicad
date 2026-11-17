// SPDX-License-Identifier: GPL-3.0-only
// vecgl.c -- Vector files on the GPU
// Copyright (c) 2026 Jakob Kastelic

// See vecgl.h.
#define _POSIX_C_SOURCE 200809L // For getline.
#define GL_GLEXT_PROTOTYPES

#include "vecgl.h"
#include "stroke.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846

// The names of the parts and of the nets, and the class of each net
// (NULL for none).
static char **parts, **nets, **classes;
static size_t nparts, nnets, nclasses;

static void *more(void *p, size_t n)
{
	p = p ? realloc(p, n) : malloc(n);
	if (!p) {
		fprintf(stderr, "out of memory\n");
		exit(2);
	}
	return p;
}

int vg_part(const char *s)
{
	for (size_t i = 0; i < nparts; i++) {
		if (!strcmp(parts[i], s)) {
			return (int)i;
		}
	}
	parts = more(parts, (nparts + 1) * sizeof *parts);
	parts[nparts] = more(NULL, strlen(s) + 1);
	strcpy(parts[nparts], s);
	return (int)nparts++;
}

const char *vg_part_name(int i)
{
	return i >= 0 && (size_t)i < nparts ? parts[i] : "";
}

int vg_net(const char *s)
{
	for (size_t i = 0; i < nnets; i++) {
		if (!strcmp(nets[i], s)) {
			return (int)i;
		}
	}
	nets = more(nets, (nnets + 1) * sizeof *nets);
	nets[nnets] = more(NULL, strlen(s) + 1);
	strcpy(nets[nnets], s);
	return (int)nnets++;
}

const char *vg_net_name(int i)
{
	return i >= 0 && (size_t)i < nnets ? nets[i] : "";
}

int vg_net_record(char *s)
{
	char *c = s + strcspn(s, " ");
	int i;

	if (*c) {
		*c++ = 0;
	}
	i = vg_net(s);
	classes = more(classes, nnets * sizeof *classes);
	for (size_t k = nclasses; k < nnets; k++) {
		classes[k] = NULL;
	}
	nclasses = nnets;
	if (!strncmp(c, "class ", 6) && c[6] && !classes[i]) {
		c += 6;
		c[strcspn(c, " ")] = 0;
		classes[i] = more(NULL, strlen(c) + 1);
		strcpy(classes[i], c);
	}
	return i;
}

const char *vg_net_class(int i)
{
	return i >= 0 && (size_t)i < nclasses ? classes[i] : NULL;
}

// The file being read: its triangles' points, its segments (six points
// a segment, of both ends, the half width and the corner), the state of
// its records, the open run, the record's box, the element it may join.
static float *pt, *sv;
static size_t npt, capt, nsv, capsv;
static struct vg_file *F;
static struct vg_run state, cur;
static double rb[4];
static long open_elem;
static int sheet; // The file has a page: it is a sheet.
static struct vg_file *files[64]; // Those read, for centering a part.
static size_t nfiles;

static void point(const double *p)
{
	if (npt + 2 > capt) {
		capt = capt ? 2 * capt : 1 << 16;
		pt = more(pt, capt * sizeof *pt);
	}
	pt[npt++] = (float)p[0];
	pt[npt++] = (float)p[1];
	for (int i = 0; i < 4; i++) {
		rb[i] = i < 2 ? fmin(rb[i], p[i % 2]) : fmax(rb[i], p[i % 2]);
	}
}

static void segment(const double *a, const double *b, double hw)
{
	if (nsv + 36 > capsv) {
		capsv = capsv ? 2 * capsv : 1 << 16;
		sv = more(sv, capsv * sizeof *sv);
	}
	for (int c = 0; c < 6; c++) {
		const float s[6] = { (float)a[0], (float)a[1], (float)b[0],
				     (float)b[1], (float)hw,   (float)c };

		memcpy(sv + nsv, s, sizeof s);
		nsv += 6;
	}
	// A half width below 0 is in pixels, not in the units of the box.
	for (int i = 0; i < 2; i++) {
		rb[i] = fmin(rb[i], fmin(a[i], b[i]) - fmax(hw, 0));
		rb[i + 2] = fmax(rb[i + 2], fmax(a[i], b[i]) + fmax(hw, 0));
	}
}

#define cross(a, b, c)                           \
	(((b)[0] - (a)[0]) * ((c)[1] - (a)[1]) - \
	 ((b)[1] - (a)[1]) * ((c)[0] - (a)[0]))

// Cuts a polygon into triangles by its ears; the rest of one not simple
// as a fan.
static void polygon(const double *p, int n)
{
	int *v = more(NULL, (size_t)n * sizeof *v), m = n, cut = 1;
	double area = 0;

	for (int i = 0; i < n; i++) {
		v[i] = i;
		area += cross(p, &p[2 * i], &p[2 * ((i + 1) % n)]);
	}
	while (m > 3 && cut) {
		cut = 0;
		for (int i = 0; i < m && m > 3; i++) {
			const double *a = &p[2 * v[(i + m - 1) % m]];
			const double *b = &p[2 * v[i]],
				     *c = &p[2 * v[(i + 1) % m]];
			int ear = cross(a, b, c) * area > 0;

			for (int k = 0; ear && k < m; k++) {
				const double *q = &p[2 * v[k]];

				ear = q == a || q == b || q == c ||
				      cross(a, b, q) * area < 0 ||
				      cross(b, c, q) * area < 0 ||
				      cross(c, a, q) * area < 0;
			}
			if (ear) {
				point(a);
				point(b);
				point(c);
				memmove(&v[i], &v[i + 1],
					(size_t)(m - i - 1) * sizeof *v);
				m--;
				cut = 1;
			}
		}
	}
	for (int i = 1; i + 1 < m; i++) {
		point(&p[2 * v[0]]);
		point(&p[2 * v[i]]);
		point(&p[2 * v[i + 1]]);
	}
	free(v);
}

// Ends the open run if the state has changed, and opens one of it.
static void run(void)
{
	if (state.dark == cur.dark && state.zone == cur.zone &&
	    state.edge == cur.edge && state.via == cur.via &&
	    state.part == cur.part && state.net == cur.net &&
	    state.text == cur.text && state.frame == cur.frame) {
		return;
	}
	cur.count = (int)(npt / 2) - cur.first;
	cur.scount = (int)(nsv / 6) - cur.sfirst;
	if (cur.count || cur.scount) {
		F->run = more(F->run, (F->nrun + 1) * sizeof *F->run);
		F->run[F->nrun++] = cur;
	}
	cur = state;
	cur.first = (int)(npt / 2);
	cur.sfirst = (int)(nsv / 6);
}

// Takes the record just drawn into the elements of its part: a text is
// one by itself; on a sheet (a file with a page) the drawings one after
// another are one, the body; on a board each record is one, but lines
// one after another (an outline).
static void element(char kind)
{
	int join = state.part >= 0 && kind != 't' && (sheet || kind == 'l');
	struct vg_elem *e = open_elem >= 0 ? &F->elem[open_elem] : NULL;

	if (state.part >= 0 && rb[2] >= rb[0] && state.dark) {
		if (join && e && e->part == state.part) {
			for (int i = 0; i < 4; i++) {
				e->b[i] = i < 2 ? fmin(e->b[i], rb[i]) :
						  fmax(e->b[i], rb[i]);
			}
		} else {
			F->elem =
				more(F->elem, (F->nelem + 1) * sizeof *F->elem);
			F->elem[F->nelem].part = state.part;
			memcpy(F->elem[F->nelem].b, rb, sizeof rb);
			open_elem = join ? (long)F->nelem : -1;
			F->nelem++;
		}
	}
	for (int i = 0; i < 4; i++) {
		F->box[i] = i < 2 ? fmin(F->box[i], rb[i]) :
				    fmax(F->box[i], rb[i]);
		rb[i] = i < 2 ? 1e300 : -1e300;
	}
}

// A text record: segments of the pen of the stroke font, its size the
// capital height.
static void text(const char *line)
{
	double size, angle, x, y, w, c, sn, *sg;
	int align, at = 0;
	size_t n;

	if (sscanf(line, "text %lf %lf %d %lf %lf %n", &size, &angle, &align,
		   &x, &y, &at) != 5 ||
	    !at) {
		return;
	}
	c = cos(angle * PI / 180);
	sn = sin(angle * PI / 180);
	n = stroke_text(line + at, size, &sg);
	w = stroke_width(line + at, size);
	x -= c * w * align / 2;
	y -= sn * w * align / 2;
	run();
	for (size_t k = 0; k < n; k++) {
		double a[2], b[2];

		a[0] = x + sg[4 * k] * c - sg[4 * k + 1] * sn;
		a[1] = y + sg[4 * k] * sn + sg[4 * k + 1] * c;
		b[0] = x + sg[4 * k + 2] * c - sg[4 * k + 3] * sn;
		b[1] = y + sg[4 * k + 2] * sn + sg[4 * k + 3] * c;
		segment(a, b, STROKE_PEN * size / 2);
	}
	free(sg);
}

static void upload(unsigned *buf, const float *p, size_t n)
{
	glGenBuffers(1, buf);
	glBindBuffer(GL_ARRAY_BUFFER, *buf);
	glBufferData(GL_ARRAY_BUFFER, (long)(n * 4), p, GL_STATIC_DRAW);
}

void vg_load(struct vg_file *f, const char *path, double via)
{
	FILE *in = fopen(path, "r");

	if (!in) {
		fprintf(stderr, "%s: cannot read\n", path);
		exit(2);
	}
	vg_load_stream(f, in, via);
	fclose(in);
}

void vg_free(struct vg_file *f)
{
	if (f->buf || f->sbuf) {
		glDeleteBuffers(1, &f->buf);
		glDeleteBuffers(1, &f->sbuf);
	}
	free(f->run);
	free(f->elem);
	memset(f, 0, sizeof *f);
	// No longer one of the files that name parts.
	for (size_t i = 0; i < nfiles; i++) {
		if (files[i] == f) {
			files[i--] = files[--nfiles];
		}
	}
}

void vg_load_stream(struct vg_file *f, FILE *in, double via)
{
	char *line = NULL;
	size_t cap = 0, capxy = 0;
	double *xy = NULL;

	memset(f, 0, sizeof *f);
	F = f;
	sheet = 0;
	npt = nsv = 0;
	open_elem = -1;
	memset(&state, 0, sizeof state);
	state.dark = 1;
	state.part = -1;
	state.net = -1;
	cur = state;
	for (int i = 0; i < 4; i++) {
		f->box[i] = rb[i] = i < 2 ? 1e300 : -1e300;
	}
	while (getline(&line, &cap, in) > 0) {
		char kind = line[0], *s = line + 4, *end;
		int n = 0;
		double w = 0, v;

		line[strcspn(line, "\n")] = 0;
		if (!strncmp(line, "page ", 5)) {
			sheet = 1;
			state.frame = 1;
		} else if (!strcmp(line, "dark") || !strcmp(line, "clear")) {
			state.dark = kind == 'd';
		} else if (!strncmp(line, "zone ", 5) || !strcmp(line, "end")) {
			state.zone = kind == 'z';
		} else if (!strncmp(line, "part", 4)) {
			// part NAME [REF]: the name keys the part.
			if (line[4]) {
				line[5 + strcspn(line + 5, " ")] = 0;
			}
			state.part = line[4] ? vg_part(line + 5) : -1;
			state.frame = 0;
			open_elem = -1;
		} else if (!strncmp(line, "net", 3) &&
			   (!line[3] || line[3] == ' ')) {
			state.net = line[3] ? vg_net_record(line + 4) : -1;
			state.frame = 0;
		} else if (!strncmp(line, "text ", 5)) {
			state.text = 1;
			text(line);
			state.text = 0;
			element('t');
		}
		if (strncmp(line, "disk ", 5) && strncmp(line, "line ", 5) &&
		    strncmp(line, "poly ", 5)) {
			continue;
		}
		w = kind == 'l' ? strtod(s, &s) : 0;
		while ((v = strtod(s, &end)), end != s) {
			if ((size_t)n + 1 > capxy) {
				capxy = capxy ? 2 * capxy : 1024;
				xy = more(xy, capxy * sizeof *xy);
			}
			xy[n++] = v;
			s = end;
		}
		state.via = kind == 'd' && n == 3 && xy[2] <= via;
		run();
		if (kind == 'd' && n == 3) {
			segment(xy, xy, xy[2]);
		} else if (kind == 'l') {
			for (int k = 2; k + 1 < n || k == 2; k += 2) {
				segment(&xy[k - 2], &xy[k < n ? k : 0],
					w > 0 && !vg_hair ? w / 2 : -0.75);
			}
		} else if (n >= 6) {
			double keep[4];

			polygon(xy, n / 2);
			// A rim of a pixel along its edge, for the edge the
			// smoothness of a stroke; the box stays the polygon's.
			memcpy(keep, rb, sizeof keep);
			for (int k = 0; k < n; k += 2) {
				segment(&xy[k], &xy[(k + 2) % n], -0.5);
			}
			memcpy(rb, keep, sizeof keep);
			// A zone's outline, for drawing the zones as outlines.
			state.edge = state.zone && state.dark;
			run();
			for (int k = 0; state.edge && k < n; k += 2) {
				segment(&xy[k], &xy[(k + 2) % n], -0.75);
			}
			state.edge = 0;
		}
		element(kind);
	}
	state.part = -2; // No run is of this state: the open one ends.
	run();
	upload(&f->buf, pt, npt);
	upload(&f->sbuf, sv, nsv);
	f->ntri = npt / 6;
	f->nseg = nsv / 36;
	if (f->nelem && nfiles < 64) {
		files[nfiles++] = f; // It names parts.
	}
	free(line);
	free(xy);
}

void vg_layer(const char *path, char *name, size_t size)
{
	FILE *in = fopen(path, "r");
	char buf[256];

	*name = 0;
	while (in && fgets(buf, sizeof buf, in)) {
		if (!strncmp(buf, "layer ", 6)) {
			snprintf(name, size, "%.*s",
				 (int)strcspn(buf + 6, " \n"), buf + 6);
			break;
		}
		if (!strncmp(buf, "disk ", 5) || !strncmp(buf, "line ", 5) ||
		    !strncmp(buf, "poly ", 5) || !strncmp(buf, "text ", 5)) {
			break; // A drawing: the layer comes before.
		}
	}
	if (in) {
		fclose(in);
	}
}

// The shader of the segments: the vertex places a corner of the quad a
// pixel past the half width (at least half a pixel); the fragment takes
// its coverage from its distance to the segment, fainter for a line
// thinner than a pixel (unless vg_solid); a half width below 0 is in
// pixels, the same at every zoom: a hairline.
static const char *const vs =
	"#version 120\n"
	"attribute vec4 seg;\n"
	"attribute vec2 hc;\n"
	"uniform vec2 view;\n"
	"uniform float zoom, solid;\n"
	"varying vec2 a, b;\n"
	"varying float hw, fade;\n"
	"void main() {\n"
	"  vec4 A = gl_ModelViewProjectionMatrix * vec4(seg.xy, 0.0, 1.0);\n"
	"  vec4 B = gl_ModelViewProjectionMatrix * vec4(seg.zw, 0.0, 1.0);\n"
	"  a = (A.xy * 0.5 + 0.5) * view;\n"
	"  b = (B.xy * 0.5 + 0.5) * view;\n"
	"  float w = hc.x < 0.0 ? -2.0 * hc.x : 2.0 * hc.x * zoom, l = length(b - a), e;\n"
	"  fade = max(min(w, 1.0), solid);\n"
	"  hw = max(w, 1.0) * 0.5;\n"
	"  e = hw + 1.0;\n"
	"  vec2 u = l > 0.0 ? (b - a) / l : vec2(1.0, 0.0), n = vec2(-u.y, u.x);\n"
	"  int c = int(hc.y + 0.5);\n"
	"  vec2 q = (c == 0 || c == 3 || c == 5) ? a - u * e : b + u * e;\n"
	"  q += (c == 0 || c == 1 || c == 3) ? -n * e : n * e;\n"
	"  gl_Position = vec4(q / view * 2.0 - 1.0, A.z, 1.0);\n"
	"  gl_FrontColor = gl_Color;\n"
	"}\n";
static const char *const fs =
	"#version 120\n"
	"uniform float brighter;\n"
	"varying vec2 a, b;\n"
	"varying float hw, fade;\n"
	"void main() {\n"
	"  vec2 p = gl_FragCoord.xy, d = b - a;\n"
	"  float l = dot(d, d);\n"
	"  float t = l > 0.0 ? clamp(dot(p - a, d) / l, 0.0, 1.0) : 0.0;\n"
	"  float r = length(p - a - t * d);\n"
	"  float c = clamp(hw + 0.5 - r, 0.0, 1.0);\n"
	"  if (c <= 0.0) discard;\n"
	"  float a = gl_Color.a * c * fade;\n"
	"  gl_FragColor = brighter > 0.0 ? vec4(gl_Color.rgb * a, a) :\n"
	"                                  vec4(gl_Color.rgb, a);\n"
	"}\n";
static GLuint prog;
static GLint u_view, u_zoom, u_solid, u_brighter, a_seg, a_hc;
int vg_solid, vg_hair, vg_brighter;

static void shader(GLenum kind, const char *src)
{
	GLuint sh = glCreateShader(kind);
	GLint ok = 0;

	glShaderSource(sh, 1, &src, NULL);
	glCompileShader(sh);
	glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
	if (!ok) {
		fprintf(stderr, "a shader does not compile\n");
		exit(2);
	}
	glAttachShader(prog, sh);
}

void vg_draw(const struct vg_file *f, int (*want)(const struct vg_run *),
	     void (*each)(const struct vg_run *))
{
	for (size_t i = 0; i < f->nrun; i++) {
		const struct vg_run *r = &f->run[i];

		if (!want(r)) {
			continue;
		}
		each(r);
		if (r->count) {
			glBindBuffer(GL_ARRAY_BUFFER, f->buf);
			glVertexPointer(2, GL_FLOAT, 0, NULL);
			glDrawArrays(GL_TRIANGLES, r->first, r->count);
		}
		if (r->scount) {
			glUseProgram(prog);
			glUniform2f(u_view, (float)vg_w, (float)vg_h);
			glUniform1f(u_zoom, (float)vg_zoom);
			glUniform1f(u_solid, (float)vg_solid);
			glUniform1f(u_brighter, (float)vg_brighter);
			if (vg_brighter) {
				glBlendEquation(GL_MAX);
			}
			glBindBuffer(GL_ARRAY_BUFFER, f->sbuf);
			glEnableVertexAttribArray((GLuint)a_seg);
			glEnableVertexAttribArray((GLuint)a_hc);
			glVertexAttribPointer((GLuint)a_seg, 4, GL_FLOAT,
					      GL_FALSE, 6 * sizeof(float),
					      NULL);
			glVertexAttribPointer((GLuint)a_hc, 2, GL_FLOAT,
					      GL_FALSE, 6 * sizeof(float),
					      (void *)(4 * sizeof(float)));
			glDrawArrays(GL_TRIANGLES, r->sfirst, r->scount);
			glBlendEquation(GL_FUNC_ADD);
			glDisableVertexAttribArray((GLuint)a_seg);
			glDisableVertexAttribArray((GLuint)a_hc);
			glUseProgram(0);
		}
	}
}

#define NEAR 1e12 // The score of a box the point is only near, a unit away.

void vg_pick(const struct vg_file *f, double x, double y, double tol, int *best,
	     double *area)
{
	for (size_t i = 0; i < f->nelem; i++) {
		const double *b = f->elem[i].b;
		double dx = fmax(fmax(b[0] - x, x - b[2]), 0);
		double dy = fmax(fmax(b[1] - y, y - b[3]), 0);
		// A box the point is in, the smallest first, comes before one
		// it is only near, the nearest first.
		double a = dx > 0 || dy > 0 ?
				   NEAR * (1 + sqrt(dx * dx + dy * dy)) :
				   (b[2] - b[0]) * (b[3] - b[1]);

		if (x >= b[0] - tol && x <= b[2] + tol && y >= b[1] - tol &&
		    y <= b[3] + tol && a < *area) {
			*area = a;
			*best = f->elem[i].part;
		}
	}
}

void vg_part_box(int part, double b[4])
{
	b[0] = b[1] = 1e300;
	b[2] = b[3] = -1e300;
	for (size_t i = 0; part >= 0 && i < nfiles; i++) {
		for (size_t k = 0; k < files[i]->nelem; k++) {
			for (int j = 0; files[i]->elem[k].part == part && j < 4;
			     j++) {
				double e = files[i]->elem[k].b[j];

				b[j] = j < 2 ? fmin(b[j], e) : fmax(b[j], e);
			}
		}
	}
}

void vg_init(void)
{
	prog = glCreateProgram();
	shader(GL_VERTEX_SHADER, vs);
	shader(GL_FRAGMENT_SHADER, fs);
	glLinkProgram(prog);
	u_view = glGetUniformLocation(prog, "view");
	u_zoom = glGetUniformLocation(prog, "zoom");
	u_solid = glGetUniformLocation(prog, "solid");
	u_brighter = glGetUniformLocation(prog, "brighter");

	a_seg = glGetAttribLocation(prog, "seg");
	a_hc = glGetAttribLocation(prog, "hc");
}
