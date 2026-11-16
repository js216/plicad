// SPDX-License-Identifier: GPL-3.0-only
// vecgl.c -- Vector files on the GPU
// Copyright (c) 2026 Jakob Kastelic

// See vecgl.h.
#define _POSIX_C_SOURCE 200809L // For getline.
#define GL_GLEXT_PROTOTYPES

#include "vecgl.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846

static char *font_file, **parts;
static size_t nparts;

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

// The glyphs of lib/fonts/nimbussans.txt: by font and code point, the
// advance and the contours cut into lines, outlines and holes.
struct glyph {
	char font;
	long code;
	double adv, units, **ct;
	int *nct, *hole, n;
};

static struct glyph *glyphs;
static size_t nglyphs;

static void read_fonts(void)
{
	FILE *in = fopen(font_file, "r");
	char *line = NULL, font = 'R';
	size_t cap = 0;
	double units = 1000;

	if (!in) {
		fprintf(stderr, "%s: cannot read\n", font_file);
		exit(2);
	}
	while (getline(&line, &cap, in) > 0) {
		struct glyph *g = nglyphs ? &glyphs[nglyphs - 1] : NULL;
		char *s = line + 1, *end;
		double a, x = 0, y = 0, v[6];
		long code;
		int k;

		if (sscanf(line, "font %c %lf", &font, &units) == 2) {
			continue;
		}
		if (sscanf(line, "glyph %ld %lf", &code, &a) == 2) {
			glyphs = more(glyphs, (nglyphs + 1) * sizeof *glyphs);
			memset(&glyphs[nglyphs], 0, sizeof *glyphs);
			glyphs[nglyphs].font = font;
			glyphs[nglyphs].code = code;
			glyphs[nglyphs].units = units;
			glyphs[nglyphs++].adv = a;
			continue;
		}
		if (!g || (line[0] != '+' && line[0] != '-')) {
			continue;
		}
		k = g->n++;
		g->ct = more(g->ct, (size_t)g->n * sizeof *g->ct);
		g->nct = more(g->nct, (size_t)g->n * sizeof *g->nct);
		g->hole = more(g->hole, (size_t)g->n * sizeof *g->hole);
		g->ct[k] = more(NULL, 2 * 16 * strlen(line) * sizeof **g->ct);
		g->nct[k] = 0;
		g->hole[k] = line[0] == '-';
		for (char op; (op = *(s += strspn(s, " "))) == 'M' ||
			      op == 'L' || op == 'C';) {
			s++;
			for (int i = 0; i < (op == 'C' ? 6 : 2); i++) {
				v[i] = strtod(s, &end);
				s = end;
			}
			// A cubic is cut into 16 lines.
			for (int i = 1; i <= (op == 'C' ? 16 : 1); i++) {
				double t = op == 'C' ? i / 16.0 : 1, m = 1 - t;
				double *p = &g->ct[k][2 * g->nct[k]++];

				p[0] = op != 'C' ?
					       v[0] :
					       m * m * m * x +
						       3 * m * m * t * v[0] +
						       3 * m * t * t * v[2] +
						       t * t * t * v[4];
				p[1] = op != 'C' ?
					       v[1] :
					       m * m * m * y +
						       3 * m * m * t * v[1] +
						       3 * m * t * t * v[3] +
						       t * t * t * v[5];
			}
			x = op == 'C' ? v[4] : v[0];
			y = op == 'C' ? v[5] : v[1];
		}
	}
	free(line);
	fclose(in);
}

static const struct glyph *glyph(char font, long code)
{
	for (size_t i = 0; i < nglyphs; i++) {
		if (glyphs[i].font == font && glyphs[i].code == code) {
			return &glyphs[i];
		}
	}
	return code == '?' ? NULL : glyph(font, '?');
}

static long utf8(const unsigned char **s)
{
	long c = *(*s)++;
	int n = c >= 0xF0 ? 3 : c >= 0xE0 ? 2 : c >= 0xC0 ? 1 : 0;

	c &= n ? 0x3F >> n : 0x7F;
	while (n-- && (**s & 0xC0) == 0x80) {
		c = (c << 6) | (*(*s)++ & 0x3F);
	}
	return c;
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
	for (int i = 0; i < 2; i++) {
		rb[i] = fmin(rb[i], fmin(a[i], b[i]) - fabs(hw));
		rb[i + 2] = fmax(rb[i + 2], fmax(a[i], b[i]) + fabs(hw));
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
	    state.part == cur.part) {
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

// A text record: the outlines of its glyphs dark and their holes clear.
static void text(const char *line)
{
	char font;
	double size, angle, x, y, w = 0, c, sn;
	int align, at = 0, dark = state.dark;
	const unsigned char *s;

	if (sscanf(line, "text %c %lf %lf %d %lf %lf %n", &font, &size, &angle,
		   &align, &x, &y, &at) != 6 ||
	    !at) {
		return;
	}
	if (!nglyphs) {
		read_fonts();
	}
	c = cos(angle * PI / 180);
	sn = sin(angle * PI / 180);
	for (s = (const unsigned char *)line + at; *s;) {
		const struct glyph *g = glyph(font, utf8(&s));

		w += g ? g->adv * size / g->units : 0;
	}
	x -= c * w * align / 2;
	y -= sn * w * align / 2;
	for (s = (const unsigned char *)line + at; *s;) {
		const struct glyph *g = glyph(font, utf8(&s));
		double sc = g ? size / g->units : 0;

		for (int k = 0; g && k < 2 * g->n; k++) {
			const double *q = g->ct[k % g->n];
			double *p;

			if (g->hole[k % g->n] != k / g->n) {
				continue;
			}
			state.dark = k < g->n ? dark : !dark;
			run();
			p = more(NULL,
				 (size_t)g->nct[k % g->n] * 2 * sizeof *p);
			for (int i = 0; i < g->nct[k % g->n]; i++) {
				p[2 * i] =
					x +
					(q[2 * i] * c - q[2 * i + 1] * sn) * sc;
				p[2 * i + 1] =
					y +
					(q[2 * i] * sn + q[2 * i + 1] * c) * sc;
			}
			polygon(p, g->nct[k % g->n]);
			free(p);
		}
		x += g ? c * g->adv * sc : 0;
		y += g ? sn * g->adv * sc : 0;
	}
	state.dark = dark;
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
}

double vg_text_width(char font, const char *s)
{
	double w = 0;

	if (!nglyphs) {
		read_fonts();
	}
	for (const unsigned char *p = (const unsigned char *)s; *p;) {
		const struct glyph *g = glyph(font, utf8(&p));

		w += g ? g->adv / g->units : 0;
	}
	return w;
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
			open_elem = -1;
		} else if (!strncmp(line, "text ", 5)) {
			text(line);
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
			polygon(xy, n / 2);
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
			snprintf(name, size, "%.*s", (int)strcspn(buf + 6, " \n"),
				 buf + 6);
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
	"varying vec2 a, b;\n"
	"varying float hw, fade;\n"
	"void main() {\n"
	"  vec2 p = gl_FragCoord.xy, d = b - a;\n"
	"  float l = dot(d, d);\n"
	"  float t = l > 0.0 ? clamp(dot(p - a, d) / l, 0.0, 1.0) : 0.0;\n"
	"  float c = clamp(hw + 0.5 - length(p - a - t * d), 0.0, 1.0);\n"
	"  gl_FragColor = vec4(gl_Color.rgb, gl_Color.a * c * fade);\n"
	"}\n";
static GLuint prog;
static GLint u_view, u_zoom, u_solid, a_seg, a_hc;
int vg_solid, vg_hair;

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
			glDisableVertexAttribArray((GLuint)a_seg);
			glDisableVertexAttribArray((GLuint)a_hc);
			glUseProgram(0);
		}
	}
}

void vg_pick(const struct vg_file *f, double x, double y, double tol, int *best,
	     double *area)
{
	for (size_t i = 0; i < f->nelem; i++) {
		const double *b = f->elem[i].b;
		double a = (b[2] - b[0] + 2 * tol) * (b[3] - b[1] + 2 * tol);

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

void vg_init(const char *argv0)
{
	const char *slash = strrchr(argv0, '/');
	size_t len = slash ? (size_t)(slash - argv0) : 1;

	font_file = more(NULL, len + 32);
	memcpy(font_file, slash ? argv0 : ".", len);
	strcpy(font_file + len, "/../lib/fonts/nimbussans.txt");
	prog = glCreateProgram();
	shader(GL_VERTEX_SHADER, vs);
	shader(GL_FRAGMENT_SHADER, fs);
	glLinkProgram(prog);
	u_view = glGetUniformLocation(prog, "view");
	u_zoom = glGetUniformLocation(prog, "zoom");
	u_solid = glGetUniformLocation(prog, "solid");
	a_seg = glGetAttribLocation(prog, "seg");
	a_hc = glGetAttribLocation(prog, "hc");
}
