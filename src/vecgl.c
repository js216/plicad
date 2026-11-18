// SPDX-License-Identifier: GPL-3.0-only
// vecgl.c -- Vector files on the GPU
// Copyright (c) 2026 Jakob Kastelic

// See vecgl.h.
#define GL_GLEXT_PROTOTYPES

#include "vecgl.h"
#include "stroke.h"
#include "util.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define FAR 1e300 // Past any box: where one to grow starts.
// The half widths, in pixels, of a hairline and of the rim of a polygon.
#define HAIR (-0.75)
#define RIM (-0.5)

enum {
	HALF_TURN = 180, // Degrees.
	// The room first made, for the floats of the points and the corners
	// of the segments, and for the numbers of a record.
	ROOM = 65536,
	XY_ROOM = 1024,
	TRI_FLOATS = 6, // A triangle's points, two floats each.
	// A segment's corners, and the floats of a corner: of both ends in
	// one buffer, of the half width and the corner in the other.
	SEG_CORNERS = 6,
	SEG_ENDS = 4,
	SEG_HC = 2,
	TEXT_FIELDS = 5, // The numbers before the text of a text record.
	LINE_LEN = 256 // Room for a line looked at for the layer.
};

// Whether s starts with pre.
static int starts(const char *s, const char *pre)
{
	return !strncmp(s, pre, strlen(pre));
}

// The names of the parts and of the nets, and the class of each net
// (NULL for none).

static void *more(void *p, size_t n)
{
	p = p ? realloc(p, n) : malloc(n);
	if (!p) {
		(void)fprintf(stderr, "out of memory\n");
		exit(2);
	}
	return p;
}

// A copy of s.
static char *copy(const char *s)
{
	size_t n = strlen(s) + 1;
	char *c = more(NULL, n);

	memcpy(c, s, n);
	return c;
}

int vg_part(struct vg *vg, const char *s)
{
	for (size_t i = 0; i < vg->nparts; i++) {
		if (!strcmp(vg->parts[i], s)) {
			return (int)i;
		}
	}
	vg->parts = (char **)more((void *)vg->parts,
				  (vg->nparts + 1) * sizeof *vg->parts);
	vg->parts[vg->nparts] = copy(s);
	return (int)vg->nparts++;
}

const char *vg_part_name(struct vg *vg, int i)
{
	return i >= 0 && (size_t)i < vg->nparts ? vg->parts[i] : "";
}

int vg_net(struct vg *vg, const char *s)
{
	for (size_t i = 0; i < vg->nnets; i++) {
		if (!strcmp(vg->nets[i], s)) {
			return (int)i;
		}
	}
	vg->nets = (char **)more((void *)vg->nets,
				 (vg->nnets + 1) * sizeof *vg->nets);
	vg->classes = (char **)more((void *)vg->classes,
				    (vg->nnets + 1) * sizeof *vg->classes);
	vg->nets[vg->nnets] = copy(s);
	vg->classes[vg->nnets] = NULL; // None until a net record gives one.
	return (int)vg->nnets++;
}

const char *vg_net_name(struct vg *vg, int i)
{
	return i >= 0 && (size_t)i < vg->nnets ? vg->nets[i] : "";
}

int vg_net_record(struct vg *vg, char *s)
{
	char *c = s + strcspn(s, " ");
	int i = 0;

	if (*c) {
		*c++ = 0;
	}
	i = vg_net(vg, s);
	if (starts(c, "class ") && c[strlen("class ")] && !vg->classes[i]) {
		c += strlen("class ");
		c[strcspn(c, " ")] = 0;
		vg->classes[i] = copy(c);
	}
	return i;
}

const char *vg_net_class(struct vg *vg, int i)
{
	return i >= 0 && (size_t)i < vg->nnets ? vg->classes[i] : NULL;
}

static void point(struct vg *vg, const double *p)
{
	if (vg->npt + 2 > vg->capt) {
		vg->capt = vg->capt ? 2 * vg->capt : ROOM;
		vg->pt = more(vg->pt, vg->capt * sizeof *vg->pt);
	}
	vg->pt[vg->npt++] = (float)p[0];
	vg->pt[vg->npt++] = (float)p[1];
	for (int i = 0; i < 4; i++) {
		vg->rb[i] = i < 2 ? fmin(vg->rb[i], p[i % 2]) :
				    fmax(vg->rb[i], p[i % 2]);
	}
}

static void segment(struct vg *vg, const double *a, const double *b, double hw)
{
	if (vg->nsv + SEG_CORNERS > vg->capsv) {
		vg->capsv = vg->capsv ? 2 * vg->capsv : ROOM;
		vg->sv = more(vg->sv, vg->capsv * SEG_ENDS * sizeof *vg->sv);
		vg->hc = more(vg->hc, vg->capsv * SEG_HC * sizeof *vg->hc);
	}
	for (int c = 0; c < SEG_CORNERS; c++) {
		const float ends[SEG_ENDS] = { (float)a[0], (float)a[1],
					       (float)b[0], (float)b[1] };
		const float hc[SEG_HC] = { (float)hw, (float)c };

		memcpy(vg->sv + (vg->nsv * SEG_ENDS), ends, sizeof ends);
		memcpy(vg->hc + (vg->nsv * SEG_HC), hc, sizeof hc);
		vg->nsv++;
	}
	// A half width below 0 is in pixels, not in the units of the box.
	for (int i = 0; i < 2; i++) {
		vg->rb[i] = fmin(vg->rb[i], fmin(a[i], b[i]) - fmax(hw, 0));
		vg->rb[i + 2] =
			fmax(vg->rb[i + 2], fmax(a[i], b[i]) + fmax(hw, 0));
	}
}

#define cross(a, b, c)                           \
	(((b)[0] - (a)[0]) * ((c)[1] - (a)[1]) - \
	 ((b)[1] - (a)[1]) * ((c)[0] - (a)[0]))

// Cuts a polygon into triangles by its ears; the rest of one not simple
// as a fan.
static void polygon(struct vg *vg, const double *p, int n)
{
	int *v = more(NULL, (size_t)n * sizeof *v);
	int m = n;
	int cut = 1;
	double area = 0;

	for (int i = 0; i < n; i++) {
		v[i] = i;
		area += cross(p, &p[2 * (size_t)i],
			      &p[2 * (size_t)((i + 1) % n)]);
	}
	while (m > 3 && cut) {
		cut = 0;
		for (int i = 0; i < m && m > 3; i++) {
			const double *a = &p[2 * (size_t)v[(i + m - 1) % m]];
			const double *b = &p[2 * (size_t)v[i]];
			const double *c = &p[2 * (size_t)v[(i + 1) % m]];
			int ear = cross(a, b, c) * area > 0;

			for (int k = 0; ear && k < m; k++) {
				const double *q = &p[2 * (size_t)v[k]];

				ear = q == a || q == b || q == c ||
				      cross(a, b, q) * area < 0 ||
				      cross(b, c, q) * area < 0 ||
				      cross(c, a, q) * area < 0;
			}
			if (ear) {
				point(vg, a);
				point(vg, b);
				point(vg, c);
				memmove(&v[i], &v[i + 1],
					(size_t)(m - i - 1) * sizeof *v);
				m--;
				cut = 1;
			}
		}
	}
	for (int i = 1; i + 1 < m; i++) {
		point(vg, &p[2 * (size_t)v[0]]);
		point(vg, &p[2 * (size_t)v[i]]);
		point(vg, &p[2 * (size_t)v[i + 1]]);
	}
	free(v);
}

// Ends the open run if the state has changed, and opens one of it.
static void run(struct vg *vg)
{
	if (vg->state.dark == vg->cur.dark && vg->state.zone == vg->cur.zone &&
	    vg->state.edge == vg->cur.edge && vg->state.via == vg->cur.via &&
	    vg->state.part == vg->cur.part && vg->state.net == vg->cur.net &&
	    vg->state.text == vg->cur.text &&
	    vg->state.frame == vg->cur.frame) {
		return;
	}
	vg->cur.count = (int)(vg->npt / 2) - vg->cur.first;
	vg->cur.scount = (int)vg->nsv - vg->cur.sfirst;
	if (vg->cur.count || vg->cur.scount) {
		vg->file->run =
			more(vg->file->run,
			     (vg->file->nrun + 1) * sizeof *vg->file->run);
		vg->file->run[vg->file->nrun++] = vg->cur;
	}
	vg->cur = vg->state;
	vg->cur.first = (int)(vg->npt / 2);
	vg->cur.sfirst = (int)vg->nsv;
}

// Takes the record just drawn into the elements of its part: a text is
// one by itself; on a sheet (a file with a page) the drawings one after
// another are one, the body; on a board each record is one, but lines
// one after another (an outline).
static void element(struct vg *vg, char kind)
{
	int join = vg->state.part >= 0 && kind != 't' &&
		   (vg->sheet || kind == 'l');
	struct vg_elem *e =
		vg->open_elem >= 0 ? &vg->file->elem[vg->open_elem] : NULL;

	if (vg->state.part >= 0 && vg->rb[2] >= vg->rb[0] && vg->state.dark) {
		if (join && e && e->part == vg->state.part) {
			for (int i = 0; i < 4; i++) {
				e->b[i] = i < 2 ? fmin(e->b[i], vg->rb[i]) :
						  fmax(e->b[i], vg->rb[i]);
			}
		} else {
			vg->file->elem = more(vg->file->elem,
					      (vg->file->nelem + 1) *
						      sizeof *vg->file->elem);
			vg->file->elem[vg->file->nelem].part = vg->state.part;
			memcpy(vg->file->elem[vg->file->nelem].b, vg->rb,
			       sizeof vg->rb);
			vg->open_elem = join ? (long)vg->file->nelem : -1;
			vg->file->nelem++;
		}
	}
	for (int i = 0; i < 4; i++) {
		vg->file->box[i] = i < 2 ? fmin(vg->file->box[i], vg->rb[i]) :
					   fmax(vg->file->box[i], vg->rb[i]);
		vg->rb[i] = i < 2 ? FAR : -FAR;
	}
}

// A text record: segments of the pen of the stroke font, its size the
// capital height.
static void text(struct vg *vg, const char *line)
{
	double size = NAN;
	double angle = NAN;
	double x = NAN;
	double y = NAN;
	double w = NAN;
	double c = NAN;
	double sn = NAN;
	double *sg = NULL;
	const char *r = line;
	int align = 0;
	int at = 0;
	size_t n = 0;

	if (scan(&r, "text %lf %lf %d %lf %lf %n", &size, &angle, &align, &x,
		 &y, &at) != TEXT_FIELDS ||
	    !at) {
		return;
	}
	c = cos(angle * PI / HALF_TURN);
	sn = sin(angle * PI / HALF_TURN);
	n = stroke_text(line + at, size, &sg);
	w = stroke_width(line + at, size);
	x -= c * w * align / 2;
	y -= sn * w * align / 2;
	run(vg);
	for (size_t k = 0; k < n; k++) {
		double a[2];
		double b[2];

		a[0] = x + sg[4 * k] * c - sg[(4 * k) + 1] * sn;
		a[1] = y + sg[4 * k] * sn + sg[(4 * k) + 1] * c;
		b[0] = x + sg[(4 * k) + 2] * c - sg[(4 * k) + 3] * sn;
		b[1] = y + sg[(4 * k) + 2] * sn + sg[(4 * k) + 3] * c;
		segment(vg, a, b, STROKE_PEN * size / 2);
	}
	free(sg);
}

static void upload(unsigned *buf, const float *p, size_t n)
{
	glGenBuffers(1, buf);
	glBindBuffer(GL_ARRAY_BUFFER, *buf);
	glBufferData(GL_ARRAY_BUFFER, (long)(n * 4), p, GL_STATIC_DRAW);
}

void vg_load(struct vg *vg, struct vg_file *f, const char *path, double via)
{
	FILE *in = fopen(path, "r");

	if (!in) {
		(void)fprintf(stderr, "%s: cannot read\n", path);
		exit(2);
	}
	vg_load_stream(vg, f, in, via);
	(void)fclose(in);
}

void vg_free(struct vg *vg, struct vg_file *f)
{
	if (f->buf || f->sbuf) {
		glDeleteBuffers(1, &f->buf);
		glDeleteBuffers(1, &f->sbuf);
		glDeleteBuffers(1, &f->hbuf);
	}
	free(f->run);
	free(f->elem);
	memset(f, 0, sizeof *f);
	// No longer one of the files that name parts.
	for (size_t i = 0; i < vg->nfiles; i++) {
		if (vg->files[i] == f) {
			vg->files[i--] = vg->files[--vg->nfiles];
		}
	}
}

// Starts reading f: no runs, the state of a file's start.
static void start_file(struct vg *vg, struct vg_file *f)
{
	memset(f, 0, sizeof *f);
	vg->file = f;
	vg->sheet = 0;
	vg->npt = vg->nsv = 0;
	vg->open_elem = -1;
	memset(&vg->state, 0, sizeof vg->state);
	vg->state.dark = 1;
	vg->state.part = -1;
	vg->state.net = -1;
	vg->cur = vg->state;
	for (int i = 0; i < 4; i++) {
		f->box[i] = vg->rb[i] = i < 2 ? FAR : -FAR;
	}
}

// The index of the part of a part record, s the text after "part ":
// NAME [REF], the name keying the part; cuts s at its name.
static int part_record(struct vg *vg, char *s)
{
	s[strcspn(s, " ")] = 0;
	return vg_part(vg, s);
}

// A record that sets the state (or a text, drawn as it is read).
static void state_record(struct vg *vg, char *line)
{
	char kind = line[0];

	if (starts(line, "page ")) {
		vg->sheet = 1;
		vg->state.frame = 1;
	} else if (!strcmp(line, "dark") || !strcmp(line, "clear")) {
		vg->state.dark = kind == 'd';
	} else if (starts(line, "zone ") || !strcmp(line, "end")) {
		vg->state.zone = kind == 'z';
	} else if (starts(line, "part")) {
		vg->state.part =
			line[4] ? part_record(vg, line + strlen("part ")) : -1;
		vg->state.frame = 0;
		vg->open_elem = -1;
	} else if (starts(line, "net") && (!line[3] || line[3] == ' ')) {
		vg->state.net = line[3] ? vg_net_record(vg, line + 4) : -1;
		vg->state.frame = 0;
	} else if (starts(line, "text ")) {
		vg->state.text = 1;
		text(vg, line);
		vg->state.text = 0;
		element(vg, 't');
	}
}

// The numbers of s into *xy, with room for *cap; returns their number.
static int numbers(const char *s, double **xy, size_t *cap)
{
	char *end = NULL;
	double v = NAN;
	int n = 0;

	while ((v = strtod(s, &end)), end != s) {
		if ((size_t)n + 1 > *cap) {
			*cap = *cap ? 2 * *cap : XY_ROOM;
			*xy = more(*xy, *cap * sizeof **xy);
		}
		(*xy)[n++] = v;
		s = end;
	}
	return n;
}

// A polygon of n numbers: its triangles, a rim of a pixel along its edge
// (for the edge the smoothness of a stroke; the box stays the
// polygon's), and a zone's outline, for drawing the zones as outlines.
static void poly_record(struct vg *vg, const double *xy, int n)
{
	double keep[4];

	polygon(vg, xy, n / 2);
	memcpy(keep, vg->rb, sizeof keep);
	for (int k = 0; k < n; k += 2) {
		segment(vg, &xy[k], &xy[(k + 2) % n], RIM);
	}
	memcpy(vg->rb, keep, sizeof keep);
	vg->state.edge = vg->state.zone && vg->state.dark;
	run(vg);
	for (int k = 0; vg->state.edge && k < n; k += 2) {
		segment(vg, &xy[k], &xy[(k + 2) % n], HAIR);
	}
	vg->state.edge = 0;
}

// A disk, line or poly record, with room *xy for its numbers.
static void drawing(struct vg *vg, const char *line, double via, double **xy,
		    size_t *cap)
{
	char kind = line[0];
	char *s = NULL;
	double w = 0;
	int n = 0;

	w = kind == 'l' ? strtod(line + 4, &s) : 0;
	n = numbers(kind == 'l' ? s : line + 4, xy, cap);
	vg->state.via = kind == 'd' && n == 3 && (*xy)[2] <= via;
	run(vg);
	if (kind == 'd' && n == 3) {
		segment(vg, *xy, *xy, (*xy)[2]);
	} else if (kind == 'l') {
		for (int k = 2; k + 1 < n || k == 2; k += 2) {
			segment(vg, &(*xy)[k - 2], &(*xy)[k < n ? k : 0],
				w > 0 && !vg->hair ? w / 2 : HAIR);
		}
	} else if (n / 2 >= 3) {
		poly_record(vg, *xy, n);
	}
	element(vg, kind);
}

void vg_load_stream(struct vg *vg, struct vg_file *f, FILE *in, double via)
{
	char *line = NULL;
	size_t cap = 0;
	size_t capxy = 0;
	double *xy = NULL;

	start_file(vg, f);
	while (get_line(&line, &cap, in) > 0) {
		line[strcspn(line, "\n")] = 0;
		state_record(vg, line);
		if (starts(line, "disk ") || starts(line, "line ") ||
		    starts(line, "poly ")) {
			drawing(vg, line, via, &xy, &capxy);
		}
	}
	vg->state.part = -2; // No run is of this state: the open one ends.
	run(vg);
	upload(&f->buf, vg->pt, vg->npt);
	upload(&f->sbuf, vg->sv, vg->nsv * SEG_ENDS);
	upload(&f->hbuf, vg->hc, vg->nsv * SEG_HC);
	f->ntri = vg->npt / TRI_FLOATS;
	f->nseg = vg->nsv / SEG_CORNERS;
	if (f->nelem && vg->nfiles < VG_FILES) {
		vg->files[vg->nfiles++] = f; // It names parts.
	}
	free(line);
	free(xy);
}

void vg_layer(const char *path, char *name, size_t size)
{
	FILE *in = fopen(path, "r");
	char buf[LINE_LEN];

	*name = 0;
	while (in && fgets(buf, sizeof buf, in)) {
		if (starts(buf, "layer ")) {
			const char *s = buf + strlen("layer ");
			size_t n = strcspn(s, " \n");

			n = n < size ? n : size - 1;
			memcpy(name, s, n);
			name[n] = 0;
			break;
		}
		if (starts(buf, "disk ") || starts(buf, "line ") ||
		    starts(buf, "poly ") || starts(buf, "text ")) {
			break; // A drawing: the layer comes before.
		}
	}
	if (in) {
		(void)fclose(in);
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

static void shader(struct vg *vg, GLenum kind, const char *src)
{
	GLuint sh = glCreateShader(kind);
	GLint ok = 0;

	glShaderSource(sh, 1, &src, NULL);
	glCompileShader(sh);
	glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
	if (!ok) {
		(void)fprintf(stderr, "a shader does not compile\n");
		exit(2);
	}
	glAttachShader(vg->prog, sh);
}

void vg_draw(struct vg *vg, const struct vg_file *f,
	     int (*want)(struct vg *vg, const struct vg_run *),
	     void (*each)(struct vg *vg, const struct vg_run *))
{
	for (size_t i = 0; i < f->nrun; i++) {
		const struct vg_run *r = &f->run[i];

		if (!want(vg, r)) {
			continue;
		}
		each(vg, r);
		if (r->count) {
			glBindBuffer(GL_ARRAY_BUFFER, f->buf);
			glVertexPointer(2, GL_FLOAT, 0, NULL);
			glDrawArrays(GL_TRIANGLES, r->first, r->count);
		}
		if (r->scount) {
			glUseProgram(vg->prog);
			glUniform2f(vg->u_view, (float)vg->w, (float)vg->h);
			glUniform1f(vg->u_zoom, (float)vg->zoom);
			glUniform1f(vg->u_solid, (float)vg->solid);
			glUniform1f(vg->u_brighter, (float)vg->brighter);
			if (vg->brighter) {
				glBlendEquation(GL_MAX);
			}
			glEnableVertexAttribArray((GLuint)vg->a_seg);
			glEnableVertexAttribArray((GLuint)vg->a_hc);
			glBindBuffer(GL_ARRAY_BUFFER, f->sbuf);
			glVertexAttribPointer((GLuint)vg->a_seg, SEG_ENDS,
					      GL_FLOAT, GL_FALSE, 0, NULL);
			glBindBuffer(GL_ARRAY_BUFFER, f->hbuf);
			glVertexAttribPointer((GLuint)vg->a_hc, SEG_HC,
					      GL_FLOAT, GL_FALSE, 0, NULL);
			glDrawArrays(GL_TRIANGLES, r->sfirst, r->scount);
			glBlendEquation(GL_FUNC_ADD);
			glDisableVertexAttribArray((GLuint)vg->a_seg);
			glDisableVertexAttribArray((GLuint)vg->a_hc);
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
				   NEAR * (1 + sqrt((dx * dx) + (dy * dy))) :
				   (b[2] - b[0]) * (b[3] - b[1]);

		if (x >= b[0] - tol && x <= b[2] + tol && y >= b[1] - tol &&
		    y <= b[3] + tol && a < *area) {
			*area = a;
			*best = f->elem[i].part;
		}
	}
}

void vg_part_box(struct vg *vg, int part, double b[4])
{
	b[0] = b[1] = FAR;
	b[2] = b[3] = -FAR;
	for (size_t i = 0; part >= 0 && i < vg->nfiles; i++) {
		for (size_t k = 0; k < vg->files[i]->nelem; k++) {
			for (int j = 0;
			     vg->files[i]->elem[k].part == part && j < 4; j++) {
				double e = vg->files[i]->elem[k].b[j];

				b[j] = j < 2 ? fmin(b[j], e) : fmax(b[j], e);
			}
		}
	}
}

void vg_init(struct vg *vg)
{
	vg->prog = glCreateProgram();
	shader(vg, GL_VERTEX_SHADER, vs);
	shader(vg, GL_FRAGMENT_SHADER, fs);
	glLinkProgram(vg->prog);
	vg->u_view = glGetUniformLocation(vg->prog, "view");
	vg->u_zoom = glGetUniformLocation(vg->prog, "zoom");
	vg->u_solid = glGetUniformLocation(vg->prog, "solid");
	vg->u_brighter = glGetUniformLocation(vg->prog, "brighter");

	vg->a_seg = glGetAttribLocation(vg->prog, "seg");
	vg->a_hc = glGetAttribLocation(vg->prog, "hc");
}
