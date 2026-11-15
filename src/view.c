// SPDX-License-Identifier: GPL-3.0-only
// view.c -- Show the meshes and pictures of a scene in a window
// Copyright (c) 2026 Jakob Kastelic

// Usage: view [-c | -b] SCENE
//
// Shows the parts of the scene file (scene.h) in an X11 window by
// OpenGL, and reads commands (scene.h) from stdin.  The left button
// turns the view, the middle or right one moves it, and the wheel zooms
// round the pointer.  A click on a part picks it: prints its name on
// stdout and shows it in the corner.  Space fits all in the view, and
// the keys i, f, b, l, r, t and u (iso, front, back, left, right, top,
// underside) set it; q quits (Escape does not).  The keys h, o and s
// act on the part under the pointer: h hides it, o shows it alone and s
// turns it see-through or back; a shows all, solid, and brings every
// part home (as the command reset).  The key plus explodes the parts out
// from the centre, a step a press, and minus brings them back, home at
// the last; the left button with shift drags the item of a part out
// along its line (explode.c).  Ctrl-Z undoes what shows and the moves,
// a key or a drag a step, and Ctrl-Y redoes them.  The key ? shows or
// hides a help of the keys and the buttons, and nothing else hides it.
// A mesh or picture file that changes reads again, and so does the scene
// file.
//
// With -c, view reads the scene and its files and prints each part and
// its triangles, with no window.  With -b, it runs the commands of
// stdin with no window, and draws its shots without OpenGL.

#include "geom.h"
#include "obj.h"
#include "scene.h"
#include "util.h"
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glx.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/inotify.h>
#include <unistd.h>

#define TURN_PER_PIXEL 0.4 // Degrees of turn for a pixel of drag.
#define WHEEL_ZOOM 0.85 // The zoom of a step of the wheel.
#define FULL_TURN 360.0
#define HELP_SHADE 0.6F // The opacity of the ground of the help.
#define HIGHLIGHT 0.35F // The share of the light colour on a picked part.
#define CLICK_SLOP 3.0 // The most drag of a click, in pixels.

enum {
	LINE_MAX = 8192, // The longest command.
	EVENTS_LEN = 16384, // Room for the events of inotify.
	LABEL_LEN = 512, // Room for the name in the corner.
	FONT_FIRST = 32, // The first and the number of glyphs of the font.
	FONT_COUNT = 96,
	LABEL_AT = 8, // The place of the name in the corner, in pixels.
	BUTTON_WHEEL_UP = 4,
	BUTTON_WHEEL_DOWN = 5,
	KEY_LEN = 8,
	DEPTH_BITS = 24,
	MATRIX = 16 // The numbers of a matrix of OpenGL.
};

enum { TURN, PAN, DRAG };

#define EXPLODE_STEP 0.25 // The step of the explode of a + or a -.

// A watched file: the directory watch and the base name, and the item,
// or NONE for the scene file.
struct watch {
	int wd;
	char *base;
	size_t item;
};

// The window and its state: the drag (its button, where it began and
// where it is), the picked name, stdin and the watches.
struct gui {
	Display *d;
	Window w;
	Atom close;
	GLXContext ctx;
	GLuint font;
	int font_w, font_h; // The size of a letter of the font.
	int help; // Whether the help shows.
	struct scene s;
	int button, moved;
	int mode; // What a drag does: TURN, PAN or DRAG.
	size_t picked, item; // The picked part, and the item of a drag.
	unsigned draw, quit;
	struct px press, last;
	char label[LABEL_LEN];
	int in_open;
	char in[LINE_MAX];
	size_t nin;
	int ino;
	struct watch *wt;
	size_t nwt, capwt;
};

// Watches the directory of path for item.
static void watch_file(struct gui *g, const char *path, size_t item)
{
	const char *slash = strrchr(path, '/');
	char *dir = xstrdup(slash ? path : ".");
	struct watch w;

	if (slash) {
		dir[slash - path + (slash == path)] = 0;
	}
	w.wd = inotify_add_watch(
		g->ino, dir, (uint32_t)IN_CLOSE_WRITE | (uint32_t)IN_MOVED_TO);
	w.base = xstrdup(slash ? slash + 1 : path);
	w.item = item;
	free(dir);
	if (w.wd < 0) {
		free(w.base);
		return;
	}
	PUSH(g->wt, g->nwt, g->capwt, w);
}

static void watch_all(struct gui *g)
{
	for (size_t i = 0; i < g->nwt; i++) {
		free(g->wt[i].base);
	}
	g->nwt = 0;
	if (g->ino < 0) {
		return;
	}
	watch_file(g, g->s.path, NONE);
	for (size_t i = 0; i < g->s.nit; i++) {
		watch_file(g, g->s.it[i].path, i);
	}
}

// Frees the buffers and textures of item i, or of all if NONE.
static void free_gl(struct gui *g, size_t item)
{
	for (size_t i = 0; i < g->s.np; i++) {
		struct part *p = &g->s.p[i];

		if (!p->gl[0] || (item != NONE && p->item != item)) {
			continue;
		}
		if (p->uv) {
			glDeleteTextures(1, p->gl);
		} else {
			glDeleteBuffers(2, p->gl);
		}
		p->gl[0] = p->gl[1] = 0;
	}
}

// Makes the buffer or texture of p.
static void upload(struct part *p)
{
	size_t n = TRI_FLOATS * p->nt * sizeof(float);

	if (p->uv) {
		glGenTextures(1, p->gl);
		glBindTexture(GL_TEXTURE_2D, p->gl[0]);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
				GL_LINEAR);
		// Near, the pixels show square, as picks and the shots of -b
		// see them.
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
				GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, p->pw, p->ph, 0,
			     GL_RGBA, GL_UNSIGNED_BYTE, p->pix);
		return;
	}
	glGenBuffers(2, p->gl);
	glBindBuffer(GL_ARRAY_BUFFER, p->gl[0]);
	glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)n, p->pos, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, p->gl[1]);
	glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)n, p->nrm, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// Draws the triangles of p from its buffers of points and normals.
static void draw_mesh(const struct part *p)
{
	glBindBuffer(GL_ARRAY_BUFFER, p->gl[0]);
	glVertexPointer(3, GL_FLOAT, 0, NULL);
	glBindBuffer(GL_ARRAY_BUFFER, p->gl[1]);
	glNormalPointer(GL_FLOAT, 0, NULL);
	glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(3 * p->nt));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

static void draw_picture(const struct part *p)
{
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, p->gl[0]);
	glColor3f(1, 1, 1);
	glPushMatrix();
	glTranslated(p->off.x, p->off.y, p->off.z);
	glBegin(GL_TRIANGLES);
	for (size_t k = 0; k < PIC_CORNERS; k++) {
		glTexCoord2fv(&p->uv[2 * k]);
		glVertex3fv(&p->pos[3 * k]);
	}
	glEnd();
	glPopMatrix();
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
}

// Draws the pictures and the see-through parts over the solid parts by
// their opacity, the farthest first, and keeps the depths of the solid
// parts.
// Draws the mesh of part p where it stands, in its colour, lighter if it
// is the picked one, and by SEE_ALPHA if see-through.
static void draw_part(const struct gui *g, struct part *p)
{
	GLfloat rgb[4];

	if (!p->gl[0]) {
		upload(p);
	}
	for (int k = 0; k < 3; k++) {
		rgb[k] = p->rgb[k];
		if (g->label[0] && !strcmp(p->name, g->label)) {
			rgb[k] += HIGHLIGHT * (1 - rgb[k]);
		}
	}
	rgb[3] = p->see ? SEE_ALPHA : 1;
	glColor4fv(rgb);
	glPushMatrix();
	glTranslated(p->off.x, p->off.y, p->off.z);
	draw_mesh(p);
	glPopMatrix();
}

static void draw_pictures(struct gui *g)
{
	size_t *order = NULL;
	size_t n = scene_pictures(&g->s, &order);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(GL_FALSE);
	for (size_t i = 0; i < n; i++) {
		struct part *p = &g->s.p[order[i]];

		if (!p->uv) {
			draw_part(g, p);
			continue;
		}
		if (!p->gl[0]) {
			upload(p);
		}
		draw_picture(p);
	}
	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
	free(order);
}

// Draws text at pixel x, y from the lower left of the window.
static void text_at(const struct gui *g, int x, int y, const char *text)
{
	glRasterPos2i(x, y);
	glListBase(g->font - FONT_FIRST);
	glCallLists((GLsizei)strlen(text), GL_UNSIGNED_BYTE, text);
}

// Darkens the ground of the help, so that it reads over the parts.
static void help_box(const struct gui *g)
{
	size_t n = 0;
	size_t w = 0;
	int h = g->s.cam.sz.h;

	for (; view_help[n]; n++) {
		w = strlen(view_help[n]) > w ? strlen(view_help[n]) : w;
	}
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0, 0, 0, HELP_SHADE);
	glRecti(0, h, (2 * LABEL_AT) + ((int)w * g->font_w),
		h - (2 * LABEL_AT) - ((int)n * g->font_h));
	glDisable(GL_BLEND);
}

// Draws the name of the picked part in the lower left corner, and the
// help (view_help) from the upper left down if it shows.
static void draw_label(const struct gui *g)
{
	const struct cam *c = &g->s.cam;

	if ((!g->label[0] && !g->help) || !g->font) {
		return;
	}
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, c->sz.w, 0, c->sz.h, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (g->help) {
		help_box(g);
	}
	glColor3f(1, 1, 1);
	text_at(g, LABEL_AT, LABEL_AT, g->label);
	for (size_t k = 0; g->help && view_help[k]; k++) {
		text_at(g, LABEL_AT,
			c->sz.h - LABEL_AT - ((int)(k + 1) * g->font_h),
			view_help[k]);
	}
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
}

// Sets the matrices of the camera.
static void camera(const struct scene *s)
{
	const struct cam *c = &s->cam;
	double a = (double)c->sz.w / (double)c->sz.h;
	double d = cam_depth(c, s);
	static const GLfloat light[4] = { (GLfloat)LIGHT_X, (GLfloat)LIGHT_Y,
					  (GLfloat)LIGHT_Z, 0 };
	GLdouble m[MATRIX];

	glViewport(0, 0, c->sz.w, c->sz.h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-c->half * a, c->half * a, -c->half, c->half, -d, d);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLightfv(GL_LIGHT0, GL_POSITION, light);
	memset(m, 0, sizeof(m));
	for (int k = 0; k < 3; k++) {
		m[(4 * 0) + k] = comp(c->r[k], 0);
		m[(4 * 1) + k] = comp(c->r[k], 1);
		m[(4 * 2) + k] = comp(c->r[k], 2);
		m[(4 * 3) + k] = -dot(c->r[k], c->c);
	}
	m[MATRIX - 1] = 1;
	glMultMatrixd(m);
}

static void draw(struct gui *g)
{
	glClearColor(g->s.back[0], g->s.back[1], g->s.back[2], 1);
	glClear((GLbitfield)GL_COLOR_BUFFER_BIT |
		(GLbitfield)GL_DEPTH_BUFFER_BIT);
	camera(&g->s);
	for (size_t i = 0; i < g->s.np; i++) {
		struct part *p = &g->s.p[i];

		if (!p->shown || !p->nt || p->uv || p->see) {
			continue;
		}
		draw_part(g, p);
	}
	draw_pictures(g);
	draw_label(g);
}

static void set_lights(void)
{
	static const GLfloat diffuse[4] = { 1 - AMBIENT, 1 - AMBIENT,
					    1 - AMBIENT, 1 };
	static const GLfloat black[4] = { 0, 0, 0, 1 };
	static const GLfloat ambient[4] = { AMBIENT, AMBIENT, AMBIENT, 1 };

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_AMBIENT, black);
	glLightfv(GL_LIGHT0, GL_SPECULAR, black);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
}

// Writes the picture of the window to path.
static void gl_shot(struct gui *g, const char *path)
{
	struct size sz = g->s.cam.sz;
	size_t row = 3 * (size_t)sz.w;
	unsigned char *a = xmalloc(row * (size_t)sz.h);
	unsigned char *b = xmalloc(row * (size_t)sz.h);

	draw(g);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, sz.w, sz.h, GL_RGB, GL_UNSIGNED_BYTE, a);
	for (size_t y = 0; y < (size_t)sz.h; y++) {
		memcpy(&b[y * row], &a[((size_t)sz.h - 1 - y) * row], row);
	}
	if (write_ppm(path, b, sz) < 0) {
		warn("view: cannot write %s", path);
	}
	free(a);
	free(b);
}

static void spin(struct gui *g, long frames)
{
	for (long k = 0; k < frames; k++) {
		cam_turn(&g->s.cam,
			 (struct px){ FULL_TURN / (double)frames, 0 });
		draw(g);
		glXSwapBuffers(g->d, g->w);
	}
	glFinish();
}

static void command(struct gui *g, char *line)
{
	struct cmd c = scene_command(&g->s, line);

	if (c.ask & CMD_SIZE) {
		XResizeWindow(g->d, g->w, (unsigned)g->s.cam.sz.w,
			      (unsigned)g->s.cam.sz.h);
	}
	if (c.ask & CMD_SHOT) {
		gl_shot(g, c.file);
	}
	if (c.ask & CMD_SPIN) {
		spin(g, c.frames);
	}
	g->draw |= c.ask & (CMD_DRAW | CMD_SHOT);
	g->quit |= c.ask & CMD_QUIT;
}

// Runs the lines of stdin that have come.
static void read_stdin(struct gui *g)
{
	ssize_t k = read(0, g->in + g->nin, sizeof(g->in) - 1 - g->nin);
	char *nl = NULL;

	if (k <= 0) {
		if (k == 0 || errno != EINTR) {
			g->in_open = 0;
		}
		return;
	}
	g->nin += (size_t)k;
	g->in[g->nin] = 0;
	while ((nl = strchr(g->in, '\n')) != NULL) {
		size_t n = (size_t)(nl - g->in) + 1;

		*nl = 0;
		command(g, g->in);
		memmove(g->in, g->in + n, g->nin - n + 1);
		g->nin -= n;
	}
	if (g->nin == sizeof(g->in) - 1) {
		warn("view: a command too long");
		g->nin = 0;
	}
}

// Forgets the pick and the drag, whose parts may have gone.
static void unpick(struct gui *g)
{
	g->picked = g->item = NONE;
	g->label[0] = 0;
	g->mode = TURN;
	g->draw = 1;
}

// Reads the scene file again, keeping the camera.
static void reload_scene(struct gui *g)
{
	struct scene s;
	struct fault e;

	if (scene_read(&s, g->s.path, &e) < 0) {
		warn("view: %s", e.msg);
		return;
	}
	free_gl(g, NONE);
	s.cam = g->s.cam;
	scene_free(&g->s);
	g->s = s;
	watch_all(g);
	unpick(g);
}

static void reload_item(struct gui *g, size_t item)
{
	struct fault e;
	const struct part *p = g->s.p;
	size_t n = g->s.np;

	for (size_t i = 0; i < n; i++) {
		if (p[i].item == item && p[i].gl[0]) {
			free_gl(g, item);
			break;
		}
	}
	if (scene_reload(&g->s, item, &e) < 0) {
		warn("view: %s", e.msg);
	}
	unpick(g);
}

static void read_watch(struct gui *g)
{
	char buf[EVENTS_LEN];
	ssize_t k = read(g->ino, buf, sizeof(buf));
	size_t at = 0;

	while (k > 0 && at + sizeof(struct inotify_event) <= (size_t)k) {
		struct inotify_event ev;
		const char *name = buf + at + sizeof(ev);

		memcpy(&ev, buf + at, sizeof(ev));
		at += sizeof(ev) + ev.len;
		for (size_t i = 0; ev.len && i < g->nwt; i++) {
			const struct watch *w = &g->wt[i];

			if (w->wd != ev.wd || strcmp(w->base, name) != 0) {
				continue;
			}
			if (w->item == NONE) {
				reload_scene(g);
				return;
			}
			reload_item(g, w->item);
		}
	}
}

static void pick(struct gui *g, struct px at)
{
	size_t i = scene_pick(&g->s, at);
	const char *name = i == NONE ? "-" : g->s.p[i].name;

	printf("%s\n", name);
	(void)fflush(stdout);
	xsnprintf(g->label, "%s", i == NONE ? "" : name);
	g->picked = i;
	g->draw = 1;
}

// Runs the key h, o, s or a in buf, pressed with the pointer at at, on
// what shows and, for a, on the moves; returns 0, or -1 if it is none of
// those.
static int show_key(struct gui *g, const char *buf, struct px at)
{
	struct scene *s = &g->s;

	if (!strcmp(buf, "a")) {
		scene_reset(s);
		scene_commit(s);
	} else if (buf[0] && !buf[1] && strchr("hos", buf[0])) {
		(void)scene_show_at(s, buf[0], at);
	} else {
		return -1;
	}
	return 0;
}

// Runs the key of the picked part, the explode or the history in buf,
// with control if ctrl; returns 0, or -1 if it is none of those.
static int arrange_key(struct gui *g, const char *buf, KeySym ks, int ctrl,
		       struct px at)
{
	struct scene *s = &g->s;

	if (ctrl && (ks == XK_z || ks == XK_y)) {
		(void)(ks == XK_z ? scene_undo(s) : scene_redo(s));
		return 0;
	}
	if (show_key(g, buf, at) == 0) {
		return 0;
	}
	if (!strcmp(buf, "+") || !strcmp(buf, "=")) {
		scene_explode(s, s->f + EXPLODE_STEP);
	} else if (!strcmp(buf, "-") && s->f > 0) {
		scene_explode(s, fmax(s->f - EXPLODE_STEP, 0));
	} else {
		return -1;
	}
	scene_commit(s);
	return 0;
}

static void key(struct gui *g, XKeyEvent *e)
{
	static const char *const view_keys[][2] = {
		{ "i", "iso" },	   { "f", "front" }, { "b", "back" },
		{ "l", "left" },   { "r", "right" }, { "t", "top" },
		{ "u", "bottom" },
	};
	char buf[KEY_LEN] = { 0 };
	KeySym ks = 0;
	int fit = 0;

	(void)XLookupString(e, buf, (int)sizeof(buf) - 1, &ks, NULL);
	if (!strcmp(buf, "?")) {
		g->help = !g->help;
		g->draw = 1;
		return;
	}
	if (!strcmp(buf, "q")) {
		g->quit = 1;
		return;
	}
	if (arrange_key(g, buf, ks, (e->state & (unsigned)ControlMask) != 0,
			(struct px){ e->x, e->y }) == 0) {
		g->draw = 1;
		return;
	}
	fit = !strcmp(buf, " ");
	for (size_t k = 0; k < sizeof(view_keys) / sizeof(view_keys[0]); k++) {
		if (!strcmp(buf, view_keys[k][0])) {
			cam_view(&g->s.cam, view_keys[k][1]);
			fit = 1;
		}
	}
	if (fit) {
		cam_fit(&g->s.cam, &g->s);
		g->draw = 1;
	}
}

static void button(struct gui *g, const XButtonEvent *e)
{
	struct px at = { e->x, e->y };

	if (e->type == ButtonRelease) {
		if (e->button == (unsigned)g->button && !g->moved &&
		    g->button == Button1 && g->mode == TURN) {
			pick(g, at);
		}
		if (g->mode == DRAG && g->moved) {
			scene_commit(&g->s);
		}
		g->button = 0;
		return;
	}
	if (e->button == BUTTON_WHEEL_UP || e->button == BUTTON_WHEEL_DOWN) {
		cam_zoom(&g->s.cam,
			 e->button == BUTTON_WHEEL_UP ? WHEEL_ZOOM :
							1 / WHEEL_ZOOM,
			 at);
		g->draw = 1;
		return;
	}
	g->button = (int)e->button;
	g->mode = e->button == Button1 ? TURN : PAN;
	g->press = g->last = at;
	g->moved = 0;
	if (e->button == Button1 && (e->state & (unsigned)ShiftMask)) {
		pick(g, at);
		g->mode = DRAG;
		g->item = g->picked == NONE ? NONE : g->s.p[g->picked].item;
	}
}

static void motion(struct gui *g, const XMotionEvent *e)
{
	XEvent next;
	struct px at = { e->x, e->y };
	struct px d;

	while (XCheckTypedWindowEvent(g->d, g->w, MotionNotify, &next)) {
		at = (struct px){ next.xmotion.x, next.xmotion.y };
	}
	if (!g->button) {
		return;
	}
	d = (struct px){ at.x - g->last.x, at.y - g->last.y };
	g->last = at;
	if (fabs(at.x - g->press.x) > CLICK_SLOP ||
	    fabs(at.y - g->press.y) > CLICK_SLOP) {
		g->moved = 1;
	}
	if (!g->moved) {
		return;
	}
	if (g->mode == DRAG && g->item < g->s.nit) {
		double k = dot(cam_shift(&g->s.cam, d),
			       scene_line_out(&g->s, g->item));

		scene_drag(&g->s, g->item, g->s.drag[g->item] + k);
	} else if (g->mode == PAN) {
		cam_pan(&g->s.cam, d);
	} else if (g->mode == TURN) {
		cam_turn(&g->s.cam, (struct px){ d.x * TURN_PER_PIXEL,
						 d.y * TURN_PER_PIXEL });
	}
	g->draw = 1;
}

static void event(struct gui *g, XEvent *e)
{
	switch (e->type) {
	case Expose:
		g->draw = 1;
		break;
	case ConfigureNotify:
		g->s.cam.sz = (struct size){ e->xconfigure.width,
					     e->xconfigure.height };
		g->draw = 1;
		break;
	case KeyPress:
		key(g, &e->xkey);
		break;
	case ButtonPress:
	case ButtonRelease:
		button(g, &e->xbutton);
		break;
	case MotionNotify:
		motion(g, &e->xmotion);
		break;
	case ClientMessage:
		g->quit |= (unsigned)((Atom)e->xclient.data.l[0] == g->close);
		break;
	default:
		break;
	}
}

static void open_window(struct gui *g, const char *path)
{
	int attr[] = { GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, DEPTH_BITS,
		       None };
	XVisualInfo *vi = NULL;
	XSetWindowAttributes wa;
	XFontStruct *font = NULL;
	Window root = 0;
	XEvent e;
	char title[LABEL_LEN];
	char *base = base_name(path);

	g->d = XOpenDisplay(NULL);
	if (!g->d) {
		die("view: cannot open the display");
	}
	root = RootWindow(g->d, DefaultScreen(g->d));
	vi = glXChooseVisual(g->d, DefaultScreen(g->d), attr);
	if (!vi) {
		die("view: no OpenGL visual");
	}
	memset(&wa, 0, sizeof(wa));
	wa.colormap = XCreateColormap(g->d, root, vi->visual, AllocNone);
	wa.event_mask = (long)((unsigned long)ExposureMask |
			       (unsigned long)StructureNotifyMask |
			       (unsigned long)KeyPressMask |
			       (unsigned long)ButtonPressMask |
			       (unsigned long)ButtonReleaseMask |
			       (unsigned long)PointerMotionMask);
	g->w = XCreateWindow(
		g->d, root, 0, 0, (unsigned)g->s.cam.sz.w,
		(unsigned)g->s.cam.sz.h, 0, vi->depth, InputOutput, vi->visual,
		(unsigned long)CWColormap | (unsigned long)CWEventMask, &wa);
	xsnprintf(title, "view %s", base);
	free(base);
	XStoreName(g->d, g->w, title);
	g->close = XInternAtom(g->d, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(g->d, g->w, &g->close, 1);
	g->ctx = glXCreateContext(g->d, vi, NULL, True);
	XFree(vi);
	if (!g->ctx || !glXMakeCurrent(g->d, g->w, g->ctx)) {
		die("view: no OpenGL context");
	}
	font = XLoadQueryFont(g->d, "fixed");
	if (font) {
		g->font = glGenLists(FONT_COUNT);
		g->font_h = font->ascent + font->descent;
		g->font_w = font->max_bounds.width;
		glXUseXFont(font->fid, FONT_FIRST, FONT_COUNT, (int)g->font);
		XFreeFont(g->d, font);
	}
	set_lights();
	XMapWindow(g->d, g->w);
	do {
		XNextEvent(g->d, &e);
		event(g, &e);
	} while (e.type != Expose);
}

static void loop(struct gui *g)
{
	while (!g->quit) {
		struct pollfd fd[3] = { { ConnectionNumber(g->d), POLLIN, 0 },
					{ g->ino, POLLIN, 0 },
					{ g->in_open ? 0 : -1, POLLIN, 0 } };

		while (XPending(g->d) && !g->quit) {
			XEvent e;

			XNextEvent(g->d, &e);
			event(g, &e);
		}
		if (g->draw && !g->quit) {
			g->draw = 0;
			draw(g);
			glXSwapBuffers(g->d, g->w);
			continue;
		}
		if (g->quit || poll(fd, 3, -1) < 0) {
			continue;
		}
		if ((unsigned)fd[1].revents & (unsigned)POLLIN) {
			read_watch(g);
		}
		if ((unsigned)fd[2].revents &
		    ((unsigned)POLLIN | (unsigned)POLLHUP)) {
			read_stdin(g);
		} else if (fd[2].revents) {
			g->in_open = 0; // Closed, or no file at all.
		}
	}
}

// Runs the command of line, without its line end.
static struct cmd run_line(struct scene *s, char *line)
{
	line[strcspn(line, "\r\n")] = 0;
	return scene_command(s, line);
}

// Runs the commands of stdin with no window.
static void batch(struct scene *s)
{
	char line[LINE_MAX];

	while (fgets(line, sizeof(line), stdin)) {
		struct cmd c = run_line(s, line);
		size_t n = (size_t)s->cam.sz.w * (size_t)s->cam.sz.h * 3;
		unsigned char *rgb = NULL;
		struct px step = { FULL_TURN / (double)(c.frames + !c.frames),
				   0 };

		if (c.ask & CMD_QUIT) {
			break;
		}
		if (!(c.ask & (CMD_SHOT | CMD_SPIN))) {
			continue;
		}
		rgb = xmalloc(n);
		for (long k = 0; k < c.frames; k++) {
			cam_turn(&s->cam, step);
			scene_render(s, rgb);
		}
		scene_render(s, rgb);
		if ((c.ask & CMD_SHOT) && write_ppm(c.file, rgb, s->cam.sz)) {
			warn("view: cannot write %s", c.file);
		}
		free(rgb);
	}
}

int main(int argc, char **argv)
{
	static struct gui g;
	struct fault e;
	int check = argc == 3 && !strcmp(argv[1], "-c");
	int nowin = argc == 3 && !strcmp(argv[1], "-b");

	if (argc != 2 + (check || nowin) || argv[argc - 1][0] == '-') {
		die("usage: view [-c | -b] SCENE");
	}
	if (scene_read(&g.s, argv[argc - 1], &e) < 0) {
		die("view: %s", e.msg);
	}
	if (check) {
		for (size_t i = 0; i < g.s.np; i++) {
			printf("%s %zu\n", g.s.p[i].name, g.s.p[i].nt);
		}
	} else if (nowin) {
		batch(&g.s);
	} else {
		g.in_open = 1;
		g.picked = g.item = NONE;
		g.ino = inotify_init1(
			(int)((unsigned)IN_NONBLOCK | (unsigned)IN_CLOEXEC));
		watch_all(&g);
		open_window(&g, argv[argc - 1]);
		loop(&g);
		free_gl(&g, NONE);
		glXMakeCurrent(g.d, None, NULL);
		glXDestroyContext(g.d, g.ctx);
		XCloseDisplay(g.d);
		for (size_t i = 0; i < g.nwt; i++) {
			free(g.wt[i].base);
		}
		free(g.wt);
	}
	scene_free(&g.s);
	close_stdout();
	return 0;
}
