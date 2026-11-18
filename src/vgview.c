// SPDX-License-Identifier: GPL-3.0-only
// vgview.c -- The viewer round vectors on the GPU
// Copyright (c) 2026 Jakob Kastelic

// See vecgl.h: the window, the view, the mouse, the common keys and
// commands, the selection and the stdin of cross-probing, which
// schview and pcbview share.

#define GL_GLEXT_PROTOTYPES

#include "vecgl.h"
#include "stroke.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glx.h>
#include <GL/glxext.h>
#include <X11/X.h>
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xinerama.h>
#include <X11/keysym.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <unistd.h>

#define VG_FILL 0.95 // The share of the window fit gives the box.
// The sRGB curve: the knee, the slope below it, the offset and scale
// and the power above it.
#define SRGB_KNEE 0.04045
#define SRGB_SLOPE 12.92
#define SRGB_OFF 0.055
#define SRGB_SCALE 1.055
#define SRGB_POWER 2.4
#define HELP_SHADE 0.6F // The opacity of the ground of the help.
#define HELP_LEAD (VG_CAP * 5 / 3) // A capital and two thirds apart.
#define FIT_SLACK 1e-9 // What fit adds to a box of no size.
#define WHEEL_ZOOM 1.25 // The zoom of a step of the wheel.
#define SAME_PLACE 1e-9 // A view this near is where it was,
#define SAME_ZOOM 1e-12 // and a zoom this near as it was.
#define PICK_PIXELS 3.0 // How near a click picks, in pixels.
enum {
	VG_WIDTH = 1000, // The first size of the window.
	VG_HEIGHT = 700,
	PAD_X = 8, // The margin of text in a box, left of it,
	PAD_Y = 6, // and above its capitals.
	CLASS_LEN = 64, // Room for the name and class of the window.
	COLOR_BITS = 8, // The bits of a color, of depth and of stencil.
	DEPTH_BITS = 24,
	STENCIL_BITS = 8,
	ATTR_SRGB = 17, // The places in the attributes of the visual of
	ATTR_SAMPLE_BUFFERS = 19, // sRGB, the sample buffers and the samples.
	ATTR_SAMPLES = 21,
	PROP_LONGS = 32, // The format of a property of longs.
	BUTTON_WHEEL_UP = 4,
	BUTTON_WHEEL_DOWN = 5,
	COMMANDS_LEN = 4096 // Room for the commands not yet read.
};

void vg_setup(struct vg *vg, void *user)
{
	memset(vg, 0, sizeof *vg);
	vg->zoom = 1;
	vg->fill = VG_FILL;
	vg->w = VG_WIDTH;
	vg->h = VG_HEIGHT;
	vg->sel = -1;
	vg->fresh = 1;
	vg->redraw = 1;
	vg->last_sel = -1;
	vg->user = user;
}

static double linear(double c)
{
	return c <= SRGB_KNEE ? c / SRGB_SLOPE :
				pow((c + SRGB_OFF) / SRGB_SCALE, SRGB_POWER);
}

void vg_color(struct vg *vg, const double c[3], double a)
{
	glColor4d(vg->srgb ? linear(c[0]) : c[0],
		  vg->srgb ? linear(c[1]) : c[1],
		  vg->srgb ? linear(c[2]) : c[2], a);
}

void vg_stroke(double x, double y, const char *s)
{
	double *seg = NULL;
	size_t n = stroke_text(s, VG_CAP, &seg);

	glLineWidth((float)(VG_CAP * VG_PEN));
	glBegin(GL_LINES);
	for (size_t i = 0; i < n; i++) {
		glVertex2d(x + seg[4 * i], y + seg[(4 * i) + 1]);
		glVertex2d(x + seg[(4 * i) + 2], y + seg[(4 * i) + 3]);
	}
	glEnd();
	glLineWidth(1);
	free(seg);
}

void vg_corner(struct vg *vg, const char *s)
{
	// Room for the descenders, a third of the capitals, and a margin.
	double w = stroke_width(s, VG_CAP) + (2 * PAD_X);
	double h = (VG_CAP * 4 / 3) + (2 * PAD_Y);

	glColor3d(0, 0, 0);
	glRectd(vg->w - w, vg->h, vg->w, vg->h - h);
	glColor3d(1, 1, 1);
	vg_stroke(vg->w - w + PAD_X, vg->h - PAD_Y - VG_CAP, s);
}

void vg_title(Display *dp, Window w, const char *path, int dir)
{
	char *t = NULL;
	const char *name = path;
	size_t n = strlen(name);

	if (dir) {
		const char *slash = strrchr(name, '/');

		n = slash ? (size_t)(slash - name) + 1 : 0;
	}
	t = malloc(n + 3);
	if (t) {
		if (dir && !n) {
			memcpy(t, "./", sizeof "./");
		} else {
			memcpy(t, name, n);
			t[n] = 0;
		}
		XStoreName(dp, w, t);
		XChangeProperty(dp, w, XInternAtom(dp, "_NET_WM_NAME", False),
				XInternAtom(dp, "UTF8_STRING", False),
				COLOR_BITS, PropModeReplace,
				(const unsigned char *)t, (int)strlen(t));
	}
	free(t);
}

// Writes the picture of the window to the shot asked for, as PPM, and
// asks for none more.
static void take_shot(struct vg *vg)
{
	FILE *f = fopen(vg->shot, "wb");
	size_t row = (size_t)vg->w * 3;
	unsigned char *px = malloc(row * (size_t)vg->h);
	int bad = !f || !px;

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	if (px) {
		glReadPixels(0, 0, vg->w, vg->h, GL_RGB, GL_UNSIGNED_BYTE, px);
	}
	if (f && px) {
		bad = fprintf(f, "P6\n%d %d\n255\n", vg->w, vg->h) < 0;
		for (int y = vg->h - 1; y >= 0; y--) {
			bad = bad ||
			      fwrite(px + ((size_t)y * row), 1, row, f) != row;
		}
	}
	if (f && fclose(f)) {
		bad = 1;
	}
	if (bad) {
		(void)fprintf(stderr, "%s: cannot write\n", vg->shot);
	}
	free(px);
	*vg->shot = 0;
}

// A frame: the viewer's drawing in linear light, multisampled, over its
// ground; its overlay and the help as they are; the picture asked for.
static void frame(struct vg *vg)
{
	// The help's lines, as the note of schview: a capital and two thirds
	// apart, in a box of margins 8 wide.
	double w = 0;
	double lh = HELP_LEAD;
	double top = vg->h - vg->top;
	size_t n = 0;

	if (vg->srgb) {
		glEnable(GL_FRAMEBUFFER_SRGB);
	}
	glViewport(0, 0, vg->w, vg->h);
	glClearColor((float)(vg->srgb ? linear(vg->viewer->bg[0]) :
					vg->viewer->bg[0]),
		     (float)(vg->srgb ? linear(vg->viewer->bg[1]) :
					vg->viewer->bg[1]),
		     (float)(vg->srgb ? linear(vg->viewer->bg[2]) :
					vg->viewer->bg[2]),
		     1);
	glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT |
		GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(vg->x - (vg->w / (2 * vg->zoom)),
		vg->x + (vg->w / (2 * vg->zoom)),
		vg->y - (vg->h / (2 * vg->zoom)),
		vg->y + (vg->h / (2 * vg->zoom)), -1, 1);
	vg->viewer->draw(vg);
	glDisable(GL_FRAMEBUFFER_SRGB);
	glLoadIdentity();
	glOrtho(0, vg->w, 0, vg->h, -1, 1);
	if (vg->viewer->overlay) {
		vg->viewer->overlay(vg);
	}
	if (vg->loading) {
		vg_corner(vg, "Loading...");
	}
	for (; vg->help && vg->viewer->help[n]; n++) {
		w = fmax(w, stroke_width(vg->viewer->help[n], VG_CAP));
	}
	glColor4f(0, 0, 0, HELP_SHADE);
	if (vg->help) {
		glRectd(0, top, w + (2 * PAD_X),
			top - ((double)n * lh) - (2 * PAD_Y));
	}
	glColor3f(1, 1, 1);
	for (size_t k = 0; k < n; k++) {
		vg_stroke(PAD_X, top - PAD_Y - VG_CAP - ((double)k * lh),
			  vg->viewer->help[k]);
	}
	if (*vg->shot) {
		take_shot(vg);
	}
	glXSwapBuffers(vg->dpy, vg->win);
}

int vg_start(struct vg *vg, int argc, char **argv, const struct vg_viewer *v)
{
	// Four samples and linear light, or none; then without linear light.
	// Each sample costs about a pixel more.
	int attr[] = { GLX_DRAWABLE_TYPE,
		       GLX_WINDOW_BIT,
		       GLX_RENDER_TYPE,
		       GLX_RGBA_BIT,
		       GLX_DOUBLEBUFFER,
		       True,
		       GLX_RED_SIZE,
		       COLOR_BITS,
		       GLX_GREEN_SIZE,
		       COLOR_BITS,
		       GLX_BLUE_SIZE,
		       COLOR_BITS,
		       GLX_DEPTH_SIZE,
		       DEPTH_BITS,
		       GLX_STENCIL_SIZE,
		       STENCIL_BITS,
		       GLX_FRAMEBUFFER_SRGB_CAPABLE_ARB,
		       True,
		       GLX_SAMPLE_BUFFERS,
		       1,
		       GLX_SAMPLES,
		       4,
		       None };
	char name[CLASS_LEN];
	char cls[CLASS_LEN];
	XClassHint hint = { name, cls };
	XSetWindowAttributes wa;
	XVisualInfo *vi = NULL;
	GLXFBConfig *fc = NULL;
	int n = 0;
	long pid = 0;

	vg->viewer = v;
	vg->dpy = XOpenDisplay(NULL);
	if (argc < 2 || argv[1][0] == '-' || !vg->dpy) {
		(void)fprintf(stderr,
			      vg->dpy ? "usage: %s FILE.vec...\n" :
					"%s: no display\n",
			      v->name);
		exit(2);
	}
	// Four samples a pixel, or none: more cost every frame and every
	// resize far more than they smooth.
	for (vg->srgb = 1; !fc && vg->srgb >= 0; vg->srgb--) {
		for (int s = 4; !fc && s >= 0; s -= 4) {
			attr[ATTR_SRGB] = vg->srgb;
			attr[ATTR_SAMPLE_BUFFERS] = !!s;
			attr[ATTR_SAMPLES] = s;
			fc = glXChooseFBConfig(vg->dpy, DefaultScreen(vg->dpy),
					       attr, &n);
			fc = n > 0 ? fc : NULL;
		}
	}
	vg->srgb++;
	vi = fc ? glXGetVisualFromFBConfig(vg->dpy, fc[0]) : NULL;
	if (!vi) {
		(void)fprintf(stderr, "%s: no OpenGL visual\n", v->name);
		exit(2);
	}
	memset(&wa, 0, sizeof wa);
	wa.colormap = XCreateColormap(vg->dpy, DefaultRootWindow(vg->dpy),
				      vi->visual, AllocNone);
	wa.event_mask = ExposureMask | StructureNotifyMask | KeyPressMask |
			ButtonPressMask | ButtonReleaseMask |
			PointerMotionMask | LeaveWindowMask;
	vg->win = XCreateWindow(vg->dpy, DefaultRootWindow(vg->dpy), 0, 0,
				(unsigned)vg->w, (unsigned)vg->h, 0, vi->depth,
				InputOutput, vi->visual,
				CWColormap | CWEventMask, &wa);
	(void)snprintf(name, sizeof name, "%s", v->name);
	(void)snprintf(cls, sizeof cls, "%c%s", v->name[0] - 'a' + 'A',
		       v->name + 1);
	XSetClassHint(vg->dpy, vg->win, &hint);
	vg_title(vg->dpy, vg->win, argv[1], v->dir_title);
	// The process, by which window placers know the window.
	pid = (long)getpid();
	XChangeProperty(vg->dpy, vg->win,
			XInternAtom(vg->dpy, "_NET_WM_PID", False), XA_CARDINAL,
			PROP_LONGS, PropModeReplace,
			(const unsigned char *)&pid, 1);
	glXMakeCurrent(vg->dpy, vg->win,
		       glXCreateNewContext(vg->dpy, fc[0], GLX_RGBA_TYPE, NULL,
					   True));
	XFree(vi);
	XFree((void *)fc);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnableClientState(GL_VERTEX_ARRAY);
	vg_init(vg);
	return 1;
}

static void fit(struct vg *vg)
{
	const double *b = vg->fit_box;

	vg->x = (b[0] + b[2]) / 2;
	vg->y = (b[1] + b[3]) / 2;
	vg->zoom = fmin(vg->w / (b[2] - b[0] + FIT_SLACK),
			vg->h / (b[3] - b[1] + FIT_SLACK)) *
		   vg->fill;
}

// A pixel of the window, from the middle.
struct pixel {
	int x, y;
};

// The pixel (px, py) of the window from its middle.
static struct pixel from_middle(const struct vg *vg, int px, int py)
{
	int cx = vg->w / 2;
	int cy = vg->h / 2;
	struct pixel p = { px - cx, py - cy };

	return p;
}

struct vg_point vg_point_at(const struct vg *vg, int px, int py)
{
	struct pixel p = from_middle(vg, px, py);
	struct vg_point w = { vg->x + (p.x / vg->zoom),
			      vg->y - (p.y / vg->zoom) };

	return w;
}

// Zooms n steps of the wheel about the pixel p from the middle.
static void wheel(struct vg *vg, struct pixel p, int n)
{
	double x = vg->x + (p.x / vg->zoom);
	double y = vg->y - (p.y / vg->zoom);

	vg->zoom *= pow(WHEEL_ZOOM, n);
	vg->x = x - p.x / vg->zoom;
	vg->y = y + p.y / vg->zoom;
	vg->fresh = 0;
}

static void drag(struct vg *vg, int dx, int dy)
{
	vg->x -= dx / vg->zoom;
	vg->y += dy / vg->zoom;
	vg->fresh = vg->fresh && !dx && !dy;
}

// The monitor of the root point (x, y), as left, top, right, bottom:
// the screen's, without Xinerama.
static void monitor(struct vg *vg, int x, int y, int m[4])
{
	XineramaScreenInfo *s = NULL;
	int n = 0;

	m[0] = m[1] = 0;
	m[2] = DisplayWidth(vg->dpy, DefaultScreen(vg->dpy));
	m[3] = DisplayHeight(vg->dpy, DefaultScreen(vg->dpy));
	if (XineramaIsActive(vg->dpy)) {
		s = XineramaQueryScreens(vg->dpy, &n);
	}
	for (int i = 0; i < n; i++) {
		if (x >= s[i].x_org && x < s[i].x_org + s[i].width &&
		    y >= s[i].y_org && y < s[i].y_org + s[i].height) {
			m[0] = s[i].x_org;
			m[1] = s[i].y_org;
			m[2] = s[i].x_org + s[i].width;
			m[3] = s[i].y_org + s[i].height;
		}
	}
	XFree(s);
}

// The pointer of a drag at an edge of monitor m goes round to the other
// edge; the last point (lx, ly) of the drag moves with it, and the
// motion queued before it is dropped, so the view goes on without a
// jump.
static void wrap(struct vg *vg, const XMotionEvent *e, const int m[4], int *lx,
		 int *ly)
{
	int x = e->x_root;
	int y = e->y_root;
	XEvent q;

	if (x <= m[0]) {
		x = m[2] - 2;
	} else if (x >= m[2] - 1) {
		x = m[0] + 1;
	}
	if (y <= m[1]) {
		y = m[3] - 2;
	} else if (y >= m[3] - 1) {
		y = m[1] + 1;
	}
	if (x == e->x_root && y == e->y_root) {
		return;
	}
	XWarpPointer(vg->dpy, None, DefaultRootWindow(vg->dpy), 0, 0, 0, 0, x,
		     y);
	XSync(vg->dpy, False);
	while (XCheckTypedWindowEvent(vg->dpy, vg->win, MotionNotify, &q)) {
	}
	*lx += x - e->x_root;
	*ly += y - e->y_root;
}

// A click: the viewer's own (its bar), or the part there, or clear.
static void click(struct vg *vg, int px, int py)
{
	if (!vg->viewer->click || !vg->viewer->click(vg, px, py)) {
		struct pixel p = from_middle(vg, px, py);

		vg->sel = vg->viewer->pick(vg, vg->x + (p.x / vg->zoom),
					   vg->y - (p.y / vg->zoom),
					   PICK_PIXELS / vg->zoom);
		if (vg->sel >= 0) {
			printf("select %s\n", vg_part_name(vg, vg->sel));
		}
	}
}

void vg_fit(struct vg *vg, const double box[4])
{
	memcpy(vg->fit_box, box, sizeof vg->fit_box);
	fit(vg);
}

// Centers the selected part; again, with nothing changed in between,
// zooms in two times about it.
static void center(struct vg *vg)
{
	double *last = vg->last;
	double b[4];

	vg_part_box(vg, vg->sel, b);
	if (b[2] < b[0]) {
		return;
	}
	if (fabs(vg->x - last[0]) + fabs(vg->y - last[1]) < SAME_PLACE &&
	    fabs(vg->zoom - last[2]) < SAME_ZOOM && vg->last_sel == vg->sel) {
		vg->zoom *= 2;
	} else {
		vg->x = (b[0] + b[2]) / 2;
		vg->y = (b[1] + b[3]) / 2;
	}
	last[0] = vg->x;
	last[1] = vg->y;
	last[2] = vg->zoom;
	vg->last_sel = vg->sel;
}

// A command of stdin: of the other viewer, or of a script.
static void command(struct vg *vg, const char *s)
{
	if (!strcmp(s, "select")) {
		vg->sel = -1;
	} else if (!strncmp(s, "select ", strlen("select "))) {
		vg->sel = vg_part(vg, s + strlen("select "));
	} else if (!strcmp(s, "center")) {
		center(vg);
	} else if (!strncmp(s, "shot ", strlen("shot "))) {
		// Drawn now, as all before it has left it; a path too long
		// for the shot is none.
		int n = snprintf(vg->shot, sizeof vg->shot, "%s",
				 s + strlen("shot "));

		if (n < 0 || (size_t)n >= sizeof vg->shot) {
			(void)fprintf(stderr, "%s: too long a path\n",
				      s + strlen("shot "));
			*vg->shot = 0;
		}
		frame(vg);
	} else if (!strcmp(s, "fit")) {
		fit(vg);
	} else if (!strcmp(s, "reload") && vg->viewer->reload) {
		vg->sel = -1;
		vg->viewer->reload(vg);
	} else if (vg->viewer->command) {
		vg->viewer->command(vg, s);
	}
}

// The key of a key press as typed, Shift and all; a digit of the keypad
// as a digit, Num Lock or not.
static KeySym key_of(XEvent *e)
{
	KeySym k = 0;
	KeySym kp = 0;
	char ch = 0;

	(void)XLookupString(&e->xkey, &ch, 1, &k, NULL);
	kp = XLookupKeysym(&e->xkey, 1);
	return kp >= XK_KP_1 && kp <= XK_KP_9 ? kp - XK_KP_1 + XK_1 : k;
}

// A mouse event: the wheel, a drag of the first button, a click, or a
// move; returns 0 for another event.  asked is whether a frame is asked
// by an earlier event of the batch.
static int mouse(struct vg *vg, XEvent *e, int asked)
{
	if (e->type == ButtonPress &&
	    (e->xbutton.button == BUTTON_WHEEL_UP ||
	     e->xbutton.button == BUTTON_WHEEL_DOWN)) {
		wheel(vg, from_middle(vg, e->xbutton.x, e->xbutton.y),
		      e->xbutton.button == BUTTON_WHEEL_UP ? 1 : -1);
	} else if (e->type == ButtonPress && e->xbutton.button == 1) {
		vg->down = 1;
		vg->moved = 0;
		vg->lx = e->xbutton.x;
		vg->ly = e->xbutton.y;
		monitor(vg, e->xbutton.x_root, e->xbutton.y_root, vg->mon);
	} else if (e->type == MotionNotify && vg->down) {
		drag(vg, e->xmotion.x - vg->lx, e->xmotion.y - vg->ly);
		vg->moved = vg->moved || e->xmotion.x != vg->lx ||
			    e->xmotion.y != vg->ly;
		vg->lx = e->xmotion.x;
		vg->ly = e->xmotion.y;
		wrap(vg, &e->xmotion, vg->mon, &vg->lx, &vg->ly);
	} else if (e->type == ButtonRelease && e->xbutton.button == 1) {
		vg->down = 0;
		if (!vg->moved) {
			click(vg, e->xbutton.x, e->xbutton.y);
		}
	} else if (e->type == MotionNotify || e->type == LeaveNotify) {
		int left = e->type == LeaveNotify;

		vg->redraw = (vg->viewer->move &&
			      vg->viewer->move(vg, left ? -1 : e->xmotion.x,
					       left ? -1 : e->xmotion.y)) ||
			     asked;
	} else {
		return 0;
	}
	return 1;
}

// A key of the viewer round the drawing, or else the viewer's own.
static void common_key(struct vg *vg, KeySym k)
{
	if (k == XK_q) {
		exit(0);
	} else if (k == XK_F5 && vg->viewer->reload) {
		// Told before the reading, which takes a while.
		vg->loading = 1;
		frame(vg);
		vg->loading = 0;
		vg->sel = -1;
		vg->viewer->reload(vg);
	} else if (k == XK_Escape) {
		vg->sel = -1;
		vg->help = 0;
		if (vg->viewer->key) {
			vg->viewer->key(vg, k);
		}
	} else if (k == XK_space) {
		fit(vg);
	} else if (k == XK_c) {
		center(vg);
	} else if (k == XK_C) {
		// About the middle of the window: c again, then C, gives back
		// the zoom.
		vg->zoom /= 2;
		vg->fresh = 0;
	} else if (k == XK_question) {
		vg->help = !vg->help;
	} else if (k && vg->viewer->key) {
		vg->viewer->key(vg, k);
	}
}

static void event(struct vg *vg, XEvent *e)
{
	KeySym k = e->type == KeyPress ? key_of(e) : 0;
	int asked =
		vg->redraw; // A frame asked by an earlier event of the batch.

	vg->redraw = 1;
	if (e->type == ConfigureNotify) {
		vg->w = e->xconfigure.width;
		vg->h = e->xconfigure.height;
		if (vg->fresh) {
			fit(vg);
		}
	} else if (!mouse(vg, e, asked)) {
		common_key(vg, k);
	}
}

void vg_loop(struct vg *vg, const double box[4])
{
	char buf[COMMANDS_LEN];
	size_t have = 0;
	int in = 0;

	memcpy(vg->fit_box, box, sizeof vg->fit_box);
	fit(vg);
	XMapWindow(vg->dpy, vg->win);
	if (setvbuf(stdout, NULL, _IOLBF, 0) != 0) {
		(void)fprintf(stderr, "%s: stdout not line by line\n",
			      vg->viewer->name);
	}
	for (;;) {
		struct pollfd fd[2] = { { ConnectionNumber(vg->dpy), POLLIN,
					  0 },
					{ in, POLLIN, 0 } };
		ssize_t r = 0;
		char *nl = NULL;

		while (XPending(vg->dpy)) {
			XEvent e;

			XNextEvent(vg->dpy, &e);
			event(vg, &e);
		}
		if (vg->redraw) {
			vg->redraw = 0;
			frame(vg);
			continue;
		}
		if (poll(fd, 2, -1) < 0) {
			exit(2);
		}
		// The input of the window first: a command (a picture) sees
		// all that came before it.
		if (((unsigned)fd[0].revents & (unsigned)POLLIN) ||
		    !((unsigned)fd[1].revents &
		      ((unsigned)POLLIN | (unsigned)POLLHUP))) {
			continue;
		}
		// The commands: whole lines, read as they come.
		r = read(in, buf + have, sizeof buf - 1 - have);
		if (r <= 0) {
			in = -1;
			continue;
		}
		have += (size_t)r;
		buf[have] = 0;
		while ((nl = strchr(buf, '\n')) != NULL) {
			*nl = 0;
			command(vg, buf);
			vg->redraw = 1;
			have -= (size_t)(nl + 1 - buf);
			memmove(buf, nl + 1, have + 1);
		}
		have = have == sizeof buf - 1 ? 0 : have; // Too long a line.
	}
}
