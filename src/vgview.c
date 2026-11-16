// SPDX-License-Identifier: GPL-3.0-only
// vgview.c -- The viewer round vectors on the GPU
// Copyright (c) 2026 Jakob Kastelic

// See vecgl.h: the window, the view, the mouse, the common keys and
// commands, the selection and the stdin of cross-probing, which
// schview and pcbview share.

#define _DEFAULT_SOURCE // For poll and read.
#define GL_GLEXT_PROTOTYPES

#include "vecgl.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glx.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

double vg_x, vg_y, vg_zoom = 1;
int vg_w = 1000, vg_h = 700, vg_sel = -1, vg_font_w, vg_font_h, vg_top;

static const struct vg_viewer *V;
static Display *d;
static Window win;
static int srgb, font_base, help, fresh = 1, draw = 1;
static char shot[4096];

static double linear(double c)
{
	return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4);
}

void vg_color(const double c[3], double a)
{
	glColor4d(srgb ? linear(c[0]) : c[0], srgb ? linear(c[1]) : c[1],
		  srgb ? linear(c[2]) : c[2], a);
}

void vg_text(int x, int y, const char *s)
{
	glRasterPos2i(x, y);
	glListBase((GLuint)font_base - 32);
	glCallLists((GLsizei)strlen(s), GL_UNSIGNED_BYTE, s);
}

// A frame: the viewer's drawing in linear light, multisampled, over its
// ground; its overlay and the help as they are; the picture asked for.
static void frame(void)
{
	size_t n = 0, w = 0;

	if (srgb) {
		glEnable(GL_FRAMEBUFFER_SRGB);
	}
	glViewport(0, 0, vg_w, vg_h);
	glClearColor((float)(srgb ? linear(V->bg[0]) : V->bg[0]),
		     (float)(srgb ? linear(V->bg[1]) : V->bg[1]),
		     (float)(srgb ? linear(V->bg[2]) : V->bg[2]), 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT |
		GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(vg_x - vg_w / 2.0 / vg_zoom, vg_x + vg_w / 2.0 / vg_zoom,
		vg_y - vg_h / 2.0 / vg_zoom, vg_y + vg_h / 2.0 / vg_zoom, -1,
		1);
	V->draw();
	glDisable(GL_FRAMEBUFFER_SRGB);
	glLoadIdentity();
	glOrtho(0, vg_w, 0, vg_h, -1, 1);
	if (V->overlay) {
		V->overlay();
	}
	for (; help && V->help[n]; n++) {
		w = strlen(V->help[n]) > w ? strlen(V->help[n]) : w;
	}
	glColor4f(0, 0, 0, 0.6F);
	if (help) {
		glRecti(0, vg_h - vg_top, 16 + (int)w * vg_font_w,
			vg_h - vg_top - 16 - (int)n * vg_font_h);
	}
	glColor3f(1, 1, 1);
	for (size_t k = 0; k < n; k++) {
		vg_text(8, vg_h - vg_top - 8 - (int)(k + 1) * vg_font_h,
			V->help[k]);
	}
	if (*shot) {
		FILE *f = fopen(shot, "wb");
		unsigned char *px = malloc((size_t)vg_w * (size_t)vg_h * 3);

		glPixelStorei(GL_PACK_ALIGNMENT, 1);
		if (px) {
			glReadPixels(0, 0, vg_w, vg_h, GL_RGB, GL_UNSIGNED_BYTE,
				     px);
		}
		if (f && px) {
			fprintf(f, "P6\n%d %d\n255\n", vg_w, vg_h);
			for (int y = vg_h - 1; y >= 0; y--) {
				fwrite(px + (size_t)y * (size_t)vg_w * 3, 1,
				       (size_t)vg_w * 3, f);
			}
		}
		if (!f || fclose(f)) {
			fprintf(stderr, "%s: cannot write\n", shot);
		}
		free(px);
		*shot = 0;
	}
	glXSwapBuffers(d, win);
}

int vg_start(int argc, char **argv, const struct vg_viewer *v)
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
		       8,
		       GLX_GREEN_SIZE,
		       8,
		       GLX_BLUE_SIZE,
		       8,
		       GLX_DEPTH_SIZE,
		       24,
		       GLX_STENCIL_SIZE,
		       8,
		       GLX_FRAMEBUFFER_SRGB_CAPABLE_ARB,
		       True,
		       GLX_SAMPLE_BUFFERS,
		       1,
		       GLX_SAMPLES,
		       4,
		       None };
	char name[64], cls[64];
	XClassHint hint = { name, cls };
	XSetWindowAttributes wa;
	XVisualInfo *vi;
	GLXFBConfig *fc = NULL;
	XFontStruct *font;
	int n = 0;

	V = v;
	d = XOpenDisplay(NULL);
	if (argc < 2 || argv[1][0] == '-' || !d) {
		fprintf(stderr,
			d ? "usage: %s FILE.vec...\n" : "%s: no display\n",
			v->name);
		exit(2);
	}
	for (srgb = 1; !fc && srgb >= 0; srgb--) {
		for (int s = 4; !fc && s >= 0; s -= 4) {
			attr[17] = srgb;
			attr[19] = !!s;
			attr[21] = s;
			fc = glXChooseFBConfig(d, DefaultScreen(d), attr, &n);
			fc = n > 0 ? fc : NULL;
		}
	}
	srgb++;
	vi = fc ? glXGetVisualFromFBConfig(d, fc[0]) : NULL;
	if (!vi) {
		fprintf(stderr, "%s: no OpenGL visual\n", v->name);
		exit(2);
	}
	memset(&wa, 0, sizeof wa);
	wa.colormap =
		XCreateColormap(d, DefaultRootWindow(d), vi->visual, AllocNone);
	wa.event_mask = ExposureMask | StructureNotifyMask | KeyPressMask |
			ButtonPressMask | ButtonReleaseMask |
			PointerMotionMask | LeaveWindowMask;
	win = XCreateWindow(d, DefaultRootWindow(d), 0, 0, (unsigned)vg_w,
			    (unsigned)vg_h, 0, vi->depth, InputOutput,
			    vi->visual, CWColormap | CWEventMask, &wa);
	snprintf(name, sizeof name, "%s", v->name);
	snprintf(cls, sizeof cls, "%c%s", v->name[0] - 'a' + 'A', v->name + 1);
	XSetClassHint(d, win, &hint);
	XStoreName(d, win, v->name);
	glXMakeCurrent(d, win,
		       glXCreateNewContext(d, fc[0], GLX_RGBA_TYPE, NULL,
					   True));
	XFree(vi);
	XFree(fc);
	font = XLoadQueryFont(d, "fixed");
	if (!font) {
		fprintf(stderr, "%s: no font fixed\n", v->name);
		exit(2);
	}
	font_base = (int)glGenLists(96);
	vg_font_w = font->max_bounds.width;
	vg_font_h = font->ascent + font->descent;
	glXUseXFont(font->fid, 32, 96, font_base);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnableClientState(GL_VERTEX_ARRAY);
	vg_init(argv[0]);
	return 1;
}

static double fit_box[4];

static void fit(void)
{
	const double *b = fit_box;

	vg_x = (b[0] + b[2]) / 2;
	vg_y = (b[1] + b[3]) / 2;
	vg_zoom =
		fmin(vg_w / (b[2] - b[0] + 1e-9), vg_h / (b[3] - b[1] + 1e-9)) *
		0.95;
}

static void wheel(int n, int px, int py)
{
	double x = vg_x + (px - vg_w / 2) / vg_zoom;
	double y = vg_y - (py - vg_h / 2) / vg_zoom;

	vg_zoom *= pow(1.25, n);
	vg_x = x - (px - vg_w / 2) / vg_zoom;
	vg_y = y + (py - vg_h / 2) / vg_zoom;
	fresh = 0;
}

static void drag(int dx, int dy)
{
	vg_x -= dx / vg_zoom;
	vg_y += dy / vg_zoom;
	fresh &= !dx && !dy;
}

// A click: the viewer's own (its bar), or the part there, or clear.
static void click(int px, int py)
{
	if (!V->click || !V->click(px, py)) {
		vg_sel = V->pick(vg_x + (px - vg_w / 2) / vg_zoom,
				 vg_y - (py - vg_h / 2) / vg_zoom, 3 / vg_zoom);
		if (vg_sel >= 0) {
			printf("select %s\n", vg_part_name(vg_sel));
		}
	}
}

// Centers the selected part; again, with nothing changed in between,
// zooms in two times about it.
static void center(void)
{
	static double last[3];
	static int last_sel = -1;
	double b[4];

	vg_part_box(vg_sel, b);
	if (b[2] < b[0]) {
		return;
	}
	if (fabs(vg_x - last[0]) + fabs(vg_y - last[1]) < 1e-9 &&
	    fabs(vg_zoom - last[2]) < 1e-12 && last_sel == vg_sel) {
		vg_zoom *= 2;
	} else {
		vg_x = (b[0] + b[2]) / 2;
		vg_y = (b[1] + b[3]) / 2;
	}
	last[0] = vg_x;
	last[1] = vg_y;
	last[2] = vg_zoom;
	last_sel = vg_sel;
}

// A command of stdin: of the other viewer, or of a script.
static void command(const char *s)
{
	if (!strcmp(s, "select")) {
		vg_sel = -1;
	} else if (!strncmp(s, "select ", 7)) {
		vg_sel = vg_part(s + 7);
	} else if (!strcmp(s, "center")) {
		center();
	} else if (!strncmp(s, "shot ", 5)) {
		// Drawn now, as all before it has left it.
		snprintf(shot, sizeof shot, "%s", s + 5);
		frame();
	} else if (V->command) {
		V->command(s);
	}
}

static void event(XEvent *e)
{
	static int down, moved, lx, ly;
	KeySym k = 0, kp;
	char ch;

	// The key as typed, Shift and all; a digit of the keypad as a digit,
	// Num Lock or not.
	if (e->type == KeyPress) {
		XLookupString(&e->xkey, &ch, 1, &k, NULL);
		kp = XLookupKeysym(&e->xkey, 1);
		k = kp >= XK_KP_1 && kp <= XK_KP_9 ? kp - XK_KP_1 + XK_1 : k;
	}

	draw = 1;
	if (e->type == ConfigureNotify) {
		vg_w = e->xconfigure.width;
		vg_h = e->xconfigure.height;
		if (fresh) {
			fit();
		}
	} else if (e->type == ButtonPress &&
		   (e->xbutton.button == 4 || e->xbutton.button == 5)) {
		wheel(e->xbutton.button == 4 ? 1 : -1, e->xbutton.x,
		      e->xbutton.y);
	} else if (e->type == ButtonPress && e->xbutton.button == 1) {
		down = 1;
		moved = 0;
		lx = e->xbutton.x;
		ly = e->xbutton.y;
	} else if (e->type == MotionNotify && down) {
		drag(e->xmotion.x - lx, e->xmotion.y - ly);
		moved |= e->xmotion.x != lx || e->xmotion.y != ly;
		lx = e->xmotion.x;
		ly = e->xmotion.y;
	} else if (e->type == ButtonRelease && e->xbutton.button == 1) {
		down = 0;
		if (!moved) {
			click(e->xbutton.x, e->xbutton.y);
		}
	} else if (e->type == MotionNotify || e->type == LeaveNotify) {
		draw = V->move &&
		       V->move(e->type == LeaveNotify ? -1 : e->xmotion.x,
			       e->type == LeaveNotify ? -1 : e->xmotion.y);
	} else if (k == XK_q) {
		exit(0);
	} else if (k == XK_space) {
		fit();
	} else if (k == XK_c) {
		center();
	} else if (k == XK_question) {
		help = !help;
	} else if (k && V->key) {
		V->key(k);
	}
}

void vg_loop(const double box[4])
{
	char buf[4096];
	size_t have = 0;
	int in = 0;

	memcpy(fit_box, box, sizeof fit_box);
	fit();
	XMapWindow(d, win);
	setvbuf(stdout, NULL, _IOLBF, 0);
	for (;;) {
		struct pollfd fd[2] = { { ConnectionNumber(d), POLLIN, 0 },
					{ in, POLLIN, 0 } };
		ssize_t r;
		char *nl;

		while (XPending(d)) {
			XEvent e;

			XNextEvent(d, &e);
			event(&e);
		}
		if (draw) {
			draw = 0;
			frame();
			continue;
		}
		if (poll(fd, 2, -1) < 0) {
			exit(2);
		}
		// The input of the window first: a command (a picture) sees
		// all that came before it.
		if ((fd[0].revents & POLLIN) ||
		    !(fd[1].revents & (POLLIN | POLLHUP))) {
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
			command(buf);
			draw = 1;
			have -= (size_t)(nl + 1 - buf);
			memmove(buf, nl + 1, have + 1);
		}
		have = have == sizeof buf - 1 ? 0 : have; // Too long a line.
	}
}
