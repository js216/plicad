// SPDX-License-Identifier: GPL-3.0-only
// xresize.c -- Resize the window of a class, as a window manager does
// Copyright (c) 2026 Jakob Kastelic

// Usage: xresize CLASS W H
//
// Resizes the first window whose class is CLASS to W by H pixels, so a
// test sees what the program in it makes of the ConfigureNotify that a
// window manager would send.  Exit status 1 if no such window shows.

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The first window under w, w too, of the class cls, or 0.
static Window find(Display *d, Window w, const char *cls)
{
	Window root = 0;
	Window up = 0;
	Window *kids = NULL;
	unsigned n = 0;
	Window found = 0;
	XClassHint h;

	if (XGetClassHint(d, w, &h)) {
		int ok = !strcmp(h.res_name, cls);

		XFree(h.res_name);
		XFree(h.res_class);
		if (ok) {
			return w;
		}
	}
	if (!XQueryTree(d, w, &root, &up, &kids, &n)) {
		return 0;
	}
	for (unsigned i = 0; i < n && !found; i++) {
		found = find(d, kids[i], cls);
	}
	if (kids) {
		XFree(kids);
	}
	return found;
}

int main(int argc, char **argv)
{
	Display *d = NULL;
	Window w = 0;
	long x = 0;
	long y = 0;

	if (argc != 4) {
		(void)fprintf(stderr, "usage: xresize CLASS W H\n");
		return 2;
	}
	x = strtol(argv[2], NULL, 10);
	y = strtol(argv[3], NULL, 10);
	d = XOpenDisplay(NULL);
	if (!d || x < 1 || y < 1) {
		(void)fprintf(stderr, "xresize: no display, or a bad size\n");
		return 2;
	}
	w = find(d, DefaultRootWindow(d), argv[1]);
	if (w) {
		XResizeWindow(d, w, (unsigned)x, (unsigned)y);
		XSync(d, False);
	}
	XCloseDisplay(d);
	return w ? 0 : 1;
}
