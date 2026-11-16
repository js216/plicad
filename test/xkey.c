// SPDX-License-Identifier: GPL-3.0-only
// xkey.c -- Type keys and move the mouse in the window of a class, by
// XTest, for a test
// Copyright (c) 2026 Jakob Kastelic

// Usage: xkey [-t] CLASS ACTION...
//
// Finds the window whose class is CLASS, moves the pointer into it, and
// does each action as a user would, so a test sees what the viewer makes
// of real input:
//   KEYSYM          types the key, with Shift where the key needs it (a
//                   key of the keypad bare, as with Num Lock off)
//   move X Y        moves the pointer to (X, Y), pixels of the window
//   click X Y       moves there and clicks the left button
//   wheel N X Y     moves there and turns the wheel N notches, in (away
//                   from the user) if N > 0, out if N < 0
//   drag DX DY      presses the left button where the pointer is, moves
//                   it by (DX, DY) and lets it go
// With -t it measures the viewer from outside: after each action it
// waits for the window to be drawn again (its next damage, by the Damage
// extension) and prints "frame MS", the time from the action to the
// picture, on stdout; 2000 if no picture comes within 2 s.

#define _DEFAULT_SOURCE

#include <X11/XKBlib.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XTest.h>
#include <X11/extensions/Xdamage.h>
#include <X11/keysym.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static Display *d;
static Window win;
static int wx, wy, px, py, timing, damage_event;
static Damage dmg;

static Window find(Window w, const char *cls)
{
	Window r, p, *k, f = 0;
	unsigned n;
	XClassHint h;

	if (XGetClassHint(d, w, &h)) {
		int ok = !strcmp(h.res_name, cls);

		XFree(h.res_name);
		XFree(h.res_class);
		if (ok) {
			return w;
		}
	}
	if (!XQueryTree(d, w, &r, &p, &k, &n)) {
		return 0;
	}
	for (unsigned i = 0; !f && i < n; i++) {
		f = find(k[i], cls);
	}
	if (k) {
		XFree(k);
	}
	return f;
}

static double now_ms(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return (double)t.tv_sec * 1e3 + (double)t.tv_nsec / 1e6;
}

// Forgets the damage so far: the next picture is the one waited for.
static void forget(void)
{
	XEvent e;

	XSync(d, False);
	while (XPending(d)) {
		XNextEvent(d, &e);
	}
	XDamageSubtract(d, dmg, None, None);
	XSync(d, False);
}

// Waits for the window's next picture and prints the time since t0.
static void picture(double t0)
{
	double t = 2000;

	for (;;) {
		struct pollfd fd = { ConnectionNumber(d), POLLIN, 0 };
		XEvent e;
		double left = t0 + 2000 - now_ms();

		while (XPending(d)) {
			XNextEvent(d, &e);
			if (e.type == damage_event + XDamageNotify) {
				t = now_ms() - t0;
				left = -1;
			}
		}
		if (left < 0 || poll(&fd, 1, (int)left) <= 0) {
			break;
		}
	}
	XDamageSubtract(d, dmg, None, None);
	printf("frame %.3f\n", t);
}

static void move(int x, int y)
{
	px = x;
	py = y;
	XTestFakeMotionEvent(d, -1, wx + x, wy + y, 0);
}

static void button(unsigned b)
{
	XTestFakeButtonEvent(d, b, True, 0);
	XTestFakeButtonEvent(d, b, False, 0);
}

static void key(const char *name)
{
	KeySym k = XStringToKeysym(name);
	KeyCode kc = XKeysymToKeycode(d, k);
	// A digit of the keypad is typed as is, Num Lock off.
	int shift = XkbKeycodeToKeysym(d, kc, 0, 0) != k && !IsKeypadKey(k);
	KeyCode sc = XKeysymToKeycode(d, XK_Shift_L);

	if (!kc) {
		fprintf(stderr, "xkey: no key %s\n", name);
		exit(2);
	}
	if (shift) {
		XTestFakeKeyEvent(d, sc, True, 0);
	}
	XTestFakeKeyEvent(d, kc, True, 0);
	XTestFakeKeyEvent(d, kc, False, 0);
	if (shift) {
		XTestFakeKeyEvent(d, sc, False, 0);
	}
}

// Does the action at argv[i]; returns the number of its words, 0 if it
// is not one.
static int act(int argc, char **argv, int i)
{
	const char *a = argv[i];
	int n = !strcmp(a, "move") || !strcmp(a, "click") ||
				!strcmp(a, "drag") ?
			3 :
		!strcmp(a, "wheel") ? 4 :
				      1;
	int v[3] = { 0, 0, 0 };

	if (i + n > argc) {
		return 0;
	}
	for (int k = 1; k < n; k++) {
		char *end;

		v[k - 1] = (int)strtol(argv[i + k], &end, 10);
		if (*end || end == argv[i + k]) {
			return 0;
		}
	}
	if (!strcmp(a, "move")) {
		move(v[0], v[1]);
	} else if (!strcmp(a, "click")) {
		move(v[0], v[1]);
		button(1);
	} else if (!strcmp(a, "wheel")) {
		move(v[1], v[2]);
		for (int k = 0; k < abs(v[0]); k++) {
			button(v[0] > 0 ? 4 : 5);
		}
	} else if (!strcmp(a, "drag")) {
		XTestFakeButtonEvent(d, 1, True, 0);
		move(px + v[0], py + v[1]);
		XTestFakeButtonEvent(d, 1, False, 0);
	} else {
		key(a);
	}
	return n;
}

int main(int argc, char **argv)
{
	Window c;
	int a = 1, err;

	timing = argc > 1 && !strcmp(argv[1], "-t");
	a += timing;
	d = argc > a + 1 ? XOpenDisplay(NULL) : NULL;
	win = 0;
	for (int i = 0; d && !win && i < 60; i++) { // Till it is mapped.
		win = find(DefaultRootWindow(d), argv[a]);
		usleep(win ? 0 : 50000);
	}
	if (!win || (timing && !XDamageQueryExtension(d, &damage_event, &err))) {
		fprintf(stderr,
			"usage: xkey [-t] CLASS ACTION... (with the window%s)\n",
			timing ? " and Damage" : "");
		return 2;
	}
	if (timing) {
		dmg = XDamageCreate(d, win, XDamageReportNonEmpty);
	}
	XTranslateCoordinates(d, win, DefaultRootWindow(d), 0, 0, &wx, &wy, &c);
	move(300, 300);
	for (int i = a + 1; i < argc;) {
		int key_like, n;
		double t0;

		if (timing) {
			forget();
		}
		t0 = now_ms();
		n = act(argc, argv, i);
		if (!n) {
			fprintf(stderr, "xkey: a bad action at %s\n", argv[i]);
			return 2;
		}
		key_like = n == 1;
		XSync(d, False);
		if (timing) {
			picture(t0);
		} else if (key_like) {
			usleep(50000);
		}
		i += n;
	}
	XCloseDisplay(d);
	return 0;
}
