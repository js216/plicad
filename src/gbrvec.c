// SPDX-License-Identifier: GPL-3.0-only
// gbrvec.c -- Turn a Gerber or Excellon file into plain vectors
// Copyright (c) 2026 Jakob Kastelic

// Usage: gbrvec FILE > FILE.vec
//
// Reads an RS-274X (Gerber) file, or an Excellon drill file, and writes
// what it draws as vec(5): one record per line, in mm,
//   layer NAME              the layer, first: KiCad's name of the file
//                           function (TF.FileFunction) of Gerber X2,
//                           F.Cu, In1.Cu, B.Cu, F.Mask, Edge.Cuts, ...,
//                           and Drill for a drill file
//   dark | clear            the polarity of what follows
//   disk X Y R              a filled circle
//   line W X Y X Y ...      a stroke of width W, round at its ends
//   poly X Y X Y ...        a filled polygon
//   name NAME X0 Y0 X1 Y1   the box round all a part's objects
//   zone NET ... end        the fill of a zone of copper
//   part NAME ... part      the objects of a part
// A part's objects are those under its component attribute (TO.C) or
// a pad attribute (TO.P, by its part);
// the names come last, sorted.  A zone is a region under a net
// attribute (TO.N) and what follows it under that attribute.  Gerber: the format and units (FS, MO),
// the standard apertures (C, R, O) and aperture macros (primitives 1,
// 4, 5, 20 and 21, with the arithmetic of their parameters), draws,
// flashes, arcs (G02, G03, as short lines) and regions (G36, G37), and
// the polarity (LP).  Excellon: tools and holes, as disks.

#define _POSIX_C_SOURCE 200809L // For strtok_r.

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define ARC 256 // Segments to a full circle: smooth however far a viewer zooms.

struct aper {
	int code;
	char type[64];
	double p[16];
	int np;
};

struct macro {
	char name[64];
	char body[8192]; // The primitives, '*' between them.
};

struct named {
	char *name;
	double box[4];
};

static struct aper *aps;
static size_t naps;
static struct macro *macs;
static size_t nmacs;
static struct named *names;
static double *contour; // The points of the region being read.
static size_t nnames;
static char current_name[512], net[512];
static int in_zone, zone_clear; // In a zone; and it has cleared.
static double unit = 1, scale = 1e-6;
static int dark = -1;
static const char *file;

// Prints the message and exits; as a macro, the exit shows to the
// analyzer.
#define die(msg) (fprintf(stderr, "%s: %s\n", file, msg), exit(2))

// The whole number of steps at or over v.
static int steps(double v)
{
	double c = ceil(v);

	return (int)c;
}

static void *grow(void *p, size_t n, size_t size)
{
	p = realloc(p, (n + 1) * size);
	if (!p) {
		die("out of memory");
	}
	memset((char *)p + n * size, 0, size);
	return p;
}

// Grows the box of the part being drawn, if any, by (x, y).
// Copies the first n bytes of src, or as many as fit, into dst.
static void copy_prefix(char *dst, size_t size, const char *src, size_t n)
{
	n = n < size - 1 ? n : size - 1;
	memcpy(dst, src, n);
	dst[n] = 0;
}

static void touch(double x, double y, double r)
{
	size_t i;

	if (!*current_name) {
		return;
	}
	for (i = 0; i < nnames && strcmp(names[i].name, current_name); i++) {
	}
	if (i == nnames) {
		names = grow(names, nnames, sizeof *names);
		names[i].name = malloc(strlen(current_name) + 1);
		if (!names[i].name) {
			die("out of memory");
		}
		strcpy(names[i].name, current_name);
		names[i].box[0] = names[i].box[1] = 1e300;
		names[i].box[2] = names[i].box[3] = -1e300;
		nnames++;
	}
	names[i].box[0] = fmin(names[i].box[0], x - r);
	names[i].box[1] = fmin(names[i].box[1], y - r);
	names[i].box[2] = fmax(names[i].box[2], x + r);
	names[i].box[3] = fmax(names[i].box[3], y + r);
}

static void end_zone(void)
{
	if (in_zone) {
		puts("end");
		in_zone = zone_clear = 0;
	}
}

static void set_dark(int d)
{
	if (in_zone && d && zone_clear) {
		end_zone();
	}
	zone_clear |= in_zone && !d;
	if (d != dark) {
		puts(d ? "dark" : "clear");
		dark = d;
	}
}

static void disk(double x, double y, double r)
{
	printf("disk %.6g %.6g %.6g\n", x, y, r);
	touch(x, y, r);
}

static void poly(const double *xy, int n)
{
	printf("poly");
	for (int i = 0; i < n; i++) {
		printf(" %.6g %.6g", xy[2 * i], xy[2 * i + 1]);
		touch(xy[2 * i], xy[2 * i + 1], 0);
	}
	printf("\n");
}

// A rectangle w by h at (x, y), turned rot degrees round the origin.
static void rect(double x, double y, double w, double h, double rot)
{
	double xy[8], c = cos(rot * PI / 180), s = sin(rot * PI / 180);
	const double dx[4] = { -1, 1, 1, -1 }, dy[4] = { -1, -1, 1, 1 };

	for (int i = 0; i < 4; i++) {
		double px = x + dx[i] * w / 2, py = y + dy[i] * h / 2;

		xy[2 * i] = px * c - py * s;
		xy[2 * i + 1] = px * s + py * c;
	}
	poly(xy, 4);
}

// The arithmetic of a macro parameter: numbers, $n, + - x / and ().
static char *ex;
static const double *vars;

static double expr(void);

static double atom(void)
{
	double v;

	while (*ex == ' ') {
		ex++;
	}
	if (*ex == '(') {
		ex++;
		v = expr();
		if (*ex == ')') {
			ex++;
		}
		return v;
	}
	if (*ex == '-') {
		ex++;
		return -atom();
	}
	if (*ex == '+') {
		ex++;
		return atom();
	}
	if (*ex == '$') {
		int k = (int)strtol(ex + 1, &ex, 10);

		return k >= 1 && k <= 16 ? vars[k - 1] : 0;
	}
	v = strtod(ex, &ex);
	return v;
}

static double term(void)
{
	double v = atom();

	for (;;) {
		if (*ex == 'x' || *ex == 'X') {
			ex++;
			v *= atom();
		} else if (*ex == '/') {
			ex++;
			v /= atom();
		} else {
			return v;
		}
	}
}

static double expr(void)
{
	double v = term();

	for (;;) {
		if (*ex == '+') {
			ex++;
			v += term();
		} else if (*ex == '-') {
			ex++;
			v -= term();
		} else {
			return v;
		}
	}
}

// Flashes macro m with parameters p at (x, y).
static void flash_macro(const struct macro *m, const double *p, double x,
			double y)
{
	char *body = malloc(strlen(m->body) + 1), *prim, *save = NULL;

	if (!body) {
		die("out of memory");
	}
	strcpy(body, m->body);
	vars = p;
	for (prim = strtok_r(body, "*", &save); prim;
	     prim = strtok_r(NULL, "*", &save)) {
		double a[256];
		int n = 0, code;

		while (isspace((unsigned char)*prim)) {
			prim++;
		}
		if (!*prim || *prim == '0' || *prim == '$') {
			continue; // A comment, or a variable defined: not handled.
		}
		ex = prim;
		while (*ex && n < 256) {
			a[n++] = expr();
			while (*ex == ' ' || *ex == '\n' || *ex == '\r') {
				ex++;
			}
			if (*ex == ',') {
				ex++;
			} else {
				break;
			}
		}
		if (n < 1) {
			continue;
		}
		code = (int)a[0];
		if (code == 1 && n >= 5) {
			disk(x + a[3] * unit, y + a[4] * unit, a[2] * unit / 2);
		} else if (code == 4 && n >= 5) {
			int k = (int)a[2];
			double *xy = malloc((size_t)(k + 1) * 2 * sizeof *xy);
			double rot = n > 5 + 2 * k ? a[5 + 2 * k] : 0;
			double c = cos(rot * PI / 180), s = sin(rot * PI / 180);

			if (!xy) {
				die("out of memory");
			}
			for (int i = 0; i <= k && 4 + 2 * i < n; i++) {
				double px = a[3 + 2 * i] * unit,
				       py = a[4 + 2 * i] * unit;

				xy[2 * i] = x + px * c - py * s;
				xy[2 * i + 1] = y + px * s + py * c;
			}
			poly(xy, k);
			free(xy);
		} else if (code == 20 && n >= 7) {
			double x1 = a[3] * unit, y1 = a[4] * unit,
			       x2 = a[5] * unit, y2 = a[6] * unit,
			       w = a[2] * unit, l = hypot(x2 - x1, y2 - y1);
			double ux = l > 0 ? (x2 - x1) / l : 1,
			       uy = l > 0 ? (y2 - y1) / l : 0;
			double xy[8] = { x1 - uy * w / 2, y1 + ux * w / 2,
					 x2 - uy * w / 2, y2 + ux * w / 2,
					 x2 + uy * w / 2, y2 - ux * w / 2,
					 x1 + uy * w / 2, y1 - ux * w / 2 };

			for (int i = 0; i < 4; i++) {
				xy[2 * i] += x;
				xy[2 * i + 1] += y;
			}
			poly(xy, 4);
		} else if (code == 21 && n >= 6) {
			double xy[8], rot = n > 6 ? a[6] : 0;
			double c = cos(rot * PI / 180), s = sin(rot * PI / 180);
			const double dx[4] = { -1, 1, 1, -1 },
				     dy[4] = { -1, -1, 1, 1 };

			for (int i = 0; i < 4; i++) {
				double px =
					a[4] * unit + dx[i] * a[2] * unit / 2;
				double py =
					a[5] * unit + dy[i] * a[3] * unit / 2;

				xy[2 * i] = x + px * c - py * s;
				xy[2 * i + 1] = y + px * s + py * c;
			}
			poly(xy, 4);
		} else if (code == 5 && n >= 6) {
			int k = (int)a[2];
			double xy[2 * 64] = { 0 }, rot = n > 6 ? a[6] : 0;

			k = k < 3 ? 3 : k > 64 ? 64 : k;
			for (int i = 0; i < k; i++) {
				double t = (rot + 360.0 * i / k) * PI / 180;

				xy[2 * i] = x + a[3] * unit +
					    a[5] * unit / 2 * cos(t);
				xy[2 * i + 1] = y + a[4] * unit +
						a[5] * unit / 2 * sin(t);
			}
			poly(xy, k);
		}
	}
	free(body);
}

static const struct aper *find_aper(int code)
{
	for (size_t i = 0; i < naps; i++) {
		if (aps[i].code == code) {
			return &aps[i];
		}
	}
	return NULL;
}

static void flash(const struct aper *a, double x, double y)
{
	if (!a) {
		return;
	}
	if (!strcmp(a->type, "C")) {
		disk(x, y, a->p[0] * unit / 2);
	} else if (!strcmp(a->type, "R") && a->np >= 2) {
		rect(x, y, a->p[0] * unit, a->p[1] * unit, 0);
	} else if (!strcmp(a->type, "O") && a->np >= 2) {
		double w = a->p[0] * unit, h = a->p[1] * unit,
		       r = fmin(w, h) / 2;

		if (w > h) {
			printf("line %.6g %.6g %.6g %.6g %.6g\n", 2 * r,
			       x - w / 2 + r, y, x + w / 2 - r, y);
		} else {
			printf("line %.6g %.6g %.6g %.6g %.6g\n", 2 * r, x,
			       y - h / 2 + r, x, y + h / 2 - r);
		}
		touch(x, y, fmax(w, h) / 2);
	} else {
		for (size_t i = 0; i < nmacs; i++) {
			if (!strcmp(macs[i].name, a->type)) {
				flash_macro(&macs[i], a->p, x, y);
			}
		}
	}
}

// Reads an extended command, %...%.
static void extended(char *s)
{
	if (!strncmp(s, "FS", 2)) {
		const char *x = strchr(s, 'X');

		if (x && isdigit((unsigned char)x[2])) {
			scale = pow(10, -(x[2] - '0'));
		}
	} else if (!strncmp(s, "MOIN", 4)) {
		unit = 25.4;
	} else if (!strncmp(s, "MOMM", 4)) {
		unit = 1;
	} else if (!strncmp(s, "LP", 2)) {
		set_dark(s[2] == 'D');
	} else if (!strncmp(s, "AD", 2) && s[2] == 'D') {
		struct aper a;
		char *p = s + 3, *c;

		memset(&a, 0, sizeof a);
		a.code = (int)strtol(p, &p, 10);
		c = strchr(p, ',');
		copy_prefix(a.type, sizeof a.type, p,
			    c ? (size_t)(c - p) : strcspn(p, "*"));
		while (c && a.np < 16) {
			a.p[a.np++] = strtod(c + 1, &c);
			if (*c != 'X') {
				break;
			}
		}
		aps = grow(aps, naps, sizeof *aps);
		aps[naps++] = a;
	} else if (!strncmp(s, "AM", 2)) {
		struct macro m;
		const char *star = strchr(s, '*');

		if (!star) {
			return;
		}
		copy_prefix(m.name, sizeof m.name, s + 2,
			    (size_t)(star - s - 2));
		copy_prefix(m.body, sizeof m.body, star + 1, strlen(star + 1));
		macs = grow(macs, nmacs, sizeof *macs);
		macs[nmacs++] = m;
	} else if (!strncmp(s, "TO.C,", 5) || !strncmp(s, "TO.P,", 5)) {
		char was[sizeof current_name];

		memcpy(was, current_name, sizeof was);
		copy_prefix(current_name, sizeof current_name, s + 5,
			    strcspn(s + 5, ",*"));
		if (strcmp(was, current_name)) {
			printf("part %s\n", current_name);
		}
	} else if (!strncmp(s, "TO.N,", 5)) {
		end_zone();
		copy_prefix(net, sizeof net, s + 5, strcspn(s + 5, ",*"));
	} else if (!strncmp(s, "TD", 2)) {
		end_zone();
		if (*current_name) {
			puts("part");
		}
		*current_name = 0;
		*net = 0;
	}
}

static long coord(const char *s, char key, int *found)
{
	const char *p = strchr(s, key);

	*found = p != NULL;
	return p ? strtol(p + 1, NULL, 10) : 0;
}

static void gerber(char *text)
{
	double x = 0, y = 0;
	int interp = 1, region = 0, code = -1;
	size_t nc = 0, cap = 0;
	const struct aper *ap = NULL;
	char *s = text;

	set_dark(1);
	while (*s) {
		char *end;

		while (isspace((unsigned char)*s)) {
			s++;
		}
		if (!*s) {
			break;
		}
		if (*s == '%') {
			end = strchr(s + 1, '%');
			if (!end) {
				break;
			}
			*end = 0;
			extended(s + 1);
			s = end + 1;
			continue;
		}
		end = strchr(s, '*');
		if (!end) {
			break;
		}
		*end = 0;
		if (s[0] == 'G' && s[1] == '0' && s[2] == '4') {
			s = end + 1;
			continue;
		}
		if (strstr(s, "G01")) {
			interp = 1;
		}
		if (strstr(s, "G02")) {
			interp = 2;
		}
		if (strstr(s, "G03")) {
			interp = 3;
		}
		if (strstr(s, "G36")) {
			region = 1;
			nc = 0;
			if (*net && !in_zone) {
				printf("zone %s\n", net);
				in_zone = 1;
			}
		}
		if (strstr(s, "G37")) {
			if (nc > 2) {
				poly(contour, (int)nc);
			}
			region = 0;
			nc = 0;
		}
		{
			const char *d = strrchr(s, 'D');
			int fx, fy, fi, fj;
			long lx = coord(s, 'X', &fx), ly = coord(s, 'Y', &fy);
			long li = coord(s, 'I', &fi), lj = coord(s, 'J', &fj);
			double nx = fx ? (double)lx * scale * unit : x,
			       ny = fy ? (double)ly * scale * unit : y;

			code = d ? atoi(d + 1) : (fx || fy ? code : -1);
			if (code >= 10) {
				ap = find_aper(code);
			} else if (code == 3) {
				flash(ap, nx, ny);
			} else if (code == 2) {
				if (region && nc > 2) {
					poly(contour, (int)nc);
				}
				nc = 0;
			} else if (code == 1) {
				// The points of the draw: straight, or an arc.
				double pts[2 * (ARC + 2)];
				int np = 0;

				pts[np++] = x;
				pts[np++] = y;
				if (interp > 1) {
					double ccx =
						x + (fi ? (double)li * scale *
								     unit :
							  0);
					double ccy =
						y + (fj ? (double)lj * scale *
								     unit :
							  0);
					double r = hypot(x - ccx, y - ccy);
					double a0 = atan2(y - ccy, x - ccx),
					       a1 = atan2(ny - ccy, nx - ccx);
					int k;

					if (interp == 3 && a1 <= a0) {
						a1 += 2 * PI;
					}
					if (interp == 2 && a1 >= a0) {
						a1 -= 2 * PI;
					}
					k = steps(fabs(a1 - a0) / (2 * PI) *
						  ARC);
					k = k < 1 ? 1 : k > ARC ? ARC : k;
					np = 0;
					for (int i = 0; i <= k; i++) {
						double t =
							a0 + (a1 - a0) * i / k;

						pts[np++] = ccx + r * cos(t);
						pts[np++] = ccy + r * sin(t);
					}
				} else {
					pts[np++] = nx;
					pts[np++] = ny;
				}
				if (region) {
					for (int i = nc ? 2 : 0; i < np;
					     i += 2) {
						if (nc + 1 >= cap) {
							cap = cap ? 2 * cap :
								    256;
							contour = grow(
								contour,
								2 * cap - 1,
								sizeof *contour);
						}
						contour[2 * nc] = pts[i];
						contour[2 * nc + 1] =
							pts[i + 1];
						nc++;
					}
				} else if (ap) {
					double w = !strcmp(ap->type, "C") ||
								   ap->np < 2 ?
							   ap->p[0] * unit :
							   fmin(ap->p[0],
								ap->p[1]) *
								   unit;

					printf("line %.6g", w);
					for (int i = 0; i < np; i += 2) {
						printf(" %.6g %.6g", pts[i],
						       pts[i + 1]);
						touch(pts[i], pts[i + 1],
						      w / 2);
					}
					printf("\n");
				}
			}
			if (code == 2 && region) {
				if (nc + 1 >= cap) {
					cap = cap ? 2 * cap : 256;
					contour = grow(contour, 2 * cap - 1,
						       sizeof *contour);
				}
				contour[0] = nx;
				contour[1] = ny;
				nc = 1;
			}
			x = nx;
			y = ny;
		}
		s = end + 1;
	}
}

static void excellon(char *text)
{
	double tools[1000] = { 0 }, d = 0, u = 1;
	char *line, *save = NULL;

	set_dark(1);
	for (line = strtok_r(text, "\n", &save); line;
	     line = strtok_r(NULL, "\n", &save)) {
		if (!strncmp(line, "INCH", 4)) {
			u = 25.4;
		} else if (line[0] == 'T') {
			int t = atoi(line + 1);

			if (t >= 0 && t < 1000) {
				const char *c = strchr(line, 'C');

				if (c) {
					tools[t] = atof(c + 1) * u;
				}
				d = tools[t];
			}
		} else if (line[0] == 'X') {
			const char *yp = strchr(line, 'Y');

			if (yp) {
				disk(atof(line + 1) * u, atof(yp + 1) * u,
				     d / 2);
			}
		}
	}
}

// KiCad's name of the layer of a file function of Gerber X2: F.Cu,
// In1.Cu, ..., B.Cu, F.Mask, F.Paste, F.Silkscreen, F.Courtyard (B. on
// the bottom), Edge.Cuts; "" for another.
static void layer_name(const char *fn, char *name, size_t size)
{
	char f[128];
	const char *side, *pos;
	int n = 0;

	copy_prefix(f, sizeof f, fn, strcspn(fn, "*%\r\n"));
	side = strstr(f, ",Bot") ? "B" : "F";
	pos = strchr(f, ',') ? strchr(strchr(f, ',') + 1, ',') : NULL;
	*name = 0;
	if (sscanf(f, "Copper,L%d", &n) == 1) {
		if (pos && !strncmp(pos, ",Top", 4)) {
			snprintf(name, size, "F.Cu");
		} else if (pos && !strncmp(pos, ",Bot", 4)) {
			snprintf(name, size, "B.Cu");
		} else {
			snprintf(name, size, "In%d.Cu", n - 1);
		}
	} else if (!strncmp(f, "Soldermask,", 11)) {
		snprintf(name, size, "%s.Mask", side);
	} else if (!strncmp(f, "Paste,", 6)) {
		snprintf(name, size, "%s.Paste", side);
	} else if (!strncmp(f, "Legend,", 7)) {
		snprintf(name, size, "%s.Silkscreen", side);
	} else if (!strncmp(f, "Other,Courtyard", 15)) {
		snprintf(name, size, "%s.Courtyard", side);
	} else if (!strncmp(f, "Profile", 7)) {
		snprintf(name, size, "Edge.Cuts");
	}
}

static int by_name(const void *a, const void *b)
{
	return strcmp(((const struct named *)a)->name,
		      ((const struct named *)b)->name);
}

int main(int argc, char **argv)
{
	FILE *f;
	char *text;
	long n;

	if (argc != 2) {
		fprintf(stderr, "usage: gbrvec FILE > FILE.vec\n");
		return 2;
	}
	file = argv[1];
	f = fopen(file, "rb");
	if (!f || fseek(f, 0, SEEK_END) || (n = ftell(f)) < 0 ||
	    fseek(f, 0, SEEK_SET)) {
		die("cannot read");
	}
	text = malloc((size_t)n + 1);
	if (!text || fread(text, 1, (size_t)n, f) != (size_t)n) {
		die("cannot read");
	}
	text[n] = 0;
	fclose(f);
	printf("# vec of %s\n",
	       strrchr(file, '/') ? strrchr(file, '/') + 1 : file);
	{
		const char *fn = strstr(text, "TF.FileFunction,");
		char layer[64] = "";

		if (fn) {
			layer_name(fn + 16, layer, sizeof layer);
		}
		if (strstr(text, "M48")) {
			snprintf(layer, sizeof layer, "Drill");
		}
		if (*layer) {
			printf("layer %s\n", layer);
		}
	}
	if (strstr(text, "M48")) {
		excellon(text);
	} else {
		gerber(text);
		end_zone();
	}
	if (nnames) {
		qsort(names, nnames, sizeof *names, by_name);
	}
	for (size_t i = 0; i < nnames; i++) {
		printf("name %s %.6g %.6g %.6g %.6g\n", names[i].name,
		       names[i].box[0], names[i].box[1], names[i].box[2],
		       names[i].box[3]);
	}
	return 0;
}
