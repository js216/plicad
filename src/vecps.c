// SPDX-License-Identifier: GPL-3.0-only
// vecps.c -- Draw vector files as pages of PostScript
// Copyright (c) 2026 Jakob Kastelic

// Usage: vecps [-w X0 Y0 X1 Y1 | -r X0 Y0 X1 Y1 | -p PART]... [-n]
//              FILE[=R,G,B[,line]]... > out.ps
//
// Draws each vec(5) file, the first underneath, in its color (each of
// R, G, B from 0 to 1, black by default): filled, or with line, its
// outlines only, so a file drawn over another shows both.  The window
// (-w, in the units of the files) is the region drawn, by default all
// of the first file; it fills a letter page, landscape.  A first file
// with page records is drawn as those pages instead, each at its size and
// scale 1: page k of each file, its records from its k-th page record to
// the next (those before the first are of the first), on page k of the
// PostScript.  Clear polarity is drawn white.  Text is set in Helvetica,
// or Helvetica-Oblique for the font I, in ISO Latin-1 (but - is the
// hyphen, as in the fonts' own encoding).  With -n the
// names of the boxes of the first file are written in them.  The
// window by default holds all the files, their lines' widths and
// disks' radii counted.  Each port of the sheet of the first file links
// to pages as schview(1) follows its clicks: its circle and name to the
// first page it lists other than its own, each number of its list but
// its own to that page (pdfmark /Link, /Fit, no border); a global links
// nowhere.
//
// A region render (-r, or -p) crops to a window and names what is in it:
// -r X0 Y0 X1 Y1 is the window; -p PART (by its name or designator,
// again for more, or a list with commas) the box of the parts named,
// grown by a fifth of its size and 1 unit.  Each part in the window
// shows its designator (from its part bracket) in its box, and each
// track and pad of a net its net's name, along the track or in the pad,
// sized to it but readable on the page.

#define _POSIX_C_SOURCE 200809L // For strtok_r.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LINE = 1 << 20 };

// Copies the first n bytes of src, or as many as fit, into dst.
static void copy_prefix(char *dst, size_t size, const char *src, size_t n)
{
	n = n < size - 1 ? n : size - 1;
	memcpy(dst, src, n);
	dst[n] = 0;
}

// Prints the message and exits; as a macro, the exit shows to the
// analyzer.
#define die(f, msg) (fprintf(stderr, "%s: %s\n", f, msg), exit(2))

// Writes s as a PostScript string in ISO Latin-1, from UTF-8.
static void ps_string(const char *s)
{
	putchar('(');
	for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
		unsigned c = *p;

		if (c == 0xCE && p[1] == 0xBC) {
			c = 0xB5; // Greek mu, as the micro sign.
			p++;
		} else if ((c == 0xC2 || c == 0xC3) && p[1] >= 0x80 &&
			   p[1] < 0xC0) {
			c = ((c & 3) << 6) | (p[1] & 0x3F);
			p++;
		} else if (c >= 0x80) {
			while (p[1] >= 0x80 && p[1] < 0xC0) {
				p++;
			}
			c = '?';
		}
		if (c == '(' || c == ')' || c == '\\') {
			putchar('\\');
			putchar((int)c);
		} else if (c < 32 || c >= 127) {
			printf("\\%03o", c);
		} else {
			putchar((int)c);
		}
	}
	putchar(')');
}

// The number of page records of a file, and the size of its page k
// (from 1), if it has one.
static int page_of(const char *path, int k, double *w, double *h)
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE);
	int n = 0;
	double pw, ph;

	while (f && buf && fgets(buf, LINE, f)) {
		if (sscanf(buf, "page %lf %lf", &pw, &ph) == 2 && ++n == k) {
			*w = pw;
			*h = ph;
		}
	}
	if (f) {
		fclose(f);
	}
	free(buf);
	return n;
}

// Grows e by the extent of a file: of its drawing records alone, each
// line by its half width and each disk by its radius.
static void extent(const char *path, double e[4])
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE);

	if (!f || !buf) {
		die(path, "cannot read");
	}
	while (fgets(buf, LINE, f)) {
		char *s = buf + 5, *end;
		double v[3], r = 0;
		int k = 0;

		if (sscanf(buf, "disk %lf %lf %lf", &v[0], &v[1], &v[2]) == 3) {
			e[0] = fmin(e[0], v[0] - v[2]);
			e[1] = fmin(e[1], v[1] - v[2]);
			e[2] = fmax(e[2], v[0] + v[2]);
			e[3] = fmax(e[3], v[1] + v[2]);
			continue;
		}
		if (strncmp(buf, "line ", 5) && strncmp(buf, "poly ", 5)) {
			continue;
		}
		if (buf[0] == 'l') {
			r = strtod(s, &end) / 2;
			s = end;
		}
		for (;; k++) {
			double x = strtod(s, &end);

			if (end == s) {
				break;
			}
			e[k % 2] = fmin(e[k % 2], x - r);
			e[k % 2 + 2] = fmax(e[k % 2 + 2], x + r);
			s = end;
		}
	}
	fclose(f);
	free(buf);
}

// A text record of a sheet, kept for the links of the flag after it.
struct span {
	char t[64];
	double size, x, y;
};

// Writes a link of a box to page n of the PDF, as pdfmark: x1 is the
// PostScript that puts the box's right on the stack.
static void link(double x0, double y0, const char *x1, double y1, int n)
{
	printf("[ /Rect [ %.3f %.3f %s %.3f ] /Border [ 0 0 0 ] /Page %d "
	       "/View [ /Fit ] /Subtype /Link /ANN pdfmark\n",
	       x0, y0, x1, y1, n);
}

// The links of a port of the sheet on page k, as schview follows its
// clicks: its circle and name to the first page it lists other than k,
// each number of its list but k to that page; its spans are the text
// records before its flag record (vec(5)), the right of each measured
// by the font.
static void port_links(const struct span *sp, int n, double x, double y,
		       int k)
{
	int first = 0;
	char right[1024], t[160];

	for (int i = 1; i < n && !first; i++) {
		if (sp[i].t[0] >= '0' && sp[i].t[0] <= '9' &&
		    atoi(sp[i].t) != k) {
			first = atoi(sp[i].t);
		}
	}
	for (int i = 0; i < n; i++) {
		int to = !i ? first :
			 sp[i].t[0] >= '0' && sp[i].t[0] <= '9' &&
					 atoi(sp[i].t) != k ?
				    atoi(sp[i].t) :
				    0;

		size_t m = 0;

		if (!to) {
			continue;
		}
		// The text as a PostScript string, its brackets escaped.
		for (const char *c = sp[i].t; *c && m + 2 < sizeof t; c++) {
			if (*c == '(' || *c == ')' || *c == '\\') {
				t[m++] = '\\';
			}
			t[m++] = *c;
		}
		t[m] = 0;
		snprintf(right, sizeof right,
			 "/VecR findfont %.3f scalefont setfont (%.159s) "
			 "stringwidth pop %.3f add",
			 sp[i].size, t, sp[i].x);
		link(sp[i].x, sp[i].y - 0.25 * sp[i].size, right,
		     sp[i].y + 0.75 * sp[i].size, to);
	}
	if (first && n) {
		char cx[64];
		double r = sp[0].size / 4;

		snprintf(cx, sizeof cx, "%.3f", x + r);
		link(x - r, y - r, cx, y + r, first);
	}
}

// Draws a file in the color, filled or outlined: its page k (from 1),
// or all of it for k 0.
static void draw(const char *path, const double rgb[3], int outline, int names,
		 int k, int links)
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE);
	int now = 1, seen = 0, nsp = 0;
	struct span sp[32];

	if (!f || !buf) {
		die(path, "cannot read");
	}
	printf("%.3f %.3f %.3f setrgbcolor\n", rgb[0], rgb[1], rgb[2]);
	while (fgets(buf, LINE, f)) {
		char *s = buf + strcspn(buf, " \n"), *end;
		char net[512], kind[16];
		double fx, fy;
		int sat = 0;

		if (!strncmp(buf, "page ", 5) && seen++) {
			now++;
		}
		if (k && now != k) {
			continue;
		}
		// The texts before a flag of a port are its spans.
		if (k && sscanf(buf, "text %*c %lf %*f %*d %lf %lf %n",
				&sp[nsp < 32 ? nsp : 31].size,
				&sp[nsp < 32 ? nsp : 31].x,
				&sp[nsp < 32 ? nsp : 31].y, &sat) == 3 &&
		    sat) {
			struct span *q = &sp[nsp < 32 ? nsp : 31];

			buf[strcspn(buf, "\n")] = 0;
			copy_prefix(q->t, sizeof q->t, buf + sat,
				    strlen(buf + sat));
			nsp += nsp < 32;
		} else if (k && sscanf(buf, "flag %511s %lf %lf %15s", net,
				       &fx, &fy, kind) == 4) {
			if (!strcmp(kind, "port") && links) {
				port_links(sp, nsp, fx, fy, k);
			}
			nsp = 0;
		} else {
			nsp = 0;
		}
		if (!strncmp(buf, "dark", 4)) {
			printf("%.3f %.3f %.3f setrgbcolor\n", rgb[0], rgb[1],
			       rgb[2]);
		} else if (!strncmp(buf, "clear", 5)) {
			if (!outline) {
				printf("1 setgray\n");
			}
		} else if (!strncmp(buf, "disk", 4)) {
			double v[3] = { 0, 0, 0 };

			for (int n = 0; n < 3; n++) {
				v[n] = strtod(s, &end);
				s = end;
			}
			printf("newpath %.4f %.4f %.4f 0 360 arc closepath %s\n",
			       v[0], v[1], v[2], outline ? "stroke" : "fill");
		} else if (!strncmp(buf, "line", 4)) {
			double w = strtod(s, &end);
			int first = 1;

			s = end;
			printf("%.4f setlinewidth newpath", outline ? 0.05 : w);
			for (;;) {
				double x = strtod(s, &end), y;

				if (end == s) {
					break;
				}
				s = end;
				y = strtod(s, &end);
				s = end;
				printf(" %.4f %.4f %s", x, y,
				       first ? "moveto" : "lineto");
				first = 0;
			}
			printf(" stroke\n");
		} else if (!strncmp(buf, "poly", 4)) {
			int first = 1;

			printf("newpath");
			for (;;) {
				double x = strtod(s, &end), y;

				if (end == s) {
					break;
				}
				s = end;
				y = strtod(s, &end);
				s = end;
				printf(" %.4f %.4f %s", x, y,
				       first ? "moveto" : "lineto");
				first = 0;
			}
			printf(" closepath %s\n", outline ? "stroke" : "fill");
		} else if (!strncmp(buf, "text ", 5)) {
			char font;
			double size, angle, x, y;
			int align, at = 0;

			if (sscanf(buf, "text %c %lf %lf %d %lf %lf %n", &font,
				   &size, &angle, &align, &x, &y, &at) == 6 &&
			    at > 0) {
				buf[strcspn(buf, "\n")] = 0;
				printf("/%s findfont %.3f scalefont setfont "
				       "%.3f %.3f moveto gsave %.3f rotate ",
				       font == 'I' ? "VecI" : "VecR", size, x,
				       y, angle);
				ps_string(buf + at);
				printf(" dup stringwidth pop %d mul -2 div 0 "
				       "rmoveto show grestore\n",
				       align);
			}
		} else if (!strncmp(buf, "name", 4) && names) {
			char name[512];
			double b[4];

			if (sscanf(buf, "name %511s %lf %lf %lf %lf", name,
				   &b[0], &b[1], &b[2], &b[3]) == 5 &&
			    b[2] - b[0] < 50) {
				printf("gsave 0 setgray (%s) dup stringwidth pop 2 div neg "
				       "%.4f add %.4f moveto show grestore\n",
				       name, (b[0] + b[2]) / 2,
				       (b[1] + b[3]) / 2);
			}
		}
	}
	fclose(f);
	free(buf);
}

// The parts the files name: by name, their designators and boxes.
struct part {
	char name[512], ref[64];
	double b[4];
	int boxed;
};

static struct part *parts;
static size_t nparts;

static struct part *part_of(const char *name)
{
	for (size_t i = 0; i < nparts; i++) {
		if (!strcmp(parts[i].name, name)) {
			return &parts[i];
		}
	}
	parts = realloc(parts, (nparts + 1) * sizeof *parts);
	if (!parts) {
		die("vecps", "out of memory");
	}
	memset(&parts[nparts], 0, sizeof *parts);
	copy_prefix(parts[nparts].name, sizeof parts[0].name, name,
		    strlen(name));
	parts[nparts].b[0] = parts[nparts].b[1] = 1e300;
	parts[nparts].b[2] = parts[nparts].b[3] = -1e300;
	return &parts[nparts++];
}

// Reads the parts of a file: their brackets (part NAME REF) and boxes.
static void read_parts(const char *path)
{
	FILE *f = fopen(path, "r");
	char buf[1200], name[512], ref[64];
	double b[4];

	if (!f) {
		die(path, "cannot read");
	}
	while (fgets(buf, sizeof buf, f)) {
		if (sscanf(buf, "part %511s %63s", name, ref) == 2) {
			copy_prefix(part_of(name)->ref, sizeof ref, ref,
				    strlen(ref));
		} else if (sscanf(buf, "name %511s %lf %lf %lf %lf", name,
				  &b[0], &b[1], &b[2], &b[3]) == 5) {
			struct part *p = part_of(name);

			for (int k = 0; k < 4; k++) {
				p->b[k] = k < 2 ? fmin(p->b[k], b[k]) :
						  fmax(p->b[k], b[k]);
			}
			p->boxed = 1;
		}
	}
	fclose(f);
}

// Writes a label: text of the size (in the units of the files), its
// middle at (x, y), turned a degrees, upright.
static void label(const char *text, double x, double y, double a, double size,
		  double gray)
{
	a = fmod(a + 360, 360);
	if (a > 90 && a <= 270) {
		a -= 180; // Upright.
	}
	printf("gsave %.3f setgray /VecR findfont %.4f scalefont setfont "
	       "%.4f %.4f translate %.3f rotate 0 %.4f moveto ",
	       gray, size, x, y, a, -size * 0.35);
	ps_string(text);
	printf(" dup stringwidth pop -2 div 0 rmoveto show grestore\n");
}

// The number of characters of a text, as a length.
static double chars(const char *s)
{
	size_t n = strlen(s);

	return (double)n;
}

static int inside(const double w[4], double x, double y)
{
	return x >= w[0] && x <= w[2] && y >= w[1] && y <= w[3];
}

// The size of a label on the page: fitted to what it names (fit, in the
// units of the files), but no smaller than 3 points and no larger than
// 7, so it is read on the page.
static double fitted(double fit, double scale)
{
	return fmin(fmax(fit * scale, 3), 7) / scale;
}

// Names what lies in the window: each part's designator in its box, and
// each net's name along its tracks and in its pads.
static void names_in(const char *path, const double w[4], double scale)
{
	FILE *f = fopen(path, "r");
	char *buf = malloc(LINE), net[512] = "";
	int in_part = 0;

	if (!f || !buf) {
		die(path, "cannot read");
	}
	while (fgets(buf, LINE, f)) {
		double v[4096];
		int n = 0;
		char *s = buf + strcspn(buf, " \n"), *end;

		buf[strcspn(buf, "\n")] = 0;
		if (!strncmp(buf, "net", 3) && (!buf[3] || buf[3] == ' ')) {
			copy_prefix(net, sizeof net, buf + 4 * !!buf[3],
				    buf[3] ? strlen(buf + 4) : 0);
			continue;
		}
		if (!strncmp(buf, "part", 4) && (!buf[4] || buf[4] == ' ')) {
			in_part = buf[4] == ' ';
			continue;
		}
		if (!*net ||
		    (strncmp(buf, "line ", 5) && strncmp(buf, "poly ", 5) &&
		     strncmp(buf, "disk ", 5))) {
			continue;
		}
		while (n < 4096 && (v[n] = strtod(s, &end), end != s)) {
			s = end;
			n++;
		}
		if (!strncmp(buf, "line ", 5) && n >= 5 && !in_part) {
			// A track: along its longest segment, in its middle.
			int best = 1;
			double len = -1;

			for (int k = 1; k + 3 < n; k += 2) {
				double l = hypot(v[k + 2] - v[k],
						 v[k + 3] - v[k + 1]);

				if (l > len) {
					len = l;
					best = k;
				}
			}
			{
				double x = (v[best] + v[best + 2]) / 2;
				double y = (v[best + 1] + v[best + 3]) / 2;
				double a = atan2(v[best + 3] - v[best + 1],
						 v[best + 2] - v[best]) *
					   180 / 3.14159265358979323846;
				double size = fitted(v[0] * 0.7, scale);

				if (inside(w, x, y) &&
				    len > size * 0.6 * chars(net)) {
					label(net, x, y, a, size, 0.15);
				}
			}
		} else if (in_part && (n >= 6 || (n == 3 && buf[0] == 'd'))) {
			// A pad: in its middle, along its longer side.
			double b[4] = { 1e300, 1e300, -1e300, -1e300 };

			if (n == 3) {
				b[0] = v[0] - v[2];
				b[1] = v[1] - v[2];
				b[2] = v[0] + v[2];
				b[3] = v[1] + v[2];
			}
			for (int k = 0; n != 3 && k + 1 < n; k += 2) {
				b[0] = fmin(b[0], v[k]);
				b[1] = fmin(b[1], v[k + 1]);
				b[2] = fmax(b[2], v[k]);
				b[3] = fmax(b[3], v[k + 1]);
			}
			if (inside(w, (b[0] + b[2]) / 2, (b[1] + b[3]) / 2)) {
				double bw = b[2] - b[0], bh = b[3] - b[1];
				double lo = fmin(bw, bh), hi = fmax(bw, bh);
				double size = fitted(
					fmin(lo * 0.6,
					     hi / (0.6 * chars(net) + 0.5)),
					scale);

				label(net, (b[0] + b[2]) / 2, (b[1] + b[3]) / 2,
				      bh > bw ? 90 : 0, size, 0.15);
			}
		}
	}
	fclose(f);
	free(buf);
}

static void usage(void)
{
	fprintf(stderr,
		"usage: vecps [-w X0 Y0 X1 Y1 | -r X0 Y0 X1 Y1 | -p PART]... "
		"[-n] FILE[=R,G,B[,line]]... > out.ps\n");
	exit(2);
}

int main(int argc, char **argv)
{
	double win[4], scale, pw = 792, ph = 612;
	int i = 1, have = 0, names = 0, page = 0, region = 0, a0;
	char **want = NULL;
	size_t nwant = 0;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if ((!strcmp(argv[i], "-w") || !strcmp(argv[i], "-r")) &&
		    i + 4 < argc) {
			region |= argv[i][1] == 'r';
			for (int k = 0; k < 4; k++) {
				win[k] = atof(argv[++i]);
			}
			have = 1;
		} else if (!strcmp(argv[i], "-p") && i + 1 < argc) {
			char *save = NULL;

			region = 1;
			for (char *t = strtok_r(argv[++i], ",", &save); t;
			     t = strtok_r(NULL, ",", &save)) {
				want = realloc(want,
					       (nwant + 1) * sizeof *want);
				if (!want) {
					die("vecps", "out of memory");
				}
				want[nwant++] = t;
			}
		} else if (!strcmp(argv[i], "-n")) {
			names = 1;
		} else {
			usage();
		}
	}
	if (i == argc) {
		usage();
	}
	a0 = i;
	for (int k = a0; region && k < argc; k++) {
		char path[4096];

		copy_prefix(path, sizeof path, argv[k], strcspn(argv[k], "="));
		read_parts(path);
	}
	if (nwant) {
		double b[4] = { 1e300, 1e300, -1e300, -1e300 }, g;

		for (size_t k = 0; k < nwant; k++) {
			int found = 0;

			for (size_t p = 0; p < nparts; p++) {
				if (parts[p].boxed &&
				    (!strcmp(parts[p].name, want[k]) ||
				     !strcmp(parts[p].ref, want[k]))) {
					for (int j = 0; j < 4; j++) {
						b[j] = j < 2 ? fmin(b[j],
								    parts[p].b[j]) :
							       fmax(b[j],
								    parts[p].b[j]);
					}
					found = 1;
				}
			}
			if (!found) {
				die(want[k], "no such part");
			}
		}
		g = fmax(b[2] - b[0], b[3] - b[1]) / 5 + 1;
		win[0] = b[0] - g;
		win[1] = b[1] - g;
		win[2] = b[2] + g;
		win[3] = b[3] + g;
		have = 1;
	}
	if (!have) {
		char path[4096];

		copy_prefix(path, sizeof path, argv[i], strcspn(argv[i], "="));
		page = page_of(path, 1, &pw, &ph);
		win[0] = win[1] = 1e300;
		win[2] = win[3] = -1e300;
		for (int k = i; k < argc; k++) {
			copy_prefix(path, sizeof path, argv[k],
				    strcspn(argv[k], "="));
			extent(path, win);
		}
	}
	printf("%%!PS-Adobe-3.0\n%%%%BoundingBox: 0 0 %.0f %.0f\n%%%%Pages: %d\n"
	       "%%%%EndComments\n",
	       pw, ph, page ? page : 1);
	printf("/VecFont { findfont dup length dict begin\n"
	       "  { 1 index /FID ne { def } { pop pop } ifelse } forall\n"
	       "  /Encoding ISOLatin1Encoding 256 array copy dup 45 /hyphen "
	       "put def currentdict end definefont "
	       "pop } def\n"
	       "/VecR /Helvetica VecFont /VecI /Helvetica-Oblique VecFont\n"
	       "/pdfmark where { pop } { userdict /pdfmark /cleartomark load "
	       "put } ifelse\n");
	// A sheet is drawn page by page, each at its size and scale 1.
	for (int pg = 1; pg <= (page ? page : 1); pg++) {
		if (page) {
			char path[4096];

			copy_prefix(path, sizeof path, argv[a0],
				    strcspn(argv[a0], "="));
			page_of(path, pg, &pw, &ph);
			win[0] = win[1] = 0;
			win[2] = pw;
			win[3] = ph;
			scale = 1;
		} else {
			scale = fmin((pw - 36) / (win[2] - win[0]),
				     (ph - 36) / (win[3] - win[1]));
		}
		printf("%%%%Page: %d %d\n", pg, pg);
		printf("<< /PageSize [ %.0f %.0f ] >> setpagedevice\n", pw, ph);
		printf("1 setlinecap 1 setlinejoin\n");
		if (!page) {
			printf("newpath 18 18 %.0f %.0f rectclip\n", pw - 36,
			       ph - 36);
		}
		printf("%.4f %.4f translate %.6f dup scale %.4f %.4f "
		       "translate\n",
		       pw / 2, ph / 2, scale, -(win[0] + win[2]) / 2,
		       -(win[1] + win[3]) / 2);
		printf("/Helvetica findfont %.4f scalefont setfont\n",
		       7 / scale);
		for (int k = a0; k < argc; k++) {
			char path[4096], *eq = strchr(argv[k], '=');
			double rgb[3] = { 0, 0, 0 };
			int outline = 0;

			copy_prefix(path, sizeof path, argv[k],
				    eq ? (size_t)(eq - argv[k]) :
					 strlen(argv[k]));
			if (eq && sscanf(eq + 1, "%lf,%lf,%lf", &rgb[0],
					 &rgb[1], &rgb[2]) != 3) {
				die(argv[k], "FILE[=R,G,B[,line]]");
			}
			outline = eq && strstr(eq, ",line") != NULL;
			draw(path, rgb, outline, names && k == a0,
			     page ? pg : 0, k == a0);
		}
		for (int k = a0; region && k < argc; k++) {
			char path[4096];

			copy_prefix(path, sizeof path, argv[k],
				    strcspn(argv[k], "="));
			names_in(path, win, scale);
		}
		for (size_t p = 0; region && p < nparts; p++) {
			const struct part *q = &parts[p];
			double x = (q->b[0] + q->b[2]) / 2;
			double y = (q->b[1] + q->b[3]) / 2;

			if (q->boxed && *q->ref && inside(win, x, y)) {
				double bw = q->b[2] - q->b[0];
				double bh = q->b[3] - q->b[1];
				double size = fitted(
					fmin(fmin(bw, bh) * 0.3,
					     fmax(bw, bh) /
						     (0.7 * chars(q->ref) + 1)),
					scale);

				label(q->ref, x, y, bh > 1.5 * bw ? 90 : 0,
				      size, 0);
			}
		}
		printf("showpage\n");
	}
	printf("%%%%EOF\n");
	return 0;
}
