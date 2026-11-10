// SPDX-License-Identifier: GPL-3.0-only
// geom.h -- 3D vectors, rigid motions and boxes
// Copyright (c) 2026 Jakob Kastelic

#ifndef GEOM_H
#define GEOM_H

#define PI 3.14159265358979323846
#define DEG (PI / 180) // One degree, in radians.
#define RIGHT 90.0 // A right angle, in degrees,
#define HALF_TURN 180.0 // and half a turn.
#define MID 0.5 // The middle of an interval, as a share of it.
#define INCH 25.4 // One inch, in mm.

enum { BOX_CORNERS = 8 };

struct vec {
	double x, y, z;
};

// The interval from lo to hi.
struct span {
	double lo, hi;
};

// The line through o along d.
struct line3 {
	struct vec o, d;
};

// p' = r p + t
struct rigid {
	double r[3][3];
	struct vec t;
};

struct vec v3(double x, double y, double z);
struct vec add(struct vec a, struct vec b);
struct vec sub(struct vec a, struct vec b);
struct vec mul(struct vec a, double s);
double dot(struct vec a, struct vec b);
struct vec cross(struct vec a, struct vec b);
double len(struct vec a);
struct vec unit(struct vec a);
double comp(struct vec v, int axis);
struct vec axis_vec(int axis, double sign);

struct rigid rig_id(void);
// Returns a after b.
struct rigid rig_mul(struct rigid a, struct rigid b);
struct rigid rig_inv(struct rigid a);
struct vec rig_pt(struct rigid a, struct vec p);
struct vec rig_dir(struct rigid a, struct vec d);
// Returns the rotation by deg degrees about the line ax.
struct rigid rig_rot(struct line3 ax, double deg);

// Returns the length of the neutral line in a bend of deg degrees with
// inside radius r and thickness t.  The neutral line lies at r + k t, with
// the ANSI k-factor of the FreeCAD SheetMetal workbench.
double bend_allowance(double deg, double r, double t, double k);

// An axis-aligned box; an empty box has lo above hi.
struct box {
	struct vec lo, hi;
};

struct box box_empty(void);
void box_add(struct box *b, struct vec p);
// Corner k of b, for k from 0 to 7: bit 0 picks hi.x, bit 1 hi.y and
// bit 2 hi.z.
struct vec box_corner(struct box b, unsigned k);
void box_pad(struct box *b, double d);
// Is p in b, edges included?
int box_has(struct box b, struct vec p);
// Do a and b share a volume?
int box_meet(struct box a, struct box b);

#endif
