// SPDX-License-Identifier: GPL-3.0-only
// geom.c -- 3D vectors, rigid motions and boxes
// Copyright (c) 2026 Jakob Kastelic

#include "geom.h"
#include <math.h>

struct vec v3(double x, double y, double z)
{
	struct vec v = { x, y, z };

	return v;
}

struct vec add(struct vec a, struct vec b)
{
	return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

struct vec sub(struct vec a, struct vec b)
{
	return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

struct vec mul(struct vec a, double s)
{
	return v3(a.x * s, a.y * s, a.z * s);
}

double dot(struct vec a, struct vec b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

struct vec cross(struct vec a, struct vec b)
{
	return v3((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z),
		  (a.x * b.y) - (a.y * b.x));
}

double len(struct vec a)
{
	return sqrt(dot(a, a));
}

struct vec unit(struct vec a)
{
	return mul(a, 1 / len(a));
}

double comp(struct vec v, int axis)
{
	if (axis == 0) {
		return v.x;
	}
	if (axis == 1) {
		return v.y;
	}
	return v.z;
}

struct vec axis_vec(int axis, double sign)
{
	return v3(axis == 0 ? sign : 0, axis == 1 ? sign : 0,
		  axis == 2 ? sign : 0);
}

struct rigid rig_id(void)
{
	struct rigid r = { { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } },
			   { 0, 0, 0 } };

	return r;
}

struct vec rig_dir(struct rigid a, struct vec d)
{
	return v3((a.r[0][0] * d.x) + (a.r[0][1] * d.y) + (a.r[0][2] * d.z),
		  (a.r[1][0] * d.x) + (a.r[1][1] * d.y) + (a.r[1][2] * d.z),
		  (a.r[2][0] * d.x) + (a.r[2][1] * d.y) + (a.r[2][2] * d.z));
}

struct vec rig_pt(struct rigid a, struct vec p)
{
	return add(rig_dir(a, p), a.t);
}

struct rigid rig_mul(struct rigid a, struct rigid b)
{
	struct rigid c = rig_id();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			c.r[i][j] = a.r[i][0] * b.r[0][j] +
				    a.r[i][1] * b.r[1][j] +
				    a.r[i][2] * b.r[2][j];
		}
	}
	c.t = rig_pt(a, b.t);
	return c;
}

struct rigid rig_inv(struct rigid a)
{
	struct rigid c = rig_id();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			c.r[i][j] = a.r[j][i];
		}
	}
	c.t = mul(rig_dir(c, a.t), -1);
	return c;
}

struct rigid rig_rot(struct line3 ax, double deg)
{
	struct vec o = ax.o;
	struct vec k = unit(ax.d);
	double th = deg * DEG;
	double c = cos(th);
	double s = sin(th);
	double v = 1 - c;
	struct rigid r = rig_id();

	r.r[0][0] = c + k.x * k.x * v;
	r.r[0][1] = k.x * k.y * v - k.z * s;
	r.r[0][2] = k.x * k.z * v + k.y * s;
	r.r[1][0] = k.y * k.x * v + k.z * s;
	r.r[1][1] = c + k.y * k.y * v;
	r.r[1][2] = k.y * k.z * v - k.x * s;
	r.r[2][0] = k.z * k.x * v - k.y * s;
	r.r[2][1] = k.z * k.y * v + k.x * s;
	r.r[2][2] = c + k.z * k.z * v;
	r.t = sub(o, rig_dir(r, o));
	return r;
}

double bend_allowance(double deg, double r, double t, double k)
{
	return (r + k * t) * fabs(deg) * DEG;
}

struct vec box_corner(struct box b, unsigned k)
{
	return v3((k & 1U) ? b.hi.x : b.lo.x, (k & 2U) ? b.hi.y : b.lo.y,
		  (k & 4U) ? b.hi.z : b.lo.z);
}

struct box box_empty(void)
{
	struct box b = { { HUGE_VAL, HUGE_VAL, HUGE_VAL },
			 { -HUGE_VAL, -HUGE_VAL, -HUGE_VAL } };

	return b;
}

void box_add(struct box *b, struct vec p)
{
	b->lo = v3(fmin(b->lo.x, p.x), fmin(b->lo.y, p.y), fmin(b->lo.z, p.z));
	b->hi = v3(fmax(b->hi.x, p.x), fmax(b->hi.y, p.y), fmax(b->hi.z, p.z));
}

void box_pad(struct box *b, double d)
{
	b->lo = sub(b->lo, v3(d, d, d));
	b->hi = add(b->hi, v3(d, d, d));
}

int box_has(struct box b, struct vec p)
{
	return p.x >= b.lo.x && p.y >= b.lo.y && p.z >= b.lo.z &&
	       p.x <= b.hi.x && p.y <= b.hi.y && p.z <= b.hi.z;
}

int box_meet(struct box a, struct box b)
{
	return a.lo.x < b.hi.x && b.lo.x < a.hi.x && a.lo.y < b.hi.y &&
	       b.lo.y < a.hi.y && a.lo.z < b.hi.z && b.lo.z < a.hi.z;
}
