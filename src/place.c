// SPDX-License-Identifier: GPL-3.0-only
// place.c -- Place the parts of an assembly
// Copyright (c) 2026 Jakob Kastelic

// Usage: place assembly.txt > assembly.st
//        place -s MESHDIR assembly.txt > assembly.scene
//
// Reads an assembly file (asm.h, assembly(5)) and writes its parts in
// place, as a stream (stream.h): first the materials they use, once
// each, then the parts.  With -s it writes instead the scene of their
// meshes for view(1) (scene.h): a mesh line for each part line, of the
// file MESHDIR/NAME.obj, with the color of its material, the steps of
// its place, and its insertion path, every turn with its point, or its
// with; a subassembly as a unit, its lines indented in its own frame
// between unit and end, its meshes under MESHDIR/NAME; and an image line
// for each picture.

#include "asm.h"
#include "fold.h"
#include "geom.h"
#include "stream.h"
#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Prints the steps st, the numbers worked out, a turn about at if it
// names no point.
static void put_steps(const struct asm_steps *st, struct vec at)
{
	for (size_t i = 0; i < st->n; i++) {
		const struct asm_step *s = &st->s[i];

		if (!s->turn) {
			printf(" move %.9g %.9g %.9g", s->d.x + 0.0,
			       s->d.y + 0.0, s->d.z + 0.0);
			continue;
		}
		{
			struct vec p = s->about ? s->at : at;

			printf(" turn %c %.9g about %.9g %.9g %.9g",
			       'x' + s->axis, s->deg, p.x + 0.0, p.y + 0.0,
			       p.z + 0.0);
		}
	}
}

// Prints line l of the scene.
static void scene_line(const struct asm_line *l)
{
	for (int i = 0; i < l->depth; i++) {
		(void)putchar('\t');
	}
	if (l->kind == ASM_END) {
		printf("end\n");
		return;
	}
	if (l->kind == ASM_IMAGE) {
		printf("image %s %s on %s", l->name, l->file, l->on);
		for (size_t i = 0; i < ASM_CORNERS; i++) {
			printf(" %.9g", l->corner[i]);
		}
		printf("\n");
		return;
	}
	if (l->kind == ASM_UNIT) {
		printf("unit %s", l->name);
	} else {
		printf("mesh %s %s", l->name, l->mesh);
	}
	if (l->has_rgb) {
		printf(" color %.4g %.4g %.4g", l->rgb[0], l->rgb[1],
		       l->rgb[2]);
	}
	put_steps(&l->place, v3(0, 0, 0));
	if (l->with) {
		printf(" with %s", l->with);
	} else if (l->has_path) {
		printf(" path");
		put_steps(&l->path, l->m.t);
	}
	printf("\n");
}

// The first material of the name of material i of part k of a, among
// those of the parts before it and its own before it, or NULL.
static const struct material *named_before(const struct assembly *a, size_t k,
					   size_t i)
{
	const char *name = a->out[k].mats[i].name;

	for (size_t j = 0; j <= k; j++) {
		size_t n = j < k ? a->out[j].nmats : i;

		for (size_t q = 0; q < n; q++) {
			if (strcmp(a->out[j].mats[q].name, name) == 0) {
				return &a->out[j].mats[q];
			}
		}
	}
	return NULL;
}

// Writes the stream of the parts in place: the materials they use, once
// each, and the parts.  One name of two materials, of a subassembly and
// another, is refused.
static void write_out(const struct assembly *a, const char *path)
{
	stream_head();
	for (size_t k = 0; k < a->nout; k++) {
		for (size_t i = 0; i < a->out[k].nmats; i++) {
			const struct material *m = &a->out[k].mats[i];
			const struct material *e = named_before(a, k, i);

			if (!e) {
				write_material(m);
			} else if (materials_differ(e, m)) {
				die("%s: two materials of the name %s", path,
				    m->name);
			}
		}
	}
	for (size_t k = 0; k < a->nout; k++) {
		write_part(&a->out[k]);
	}
}

int main(int argc, char **argv)
{
	struct assembly a;
	const char *meshdir = NULL;

	if (argc == 4 && !strcmp(argv[1], "-s")) {
		meshdir = argv[2];
	} else if (argc != 2) {
		die("usage: place [-s MESHDIR] assembly.txt");
	}
	{
		struct asm_file f = { argv[argc - 1], meshdir };

		asm_read(&a, f);
	}
	if (meshdir) {
		printf("# The scene of %s, by place -s\n", argv[argc - 1]);
		for (size_t i = 0; i < a.nln; i++) {
			scene_line(&a.ln[i]);
		}
	} else {
		write_out(&a, argv[argc - 1]);
	}
	asm_free(&a);
	close_stdout();
	return 0;
}
