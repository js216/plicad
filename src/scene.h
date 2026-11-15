// SPDX-License-Identifier: GPL-3.0-only
// scene.h -- The parts, the camera and the commands of view(1)
// Copyright (c) 2026 Jakob Kastelic

// A scene file names the meshes and pictures to show, one a line:
//   mesh NAME FILE.obj [colour R G B] [move X Y Z] [turn X Y Z DEG]...
//   image NAME FILE.ppm X0 Y0 Z0 X1 Y1 Z1 X2 Y2 Z2
//   view iso|front|back|left|right|top|bottom
//   background R G B        each 0 to 1; FreeCAD's grey by default
// Each part of an OBJ file of more than one part is NAME/PART.  The
// colour (each 0 to 1) overrides that of the file; the moves and turns
// (about the axis X Y Z through the origin) apply in their order.  A
// picture (PPM P6, or PAM P7 of RGB or RGB_ALPHA, as magick in.png
// out.pam writes it) fills the parallelogram of its corners X0 (lower
// left), X1 (lower right) and X2 (upper left).  Pictures show over the
// meshes by their opacity, the farthest first, and a pixel of no
// opacity is not there for a pick.  Files lie relative to the scene
// file, and # starts a comment.
//
// The commands, from stdin:
//   hide PATTERN, show PATTERN, only PATTERN, all
//   view NAME, fit, turn DX DY (degrees), zoom FACTOR, size W H
//   pick X Y        prints the name of the part at the pixel, or -
//   shot FILE.ppm   writes the picture
//   list            prints the parts, their triangles, whether shown,
//                   whether see-through and the centre of the box of each
//                   where it stands
//   explode F       moves the parts out by F (0 home), drag NAME D drags
//                   an item, or that of a part, D mm out (explode.c)
//   key h|o|s X Y   hides the part at the pixel, shows it alone, or
//                   turns it see-through or back
//   see PATTERN, solid PATTERN
//                   shows the parts see-through, or solid again
//   home            sets every part home; reset also shows all, as the
//                   space key; undo, redo walk the history of what shows
//                   and of the moves
//   spin N          draws N frames of a turn about the z axis
//   help            prints the help of the key ? (view_help)
//   quit
// A pattern matches names with * for any run of characters.

#ifndef SCENE_H
#define SCENE_H

#include "geom.h"
#include "obj.h"
#include <stddef.h>

// A line of the scene file: its kind, name and file, its colour if it
// gives one, its place (rows of a turn, then a move), the corners of a
// picture, and the line of the scene file.
struct item {
	int image;
	char *name, *path;
	int has_rgb;
	float rgb[3];
	struct rigid m;
	int moved;
	struct vec corner[3];
	size_t line;
};

// A part: its name and item, its colour, whether it shows, its
// triangles in the world (nine floats each in pos, their normals in nrm),
// its box, and for a picture its pixels and the texture coordinates of
// its corners.
struct part {
	char *name;
	size_t item;
	float rgb[3];
	int shown;
	float *pos, *nrm, *uv;
	size_t nt;
	struct box box;
	unsigned char *pix; // Four bytes a pixel, the last its opacity.
	int pw, ph;
	int clear; // Whether some pixel of the picture lets light through.
	unsigned gl[2]; // The buffers or texture of view.c, or 0.
	struct vec off; // The move of the part out of its home (explode.c).
	int see; // Whether the part shows see-through, at SEE_ALPHA.
};

#define SEE_ALPHA 0.5F // The opacity of a see-through part.

// A state of the history of undo: what shows (bit 0) and see-through
// (bit 1), the factor of the explode
// (0 for none) and the drag of each item along its line out.
struct state {
	unsigned char *shown;
	double *drag;
	double f;
};

// A point of the window, in pixels from its top left corner.
struct px {
	double x, y;
};

// The size of a picture, in pixels.
struct size {
	int w, h;
};

// The camera: the point at the middle of the view, the rows of the turn
// from the world into the view (right, up, towards the eye), half the
// height of the view in mm, and the size of the window in pixels.
struct cam {
	struct vec c;
	struct vec r[3];
	double half;
	struct size sz;
};

struct scene {
	const char *path;
	struct item *it;
	size_t nit, capit;
	struct part *p;
	size_t np, capp;
	struct cam cam;
	char *view; // The view of the scene file, or NULL.
	float back[3]; // The colour of the background.
	int fitted;
	double f; // The factor of the explode.
	double *drag; // The drag of each item, in mm out from its home.
	struct state *hist; // The states, hist[at] the one now.
	size_t nhist, caphist, at;
};

// Reads the scene at path and the files it names, or returns -1 with
// the fault in e and s empty.
int scene_read(struct scene *s, const char *path, struct fault *e);
void scene_free(struct scene *s);
// Reads again the files of item i, or returns -1 with the fault in e
// and the item as it was.
int scene_reload(struct scene *s, size_t i, struct fault *e);

// Sets the camera to a standard view, or returns -1 for an unknown one.
int cam_view(struct cam *c, const char *name);
// Fits the camera to the parts that show.
void cam_fit(struct cam *c, const struct scene *s);
// Turns the camera about the world z axis by d.x and about its right by
// d.y, in degrees, round the middle of the view.
void cam_turn(struct cam *c, struct px d);
// The move in the world, on the plane of the view, of d pixels.
struct vec cam_shift(const struct cam *c, struct px d);
// Moves the view by d pixels.
void cam_pan(struct cam *c, struct px d);
// Zooms by f round the pixel at: a factor below 1 brings the parts
// nearer.
void cam_zoom(struct cam *c, double f, struct px at);
// The depth of the parts from the middle of the view, either way.
double cam_depth(const struct cam *c, const struct scene *s);

// The part under pixel at, or NONE.
size_t scene_pick(const struct scene *s, struct px at);
// Runs the key h, o or s on the part under pixel at: hides it, shows it
// alone, or turns it see-through or back; returns -1 if no part is there.
int scene_show_at(struct scene *s, int key, struct px at);

// The keys and the buttons of view, a line each, for the overlay of its
// key ? and its command help; NULL at the end.
extern const char *const view_help[];

// What a command asks of the program that draws.
// The corners of the two triangles of a picture.
enum { PIC_CORNERS = 6 };

#define CMD_DRAW 1U // Draw again.
#define CMD_SHOT 2U // Write the picture to the file of the command.
#define CMD_SPIN 4U // Draw the frames of a spin.
#define CMD_QUIT 8U
#define CMD_SIZE 16U // Take the size of the camera for the window.

// The result of a command: what it asks, the file of a shot and the
// frames of a spin.
struct cmd {
	unsigned ask;
	char *file;
	long frames;
};

// Runs the command of line on s, its answers on stdout and its faults
// on stderr; returns what it asks.
struct cmd scene_command(struct scene *s, char *line);

// Draws the scene into rgb, three bytes a pixel, the top row first, by
// the camera, with no display.
void scene_render(const struct scene *s, unsigned char *rgb);
// Explodes and drags (explode.c).  The centre of the scene is that of
// the box of all its parts at home; each part moves out from it, along
// the line to the centre of its own box, by f times their distance; each
// item moves on along the line from the centre to that of the box of its
// parts by its drag, from 0 (home) up.  An item at the centre does not
// drag, and a part at the centre stays.
//
// Sets the moves of the parts by the factor and the drags.
void scene_place(struct scene *s);
// Sets the factor of the explode to f, and the moves.
void scene_explode(struct scene *s, double f);
// Sets every part home: no explode, no drags.
void scene_home(struct scene *s);
// Shows every part, solid, and sets every part home.
void scene_reset(struct scene *s);
// Sets the drag of item i to d, 0 at least, and the moves.
void scene_drag(struct scene *s, size_t i, double d);
// The unit line out of item i, or 0 0 0 for an item at the centre.
struct vec scene_line_out(const struct scene *s, size_t i);
// Keeps the state now in the history, after those to undo, and drops
// those to redo; a state like the one now is not kept again.
void scene_commit(struct scene *s);
// Goes back, or forth, one state of the history; returns 0, or -1 if
// there is none.
int scene_undo(struct scene *s);
int scene_redo(struct scene *s);
// Forgets the history, keeping the state now as its only one.
void scene_history_reset(struct scene *s);
// The box of part p where it stands.
struct box part_box_at(const struct part *p);

// The parts that show by blending, the pictures and the see-through
// parts, in *out, the farthest first, and their number.
size_t scene_pictures(const struct scene *s, size_t **out);
// Writes the pixels rgb of size sz as a PPM file, or returns -1.
int write_ppm(const char *path, const unsigned char *rgb, struct size sz);
// The light falls along -(LIGHT_X, LIGHT_Y, LIGHT_Z) of the view: from
// the upper left, behind the eye.  A face takes AMBIENT of its colour,
// and the rest by the light on it.
#define LIGHT_X (-0.3)
#define LIGHT_Y 0.5
#define LIGHT_Z 0.8
#define AMBIENT 0.3F

#endif
