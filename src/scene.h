// SPDX-License-Identifier: GPL-3.0-only
// scene.h -- The parts, the camera and the commands of view(1)
// Copyright (c) 2026 Jakob Kastelic

// A scene file (scene(5), in the language of plicad(7)) names the
// meshes and pictures to show, one a line:
//   mesh NAME FILE.obj [color R G B] [PLACE] [path STEP... | with NAME]
//   image NAME FILE.ppm on MESH X0 Y0 Z0 X1 Y1 Z1 X2 Y2 Z2
//   unit NAME [PLACE] [path STEP... | with NAME]
//   end
//   view iso|front|back|left|right|top|bottom
//   background R G B        each 0 to 1; FreeCAD's gray by default
// where PLACE and each STEP is a move DX DY DZ or a turn x|y|z DEG
// [about X Y Z].  Each part of an OBJ file of more than one part is
// NAME/PART.  The color (each 0 to 1) overrides that of the file; the
// moves and turns of the place apply in their order, a turn about the
// origin unless it names a point, and those of a path about the origin
// of the part at its place.  A picture (PPM P6, or PAM P7 of RGB or
// RGB_ALPHA, as magick in.png out.pam writes it) fills the
// parallelogram of its corners X0 (lower left), X1 (lower right) and X2
// (upper left), in the frame of the mesh MESH of a line above, and moves
// with it.  Pictures show over the meshes by their opacity, the
// farthest first, and a pixel of no opacity is not there for a pick.
// The lines between unit and end, a subassembly, lie in the frame of the
// unit at its place, and their names under its name: a mesh NAME is
// UNIT/NAME, and so are the names of with and on.  Units nest.  Files
// lie relative to the scene file.
//
// The commands, from stdin:
//   hide NAME, show NAME, only NAME, all
//   view NAME, fit, turn DX DY (degrees), zoom FACTOR
//   pick X Y        prints the name of the part at the pixel, or -
//   select [NAME]   picks the part NAME, or the first part of the unit
//                   NAME, as a click does, or none (scene_select_name)
//   center          centers the view on the picked part, as the key c
//                   (scene_center)
//   shot FILE.ppm   writes the picture
//   list            prints the parts, their triangles, whether shown,
//                   whether see-through and the center of the box of each
//                   where it stands
//   play K          puts the parts where K motions of the sequence have
//                   them, 0 to the number of motions, a fraction part of
//                   the way along the next path (moves.c)
//   move NAME DX DY DZ
//                   moves a part or a unit on by DX DY DZ, as a drag
//   key h|o|s X Y   hides the part at the pixel, shows it alone, or
//                   turns it see-through or back
//   key H|O|S X Y   the same for its unit, level levels up
//   level K         sets level, 1 at least
//   label X Y       prints the name of the part at the pixel with its
//                   unit in brackets, or -
//   see NAME, solid NAME
//                   shows the parts see-through, or solid again
//   home            sets every part home; reset also shows all, as the
//                   space key; undo, redo walk the history of what shows
//                   and of the moves
//   spin N          draws N frames of a turn about the z axis
//   help            prints the help of the key ? (view_help)
//   quit
// A NAME names a part, or every part of a unit: a mesh of several
// parts, NAME/PART, or a subassembly.

#ifndef SCENE_H
#define SCENE_H

#include "bvh.h"
#include "geom.h"
#include "obj.h"
#include <stddef.h>

// A line of the scene file: its kind, name and file, its color if it
// gives one, its place (rows of a turn, then a move), the corners of a
// picture, and the line of the scene file.
// A step of an insertion path: a move by d, or a turn by deg degrees
// about the line through at along ax, in the world.
struct pstep {
	int turn, about;
	struct vec d, at, ax;
	double deg;
};

// An item may be a unit, of no file, whose items name it as up.
struct item {
	int image, unit;
	size_t up; // The unit the item lies in, or NONE.
	char *name, *path;
	int has_rgb;
	float rgb[3];
	struct rigid m;
	int moved;
	struct vec corner[3];
	size_t line;
	struct pstep *steps; // The insertion path, ending at the place m.
	size_t nsteps, capsteps;
	char *with; // The item it travels with, or NULL.
	size_t on; // The mesh a picture lies on, or NONE.
};

// A part: its name and item, its color, whether it shows, its
// triangles in the world (nine floats each in pos, their normals in nrm),
// its box, and for a picture its pixels and the texture coordinates of
// its corners.
struct part {
	char *name;
	size_t item;
	float rgb[3];
	int shown;
	float *pos, *nrm, *uv;
	float *col; // The color of each triangle (its coats), or NULL.
	float *grade; // The VDI grade of the texture of each triangle, below
		// 0 for none, or NULL for none at all.
	size_t nt;
	struct box box;
	unsigned char *pix; // Four bytes a pixel, the last its opacity.
	int pw, ph;
	int clear; // Whether some pixel of the picture lets light through.
	unsigned gl[4]; // The buffers or texture of view.c, or 0.
	size_t plain; // Of view.c: the triangles first in its buffers, untextured.
	struct rigid mot; // Its motion from its place to where it stands.
	struct bvh bvh; // The tree of its triangles, for picks.
	int see; // Whether the part shows see-through, at SEE_ALPHA.
};

#define SEE_ALPHA 0.5F // The opacity of a see-through part.

// A state of the history of undo: what shows (bit 0) and see-through
// (bit 1), the motions of the sequence done and the move of each item.
struct state {
	unsigned char *shown;
	struct vec *drag;
	double play;
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
	float back[3]; // The color of the background.
	int fitted;
	size_t *seq; // The items that move, in the order they go in,
	size_t nseq, capseq; // as assemble(1) has them,
	double play; // and how many have, the rest at their starts.
	struct vec *drag; // The move of each item from its home, in mm.
	int level; // The units up that H, O, S and a drag act on, 1 or more.
	size_t picked; // The picked part, or NONE.
	int centered; // Whether center centered the view, with no change of
		// the view or the pick since.
	int touched; // Whether the view was set since the scene was read.
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
// With one part alone in view, o shows all the parts again, wherever the
// pointer is, their see-through and their moves as they are.  The keys
// H, O and S do the same for the unit of the part, s->level units up,
// and O shows all again when that unit alone is in view.
int scene_show_at(struct scene *s, int key, struct px at);
// The name of the unit of part i, s->level units up: its name less as
// many of its last parts, but not its first; a new string.
char *scene_unit(const struct scene *s, size_t i);
// The name of part i with the last part of its unit in brackets, as
// front/[knob]/cap; a new string.
char *scene_label(const struct scene *s, size_t i);
// Whether part or item name lies in unit, or is it.
int scene_in_unit(const char *unit, const char *name);

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
// The sequence and the moves (moves.c).  The sequence holds the items
// of a path or none, in the order assemble(1) puts them in: the items of
// a unit, then the unit; an item that travels with another moves with
// it.  An item not yet in stands at the start of its path, one going in
// along it, and the rest at their places; each part then moves on by
// the move of its item.  A picture moves with the mesh or the unit it
// lies on.
//
// Sets the motions of the parts by the sequence and the moves.
void scene_place(struct scene *s);
// Picks part i, or none for NONE.
void scene_select(struct scene *s, size_t i);
// Picks the part of the name, or the first part of the unit of the name,
// or none for NULL or a name of none.
void scene_select_name(struct scene *s, const char *name);
// Turns the view about, and centers it on, the middle of the box of the
// picked part where it stands, at the same zoom and direction, or, if it
// did so last with no change of the view or the pick since, zooms in
// two times about it; does nothing with no part picked.
void scene_center(struct scene *s);
// Notes that the view changed by other than scene_center.
void scene_cam_moved(struct scene *s);
// Sets the size of the view to sz, and fits all to it if nothing set the
// view since the scene was read: so the view opens fitted to the window
// that the window manager gives.
void scene_resize(struct scene *s, struct size sz);
// Builds the sequence of the items, all of it done.
void scene_sequence(struct scene *s);
// Sets the motions of the sequence done to k, a fraction part of the
// way along the next path, and the motions of the parts.
void scene_play(struct scene *s, double k);
// Sets every part home: the whole sequence done, no drags.
void scene_home(struct scene *s);
// Shows every part, solid, and sets every part home.
void scene_reset(struct scene *s);
// Moves the items of the part or unit name on by d; returns how many.
size_t scene_move(struct scene *s, const char *name, struct vec d);
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
// the upper left, behind the eye.  A face takes AMBIENT of its color,
// and the rest by the light on it.
#define LIGHT_X (-0.3)
#define LIGHT_Y 0.5
#define LIGHT_Z 0.8
#define AMBIENT 0.3F

#endif
