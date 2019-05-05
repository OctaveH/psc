#ifndef WALL_INFO_H_INCLUDED
#define WALL_INFO_H_INCLUDED

/*
* Climbing wall parameters definition
*/

#define HEIGHT 3.0
#define WIDTH 2.0
#define THICKNESS 0.5
#define POSITION_X -1.0-0.25
#define POSITION_Y 0.0
#define POSITION_Z 1.51
#define ORIENTATION_X 1.0
#define ORIENTATION_Y 0.0
#define ORIENTATION_Z 0.0

#define NB_HOLDS 5

#define X1 -0.95
#define Y1 0.0
#define Z1 0.2

#define X2 -0.95
#define Y2 0.3
#define Z2 0.3

#define X3 -0.95
#define Y3 0.5
#define Z3 1.2

#define X4 -0.95
#define Y4 -0.2
#define Z4 1.7

#define X5 -0.95
#define Y5 0.9
#define Z5 2.5

static const vec3 _holds[] = {
{X1, Y1, Z1},
{X2, Y2, Z2},
{X3, Y3, Z3},
{X4, Y4, Z4},
{X5, Y5, Z5}
};




#endif // WALL_INFO_H_INCLUDED
