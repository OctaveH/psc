#ifndef WALL_H_INCLUDED
#define WALL_H_INCLUDED

#include <ode/ode.h>

#include "../include/util.h"

typedef struct {
  int lh_hold;
  int rh_hold;
  int lf_hold;
  int rf_hold;
} Stance;

/*
* ClimbingWall class
*/
class ClimbingWall
{
private:
    dWorldID world;
    dSpaceID space;
    dMatrix3 R; // rotation matrix

public:
    vec3 position;
    dReal height, width, thickness;
    vec3 orientation; //normal vector
    dBodyID body;
    dGeomID geom;
    int nb_holds;
    vec3* holds;

    ClimbingWall(dWorldID _world, dSpaceID _space);
    void draw();
};

#endif // WALL_H_INCLUDED
