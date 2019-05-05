#include <drawstuff/drawstuff.h>

#include "../include/wall.h"
#include "../include/wall_info.h"

ClimbingWall::ClimbingWall(dWorldID _world, dSpaceID _space)
{
    world = _world;
    space = _space;

    height = HEIGHT;
    width = WIDTH;
    thickness = THICKNESS;
    position = {POSITION_X, POSITION_Y, POSITION_Z};
    orientation = {ORIENTATION_X, ORIENTATION_Y, ORIENTATION_Z};
    nb_holds = NB_HOLDS;

    body = dBodyCreate(world);
    dBodySetKinematic(body); // set the wall static (=infinite mass)

    geom = dCreateBox(space, thickness, width, height);
    dGeomSetBody(geom, body);

    dBodySetPosition(body, position[0], position[1], position[2]);

    dRSetIdentity(R);
    dRFrom2Axes(R, 1.0, 0, 0, orientation[0], orientation[1], orientation[2]);
    dBodySetRotation(body, R);

    holds = new vec3[NB_HOLDS];
    for (int i = 0; i < NB_HOLDS; i++)
    {
        holds[i] = _holds[i];
    }
}

void ClimbingWall::draw()
{
    dsSetColor(0.9, 0.8, 0.7);
    const double pos[] = {position[0], position[1], position[2]};
    const double sides[] = {thickness, width, height};
    dsDrawBoxD(pos, R, sides);
    for (int i = 0; i < nb_holds; i++)
    {
        const double pos[] = {holds[i][0], holds[i][1], holds[i][2]};
        dsDrawSphereD(pos, R, 0.1);
    }
}
