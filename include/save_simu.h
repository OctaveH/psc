#ifndef SAVE_SIMU_H_INCLUDED
#define SAVE_SIMU_H_INCLUDED

#include "body.h"

typedef std::array<dReal,4> vec4;

struct save_simu
{
    int nb_bodies;
    dBodyID* bodies;
    dGeomID* geoms;

    vec4* positions;
    vec4* quaternions;
    vec4* linear_velocities;
    vec4* angular_velocities;

    save_simu();
    save_simu(int _nb_bodies, dBodyID* _bodies, dGeomID* _geoms);
    ~save_simu();
};

void save(save_simu &data);
void load(const save_simu &data);

#endif // SAVE_SIMU_H_INCLUDED
