#ifndef SAVE_SIMU_H_INCLUDED
#define SAVE_SIMU_H_INCLUDED

#include <array>
#include "climber.h"

typedef std::array<dReal,4> vec4;

struct SaveSimu
{
    int nb_bodies;
    dBodyID* bodies;
    dGeomID* geoms;

    vec4* positions;
    vec4* quaternions;
    vec4* linear_velocities;
    vec4* angular_velocities;

    SaveSimu();
    SaveSimu(int _nb_bodies, dBodyID* _bodies, dGeomID* _geoms);
    ~SaveSimu();
};

void save(SaveSimu &data);
void load(const SaveSimu &data);

#endif // SAVE_SIMU_H_INCLUDED
