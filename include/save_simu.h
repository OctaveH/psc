#ifndef SAVE_SIMU_H_INCLUDED
#define SAVE_SIMU_H_INCLUDED

#include "body.h"

typedef std::array<dReal,4> vec4;

class save_simu
{
public:
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
    void save();
    void load();
};

#endif // SAVE_SIMU_H_INCLUDED
