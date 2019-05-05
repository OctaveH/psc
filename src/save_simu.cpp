#include <iostream>
#include <algorithm>

#include "../include/save_simu.h"

save_simu::save_simu()
{
    return;
}

save_simu::save_simu(int _nb_bodies, dBodyID* _bodies, dGeomID* _geoms)
{
    nb_bodies = _nb_bodies;
    bodies = _bodies;
    geoms = _geoms;
    positions = new vec4[nb_bodies];
    quaternions =  new vec4[nb_bodies];
    linear_velocities =  new vec4[nb_bodies];
    angular_velocities =  new vec4[nb_bodies];
    //save();
}

save_simu::~save_simu()
{
    delete positions;
    delete quaternions;
    delete linear_velocities;
    delete angular_velocities;
}

void save(save_simu &data)
{
    for(int i = 0; i < data.nb_bodies; ++i)
    {
        const dReal* p;
        p = dBodyGetPosition(data.bodies[i]);
        data.positions[i] = {p[0], p[1], p[2], p[3]};

        p = dBodyGetQuaternion(data.bodies[i]); // saving rotations
        data.quaternions[i] = {p[0], p[1], p[2], p[3]};

        p = dBodyGetLinearVel(data.bodies[i]);
        data.linear_velocities[i] = {p[0], p[1], p[2], p[3]};

        p = dBodyGetAngularVel(data.bodies[i]);
        data.angular_velocities[i] = {p[0], p[1], p[2], p[3]};
    }
}

void load(const save_simu &data)
{
    for(int i = 0; i < data.nb_bodies; i++)
    {
        std::cout << "posx : " << data.positions[i][0];
        dBodySetPosition(data.bodies[i], data.positions[i][0], data.positions[i][1], data.positions[i][2]);
        dBodySetQuaternion(data.bodies[i], &(data.quaternions[i][0]));
        dBodySetLinearVel(data.bodies[i], data.linear_velocities[i][0], data.linear_velocities[i][1], data.linear_velocities[i][2]);
        dBodySetAngularVel(data.bodies[i], data.angular_velocities[i][0], data.angular_velocities[i][1], data.angular_velocities[i][2]);
    }
}
