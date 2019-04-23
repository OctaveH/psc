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
    //TODO
}

void save_simu::save()
{
    //std::cout << "nb_bodies : " << nb_bodies << std::endl;
    for(int i = 0; i < nb_bodies; i++)
    {
        //std::cout << "boucle : " << i << std::endl;
        //std::cout << "posz : " << dBodyGetPosition(bodies[i])[2] << std::endl;
        const dReal* p;
        p = dBodyGetPosition(bodies[i]);
        positions[i] = {p[0], p[1], p[2], p[3]};
        //std::cout << "posz_cp : " << positions[i][2] << std::endl;

        p = dBodyGetQuaternion(bodies[i]);
        quaternions[i] = {p[0], p[1], p[2], p[3]};
        p = dBodyGetLinearVel(bodies[i]);
        linear_velocities[i] = {p[0], p[1], p[2], p[3]};
        p = dBodyGetAngularVel(bodies[i]);
        angular_velocities[i] = {p[0], p[1], p[2], p[3]};
    }
    std::cout << "posz head : " << positions[0][2] << std::endl;
    std::cout << "posz 2 : " << positions[1][2] << std::endl;
}

void save_simu::load()
{
    for(int i = 0; i < nb_bodies; i++)
    {
        std::cout << "posx : " << positions[i][0];
        dBodySetPosition(bodies[i], positions[i][0], positions[i][1], positions[i][2]);
        dBodySetQuaternion(bodies[i], &(quaternions[i][0]));
        dBodySetLinearVel(bodies[i], linear_velocities[i][0], linear_velocities[i][1], linear_velocities[i][2]);
        dBodySetAngularVel(bodies[i], angular_velocities[i][0], angular_velocities[i][1], angular_velocities[i][2]);
    }
}
