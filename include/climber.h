#ifndef CLIMBER_H
#define CLIMBER_H

#include <ode/ode.h>

#include "vec3.h"
#include "climber_info.h"
#include "wall.h"


// BodyPart struct to store the body and the geoms together
typedef struct BodyPart {
    dReal length, radius;
    dBodyID body;
    dGeomID geom;
} BodyPart;

/*
 *  Climber class: provides methods to create and draw a climber, given an ODE world and space.
 */
class Climber {
    private:
        dWorldID world;  // the world in which the climber will be placed
        dSpaceID space;  // the space in which the climber will be placed
        vec3 offset; // the point in which the climber will be placed
        int partc, jointc; // body count and joint count
        dReal density; // the density of the body - TODO: getters and setters

        int addCapsuleWithMass(const dVector3& p1, const dVector3& p2, dReal radius, dReal mass);
        int addCapsule(const dVector3& p1, const dVector3& p2, dReal radius);
        dJointID addFixedJoint(int part1, int part2);
        dJointID addBallJoint(int part1, int part2, const dVector3 anchor);
        dJointID addAMotor(int part1, int part2, const dVector3 anchor);
        dJointID addHingeJoint(int part1, int part2, const dVector3 anchor, const dVector3 axis,
            dReal _loStop, dReal _hiStop);
        dJointID addUniversalJoint(int part1, int part2, const dVector3 anchor, const dVector3 axis1,
            const dVector3 axis2, dReal _loStop1, dReal _hiStop1, dReal _loStop2, dReal _hiStop2);

    public:
        BodyPart parts[15];   // store body part information
        dJointID joints[14];  // store joint ID's for easy iteration
        dReal total_mass;     // the total mass of the climber

        // Body part indexes
        int head;
        int hbody, lbody;
        int arm_right, arm_left;
        int forearm_right, forearm_left;
        int hand_right, hand_left;
        int thigh_right, thigh_left;
        int leg_right, leg_left;
        int foot_right, foot_left;

        // Joints IDs
        dJointID spine;
        dJointID neck;
        dJointID neck_motor;
        dJointID hip_right, hip_left;
        dJointID knee_right, knee_left;
        dJointID ankle_right, ankle_left;
        dJointID shoulder_right, shoulder_left;
        dJointID shoulder_right_motor, shoulder_left_motor;
        dJointID elbow_right, elbow_left;
        dJointID wrist_right, wrist_left;

        Climber(dWorldID _world, dSpaceID _space, dVector3 _offset);
        void draw();
        void setTargetVelocities(dReal *velocities);
        void getAngularVelocities(dReal *velocities);
        float cost(ClimbingWall* wallptr, Stance target_stance);
};

vec3 pos_end_bodypart(dBodyID body, dJointID joint);

#endif
