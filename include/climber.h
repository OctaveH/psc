#ifndef CLIMBER_H
#define CLIMBER_H

#include <ode/ode.h>
#include "vec3.h"
#include "climber_info.h"

/*
 *  Body part positions for a T-pose facing the +x direction
 *  (Note: do not change the order of these declarations)
 */

// Feet
const vec3 FOOT_RIGHT_1 = { -LEG_RADIUS, HIP_WIDTH/2, FOOT_RADIUS };
const vec3 FOOT_RIGHT_2 = { FOOT_LENGTH-LEG_RADIUS, HIP_WIDTH/2, FOOT_RADIUS };

const vec3 FOOT_LEFT_1 = { -LEG_RADIUS, -HIP_WIDTH/2, FOOT_RADIUS };
const vec3 FOOT_LEFT_2 = { FOOT_LENGTH-LEG_RADIUS, -HIP_WIDTH/2, FOOT_RADIUS };

// Legs
const vec3 LEG_RIGHT_1 = { 0.0, HIP_WIDTH/2, 2*FOOT_RADIUS };
const vec3 LEG_RIGHT_2 = { 0.0, HIP_WIDTH/2, LEG_RIGHT_1[2]+LEG_LENGTH };

const vec3 LEG_LEFT_1 = { 0.0, -HIP_WIDTH/2, 2*FOOT_RADIUS };
const vec3 LEG_LEFT_2 = { 0.0, -HIP_WIDTH/2, LEG_LEFT_1[2]+LEG_LENGTH };

// Thighs
const vec3 THIGH_RIGHT_1 = { 0.0, HIP_WIDTH/2, LEG_RIGHT_2[2] };
const vec3 THIGH_RIGHT_2 = { 0.0, HIP_WIDTH/2, THIGH_RIGHT_1[2]+THIGH_LENGTH };

const vec3 THIGH_LEFT_1 = { 0.0, -HIP_WIDTH/2, LEG_LEFT_2[2] };
const vec3 THIGH_LEFT_2 = { 0.0, -HIP_WIDTH/2, THIGH_LEFT_1[2]+THIGH_LENGTH };

//  Lower Body
const vec3 LBODY_1 = { 0.0, 0.0, THIGH_RIGHT_2[2] };
const vec3 LBODY_2 = { 0.0, 0.0, LBODY_1[2]+LBODY_LENGTH };

// High (Top) Body
const vec3 HBODY_1 = { 0.0, 0.0, LBODY_2[2] };
const vec3 HBODY_2 = { 0.0, 0.0, HBODY_1[2]+HBODY_LENGTH };

// Head
const vec3 HEAD_1 = { 0.0, 0.0, HBODY_2[2]+NECK_LENGTH };
const vec3 HEAD_2 = { 0.0, 0.0, HEAD_1[2]+HEAD_LENGTH };

// Arms
const vec3 ARM_RIGHT_1 = { 0.0, SHOULDER_WIDTH/2, HBODY_2[2] };
const vec3 ARM_RIGHT_2 = { 0.0, ARM_RIGHT_1[1]+ARM_LENGTH, ARM_RIGHT_1[2] };

const vec3 ARM_LEFT_1 = { 0.0, -SHOULDER_WIDTH/2, HBODY_2[2] };
const vec3 ARM_LEFT_2 = { 0.0, ARM_RIGHT_1[1]-ARM_LENGTH, ARM_LEFT_1[2] };

// Forearms
const vec3 FOREARM_RIGHT_1 = { 0.0, ARM_RIGHT_2[1], HBODY_2[2] };
const vec3 FOREARM_RIGHT_2 = { 0.0, FOREARM_RIGHT_1[1]+FOREARM_LENGTH, FOREARM_RIGHT_1[2] };

const vec3 FOREARM_LEFT_1 = { 0.0, ARM_LEFT_2[1], HBODY_2[2] };
const vec3 FOREARM_LEFT_2 = { 0.0, FOREARM_LEFT_1[1]-FOREARM_LENGTH, FOREARM_LEFT_1[2] };

// Hands
const vec3 HAND_RIGHT_1 = { 0.0, FOREARM_RIGHT_2[1], HBODY_2[2] };
const vec3 HAND_RIGHT_2 = { 0.0, HAND_RIGHT_1[1]+HAND_LENGTH, HAND_RIGHT_1[2] };

const vec3 HAND_LEFT_1 = { 0.0, FOREARM_LEFT_2[1], HBODY_2[2] };
const vec3 HAND_LEFT_2 = { 0.0, HAND_LEFT_1[1]-HAND_LENGTH, HAND_LEFT_1[2] };

/*
 *  Joint positions for a T-pose facing the +x direction
 *  (Note: do not change the order of these declarations)
 */

// Neck
vec3 NECK_POS = HBODY_2 + vec3({ 0.0, 0.0, NECK_LENGTH/2});

// Hip
vec3 HIP_RIGHT_POS = THIGH_RIGHT_2;
vec3 HIP_LEFT_POS = THIGH_LEFT_2;

// Knees
vec3 KNEE_RIGHT_POS = LEG_RIGHT_2;
vec3 KNEE_LEFT_POS = LEG_LEFT_2;

// Ankles
vec3 ANKLE_RIGHT_POS = LEG_RIGHT_1;
vec3 ANKLE_LEFT_POS = LEG_LEFT_1;

// Shoulders
vec3 SHOULDER_RIGHT_POS = HBODY_2 + vec3({ 0.0, SHOULDER_WIDTH/2, 0.0});
vec3 SHOULDER_LEFT_POS = HBODY_2 + vec3({ 0.0, -SHOULDER_WIDTH/2, 0.0});

// Elbows
vec3 ELBOW_RIGHT_POS = ARM_RIGHT_2;
vec3 ELBOW_LEFT_POS = ARM_LEFT_2;

// Wrists
vec3 WRIST_RIGHT_POS = FOREARM_RIGHT_2;
vec3 WRIST_LEFT_POS = FOREARM_LEFT_2;

/*
 *  Climber class: provides methods to create and draw a climber, given an ODE world and space.
 */
class Climber {
    private:
        dWorldID world;  // the world in which the climber will be placed
        dSpaceID space;  // the space in which the climber will be placed
        vec3 offset; // the point in which the climber will be placed
        int bodyc, jointc; // body count and joint count
        dReal density; // the density of the body - TODO: getters and setters

        dBodyID addCapsuleWithMass(const dVector3& p1, const dVector3& p2, dReal radius, dReal mass);
        dBodyID addCapsule(const dVector3& p1, const dVector3& p2, dReal radius);
        dJointID addFixedJoint(dBodyID body1, dBodyID body2);
        dJointID addBallJoint(dBodyID body1, dBodyID body2, dVector3 anchor);
    
    public:
        dBodyID bodies[15];   // store body ID's for easy iteration
        dGeomID geoms[15];    // store geom ID's for easy iteration
        dJointID joints[14];  // store joint ID's for easy iteration
        dReal total_mass;     // the total mass of the climber

        // Body part IDs
        dBodyID head;
        dBodyID hbody, lbody;
        dBodyID arm_right, arm_left;
        dBodyID forearm_right, forearm_left;
        dBodyID hand_right, hand_left;
        dBodyID thigh_right, thigh_left;
        dBodyID leg_right, leg_left;
        dBodyID foot_right, foot_left;
 
        // Joints IDs
        dJointID spine;
        dJointID neck;
        dJointID hip_right, hip_left;
        dJointID knee_right, knee_left;
        dJointID ankle_right, ankle_left;
        dJointID shoulder_right, shoulder_left;
        dJointID elbow_right, elbow_left;
        dJointID wrist_right, wrist_left;

        Climber(dWorldID _world, dSpaceID _space);
        void addClimber(dVector3 offset);
};

#endif