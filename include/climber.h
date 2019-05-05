#ifndef CLIMBER_H
#define CLIMBER_H

#include <ode/ode.h>

#include "vec3.h"
#include "climber_info.h"
#include "wall.h"

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
const vec3 ARM_LEFT_2 = { 0.0, ARM_LEFT_1[1]-ARM_LENGTH, ARM_LEFT_1[2] };

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
const vec3 NECK_POS = HBODY_2 + vec3({ 0.0, 0.0, NECK_LENGTH/2});

// Hip
const vec3 HIP_RIGHT_POS = THIGH_RIGHT_2;
const vec3 HIP_LEFT_POS = THIGH_LEFT_2;

// Knees
const vec3 KNEE_RIGHT_POS = LEG_RIGHT_2;
const vec3 KNEE_LEFT_POS = LEG_LEFT_2;

// Ankles
const vec3 ANKLE_RIGHT_POS = LEG_RIGHT_1;
const vec3 ANKLE_LEFT_POS = LEG_LEFT_1;

// Shoulders
const vec3 SHOULDER_RIGHT_POS = HBODY_2 + vec3({ 0.0, SHOULDER_WIDTH/2, 0.0});
const vec3 SHOULDER_LEFT_POS = HBODY_2 + vec3({ 0.0, -SHOULDER_WIDTH/2, 0.0});

// Elbows
const vec3 ELBOW_RIGHT_POS = ARM_RIGHT_2;
const vec3 ELBOW_LEFT_POS = ARM_LEFT_2;

// Wrists
const vec3 WRIST_RIGHT_POS = FOREARM_RIGHT_2;
const vec3 WRIST_LEFT_POS = FOREARM_LEFT_2;

// BodyPart struct to store the body and the geoms together
typedef struct BodyPart {
    dReal length, radius;
    dBodyID body;
    dGeomID geom;
} BodyPart;

// Maximum torque
const dReal F_MAX = 100.0;

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
        float cost(ClimbingWall* wallptr, Stance target_stance);
};

vec3 pos_end_bodypart(dBodyID body, dJointID joint);

#endif
