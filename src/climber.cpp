#include <math.h>

#include <drawstuff/drawstuff.h>

#include "../include/vec3.h"
#include "../include/climber.h"
#include "../include/climber_info.h"

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

// Cost function constants
#define K_DIR 0.5 // 1
#define K_SIGMA 0.0025 // meters
#define K_VEL 100 // m/s^2
#define K_POS_FREE 0.087 // rad
#define K_POS_CLIMB 0.785 // rad
#define K_COM 20 // meters
#define K_FORCE 250 // Newtons
#define R_H 0.125 // meters
#define THETA_0 2.09 // rad

// Maximum torque
const dReal F_MAX = 100.0;

const vec3 rightAxis = { 0.0, 1.0, 0.0 };
const vec3 leftAxis = { 0.0, -1.0, 0.0 };
const vec3 upAxis = { 0.0, 0.0, 1.0 };
const vec3 downAxis = { 0.0, 0.0, -1.0 };
const vec3 bkwdAxis = { 1.0, 0.0, 0.0 };
const vec3 fwdAxis = { -1.0, 0.0, 0.0 };

Climber::Climber(dWorldID _world, dSpaceID _space, dVector3 _offset) {
    world = _world;
    space = _space;

    // set the offset that will be used to place all parts
    offset = _offset;
    density = 1000.0; // temporary use of homogeneous density

    // create all body parts
    partc = 0;
    jointc = 0;

    head = addCapsule(HEAD_1.vec, HEAD_2.vec, HEAD_RADIUS);
    parts[head].length = HEAD_LENGTH;
    parts[head].radius = HEAD_RADIUS;

	hbody = addCapsule(HBODY_1.vec, HBODY_2.vec, HBODY_RADIUS);
    parts[hbody].length = HBODY_LENGTH;
    parts[hbody].radius = HBODY_RADIUS;
    lbody = addCapsule(LBODY_1.vec, LBODY_2.vec, LBODY_RADIUS);
    parts[lbody].length = LBODY_LENGTH;
    parts[lbody].radius = LBODY_RADIUS;

    arm_right = addCapsule(ARM_RIGHT_1.vec, ARM_RIGHT_2.vec, ARM_RADIUS);
    parts[arm_right].length = ARM_LENGTH;
    parts[arm_right].radius = ARM_RADIUS;
    arm_left = addCapsule(ARM_LEFT_1.vec, ARM_LEFT_2.vec, ARM_RADIUS);
    parts[arm_left].length = ARM_LENGTH;
    parts[arm_left].radius = ARM_RADIUS;

    forearm_right = addCapsule(FOREARM_RIGHT_1.vec, FOREARM_RIGHT_2.vec, FOREARM_RADIUS);
    parts[forearm_right].length = FOREARM_LENGTH;
    parts[forearm_right].radius = FOREARM_RADIUS;
    forearm_left = addCapsule(FOREARM_LEFT_1.vec, FOREARM_LEFT_2.vec, FOREARM_RADIUS);
    parts[forearm_left].length = FOREARM_LENGTH;
    parts[forearm_left].radius = FOREARM_RADIUS;

    hand_right = addCapsule(HAND_RIGHT_1.vec, HAND_RIGHT_2.vec, HAND_RADIUS);
    parts[hand_right].length = HAND_LENGTH;
    parts[hand_right].radius = HAND_RADIUS;
    hand_left = addCapsule(HAND_LEFT_1.vec, HAND_LEFT_2.vec, HAND_RADIUS);
    parts[hand_left].length = HAND_LENGTH;
    parts[hand_left].radius = HAND_RADIUS;

    thigh_right = addCapsule(THIGH_RIGHT_1.vec, THIGH_RIGHT_2.vec, THIGH_RADIUS);
    parts[thigh_right].length = THIGH_LENGTH;
    parts[thigh_right].radius = THIGH_RADIUS;
    thigh_left = addCapsule(THIGH_LEFT_1.vec, THIGH_LEFT_2.vec, THIGH_RADIUS);
    parts[thigh_left].length = THIGH_LENGTH;
    parts[thigh_left].radius = THIGH_RADIUS;

    leg_right = addCapsule(LEG_RIGHT_1.vec, LEG_RIGHT_2.vec, LEG_RADIUS);
    parts[leg_right].length = LEG_LENGTH;
    parts[leg_right].radius = LEG_RADIUS;
    leg_left = addCapsule(LEG_LEFT_1.vec, LEG_LEFT_2.vec, LEG_RADIUS);
    parts[leg_left].length = LEG_LENGTH;
    parts[leg_left].radius = LEG_RADIUS;

    foot_right = addCapsule(FOOT_RIGHT_1.vec, FOOT_RIGHT_2.vec, FOOT_RADIUS);
    parts[foot_right].length = FOOT_LENGTH;
    parts[foot_right].radius = FOOT_RADIUS;
    foot_left = addCapsule(FOOT_LEFT_1.vec, FOOT_LEFT_2.vec, FOOT_RADIUS);
    parts[foot_left].length = FOOT_LENGTH;
    parts[foot_left].radius = FOOT_RADIUS;

    // create all joints
    spine = addFixedJoint(lbody, hbody);

    neck = addBallJoint(hbody, head, NECK_POS.vec);
    neck_motor = addAMotor(hbody, head, NECK_POS.vec);

    hip_right = addUniversalJoint(thigh_right, lbody, HIP_RIGHT_POS.vec, bkwdAxis.vec, rightAxis.vec, -0.1*M_PI, 0.3*M_PI, -0.15*M_PI, 0.75*M_PI);
    hip_left = addUniversalJoint(thigh_left, lbody, HIP_LEFT_POS.vec, fwdAxis.vec, rightAxis.vec, -0.1*M_PI, 0.3*M_PI, -0.15*M_PI, 0.75*M_PI);

    knee_right = addHingeJoint(leg_right, thigh_right, KNEE_RIGHT_POS.vec, leftAxis.vec, 0.0, 0.75*M_PI);
    knee_left = addHingeJoint(leg_left, thigh_left, KNEE_LEFT_POS.vec, leftAxis.vec, 0.0, 0.75*M_PI);

    ankle_right = addHingeJoint(foot_right, leg_right, ANKLE_RIGHT_POS.vec, rightAxis.vec, -0.1*M_PI, 0.05*M_PI);
    ankle_left = addHingeJoint(foot_left, leg_left, ANKLE_LEFT_POS.vec, rightAxis.vec, -0.1*M_PI, 0.05*M_PI);

    shoulder_right = addBallJoint(hbody, arm_right, SHOULDER_RIGHT_POS.vec);
    shoulder_right_motor = addAMotor(hbody, arm_right, SHOULDER_RIGHT_POS.vec);
    shoulder_left = addBallJoint(hbody, arm_left, SHOULDER_LEFT_POS.vec);
    shoulder_left_motor = addAMotor(hbody, arm_left, SHOULDER_LEFT_POS.vec);

    elbow_right = addHingeJoint(arm_right, forearm_right, ELBOW_RIGHT_POS.vec, downAxis.vec, 0.0, 0.6*M_PI);
    elbow_left = addHingeJoint(arm_left, forearm_left, ELBOW_LEFT_POS.vec, upAxis.vec, 0.0, 0.6*M_PI);

    wrist_right = addHingeJoint(forearm_right, hand_right, WRIST_RIGHT_POS.vec, fwdAxis.vec, -0.1*M_PI, 0.2*M_PI);
    wrist_left = addHingeJoint(forearm_left, hand_left, WRIST_LEFT_POS.vec, bkwdAxis.vec, -0.1*M_PI, 0.2*M_PI);
}

int Climber::addCapsuleWithMass(const dVector3& p1, const dVector3& p2, dReal radius, dReal mass) {
    // calculate actual positions (with offset)
    vec3 pos1 = vec3(p1) + offset;
	vec3 pos2 = vec3(p2) + offset;

    // calculate length of the capsule (control this if want the capsules to touch)
	dReal cap_len = dist3(pos1, pos2)-radius;

    // create capsule
	dBodyID capsule = dBodyCreate(world);

    // create capsule mass and bind it to the body
    dMass m;
	dMassSetZero(&m);
	dMassSetCapsuleTotal(&m, mass, 3, radius, cap_len);
	dBodySetMass(capsule, &m);
    total_mass += mass; // keep track of the total mass

    // create capsule geom and bind it to the body
    dGeomID geom=dCreateCapsule(space, radius, cap_len);
    dGeomSetBody(geom, capsule);

    // set the body position to its center of mass
    vec3 center_of_mass = 0.5*(pos1+pos2);
	dBodySetPosition(capsule, center_of_mass[0], center_of_mass[1], center_of_mass[2]);

    // determine the capsule's rotation from its pos1---pos2 axis
    vec3 xa, ya, za; // x, y and z axis

    za = unit3(pos2-pos1); // set the z axis to the capsule's axi

    if(abs(dot3({1.0, 0.0, 0.0}, za)) < 0.7)
        xa = {1.0, 0.0, 0.0};
    else
        xa = {0.0, 1.0, 0.0}; // choose an initial value for x axis

    ya = cross(za, xa); // calculate the y axis
    xa = unit3(cross(ya, za)); // calculate the final value for the x axis

    dReal rot[12]={ xa[0], ya[0], za[0],
                    xa[1], ya[1], za[1],
                    xa[2], ya[2], za[2],
                      0.0,   0.0,   0.0 }; // create the rotation matrix

    // set the capsule's rotation
    dBodySetRotation(capsule, rot);

	parts[partc].geom = geom; // add geom to geom array
	parts[partc].body = capsule; // add capsule to body array

	return partc++;
}

int Climber::addCapsule(const dVector3& p1, const dVector3& p2, dReal radius) {
    // calculate actual positions (with offset)
    vec3 pos1 = vec3(p1)+offset;
	vec3 pos2 = vec3(p2)+offset;

    // calculate length of the capsule (control this if want the capsules to touch)
	dReal cap_len = dist3(pos1, pos2)-2*radius;

    // create capsule
	dBodyID capsule = dBodyCreate(world);

    // create capsule mass and bind it to the body
    dMass m;
	dMassSetZero(&m);
	dMassSetCapsule(&m, density, 3, radius, cap_len);
	dBodySetMass(capsule, &m);
    total_mass += m.mass; // keep track of the total mass

    // create capsule geom and bind it to the body
    dGeomID geom = dCreateCapsule(space, radius, cap_len);
    dGeomSetBody(geom, capsule);

    // set the body position to its center of mass
    vec3 center_of_mass = 0.5*(pos1+pos2);
	dBodySetPosition(capsule, center_of_mass[0], center_of_mass[1], center_of_mass[2]);

    // determine the capsule's rotation from its pos1---pos2 axis
    vec3 xa, ya, za; // x, y and z axis

    za = unit3(pos2-pos1); // set the z axis to the capsule's axi

    if(fabs(dot3({1.0, 0.0, 0.0}, za)) < 0.7)
        xa = {1.0, 0.0, 0.0};
    else
        xa = {0.0, 1.0, 0.0}; // choose an initial value for x axis

    ya = cross(za, xa); // calculate the y axis
    xa = unit3(cross(ya, za)); // calculate the final value for the x axis

    //printf("xa: %f %f %f\n", xa[0], xa[1], xa[2]);
    //printf("ya: %f %f %f\n", ya[0], ya[1], ya[2]);
    //printf("za: %f %f %f\n", za[0], za[1], za[2]);

    dMatrix3 rot;
    dRSetIdentity(rot);
    dRFrom2Axes(rot, xa[0], xa[1], xa[2], ya[0], ya[1], ya[2]);

    /*
    dMatrix3 rot ={ xa[0], ya[0], za[0],
                    xa[1], ya[1], za[1],
                    xa[2], ya[2], za[2],
                      0.0,   0.0,   0.0 }; // create the rotation matrix
	*/

    // set the capsule's rotation
    dBodySetRotation(capsule, rot);

	parts[partc].geom = geom; // add geom to geom array
	parts[partc].body = capsule; // add capsule to body array

	return partc++;
}

dJointID Climber::addFixedJoint(int part1, int part2) {
    dJointID joint = dJointCreateFixed(world, 0); // allocate joint normally

    dJointAttach(joint, parts[part1].body, parts[part2].body); // attach the two bodies with this joint
    dJointSetFixed(joint); // set the joint to fixed

    joints[jointc++]=joint; // add to joint array

    return joint;
}

dJointID Climber::addBallJoint(int part1, int part2, const dVector3 anchor){
    dJointID joint=dJointCreateBall(world, 0); // allocate joint normally

    dJointAttach(joint, parts[part1].body, parts[part2].body); // attach the two bodies with this joint
    dJointSetBallAnchor(joint, anchor[0]+offset[0], anchor[1]+offset[1], anchor[2]+offset[2]); // set this joint to ball anchor

    joints[jointc++] = joint; // add the joint to the array

    return joint;
}

dJointID Climber::addAMotor(int part1, int part2, const dVector3 anchor){
    dJointID motor = dJointCreateAMotor(world, 0); // allocate joint normally

    dJointAttach(motor, parts[part1].body, parts[part2].body); // attach the two bodies with this joint
    dJointSetAMotorMode(motor, dAMotorUser);

    dJointSetAMotorNumAxes(motor, 3);
    dJointSetAMotorAxis(motor, 0, 0, fwdAxis[0], fwdAxis[1], fwdAxis[2]);
    dJointSetAMotorAxis(motor, 1, 0, rightAxis[0], rightAxis[1], rightAxis[2]);
    dJointSetAMotorAxis(motor, 2, 0, upAxis[0], upAxis[1], upAxis[2]);

    return motor;
}

dJointID Climber::addHingeJoint(int part1, int part2, const dVector3 anchor, const dVector3 axis,
                                dReal _loStop, dReal _hiStop) {
    vec3 hinge_anchor = vec3(anchor) + offset;
    dJointGroupID joint_group = dJointGroupCreate(0);
    dJointID joint = dJointCreateHinge(world, joint_group);

    dJointAttach(joint, parts[part1].body, parts[part2].body);
    dJointSetHingeAnchor(joint, hinge_anchor[0], hinge_anchor[1], hinge_anchor[2]);
    dJointSetHingeAxis(joint, axis[0], axis[1], axis[2]);
    dJointSetHingeParam(joint, dParamLoStop, _loStop);
    dJointSetHingeParam(joint, dParamHiStop, _hiStop);

    joints[jointc++] = joint;

    return joint;
}

dJointID Climber::addUniversalJoint(int part1, int part2, const dVector3 anchor, const dVector3 axis1,
    const dVector3 axis2, dReal _loStop1, dReal _hiStop1, dReal _loStop2, dReal _hiStop2) {
    vec3 universal_anchor = vec3(anchor) + offset;
    dJointGroupID joint_group = dJointGroupCreate(0);
    dJointID joint = dJointCreateUniversal(world, joint_group);

    dJointAttach(joint, parts[part1].body, parts[part2].body);
    dJointSetUniversalAnchor(joint, universal_anchor[0], universal_anchor[1], universal_anchor[2]);
    dJointSetUniversalAxis1(joint, axis1[0], axis2[1], axis1[2]);
    dJointSetUniversalAxis2(joint, axis2[0], axis2[1], axis2[2]);
    dJointSetUniversalParam(joint, dParamLoStop1, _loStop1);
    dJointSetUniversalParam(joint, dParamLoStop2, _loStop2);
    dJointSetUniversalParam(joint, dParamHiStop1, _hiStop1);
    dJointSetUniversalParam(joint, dParamHiStop2, _hiStop2);

    joints[jointc++] = joint;

    return joint;
}

void Climber::draw() {
    for(int i=0; i<15; ++i) {
        const dReal *pos = dBodyGetPosition(parts[i].body);
        const dReal *R = dBodyGetRotation(parts[i].body);

        const double _pos[4] = { pos[0], pos[1], pos[2], pos[3] };
        const double _R[12] = { R[0], R[ 1], R[ 2],
                                R[3], R[ 4], R[ 5],
                                R[6], R[ 7], R[ 8],
                                R[9], R[10], R[11] };

        dsDrawCapsuleD(_pos, _R, parts[i].length, parts[i].radius);
    }
}

void Climber::setTargetVelocities(dReal *velocities) {
    int i = 0;

    // neck
    dJointSetAMotorParam(neck_motor, dParamVel1, velocities[i++]);
    dJointSetAMotorParam(neck_motor, dParamVel2, velocities[i++]);
    dJointSetAMotorParam(neck_motor, dParamVel3, velocities[i++]);
    dJointSetAMotorParam(neck_motor, dParamFMax1, F_MAX);
    dJointSetAMotorParam(neck_motor, dParamFMax2, F_MAX);
    dJointSetAMotorParam(neck_motor, dParamFMax3, F_MAX);

    // hips
    dJointSetUniversalParam(hip_right, dParamVel1, velocities[i++]);
    dJointSetUniversalParam(hip_right, dParamVel2, velocities[i++]);
    dJointSetUniversalParam(hip_right, dParamFMax1, F_MAX);
    dJointSetUniversalParam(hip_right, dParamFMax2, F_MAX);

    dJointSetUniversalParam(hip_left, dParamVel1, velocities[i++]);
    dJointSetUniversalParam(hip_left, dParamVel2, velocities[i++]);
    dJointSetUniversalParam(hip_left, dParamFMax1, F_MAX);
    dJointSetUniversalParam(hip_left, dParamFMax2, F_MAX);

    // knees
    dJointSetHingeParam(knee_right, dParamVel, velocities[i++]);
    dJointSetHingeParam(knee_right, dParamFMax, F_MAX);

    dJointSetHingeParam(knee_left, dParamVel, velocities[i++]);
    dJointSetHingeParam(knee_left, dParamFMax, F_MAX);

    // ankles
    dJointSetHingeParam(ankle_right, dParamVel, velocities[i++]);
    dJointSetHingeParam(ankle_right, dParamFMax, F_MAX);

    dJointSetHingeParam(ankle_left, dParamVel, velocities[i++]);
    dJointSetHingeParam(ankle_left, dParamFMax, F_MAX);

    // shoulders
    dJointSetAMotorParam(shoulder_right_motor, dParamVel1, velocities[i++]);
    dJointSetAMotorParam(shoulder_right_motor, dParamVel2, velocities[i++]);
    dJointSetAMotorParam(shoulder_right_motor, dParamVel3, velocities[i++]);
    dJointSetAMotorParam(shoulder_right_motor, dParamFMax1, F_MAX);
    dJointSetAMotorParam(shoulder_right_motor, dParamFMax2, F_MAX);
    dJointSetAMotorParam(shoulder_right_motor, dParamFMax3, F_MAX);

    dJointSetAMotorParam(shoulder_left_motor, dParamVel1, velocities[i++]);
    dJointSetAMotorParam(shoulder_left_motor, dParamVel2, velocities[i++]);
    dJointSetAMotorParam(shoulder_left_motor, dParamVel3, velocities[i++]);
    dJointSetAMotorParam(shoulder_left_motor, dParamFMax1, F_MAX);
    dJointSetAMotorParam(shoulder_left_motor, dParamFMax2, F_MAX);
    dJointSetAMotorParam(shoulder_left_motor, dParamFMax3, F_MAX);

    // elbows
    dJointSetHingeParam(elbow_right, dParamVel, velocities[i++]);
    dJointSetHingeParam(elbow_right, dParamFMax, F_MAX);

    dJointSetHingeParam(elbow_left, dParamVel, velocities[i++]);
    dJointSetHingeParam(elbow_left, dParamFMax, F_MAX);

    // wrists
    dJointSetHingeParam(wrist_right, dParamVel, velocities[i++]);
    dJointSetHingeParam(wrist_right, dParamFMax, F_MAX);

    dJointSetHingeParam(wrist_left, dParamVel, velocities[i++]);
    dJointSetHingeParam(wrist_left, dParamFMax, F_MAX);
}

void Climber::getAngularVelocities(dReal *velocities) {
    int i = 0;

    // neck
    velocities[i++] = dJointGetAMotorParam(neck_motor, dParamVel1);
    velocities[i++] = dJointGetAMotorParam(neck_motor, dParamVel2);
    velocities[i++] = dJointGetAMotorParam(neck_motor, dParamVel3);

    // hips
    velocities[i++] = dJointGetUniversalParam(hip_right, dParamVel1);
    velocities[i++] = dJointGetUniversalParam(hip_right, dParamVel2);

    velocities[i++] = dJointGetUniversalParam(hip_left, dParamVel1);
    velocities[i++] = dJointGetUniversalParam(hip_left, dParamVel2);

    // knees
    velocities[i++] = dJointGetHingeParam(knee_right, dParamVel);

    velocities[i++] = dJointGetHingeParam(knee_left, dParamVel);

    // ankles
    velocities[i++] = dJointGetHingeParam(ankle_right, dParamVel);

    velocities[i++] = dJointGetHingeParam(ankle_left, dParamVel);

    // shoulders
    velocities[i++] = dJointGetAMotorParam(shoulder_right_motor, dParamVel1);
    velocities[i++] = dJointGetAMotorParam(shoulder_right_motor, dParamVel2);
    velocities[i++] = dJointGetAMotorParam(shoulder_right_motor, dParamVel3);

    velocities[i++] = dJointGetAMotorParam(shoulder_left_motor, dParamVel1);
    velocities[i++] = dJointGetAMotorParam(shoulder_left_motor, dParamVel2);
    velocities[i++] = dJointGetAMotorParam(shoulder_left_motor, dParamVel3);

    // elbows
    velocities[i++] = dJointGetHingeParam(elbow_right, dParamVel);

    velocities[i++] = dJointGetHingeParam(elbow_left, dParamVel);

    // wrists
    velocities[i++] = dJointGetHingeParam(wrist_right, dParamVel);

    velocities[i++] = dJointGetHingeParam(wrist_left, dParamVel);
}

float Climber::cost(ClimbingWall* wallptr, Stance target_stance)
{
    float c = 0;

    // 1. Distance to the target stance
    float square_dist = 0;
    if (target_stance.lh_hold != -1)
    {
        vec3 pos = pos_end_bodypart(parts[hand_left].body, wrist_left);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.lh_hold],pos), 2);
    }
    if (target_stance.rh_hold != -1)
    {
        vec3 pos = pos_end_bodypart(parts[hand_right].body, wrist_right);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.rh_hold],pos), 2);
    }
    if (target_stance.lf_hold != -1)
    {
        vec3 pos = pos_end_bodypart(parts[foot_left].body, ankle_left);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.lf_hold],pos), 2);
    }
    if (target_stance.rf_hold != -1)
    {
        vec3 pos = pos_end_bodypart(parts[foot_right].body, ankle_right);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.rf_hold],pos), 2);
    }
    c += square_dist / (K_SIGMA*K_SIGMA);

    // 2. Distance to the wall
    vec3 barycentre = {0,0,0};
    dReal mass = 0.0;
    for(int i = 0; i < 15; i++)
    {
        const dReal* pos = dBodyGetPosition(parts[i].body);
        dReal mass_part = parts[i].length * parts[i].radius * parts[i].radius;
        barycentre = barycentre + mass_part * (vec3){pos[0], pos[1], pos[2]};
        mass += mass_part;
    }
    barycentre = barycentre / mass;
    float distance = dot3(barycentre - wallptr->position, wallptr->orientation) - wallptr->thickness / 2;
    c += distance*distance / (K_COM*K_COM);

    // 3. Distance to the prefered posture
    // TODO

    // 4. Facing toward the wall
    // TODO

    // 5. Members velocities
    // TODO

    // 6. Torques and forces
    // TODO

    return c;
}

vec3 pos_end_bodypart(dBodyID body, dJointID joint)
{
    const dReal* m = dBodyGetPosition(body); // position of the center of the body part
    dVector3 j; // position of the joint
    dJointGetHingeAnchor2(joint, j);
    vec3 member_end_pos = {2*m[0]-j[0],2*m[1]-j[1],2*m[2]-j[2]};
    return member_end_pos;
}
