#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "../include/util.h"
#include "../include/body.h"
#include "../include/wall.h"

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

const dReal UPPER_ARM_LEN = 0.30;
const dReal FORE_ARM_LEN = 0.25;
const dReal HAND_LEN = 0.13; // wrist to mid-fingers only
const dReal FOOT_LEN = 0.18; // ankles to base of ball of foot only
const dReal HEEL_LEN = 0.05;

const dReal BROW_H = 1.68;
const dReal MOUTH_H = 1.53;
const dReal NECK_H = 1.50;
const dReal SHOULDER_H = 1.37;
const dReal CHEST_H = 1.35;
const dReal HIP_H = 0.86;
const dReal KNEE_H = 0.48;
const dReal ANKLE_H = 0.08;

const dReal SHOULDER_W = 0.41;
const dReal CHEST_W = 0.36; // actually wider, but we want narrower than shoulders (esp. with large radius)
const dReal LEG_W = 0.28; // between middles of upper legs
const dReal PELVIS_W = 0.25; // actually wider, but we want smaller than hip width

/*
 *  Body part positions for a T-pose facing the -x direction
 */

const vec3 R_SHOULDER_POS = { 0.0,SHOULDER_W * 0.5, SHOULDER_H};
const vec3 L_SHOULDER_POS = { 0.0,-SHOULDER_W * 0.5, SHOULDER_H};
const vec3 a1= { 0.0,-UPPER_ARM_LEN, 0.0};
const vec3 a2= { 0.0,-FORE_ARM_LEN, 0.0};
const vec3 a3= { 0.0,-HAND_LEN, 0.0};
const vec3 R_ELBOW_POS = R_SHOULDER_POS-a1;
const vec3 L_ELBOW_POS = L_SHOULDER_POS+a1;
const vec3 R_WRIST_POS = R_ELBOW_POS-a2;
const vec3 L_WRIST_POS = L_ELBOW_POS+a2;
const vec3 R_FINGERS_POS = R_WRIST_POS-a3;
const vec3 L_FINGERS_POS = L_WRIST_POS+a3;

const vec3 R_HIP_POS = { 0.0,LEG_W * 0.5, HIP_H};
const vec3 L_HIP_POS = { 0.0,-LEG_W * 0.5, HIP_H};
const vec3 R_KNEE_POS = { 0.0,LEG_W * 0.5, KNEE_H};
const vec3 L_KNEE_POS = { 0.0,-LEG_W * 0.5, KNEE_H};
const vec3 R_ANKLE_POS = { 0.0,LEG_W * 0.5, ANKLE_H};
const vec3 L_ANKLE_POS = { 0.0,-LEG_W * 0.5, ANKLE_H};
const vec3 a4= { -HEEL_LEN,0.0, 0.0};
const vec3 a5= { -FOOT_LEN,0.0, 0.0};
const vec3 R_HEEL_POS = R_ANKLE_POS-a4;
const vec3 L_HEEL_POS = L_ANKLE_POS-a4;
const vec3 R_TOES_POS = R_ANKLE_POS+a5;
const vec3 L_TOES_POS = L_ANKLE_POS+a5;

const vec3 rightAxis = { 0.0,1.0, 0.0};
const vec3 leftAxis = { 0.0,-1.0, 0.0};
const vec3 upAxis = { 0.0,0.0, 1.0};
const vec3 downAxis = { 0.0,0.0, -1.0};
const vec3 bkwdAxis = { 1.0,0.0, 0.0};
const vec3 fwdAxis = { -1.0,0.0, 0.0};

Ragdoll::Ragdoll (dWorldID _world, dSpaceID _space, dReal _density, vec3 _offset)
{
    world=_world;
    space=_space;
    density=_density;
    offset=_offset;

    chest = addBody({ 0.0,-CHEST_W * 0.5, CHEST_H}, { 0.0,CHEST_W * 0.5, CHEST_H}, 0.13);
    belly = addBody({ 0.0,0.0, CHEST_H - 0.1}, { 0.0,0.0, HIP_H + 0.1}, 0.125);
    midSpine = addFixedJoint(chest, belly);
    pelvis = addBody({ 0.0,-PELVIS_W * 0.5, HIP_H}, { 0.0,PELVIS_W * 0.5, HIP_H}, 0.125);
    lowSpine = addFixedJoint(belly, pelvis);

    head = addBody({ 0.0,0.0, BROW_H}, { 0.0,0.0, MOUTH_H}, 0.11);
    neck = addBallJoint(chest, head, { 0.0,0.0, NECK_H});
    neckMotor = addAMotor(chest, head);

    rightUpperLeg = addBody(R_HIP_POS, R_KNEE_POS, 0.11);
    rightHip = addUniversalJoint(pelvis, rightUpperLeg,R_HIP_POS, -fwdAxis, -rightAxis, -0.1*M_PI, 0.3*M_PI, -0.15*M_PI,0.75*M_PI);
    leftUpperLeg = addBody(L_HIP_POS, L_KNEE_POS, 0.11);
    leftHip = addUniversalJoint(pelvis, leftUpperLeg,L_HIP_POS, -fwdAxis, -rightAxis, -0.1*M_PI, 0.3*M_PI, -0.15*M_PI,0.75*M_PI);

    rightLowerLeg = addBody(R_KNEE_POS, R_ANKLE_POS, 0.09);
    rightKnee = addHingeJoint(rightUpperLeg,rightLowerLeg, R_KNEE_POS, -leftAxis, 0.0, M_PI*0.75);
    leftLowerLeg = addBody(L_KNEE_POS, L_ANKLE_POS, 0.09);
    leftKnee = addHingeJoint(leftUpperLeg,leftLowerLeg, L_KNEE_POS, -leftAxis, 0.0, M_PI*0.75);

    rightFoot = addBody(R_HEEL_POS, R_TOES_POS, 0.09);
    rightAnkle = addHingeJoint(rightLowerLeg,rightFoot, R_ANKLE_POS, -rightAxis, -0.1*M_PI, 0.05*M_PI);
    leftFoot = addBody(L_HEEL_POS, L_TOES_POS, 0.09);
    leftAnkle = addHingeJoint(leftLowerLeg, leftFoot, L_ANKLE_POS,  -rightAxis, -0.1*M_PI, 0.05*M_PI);

    rightUpperArm = addBody(R_SHOULDER_POS, R_ELBOW_POS, 0.08);
    rightShoulder = addBallJoint(chest, rightUpperArm, R_SHOULDER_POS);
    rightShoulderMotor = addAMotor(chest, rightUpperArm);
    leftUpperArm = addBody(L_SHOULDER_POS, L_ELBOW_POS, 0.08);
    leftShoulder = addBallJoint(chest, leftUpperArm,L_SHOULDER_POS);
    leftShoulderMotor = addAMotor(chest, leftUpperArm);

    rightForeArm = addBody(R_ELBOW_POS, R_WRIST_POS, 0.075);
    rightElbow = addHingeJoint(rightUpperArm, rightForeArm, R_ELBOW_POS, downAxis, 0.0, 0.6*M_PI);
    leftForeArm = addBody(L_ELBOW_POS, L_WRIST_POS, 0.075);
    leftElbow = addHingeJoint(leftUpperArm, leftForeArm, L_ELBOW_POS, upAxis, 0.0, 0.6*M_PI);

    rightHand = addBody(R_WRIST_POS, R_FINGERS_POS, 0.075);
    rightWrist = addHingeJoint(rightForeArm, rightHand, R_WRIST_POS, -fwdAxis, -0.1*M_PI, 0.2*M_PI);
    leftHand = addBody(L_WRIST_POS, L_FINGERS_POS, 0.075);
    leftWrist = addHingeJoint(leftForeArm,leftHand, L_WRIST_POS, -bkwdAxis, -0.1*M_PI, 0.2*M_PI);
}

dBodyID Ragdoll::addBody(vec3 p1,vec3 p2,num_type radius)
{
    p1 = p1+offset;
    p2 = p2+offset;
    num_type cyllen = dist3(p1, p2) - radius;

    dBodyID body = dBodyCreate(world);
    dMass m;
    dMassSetZero(&m);
    dMassSetCapsule(&m, density, 3, radius, cyllen);
    dBodySetMass(body, &m);

    vec3 center_of_mass=0.5*(p1+p2);
    dBodySetPosition(body, center_of_mass[0], center_of_mass[1], center_of_mass[2]);

    dGeomID geom=dCreateCapsule(space, radius, cyllen);
    dGeomSetBody(geom, body);

    vec3 za=unit3(p2-p1);
    vec3 xa= {0.0, 0.0, 1.0};
    vec3 vec= {0.0, 1.0, 0.0};
    if (fabs(dot3(vec,za))<0.7)
    {
        xa=vec;
    }
    vec3 ya=cross(za, xa);
    xa=unit3(cross(ya, za));
    dMatrix3 rot;
    dRSetIdentity(rot);
    dRFrom2Axes(rot, xa[0], xa[1], xa[2], ya[0], ya[1], ya[2]);
    dBodySetRotation(body, rot);
    total_mass+=m.mass;
    body_parts[b].body = body;
    body_parts[b].geom = geom;
    body_parts[b].length = cyllen;
    body_parts[b].radius = radius;
    b++;
    return body;
}

dJointID Ragdoll::addFixedJoint(dBodyID body1, dBodyID body2)
{
    dJointGroupID group_joint =dJointGroupCreate (0);
    dJointID joint=dJointCreateFixed(world, group_joint);
    dJointAttach(joint, body1, body2);
    dJointSetFixed(joint);
    joints[j]=joint;
    j++;
    return joint;
}

dJointID Ragdoll::addBallJoint(dBodyID body1, dBodyID body2, vec3 anchor)
{
    dJointGroupID group_joint =dJointGroupCreate (0); //0cf doc ode
    dJointID joint=dJointCreateBall(world, group_joint);
    dJointAttach(joint, body1, body2);
    dJointSetBallAnchor(joint,anchor[0]+offset[0], anchor[1]+offset[1], anchor[2]+offset[2]);
    joints[j]=joint;
    j++;
    return joint;
}

dJointID Ragdoll::addAMotor(dBodyID body1, dBodyID body2)
{
    dJointID motor = dJointCreateAMotor(world, 0); // allocate joint normally

    dJointAttach(motor, body1, body2); // attach the two bodies with this joint
    dJointSetAMotorMode(motor, dAMotorUser);

    dJointSetAMotorNumAxes(motor, 3);
    dJointSetAMotorAxis(motor, 0, 0, bkwdAxis[0], bkwdAxis[1], bkwdAxis[2]);
    dJointSetAMotorAxis(motor, 1, 0, rightAxis[0], rightAxis[1], rightAxis[2]);
    dJointSetAMotorAxis(motor, 2, 0, upAxis[0], upAxis[1], upAxis[2]);

    return motor;
}

dJointID Ragdoll::addHingeJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis, dReal _loStop, dReal _hiStop)
{
    anchor=anchor+offset;
    dJointGroupID group_joint =dJointGroupCreate (0);
    dJointID joint=dJointCreateHinge(world, group_joint);
    dJointAttach(joint, body1, body2);
    dJointSetHingeAnchor(joint, anchor[0], anchor[1], anchor[2]);
    dJointSetHingeAxis(joint, axis[0], axis[1], axis[2]);
    dJointSetHingeParam(joint, dParamLoStop, _loStop);
    dJointSetHingeParam(joint, dParamHiStop, _hiStop);
    joints[j]=joint;
    j++;
    return joint;
}

dJointID Ragdoll::addHingeJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis)
{
    anchor=anchor+offset;
    dJointGroupID group_joint =dJointGroupCreate (0);
    dJointID joint=dJointCreateHinge(world, group_joint);
    dJointAttach(joint, body1, body2);
    dJointSetHingeAnchor(joint, anchor[0], anchor[1], anchor[2]);
    dJointSetHingeAxis(joint, axis[0], axis[1], axis[2]);
    dJointSetHingeParam(joint, dParamLoStop, -dInfinity);
    dJointSetHingeParam(joint, dParamHiStop, dInfinity);
    joints[j]=joint;
    j++;
    return joint;
}

dJointID Ragdoll::addUniversalJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis1, vec3 axis2, dReal _loStop1, dReal _hiStop1, dReal _loStop2, dReal _hiStop2)
{
    anchor=anchor+offset;
    dJointGroupID group_joint =dJointGroupCreate (0);
    dJointID joint=dJointCreateUniversal(world, group_joint);
    dJointAttach(joint, body1, body2);
    dJointSetUniversalAnchor(joint, anchor[0], anchor[1], anchor[2]);
    dJointSetUniversalAxis1(joint, axis1[0], axis1[1], axis1[2]);
    dJointSetUniversalAxis2(joint, axis2[0], axis2[1], axis2[2]);
    dJointSetUniversalParam(joint, dParamLoStop1, _loStop1);
    dJointSetUniversalParam(joint, dParamHiStop1, _hiStop1);
    dJointSetUniversalParam(joint, dParamLoStop2, _loStop2);
    dJointSetUniversalParam(joint, dParamHiStop2, _hiStop2);
    joints[j]=joint;
    j++;
    return joint;
}

dJointID Ragdoll::addUniversalJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis1, vec3 axis2)
{
    anchor=anchor+offset;
    dJointGroupID group_joint =dJointGroupCreate (0);
    dJointID joint=dJointCreateUniversal(world, group_joint);
    dJointAttach(joint, body1, body2);
    dJointSetUniversalAnchor(joint, anchor[0], anchor[1], anchor[2]);
    dJointSetUniversalAxis1(joint, axis1[0], axis1[1], axis1[2]);
    dJointSetUniversalAxis2(joint, axis2[0], axis2[1], axis2[2]);
    dJointSetUniversalParam(joint, dParamLoStop, -dInfinity);
    dJointSetUniversalParam(joint, dParamHiStop, dInfinity);
    dJointSetUniversalParam(joint, dParamLoStop1, -dInfinity);
    dJointSetUniversalParam(joint, dParamHiStop1, dInfinity);
    joints[j]=joint;
    j++;
    return joint;
}

void Ragdoll::draw()
{
    dsSetColor(0.5, 0.2, 0.2);
    const dReal *R, *pos; // rotation matrix and position vector of the body part
    for(int i=0; i<16; ++i)
    {
        pos = dBodyGetPosition(body_parts[i].body);
        R = dBodyGetRotation(body_parts[i].body);

        dsDrawCapsuleD(pos, R, body_parts[i].length, body_parts[i].radius);
    }
}

void Ragdoll::setTargetAngles(dReal* angles, dReal error)
{
    int i = 0;
    dReal F_MAX = 1000;

    // dJointSetAMotorParam
    dJointSetAMotorParam(neckMotor, dParamLoStop1, angles[i]); //0
    dJointSetAMotorParam(neckMotor, dParamHiStop1, angles[i++]+error);
    dJointSetAMotorParam(neckMotor, dParamLoStop2, angles[i]); //1
    dJointSetAMotorParam(neckMotor, dParamHiStop2, angles[i++]+error);
    dJointSetAMotorParam(neckMotor, dParamLoStop3, angles[i]); //2
    dJointSetAMotorParam(neckMotor, dParamHiStop3, angles[i++]+error);
    dJointSetAMotorParam(neckMotor, dParamFMax1, F_MAX);
    dJointSetAMotorParam(neckMotor, dParamFMax2, F_MAX);
    dJointSetAMotorParam(neckMotor, dParamFMax3, F_MAX);

    // hips
    dJointSetUniversalParam(rightHip, dParamLoStop1, angles[i]); //3
    dJointSetUniversalParam(rightHip, dParamHiStop1, angles[i++]+error);
    dJointSetUniversalParam(rightHip, dParamLoStop2, angles[i]); //4
    dJointSetUniversalParam(rightHip, dParamHiStop2, angles[i++]+error);
    dJointSetUniversalParam(rightHip, dParamFMax1, F_MAX);
    dJointSetUniversalParam(rightHip, dParamFMax2, F_MAX);

    dJointSetUniversalParam(leftHip, dParamLoStop1, angles[i]); //5
    dJointSetUniversalParam(leftHip, dParamHiStop1, angles[i++]+error);
    dJointSetUniversalParam(leftHip, dParamLoStop2, angles[i]); //6
    dJointSetUniversalParam(leftHip, dParamHiStop2, angles[i++]+error);
    dJointSetUniversalParam(leftHip, dParamFMax1, F_MAX);
    dJointSetUniversalParam(leftHip, dParamFMax2, F_MAX);

    // knees
    dJointSetHingeParam(rightKnee, dParamLoStop1, angles[i]); //7
    dJointSetHingeParam(rightKnee, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(rightKnee, dParamFMax, F_MAX);

    dJointSetHingeParam(leftKnee, dParamLoStop1, angles[i]); //8
    dJointSetHingeParam(leftKnee, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(leftKnee, dParamFMax, F_MAX);

    // ankles
    dJointSetHingeParam(rightAnkle, dParamLoStop1, angles[i]); //9
    dJointSetHingeParam(rightAnkle, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(rightAnkle, dParamFMax, F_MAX);

    dJointSetHingeParam(leftAnkle, dParamLoStop1, angles[i]); //10
    dJointSetHingeParam(leftAnkle, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(leftAnkle, dParamFMax, F_MAX);

    // shoulders
    dJointSetAMotorParam(rightShoulderMotor, dParamLoStop1, angles[i]); //11
    dJointSetAMotorParam(rightShoulderMotor, dParamHiStop1, angles[i++]+error);
    dJointSetAMotorParam(rightShoulderMotor, dParamLoStop2, angles[i]); //12
    dJointSetAMotorParam(rightShoulderMotor, dParamHiStop2, angles[i++]+error);
    dJointSetAMotorParam(rightShoulderMotor, dParamLoStop3, angles[i]); //13
    dJointSetAMotorParam(rightShoulderMotor, dParamHiStop3, angles[i++]+error);
    dJointSetAMotorParam(rightShoulderMotor, dParamFMax1, F_MAX);
    dJointSetAMotorParam(rightShoulderMotor, dParamFMax2, F_MAX);
    dJointSetAMotorParam(rightShoulderMotor, dParamFMax3, F_MAX);

    dJointSetAMotorParam(leftShoulderMotor, dParamLoStop1, angles[i]); //14
    dJointSetAMotorParam(leftShoulderMotor, dParamHiStop1, angles[i++]+error);
    dJointSetAMotorParam(leftShoulderMotor, dParamLoStop2, angles[i]); //15
    dJointSetAMotorParam(leftShoulderMotor, dParamHiStop2, angles[i++]+error);
    dJointSetAMotorParam(leftShoulderMotor, dParamLoStop3, angles[i]); //16
    dJointSetAMotorParam(leftShoulderMotor, dParamHiStop3, angles[i++]+error);
    dJointSetAMotorParam(leftShoulderMotor, dParamFMax1, F_MAX);
    dJointSetAMotorParam(leftShoulderMotor, dParamFMax2, F_MAX);
    dJointSetAMotorParam(leftShoulderMotor, dParamFMax3, F_MAX);

    // elbows
    dJointSetHingeParam(rightElbow, dParamLoStop1, angles[i]); //17
    dJointSetHingeParam(rightElbow, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(rightElbow, dParamFMax, F_MAX);

    dJointSetHingeParam(leftElbow, dParamLoStop1, angles[i]); //18
    dJointSetHingeParam(leftElbow, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(leftElbow, dParamFMax, F_MAX);

    // wrists
    dJointSetHingeParam(rightWrist, dParamLoStop1, angles[i]); //19
    dJointSetHingeParam(rightWrist, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(rightWrist, dParamFMax, F_MAX);

    dJointSetHingeParam(leftWrist, dParamLoStop1, angles[i]); //20
    dJointSetHingeParam(leftWrist, dParamHiStop1, angles[i++]+error);
    dJointSetHingeParam(leftWrist, dParamFMax, F_MAX);
}

float Ragdoll::cost(ClimbingWall* wallptr, Stance target_stance)
{
    float c = 0;

    // 1. Distance to the target stance
    float square_dist = 0;
    if (target_stance.lh_hold != -1)
    {
        vec3 pos = pos_end_bodypart(leftHand, leftWrist);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.lh_hold],pos), 2);
    }
    if (target_stance.rh_hold != -1)
    {
        vec3 pos = pos_end_bodypart(rightHand, rightWrist);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.rh_hold],pos), 2);
    }
    if (target_stance.lf_hold != -1)
    {
        vec3 pos = pos_end_bodypart(leftFoot, leftAnkle);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.lf_hold],pos), 2);
    }
    if (target_stance.rf_hold != -1)
    {
        vec3 pos = pos_end_bodypart(rightFoot, rightAnkle);
        square_dist += std::pow(dist3(wallptr->holds[target_stance.rf_hold],pos), 2);
    }
    c += square_dist / (K_SIGMA*K_SIGMA);

    // 2. Distance to the wall
    vec3 barycentre = {0,0,0};
    dReal mass = 0.0;
    for(int i = 0; i < 16; i++)
    {
        const dReal* pos = dBodyGetPosition(body_parts[i].body);
        dReal mass_part = body_parts[i].length * body_parts[i].radius * body_parts[i].radius;
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
