//#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "../include/util.h"
#include "../include/body.h"

const dReal UPPER_ARM_LEN=0.30;
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

const vec3 R_SHOULDER_POS = { 0.0,-SHOULDER_W * 0.5, SHOULDER_H};
const vec3 L_SHOULDER_POS = { 0.0,SHOULDER_W * 0.5, SHOULDER_H};
const vec3 a1={ 0.0,UPPER_ARM_LEN, 0.0};
const vec3 a2={ 0.0,FORE_ARM_LEN, 0.0};
const vec3 a3={ 0.0,HAND_LEN, 0.0};
const vec3 R_ELBOW_POS = R_SHOULDER_POS-a1;
const vec3 L_ELBOW_POS = L_SHOULDER_POS+a1;
const vec3 R_WRIST_POS = R_ELBOW_POS-a2;
const vec3 L_WRIST_POS = L_ELBOW_POS+a2;
const vec3 R_FINGERS_POS = R_WRIST_POS-a3;
const vec3 L_FINGERS_POS = L_WRIST_POS+a3;

const vec3 R_HIP_POS = { 0.0,-LEG_W * 0.5, HIP_H};
const vec3 L_HIP_POS = { 0.0,LEG_W * 0.5, HIP_H};
const vec3 R_KNEE_POS = { 0.0,-LEG_W * 0.5, KNEE_H};
const vec3 L_KNEE_POS = { 0.0,LEG_W * 0.5, KNEE_H};
const vec3 R_ANKLE_POS = { 0.0,-LEG_W * 0.5, ANKLE_H};
const vec3 L_ANKLE_POS = { 0.0,LEG_W * 0.5, ANKLE_H};
const vec3 a4={ HEEL_LEN,0.0, 0.0};
const vec3 a5={ FOOT_LEN,0.0, 0.0};
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

    rightUpperLeg = addBody(R_HIP_POS, R_KNEE_POS, 0.11);
    rightHip = addUniversalJoint(pelvis, rightUpperLeg,R_HIP_POS, fwdAxis, rightAxis, -0.1*M_PI, 0.3*M_PI, -0.15*M_PI,0.75*M_PI);
    leftUpperLeg = addBody(L_HIP_POS, L_KNEE_POS, 0.11);
    leftHip = addUniversalJoint(pelvis, leftUpperLeg,L_HIP_POS, fwdAxis, rightAxis, -0.1*M_PI, 0.3*M_PI, -0.15*M_PI,0.75*M_PI);

    rightLowerLeg = addBody(R_KNEE_POS, R_ANKLE_POS, 0.09);
    rightKnee = addHingeJoint(rightUpperLeg,rightLowerLeg, R_KNEE_POS, leftAxis, 0.0, M_PI*0.75);
    leftLowerLeg = addBody(L_KNEE_POS, L_ANKLE_POS, 0.09);
    leftKnee = addHingeJoint(leftUpperLeg,leftLowerLeg, L_KNEE_POS, leftAxis, 0.0, M_PI*0.75);

    rightFoot = addBody(R_HEEL_POS, R_TOES_POS, 0.09);
    rightAnkle = addHingeJoint(rightLowerLeg,rightFoot, R_ANKLE_POS, rightAxis, -0.1*M_PI, 0.05*M_PI);
    leftFoot = addBody(L_HEEL_POS, L_TOES_POS, 0.09);
    leftAnkle = addHingeJoint(leftLowerLeg, leftFoot, L_ANKLE_POS,  rightAxis, -0.1*M_PI, 0.05*M_PI);

    rightUpperArm = addBody(R_SHOULDER_POS, R_ELBOW_POS, 0.08);
    rightShoulder = addBallJoint(chest, rightUpperArm, R_SHOULDER_POS);
    leftUpperArm = addBody(L_SHOULDER_POS, L_ELBOW_POS, 0.08);
    leftShoulder = addBallJoint(chest, leftUpperArm,L_SHOULDER_POS);

    rightForeArm = addBody(R_ELBOW_POS, R_WRIST_POS, 0.075);
    rightElbow = addHingeJoint(rightUpperArm, rightForeArm, R_ELBOW_POS, downAxis, 0.0, 0.6*M_PI);
    leftForeArm = addBody(L_ELBOW_POS, L_WRIST_POS, 0.075);
    leftElbow = addHingeJoint(leftUpperArm, leftForeArm, L_ELBOW_POS, upAxis, 0.0, 0.6*M_PI);

    rightHand = addBody(R_WRIST_POS, R_FINGERS_POS, 0.075);
    rightWrist = addHingeJoint(rightForeArm, rightHand, R_WRIST_POS, fwdAxis, -0.1*M_PI, 0.2*M_PI);
    leftHand = addBody(L_WRIST_POS, L_FINGERS_POS, 0.075);
    leftWrist = addHingeJoint(leftForeArm,leftHand, L_WRIST_POS, bkwdAxis, -0.1*M_PI, 0.2*M_PI);
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
    std::cout << "l. 223 rang : " << j;
    joints[j]=joint;
    std::cout << " ok" << std::endl;
    j++;
    return joint;
    //ajouter joint � joints
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
