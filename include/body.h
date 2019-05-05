#ifndef BODY_H_INCLUDED
#define BODY_H_INCLUDED
#include <iostream>
#include "../include/util.h"
#include "../include/wall.h"

typedef struct {
  dBodyID body;
  dGeomID geom;
  dReal length;
  dReal radius;
} body_part;


class Ragdoll{
public:
    dWorldID world;
    dSpaceID space;
    dReal density;
    body_part body_parts[16];
    dJointID joints[15];
    dReal total_mass = 0.0;
    vec3 offset; //=(0.0, 0.0, 0.0)
    int b=0;
    int j=0;


    dBodyID chest;
	dBodyID belly;
	dJointID midSpine;
	dBodyID pelvis;
	dJointID lowSpine;

	dBodyID head;
	dJointID neck;
	dJointID neckMotor;

	dBodyID rightUpperLeg;
	dJointID rightHip;
	dBodyID leftUpperLeg;
	dJointID leftHip;

	dBodyID rightLowerLeg;
	dJointID rightKnee;
	dBodyID leftLowerLeg;
	dJointID leftKnee;

	dBodyID rightFoot;
	dJointID rightAnkle;
	dBodyID leftFoot;
	dJointID leftAnkle;

	dBodyID rightUpperArm;
	dJointID rightShoulder;
	dJointID rightShoulderMotor;
	dBodyID leftUpperArm;
	dJointID leftShoulder;
	dJointID leftShoulderMotor;

	dBodyID rightForeArm;
	dJointID rightElbow;
	dBodyID leftForeArm;
	dJointID leftElbow;

	dBodyID rightHand;
	dJointID rightWrist;
	dBodyID leftHand;
	dJointID leftWrist;

    Ragdoll (dWorldID _world, dSpaceID _space, dReal _density, vec3 _offset);
    dBodyID addBody(vec3 p1,vec3 p2,num_type radius);
    dJointID addFixedJoint(dBodyID body1, dBodyID body2);
    dJointID addBallJoint(dBodyID body1, dBodyID body2, vec3 anchor);
    dJointID addAMotor(dBodyID body1, dBodyID body2);
    dJointID addHingeJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis, dReal _loStop, dReal _hiStop);
    dJointID addHingeJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis);
    dJointID addUniversalJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis1, vec3 axis2, dReal _loStop1, dReal _loStop2, dReal _hiStop1, dReal _hiStop2);
    dJointID addUniversalJoint(dBodyID body1, dBodyID body2, vec3 anchor, vec3 axis1, vec3 axis2);
    void draw();
    void setTargetAngles(dReal* angles, dReal error);

    float cost(ClimbingWall* wallptr, Stance target_stance);
};

vec3 pos_end_bodypart(dBodyID body, dJointID joint);

#endif // BODY_H_INCLUDED
