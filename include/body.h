#ifndef BODY_H_INCLUDED
#define BODY_H_INCLUDED
#include <iostream>
//#include <array>
//#include <cmath>
#include "../include/util.h"

class Ragdoll{
public:
    //const static int n=20;
    dWorldID world;
    dSpaceID space;
    num_type density;
    dBodyID bodies[16];
    dGeomID geoms[16];
    dJointID joints[15];
    num_type totalMass = 0.0;
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
	dBodyID leftUpperArm;
	dJointID leftShoulder;

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
};

#endif // BODY_H_INCLUDED
