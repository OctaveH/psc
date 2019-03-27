//#include <iostream>
#include <array>
#include <cmath>
#include <ode/ode.h>
#include "../include/util.h"
#include "../include/body.h"  //pb avec le include

const dReal UPPER_ARM_LEN=0.30; //dReal ou num_type?
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

const vec3 R_SHOULDER_POS = {-SHOULDER_W * 0.5, SHOULDER_H, 0.0};
const vec3 L_SHOULDER_POS = {SHOULDER_W * 0.5, SHOULDER_H, 0.0};
const vec3 a1={UPPER_ARM_LEN, 0.0, 0.0};
const vec3 a2={FORE_ARM_LEN, 0.0, 0.0};
const vec3 a3={HAND_LEN, 0.0, 0.0};
const vec3 R_ELBOW_POS = R_SHOULDER_POS-a1;
const vec3 L_ELBOW_POS = L_SHOULDER_POS+a1;
const vec3 R_WRIST_POS = R_ELBOW_POS-a2;
const vec3 L_WRIST_POS = L_ELBOW_POS+a2;
const vec3 R_FINGERS_POS = R_WRIST_POS-a3;
const vec3 L_FINGERS_POS = L_WRIST_POS+a3;

const vec3 R_HIP_POS = {-LEG_W * 0.5, HIP_H, 0.0};
const vec3 L_HIP_POS = {LEG_W * 0.5, HIP_H, 0.0};
const vec3 R_KNEE_POS = {-LEG_W * 0.5, KNEE_H, 0.0};
const vec3 L_KNEE_POS = {LEG_W * 0.5, KNEE_H, 0.0};
const vec3 R_ANKLE_POS = {-LEG_W * 0.5, ANKLE_H, 0.0};
const vec3 L_ANKLE_POS = {LEG_W * 0.5, ANKLE_H, 0.0};
const vec3 a4={0.0, 0.0, HEEL_LEN};
const vec3 a5={0.0, 0.0, FOOT_LEN};
const vec3 R_HEEL_POS = R_ANKLE_POS-a4;
const vec3 L_HEEL_POS = L_ANKLE_POS-a4;
const vec3 R_TOES_POS = R_ANKLE_POS+a5;
const vec3 L_TOES_POS = L_ANKLE_POS+a5;

/*
problème de type: quand est ce qu'on utilise dReal, dMat3, dVector3?
A faire remplir les tableaux geoms et bodies (dans addBody()) et joints (dans setJoint())
Faire le headerdile
*/



    Ragdoll::Ragdoll (dWorldID _world, dSpaceID _space, dReal _density, vec3 _offset){

        world=_world;
        space=_space;
        density=_density;
        offset=_offset;

		chest = addBody({-CHEST_W * 0.5, CHEST_H, 0.0},{CHEST_W * 0.5, CHEST_H, 0.0}, 0.13);
		belly = addBody({0.0, CHEST_H - 0.1, 0.0},{0.0, HIP_H + 0.1, 0.0}, 0.125);
		midSpine = addFixedJoint(chest, belly);
		pelvis = addBody({-PELVIS_W * 0.5, HIP_H, 0.0},{PELVIS_W * 0.5, HIP_H, 0.0}, 0.125);
		lowSpine = addFixedJoint(belly, pelvis);

		head = addBody({0.0, BROW_H, 0.0}, {0.0, MOUTH_H, 0.0}, 0.11);
		neck = addBallJoint(chest, head,{0.0, NECK_H, 0.0});

		rightUpperLeg = addBody(R_HIP_POS, R_KNEE_POS, 0.11);
		rightHip = addBallJoint(pelvis, rightUpperLeg,R_HIP_POS);
		leftUpperLeg = addBody(L_HIP_POS, L_KNEE_POS, 0.11);
		leftHip = addBallJoint(pelvis, leftUpperLeg,L_HIP_POS);

		rightLowerLeg = addBody(R_KNEE_POS, R_ANKLE_POS, 0.09);
		rightKnee = addBallJoint(rightUpperLeg,rightLowerLeg, R_KNEE_POS);
		leftLowerLeg = addBody(L_KNEE_POS, L_ANKLE_POS, 0.09);
		leftKnee = addBallJoint(leftUpperLeg,leftLowerLeg, L_KNEE_POS);

		rightFoot = addBody(R_HEEL_POS, R_TOES_POS, 0.09);
		rightAnkle = addBallJoint(rightLowerLeg,rightFoot, R_ANKLE_POS);
		leftFoot = addBody(L_HEEL_POS, L_TOES_POS, 0.09);
		leftAnkle = addBallJoint(leftLowerLeg, leftFoot, L_ANKLE_POS);

		rightUpperArm = addBody(R_SHOULDER_POS, R_ELBOW_POS, 0.08);
		rightShoulder = addBallJoint(chest, rightUpperArm, R_SHOULDER_POS);
		leftUpperArm = addBody(L_SHOULDER_POS, L_ELBOW_POS, 0.08);
		leftShoulder = addBallJoint(chest, leftUpperArm,L_SHOULDER_POS);

		rightForeArm = addBody(R_ELBOW_POS, R_WRIST_POS, 0.075);
		rightElbow = addBallJoint(rightUpperArm, rightForeArm, R_ELBOW_POS);
        leftForeArm = addBody(L_ELBOW_POS, L_WRIST_POS, 0.075);
		leftElbow = addBallJoint(leftUpperArm, leftForeArm, L_ELBOW_POS);

		rightHand = addBody(R_WRIST_POS, R_FINGERS_POS, 0.075);
		rightWrist = addBallJoint(rightForeArm, rightHand, R_WRIST_POS);
		leftHand = addBody(L_WRIST_POS, L_FINGERS_POS, 0.075);
		leftWrist = addBallJoint(leftForeArm,leftHand, L_WRIST_POS);
    }

    dBodyID Ragdoll::addBody(vec3 p1,vec3 p2,num_type radius){
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

        dGeomID geom=dCreateCylinder(space, radius, cyllen);
        dGeomSetBody(geom, body);

        vec3 za=unit3(p2-p1);
        vec3 xa={0.0, 1.0, 0.0};
        vec3 vec={1.0, 0.0, 0.0};
        if (fabs(dot3(vec,za))<0.7) {xa=vec;}
        vec3 ya=cross(za, xa);
        xa=unit3(cross(ya, za));
        dReal rot[12]={xa[0], ya[0], za[0], xa[1], ya[1], za[1], xa[2], ya[2], za[2], 0.0, 0.0, 0.0};
		dBodySetRotation(body, rot);
		totalMass+=m.mass;
		geoms[b]=geom;
		bodies[b]=body;
		b++;
		return body;

    }

   /*
	dBody addBody(self, p1, p2, radius):
		"""
		Adds a capsule body between joint positions p1 and p2 and with given
		radius to the ragdoll.
		"""

		p1 = add3(p1, self.offset)
		p2 = add3(p2, self.offset)

		# cylinder length not including endcaps, make capsules overlap by half
		#   radius at joints
		cyllen = dist3(p1, p2) - radius

		body = ode.Body(self.world)
		m = ode.Mass()
		m.setCappedCylinder(self.density, 3, radius, cyllen)
		body.setMass(m)

		# set parameters for drawing the body   !!!!!!je sas pas trop à quoi ça correspond (mais je
		body.shape = "capsule"                        pense qu'on en a pas besoin!!!!!!!
		body.length = cyllen
		body.radius = radius

		# create a capsule geom for collision detection
		geom = ode.GeomCCylinder(self.space, radius, cyllen)
		geom.setBody(body)

		# define body rotation automatically from body axis
		za = norm3(sub3(p2, p1))
		if (abs(dot3(za, (1.0, 0.0, 0.0))) < 0.7): xa = (1.0, 0.0, 0.0)
		else: xa = (0.0, 1.0, 0.0)
		ya = cross(za, xa)
		xa = norm3(cross(ya, za))
		ya = cross(za, xa)
		rot = (xa[0], ya[0], za[0], xa[1], ya[1], za[1], xa[2], ya[2], za[2])

		body.setPosition(mul3(add3(p1, p2), 0.5))
		body.setRotation(rot)

		self.bodies.append(body)
		self.geoms.append(geom)

		self.totalMass += body.getMass().mass

		return body
		*/

	/*def addFixedJoint(self, body1, body2):
		joint = ode.FixedJoint(self.world)
		joint.attach(body1, body2)
		joint.setFixed()

		joint.style = "fixed"  //c'est quoi le style???
		self.joints.append(joint)

		return joint*/

    dJointID Ragdoll::addFixedJoint(dBodyID body1, dBodyID body2){
        dJointGroupID group_joint =dJointGroupCreate (0); //0cf doc ode
        dJointID joint=dJointCreateFixed(world, group_joint);
        dJointAttach(joint, body1, body2);
        //il faut rajouter le joint à joint
        joints[j]=joint;
        j++;
        return joint;
    }

    dJointID Ragdoll::addBallJoint(dBodyID body1, dBodyID body2, vec3 anchor){
        dJointGroupID group_joint =dJointGroupCreate (0); //0cf doc ode
        dJointID joint=dJointCreateBall(world, group_joint);
        dJointAttach(joint, body1, body2);
        dJointSetBallAnchor(joint,anchor[0], anchor[1], anchor[2]);
        joints[j]=joint;
        j++;
        return joint;
        //ajouter joint à joints
    }
	/*def addBallJoint(self, body1, body2, anchor):

		anchor = add3(anchor, self.offset)

		# create the joint
		joint = ode.BallJoint(self.world)
		joint.attach(body1, body2)
		joint.setAnchor(anchor)

		joint.style = "ball"
		self.joints.append(joint)

		return joint*/

