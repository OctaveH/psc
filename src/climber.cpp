//#include "../include/climber.h"
//#include "../include/climber_info.h"

#include <math.h>

#include "../include/vec3.h"
#include "../include/climber_info.h"
#include "../include/climber.h"

Climber::Climber(dWorldID _world, dSpaceID _space) {
    world = _world;
    space = _space;
}

dBodyID Climber::addCapsuleWithMass(const dVector3& p1, const dVector3& p2, dReal radius, dReal mass) {
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

	geoms[bodyc] = geom; // add geom to geom array
	bodies[bodyc] = capsule; // add capsule to body array
	++bodyc;

	return capsule;
}

dBodyID Climber::addCapsule(const dVector3& p1, const dVector3& p2, dReal radius) {
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
	dMassSetCapsule(&m, density, 3, radius, cap_len);
	dBodySetMass(capsule, &m);
    total_mass += m.mass; // keep track of the total mass

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

	geoms[bodyc] = geom; // add geom to geom array
	bodies[bodyc] = capsule; // add capsule to body array
	++bodyc;

	return capsule;
}

dJointID Climber::addFixedJoint(dBodyID body1, dBodyID body2) {
    dJointID joint = dJointCreateFixed(world, 0); // allocate joint normally
        
    dJointAttach(joint, body1, body2); // attach the two bodies with this joint
    dJointSetFixed(joint); // set the joint to fixed
        
    joints[jointc++]=joint; // add to joint array

    return joint;
}

dJointID Climber::addBallJoint(dBodyID body1, dBodyID body2, dVector3 anchor){
    dJointID joint=dJointCreateBall(world, 0); // allocate joint normally

    dJointAttach(joint, body1, body2); // attach the two bodies with this joint
    dJointSetBallAnchor(joint, anchor[0], anchor[1], anchor[2]); // set this joint to ball anchor

    joints[jointc++] = joint; // add the joint to the array

    return joint;
}

void Climber::addClimber(dVector3 _offset) {
    // set the offset that will be used to place all parts
    offset = vec3(_offset);
    density = 1.0; // temporary use of homogeneous density

    // create all body parts
    head = addCapsule(HEAD_1.vec, HEAD_2.vec, HEAD_RADIUS);

	hbody = addCapsule(HBODY_1.vec, HBODY_2.vec, HBODY_RADIUS);
    lbody = addCapsule(LBODY_1.vec, LBODY_2.vec, LBODY_RADIUS);

    arm_right = addCapsule(ARM_RIGHT_1.vec, ARM_RIGHT_2.vec, ARM_RADIUS);
    arm_left = addCapsule(ARM_LEFT_1.vec, ARM_LEFT_2.vec, ARM_RADIUS);

    forearm_right = addCapsule(FOREARM_RIGHT_1.vec, FOREARM_RIGHT_2.vec, FOREARM_RADIUS);
    forearm_left = addCapsule(FOREARM_LEFT_1.vec, FOREARM_LEFT_1.vec, FOREARM_RADIUS);

    hand_right = addCapsule(HAND_RIGHT_1.vec, HAND_RIGHT_2.vec, HAND_RADIUS);
    hand_left = addCapsule(HAND_LEFT_1.vec, HAND_LEFT_2.vec, HAND_RADIUS);

    thigh_right = addCapsule(THIGH_RIGHT_1.vec, THIGH_RIGHT_2.vec, THIGH_RADIUS);
    thigh_left = addCapsule(THIGH_LEFT_1.vec, THIGH_LEFT_2.vec, THIGH_RADIUS);
    
    leg_right = addCapsule(LEG_RIGHT_1.vec, LEG_RIGHT_2.vec, LEG_RADIUS);
    leg_left = addCapsule(LEG_LEFT_1.vec, LEG_LEFT_2.vec, LEG_RADIUS);

    foot_right = addCapsule(FOOT_RIGHT_1.vec, FOOT_RIGHT_2.vec, FOOT_RADIUS);
    foot_left = addCapsule(FOOT_LEFT_1.vec, FOOT_LEFT_2.vec, FOOT_RADIUS);

    // create all joints
    spine = addFixedJoint(lbody, hbody);

    neck = addBallJoint(hbody, head, NECK_POS.vec);

    hip_right = addBallJoint(thigh_right, lbody, HIP_RIGHT_POS.vec);
    hip_left = addBallJoint(thigh_left, lbody, HIP_LEFT_POS.vec);

    knee_right = addBallJoint(leg_right, thigh_right, KNEE_RIGHT_POS.vec);
    knee_left = addBallJoint(leg_right, thigh_right, KNEE_RIGHT_POS.vec);

    ankle_right = addBallJoint(foot_right, leg_right, ANKLE_RIGHT_POS.vec);
    ankle_left = addBallJoint(foot_left, leg_left, ANKLE_LEFT_POS.vec);

    shoulder_right = addBallJoint(hbody, arm_right, SHOULDER_RIGHT_POS.vec);
    shoulder_left = addBallJoint(hbody, arm_left, SHOULDER_LEFT_POS.vec);

    elbow_right = addBallJoint(arm_right, forearm_right, ELBOW_RIGHT_POS.vec);
    elbow_left = addBallJoint(arm_left, forearm_left, ELBOW_LEFT_POS.vec);

    wrist_right = addBallJoint(forearm_right, hand_right, WRIST_RIGHT_POS.vec);
    wrist_left = addBallJoint(forearm_left, hand_left, WRIST_LEFT_POS.vec);
}
