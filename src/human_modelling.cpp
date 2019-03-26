#include <iostream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "../include/util.h"

static dWorldID world;
static dSpaceID space;

const dReal density = 1.0;
const dReal length = 0.8;
const dReal radius = 0.2;

dBodyID cap;
dGeomID geom;
static dGeomID  ground;
static dJointGroupID contactgroup;
//static int flag = 0;
dsFunctions fn;

//const dReal   radius = 0.2;
//const dReal   mass   = 1.0;

typedef struct {
  dBodyID body;
  dGeomID geom;
} MyObject;
MyObject ball;

void test1() // test partie 1 opération sur les vecteurs
{
    num_type n= 1.5;
    vec3 u={1, 1, 1};
    vec3 v={2, 3, 4};
    mat3 A={u,u,u};

    std :: cout << "u=" <<u << ", v=" << v <<", n=" << n << std::endl;

    vec3 a=u+v;
    vec3 b=u-v;
    vec3 c=-u;
    vec3 d=n*v;
    vec3 e=v/n;
    vec3 f=unit3(u);
    vec3 g=cross(u, v);
    vec3 h=project3(u, v);
    vec3 i=A*v;
    vec3 j=zaxis(A);

    num_type s=sign(n);
    num_type m=norm(u);
    num_type t=dot3(u, v);
    num_type z=angle3(u, v);

    mat3 B=transpose(A);
    mat3 C=rotMatrix(u, n);

    mat4 W=makeOpenGLMatrix(A, v);

    std::cout << "test addition u+v: " << a << std::endl;
    std::cout << "test soustraction u-v: " << b << std::endl;
    std::cout << "test opposé -u: " << c << std::endl;
    std::cout << "test multiplication par un scalaire n*v: " << d << std::endl;
    std::cout << "test division par un scalaire v/n: " << e << std::endl;
    std::cout << "test unit3(u): " << f << std::endl;
    std::cout << "test cross(u,v): " << g << std::endl;
    std::cout << "test project3(u,v): " << h << std::endl;
    std::cout << "test A*v, multiplication par une matrice: " << i << std::endl;
    std::cout << "test zaxis(A): " << j << std::endl;

    std::cout << "sign(n): " << s << ",  norm(u): " << m << ", dot3(u,v): " << t << ", angle3(u,v): " << z << std::endl;
}

//start function
void start()
{
    //Set a camera
    static float xyz[3] = {0.0, -5.0, 1.0};
    static float hpr[3] = {90.0, 0.0, 0.0};
    dsSetViewpoint(xyz, hpr);
}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
   dBodyID b1 = dGeomGetBody(o1);
   dBodyID b2 = dGeomGetBody(o2);
   dContact contact;
   contact.surface.mode = dContactBounce | dContactSoftCFM;
   // friction parameter
   contact.surface.mu = 500;
   // bounce is the amount of "bouncyness".
   contact.surface.bounce = 0.2;
   // bounce_vel is the minimum incoming velocity to cause a bounce
   // contact.surface.bounce_vel = 0.001;
   // constraint force mixing parameter
   // contact.surface.soft_cfm = 0.001;
   if (int numc = dCollide (o1,o2,1,&contact.geom,sizeof(dContact))) {
       dJointID c = dJointCreateContact (world,contactgroup,&contact);
       dJointAttach (c,b1,b2);
   }
}

static void simLoop (int pause)
{
    const dReal *pos, *R; // position, rotation matrix

    dSpaceCollide(space, 0, &nearCallback);

    dWorldStep(world, 0.05); // Step a simulation world, time step is 0.05

    dJointGroupEmpty(contactgroup);

    dsSetColor(0.5, 0.2, 0.2);
    pos = dBodyGetPosition(cap);
    R = dBodyGetRotation(cap);
    dsDrawCapsuleD(pos, R, length, radius);
}

//create a capsule body and corresponding geom.
void createCapsule(dBodyID* body, dGeomID* geom, dWorldID world, dSpaceID space, dReal density, dReal length, dReal radius)
{
    //create capsule body (aligned along the z-axis so that it matches the
	//GeomCCylinder created below, which is aligned along the z-axis by
	//default)
	*body = dBodyCreate(world);
	dMass M;
	dMassSetZero(&M);
	dMassSetCapsule(&M, density, 3, radius, length);
	dBodySetMass(*body, &M);

	// Je sais pas comment écrire les trois lignes, "set parameters for drawing the body


	// create a capsule geom for collision detection
	*geom = dCreateCapsule(space, radius, length);
	dGeomSetBody(*geom, *body);
}

int main(int argc, char **argv)
{
    //dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;
    //dMass m1;

    //for drawstuff
    dsFunctions fn;
    fn.version = DS_VERSION; //the version of drawstuff
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = NULL; //no command function for keyboard
    fn.stop = NULL; //no stop function
    fn.path_to_textures = "../../drawstuff/textures";

    dInitODE();
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world,0,0,-0.005);

    // set the initial simulation loo parameters
    const int fps = 60;
    const dReal dt = 1.0 / fps;
    const int stepsPerFrame = 2;
    const int SloMo = 1;
    bool Paused = false;
    int numiter = 0;

    //create a capsule
    createCapsule(&cap, &geom, world, space, density, length, radius);
    dBodySetPosition(cap, 0.0, 0.0, 2.0);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1.0, 1.0, 0.0, 0.01);
    dBodySetRotation(cap, R);
    ground = dCreatePlane(space, 0, 0, 1, 0);

    //Simulation loop
    // argc, argv are argument of main function.
    //Window size is 352 x 288 pixels
    //fn is a structure of drawstuff
    dsSimulationLoop(argc, argv, 352*3, 288*3, &fn);

    dWorldDestroy(world); // Destroy the world
    dCloseODE();          // Close ODE

    return 0;
}
