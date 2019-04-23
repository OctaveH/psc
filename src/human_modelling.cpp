#define NB_PARTS 16
#define NB_JOINTS 15

#include <iostream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h>
#include <unistd.h>

#include "../include/save_simu.h"
#include "../include/util.h"
#include "../include/body.h"

// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static Ragdoll* climberptr;

static save_simu sauvegarde;

dReal velocities[25];
dReal t = 0.0;

static bool to_load = false;
static bool to_save = true;
static int frame_count = 0;

const dReal density = 1000.0;
const vec3 offset = {0.0, 0.0, 1.0};

const dReal length = 0.1;
const dReal radius = 0.2;

dBodyID cap;
dGeomID geom;
static dGeomID  ground;
//static int flag = 0;
dsFunctions fn;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (dAreConnected(b1, b2))
        return;
    dContact contact;
    contact.surface.mode = dContactBounce | dContactSoftCFM;
    // friction parameter
    contact.surface.mu = 500;
    // bounce is the amount of "bouncyness".
    contact.surface.bounce = 0.2;
    // bounce_vel is the minimum incoming velocity to cause a bounce
    contact.surface.bounce_vel = 0.1;
    // constraint force mixing parameter
    contact.surface.soft_cfm = 0.001;
    if (int numc = dCollide (o1,o2,1,&contact.geom,sizeof(dContact))) {
        dJointID c = dJointCreateContact (world,contactgroup,&contact);
        dJointAttach (c,b1,b2);
    }
}

//start function - set view point
void start()
{
    //Set a camera
    static float xyz[3] = {2.5, 2.5, 2.5};
    static float hpr[3] = {225.0, -25.0, 0.0};
    dsSetViewpoint(xyz, hpr);

    for (int i = 0; i < 25; ++i) {
        velocities[i] = 0;
    }
    velocities[0] = 1.0;
}

// Simulation loop
static void simLoop (int pause)
{
    // find collisions and add contact joints
    dSpaceCollide(space, 0, &nearCallback);

    usleep(5000);
    // step the simulation
    dWorldStep(world, 0.005); // Step a simulation world, time step is 0.05

    // remove all contact joints
    dJointGroupEmpty(contactgroup);

    // Save or load a simulation state if needed
    if (to_save)
    {
        to_save = false;
        sauvegarde.save();
    }
    if (to_load)
    {
        to_load = false;
        sauvegarde.load();
    }

    climberptr->draw();

    //Drawing the blue capsule
    const dReal *pos, *R; // position, rotation matrix
    dsSetColor(0,0,1);
    pos = dBodyGetPosition(cap);
    R = dBodyGetRotation(cap);
    dsDrawCapsuleD(pos, R, length, radius);
//Pour ajouter un temps de pause :
//    char a = ' ';
//    while (a == ' ' && frame_count % 10 == 0)
//        std::cin >> a;
//    frame_count++;
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

	// create a capsule geom for collision detection
	*geom = dCreateCapsule(space, radius, length);
	dGeomSetBody(*geom, *body);
}

void command(int cmd)
{
    switch (cmd)
    {
    case 's': // sauvegarder l'état actuel de la simulation
        to_save = true;
        break;
    case 'l': // charger l'état sauvegardé
        to_load = true;
        break;
    }
}

int main(int argc, char **argv)
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = NULL;
    fn.path_to_textures = "../../drawstuff/textures";

    dInitODE();

    // create world
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    dWorldSetERP(world, 0.5);
    dWorldSetCFM(world, 0.0001);
    dWorldSetGravity(world,0,0,-9.81);

    // create floor
    contactgroup = dJointGroupCreate(0);

    climberptr = new Ragdoll(world, space, density, offset);

    // Configuration de la sauvegarde
    dBodyID bodies[NB_PARTS];
    dGeomID geoms[NB_PARTS];
    for (int i = 0; i < NB_PARTS; i++)
    {
        bodies[i] = climberptr->body_parts[i].body;
        geoms[i] = climberptr->body_parts[i].geom;
    }
    sauvegarde = save_simu(NB_PARTS, bodies, geoms);

    // set the initial simulation loop parameters
    const int fps = 60;
    const dReal dt = 1.0 / fps;
    const int stepsPerFrame = 2;
    const int SloMo = 1;
    bool Paused = false;
    int numiter = 0;

    // create a capsule at a random position
    createCapsule(&cap, &geom, world, space, density*2, length, radius);
    srand(time(NULL));
    dReal x = (((double)rand())/RAND_MAX - 0.5) * 0.4;
    dReal y = (((double)rand())/RAND_MAX - 0.5) * 0.4;
    dBodySetPosition(cap, x, y, radius);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1.0, 1.0, 0.0, 0.01);
    dBodySetRotation(cap, R);
    ground = dCreatePlane(space, 0, 0, 1, 0);

    // run simulation
    dsSimulationLoop(argc, argv, 352*3, 288*3, &fn);

    // clean up
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
