#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "../include/climber.h"

// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static Climber* climberptr;

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
   contact.surface.mu = 0;
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

// start simulation - set viewpoint
static void start() {
   static float xyz[3] = {2.0f,-2.0f,2.0f};
   static float hpr[3] = {140.000f,-17.0000f,0.0000f};
   dsSetViewpoint(xyz, hpr);
}

// simulation loop
static void simLoop(int pause) {
   // find collisions and add contact joints
   dSpaceCollide(space, 0, &nearCallback);
   
   // step the simulation
   dWorldQuickStep(world, 0.01);  
   
   // remove all contact joints
   dJointGroupEmpty(contactgroup);

   // redraw sphere at new location
   climberptr->draw();
}

int main (int argc, char **argv) {
   // setup pointers to drawstuff callback functions
   dsFunctions fn;
   fn.version = DS_VERSION;
   fn.start = &start;
   fn.step = &simLoop;
   fn.stop = 0;
   fn.command = 0;
   fn.path_to_textures = "./textures";

   dInitODE ();

   // create world
   world = dWorldCreate();
   space = dHashSpaceCreate(0);
   dWorldSetGravity(world, 0, 0, -9.81);
   dWorldSetERP(world, 0.1);
   dWorldSetCFM(world, 1e-4);
   
   // create floor
   dCreatePlane(space, 0, 0, 1, 0);
   contactgroup = dJointGroupCreate(0);

   // create climber
   dVector3 offset = { 0, 0, 2};
   climberptr = new Climber(world, space, offset);

   // run simulation
   dsSimulationLoop(argc, argv, 640, 480, &fn);
   
   // clean up
   dJointGroupDestroy(contactgroup);
   dSpaceDestroy(space);
   dWorldDestroy(world);
   dCloseODE();
   
   return 0;
}