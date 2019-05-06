#include <iostream>
#include <time.h>
#include <unistd.h>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "../include/climber.h"
#include "../include/wall.h"

// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

// objects
static Climber* climberptr;
static ClimbingWall* wallptr;

const int DIM = 44; // the number of dimensions on the mov_step function
const int N_VEL = 21; // number of entries on the target velocity vector

// global variables
dReal best_sol[DIM];
dReal step = 0.03;
dReal t = 0.0;

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
   contact.surface.mu = 0.5;
   // bounce is the amount of "bouncyness".
   contact.surface.bounce = 0;
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
   static float xyz[3] = {-2.0f,-2.0f,2.0f};
   static float hpr[3] = {50.000f,-17.0000f,0.0000f};
   dsSetViewpoint(xyz, hpr);
}

void control(dReal t, dReal *x) {
   // decode input
   const dReal dur1 = 0.25 + 0.05*x[0];
   const dReal dur2 = 0.25 + 0.05*x[1];
   dReal vels1[N_VEL];
   dReal vels2[N_VEL];

   const dReal total_dur = dur1 + dur2;
   for (int i = 0; i < N_VEL; ++i) {
      vels1[i] = (dReal) (0.2*x[i+2] - 1.0);
      vels2[i] = (dReal) (0.2*x[i+N_VEL+2] - 1.0);
   }

   // simulate the movement
   dReal t_vels[N_VEL];
   
   // linear interpolation of the target velocities
   if (t <= dur1) {
      // get start velocities
      dReal vels0[N_VEL];
      climberptr->getAngularVelocities(vels0);

      for (int i = 0; i < N_VEL; ++i) 
         t_vels[i] = vels0[i] + (vels1[i]-vels0[i])*t/dur1; 
   }
   else if (t <= total_dur) {
      for (int i = 0; i < N_VEL; ++i)
         t_vels[i] = vels1[i] + (vels2[i]-vels1[i])*(t-dur1)/(dur2-dur1);
   }
   else {
      exit(0);
   }

   climberptr->setTargetVelocities(t_vels);
}

// simulation loop
static void simLoop(int pause) {
   usleep(50000);
   
   // find collisions and add contact joints
   dSpaceCollide(space, 0, &nearCallback);
   
   // control the climber's movement
   control(t, best_sol);
   t += step;

   // step the simulation
   dWorldQuickStep(world, step);  
   
   // remove all contact joints
   dJointGroupEmpty(contactgroup);

   // redraw the climber and wall
   climberptr->draw();
   wallptr->draw();
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

   dInitODE();

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
   dVector3 offset = { 0, 0, 0};
   climberptr = new Climber(world, space, offset);

   // create wall
   wallptr = new ClimbingWall(world, space);

   // get best solution from stdin
   for (int i = 0; i < DIM; ++i)
      std::cin >> best_sol[i];

   // run simulation
   dsSimulationLoop(argc, argv, 640, 480, &fn);
   
   // clean up
   dJointGroupDestroy(contactgroup);
   dSpaceDestroy(space);
   dWorldDestroy(world);
   dCloseODE();
   
   return 0;
}