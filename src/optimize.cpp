#include <ode/ode.h>
#include "cmaes.h"

#include "../include/climber.h"

using namespace libcmaes;


const int DIM = 44; // the number of dimensions on the mov_step function
const int N = 21; // number of entries on the target velocity vector

// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static Climber* climberptr;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2) {
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
   // create world
   world = dWorldCreate();
   space = dHashSpaceCreate(0);
   dWorldSetGravity(world, 0, 0, -9.81);
   dWorldSetERP(world, 0.1);
   dWorldSetCFM(world, 1e-4);

   // create floor
   dCreatePlane(space, 0, 0, 1, 0);

   // Create climber
   dVector3 offset = { 0, 0, 0};
   climberptr = new Climber(world, space, offset);
}

// simulation loop
static void simLoop(dReal step) {
   // find collisions and add contact joints
   dSpaceCollide(space, 0, &nearCallback);
   
   // step the simulation
   dWorldQuickStep(world, step);  
   
   // remove all contact joints
   dJointGroupEmpty(contactgroup);
}

/*  Input vector x:
 *  
 *  - x[0] is the duration of the first interval
 *  - x[1] is the duration of the second interval
 *  - x[  2:  N+1] are the target angular velocities of the first keyframe
 *  - x[N+2:2*N+1] are the target angular velocities of the second keyframe
 * 
 */

FitFunc mov_step = [](const double *x, const int N) {
   // decode input
   const dReal dur1 = 0.25 + 0.05*x[0];
   const dReal dur2 = 0.25 + 0.05*x[1];
   dReal vels1[N];
   dReal vels2[N];
   
   const dReal total_dur = dur1 + dur2;
   for (int i = 0; i < N; ++i) {
      vels1[i] = (dReal) x[i+2];
      vels2[i] = (dReal) x[i+N+2];
   }

   // TODO: load last save

   // get start velocities
   dReal vels0[N];
   climberptr->getAngularVelocities(vels0);

   // simulate the movement
   dReal cost = 0;
   dReal t_vels[N];
   const dReal step = 0.01;
   
   for (dReal t = 0; t < total_dur; t += step) {
      // linear interpolation of the target velocities
      if (t <= dur1) {
         for (int i = 0; i < N; ++i) 
            t_vels[i] = vels0[i] + vels1[i]*t/dur1; 
      }
      else {
         for (int i = 0; i < N; ++i)
            t_vels[i] = vels1[i] + (vels2[i]-vels1[i])*(t-dur1)/(dur2-dur1);
      }
      climberptr->setTargetVelocities(t_vels);

      // step the simulation
      simLoop(step);

      // add cost
      // cost += climberptr->cost(?, ?)/total_dur;
   }

   return cost;
};

int main (int argc, char **argv) {
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

   // run optimization
   start();

   double sigma = 1.0; // initial step-size, i.e. estimated initial parameter error
   std::vector<double> x0(DIM, 0.0); // initialize x0 as 0 in all dimensions
   CMAParameters<> cmaparams(x0, sigma);
   cmaparams.set_algo(aCMAES);  // select active CMA-ES as algorithm (default is CMA-ES). double sigma = 0.1

   CMASolutions cmasols = cmaes<>(mov_step, cmaparams);

   std::cout << "best solution: " << cmasols << std::endl;
   std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
   std::cout << cmasols.run_status(); // the optimization status, failed if < 0

   // clean up
   dJointGroupDestroy(contactgroup);
   dSpaceDestroy(space);
   dWorldDestroy(world);
   dCloseODE();
   
   return 0;
}