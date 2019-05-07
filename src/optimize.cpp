#include <iostream>

#include <ode/ode.h>
#include <libcmaes/cmaes.h>

#include "../include/climber.h"
#include "../include/wall.h"
#include "../include/save_simu.h"

using namespace libcmaes;


const int DIM = 44; // the number of dimensions on the mov_step function
const int N_VEL = 21; // number of entries on the target velocity vector

// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

// objects
static Climber *climberptr;
static ClimbingWall *wallptr;

// target stance
Stance target_stance;

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
   contact.surface.mu = 50;
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
 */

FitFunc mov_step = [](const double *x, const int N) {
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

   // TODO: load last save

   // get start velocities
   dReal vels0[N_VEL];
   climberptr->getAngularVelocities(vels0);

   // simulate the movement
   dReal cost = 0;
   dReal t_vels[N_VEL];
   const dReal step = 0.03;
   
   for (dReal t = 0; t < total_dur; t += step) {
      // linear interpolation of the target velocities
      if (t <= dur1) {
         for (int i = 0; i < N_VEL; ++i) 
            t_vels[i] = vels0[i] + (vels1[i]-vels0[i])*t/dur1; 
      }
      else {
         for (int i = 0; i < N_VEL; ++i)
            t_vels[i] = vels1[i] + (vels2[i]-vels1[i])*(t-dur1)/(dur2-dur1);
      }
      climberptr->setTargetVelocities(t_vels);

      // step the simulation
      simLoop(step);

      // add cost
      cost += climberptr->cost(wallptr, target_stance);
   }

   return cost;
};

int main (int argc, char **argv) {
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

   // create wall
   wallptr = new ClimbingWall(world, space);

   // create climber
   dVector3 offset = { 0, 0, 0};
   climberptr = new Climber(world, space, offset);

   // set target stance
   target_stance.lf_hold = -1;
   target_stance.lh_hold =  2;
   target_stance.rf_hold = -1;
   target_stance.rh_hold = -1;

   // run optimization
   double sigma = 1.0; // initial step-size, i.e. estimated initial parameter error
   std::vector<double> x0(DIM, 0.0); // initialize x0 as 0 in all dimensions

   double lbounds[DIM], ubounds[DIM];
   for (int i=0; i < DIM; ++i) {
      lbounds[i] =  0.0;
      ubounds[i] = 10.0;
   }
   GenoPheno<pwqBoundStrategy> gp(lbounds, ubounds, DIM);

   CMAParameters<GenoPheno<pwqBoundStrategy>> cmaparams(x0, sigma, -1, 0, gp);
   cmaparams.set_algo(aCMAES);  // select active CMA-ES as algorithm (default is CMA-ES). double sigma = 0.1

   CMASolutions cmasols = cmaes<GenoPheno<pwqBoundStrategy>>(mov_step, cmaparams);

   std::vector<double> best_sol = cmasols.best_candidate().get_x();
   for (int i = 0; i < DIM; ++i)
      std::cout << best_sol[i] << " ";
   std::cout << std::endl;

   std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds" << std::endl;

   // clean up
   dJointGroupDestroy(contactgroup);
   dSpaceDestroy(space);
   dWorldDestroy(world);
   dCloseODE();
   
   return 0;
}