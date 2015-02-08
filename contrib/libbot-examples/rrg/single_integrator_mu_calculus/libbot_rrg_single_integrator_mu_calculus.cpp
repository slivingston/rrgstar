// Standard header files
#include<iostream>
using namespace std;


// SMP HEADER FILES ------
#include <smp/components/samplers/uniform.hpp>                       
#include <smp/components/extenders/single_integrator.hpp>    
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/collision_checkers/mu_calculus.hpp>         
#include <smp/components/model_checkers/mu_calculus.hpp>

#include <smp/planners/rrg.hpp>

#include <smp/planner_utils/trajectory.hpp>  

#include <smp/interfaces/libbot.hpp>


// PARAMETERS TO THE PROBLEM *****************************************************************************************************
// *
#define NUM_DIMENSIONS 3     // Change the number of dimensions from here
                             // Scale it up to 6 - 10 dimensions to see the convergence of RRT* towards an optimal solution
                             //   in very high dimensional configuration spaces without employing any heuristics. 
#define EXTENSION_LENGTH  10.0   // Maximum length of an extension. This should scale up with sqrt(d) and with L, where d is the
                                 //   dimensionality of space and L is the side length of a box containing the obstacle free space.
// *
// *******************************************************************************************************************************


// SMP TYPE DEFINITIONS -------
using namespace smp;

// State, input, vertex_data, and edge_data definitions
typedef state_single_integrator<NUM_DIMENSIONS> state_t;
typedef input_single_integrator input_t;
typedef model_checker_mu_calculus_vertex_data vertex_data_t;
typedef model_checker_mu_calculus_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef state_t state;
  typedef input_t input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
} typeparams; 

// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef sampler_uniform<typeparams,NUM_DIMENSIONS> sampler_t;
typedef distance_evaluator_kdtree<typeparams,NUM_DIMENSIONS> distance_evaluator_t;
typedef extender_single_integrator<typeparams,NUM_DIMENSIONS> extender_t;
typedef collision_checker_mu_calculus<typeparams,NUM_DIMENSIONS> collision_checker_t;
typedef model_checker_mu_calculus<typeparams> model_checker_t;

// Define all algorithm types
typedef rrg<typeparams>  rrg_t;

// Define interface types
typedef interface_libbot<typeparams> interface_t;
typedef interface_libbot_environment environment_t;








int
main () {





  // 1. CREATE PLANNING OBJECTS
  
  // 1.a Create the components
  sampler_t sampler;
  distance_evaluator_t distance_evaluator;
  extender_t extender;
  collision_checker_t collision_checker;
  model_checker_t model_checker;

  // 1.b Create the planner algorithm
  rrg_t planner (sampler, distance_evaluator, extender, collision_checker, model_checker);

  planner.parameters.set_phase (2);   // The phase parameter can be used to run the algorithm as an RRT, 
                                      // See the documentation of the RRG algorithm for more information.

  planner.parameters.set_gamma (35.0);    // Set this parameter should be set at least to the side length of
                                          //   the (bounded) state space. E.g., if the state space is a box
                                          //   with side length L, then this parameter should be set to at 
                                          //   least L for rapid and efficient convergence in trajectory space.
  planner.parameters.set_dimension (NUM_DIMENSIONS);
  planner.parameters.set_max_radius (EXTENSION_LENGTH); 

  // 1.c Create the visualization interface
  interface_t interface;





  // 2. INITALIZE PLANNING OBJECTS

  // 2.a Initialize the sampler
  region<NUM_DIMENSIONS> sampler_support;
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    sampler_support.center[i] = 0.0;
    sampler_support.size[i] = 20.0;
  }
  sampler.set_support (sampler_support);  

  
  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.


  // 2.c Initialize the extender
  extender.set_max_length (EXTENSION_LENGTH);

 
  // 2.d Initialize the collision checker
  region<NUM_DIMENSIONS> obstacle_new;
  for (int i = 0; i < 2; i++) {
    obstacle_new.center[i] = 5.0;
    obstacle_new.size[i] = 5.0;
  }
  obstacle_new.center[1] = 4.0;
  for (int i = 2; i < NUM_DIMENSIONS; i++) {
    obstacle_new.center[i] = 0.0;
    obstacle_new.size[i] = 20.0;
  }
  collision_checker.add_region (obstacle_new);

  for (int i = 0; i < 2; i++) {
    obstacle_new.center[i] = -5.0;
    obstacle_new.size[i] = 5.0;
  }
  collision_checker.add_region (obstacle_new);

  obstacle_new.center[0] = 5.0;
  obstacle_new.center[1] = -5.0; 
  for (int i = 0; i < 2; i++) {
    obstacle_new.size[i] = 5.0;
  }
  if (NUM_DIMENSIONS >=3 ) {
    obstacle_new.center[2] = 7.0;
    obstacle_new.size[2] = 5.0;
  }
  collision_checker.add_region (obstacle_new);
  if (NUM_DIMENSIONS >=3 ) {
    obstacle_new.center[2] = -7.0;
    obstacle_new.size[2] = 5.0;
  }
  collision_checker.add_region (obstacle_new);

  obstacle_new.center[0] = -5.0;
  obstacle_new.center[1] = 5.0; 
  obstacle_new.size[0] = 10.0;
  obstacle_new.size[1] = 5.0;
  if (NUM_DIMENSIONS >=3 ) {
    obstacle_new.center[2] = 0.0;
    obstacle_new.size[2] = 10.0;
  }
  collision_checker.add_region (obstacle_new);

  
  // 2.e Initialize the model checker
  //     TODO: Add in the regions and the formula.


  // 2.f Initialize the planner
  state_t *state_initial = new state_t;
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    state_initial->state_vars[i] = 0.0;
  }
  planner.initialize (state_initial);

  
  // 2.g Initialize the libbot interface.
  interface.set_planner (&planner);
  if (NUM_DIMENSIONS == 2)
    interface.visualize_2d();
  else
    interface.visualize_3d();



  



  // 3. PUBLISH THE ENVIRONMENT AS A MESAGE THROUGH THE INTERFACE
  environment_t environment;
  region<3> goal;
  goal.center[0] = 8.0;
  goal.center[1] = 8.0;
  goal.center[2] = 0.0;
  goal.size[0] = 2.0;
  goal.size[1] = 2.0;
  goal.size[2] = 20.0;
  environment.set_goal_region (goal);
  
  region<3> operating;
  operating.center[0] = 0.0;
  operating.center[1] = 0.0;
  operating.center[2] = 0.0;
  operating.size[0] = 20.0;
  operating.size[1] = 20.0;
  operating.size[2] = 20.0;
  environment.set_operating_region (operating);
  
  region<3> obstacle;
  obstacle.center[0] = 5.0;
  obstacle.center[1] = 4.0;
  obstacle.center[2] = 0.0;
  obstacle.size[0] = 5.0;
  obstacle.size[1] = 5.0;
  obstacle.size[2] = 20.0;
  environment.add_obstacle (obstacle);

  obstacle.center[0] = -5.0;
  obstacle.center[1] = -5.0;
  obstacle.center[2] = 0.0;
  obstacle.size[0] = 5.0;
  obstacle.size[1] = 5.0;
  obstacle.size[2] = 20.0;
  environment.add_obstacle (obstacle);
  
  obstacle.center[0] = 5.0;
  obstacle.center[1] = -5.0;
  obstacle.center[2] = 7.0;
  obstacle.size[0] = 5.0;
  obstacle.size[1] = 5.0;
  obstacle.size[2] = 5.0;
  environment.add_obstacle (obstacle);

  obstacle.center[0] = 5.0;
  obstacle.center[1] = -5.0;
  obstacle.center[2] = -7.0;
  obstacle.size[0] = 5.0;
  obstacle.size[1] = 5.0;
  obstacle.size[2] = 5.0;
  environment.add_obstacle (obstacle);

  obstacle.center[0] = -5.0;
  obstacle.center[1] = 5.0;
  obstacle.center[2] = 0.0;
  obstacle.size[0] = 10.0;
  obstacle.size[1] = 5.0;
  obstacle.size[2] = 10.0;
  environment.add_obstacle (obstacle);
  
  interface.publish_environment (environment);

  




  // 4. RUN THE PLANNER 
  for (int i = 0; i < 1000; i++){
    planner.iteration ();
    
    if (i%10 == 0){
      cout << "Iteration : " << i << endl;
      interface.publish_data ();
    }
  }
  interface.publish_data ();
  
  
  



  // 5. PUBLISH THE RESULTS THROUGH THE LIBBOT INTERFACE
  trajectory_t trajectory_final;
  model_checker.get_solution (trajectory_final);
  interface.publish_trajectory (trajectory_final);



  
  return 1;
  
}
