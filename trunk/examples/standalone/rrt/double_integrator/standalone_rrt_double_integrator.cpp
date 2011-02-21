// Standard header files
#include<iostream>
using namespace std;


// SMP HEADER FILES ------
#include <smp/components/samplers/uniform.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/double_integrator.hpp>
#include <smp/components/collision_checkers/standard.hpp>
#include <smp/components/model_checkers/reachability.hpp>

#include <smp/planners/rrt.hpp>

#include <smp/planner_utils/trajectory.hpp>


// SMP TYPE DEFINITIONS -------
using namespace smp;

// State, input, vertex_data, and edge_data definitions
typedef state_double_integrator<2> state_t;
typedef input_double_integrator<2> input_t;
typedef model_checker_reachability_vertex_data vertex_data_t;
typedef model_checker_reachability_edge_data edge_data_t;

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
typedef sampler_uniform<typeparams,4> sampler_t;
typedef distance_evaluator_kdtree<typeparams,4> distance_evaluator_t;
typedef extender_double_integrator<typeparams,2> extender_t;
typedef collision_checker_standard<typeparams,4> collision_checker_t;
typedef model_checker_reachability<typeparams,4> model_checker_t;

// Define all algorithm types
typedef rrt<typeparams>  rrt_t;





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
  rrt_t planner(sampler, distance_evaluator, extender, collision_checker, model_checker);





  // 2. INITALIZE PLANNING OBJECTS

  // 2.a Initialize the sampler
  region<4> sampler_support;
  for (int i = 0; i < 2; i++) {
    sampler_support.center[i] = 0.0;
    sampler_support.size[i] = 20.0;
  }
  for (int i = 2; i < 4; i++) {
    sampler_support.center[i] = 0.0;
    sampler_support.size[i] = 2.0;
  }
  sampler.set_support (sampler_support);
  

  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.


  // 2.c Initialize the extender

 
  // 2.d Initialize the collision checker
  region<4> obstacle_new;
  for (int i = 0; i < 2; i++) {
    obstacle_new.center[i] = 5.0;
    obstacle_new.size[i] = 5.0;
  }
  for (int i = 2; i < 4; i++) {
    obstacle_new.center[i] = 0.0;
    obstacle_new.size[i] = 20.0;
  }
  collision_checker.add_obstacle (obstacle_new);
  

  // 2.e Initialize the model checker
  region<4> region_goal;
  for (int i = 0; i < 2; i++) {
    region_goal.center[i] = 8.0;
    region_goal.size[i] = 2.0;
  }
  for (int i = 2; i < 4; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 20.0;
  }
  model_checker.set_goal_region (region_goal);

  
  // 2.f Initialize the planner
  state_t *state_initial = new state_t;
  for (int i = 0; i < 4; i++) {
    state_initial->state_vars[i] = 0.0;
  }
  planner.initialize (state_initial);





  

  // 3. RUN THE PLANNER 
  for (int i = 0; i < 1000; i++){

    planner.iteration ();
    
    if (i%100 == 0){
      cout << "Iteration : " << i << endl;
    }

  }


  
  


  // 4. GET THE RESULTS 
  trajectory_t trajectory_final;
  model_checker.get_solution (trajectory_final);



  return 1;
  
}
