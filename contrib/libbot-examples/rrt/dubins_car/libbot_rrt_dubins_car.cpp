// Standard header files
#include<iostream>
using namespace std;


// SMP HEADER FILES ------
#include <smp/components/samplers/uniform.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/dubins.hpp>
#include <smp/components/collision_checkers/standard.hpp>
#include <smp/components/model_checkers/reachability.hpp>

#include <smp/planners/rrt.hpp>

#include <smp/planner_utils/trajectory.hpp>

#include <smp/interfaces/libbot.hpp>


// SMP TYPE DEFINITIONS -------
using namespace smp;

// State, input, vertex_data, and edge_data definitions
typedef state_dubins state_t;
typedef input_dubins input_t;
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
typedef sampler_uniform<typeparams,3> sampler_t;
typedef distance_evaluator_kdtree<typeparams,3> distance_evaluator_t;
typedef extender_dubins<typeparams> extender_t;
typedef collision_checker_standard<typeparams,2> collision_checker_t;
typedef model_checker_reachability<typeparams,2> model_checker_t;

// Define all algorithm types
typedef rrt<typeparams>  rrt_t;

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
  rrt_t planner(sampler, distance_evaluator, extender, collision_checker, model_checker);

  // 1.c Create the visualization interface
  interface_t interface;
  


  

  // 2. INITALIZE PLANNING OBJECTS

  // 2.a Initialize the sampler
  region<3> sampler_support;
  sampler_support.center[0] = 0.0;
  sampler_support.center[1] = 0.0;
  sampler_support.center[2] = 0.0;
  sampler_support.size[0] = 20.0;
  sampler_support.size[1] = 20.0;
  sampler_support.size[2] = 2.0*M_PI;
  sampler.set_support (sampler_support);


  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.


  // 2.c Initialize the extender

 
  // 2.d Initialize the collision checker
  region<2> obstacle_new;
  for (int i = 0; i < 2; i++) {
    obstacle_new.center[i] = 5.0;
    obstacle_new.size[i] = 5.0;
  }
  collision_checker.add_obstacle (obstacle_new);
  planner.init_collision_checker (collision_checker);
  
  
  // 2.e Initialize the model checker
  region<2> region_goal;
  region_goal.center[0] = 8.0;
  region_goal.center[1] = 8.0;  
  region_goal.size[0] = 2.0;
  region_goal.size[1] = 2.0;
  model_checker.set_goal_region (region_goal);
    
  
  // 2.f Initialize the planner
  state_t *state_initial = new state_t;
  for (int i = 0; i < 3; i++) {
    state_initial->state_vars[i] = 0.0;
  }
  planner.initialize (state_initial);
  
  
  // 2.g Initialize the libbot interface.
  interface.set_planner (&planner);
  interface.visualize_2d();



  


  // 3. PUBLISH THE ENVIRONMENT AS A MESAGE THROUGH THE INTERFACE
  environment_t environment;
  region<3> goal;
  goal.center[0] = 8.0;
  goal.center[1] = 8.0;
  goal.center[2] = 0.5;
  goal.size[0] = 2.0;
  goal.size[1] = 2.0;
  goal.size[2] = 1.0;
  environment.set_goal_region (goal);
  
  region<3> operating;
  operating.center[0] = 0.0;
  operating.center[1] = 0.0;
  operating.center[2] = 5.0;
  operating.size[0] = 20.0;
  operating.size[1] = 20.0;
  operating.size[2] = 10.0;
  environment.set_operating_region (operating);
  
  region<3> obstacle;
  obstacle.center[0] = 5.0;
  obstacle.center[1] = 5.0;
  obstacle.center[2] = 1.0;
  obstacle.size[0] = 5.0;
  obstacle.size[1] = 5.0;
  obstacle.size[2] = 2.0;
  environment.add_obstacle (obstacle);
  
  interface.publish_environment (environment);  





  
  // 4. RUN THE PLANNER 
  for (int i = 0; i < 2000; i++){
    planner.iteration ();
    
    if (i%100 == 0) {
      cout << "Iteration: " << i << endl;
      interface.publish_data ();
    }

  }
  
  


  // 5. PUBLISH THE RESULTS THROUGH THE LIBBOT INTERFACE
  interface.publish_data ();
  trajectory_t trajectory_final;
  model_checker.get_solution (trajectory_final);
  interface.publish_trajectory (trajectory_final);

  
  
  return 1;
  
}
