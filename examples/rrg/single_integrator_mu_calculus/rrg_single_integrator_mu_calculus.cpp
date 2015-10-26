// Standard header files
#include<iostream>
using namespace std;


// RRGLIB HEADER FILES ------
#include <components/samplers/uniform.hpp>
#include <components/extenders/single_integrator.hpp>
#include <components/distance_evaluators/kdtree.hpp>
#include <components/collision_checkers/mu_calculus.hpp>
#include <components/model_checkers/mu_calculus.hpp>

#include <planners/rrg.hpp>

#include <planner_utils/trajectory.hpp>


// PARAMETERS TO THE PROBLEM *******************************************
// *

/* Change the number of dimensions from here.  Scale it up to 6 - 10 dimensions
   to see the convergence of RRT* towards an optimal solution in very high
   dimensional configuration spaces without employing any heuristics. */
#define NUM_DIMENSIONS 2

/* Maximum length of an extension. This should scale up with sqrt(d) and with L,
   where d is the dimensionality of space and L is the side length of a box
   containing the obstacle free space. */
#define EXTENSION_LENGTH  1.0
// *
// *********************************************************************


// RRGLIB TYPE DEFINITIONS -------
using namespace rrglib;

// State, input, vertex_data, and edge_data definitions
typedef state_single_integrator<NUM_DIMENSIONS> state_t;
typedef input_single_integrator<NUM_DIMENSIONS> input_t;
typedef region<NUM_DIMENSIONS> region_t;
typedef model_checker_mu_calculus_vertex_data vertex_data_t;
typedef model_checker_mu_calculus_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef state_t state;
  typedef input_t input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
  typedef region_t region;
} typeparams; 

// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef sampler_uniform<typeparams,NUM_DIMENSIONS> sampler_t;
typedef distance_evaluator_kdtree<typeparams,NUM_DIMENSIONS> distance_evaluator_t;
typedef extender_single_integrator<typeparams,NUM_DIMENSIONS> extender_t;
typedef collision_checker_mu_calculus<typeparams> collision_checker_t;
typedef model_checker_mu_calculus<typeparams> model_checker_t;

// Define all algorithm types
typedef rrg<typeparams>  rrg_t;





int
main ()
{
  // 1. CREATE PLANNING OBJECTS
  
  // 1.a Create the components
  sampler_t sampler;
  distance_evaluator_t distance_evaluator;
  extender_t extender;
  collision_checker_t collision_checker;
  model_checker_t model_checker;
  model_checker.add_labeler( &collision_checker );

  // 1.b Create the planner algorithm
  rrg_t planner(sampler, distance_evaluator, extender,
				collision_checker, model_checker);

  /* The phase parameter can be used to run the algorithm as an RRT,
     See the documentation of the RRG algorithm for more
     information. */
  planner.parameters.set_phase (2);

  /* Set this parameter should be set at least to the side length of
     the (bounded) state space. E.g., if the state space is a box with
     side length L, then this parameter should be set to at least L
     for rapid and efficient convergence in trajectory space. */
  planner.parameters.set_gamma (35.0);
  planner.parameters.set_dimension (NUM_DIMENSIONS);
  planner.parameters.set_max_radius (EXTENSION_LENGTH);




  // 2. INITALIZE PLANNING OBJECTS

  // 2.a Initialize the sampler
  region_t sampler_support;
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    sampler_support.center[i] = 0.0;
    sampler_support.size[i] = 20.0;
  }
  sampler.set_support (sampler_support);  

  
  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.


  // 2.c Initialize the extender
  extender.set_max_length(EXTENSION_LENGTH);

 
  // 2.d Initialize the collision checker
  region_t R;
  R.center[0] = R.center[1] = -3.5;
  R.size[0] = R.size[1] = 1.0;
  if (NUM_DIMENSIONS >= 3) {
	  R.center[2] = 5.0;
	  R.size[2] = 2.0;
  }
  collision_checker.add_region( R );

  R.center[0] = 5.5;
  R.center[1] = 1.5;
  R.size[0] = R.size[1] = 1.0;
  if (NUM_DIMENSIONS >= 3) {
	  R.center[2] = 5.0;
	  R.size[2] = 2.0;
  }
  collision_checker.add_region( R );

  R.center[0] = R.center[1] = 2.05;
  R.size[0] = R.size[1] = 3.9;
  if (NUM_DIMENSIONS >= 3) {
	  R.center[2] = 5.0;
	  R.size[2] = 2.0;
  }
  collision_checker.add_region( R );

  
  // 2.e Initialize the model checker
  //     TODO: Add in the regions and the formula.
  /* NOTE that the formula is currently hard-coded in ParseTree::parseFormula()
     as the parse tree provided in parseFormulaLoop4(), both defined in the file
     inc_mu_mc/pt.cpp */


  // 2.f Initialize the planner
  state_t *state_initial = new state_t;
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    state_initial->state_vars[i] = 0.0;
  }
  planner.initialize (state_initial);

  


  // 3. RUN THE PLANNER 
  for (int i = 0; i < 10000 && !planner.has_feasible(); i++)
    planner.iteration ();
  


  // 4. GET THE RESULTS
  planner.dump_json();
  
  return 0;
}
