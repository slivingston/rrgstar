// Standard header files
#include<iostream>
using namespace std;


// RRGLIB HEADER FILES ------
#include <components/samplers/uniform.hpp>
#include <components/distance_evaluators/kdtree.hpp>
#include <components/extenders/double_integrator.hpp>
#include <components/collision_checkers/mu_calculus.hpp>
#include <components/model_checkers/mu_calculus_pg.hpp>
#include <components/cost_evaluators/time.hpp>

#include <planners/rrgstar.hpp>

#include <planner_utils/trajectory.hpp>


// PARAMETERS TO THE PROBLEM *******************************************
// *

/* Change the number of dimensions from here.
   Scale it up to 6 - 10 dimensions to see the convergence of RRT*
   towards an optimal solution in very high dimensional configuration
   spaces without employing any heuristics. */
#define NUM_DIMENSIONS 2

/* Maximum length of an extension. This should scale up with sqrt(d)
   and with L, where d is the dimensionality of space and L is the
   side length of a box containing the obstacle free space. */
#define EXTENSION_LENGTH  5.0
// *
// *********************************************************************


// RRGLIB TYPE DEFINITIONS -------
using namespace rrglib;

// State, input, vertex_data, and edge_data definitions
typedef state_double_integrator<NUM_DIMENSIONS> state_t;
typedef input_double_integrator<NUM_DIMENSIONS> input_t;
typedef region<NUM_DIMENSIONS> region_t;
typedef model_checker_mu_calculus_pg_vertex_data vertex_data_t;
typedef model_checker_mu_calculus_pg_edge_data edge_data_t;

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
typedef sampler_uniform<typeparams,NUM_DIMENSIONS*2> sampler_t;
typedef distance_evaluator_kdtree<typeparams,NUM_DIMENSIONS*2> distance_evaluator_t;
typedef extender_double_integrator<typeparams,NUM_DIMENSIONS> extender_t;
typedef collision_checker_mu_calculus<typeparams> collision_checker_t;
typedef model_checker_mu_calculus_pg<typeparams> model_checker_t;
typedef cost_evaluator_time<typeparams> cost_evaluator_t;

// Define all algorithm types
typedef rrgstar<typeparams>  rrgstar_t;


int main( int argc, char **argv )
{
    // 1. CREATE PLANNING OBJECTS

    // 1.a Create the components
    sampler_t sampler;
    distance_evaluator_t distance_evaluator;
    extender_t extender;
    collision_checker_t collision_checker;
    model_checker_t model_checker;
    cost_evaluator_t cost_evaluator;
    model_checker.add_labeler( &collision_checker );
    model_checker.add_costeval( &cost_evaluator );

    // 1.b Create the planner algorithm
    rrgstar_t planner(sampler, distance_evaluator, extender,
                      collision_checker, model_checker, cost_evaluator);
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
    region<NUM_DIMENSIONS*2> sampler_support;
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
        sampler_support.center[i] = 0.0;
        sampler_support.size[i] = 20.0;
    }
    for (int i = NUM_DIMENSIONS; i < NUM_DIMENSIONS*2; i++) {
        sampler_support.center[i] = 0.0;
        sampler_support.size[i] = 2.0;
    }
    sampler.set_support (sampler_support);


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
    /* NOTE that the formula is currently generated using a reach-avoid template
       of the form ([]<> p1 & []<> p2 & ... & []!p_m) and implemented in
       ParseTree::genFormulaReachAvoid(), which is called from
       ParseTree::parseFormula(), both defined in the file inc_mu_mc/pt.cpp.
       The number of goal regions and obstacles can be chosen when instantiating
       model_checker_t; default is 2 goals, 1 obstacle. */


    // 2.f Initialize the planner
    state_t *state_initial = new state_t;
    for (int i = 0; i < NUM_DIMENSIONS*2; i++) {
        state_initial->state_vars[i] = 0.0;
    }
    planner.initialize (state_initial);


    // 3. RUN THE PLANNER
    int num_it = 1000;
    if (argc >= 2) {
        num_it = strtol( argv[1], NULL, 10 );
        if (num_it < 0) {
            std::cerr << "Number of iterations must be nonnegative." << std::endl;
            return -1;
        }
    }
    int i;
    for (i = 0; i < num_it; i++)
        planner.iteration();

    std::cerr << "number of iterations is " << i << std::endl;


    // 4. GET THE RESULTS
    planner.dump_json();

    return 0;
}
