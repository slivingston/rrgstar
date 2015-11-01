#ifndef _RRGLIB_COST_EVALUATOR_TIME_HPP_
#define _RRGLIB_COST_EVALUATOR_TIME_HPP_

#include <cassert>
#include <iostream>

#include <components/cost_evaluators/time.h>

#include <components/cost_evaluators/base.hpp>


using namespace std;


template< class typeparams >
int rrglib::cost_evaluator_time<typeparams>
::ce_update_vertex_cost (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams >
int rrglib::cost_evaluator_time<typeparams>
::ce_update_edge_cost (edge_t *edge_in) {
  
  return 1;
}



template< class typeparams >
double rrglib::cost_evaluator_time<typeparams>
::evaluate_cost_trajectory (state_t *state_initial_in,
			    trajectory_t *trajectory_in,
			    state_t *state_final_in) {

  double total_time = 0.0;
  for (typename list<input_t*>::iterator iter = trajectory_in->list_inputs.begin();
       iter != trajectory_in->list_inputs.end(); iter++) {
    
    input_t *input_curr = *iter;
    
    total_time += (*input_curr)[0];
    
  }
  assert( total_time > 0.0 );
  return total_time;
};

#endif
