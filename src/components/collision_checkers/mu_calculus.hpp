#ifndef _RRGLIB_COLLISION_CHECKER_MU_CALCULUS_HPP_
#define _RRGLIB_COLLISION_CHECKER_MU_CALCULUS_HPP_


#include <components/collision_checkers/mu_calculus.h>

#include <components/collision_checkers/base.hpp>
#include <common/region.hpp>


template< class typeparams >
rrglib::collision_checker_mu_calculus<typeparams>
::collision_checker_mu_calculus () {
  
  num_discretization_steps = 20;
  discretization_length = 0.1;  
  discretization_method = 2;
}


template< class typeparams >
rrglib::collision_checker_mu_calculus<typeparams>
::~collision_checker_mu_calculus () {

  for (typename list<region_t*>::iterator iter = list_regions.begin();
       iter != list_regions.end(); iter++) {
    
    region_t *region_curr = *iter;
    
    delete region_curr;
  }
}


template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::cc_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::cc_update_insert_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::cc_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::cc_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::check_collision_state (state_t *state_in) {

  return 1; // Accept all states

  if (list_regions.size() == 0)
    return 1;
  
  for (typename list<region_t *>::iterator iter = list_regions.begin(); iter != list_regions.end(); iter++) {
    region_t *region_curr = *iter;
    
    bool collision = true; 

    for (int i = 0; i < state_in->size(); i++) {
      
      if ( fabs((*state_in)[i] - region_curr->center[i]) >= region_curr->size[i]/2.0) 
	collision = false;
    }
    
    if (collision)  {
      return 0;
    }
  }
  
  return 1;
}



template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::get_region_index( state_t *x ) {

  int idx_curr = 0;
  
  for (typename list<region_t*>::iterator it_region = list_regions.begin();
       it_region != list_regions.end(); it_region++) {
    
    region_t *region_curr = *it_region;
    idx_curr++;
    
    bool state_in_this_region = true;
    
    // Check whether the state is in this region
    for (int i = 0; i < region_curr->numDim(); i++) {
      
      if ( fabs ((*x)[i] - region_curr->center[i]) >= region_curr->size[i]/2.0 ) {
	state_in_this_region = false;
	break;
      }
    }

    // If the state is in this region then immediately return the current region index
    if (state_in_this_region == true) {
      return idx_curr;
    }
  }
  

  // If the execution got here, then state_in is not in any particular region,
  //    in which case this function returns zero.
  return 0;
}


template< class typeparams >
std::list<int> rrglib::collision_checker_mu_calculus<typeparams>
::get_region_indices( state_t *x )
{
	std::list<int> indices;
	int idx_curr = 0;

	for (typename list<region_t*>::iterator it_region = list_regions.begin();
		 it_region != list_regions.end(); it_region++) {
		region_t *region_curr = *it_region;
		idx_curr++;

		bool state_in_this_region = true;

		// Check whether the state is in this region
		for (int i = 0; i < region_curr->numDim(); i++) {
			if ( fabs ((*x)[i] - region_curr->center[i]) >= region_curr->size[i]/2.0 ) {
				state_in_this_region = false;
				break;
			}
		}

		if (state_in_this_region)
			indices.push_back( idx_curr );
	}

	if (indices.size() == 0)
		indices.push_back( 0 );

	return indices;
}

template< class typeparams >
bool rrglib::collision_checker_mu_calculus<typeparams>
::is_match( list<int> indices1, list<int> indices2 )
{
	bool match = true;
	if (indices1.size() != indices2.size()) {
		match = false;
	} else {
		for (std::list<int>::iterator it_curr_idx = indices1.begin();
			 it_curr_idx != indices1.end(); it_curr_idx++) {
			std::list<int>::iterator it_prev_idx = indices2.begin();
			for (; it_prev_idx != indices2.end(); it_prev_idx++)
				if (*it_prev_idx == *it_curr_idx)
					break;
			if (it_prev_idx == indices2.end()) {
				match = false;
				break;
			}
		}
	}
	return match;
}

// returns a negative number to indicate error
// returns 0 if there is a collision
// returns 1 if no collision
template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::check_collision_trajectory (trajectory_t *trajectory_in) {

  
  if (list_regions.size() == 0)
    return 1;
  
  if (trajectory_in->list_states.size() == 0)
    return 1;

  
  int num_traversals = 0;  // Number of transitions from one region to another.
  
  // Start the collision checking procedure with the first state in the trajectory
  typename list<state_t*>::iterator iter = trajectory_in->list_states.begin();
  state_t *state_prev = *iter;
  state_t x_inc;

  // Determine the region that the first state in the trajectory lies in.
  std::list<int> idx_region_prev = get_region_indices( state_prev );
  std::list<int> idx_region_curr;

  // Continue with the remaining states.
  iter++;

  for (; iter != trajectory_in->list_states.end(); iter++) {
    
    state_t *state_curr = *iter;
    
    if (discretization_method != 0) { 

      // Compute the increments 
      double dist_total = 0.0;
	  state_t increments;
      for (int i = 0; i < state_curr->size(); i++) {
	double increment_curr = (*state_curr)[i] - (*state_prev)[i];
	dist_total += increment_curr * increment_curr;
	increments[i] = increment_curr;
      }
      dist_total = sqrt(dist_total);


      // Compute the number of increments
      int num_increments;    
      if (discretization_method == 1) {
	num_increments = num_discretization_steps;
      }
      else if (discretization_method == 2){
	num_increments = (int) floor(dist_total/discretization_length);
      }


      if (num_increments > 0) { // Execute the remaining only if the discretization is required.

        for (int i = 0; i < state_curr->size(); i++)  // Normalize the increments.
	  increments[i] = increments[i]/((double)(num_increments+1));
	
	for (int idx_state = 1; idx_state <= num_increments; idx_state++){
	  
	  for (int i = 0; i < state_curr->size(); i++)
	    x_inc[i] = (*state_prev)[i] + increments[i]*idx_state;
	    
	  // Check the region of the current interpolated state
	  idx_region_curr = get_region_indices( &x_inc );
	  
	  if (!is_match( idx_region_curr, idx_region_prev )) {
	    num_traversals++;
	    if (num_traversals >= 2) // If the number of traversals exceeds one then
	      return 0;              //   return collision.
	  }
	  idx_region_prev = idx_region_curr;
	  
	}
      }
    }
    
    idx_region_curr = get_region_indices( state_curr );
	  if (!is_match( idx_region_curr, idx_region_prev )) {
      num_traversals++;
      if (num_traversals >= 2)  // If the number of traversals exceeds one then 
	return 0;               //   return collision.
    }
    idx_region_prev = idx_region_curr;

    state_prev = state_curr;

    
  }     
  
  
  // If the execution reaches this point, then 
  return 1; // return no collision.
}


template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::set_discretization_steps (int num_discretization_steps_in) {
  
  if (num_discretization_steps <= 0) {
    num_discretization_steps = 0;
    discretization_length = 0;  
    discretization_method = 0;
  }
  else {
    num_discretization_steps = num_discretization_steps_in;
    discretization_method = 1;
  }
  
  return 1;
}


template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::set_discretization_length (double discretization_length_in) {
  
  if (discretization_length <= 0.0) {
    num_discretization_steps = 0;
    discretization_length = 0.05;  
    discretization_method = 0;
  }
  else {
    discretization_length = discretization_length_in;
    discretization_method = 2;
  }
  
  return 1;
}


template< class typeparams >
int rrglib::collision_checker_mu_calculus<typeparams>
::add_region (region_t &obstacle_in) {
  
  list_regions.push_back (new region_t(obstacle_in));
  
  return 1;
}


#endif
