#ifndef _RRGLIB_STATE_ARRAY_DOUBLE_HPP_
#define _RRGLIB_STATE_ARRAY_DOUBLE_HPP_

#include <components/extenders/state_array_double.h>

#include <iostream>

using namespace std;

template <int NUM_STATES>
rrglib::state_array_double<NUM_STATES>
::state_array_double () {

  for (int i = 0; i < NUM_STATES; i++) 
    state_vars[i] = 0.0;

}


template <int NUM_STATES>
rrglib::state_array_double<NUM_STATES>
::state_array_double (const rrglib::state_array_double<NUM_STATES> &state_in) {

  for (int i = 0; i < NUM_STATES; i++) 
	  state_vars[i] = state_in.state_vars[i];
  
}


template <int NUM_STATES>
rrglib::state_array_double<NUM_STATES>
::~state_array_double () {

}


template <int NUM_STATES>
bool
rrglib::state_array_double<NUM_STATES>
::operator==( const state_array_double<NUM_STATES> &state_in ) const
{
	for (int i = 0; i < NUM_STATES; i++) {
		if (this->state_vars[i] != state_in.state_vars[i])
			return false;
	}
	return true;
}


template<int NUM_STATES>
bool
rrglib::state_array_double<NUM_STATES>
::eqtol( const state_array_double<NUM_STATES> &state_in, double tol ) const
{
	for (int i = 0; i < NUM_STATES; i++) {
		if ((this->state_vars[i] - state_in.state_vars[i]) > tol
			|| (state_in.state_vars[i] - this->state_vars[i]) > tol)
			return false;
	}
	return true;
}


template <int NUM_STATES>
const rrglib::state_array_double<NUM_STATES> &
rrglib::state_array_double<NUM_STATES>
::operator=(const rrglib::state_array_double<NUM_STATES> &state_in) {

  if (&state_in != this) {
    for (int i = 0; i < NUM_STATES; i++) 
      state_vars[i] = state_in.state_vars[i];
  }

  return *this;
}

template <int NUM_STATES>
double&
rrglib::state_array_double<NUM_STATES>
::operator[] (int index_in) {

  if ( (index_in < NUM_STATES) && (index_in >= 0) )
    return state_vars[index_in];
  
  // TODO: ideally throw an exception also.
  return state_vars[0];
}


#endif
