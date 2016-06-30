#ifndef _RRGLIB_SYSTEM_SINGLE_INTEGRATOR_HPP_
#define _RRGLIB_SYSTEM_SINGLE_INTEGRATOR_HPP_


#include <components/extenders/single_integrator.h>

#include <components/extenders/state_array_double.hpp>
#include <components/extenders/input_array_double.hpp>
#include <components/extenders/base.hpp>

#include <iostream>
#include <cmath>


using namespace std;


template< class typeparams, int NUM_DIMENSIONS >
int rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::ex_update_insert_vertex (vertex_t *vertex_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::ex_update_insert_edge (edge_t *edge_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::ex_update_delete_vertex (vertex_t *vertex_in){

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::ex_update_delete_edge (edge_t *edge_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::extender_single_integrator () {

    max_length = 1.0;
}


template< class typeparams, int NUM_DIMENSIONS >
rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::~extender_single_integrator () {

}

template< class typeparams, int NUM_DIMENSIONS >
int rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::set_max_length (double max_length_in) {

    if (max_length_in <= 0.0)
        return 0;

    max_length = max_length_in;

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int rrglib::extender_single_integrator< typeparams, NUM_DIMENSIONS >
::extend( state_t *state_from_in, state_t *state_towards_in,
            int *exact_connection_out, trajectory_t *trajectory_out,
            list<state_t*> *intermediate_vertices_out )
{
    if (max_length <= 0.0)
        return 0;

    trajectory_out->list_states.clear();
    trajectory_out->list_inputs.clear();
    intermediate_vertices_out->clear();

    double dists[NUM_DIMENSIONS];
    double dist = 0.0;

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
        dists[i] = (*state_towards_in)[i] - (*state_from_in)[i];
        dist += dists[i] * dists[i];
    }
    dist = sqrt(dist);

    state_t *state_new;
    input_t *input_new = new input_t;

    if (dist < max_length) {
        state_new = new state_t( *state_towards_in );
        (*input_new)[0] = dist;
        for (int i = 0; i < NUM_DIMENSIONS; i++)
            (*input_new)[i+1] = ((*state_towards_in)[i]-(*state_from_in)[i])/dist;

        *exact_connection_out = 1;
    } else {
        state_new = new state_t;
        (*input_new)[0] = max_length;
        for (int i = 0; i < NUM_DIMENSIONS; i++) {
            (*state_new)[i] = (*state_from_in)[i] + dists[i]/dist*max_length;
            (*input_new)[i+1] = dists[i]/dist;
        }
        *exact_connection_out = 0;
    }

    trajectory_out->list_states.push_back(state_new);
    trajectory_out->list_inputs.push_back(input_new);

    return 1;
}


#endif
