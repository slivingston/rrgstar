#ifndef _RRGLIB_BRANCH_AND_BOUND_BASE_HPP_
#define _RRGLIB_BRANCH_AND_BOUND_BASE_HPP_


#include <utils/branch_and_bound_base.h>

#include <planner_utils/vertex_edge.hpp>
#include <planners/base.hpp>


template< class typeparams >
rrglib::branch_and_bound_base<typeparams>
::branch_and_bound_base () {

    planner_bnb = NULL;

    upper_bound_cost = -1.0;
}


template< class typeparams >
rrglib::branch_and_bound_base<typeparams>
::~branch_and_bound_base () {

}


template< class typeparams >
int rrglib::branch_and_bound_base<typeparams>
::set_planner (planner_t *planner_in) {

    planner_bnb = planner_in;

    if (planner_bnb == NULL)
        upper_bound_cost = -1.0;

    return 1;
}


template< class typeparams >
int rrglib::branch_and_bound_base<typeparams>
::set_upper_bound_cost (double upper_bound_cost_in) {

    if (upper_bound_cost_in <= 0.0)
        upper_bound_cost = -1.0;

    upper_bound_cost = upper_bound_cost_in;

    return 1;
}


#endif
