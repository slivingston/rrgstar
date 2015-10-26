#ifndef _RRGLIB_VERTEX_EDGE_HPP_
#define _RRGLIB_VERTEX_EDGE_HPP_

#include <planner_utils/vertex_edge.h>


#include <planner_utils/trajectory.hpp>


template< class typeparams >
rrglib::edge< typeparams >
::edge () {
  
  vertex_src = 0;
  vertex_dst = 0;
  trajectory_edge = 0;
}


template< class typeparams >
rrglib::edge< typeparams >
::~edge () {

  delete trajectory_edge;
}


template< class typeparams >
rrglib::vertex< typeparams >
::vertex () {

  state = NULL;
  incoming_edges.clear();
  outgoing_edges.clear();
}


template< class typeparams >
rrglib::vertex< typeparams >
::~vertex () {

  if (state)
    delete state;

  incoming_edges.clear();
  outgoing_edges.clear();
}

#endif
