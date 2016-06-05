/**
   \example rrg/single_integrator.cpp

   \brief <B> RRG in configuration spaces (single integrator) with mu-calculus model checking.</B>

   This file provides the code for running the RRG algorithm for a single integrator with
   bounds on velocity, i.e., planning in configuration spaces, for problem domains described
   by a mu-calculus formula. The system can be run in arbitrary dimensions.

   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter.

   \ingroup examples
*/


/**
    \example rrg/double_integrator.cpp

    \brief <B> RRG with a double integrator on the plane. </B>

    This file provides the code for running the RRG algorithm for a double integrator model
    in two dimensions. This model involves a system with differential constraints with a
    4-dimensional state space and under-actuation.

    \ingroup examples
*/


/**
    \example rrg/dubins_car.cpp

    \brief <B> RRG with a Dubins car model. </B>

    This file provides the code for running the RRG algorithm for a Dubins car model, i.e.,
    a robot with constraints on minimum turning radius. This robot model is usually adequate
    for describing many car-like robotic systems. The model involves a system with kinematic
    constraints and under-actuation. The dimensionality of the state space is three.

    \ingroup examples
*/


/**
    \example rrg/dubins_double_integrator_airplane.cpp

    \brief <B> RRG with an airplane model (as a combined dubins car double integrator). </B>


    This file provides the code for running the RRG algorithm for a simple airplane model
    that consists of a combined dubins car (on the plane) and a double integrator (for the
    altitute), which are decoupled. This model involves a 5 dimensional state space with
    non-holonomic differential constraints with under-actuation.

    \ingroup examples
*/
