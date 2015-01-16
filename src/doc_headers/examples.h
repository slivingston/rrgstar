/**
   \example standalone_rrtstar_single_integrator.cpp
  
   \brief <B> RRT* in configuration spaces (single integrator).</B>
  
   This file provides the code for running the RRT* algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces. The system can be run in 
   arbitrary dimensions. The RRT* algorithm scales well up to 10 dimensions converging 
   close to an optimal solution in a few seconds, in the examples provided in this file.
   
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 
  
   \ingroup examples
*/


/**  
   \example standalone_rrtstar_single_integrator_trajectory_biasing.cpp
  
   \brief <B> RRT* in configuration spaces (single integrator) with trajectory biasing 
          heuristic. </B>
  
   This file provides the code for running the RRT* algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces using a trajectory biasing heuristic, 
   which concentrates the samples around the current best trajectory in the RRT*. The system can be 
   run in arbitrary dimensions. 
  
   The RRT* algorithm scales well beyond 20 dimensions (upto 50) converging 
   close to an optimal solution in a few seconds, in the examples provided in this file when
   using the trajectory biasing heuristic. 
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. Also 
   the trajectory biasing parameters can be varied from the within the main function.
  
   \ingroup examples
*/


/** 
    \example standalone_rrtstar_single_integrator_trajectory_biasing_and_bnb.cpp
  
    \brief <B> RRT* in configuration spaces (single integrator) with trajectory biasing 
           and branch and bound heuristics. </B>
  
    This file provides the code for running the RRT* algorithm for a single integrator with 
    bounds on velocity, i.e., planning in configuration spaces using a trajectory biasing 
    heuristic together with a branch and bound heuristic. The trajectory bias heuristic 
    concentrates the samples around the current best trajectory in the RRT*, while the 
    branch and bound heuristic deletes those vertices that can not lead to (or very unlikely to 
    lead to) an optimal solution.
  
    The system can be run in arbitrary dimensions. The RRT* algorithm scales well beyond 
    30 dimensions (upto 70) converging close to an optimal solution in a few seconds, 
    in the examples provided in this file when using the trajectory biasing heuristic. 
    
  
    The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. Also 
    the trajectory biasing parameters can be varied from the within the main function.
  
    \ingroup examples
*/


/** 
    \example standalone_rrtstar_single_integrator_halton.cpp
  
    \brief <B> RRT* in configuration spaces (single integrator) with the low-discrepency
               and low-dispersion Halton quasi-random sample sequence. </B>
  
    This file provides the code for running the RRT* algorithm for a single integrator with 
    bounds on velocity, i.e., planning in configuration spaces with the Halton sample sequence. 
    The Halton sample sequence is a low-discrepency and low-dispersion sample sequence. 
      
    The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. Also 
    the trajectory biasing parameters can be varied from the within the main function.
  
    \ingroup examples
*/


/** 
    \example standalone_rrtstar_double_integrator.cpp
  
    \brief <B> RRT* with a double integrator on the plane. </B>
  
    This file provides the code for running the RRT* algorithm for a double integrator model
    in two dimensions. This model involves a system with differential constraints with a 
    4-dimensional state space and under-actuation.

    \ingroup examples
*/


/** 
    \example standalone_rrtstar_dubins_car.cpp
  
    \brief <B> RRT* with a Dubins car model. </B>
  
    This file provides the code for running the RRT* algorithm for a Dubins car model, i.e.,
    a robot with constraints on minimum turning radius. This robot model is usually adequate 
    for describing many car-like robotic systems. The model involves a system with kinematic
    constraints and under-actuation. The dimensionality of the state space is three.

    \ingroup examples
*/


/** 
    \example standalone_rrtstar_dubins_double_integrator_airplane.cpp
  
    \brief <B> RRT* with an airplane model (as a combined dubins car double integrator).</B>
           
  
    This file provides the code for running the RRT* algorithm for a simple airplane model
    that consists of a combined dubins car (on the plane) and a double integrator (for the 
    altitute), which are decoupled. This model involves a 5 dimensional state space with
    non-holonomic differential constraints with under-actuation.
  
    \ingroup examples
*/








/**
   \example standalone_rrg_single_integrator.cpp
  
   \brief <B> RRG in configuration spaces (single integrator).</B>
  
   This file provides the code for running the RRG algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces. The system can be run in 
   arbitrary dimensions. 
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 
  
   \ingroup examples
*/


/**
   \example standalone_rrg_single_integrator_mu_calculus.cpp
  
   \brief <B> RRG in configuration spaces (single integrator) with mu-calculus model checking.</B>
  
   This file provides the code for running the RRG algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces, for problem domains described 
   by a mu-calculus formula. 
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 
  
   \ingroup examples
*/


/** 
    \example standalone_rrg_double_integrator.cpp
  
    \brief <B> RRG with a double integrator on the plane. </B>
  
    This file provides the code for running the RRG algorithm for a double integrator model
    in two dimensions. This model involves a system with differential constraints with a 
    4-dimensional state space and under-actuation. 
    
    \ingroup examples
*/


/** 
    \example standalone_rrg_dubins_car.cpp
  
    \brief <B> RRG with a Dubins car model. </B>
  
    This file provides the code for running the RRG algorithm for a Dubins car model, i.e.,
    a robot with constraints on minimum turning radius. This robot model is usually adequate 
    for describing many car-like robotic systems. The model involves a system with kinematic
    constraints and under-actuation. The dimensionality of the state space is three.

    \ingroup examples
*/


/** 
    \example standalone_rrg_dubins_double_integrator_airplane.cpp
  
    \brief <B> RRG with an airplane model (as a combined dubins car double integrator). </B>
           
  
    This file provides the code for running the RRG algorithm for a simple airplane model
    that consists of a combined dubins car (on the plane) and a double integrator (for the 
    altitute), which are decoupled. This model involves a 5 dimensional state space with
    non-holonomic differential constraints with under-actuation.
  
    \ingroup examples
*/












/** 
    \example standalone_rrt_double_integrator.cpp
  
    \brief <B> RRT with a double integrator on the plane. </B>
  
    This file provides the code for running the RRT algorithm for a double integrator model
    in two dimensions. This model involves a system with differential constraints with a 
    4-dimensional state space and under-actuation. 
    
    \ingroup examples
*/


/** 
    \example standalone_rrt_dubins_car.cpp
  
    \brief <B> RRT with a Dubins car model. </B>
  
    This file provides the code for running the RRT algorithm for a Dubins car model, i.e.,
    a robot with constraints on minimum turning radius. This robot model is usually adequate 
    for describing many car-like robotic systems. The model involves a system with kinematic
    constraints and under-actuation. The dimensionality of the state space is three.

    \ingroup examples
*/


/** 
    \example standalone_rrt_dubins_double_integrator_airplane.cpp
  
    \brief <B> RRT with an airplane model (as a combined dubins car double integrator). </B>
           
  
    This file provides the code for running the RRT algorithm for a simple airplane model
    that consists of a combined dubins car (on the plane) and a double integrator (for the 
    altitute), which are decoupled. This model involves a 5 dimensional state space with
    non-holonomic differential constraints with under-actuation.
  
    \ingroup examples
*/


/**
   \example standalone_rrt_single_integrator.cpp
  
   \brief <B> RRT in configuration spaces (single integrator).</B>
  
   This file provides the code for running the RRT algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces. The system can be run in 
   arbitrary dimensions. 
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 

   \ingroup examples
*/
































/**
   \example libbot_rrtstar_single_integrator.cpp
  
   \brief <B> RRT* in configuration spaces (single integrator) using the 
              libbot interface for visualization.</B>
  
   This file provides the code for running the RRT* algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces. The system can be run in 
   arbitrary dimensions. The RRT* algorithm scales well up to 10 dimensions converging 
   close to an optimal solution in a few seconds, in the examples provided in this file.
   The visualization is done using libbot viewer with projecting the trajectories to its 
   first three state variables (only the first two state components are shown if the 
   dimensionality of the space is chosen to be two dimensional only).
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 
  
   \ingroup examples
*/


/**  
   \example libbot_rrtstar_single_integrator_trajectory_biasing.cpp
  
   \brief <B> RRT* in configuration spaces (single integrator) with trajectory biasing 
              heuristic using the libbot interface for visualization. </B>
  
   This file provides the code for running the RRT* algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces using a trajectory biasing heuristic, 
   which concentrates the samples around the current best trajectory in the RRT*. The system can be 
   run in arbitrary dimensions. 
  
   The RRT* algorithm scales well beyond 20 dimensions (upto 50) converging 
   close to an optimal solution in a few seconds, in the examples provided in this file when
   using the trajectory biasing heuristic. The visualization is done using libbot viewer 
   with projecting the trajectories to its first three state variables (only the first 
   two state components are shown if the dimensionality of the space is chosen to be 
   two dimensional only).
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. Also 
   the trajectory biasing parameters can be varied from the within the main function.
  
   \ingroup examples
*/


/** 
    \example libbot_rrtstar_single_integrator_trajectory_biasing_and_bnb.cpp
  
    \brief <B> RRT* in configuration spaces (single integrator) with trajectory biasing 
           and branch and bound heuristics using the libbot interface for visualization. </B>
  
    This file provides the code for running the RRT* algorithm for a single integrator with 
    bounds on velocity, i.e., planning in configuration spaces using a trajectory biasing 
    heuristic together with a branch and bound heuristic. The trajectory bias heuristic 
    concentrates the samples around the current best trajectory in the RRT*, while the 
    branch and bound heuristic deletes those vertices that can not lead to (or very unlikely to 
    lead to) an optimal solution.
  
    The system can be run in arbitrary dimensions. The RRT* algorithm scales well beyond 
    30 dimensions (upto 70) converging close to an optimal solution in a few seconds, 
    in the examples provided in this file when using the trajectory biasing heuristic. 
    The visualization is done using libbot viewer with projecting the trajectories 
    to its first three state variables (only the first two state components are shown 
    if the dimensionality of the space is chosen to be two dimensional only).
  
    The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. Also 
    the trajectory biasing parameters can be varied from the within the main function.
  
    \ingroup examples
*/


/** 
    \example libbot_rrtstar_single_integrator_halton.cpp
  
    \brief <B> RRT* in configuration spaces (single integrator) with the low-discrepency
               and low-dispersion Halton quasi-random sample sequence using libbot for visualization. </B>
  
    This file provides the code for running the RRT* algorithm for a single integrator with 
    bounds on velocity, i.e., planning in configuration spaces with the Halton sample sequence. 
    The Halton sample sequence is a low-discrepency and low-dispersion sample sequence. 
      
    The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. Also 
    the trajectory biasing parameters can be varied from the within the main function.
  
    \ingroup examples
*/


/** 
    \example libbot_rrtstar_double_integrator.cpp
  
    \brief <B> RRT* with a double integrator on the plane using the libbot interface 
               for visualization. </B>
  
    This file provides the code for running the RRT* algorithm for a double integrator model
    in two dimensions. This model involves a system with differential constraints with a 
    4-dimensional state space and under-actuation.

    \ingroup examples
*/


/** 
    \example libbot_rrtstar_dubins_car.cpp
  
    \brief <B> RRT* with a Dubins car model using the libbot interface for visualization. </B>
  
    This file provides the code for running the RRT* algorithm for a Dubins car model, i.e.,
    a robot with constraints on minimum turning radius. This robot model is usually adequate 
    for describing many car-like robotic systems. The model involves a system with kinematic
    constraints and under-actuation. The dimensionality of the state space is three.

    \ingroup examples
*/


/** 
    \example libbot_rrtstar_dubins_double_integrator_airplane.cpp
  
    \brief <B> RRT* with an airplane model (as a combined dubins car double integrator) 
           using the libbot interface for visualization. </B>
  
    This file provides the code for running the RRT* algorithm for a simple airplane model
    that consists of a combined dubins car (on the plane) and a double integrator (for the 
    altitute), which are decoupled. This model involves a 5 dimensional state space with
    non-holonomic differential constraints with under-actuation.
  
    \ingroup examples
*/








/**
   \example libbot_rrg_single_integrator.cpp
  
   \brief <B> RRG in configuration spaces (single integrator) using the libbot interface
              for visualization.</B>
  
   This file provides the code for running the RRG algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces. The system can be run in 
   arbitrary dimensions. The RRT* algorithm scales well up to 10 dimensions converging 
   close to an optimal solution in a few seconds, in the examples provided in this file.
   The visualization is done using libbot viewer with projecting the trajectories to its 
   first three state variables (only the first two state components are shown if the 
   dimensionality of the space is chosen to be two dimensional only).
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 
  
    Note that planners that maintain graphs, such as the RRG or the PRM and variants, usually
    involve many edges (substantially more than the vertices). Because each edge carries a 
    substantial amount of information regarding the trajectory connecting its source and
    destination vertices, publishing the data, i.e., the graph itself, through the libbot 
    messaging system (called LCM) is impossible for large number of iterations.

   \ingroup examples
*/


/**
   \example libbot_rrg_single_integrator_mu_calculus.cpp
  
   \brief <B> RRG in configuration spaces (single integrator) with mu-calculus model checking
               using libbot for visualization. </B>
  
   This file provides the code for running the RRG algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces, for problem domains described 
   by a mu-calculus formula. 
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 
  
   \ingroup examples
*/


/** 
    \example libbot_rrg_double_integrator.cpp
  
    \brief <B> RRG with a double integrator on the plane using the libbot interface 
               for visualization. </B>
  
    This file provides the code for running the RRG algorithm for a double integrator model
    in two dimensions. This model involves a system with differential constraints with a 
    4-dimensional state space and under-actuation. 
    
    Note that planners that maintain graphs, such as the RRG or the PRM and variants, usually
    involve many edges (substantially more than the vertices). Because each edge carries a 
    substantial amount of information regarding the trajectory connecting its source and
    destination vertices, publishing the data, i.e., the graph itself, through the libbot 
    messaging system (called LCM) is impossible for large number of iterations.

    \ingroup examples
*/


/** 
    \example libbot_rrg_dubins_car.cpp
  
    \brief <B> RRG with a Dubins car model using the libbot interface for visualization. </B>
  
    This file provides the code for running the RRG algorithm for a Dubins car model, i.e.,
    a robot with constraints on minimum turning radius. This robot model is usually adequate 
    for describing many car-like robotic systems. The model involves a system with kinematic
    constraints and under-actuation. The dimensionality of the state space is three.

    Note that planners that maintain graphs, such as the RRG or the PRM and variants, usually
    involve many edges (substantially more than the vertices). Because each edge carries a 
    substantial amount of information regarding the trajectory connecting its source and
    destination vertices, publishing the data, i.e., the graph itself, through the libbot 
    messaging system (called LCM) is impossible for large number of iterations.

    \ingroup examples
*/


/** 
    \example libbot_rrg_dubins_double_integrator_airplane.cpp
  
    \brief <B> RRG with an airplane model (as a combined dubins car double integrator) 
           using the libbot interface for visualization. </B>
  
    This file provides the code for running the RRG algorithm for a simple airplane model
    that consists of a combined dubins car (on the plane) and a double integrator (for the 
    altitute), which are decoupled. This model involves a 5 dimensional state space with
    non-holonomic differential constraints with under-actuation.

    Note that planners that maintain graphs, such as the RRG or the PRM and variants, usually
    involve many edges (substantially more than the vertices). Because each edge carries a 
    substantial amount of information regarding the trajectory connecting its source and
    destination vertices, publishing the data, i.e., the graph itself, through the libbot 
    messaging system (called LCM) is impossible for large number of iterations.
  
    \ingroup examples
*/










/** 
    \example libbot_rrt_double_integrator.cpp
  
    \brief <B> RRT with a double integrator on the plane using the libbot interface 
               for visualization. </B>
  
    This file provides the code for running the RRT algorithm for a double integrator model
    in two dimensions. This model involves a system with differential constraints with a 
    4-dimensional state space and under-actuation. 
    
    \ingroup examples
*/


/** 
    \example libbot_rrt_dubins_car.cpp
  
    \brief <B> RRT with a Dubins car model using the libbot interface for visualization. </B>
  
    This file provides the code for running the RRT algorithm for a Dubins car model, i.e.,
    a robot with constraints on minimum turning radius. This robot model is usually adequate 
    for describing many car-like robotic systems. The model involves a system with kinematic
    constraints and under-actuation. The dimensionality of the state space is three.

    \ingroup examples
*/


/** 
    \example libbot_rrt_dubins_double_integrator_airplane.cpp
  
    \brief <B> RRT with an airplane model (as a combined dubins car double integrator) 
           using the libbot interface for visualization. </B>
  
    This file provides the code for running the RRT algorithm for a simple airplane model
    that consists of a combined dubins car (on the plane) and a double integrator (for the 
    altitute), which are decoupled. This model involves a 5 dimensional state space with
    non-holonomic differential constraints with under-actuation.
  
    \ingroup examples
*/


/**
   \example libbot_rrt_single_integrator.cpp
  
   \brief <B> RRT in configuration spaces (single integrator) using the libbot 
              interface for visualization.</B>
  
   This file provides the code for running the RRT algorithm for a single integrator with 
   bounds on velocity, i.e., planning in configuration spaces. The system can be run in 
   arbitrary dimensions. The visualization is done using libbot viewer with projecting 
   the trajectories to its first three state variables (only the first two state 
   components are shown if the dimensionality of the space is chosen to be two dimensional only).
  
   The dimensionality of the space can be adjusted using the NUM_DIMENSIONS parameter. 

   \ingroup examples
*/




























