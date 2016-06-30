/*! \file components/model_checkers/mu_calculus.h
  \brief The mu-calculus model checker.

  This includes an implementation of the mu-calculus model checker.
*/


#ifndef _RRGLIB_MODEL_CHECKER_MU_CALCULUS_H_
#define _RRGLIB_MODEL_CHECKER_MU_CALCULUS_H_


#include <components/model_checkers/base.h>
#include <components/collision_checkers/mu_calculus.h>

#include <external_libraries/inc_mu_mc/ms.h>
#include <external_libraries/inc_mu_mc/pt.h>


namespace rrglib {

    //! Implements the vertex data for mu-calculus model checking if using rrg
    /*!
      The data stored in each vertex of the graph required for the mu-calculus model
      checking operation.
    */
    class model_checker_mu_calculus_vertex_data {

    public:

        //! State variable for mu-calculus model checking library.
        /*!
          This variable is a data type used by the external mu-calculus model
          checking library. The class holds the set of all sub-formula of the
          specification that the associated vertex satisfies. It is incrementally
          updated by the procedure.
        */
        MS_state *state;
    };


    //! Implements the edge data for mu-calculus model checking
    /*!
      This empty class is implemented for the sake of completeness.
    */
    class model_checker_mu_calculus_edge_data {

    };


    //! Implements the mu-calculus model checker.
    /*!
      This class inherits from the model_checker_base class. It implements the
      mu-calculus model checker using the mu-calculus external libraries.

      \ingroup model_checkers
    */
    template< class typeparams >
    class model_checker_mu_calculus : public model_checker_base<typeparams>{

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef trajectory<typeparams> trajectory_t;

        int uid_counter;

        bool found_solution;

        collision_checker_mu_calculus<typeparams> *collision_checker;

    public:

        //! An instance from the mu-calculus model checker external library.
        /*!
          This variable instantiates one of the main classes of the external
          library that carries out the mu-calculus model checking operation.
        */
        rModelChecker ms;

        void add_labeler( collision_checker_mu_calculus<typeparams> *collision_checker_in );

        model_checker_mu_calculus( unsigned int number_of_goals = 2,
                                   unsigned int number_of_obstacles = 1 );

        ~model_checker_mu_calculus ();


        int mc_update_insert_vertex (vertex_t *vertex_new);

        int mc_update_insert_edge (edge_t *edge_new);

        int mc_update_delete_vertex (vertex_t *vertex_new);

        int mc_update_delete_edge (edge_t *edge_new);

        bool has_feasible() const;

        int get_solution (trajectory_t &trajectory_out);

    };

}


#endif
