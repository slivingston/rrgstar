/*! \file components/model_checkers/mu_calculus.h
  \brief The mu-calculus model checker.

  This includes an implementation of the mu-calculus model checker.
*/


#ifndef _SMP_MODEL_CHECKER_MU_CALCULUS_PG_H_
#define _SMP_MODEL_CHECKER_MU_CALCULUS_PG_H_

#include <smp/components/model_checkers/base.h>
#include <smp/components/collision_checkers/mu_calculus.h>
#include <smp/components/cost_evaluators/base.h>

#include <smp/external_libraries/inc_mu_mc/ms.h>
#include <smp/external_libraries/inc_mu_mc/pt.h>




namespace smp {

	//! Implements the vertex data for mu-calculus model checking if using rrgstar
	class model_checker_mu_calculus_pg_vertex_data {
	public:
		PGState *state;
	};

	//! Implements the edge data for mu-calculus model checking
	/*!
	  This empty class is implemented for the sake of completeness.
	*/
	class model_checker_mu_calculus_pg_edge_data {
	};


	//! Implements the mu-calculus model checker.
	/*!
	  This class inherits from the model_checker_base class. It implements the
	  mu-calculus model checker using the mu-calculus external libraries that are
	  included with the smp library.

	  \ingroup model_checkers
	*/
	template< class typeparams >
	class model_checker_mu_calculus_pg : public model_checker_base<typeparams>{

		typedef typename typeparams::state state_t;
		typedef typename typeparams::input input_t;

		typedef vertex<typeparams> vertex_t;
		typedef edge<typeparams> edge_t;
		typedef typename typeparams::vertex_data vertex_data_t;
		typedef typename typeparams::edge_data edge_data_t;

		typedef trajectory<typeparams> trajectory_t;

		bool found_solution;

		collision_checker_mu_calculus<typeparams> *collision_checker;
		cost_evaluator_base<typeparams> *cost_evaluator;

	public:

		//! An instance from the mu-calculus model checker external library.
		/*!
		  This variable instantiates one of the main classes of the external
		  library that carries out the mu-calculus model checking operation.
		*/
		ModelCheckerPG mcpg;

		void add_labeler( collision_checker_mu_calculus<typeparams> *collision_checker_in );
		void add_costeval( cost_evaluator_base<typeparams> *cost_evaluator_in );

		model_checker_mu_calculus_pg();
		~model_checker_mu_calculus_pg();

		int mc_update_insert_vertex( vertex_t *vertex_new );
		int mc_update_insert_edge( edge_t *edge_new );
		int mc_update_delete_vertex( vertex_t *vertex_new );
		int mc_update_delete_edge( edge_t *edge_new );
		bool has_feasible() const;

		int get_solution( trajectory_t &trajectory_out );
	};

}

#endif
