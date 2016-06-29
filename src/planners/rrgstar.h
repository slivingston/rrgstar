/*! \file planners/rrgstar.h
  \brief An optimal Rapidly-exploring Random Graph (RRG) algorithm for mu-calculus specifications
 */

#ifndef _RRGLIB_PLANNER_RRGSTAR_H_
#define _RRGLIB_PLANNER_RRGSTAR_H_


#include <components/model_checkers/mu_calculus_pg.h>
#include <planners/base_incremental.h>
#include <planners/planner_parameters.h>

#include <components/cost_evaluators/base.h>


namespace rrglib {

    template< class typeparams >
    class rrgstar : public planner_incremental< typeparams > {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;

        typedef trajectory<typeparams> trajectory_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef planner_incremental<typeparams> planner_t;

        typedef planner_parameters parameters_t;

        typedef sampler_base<typeparams> sampler_t;
        typedef distance_evaluator_base<typeparams> distance_evaluator_t;
        typedef extender_base<typeparams> extender_t;
        typedef collision_checker_base<typeparams> collision_checker_t;
        typedef model_checker_mu_calculus_pg<typeparams> model_checker_t;
        typedef cost_evaluator_base<typeparams> cost_evaluator_t;


    private:

        model_checker_t &model_checker;
        cost_evaluator_t &cost_evaluator;


        // This function adds the given state to the beginning of the trajectory and calls the collision checker.
        int check_extended_trajectory_for_collision (state_t *state, trajectory_t *trajectory) {

            trajectory->list_states.push_front (state);
            int collision_check = this->collision_checker.check_collision_trajectory (trajectory);
            trajectory->list_states.pop_front ();

            return collision_check;
        }


    public:

        //! Algorithm parameters
        /*!
          This class stores the parameters used by the algorithm. These parameters
          can be modified by the user using the methods provided by the class
          planner_parameters.
        */
        parameters_t parameters;


        rrgstar();
        ~rrgstar();

        /**
         * \brief A constructor that initializes all components.
         *
         * This is the recommended constructor that initializes all components all at once.
         * It calls the corresponding constructor of the base class
         * planner_incremental<typeparams> with the same arguments.
         */
        rrgstar( sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in,
                 collision_checker_t &collision_checker_in, model_checker_t &model_checker_in,
                 cost_evaluator_t &cost_evaluator_in );

        int initialize( state_t *initial_state_in = 0 );

        int iteration ();

        bool has_feasible() const;
        double current_min_cost() const;  // Only defined if has_feasible()

        void dump_json( bool include_graph = true ) const;
        void dump_json( std::ostream &s, bool include_graph = true ) const;

        void dumpArenaDOT( std::ofstream &outf ) const
            { model_checker.mcpg.dumpDOT( outf ); }
    };

}


#endif
