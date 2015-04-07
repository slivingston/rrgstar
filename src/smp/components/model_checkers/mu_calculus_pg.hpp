#ifndef _SMP_MODEL_CHECKER_MU_CALCULUS_PG_HPP_
#define _SMP_MODEL_CHECKER_MU_CALCULUS_PG_HPP_

#include <smp/components/model_checkers/mu_calculus_pg.h>
#include <smp/components/model_checkers/base.hpp>


template< class typeparams >
smp::model_checker_mu_calculus_pg<typeparams>
::model_checker_mu_calculus_pg()
	: collision_checker(NULL), cost_evaluator(NULL)
{
	found_solution = false;
	mcpg.pt.parseFormula( "Phi" ); // Formula parse tree is hardcoded now.
}


template< class typeparams >
smp::model_checker_mu_calculus_pg<typeparams>
::~model_checker_mu_calculus_pg()
{ }

template< class typeparams>
void smp::model_checker_mu_calculus_pg<typeparams>
::add_labeler( collision_checker_mu_calculus<typeparams> *collision_checker_in )
{
	collision_checker = collision_checker_in;
}

template< class typeparams>
void smp::model_checker_mu_calculus_pg<typeparams>
::add_costeval( cost_evaluator_base<typeparams> *cost_evaluator_in )
{
	cost_evaluator = cost_evaluator_in;
}


template< class typeparams >
int smp::model_checker_mu_calculus_pg<typeparams>
::mc_update_insert_vertex( vertex_t *vertex_in )
{
	// Create a new state
	PGState *pg_state_new = new PGState;

	state_t *state_curr = vertex_in->state;

	if (!collision_checker) {
		std::cerr << "WARNING: No labelers have been added to the model checker." << std::endl;
		return 0;
	}

	// Add propositions to pg_state

	// TODO: The current implementation assumes disjoint regions, so that at most
	// one prop need be added to the state labeling. Relax this assumption.
	int slabel = collision_checker->get_region_index( state_curr );
	if (slabel)
		pg_state_new->addprop( slabel );

	pg_state_new->data = (void *)vertex_in;

	// Add state pointer into vertex data
	vertex_in->data.state = pg_state_new;

	// Add the new state to the model checker
	mcpg.addState( pg_state_new );

	return 1;
}


template< class typeparams >
int smp::model_checker_mu_calculus_pg<typeparams>
::mc_update_insert_edge( edge_t *edge_in )
{
	vertex_t *vertex_src = edge_in->vertex_src;
	vertex_t *vertex_dst = edge_in->vertex_dst;

	if (cost_evaluator) {
		if (mcpg.addTransition( vertex_src->data.state, vertex_dst->data.state,
								cost_evaluator->evaluate_cost_trajectory( vertex_src->state,
																		  edge_in->trajectory_edge )))
			found_solution = true;
	} else {
		if (mcpg.addTransition( vertex_src->data.state, vertex_dst->data.state ))
			found_solution = true;
	}

	return 1;
}


template< class typeparams >
int smp::model_checker_mu_calculus_pg<typeparams>
::mc_update_delete_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams >
int smp::model_checker_mu_calculus_pg<typeparams>
::mc_update_delete_edge (edge_t *edge_in) {

  return 1;
}

template< class typeparams >
bool smp::model_checker_mu_calculus_pg<typeparams>
::has_feasible() const
{
	return found_solution;
}


template< class typeparams >
int smp::model_checker_mu_calculus_pg<typeparams>
::get_solution( trajectory_t &trajectory_out )
{
	// TODO: also put in the inputs...

	if (found_solution == false)
		return 0;

	PGStateList state_list = mcpg.getTrajectory();

	list<vertex_t*> list_vertices;

	for (PGStateList::iterator it_pg_state = state_list.begin();
		 it_pg_state != state_list.end(); it_pg_state++) {
		vertex_t *vertex_curr = (vertex_t*) ((*it_pg_state)->data);
		list_vertices.push_back(vertex_curr);
	}

	vertex_t *vertex_prev = list_vertices.front();
	list_vertices.pop_front();

	trajectory_out.list_states.push_back (vertex_prev->state);

	for (typename list<vertex_t*>::iterator it_vertex = list_vertices.begin();
		 it_vertex != list_vertices.end(); it_vertex++) {

		vertex_t *vertex_curr = *it_vertex;

		if (vertex_curr == vertex_prev)
			continue;

		// find the edge between these two vertices
		edge_t *edge_found = 0;
		for (typename list<edge_t*>::iterator it_edge
				 = vertex_curr->incoming_edges.begin();
			 it_edge != vertex_curr->incoming_edges.end(); it_edge++) {

			edge_t *edge_curr = *it_edge;

			// TODO: Is it not redundant to check this, given that here we are
			// enumerating the list of incoming edges for vertex_curr ?
			if ((edge_curr->vertex_src == vertex_prev)
				&& (edge_curr->vertex_dst == vertex_curr)) {
				edge_found = edge_curr;
				break;
			}
		}

		if (edge_found == 0) {
			cout << "No such edge" << endl;
			return 0;
		}

		// Add all trajectory on this edge to the new trajectory
		for (typename list<state_t*>::iterator it_state
				 = edge_found->trajectory_edge->list_states.begin();
			 it_state != edge_found->trajectory_edge->list_states.end();
			 it_state++) {

			state_t *state_curr = *it_state;

			trajectory_out.list_states.push_back (state_curr);
		}

		trajectory_out.list_states.push_back (vertex_curr->state);

		vertex_prev = vertex_curr;

	}

	return 1;
}


#endif
