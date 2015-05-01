#include "ms.h"
#include "pt.h"
#include <assert.h>


void ModelCheckerPG::dumpDOT( std::ofstream &outf )
{
	propositionSet_it prp_it;
	int pcount;

	outf << "digraph G {\n \"\" [shape=none]" << std::endl;

	outf << "\"\" -> \"" << this->initialVertex << "\"" << std::endl;

	for (PGStateSet_it it_state = this->states.begin();
		 it_state != this->states.end(); it_state++) {
		for (PGVertexSet_it it_vertex = (*it_state)->vertices.begin();
			 it_vertex != (*it_state)->vertices.end(); it_vertex++) {

			outf << "\"" << *it_vertex << "\" [label=\"";
			outf << *it_state << " {";
			pcount = 0;
			for (prp_it = (*it_state)->labeledPrp.begin();
				 prp_it != (*it_state)->labeledPrp.end(); prp_it++) {

				if (pcount > 0)
					outf << ", ";
				outf << "p" << *prp_it;
				pcount++;
			}
			outf << "}\\n";
			printFormula( (*it_vertex)->subformula, outf );
			outf << "\"]" << std::endl;

			for (PGVertexSet_it it_vertex_succ = (*it_vertex)->succVertices.begin();
				 it_vertex_succ != (*it_vertex)->succVertices.end(); it_vertex_succ++) {

				outf << "\"" << *it_vertex << "\" -> \"" << *it_vertex_succ << "\"" << std::endl;

			}

		}
	}
	outf << "}";
}


void ModelCheckerPG::optRewire( PGState *z_new )
{
	double c_new;
	bool updated;

	for (PGVertexSet_it z_new_v_it = z_new->vertices.begin();
		 z_new_v_it != z_new->vertices.end(); z_new_v_it++) {

		if ((*z_new_v_it)->subformula->parent != NULL
			&& (*z_new_v_it)->subformula->parent->type == PT_SUC) {
			// Like lines 21-28 of Algorithm 5 of the ACC 2012 paper

			updated = false;
			for (PGVertexSet_it rv_it = (*z_new_v_it)->RV.begin();
				 rv_it != (*z_new_v_it)->RV.end(); rv_it++) {

				list<PGVertex *>key_existing;
				key_existing.push_back( *rv_it );
				key_existing.push_back( *z_new_v_it );

				assert( Cost.find( key_existing ) != Cost.end() );

				for (PGVertexSet_it pre_it = (*z_new_v_it)->predVertices.begin();
					 pre_it != (*z_new_v_it)->predVertices.end(); pre_it++) {

					if ((*pre_it)->subformula->type != PT_SUC
						|| *((*pre_it)->subformula->children.begin()) != (*z_new_v_it)->subformula)
						continue;

					list<PGVertex *> key;
					key.push_back( *rv_it );
					key.push_back( *pre_it );

					assert( Cost.find( key ) != Cost.end() );

					c_new = Cost[key] + ((*pre_it)->state == (*z_new_v_it)->state ? 0 : (*pre_it)->state->edgeCost[(*z_new_v_it)->state]);
					if (c_new < Cost[key_existing]) {

						Pr[key_existing] = *pre_it;
						Cost[key_existing] = c_new;
						updated = true;

					}

				}

			}

			if (updated)
				propagateCost( *z_new_v_it );

		} else if ((*z_new_v_it)->subformula->type == PT_SUC) {
			// Like lines 29-36 of Algorithm 5 of the ACC 2012 paper

			for (PGVertexSet_it post_it = (*z_new_v_it)->succVertices.begin();
				 post_it != (*z_new_v_it)->succVertices.end(); post_it++) {

				if ((*post_it)->subformula != *((*z_new_v_it)->subformula->children.begin()))
					continue;

				updated = false;
				for (PGVertexSet_it rv_it = (*post_it)->RV.begin();
					 rv_it != (*post_it)->RV.end(); rv_it++) {

					if ((*z_new_v_it)->RV.find( *rv_it ) == (*z_new_v_it)->RV.end())
						continue;

					list<PGVertex *>key;
					key.push_back( *rv_it );
					key.push_back( *z_new_v_it );

					assert( Cost.find( key ) != Cost.end() );

					c_new = Cost[key] + ((*z_new_v_it)->state == (*post_it)->state ? 0 : (*z_new_v_it)->state->edgeCost[(*post_it)->state]);

					list<PGVertex *> key_existing;
					key_existing.push_back( *rv_it );
					key_existing.push_back( *post_it );

					assert( Cost.find( key_existing ) != Cost.end() );

					if (c_new < Cost[key_existing]) {

						Pr[key_existing] = *z_new_v_it;
						Cost[key_existing] = c_new;

						updated = true;

					}

				}

				if (updated)
					propagateCost( *post_it );

			}

		}
	}

}


void ModelCheckerPG::propagateCost( PGVertex *changed_vertex )
{
	PGVertexSet visited;
	PGVertexSet current;
	PGVertexSet next;
	PGVertexSet_it it_current_v;
	PGVertex *current_v;
	double c_new;
	for (PGStateSet_it it_state = this->states.begin();
		 it_state != this->states.end(); it_state++) {
		for (PGVertexSet_it it_start_v = (*it_state)->vertices.begin();
			 it_start_v != (*it_state)->vertices.end(); it_start_v++) {


			visited.clear();
			visited.insert( changed_vertex );
			current.clear();
			next.clear();
			next.insert( changed_vertex );
			while (next.size() > 0) {
				current.swap( next );
				while (current.size() > 0) {
					it_current_v = current.begin();
					current_v = *it_current_v;
					current.erase( it_current_v );

					if (current_v->RV.find( *it_start_v ) == current_v->RV.end())
						continue;

					list <PGVertex *>key_existing;
					key_existing.push_back( *it_start_v );
					key_existing.push_back( current_v );
					assert( Cost.find( key_existing ) != Cost.end() );
					for (PGVertexSet_it it_predecessor_v = current_v->predVertices.begin();
						 it_predecessor_v != current_v->predVertices.end(); it_predecessor_v++) {

						if ((*it_predecessor_v)->RV.find( *it_start_v ) == (*it_predecessor_v)->RV.end())
							continue;

						list <PGVertex *>key;
						key.push_back( *it_start_v );
						key.push_back( *it_predecessor_v );
						assert( Cost.find( key ) != Cost.end() );
						c_new = Cost[key] + ((*it_predecessor_v)->state == current_v->state ? 0 : (*it_predecessor_v)->state->edgeCost[current_v->state]);
						if (c_new < Cost[key_existing]) {
							Pr[key_existing] = *it_predecessor_v;
							Cost[key_existing] = c_new;
						}

					}

					for (PGVertexSet_it it_successor_v = current_v->succVertices.begin();
						 it_successor_v != current_v->succVertices.end(); it_successor_v++) {

						if (visited.find( *it_successor_v ) != visited.end())
							continue;

						next.insert( *it_successor_v );
						visited.insert( *it_successor_v );

					}
				}
			}

		}
	}
}


int getColor( PGVertex *vertex )
{
	if (vertex->subformula->type == PT_GFP) {
		if (vertex->subformula->AD % 2 == 0) {
			return vertex->subformula->AD;
		} else {
			return vertex->subformula->AD + 1;
		}
	} else if (vertex->subformula->type == PT_LFP) {
		if (vertex->subformula->AD % 2 == 0) {
			return vertex->subformula->AD + 1;
		} else {
			return vertex->subformula->AD;
		}
	}

	return 0;
}


MS_state::MS_state()
	: vertices(), successors(), predecessors(), labeledPrp()
{
	this->data = 0;
	this->identifier = -1;
	this->labeledPrp.clear();
}

PGState::PGState()
	: vertices(), successors(), predecessors(), labeledPrp()
{
	this->data = 0;
	this->labeledPrp.clear();
}


bool MS_state::addprop( int newprop )
{
	this->labeledPrp.insert( newprop );
	return true;
}

bool PGState::addprop( int newprop )
{
	this->labeledPrp.insert( newprop );
	return true;
}


ModelCheckerPG::ModelCheckerPG()
{
	this->initialState = NULL;
	this->initialVertex = NULL;

	this->states.clear();

	this->lV.clear();
	this->nV.clear();
	this->Pr.clear();
	this->Cost.clear();

	this->last_min_cost = -1.0;
}


rModelChecker::rModelChecker()
{
	this->initialState = NULL;
	this->initialVertex = NULL;

	this->states.clear();

	this->satVertices.clear();

	this->num_local_updates = 0;
	this->num_update_reachabilities = 0;
}


rModelChecker::~rModelChecker()
{ }


CT_vertex *
rModelChecker::addVertex( CT_vertex *parentVertex, MS_state *state,
						  PT_node *subformula )
{

#if !TRUST_ME
	if (this->states.find (state) == this->states.end()) {
		cout << "ERROR: rModelChecker::addVertex: state is not in this->states";
		exit (1);
	}
#endif

	CT_vertex *vertexNew;

	// Check whether the vertex already exists
	bool vertexFound = false;
	for (vertexSet_it iter = state->vertices.begin();
		 iter != state->vertices.end(); iter++) {
		CT_vertex *vertexCurr = *iter;
		if (vertexCurr->subformula == subformula){
			vertexNew = vertexCurr;
			vertexFound = true;
			break;
		}
	}

	// Create a new vertex if one does not exist
	if (!vertexFound) {
		vertexNew = new CT_vertex;
		vertexNew->state = state;
		vertexNew->subformula = subformula;
		vertexNew->succVertices.clear();
		vertexNew->predVertices.clear();
		vertexNew->reachingVertices.clear();
		PT_node *parentSubformula = parentVertex->subformula;

		if ((parentSubformula->type == PT_GFP)
			&& !(this->pt.compareFormulaSize( subformula, parentSubformula ))) {
			//   then place it to its own reachingVertices
			vertexNew->reachingVertices.insert( parentVertex );
#if VERBOSE_DBG
			cout << "Insert GFP to reachability" << endl;
#endif
		}
		state->vertices.insert (vertexNew);
	}

	// Update the predVertices and succVertices of both of the vertices
	if (vertexNew->predVertices.find( parentVertex ) == vertexNew->predVertices.end() )
		vertexNew->predVertices.insert( parentVertex );

	if (parentVertex->succVertices.find( vertexNew ) == parentVertex->succVertices.end() )
		parentVertex->succVertices.insert( vertexNew );

	bool reachabilityUpdateOccurred = this->UpdateReachability( parentVertex, vertexNew );

	if ((!vertexFound) || reachabilityUpdateOccurred)
		return vertexNew;

	return NULL;
}


bool // Returns true if a modification is made, it returns false otherwise
rModelChecker::UpdateReachability( CT_vertex *vertexFrom, CT_vertex *vertexTo )
{
	bool addedNewReachingVertex = false;


	this->num_update_reachabilities++;

	// Compute  vertexTo->reachingVertices =
	//          {vertex \in vertexFrom->reachingVertices : vertex->subformula >= vertexTo->subformula} \cup vertexTo->reachingVertices
	//  - The computation is done in linear time
	vertexSet_it iterTo = vertexTo->reachingVertices.begin();
	vertexSet_it iterToPrev = iterTo;
	vertexSet_it iterToEnd = vertexTo->reachingVertices.end();
	for (vertexSet_it iterFrom = vertexFrom->reachingVertices.begin();
		 iterFrom != vertexFrom->reachingVertices.end(); iterFrom++) {
		while ((*iterTo < *iterFrom) && (iterTo != iterToEnd)) {
			iterToPrev = iterTo;
			iterTo++;
		}
		if ((*iterFrom == *iterTo) && (iterTo != iterToEnd)) {
			//cout<< "yes" << endl;
			continue;
		}
		if (this->pt.compareFormulaSize( vertexTo->subformula,
										 (*iterFrom)->subformula))
			continue;
		vertexTo->reachingVertices.insert (iterToPrev, *iterFrom);
		addedNewReachingVertex = true;
	}

	// If vertexTo is a PT_VAR type subformula, then check if vertexTo is a sat node
	if ((vertexTo->subformula->type == PT_VAR) ) {
		// Check whether this node is reachable from a vertex = (state, BindingFormula(var))
		PT_node *bindingSubformula = this->pt.getBoundFormula (vertexTo->subformula);
		// Search the reaching vertices for vertex = (state, bindingSubformula)
		bool gfpLoopFound = false;
		for (vertexSet_it iter = vertexTo->reachingVertices.begin();
			 iter != vertexTo->reachingVertices.end(); iter++) {
			CT_vertex *vertexCurr = *iter;
			if ((vertexCurr->state == vertexTo->state)
				&& (vertexCurr->subformula == bindingSubformula)) {
				gfpLoopFound = true;
				break;
			}
		}
		if (gfpLoopFound) {
			this->satVertices.insert (vertexTo);
		}
	}

	// If added a new reaching vertex into reachingVertices in vertexTo,
	//   then run UpdateReachability for each successor vertex of vertexTo
	if (addedNewReachingVertex) {
		for (vertexSet_it iter = vertexTo->succVertices.begin();
			 iter != vertexTo->succVertices.end(); iter++) {
			this->UpdateReachability (vertexTo,*iter);
		}
	}

	return addedNewReachingVertex;
}


bool // Returns true if a witness is found, otherwise it returns false
rModelChecker::LocalUpdate( CT_vertex *vertex )
{

	this->num_local_updates++;

	MS_state *stateThis = vertex->state;
	PT_node *subformulaThis = vertex->subformula;

#if VERBOSE_DBG
	cout << "state : " << stateThis->identifier << " - subformula : " << subformulaThis->type << endl;
	cout << "  reachable states-subformula:" << endl;
	for (vertexSet_it iter = vertex->reachingVertices.begin();
		 iter != vertex->reachingVertices.end(); iter++){
		CT_vertex *reachingVertex = *iter;
		cout << "     - reaching state: "  << reachingVertex->state->identifier << endl;
	}
#endif

	bool foundWitness = false;

	// If subformulaThis is a suc-formula then make sure it is in the list of subformulae of this stateThis
	if (subformulaThis->type == PT_SUC) {
		if (stateThis->sucSubformulaeVertices.find (vertex) == stateThis->sucSubformulaeVertices.end()) {
			stateThis->sucSubformulaeVertices.insert (vertex);
#if VERBOSE_DBG
			cout << "  --> added suc-subformula : " << stateThis->identifier  << endl;
#endif
		}
	}

	// 1. ATOMIC PROPOSITIONS
	if (subformulaThis->type == PT_PRP || subformulaThis->type == PT_NPRP) {
		// Check whether this literal is satisfied in this state
		//   if so, then we found a witness since this node is reachable from root
		int prpCurr = ((PT_prp *)subformulaThis)->prp;
		if (stateThis->labeledPrp.find ( prpCurr ) != stateThis->labeledPrp.end() ) {
			if (subformulaThis->type == PT_PRP) {
				cout << "FOUND A WITNESS: PRP" << endl;
				this->satVertices.insert (vertex);
				foundWitness = true;
			}
		} else if (subformulaThis->type == PT_NPRP) {
			cout << "FOUND A WITNESS: NPRP" << endl;
			this->satVertices.insert (vertex);
			foundWitness = true;
		}
	}

	// 2. VARIABLES
	if (subformulaThis->type == PT_VAR) {
		// Check whether this node is reachable from a vertex = (state, BindingFormula(var))

		PT_node *bindingSubformula = this->pt.getBoundFormula (subformulaThis);

		// Search the reaching vertices for vertex = (state, bindingSubformula)
		bool gfpLoopFound = false;
		for (vertexSet_it iter = vertex->reachingVertices.begin();
			 iter != vertex->reachingVertices.end(); iter++) {
			CT_vertex *vertexCurr = *iter;
			if ((vertexCurr->state == stateThis)
				&& (vertexCurr->subformula == bindingSubformula))  {
				gfpLoopFound = true;
				break;
			}
		}

		// If vertex = (state, bindingSubformula) if found to be reaching,
		//    then declare wictory
		if (gfpLoopFound) {
			this->satVertices.insert (vertex);
			foundWitness = true;
		}

		// Create the new node with the bindingSubformula
		CT_vertex *vertexNew = this->addVertex (vertex, stateThis, bindingSubformula);
		if (vertexNew) {
			if (this->LocalUpdate (vertexNew))
				foundWitness = true;
		}
	}

	// 3. AND OPERATOR
	if (subformulaThis->type == PT_AND) {
#if !TRUST_ME
		if ( ((PT_operator *)subformulaThis)->children.size() != 2) {
			cout << "ERROR: rModelChecker::LocalUpdate: AND OPERATOR does not have 2 children" << endl;
			exit (1);
		}
#endif

		subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin();

		// Get left subformula
		PT_node *subformulaLeft = *iter;
		// Get right subformula
		iter++;
		PT_node *subformulaRight = *iter;

		PT_node *subformulaChild = NULL;

		if (subformulaLeft->type == PT_PRP) {

			// Create a new node using subformulaRight
			if (stateThis->labeledPrp.find( ((PT_prp *)subformulaLeft)->prp ) != stateThis->labeledPrp.end())
				subformulaChild = subformulaRight;

		} else if (subformulaRight->type == PT_PRP) {

			// Create a new node using subformulaLeft
			if (stateThis->labeledPrp.find( ((PT_prp *)subformulaRight)->prp ) != stateThis->labeledPrp.end())
				subformulaChild = subformulaLeft;

		} else if (subformulaLeft->type == PT_NPRP) {

			// Create a new node using subformulaRight
			if (stateThis->labeledPrp.find( ((PT_prp *)subformulaLeft)->prp ) == stateThis->labeledPrp.end())
				subformulaChild = subformulaRight;
		} else if (subformulaRight->type == PT_NPRP) {

			// Create a new node using subformulaLeft
			if (stateThis->labeledPrp.find( ((PT_prp *)subformulaRight)->prp ) == stateThis->labeledPrp.end())
				subformulaChild = subformulaLeft;

		} else {
			cout << "ERROR: rModelChecker::LocalUpdate: No child of the AND OPERATOR is a literal" << endl;
			exit (1);
		}

		/* If stateThis satisfies the proposition, then add the vertex and
		   update the reachability graph */
		if (subformulaChild) {
			CT_vertex *vertexNew = addVertex( vertex, stateThis, subformulaChild );
			if (vertexNew)
				if (this->LocalUpdate( vertexNew ))
					foundWitness = true;
		}
	}

	// 4. OR OPERATOR
	if (subformulaThis->type == PT_OR) {
#if !TRUST_ME
		if (((PT_operator *)subformulaThis)->children.size() != 2) {
			cout << "ERROR: rModelChecker::LocalUpdate: OR OPERATOR does not have 2 children" << endl;
			exit (1);
		}
#endif

		/* Add both of the child subformula to the model checker and update the
		   reachabilities */
		for (subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin();
			 iter != ((PT_operator *)subformulaThis)->children.end(); iter++) {
			PT_node *subformulaChild = *iter;
			CT_vertex *vertexNew = this->addVertex (vertex, stateThis,subformulaChild);
			if (vertexNew) {
				if (this->LocalUpdate( vertexNew ))
					foundWitness = true;
			}
			if (foundWitness)
				break;
		}
	}

	// 5. LFP OPERATOR
	if (subformulaThis->type == PT_LFP) {
#if !TRUST_ME
		if (((PT_operator *)subformulaThis)->children.size() != 1) {
			cout << "ERROR: rModelChecker::LocalUpdate: LFP OPERATOR does not have 1 children" << endl;
			exit (1);
		}
#endif

		subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin();
		PT_node *subformulaChild = *iter;

		CT_vertex *vertexNew = this->addVertex( vertex,stateThis, subformulaChild );
		if (vertexNew) {
			if (this->LocalUpdate (vertexNew))
				foundWitness = true;
		}
	}

	// 6. GFP OPEARATOR
	if (subformulaThis->type == PT_GFP) {
#if !TRUST_ME
		if (((PT_operator *)subformulaThis)->children.size() != 1) {
			cout << "ERROR: rModelChecker::LocalUpdate: GFP OPERATOR does not have 1 children" << endl;
			exit (1);
		}
#endif

		subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin();
		PT_node *subformulaChild = *iter;

		CT_vertex *vertexNew = this->addVertex (vertex,stateThis, subformulaChild);
		if (vertexNew) {
			if (this->LocalUpdate (vertexNew))
				foundWitness = true;
		}
	}

	// 7. SUC OPERATOR
	if (subformulaThis->type == PT_SUC) {
#if !TRUST_ME
		if ( ((PT_operator *)subformulaThis)->children.size() != 1) {
			cout << "ERROR: rModelChecker::LocalUpdate: LFP OPERATOR does not have 1 children" << endl;
			exit (1);
		}
#endif

		subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin();
		PT_node *subformulaChild = *iter;

		for (stateSet_it ssiter = stateThis->successors.begin();
			 ssiter != stateThis->successors.end(); ssiter++) {
			MS_state *stateSucc = *ssiter;
			CT_vertex *vertexNew = this->addVertex (vertex,stateSucc, subformulaChild);
			if (vertexNew) {
				if (this->LocalUpdate (vertexNew))
					foundWitness = true;
			}
			if (foundWitness)
				break;
		}
	}

	return foundWitness;
}


bool
ModelCheckerPG::addState( PGState *state )
{

#if !TRUST_ME

	for (PGStateSet_it iter = this->states.begin();
		 iter != this->states.end(); iter++)
		if (*iter == state) {
			cout << "ERROR: rModelChecker::addState: state already exists" << endl;
			exit(1);
		}
#endif

	if (this->initialState == NULL) {

		this->initialState = state;

		PGVertex *vertexNew = new PGVertex;
		vertexNew->state = state;
		vertexNew->subformula = this->pt.getRoot();
		vertexNew->succVertices.clear();
		vertexNew->predVertices.clear();
		vertexNew->RV.clear();
		vertexNew->color = getColor( vertexNew );

		this->initialVertex = vertexNew;

		/* Adding the initial vertex into the RV set of the initial vertex has
		   the same effect as the "dummy vertex" used in the ACC 2012 paper. */
		vertexNew->RV.insert( this->initialVertex );
		list<PGVertex *> key;
		key.push_back( this->initialVertex );
		key.push_back( this->initialVertex );
		Cost[key] = 0.0;

		state->vertices.clear();
		state->successors.clear();
		state->predecessors.clear();

		state->vertices.insert( vertexNew );

#if !TRUSTME
		this->states.insert( state );
#endif

		// TODO: Check for witness upon this special first step
		this->updateArena( NULL, this->initialVertex );

	} else {

#if !TRUSTME
		this->states.insert( state );
#endif

		state->vertices.clear();
		state->successors.clear();
		state->predecessors.clear();

	}

	return false;
}


bool
rModelChecker::addState( MS_state *state )
{

#if !TRUST_ME

	for (stateSet_it iter = this->states.begin();
		 iter != this->states.end(); iter++)
		if (*iter == state) {
			cout << "ERROR: rModelChecker::addState: state already exists" << endl;
			exit(1);
		}
#endif

	if (this->initialState == NULL) {

		this->initialState = state;

		CT_vertex *vertexNew = new CT_vertex;
		vertexNew->state = state;
		vertexNew->subformula = this->pt.getRoot();
		vertexNew->succVertices.clear();
		vertexNew->predVertices.clear();
		vertexNew->reachingVertices.clear();

//     vertexNew->reachingVertices.insert (vertexNew);
		this->initialVertex = vertexNew;

		state->vertices.clear();
		state->successors.clear();
		state->predecessors.clear();

		state->vertices.insert( vertexNew );

#if !TRUSTME
		this->states.insert( state );
#endif

		this->LocalUpdate( vertexNew );
	} else {

		// TODO: This block is redundant with the second part of the if-block above.
		this->states.insert( state );

		state->vertices.clear();
		state->successors.clear();
		state->predecessors.clear();

	}

	return false;
}


void
ModelCheckerPG::updateArena( PGVertex *vertex_from, PGVertex *vertex_to )
{
	// If edge between these vertices already exists, do nothing.
	if (vertex_from != NULL
		&& vertex_from->succVertices.find( vertex_to ) != vertex_from->succVertices.end())
		return;

	// Remove if path cannot be winning for Player 1, hence not correct
	PT_prp *conjunctPrp = NULL;
	if (vertex_to->subformula->type == PT_AND)
		conjunctPrp = this->pt.getConjunctPrp( (PT_operator *)(vertex_to->subformula) );
	if ((vertex_to->subformula->type == PT_PRP
		 && (vertex_to->state->labeledPrp.find( ((PT_prp *)vertex_to->subformula)->prp )
			 == vertex_to->state->labeledPrp.end()))
		|| (vertex_to->subformula->type == PT_NPRP
			&& (vertex_to->state->labeledPrp.find( ((PT_prp *)vertex_to->subformula)->prp )
				!= vertex_to->state->labeledPrp.end()))
		|| (vertex_to->subformula->type == PT_AND
			&& ((conjunctPrp->type == PT_PRP && (vertex_to->state->labeledPrp.find( conjunctPrp->prp )
												 == vertex_to->state->labeledPrp.end()))
				|| (conjunctPrp->type == PT_NPRP && (vertex_to->state->labeledPrp.find( conjunctPrp->prp )
													 != vertex_to->state->labeledPrp.end()))))) {

		// TODO: Like lines 3-7 of Algorithm 3 of the ACC 2012 paper
		vertex_to->state->vertices.erase( vertex_to );
		delete vertex_to;
		return;

	}

	if (vertex_from != NULL
		&& vertex_from->subformula->type == PT_GFP
		&& this->nV.find( vertex_from ) == this->nV.end()) {
		// TODO: && acceptVertex()
		this->nV.insert( vertex_from );
	}
	if ((vertex_to->subformula->type == PT_PRP
			|| vertex_to->subformula->type == PT_NPRP)
		&& this->lV.find( vertex_to ) == this->lV.end()) {
		this->lV.insert( vertex_to );
	}

	/* TODO: Add separately to V1 or V2 vertex sets corresponding to lines 13-16
	   in Algorithm 3 of the ACC 2012 paper, which may not be correct. */

	// Create the edge
	if (vertex_from != NULL) {
		vertex_from->succVertices.insert( vertex_to );
		vertex_to->predVertices.insert( vertex_from );
	}

	if (vertex_to->subformula->type == PT_AND
		&& ((conjunctPrp->type == PT_PRP
			 && (vertex_to->state->labeledPrp.find( conjunctPrp->prp )
				 != vertex_to->state->labeledPrp.end()))
			|| (conjunctPrp->type == PT_NPRP
				&& (vertex_to->state->labeledPrp.find( conjunctPrp->prp )
					== vertex_to->state->labeledPrp.end())))) {

		subformulaeSet_it sf = vertex_to->subformula->children.begin();
		if (*sf == conjunctPrp)
			sf++;

		PGVertex *vertex_to2 = NULL;
		for (PGVertexSet_it v_to = vertex_to->state->vertices.begin();
			 v_to != vertex_to->state->vertices.end(); v_to++) {
			if ((*v_to)->subformula == *sf) {
				vertex_to2 = *v_to;
				break;
			}
		}
		if (vertex_to2 == NULL) {
			vertex_to2 = new PGVertex;
			vertex_to2->state = vertex_to->state;
			vertex_to->state->vertices.insert( vertex_to2 );
			vertex_to2->subformula = *sf;
			vertex_to2->succVertices.clear();
			vertex_to2->predVertices.clear();
			vertex_to2->RV.clear();
			vertex_to2->color = getColor( vertex_to2 );
		}

		updateArena( vertex_to, vertex_to2 );

	}

	if (vertex_to->subformula->type == PT_OR) {

		subformulaeSet_it sf = vertex_to->subformula->children.begin();

		PGVertex *vertex_to2 = NULL;
		for (PGVertexSet_it v_to = vertex_to->state->vertices.begin();
			 v_to != vertex_to->state->vertices.end(); v_to++) {
			if ((*v_to)->subformula == *sf) {
				vertex_to2 = *v_to;
				break;
			}
		}
		if (vertex_to2 == NULL) {
			vertex_to2 = new PGVertex;
			vertex_to2->state = vertex_to->state;
			vertex_to->state->vertices.insert( vertex_to2 );
			vertex_to2->subformula = *sf;
			vertex_to2->succVertices.clear();
			vertex_to2->predVertices.clear();
			vertex_to2->RV.clear();
			vertex_to2->color = getColor( vertex_to2 );
		}

		sf++;
		PGVertex *vertex_to3 = NULL;
		for (PGVertexSet_it v_to = vertex_to->state->vertices.begin();
			 v_to != vertex_to->state->vertices.end(); v_to++) {
			if ((*v_to)->subformula == *sf) {
				vertex_to3 = *v_to;
				break;
			}
		}
		if (vertex_to3 == NULL) {
			vertex_to3 = new PGVertex;
			vertex_to3->state = vertex_to->state;
			vertex_to->state->vertices.insert( vertex_to3 );
			vertex_to3->subformula = *sf;
			vertex_to3->succVertices.clear();
			vertex_to3->predVertices.clear();
			vertex_to3->RV.clear();
			vertex_to3->color = getColor( vertex_to3 );
		}

		updateArena( vertex_to, vertex_to2 );
		updateArena( vertex_to, vertex_to3 );

	}

	if (vertex_to->subformula->type == PT_SUC) {
		for (PGStateSet_it next_state = vertex_to->state->successors.begin();
			 next_state != vertex_to->state->successors.end(); next_state++) {

			PGVertex *vertex_to2 = NULL;
			for (PGVertexSet_it v_to = (*next_state)->vertices.begin();
				 v_to != (*next_state)->vertices.end(); v_to++) {
				if ((*v_to)->subformula == *(vertex_to->subformula->children.begin())) {
					vertex_to2 = *v_to;
					break;
				}
			}
			if (vertex_to2 == NULL) {
				vertex_to2 = new PGVertex;
				vertex_to2->state = *next_state;
				vertex_to2->state->vertices.insert( vertex_to2 );
				vertex_to2->subformula = *(vertex_to->subformula->children.begin());
				vertex_to2->succVertices.clear();
				vertex_to2->predVertices.clear();
				vertex_to2->RV.clear();
				vertex_to2->color = getColor( vertex_to2 );
			}

			updateArena( vertex_to, vertex_to2 );

		}
	}

	if (vertex_to->subformula->type == PT_VAR) {

		PT_node *bf = this->pt.getBoundFormula( vertex_to->subformula );

		PGVertex *vertex_to2 = NULL;
		for (PGVertexSet_it v_to = vertex_to->state->vertices.begin();
			 v_to != vertex_to->state->vertices.end(); v_to++) {
			if ((*v_to)->subformula == bf) {
				vertex_to2 = *v_to;
				break;
			}
		}
		if (vertex_to2 == NULL) {
			vertex_to2 = new PGVertex;
			vertex_to2->state = vertex_to->state;
			vertex_to->state->vertices.insert( vertex_to2 );
			vertex_to2->subformula = bf;
			vertex_to2->succVertices.clear();
			vertex_to2->predVertices.clear();
			vertex_to2->RV.clear();
			vertex_to2->color = getColor( vertex_to2 );
		}

		updateArena( vertex_to, vertex_to2 );

	}

	if (vertex_to->subformula->type == PT_LFP
		|| vertex_to->subformula->type == PT_GFP) {

		subformulaeSet_it sf = vertex_to->subformula->children.begin();

		PGVertex *vertex_to2 = NULL;
		for (PGVertexSet_it v_to = vertex_to->state->vertices.begin();
			 v_to != vertex_to->state->vertices.end(); v_to++) {
			if ((*v_to)->subformula == *sf) {
				vertex_to2 = *v_to;
				break;
			}
		}
		if (vertex_to2 == NULL) {
			vertex_to2 = new PGVertex;
			vertex_to2->state = vertex_to->state;
			vertex_to->state->vertices.insert( vertex_to2 );
			vertex_to2->subformula = *sf;
			vertex_to2->succVertices.clear();
			vertex_to2->predVertices.clear();
			vertex_to2->RV.clear();
			vertex_to2->color = getColor( vertex_to2 );
		}

		updateArena( vertex_to, vertex_to2 );
	}

	if (vertex_from != NULL)
		updateVertexSets( vertex_from, vertex_to );
}


void
ModelCheckerPG::updateVertexSets( PGVertex *vertex_from, PGVertex *vertex_to )
{
	bool updatedVertexSet = false;

	double edge_cost = vertex_from->state == vertex_to->state ? 0 : vertex_from->state->edgeCost[vertex_to->state];

	for (PGVertexSet_it rv_it = vertex_from->RV.begin(); rv_it != vertex_from->RV.end(); rv_it++) {

		if (vertex_to->RV.find( *rv_it ) == vertex_to->RV.end()
			&& (vertex_to->color <= (*rv_it)->color || *rv_it == this->initialVertex)) {

			vertex_to->RV.insert( *rv_it );

			list<PGVertex *> key;
			key.push_back( *rv_it );
			key.push_back( vertex_to );

			list<PGVertex *> key_base;
			key_base.push_back( *rv_it );
			key_base.push_back( vertex_from );

			assert( Cost.find( key_base ) != Cost.end() );
			assert( Pr.find( key ) == Pr.end() );

			this->Pr[key] = vertex_from;
			this->Cost[key] = Cost[key_base] + edge_cost;

			updatedVertexSet = true;
		}
	}

	if (this->nV.find( vertex_from ) != this->nV.end()) {

		if (vertex_to->RV.find( vertex_from ) == vertex_to->RV.end()
			&& (vertex_to->color <= vertex_from->color || vertex_from == this->initialVertex)) {

			vertex_to->RV.insert( vertex_from );

			list<PGVertex *> key;
			key.push_back( vertex_from );
			key.push_back( vertex_to );

			this->Pr[key] = vertex_from;
			this->Cost[key] = edge_cost;

			updatedVertexSet = true;
		}

	}

	if (updatedVertexSet) {
		for (PGVertexSet_it next_v_to = vertex_to->succVertices.begin();
			 next_v_to != vertex_to->succVertices.end(); next_v_to++)
			updateVertexSets( vertex_to, *next_v_to );
	}
}


/* Version that restricts enumeration to only vertices that have been added to
   the arena thus far and have associated formula of the form suc f. This is
   different from AddTransition (page 739) in the ACC 2012 paper. */
bool
ModelCheckerPG::addTransition( PGState *state_from, PGState *state_to, double edge_cost )
{

	bool foundWitness = false;

#if !TRUST_ME
	if (state_from->successors.find (state_to) != state_from->successors.end()) {
     cerr << "ERROR: rModelChecker::addTransition: transition already exists" << endl;
     exit (-1);
	}
	if (state_to->predecessors.find (state_from) != state_to->predecessors.end() ) {
     cerr << "ERROR: rModelChecker::addTransition: transition already exists" << endl;
     exit (-1);
	}
#endif

	state_from->successors.insert( state_to );
	state_to->predecessors.insert( state_from );

	if (edge_cost >= 0)
		state_from->edgeCost[state_to] = edge_cost;


	for (PGVertexSet_it v_from = state_from->vertices.begin();
		 v_from != state_from->vertices.end(); v_from++) {
		if ((*v_from)->subformula->type != PT_SUC)
			continue;

		// Create arena vertices if needed
		PGVertex *vertex_to = NULL;
		for (PGVertexSet_it v_to = state_to->vertices.begin();
			 v_to != state_to->vertices.end(); v_to++) {
			if ((*v_to)->subformula->parent == (*v_from)->subformula) {
				vertex_to = *v_to;
				break;
			}
		}
		if (vertex_to == NULL) {
			vertex_to = new PGVertex;
			vertex_to->state = state_to;
			state_to->vertices.insert( vertex_to );
			vertex_to->subformula = *((*v_from)->subformula->children.begin());
			vertex_to->succVertices.clear();
			vertex_to->predVertices.clear();
			vertex_to->RV.clear();
			vertex_to->color = getColor( vertex_to );
		}

		updateArena( *v_from, vertex_to );

		if (this->lV.size() > 0) {
			std::cerr << "nonempty lV" << std::endl;
			foundWitness = true;
		} else {
			double min_loop_cost = -1;
			for (PGVertexSet_it nuv = this->nV.begin(); nuv != this->nV.end();
				 nuv++) {
				if ((*nuv)->RV.find( *nuv ) != (*nuv)->RV.end()) {
					list<PGVertex *> key;
					key.push_back( *nuv );
					key.push_back( *nuv );
					if (min_loop_cost < 0 || this->Cost[key] < min_loop_cost)
						min_loop_cost = this->Cost[key];
					foundWitness = true;
				}
			}
			if (foundWitness) {
				if (this->last_min_cost < 0 || min_loop_cost < this->last_min_cost) {
					this->last_min_cost = min_loop_cost;
					std::cerr << min_loop_cost << std::endl;
				}
			}
		}
	}

	return foundWitness;
}

bool
rModelChecker::addTransition( MS_state *state_from, MS_state *state_to )
{

	bool foundWitness = false;

#if !TRUST_ME
	if (state_from->successors.find (state_to) != state_from->successors.end()) {
     cerr << "ERROR: rModelChecker::addTransition: transition already exists" << endl;
     exit (-1);
	}
	if (state_to->predecessors.find (state_from) != state_to->predecessors.end() ) {
     cerr << "ERROR: rModelChecker::addTransition: transition already exists" << endl;
     exit (-1);
	}
#endif

	state_from->successors.insert( state_to );
	state_to->predecessors.insert( state_from );

	for (vertexSet_it iter = state_from->sucSubformulaeVertices.begin();
		 iter != state_from->sucSubformulaeVertices.end(); iter++) {

		CT_vertex *vertexCurr = *iter;
		PT_node *subformulaCurr = vertexCurr->subformula;

#if !TRUST_ME
		if (subformulaCurr->type != PT_SUC) {
			cout << "ERROR: rModelChecker::addTransition: Successor vertex does not encode a succcesor type " << endl;
			exit (1);
		}
		if (((PT_operator *)subformulaCurr)->children.size() != 1) {
			cout << "ERROR: rModelChecker::addTransition: Successor operator does not have 1 child " << endl;
			exit (1);
		}
#endif

		subformulaeSet_it sfiter = ((PT_operator *)subformulaCurr)->children.begin();
		PT_node *subformulaChild = *sfiter;

		CT_vertex *vertexNew = this->addVertex (vertexCurr, state_to, subformulaChild);
		if (vertexNew) {
			if (this->LocalUpdate (vertexNew))
				foundWitness = true;
		}
		if (foundWitness)
			break;

	}
	return foundWitness;
}

stateList
getStateTrajectoryBetweenVertices (CT_vertex *vertexInitial, CT_vertex *vertexFinal )
{

	vertexList *solutionVertexList = NULL;
	vertexListSet vertexListsCurr;

	vertexList *vertexListFinal = new vertexList;
	vertexListFinal->push_back (vertexFinal);

	vertexListsCurr.insert (vertexListFinal);

	// If the final formula is a PT_PRP type, then just trace back to the vertexInitial.
	vertexSet verticesProcessed;
	verticesProcessed.clear ();

	verticesProcessed.insert (vertexFinal);

	bool vertex_exists = true;
	while (vertex_exists) {

		vertex_exists = false;
		bool solutionFound = false;
		vertexListSet vertexListsNext;
		for (vertexListSet_it iter = vertexListsCurr.begin(); iter != vertexListsCurr.end(); iter++) {
			vertexList *vertexListThis = *iter;
			CT_vertex *vertexLastInList = *(vertexListThis->rbegin());
			if ( vertexLastInList == vertexInitial) {
// 	cout << "found initial vertex" << endl;
				solutionVertexList = vertexListThis;
				solutionFound = true;
				break;
			}
			for (vertexSet_it iter2 = vertexLastInList->predVertices.begin();
				 iter2 != vertexLastInList->predVertices.end(); iter2++) {
				CT_vertex *vertexPred = *iter2;
				if (verticesProcessed.find (vertexPred) == verticesProcessed.end() ) {
					vertexList *vertexListNew = new vertexList;
					for (vertexList_it iter3 = vertexListThis->begin();
						 iter3 != vertexListThis->end(); iter3++) {
						vertexListNew->push_back (*iter3);
					}
					vertexListNew->push_back (vertexPred);
					vertexListsNext.insert (vertexListNew);
					verticesProcessed.insert (vertexPred);
					vertex_exists = true;
				}
			}
		}
		if (!solutionFound) {
			for (vertexListSet_it iter = vertexListsCurr.begin();
				 iter != vertexListsCurr.end(); iter++) {
				vertexList *vertexListThis = *iter;
				delete vertexListThis;
			}
			vertexListsCurr.clear();
			for (vertexListSet_it iter = vertexListsNext.begin();
				 iter != vertexListsNext.end(); iter++) {
				vertexListsCurr.insert(*iter);
			}
		}
	}

  // revert back the stateList

//  cout << "Solution vertex list size :" << solutionVertexList->size() << endl;

	stateList trajectory;
	trajectory.clear();
	MS_state *stateLastInTrajectory = NULL;
	for (vertexList_rit iter = solutionVertexList->rbegin();
		 iter != solutionVertexList->rend(); iter++) {
		CT_vertex *vertexThis = *iter;
		MS_state *stateThis = vertexThis->state;
		if (stateThis != stateLastInTrajectory) {
			trajectory.push_back (stateThis);
			stateLastInTrajectory = stateThis;
		}
	}

	return trajectory;

}


PGStateList
ModelCheckerPG::getTrajectory()
{

	PGStateList trajectory;

	if (this->lV.size() > 0) {

		// TODO
		assert( false );

	} else {

		PGVertex *loopv = NULL;
		double min_loop_cost = -1;  // Ignored until loopv != NULL
		for (PGVertexSet_it nuv = this->nV.begin(); nuv != this->nV.end();
			 nuv++) {
			if ((*nuv)->RV.find( *nuv ) != (*nuv)->RV.end()) {
				list<PGVertex *> key;
				key.push_back( *nuv );
				key.push_back( *nuv );
				if (loopv == NULL || this->Cost[key] < min_loop_cost) {
					loopv = *nuv;
					min_loop_cost = this->Cost[key];
				}
			}
		}
		if (loopv == NULL)  // No feasible nu-loop
			return trajectory;

		PGVertex *knotv = NULL;
		double min_prefix_cost = -1;

		PGVertex *curr = loopv;
		do {

			list<PGVertex *> key;
			key.push_back( loopv );
			key.push_back( curr );
			curr = Pr[key];

			key.clear();
			key.push_back( this->initialVertex );
			key.push_back( curr );
			if (knotv == NULL || Cost[key] < min_prefix_cost) {
				knotv = curr;
				min_prefix_cost = Cost[key];
			}
			trajectory.push_front( curr->state );

		} while (curr != loopv);

		while (*(trajectory.begin()) != knotv->state) {
			PGState *front = *(trajectory.begin());
			trajectory.pop_front();
			trajectory.push_back( front );
		}

		curr = knotv;
		do {

			list<PGVertex *> key;
			key.push_back( this->initialVertex );
			key.push_back( curr );
			assert( Pr.find( key ) != Pr.end() );
			curr = Pr[key];
			trajectory.push_front( curr->state );

		} while (curr != this->initialVertex);

		std::cerr << "ModelCheckerPG::getTrajectory(): returned trajectory cost is "
				  << min_prefix_cost << " + " << min_loop_cost
				  << " = " << min_prefix_cost + min_loop_cost << std::endl;

	}

	return trajectory;
}


stateList
rModelChecker::getTrajectory()
{

	stateList trajectory;

	if (this->satVertices.empty())
		return trajectory;

	// Pick one of the sat vertices:
	vertexSet_it iterVertexFinal = this->satVertices.begin();
	CT_vertex *vertexFinal = *iterVertexFinal;

	MS_state *stateFinal = vertexFinal->state;
	PT_node *subformulaFinal = vertexFinal->subformula;

	if (subformulaFinal->type == PT_PRP) {

		trajectory = getStateTrajectoryBetweenVertices( this->initialVertex, vertexFinal );

//    cout << "Trajectory num states     :" << trajectory.size() << endl;

	} else if (subformulaFinal->type == PT_VAR) {

		// Get the loop
		stateList trajectoryLoop;
		CT_vertex *vertexInitial = NULL;
		PT_node *subformulaInitial = this->pt.getBoundFormula( subformulaFinal );
		for ( vertexSet_it iter = stateFinal->vertices.begin(); iter != stateFinal->vertices.end(); iter++ ) {
			if ( (*iter)->subformula == subformulaInitial ) {
				vertexInitial = *iter;
				break;
			}
		}
		if (vertexInitial == NULL) {
			cout << "ERROR: rModelChecker::getTrajectory: no loop found even though claimed earlier" << endl;
			exit (1);
		}
		trajectoryLoop = getStateTrajectoryBetweenVertices (vertexInitial, vertexFinal);

		// Get the path that connects initialVertex to the loop
		stateList trajectoryPath;
		trajectoryPath = getStateTrajectoryBetweenVertices( this->initialVertex, vertexInitial );

		// TODO: Only run loop if being verbose?
		for (stateList_it iter = trajectoryPath.begin();
			 iter != trajectoryPath.end(); iter++)  {
			for (stateList_it iter2 = trajectoryPath.begin();
				 iter2 != iter; iter2++) {
//	if (*iter2 == *iter)
//	  cout << "Path contains a minor loop" << endl;
			}
		}

		// TODO: Only run loop if being verbose?
		int k1 = 0;
		for (stateList_it iter = trajectoryLoop.begin();
			 iter != trajectoryLoop.end(); iter++)  {
			int k2 = 0;
			for (stateList_it iter2 = trajectoryLoop.begin();
				 iter2 != iter; iter2++) {
//	if ( (*iter2 == *iter) && (iter2 != trajectoryLoop.begin()) )
//	  cout << "Loop contains a minor loop : " << k1 << " - " << k2 << endl;
				k2++;
			}
			k1++;
		}
//    cout << "k1 : " << trajectoryLoop.size() << endl;

    // Concatanate trajectoryPath and trajectoryLoop to form trajectory
//     int k =0;
//     for (stateList_it iter = trajectoryPath.begin(); iter != trajectoryPath.end(); iter++) {
//       k++;
//       if (k > 20)
// 	continue;
//       trajectory.push_back (*iter);
//     }
//     int k = 0;
//     for (stateList_it iter = trajectoryLoop.begin(); iter != trajectoryLoop.end(); iter++)  {
//       k++;
//       if ( (k > 124) )
// 	continue;

//       trajectory.push_back (*iter);
//     }
//     cout << "k :" << k << endl;


		for (stateList_it iter = trajectoryPath.begin();
			 iter != trajectoryPath.end(); iter++)
			trajectory.push_back (*iter);
    for (stateList_it iter = trajectoryLoop.begin();
		 iter != trajectoryLoop.end(); iter++)
		trajectory.push_back (*iter);

	} else {
		cout << "ERROR: rModelChecker::getTrajectory: final subformula type is neither PT_PRP nor PT_VAR" << endl;
		exit(1);
	}

	return trajectory;
}


MS_state *
rModelChecker::findStateById( int id )
{

	MS_state *state = NULL;

	for (stateSet_it iter = this->states.begin();
		 iter != this->states.end(); iter++)
		if ( (*iter)->identifier == id)
			state = *iter;

	return state;
}


bool
rModelChecker::addTransitionById( int id_from, int id_to )
{

	MS_state *state_from = this->findStateById( id_from );
	MS_state *state_to = this->findStateById( id_to );

#if !TRUST_ME
	if ((state_from == NULL) || (state_to == NULL)) {
		cout << "ERROR: rModelChecker::addTransitionbyID: state not found " << endl;
		exit(1);
	}
#endif

	return this->addTransition (state_from, state_to);

}
