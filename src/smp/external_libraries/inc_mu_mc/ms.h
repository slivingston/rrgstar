#ifndef __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_MS_
#define __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_MS_

#include <iostream>
#include <fstream>
#include <set>
#include <map>
#include <list>
#include <algorithm>
#include <utility>

#include <cstdlib>
#include <cstdio>

#include "pt.h"

#define TRUST_ME 0
#define VERBOSE_DBG 0

using namespace std;

class CT_vertex;
class MS_state;
class PGVertex;
class PGState;

typedef set<CT_vertex *> vertexSet;
typedef set<CT_vertex *>::iterator vertexSet_it;
typedef set<PGVertex *> PGVertexSet;
typedef set<PGVertex *>::iterator PGVertexSet_it;
typedef list<CT_vertex *> vertexList;
typedef list<CT_vertex *>::iterator vertexList_it;
typedef list<CT_vertex *>::reverse_iterator vertexList_rit;
typedef set<MS_state*> stateSet;
typedef set<MS_state*>::iterator stateSet_it;
typedef set<PGState*> PGStateSet;
typedef set<PGState*>::iterator PGStateSet_it;
typedef map<MS_state*, stateSet> stateMap;
typedef map<MS_state*, vertexSet> stateVertexSetMap;
typedef list<MS_state*> stateList;
typedef list<MS_state*>::iterator stateList_it;
typedef list<PGState*> PGStateList;
typedef list<PGState*>::iterator PGStateList_it;
typedef set<vertexList*> vertexListSet;
typedef set<vertexList*>::iterator vertexListSet_it;


class MS_state {
public:

    void *data;  // A data element

	int identifier;  // A unique identifier for the state

    vertexSet vertices;  // A set of vertices that involve this state
    stateSet successors;    // A set of successor states
    stateSet predecessors;  // A set of predecessor states

	// A set of vertices that involve this state and a successor subformula
    vertexSet sucSubformulaeVertices;

	// All the labeled propositions that the state satisfies
	propositionSet labeledPrp;

	MS_state();

	/* Add an atomic proposition to the list of atomic propositions satisfied in
	   this state. */
	bool addprop( int newprop );

};


class CT_vertex {

public:

	CT_vertex *parent;
	vertexSet children;
	bool ReturnValue;  // TODO: Where is or was this used?

	MS_state *state;       // pointer to the state that this vertex encodes
	PT_node  *subformula;  // pointer to the subformula that this vertex encodes

    vertexSet succVertices;  // All the successor vertices of this vertex
    vertexSet predVertices;  // All the predecessor vertices of this vertex

	// The nu-vertices from which this node is reachable.
	/* This corresponds to "Reaches" (the authors later also call it "Reach") in
	   the CDC 2009 paper, pages 2227-2228. */
    vertexSet reachingVertices;

};

class PGState {
public:

    void *data;  // A data element

	map<PGState *, double> edgeCost;
    PGVertexSet vertices;  // A set of vertices that involve this state
    PGStateSet successors;    // A set of successor states
    PGStateSet predecessors;  // A set of predecessor states

	// All the labeled propositions that the state satisfies
	propositionSet labeledPrp;

	PGState();

	/* Add an atomic proposition to the list of atomic propositions satisfied in
	   this state. */
	bool addprop( int newprop );

};

class PGVertex {
public:

	PGVertex *parent;
	PGVertexSet children;

	int color;  // Also known as chi-value

	PGState *state;       // pointer to the state that this vertex encodes
	PT_node  *subformula;  // pointer to the subformula that this vertex encodes

    PGVertexSet succVertices;  // All the successor vertices of this vertex
    PGVertexSet predVertices;  // All the predecessor vertices of this vertex

	// As in the ACC 2012 paper
	PGVertexSet RV;
};


class rModelChecker {
public:
    ParseTree pt;  // Parse tree

    MS_state *initialState;    // Root state
    CT_vertex *initialVertex;  // Root vertex

	/* The set of all ms_state* that are currently expanded (reachable to the
	   initial vertex). */
    stateSet states;

	/* These are either satisfied literal nodes or nodes with variables that
	   manage find a nu-loop. */
    vertexSet satVertices;

    int num_local_updates;
    int num_update_reachabilities;

//     vertexSetIterPair_t *vertexSetIterPairArray;

    rModelChecker();
    ~rModelChecker();

	// Add a ms_state* to the model
	bool addState( MS_state *state );

	/* Add a transition from an ms_state* to another ms_state* NOTE: both
	   ms_state* have to be added earlier. */
	bool addTransition( MS_state *state_from, MS_state *state_to );

    bool addTransitionById( int id_from, int id_to );
    MS_state* findStateById( int id );

    stateList getTrajectory();

private:   // other hidden functions
	/* Create new vertex and return address.
	   If the vertex exists, return address only. */
    CT_vertex *addVertex( CT_vertex *parentVertex, MS_state *state,
						  PT_node *subformula );

	// Expand the node
    bool LocalUpdate (CT_vertex *vertex);

	// Update the reachability information in CT vertex
    bool UpdateReachability (CT_vertex *vertexFrom, CT_vertex *vertexTo);
};


class ModelCheckerPG {
public:
    ParseTree pt;  // Parse tree

    PGState *initialState;  // Root state
    PGVertex *initialVertex;  // Root vertex

	/* The set of all ms_state* that are currently expanded (reachable to the
	   initial vertex). */
    PGStateSet states;

    ModelCheckerPG();

	void updateArena( PGVertex *vertex_from, PGVertex *vertex_to );
	void updateVertexSets( PGVertex *vertex_from, PGVertex *vertex_to );

	// Add a ms_state* to the model
	bool addState( PGState *state );

	/* Add a transition from an ms_state* to another ms_state* NOTE: both
	   ms_state* have to be added earlier.

	   If edge_cost is nonnegative, then it is assigned as the cost of
	   transitioning from state_from to state_to. If it is negative (default),
	   it is ignored. */
	bool addTransition( PGState *state_from, PGState *state_to, double edge_cost = -1 );

    PGStateList getTrajectory();

	/* Implement corrected lines 21-36 of Algorithm 5 of the ACC 2012 paper.
	   The function name is supposed to abbreviate "optimization rewiring". */
	void optRewire( PGState *z_new );

	void propagateCost( PGVertex *changed_vertex );

	/* Print game arena to given file stream in DOT format. */
	void dumpDOT( std::ofstream &outf );

	double getCurrentMinCost() const
	{ return last_min_cost; }

private:

	double last_min_cost;

	// As in the ACC 2012 paper
	PGVertexSet lV;
	PGVertexSet nV;
	map<list<PGVertex *>, PGVertex*> Pr;
	map<list<PGVertex *>, double> Cost;

};


#if 0
class ModelSynthesizer {
public:
	MS_state *initialState;
	CT_vertex *initialVertex;

	stateSet states;
	stateMap successors;
	stateMap predecessors;

	stateVertexSetMap sucSubformulae;

	ParseTree pt;

	ModelSynthesizer ();
	~ModelSynthesizer ();

// 	int addNewStateToStructure (MS_state *newState, stateSet succs, stateSet preds);

	CT_vertex *addNode (CT_vertex *vertex, MS_state *state, PT_node *subformula);

	bool localUpdate (CT_vertex *vertex);

	bool addState (MS_state *state);

	bool addTransition (MS_state *state_from, MS_state *state_to);

	MS_state *findStatebyID(int identifier);

// 	int addTransitionbyID (int identifier_from, int indentifier_to);

	stateList getTrajectory ();
};
#endif

#endif
