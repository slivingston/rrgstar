#ifndef __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_PT_
#define __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_PT_


// Parse Tree

#include <iostream>
#include <fstream>
#include <string>
#include <set>

#include <cstdlib>
#include <cstdio>

using namespace std;

enum PT_type {
	PT_PRP = 1, // atomic proposition
	PT_NPRP, /* not proposition. Note that we are assuming negation only
			    operates directly on atomic propositions, which is consistent
			    with deterministic mu calculus. */
	PT_VAR,
	PT_LFP,
	PT_GFP,
	PT_SUC,
	PT_AND,
	PT_OR
};

class PT_node;
class PT_prp;
class PT_var;
class PT_operator;

typedef set<PT_node*> subformulaeSet;
typedef set<PT_node*>::iterator subformulaeSet_it;

typedef set<int> propositionSet;
typedef set<int>::iterator propositionSet_it;

class PT_node {
public: 
	PT_type type;
	PT_node *parent;
	subformulaeSet children;
	void printType ();
	int AD;  // Alternation depth
};

// For PT_PRP and PT_NPRP
class PT_prp:public PT_node {
public:
	int prp;
};

class PT_var:public PT_node {
public:
	int var;
};

class PT_operator:public PT_node {
public:
    int boundVar;  // -1 for operators other than LFP and GFP
};


void printFormula( PT_node *ptnode, std::ofstream &out );

class ParseTree {
public: 
	ParseTree ();
	~ParseTree ();
	int parseFormula (string s);
	bool isEmpty ();
// 	subformulaeSet& getSubformulaeSuc ();
// 	subformulaeSet& getSubformulaeMu ();
// 	subformulaeSet& getSubformulaeNu ();
// 	void SearchBranchForType (PT_node *node, subformulaeSet &sfSet, PT_type nodeType);

	PT_node *getRoot ();
	PT_node *getBoundFormula (PT_node *node_var);
	
	// Debug/Test/Visualization related functions
	void printParseTree (PT_node *ptnode);

    bool compareFormulaSize (PT_node *ptnode_a, PT_node *ptnode_b); // True if ptnode_a < ptnode_b in the parse tree
	
	// The following two functions are not used anymore...
// 	bool booleanCheck (PT_node *root, subformulaeSet &satisfiedSFThis, 
//                        subformulaeSet &satisfiedSFNext, propositionSet &satisfiedPrp);
// 	bool fixedpointCheck (PT_node *root, subformulaeSet &satisfiedSFThis, 
//                           subformulaeSet &satisfiedSFNext, propositionSet &satisfiedPrp, int var, bool lfp);

	// Not reentrant (i.e., not thread-safe)
	void sucCache( subformulaeSet &sucFormulae, PT_node *node );

	/* Return literal that is in given conjunct. Note that in the deterministic
	   mu-calculus, at least one of the precisely two children of an AND formula
	   is a literal, i.e., of type PT_PRP or PT_NPRP.

	   If andNode is not of type PT_AND or if some other error occurs, then NULL
	   is returned. */
	// TODO: This does not need to be a member function. Move it outside ParseTree
	PT_prp *getConjunctPrp( PT_operator *andNode );

	subformulaeSet sucFormulae;

private:
	PT_node *root;
};

#endif
