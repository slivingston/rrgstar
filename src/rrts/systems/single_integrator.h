#ifndef __RRTS_SYSTEM_H_
#define __RRTS_SYSTEM_H_

#include <vector>
#include <list>

using namespace std;


class region {

    int numDimensions;
    
public:    
    
    double *center;
    double *size;

    region ();
    ~region ();
    
    int setNumDimensions (int numDimensionsIn);
};



class State {

    int numDimensions;
    double *x;

    int setNumDimensions (int numDimensions);
    
public:
    
    State ();
    ~State ();
    State (const State &stateIn);
    State& operator= (const State &stateIn);
    double& operator[] (const int i) {return x[i];}
    
    friend class System;
    friend class Trajectory;
};


class Trajectory {
    
    State *endState; 
    double totalVariation;  
    
public:    
    
    Trajectory ();
    ~Trajectory ();
    Trajectory (const Trajectory &trajectoryIn);
    Trajectory& operator= (const Trajectory &trajectoryIn);
    
    int getEndState (State &endStateOut);
    State& getEndState () {return *endState;}
    State& getEndState () const {return *endState;}
    double evaluateCost ();
    
    friend class System;
};


class System {
    
    int numDimensions;
    bool IsInCollision (double *stateIn);

    State rootState;
    
public:    

    region regionOperating;
    region regionGoal;
    list<region*> obstacles;
    
    System ();
    ~System ();
    
    int setNumDimensions (int numDimensionsIn);

    int getNumDimensions () {return numDimensions;}    
    State& getRootState () {return rootState;}
    int getStateKey (State &stateIn, double *stateKey);

    bool isReachingTarget (State &stateIn);

    int sampleState (State &randomStateOut); 
    int sampleGoalState (State &randomStateOut); 

    int extendTo (State &stateFromIn, State &stateTowardsIn, 
                  Trajectory &trajectoryOut, bool &exactConnectionOut); 

    double evaluateExtensionCost (State &stateFromIn, State &stateTowardsIn, bool &exactConnectionOut);

    double evaluateCostToGo (State& stateIn);
    
    int getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut);

};



typedef struct _SingleIntegrator {

    typedef State StateType;
    typedef Trajectory TrajectoryType;
    typedef System SystemType;
    
} SingleIntegrator;

#endif
