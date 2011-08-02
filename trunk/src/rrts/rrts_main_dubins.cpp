#define LIBBOT_PRESENT 1

#include <iostream>

#include <ctime>


#if LIBBOT_PRESENT
#include <common/globals.h>
#include "./../../../libbot/src/lcmtypes/lcmtypes.h"
#endif


using namespace std;

#include "systems/dubins_car.hpp"
#include "rrts.hpp"

typedef DubinsCar::StateType state_t;
typedef DubinsCar::TrajectoryType trajectory_t;
typedef DubinsCar::SystemType system_t;

typedef RRTstar::Vertex <DubinsCar> vertex_t;
typedef RRTstar::Planner <DubinsCar> planner_t; 


#if LIBBOT_PRESENT
int publishTree (lcm_t *lcm, planner_t& planner, system_t& system);
int publishTraj (lcm_t *lcm, planner_t& planner, system_t& system);
#endif


int main () {
  
  
  planner_t rrts;
  
  cout << "RRTstar is alive" << endl;
  

#if LIBBOT_PRESENT
  lcm_t *lcm = globals_get_lcm ();
#endif
  

  // Initialize the system
  //   : Here we initialize the operating region and the obstacles
  //     as a list. However, the System class code can be modified 
  //     easily to change the obstacle representation to a cost map
  
  system_t system;
    
  system.regionOperating.center[0] = 10.0;
  system.regionOperating.center[1] = 0.0;
  system.regionOperating.center[2] = 0.0;
  system.regionOperating.size[0] = 30.0;
  system.regionOperating.size[1] = 8.0;
  system.regionOperating.size[2] = 2.0 * M_PI;
  
  system.regionGoal.center[0] = 15.0;
  system.regionGoal.center[1] = 0.0;
  system.regionGoal.center[2] = 0.0;
  system.regionGoal.size[0] = 2.0;
  system.regionGoal.size[1] = 2.0;
  system.regionGoal.size[2] = 0.1 * M_PI;

  region *obstacle;
  obstacle = new region;
  obstacle->center[0] = 7.0;
  obstacle->center[1] = 0.0;
  obstacle->center[2] = 0.0;
  obstacle->size[0] = 5.0;
  obstacle->size[1] = 5.0;
  obstacle->size[2] = 0.0;
  system.obstacles.push_front (obstacle);
  
  // Tell the planner the system class
  rrts.setSystem (system);

  // Create the root vertex
  vertex_t &root = rrts.getRootVertex();  
  state_t &rootState = root.getState();
  rootState[0] = 0.0;
  rootState[1] = 0.0;
  rootState[2] = 0.0;

  // Initialize the planner
  rrts.initialize ();
  
  // Set planner parameters
  rrts.setGamma (2.0);
  rrts.setGoalSampleFrequency (0.1);



  // Run the planner until the goal region is reached 
  clock_t start = clock();
  clock_t timePrev = clock ();

  int iterationCounter = 0;
  while (1) {
    
    clock_t timeCurr = clock();
    if (((double)(timeCurr-timePrev))/CLOCKS_PER_SEC < 1.5)  {
      if (iterationCounter%100 == 0) {
	cout << "Cost : " << rrts.getBestVertexCost() << endl;
	publishTraj (lcm, rrts, system);
	publishTree (lcm, rrts, system);
      }
      rrts.iteration ();
      iterationCounter++;
      
    }
    else {

      publishTree(lcm, rrts, system);
      break;
      
      if (rrts.switchRoot (5.0) <= 0)
	break;
      timePrev = clock ();
    }

  }

  clock_t finish = clock();
  cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;


  return 1;
}




#if LIBBOT_PRESENT

int publishTraj (lcm_t *lcm, planner_t& planner, system_t& system) {


  cout << "Publishing trajectory -- start" << endl;
  
  vertex_t& vertexBest = planner.getBestVertex ();

  if (&vertexBest == NULL) {
    cout << "No best vertex" << endl;
    return 0;
  }

  list<double*> stateList;

  planner.getBestTrajectory (stateList);
  
  lcmtypes_smp_trajectory_t *opttraj = (lcmtypes_smp_trajectory_t *) malloc (sizeof (lcmtypes_smp_trajectory_t));
  
  opttraj->num_states = stateList.size();
  opttraj->states = (lcmtypes_smp_state_t *) malloc (opttraj->num_states * sizeof (lcmtypes_smp_state_t));
  
  int stateIndex = 0;
  for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
    
    double* stateRef = *iter;
    opttraj->states[stateIndex].x = stateRef[0];
    opttraj->states[stateIndex].y = stateRef[1];
    opttraj->states[stateIndex].z = 0.0;
    
    delete [] stateRef;

    stateIndex++;
  }
  

  lcmtypes_smp_trajectory_t_publish (lcm, "SMP_TRAJECTORY", opttraj);

  lcmtypes_smp_trajectory_t_destroy (opttraj);

  cout << "Publishing trajectory -- end" << endl;
  


  return 1;
}


int publishTree (lcm_t *lcm, planner_t& planner, system_t& system) {

  
  cout << "Publishing the tree -- start" << endl;


  lcmtypes_smp_graph_t *graph = (lcmtypes_smp_graph_t *) malloc (sizeof (lcmtypes_smp_graph_t));
  graph->num_vertices = planner.numVertices; 
  
  
  if (graph->num_vertices > 0) {    
    
    graph->vertices = (lcmtypes_smp_vertex_t *) malloc (graph->num_vertices * sizeof(lcmtypes_smp_vertex_t));

    int vertexIndex = 0;
    for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {

      
      vertex_t &vertexCurr = **iter;
      state_t &stateCurr = vertexCurr.getState ();
      
      graph->vertices[vertexIndex].state.x = stateCurr[0];
      graph->vertices[vertexIndex].state.y = stateCurr[1];
      graph->vertices[vertexIndex].state.z = 0.0;
      
      vertexIndex++;
      
    }
    
  }
  else {
    graph->vertices = NULL;
  }
  
  if (graph->num_vertices > 1) {
    
    graph->num_edges = graph->num_vertices - 1;
    graph->edges = (lcmtypes_smp_edge_t *) malloc (graph->num_edges * sizeof(lcmtypes_smp_edge_t));
    

    int edgeIndex = 0;
    for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
      
      vertex_t &vertexCurr = **iter;
      
      vertex_t &vertexParent = vertexCurr.getParent();

      if ( &vertexParent == NULL ) 
  	continue;

      state_t &stateCurr = vertexCurr.getState ();
      state_t &stateParent = vertexParent.getState();
      

      graph->edges[edgeIndex].vertex_src.state.x = stateParent[0];
      graph->edges[edgeIndex].vertex_src.state.y = stateParent[1];
      graph->edges[edgeIndex].vertex_src.state.z = 0.0;


      graph->edges[edgeIndex].vertex_dst.state.x = stateCurr[0];
      graph->edges[edgeIndex].vertex_dst.state.y = stateCurr[1];
      graph->edges[edgeIndex].vertex_dst.state.z = 0.0;
      
      list<double*> trajectory;
      if (system.getTrajectory (stateParent, stateCurr, trajectory) == 0) {
	cout << "ERROR: Trajectory can not be regenerated" << endl;
	return 0;
      }
            
      graph->edges[edgeIndex].trajectory.num_states = trajectory.size();
      
      if ( graph->edges[edgeIndex].trajectory.num_states > 0 ) {
	graph->edges[edgeIndex].trajectory.states 
	  = (lcmtypes_smp_state_t *) malloc (graph->edges[edgeIndex].trajectory.num_states * sizeof (lcmtypes_smp_state_t));
	
	int stateIndex = 0;
	for (list<double*>::iterator it_state = trajectory.begin(); it_state != trajectory.end(); it_state++) {
	  double *stateCurr = *it_state;
	  graph->edges[edgeIndex].trajectory.states[stateIndex].x = stateCurr[0];
	  graph->edges[edgeIndex].trajectory.states[stateIndex].y = stateCurr[1];
	  graph->edges[edgeIndex].trajectory.states[stateIndex].z = 0.0;
	  stateIndex++;
	  delete [] stateCurr;
	}
	
      }
      edgeIndex++;
    }

  }
  else {
    graph->num_edges = 0;
    graph->edges = NULL;
  }

  lcmtypes_smp_graph_t_publish (lcm, "SMP_GRAPH", graph);

  lcmtypes_smp_graph_t_destroy (graph);

  cout << "Publishing the tree -- end" << endl;

  return 1;
}

#endif
