#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>


#if LIBBOT_PRESENT
#include <common/globals.h>
#include "./../../../libbot/src/lcmtypes/lcmtypes.h"
#endif

using namespace std;


#include "systems/single_integrator.hpp"
#include "rrts.hpp"


typedef SingleIntegrator::StateType state_t;
typedef SingleIntegrator::TrajectoryType trajectory_t;
typedef SingleIntegrator::SystemType system_t;

typedef RRTstar::Vertex <SingleIntegrator> vertex_t;
typedef RRTstar::Planner <SingleIntegrator> planner_t; 


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

  system_t system;
  
  system.setNumDimensions (3);
  
  system.regionOperating.setNumDimensions(3);
  system.regionOperating.center[0] = 0.0;
  system.regionOperating.center[1] = 0.0;
  system.regionOperating.center[2] = 0.0;
  system.regionOperating.size[0] = 20.0;
  system.regionOperating.size[1] = 20.0;
  system.regionOperating.size[2] = 10.0;
  
  system.regionGoal.setNumDimensions(3);
  system.regionGoal.center[0] = 2.0;
  system.regionGoal.center[1] = 2.0;
  system.regionGoal.center[2] = 2.0;
  system.regionGoal.size[0] = 2.0;
  system.regionGoal.size[1] = 2.0;
  system.regionGoal.size[2] = 2.0;
  
  
  region *obstacle;
  
  obstacle = new region;
  obstacle->setNumDimensions(3);
  obstacle->center[0] = 0.5;
  obstacle->center[1] = 0.5;
  obstacle->center[2] = 0.5;
  obstacle->size[0] = 0.5;
  obstacle->size[1] = 0.5;
  obstacle->size[2] = 0.5;
  
  system.obstacles.push_front (obstacle);
  
  
  rrts.setSystem (system);
  
  vertex_t &root = rrts.getRootVertex();  
  state_t &rootState = root.getState();
  rootState[0] = 0.0;
  rootState[1] = 0.0;
  rootState[2] = 0.0;
  
  rrts.initialize ();
  
  rrts.setGamma (1.5);
  rrts.setGoalSampleFrequency (0.1);
  
  clock_t start = clock();
  
  for (int i = 0; i < 10000; i++) 
    rrts.iteration ();
  
  clock_t finish = clock();
  cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;
    
#if LIBBOT_PRESENT
  publishTree (lcm, rrts, system);
  publishTraj (lcm, rrts, system);
#endif  

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
    if (system.getNumDimensions() > 2)
      opttraj->states[stateIndex].z = stateRef[2];
    else
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


  bool plot3d = (system.getNumDimensions() > 2);

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
      if (plot3d) 
	graph->vertices[vertexIndex].state.z = stateCurr[2];
      else 
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
      if (plot3d)
	graph->edges[edgeIndex].vertex_src.state.z = stateParent[2];
      else 
	graph->edges[edgeIndex].vertex_src.state.z = 0.0;


      graph->edges[edgeIndex].vertex_dst.state.x = stateCurr[0];
      graph->edges[edgeIndex].vertex_dst.state.y = stateCurr[1];
      if (plot3d)
	graph->edges[edgeIndex].vertex_dst.state.z = stateCurr[2];
      else 
	graph->edges[edgeIndex].vertex_dst.state.z = 0.0;

      graph->edges[edgeIndex].trajectory.num_states = 0;
      graph->edges[edgeIndex].trajectory.states = NULL;
      
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
