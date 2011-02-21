#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <gdk/gdkkeysyms.h>

#include <lcm/lcm.h>

#include <bot/bot_core.h>
#include <bot/gtk/gtk_util.h>
#include <bot/viewer/viewer.h>
#include <bot/gl/gl_util.h>

#include <common/geometry.h>
#include <common/globals.h>

#include <lcmtypes/lcmtypes.h>

#define RENDERER_NAME "Tree"
#define MAX_HISTORY 1100

#define SLIDER_HISTORY "History"
#define CHECKBOX_RENDER_GOAL_TRAJ "Render Best Trajectory"

#define CHECKBOX_USE_CONST_NODES "Use Constant Nodes"
#define CHECKBOX_RUN_OPTRRT "Run OptRRT Algorithm"
#define SLIDER_NUM_ITERATIONS "Iterations" 
#define SLIDER_NUM_CONST_NODES "Const Nd" 
#define SLIDER_MSG_PUB_INTERVAL "Msg Interval" 


typedef struct _region_2d_t {
    double center[2];
    double size[2];
} region_2d_t;


typedef struct _tree_renderer_t {
    Renderer renderer;    
    EventHandler ehandler;
    lcm_t *lcm; 
    Viewer *viewer;    
    BotGtkParamWidget *pw;
    
    int index_trees;
    int num_trees;
    lcmtypes_opttree_tree_t *trees[MAX_HISTORY];
    int slider_history_no;
    int max_num_iterations;
    gboolean use_const_nodes;
    int num_const_nodes;
    gboolean run_optrrt;
    int msg_pub_interval;
    gboolean render_goal_traj;

    GMutex *mutex;

    double mouseNodeXY[2];

    // Operating region
    int key_operating;
    region_2d_t region_operating;
    region_2d_t last_region_operating;

    // Goal region
    int key_goal;
    region_2d_t region_goal;
    region_2d_t last_region_goal;

    // Obstacles
    int key_obstacles;
    GSList *list_region_obstacles;
    region_2d_t *hover_obstacle_region; 
    region_2d_t last_region_obstacles;

    double last_xy[2];   // Used moving and resizing operating, goal, and obstacle regions

    double cost_last;    // cost of the current best path 

} ot_renderer_t;


void ot_renderer_draw (Viewer *viewer, Renderer *renderer);



void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *name,
                         ot_renderer_t *self) {

    self->slider_history_no = bot_gtk_param_widget_get_int (self->pw, SLIDER_HISTORY);
    
    self->run_optrrt = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_RUN_OPTRRT);

    self->use_const_nodes = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_USE_CONST_NODES);
    
    self->max_num_iterations = bot_gtk_param_widget_get_int (self->pw,  SLIDER_NUM_ITERATIONS) * 1000;

    self->num_const_nodes = bot_gtk_param_widget_get_int (self->pw,  SLIDER_NUM_CONST_NODES);

    self->msg_pub_interval = bot_gtk_param_widget_get_int (self->pw, SLIDER_MSG_PUB_INTERVAL );
    
    return;
}


static void
on_clear_button(GtkWidget *button, ot_renderer_t *self)
{

    g_mutex_lock (self->mutex);


    for (int i = 0; i < MAX_HISTORY; i++) 
        if (self->trees[i]) {
            lcmtypes_opttree_tree_t_destroy (self->trees[i]);
            self->trees[i] = NULL;
        }
    self->num_trees = 0;
    self->index_trees = 0;
    self->slider_history_no = 1;
    
    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_HISTORY, 0);
    

    bot_gtk_param_widget_modify_int (self->pw, SLIDER_HISTORY, 
                                     1, 2,
                                     1, self->slider_history_no);


    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_start_button(GtkWidget *button, ot_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    
    
    lcmtypes_opttree_command_t *opttree_command
        = (lcmtypes_opttree_command_t *) malloc (sizeof (lcmtypes_opttree_command_t));
    
    opttree_command->opttree_command = LCMTYPES_OPTTREE_COMMAND_T_START;
    
    opttree_command->max_num_iterations = self->max_num_iterations;
    opttree_command->msg_pub_interval = self->msg_pub_interval;

    opttree_command->max_num_iterations = self->max_num_iterations;
    opttree_command->num_const_nodes = self->num_const_nodes;
    opttree_command->run_optrrt = self->run_optrrt;
    opttree_command->use_const_nodes = self->use_const_nodes;

    opttree_command->environment.operating.center[0] = self->region_operating.center[0];
    opttree_command->environment.operating.center[1] = self->region_operating.center[1];
    opttree_command->environment.operating.size[0] = self->region_operating.size[0];
    opttree_command->environment.operating.size[1] = self->region_operating.size[1];

    opttree_command->environment.goal.center[0] = self->region_goal.center[0];
    opttree_command->environment.goal.center[1] = self->region_goal.center[1];
    opttree_command->environment.goal.size[0] = self->region_goal.size[0];
    opttree_command->environment.goal.size[1] = self->region_goal.size[1];

    opttree_command->environment.num_obstacles = g_slist_length (self->list_region_obstacles);

    double bounding_area = opttree_command->environment.operating.size[0]*opttree_command->environment.operating.size[1];
    
    double obs_area = 0;

    if (opttree_command->environment.num_obstacles == 0) {
        opttree_command->environment.obstacles = NULL;
    }
    else {
        opttree_command->environment.obstacles
            = (lcmtypes_region_2d_t *) malloc (opttree_command->environment.num_obstacles * sizeof (lcmtypes_region_2d_t));

        
        GSList *list_obstacle_region_curr = self->list_region_obstacles;
        int obstacle_index = 0;
        while (list_obstacle_region_curr) {
            region_2d_t *obstacle_region_curr = (region_2d_t *) (list_obstacle_region_curr->data);
            opttree_command->environment.obstacles[obstacle_index].center[0] = obstacle_region_curr->center[0];
            opttree_command->environment.obstacles[obstacle_index].center[1] = obstacle_region_curr->center[1];
            opttree_command->environment.obstacles[obstacle_index].size[0] = obstacle_region_curr->size[0];
            opttree_command->environment.obstacles[obstacle_index].size[1] = obstacle_region_curr->size[1];
            obs_area += obstacle_region_curr->size[0]*obstacle_region_curr->size[1];
            obstacle_index++;
            list_obstacle_region_curr = g_slist_next (list_obstacle_region_curr);
        }
        

    }

    printf ("Obstacle Area: %3.5lf, Remaining Area: %3.5lf, gamma_1: %3.5lf\n", 
            obs_area, bounding_area - obs_area, sqrt(bounding_area - obs_area) );


    lcmtypes_opttree_command_t_publish (self->lcm, "OPTTREE_COMMAND", opttree_command);
    
    lcmtypes_opttree_command_t_destroy (opttree_command);
    
    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


int save_scene_from_history (ot_renderer_t *self, int history_no, char name_ext[10]) {


    if (self->trees[history_no]) {

        lcmtypes_opttree_tree_t *tree_this = self->trees[history_no];


        // Print nodes
        char nodes_name[80] = ""; 
        strcat (nodes_name, "nodes");
        strcat (nodes_name, name_ext);
        strcat (nodes_name, ".txt");
        FILE *f_nodes = fopen (nodes_name, "wt");
        
        for (int i = 0; i < tree_this->num_nodes; i++) {
            lcmtypes_opttree_node_t *node_this = &(tree_this->nodes[i]);
            fprintf (f_nodes, "%d, %3.2lf, %3.2lf, %3.2lf\n", node_this->nodeid, 
                     node_this->state.x, node_this->state.y, node_this->distance_from_root);
        }
        
        fclose (f_nodes);

        
        // Print edges
        char edges_name[80] = "";
        strcat(edges_name, "edges");
        strcat(edges_name, name_ext);
        strcat(edges_name, ".txt");
        FILE *f_edges = fopen (edges_name, "wt");
        for (int i = 0; i < tree_this->num_edges; i++)
            fprintf (f_edges, "%d, %d \n", tree_this->edges[i][0] + 1, tree_this->edges[i][1] + 1);
        fclose (f_edges);

        
        // Check to whether there is a path reaching the goal region
        int minDistFromRootIndex = -1;
        double minDistFromRoot = DBL_MAX;
        for (int i = 0; i < tree_this->num_nodes; i++) {
            if ( (tree_this->nodes[i].state.x >= self->region_goal.center[0] - self->region_goal.size[0]/2.0) && 
                 (tree_this->nodes[i].state.x <= self->region_goal.center[0] + self->region_goal.size[0]/2.0) &&
                 (tree_this->nodes[i].state.y >= self->region_goal.center[1] - self->region_goal.size[1]/2.0) && 
                 (tree_this->nodes[i].state.y <= self->region_goal.center[1] + self->region_goal.size[1]/2.0) ) {
                
                // Calculate the distance from the root. 
                if (tree_this->nodes[i].distance_from_root < minDistFromRoot) {
                    minDistFromRoot = tree_this->nodes[i].distance_from_root;
                    minDistFromRootIndex = i;
                }
            }
        }

        
        // Print the optimal path
        char optpath_name[80] = "";
        strcat(optpath_name, "optpath");
        strcat(optpath_name, name_ext);
        strcat(optpath_name, ".txt");
        FILE *f_optpath = fopen (optpath_name, "wt");

        if (minDistFromRootIndex != -1) {
            

            int nodeCount = 0;
            
            int indexCurr = minDistFromRootIndex;
            int indexParent = -1;

            while (1) {
                for (int i = 0; i < tree_this->num_edges; i++) {         // Find the parent of the current index
                    if (tree_this->edges[i][0] == indexCurr){
                        indexParent = tree_this->edges[i][1];
                        break;
                    }
                }

                if (indexParent >= 0) 
                    fprintf (f_optpath, "%5.5lf, %5.5lf, %5.5lf, %5.5lf\n",
                             tree_this->nodes[indexParent].state.x, tree_this->nodes[indexParent].state.y,
                             tree_this->nodes[indexCurr].state.x, tree_this->nodes[indexCurr].state.y);
                
                indexCurr = indexParent;
                indexParent = -1;
                if (indexParent == minDistFromRootIndex)
                    break;
                nodeCount++;
                if (nodeCount >= tree_this->num_nodes)
                    break;
            }
            
        }

        fclose (f_optpath);

    }

    return 1;

}


static void
on_save_button(GtkWidget *button, ot_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    

    FILE *f_env = fopen ("env.txt", "wt");
    
    // Print the operating region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_operating.center[0], self->region_operating.center[1],
             self->region_operating.size[0], self->region_operating.size[1]);

    // Print the goal region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_goal.center[0], self->region_goal.center[1],
             self->region_goal.size[0], self->region_goal.size[1]);
    
    // Print the obstacles
    GSList *obstacle_list_ptr = self->list_region_obstacles;
    while (obstacle_list_ptr) {

        region_2d_t *obstacle_this = (region_2d_t *) (obstacle_list_ptr->data);
        
        fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                 obstacle_this->size[0], obstacle_this->size[1]);

        obstacle_list_ptr = g_slist_next (obstacle_list_ptr);
    }

    fclose (f_env);


    // Save the tree
    save_scene_from_history (self, (self->index_trees - self->slider_history_no)%MAX_HISTORY, "");
    

    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_save_history_button(GtkWidget *button, ot_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    

    FILE *f_env = fopen ("env.txt", "wt");
    
    // Print the operating region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_operating.center[0], self->region_operating.center[1],
             self->region_operating.size[0], self->region_operating.size[1]);

    // Print the goal region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_goal.center[0], self->region_goal.center[1],
             self->region_goal.size[0], self->region_goal.size[1]);
    
    // Print the obstacles
    GSList *obstacle_list_ptr = self->list_region_obstacles;
    while (obstacle_list_ptr) {

        region_2d_t *obstacle_this = (region_2d_t *) (obstacle_list_ptr->data);
        
        fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                 obstacle_this->size[0], obstacle_this->size[1]);

        obstacle_list_ptr = g_slist_next (obstacle_list_ptr);
    }

    fclose (f_env);


    for (int i = 0; i < self->num_trees; i++) {
        // Save the tree
        char name[10];
        sprintf (name, "%d", i);
        save_scene_from_history (self, i, name);
    }


    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}





static void
on_save_env_button(GtkWidget *button, ot_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    

    FILE *f_env = fopen ("env1.txt", "wt");
    
    // Print the operating region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_operating.center[0], self->region_operating.center[1],
             self->region_operating.size[0], self->region_operating.size[1]);

    // Print the goal region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_goal.center[0], self->region_goal.center[1],
             self->region_goal.size[0], self->region_goal.size[1]);
    
    // Print the obstacles
    GSList *obstacle_list_ptr = self->list_region_obstacles;
    while (obstacle_list_ptr) {

        region_2d_t *obstacle_this = (region_2d_t *) (obstacle_list_ptr->data);
        
        fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                 obstacle_this->size[0], obstacle_this->size[1]);

        obstacle_list_ptr = g_slist_next (obstacle_list_ptr);
    }

    fclose (f_env);

    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}

static void
on_load_env_button(GtkWidget *button, ot_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    
    FILE *f_env = fopen ("env1.txt", "r");
    
    region_2d_t region_operating;
    region_2d_t region_goal;
    GSList *obstacle_list_ptr = NULL;


    // Read the operating, goal, and obstacle regions from the file
    fscanf (f_env, "%lf, %lf, %lf, %lf\n", &(region_operating.center[0]), &(region_operating.center[1]),
            &(region_operating.size[0]), &(region_operating.size[1]));

    fscanf (f_env, "%lf, %lf, %lf, %lf\n", &(region_goal.center[0]), &(region_goal.center[1]),
            &(region_goal.size[0]), &(region_goal.size[1]) );

    while (!feof (f_env)) {
        region_2d_t *obstacle_this = (region_2d_t *) malloc (sizeof(region_2d_t));
        
        fscanf (f_env, "%lf, %lf, %lf, %lf\n", &(obstacle_this->center[0]), &(obstacle_this->center[1]),
                &(obstacle_this->size[0]), &(obstacle_this->size[1]));
        
        obstacle_list_ptr = g_slist_prepend (obstacle_list_ptr, obstacle_this);

    }


    // Print the operating, goal, and obstacle regions to the screen
    printf ("operating region : %3.2lf, %3.2lf, %3.2lf, %3.2lf\n", region_operating.center[0], region_operating.center[1], 
            region_operating.size[0], region_operating.size[1]);
    printf ("goal region      : %3.2lf, %3.2lf, %3.2lf, %3.2lf\n", region_goal.center[0], region_goal.center[1], 
            region_goal.size[0], region_goal.size[1]);

    printf ("Obstacles        :\n");
    GSList *obstacle_list_this_ptr = obstacle_list_ptr;
    while (obstacle_list_this_ptr) {
        region_2d_t *obstacle_this = (region_2d_t *) (obstacle_list_this_ptr->data);
        printf ("                 : %3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                obstacle_this->size[0], obstacle_this->size[1]);
        obstacle_list_this_ptr = g_slist_next (obstacle_list_this_ptr);
    }


    // Free the current obstacle list
    GSList *region_obstacles_ptr = self->list_region_obstacles;
    while (region_obstacles_ptr) {
        region_2d_t *obstacle_this = (region_2d_t *) (region_obstacles_ptr->data);
        free (obstacle_this);
        region_obstacles_ptr = g_slist_next (region_obstacles_ptr);
    }
    g_slist_free (self->list_region_obstacles);
    self->list_region_obstacles = NULL;

    // Modify the operating region, goal region, and the obstacle region accordingly
    self->region_operating = region_operating;
    self->region_goal = region_goal;
    obstacle_list_this_ptr = obstacle_list_ptr;
    while (obstacle_list_this_ptr) {
        region_2d_t *obstacle_this = (region_2d_t *) (obstacle_list_this_ptr->data);
        self->list_region_obstacles = g_slist_prepend (self->list_region_obstacles, obstacle_this);
        obstacle_list_this_ptr = g_slist_next (obstacle_list_this_ptr);
    }
    

    // Free the temporary memory
    g_slist_free (obstacle_list_ptr);

    fclose (f_env);

    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}




static void
on_opttree_tree (const lcm_recv_buf_t *buf, const char *channel,
                 const lcmtypes_opttree_tree_t *msg, void *user) {
    ot_renderer_t *self = (ot_renderer_t *)user;

//     printf ("Got tree message \n");
//     printf ("Num edges : %d \n", msg->num_edges);

    printf ("%3.5lf,\n", self->cost_last);

    g_mutex_lock (self->mutex);


    if (self->trees[self->index_trees])
        lcmtypes_opttree_tree_t_destroy (self->trees[self->index_trees]);
    self->trees[self->index_trees] = lcmtypes_opttree_tree_t_copy (msg);
    self->index_trees++;
    if (self->index_trees >= MAX_HISTORY)
        self->index_trees = 0;
    
    self->num_trees++;
    if (self->num_trees >= MAX_HISTORY)
        self->num_trees = MAX_HISTORY;

    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_HISTORY, 1);

    bot_gtk_param_widget_modify_int (self->pw, SLIDER_HISTORY, 
                                     1, self->num_trees,
                                     1, self->slider_history_no);

    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


void
ot_renderer_destroy (Renderer *renderer) {

    return;
}


static int 
mouse_press (Viewer *viewer, EventHandler *ehandler,
             const double ray_start[3], const double ray_dir[3], 
             const GdkEventButton *event)
{

    ot_renderer_t *self = (ot_renderer_t *)ehandler->user;

    double xy[2];
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), POINT3D(ray_dir), 
            0, POINT2D(xy));

    int control = event->state & GDK_CONTROL_MASK;
    int shift = event->state & GDK_SHIFT_MASK;

    self->last_xy[0] = xy[0];
    self->last_xy[1] = xy[1];

    self->last_region_goal.center[0] = self->region_goal.center[0];
    self->last_region_goal.center[1] = self->region_goal.center[1];
    self->last_region_goal.size[0] = self->region_goal.size[0];
    self->last_region_goal.size[1] = self->region_goal.size[1];

    self->last_region_operating.center[0] = self->region_operating.center[0];
    self->last_region_operating.center[1] = self->region_operating.center[1];
    self->last_region_operating.size[0] = self->region_operating.size[0];
    self->last_region_operating.size[1] = self->region_operating.size[1];

    if (self->hover_obstacle_region) {
        self->last_region_obstacles.center[0] = self->hover_obstacle_region->center[0];
        self->last_region_obstacles.center[1] = self->hover_obstacle_region->center[1];
        self->last_region_obstacles.size[0] = self->hover_obstacle_region->size[0];
        self->last_region_obstacles.size[1] = self->hover_obstacle_region->size[1];
    }

    if (self->key_obstacles && control) {
        if (!(self->hover_obstacle_region)) {
            region_2d_t *region = (region_2d_t *) malloc (sizeof(region_2d_t));
            region->center[0] = xy[0];
            region->center[1] = xy[1];
            region->size[0] = 1.0;
            region->size[1] = 1.0;
            self->list_region_obstacles = g_slist_prepend (self->list_region_obstacles, region);
        }
    }

    // only handle control key pressed.
    if (!control)
        return 0;

    if (self->key_operating || self->key_goal || self->key_obstacles)
        return 0;

    self->mouseNodeXY[0] = xy[0];
    self->mouseNodeXY[1] = xy[1];

    viewer_request_redraw(self->viewer);

    return 1;
}


static int mouse_motion (Viewer *viewer, EventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], const GdkEventMotion *event)
{
    ot_renderer_t *self = (ot_renderer_t*) ehandler->user;

    double xy[2];
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), POINT3D(ray_dir), 
            0, POINT2D(xy));

    int control = event->state & GDK_CONTROL_MASK;
    int shift = event->state & GDK_SHIFT_MASK;

    if (!(self->key_obstacles))
        self->hover_obstacle_region = NULL;

    if ( (self->key_obstacles) && (!control) && (!shift) ){
        int ll = 0;
        int ll2 = 0;
        double minDist = DBL_MAX;
        region_2d_t *minDistRegion = NULL;
        GSList *list_obstacle_curr = self->list_region_obstacles;
        while (list_obstacle_curr) {
            ll++;
            region_2d_t *region_obstacle_curr = (region_2d_t *) list_obstacle_curr->data; 
            if ( (fabs(region_obstacle_curr->center[0] - xy[0]) <= region_obstacle_curr->size[0]/2.0) &&
                 (fabs(region_obstacle_curr->center[1] - xy[1]) <= region_obstacle_curr->size[1]/2.0) ) {
                ll2++;
                double dist_curr = (region_obstacle_curr->center[0] - xy[0])*(region_obstacle_curr->center[0] - xy[0]) +
                    (region_obstacle_curr->center[1] - xy[1])*(region_obstacle_curr->center[1] - xy[1]);
                if (dist_curr <= minDist) {
                    minDist = dist_curr;
                    minDistRegion = region_obstacle_curr;
                }
            }
            list_obstacle_curr = g_slist_next (list_obstacle_curr);
        }
        self->hover_obstacle_region = minDistRegion;
    }

    if (!(event->state & GDK_BUTTON1_MASK))
        return 0;

    double xy_diff[2] = {
        xy[0] - self->last_xy[0],
        xy[1] - self->last_xy[1]
    };

    if (self->key_goal) {
        if (control) {
            self->region_goal.center[0] = self->last_region_goal.center[0] + xy_diff[0];
            self->region_goal.center[1] = self->last_region_goal.center[1] + xy_diff[1];
            viewer_request_redraw(self->viewer);
        }         
        if (shift) {
            self->region_goal.size[0] = self->last_region_goal.size[0] + xy_diff[0];
            self->region_goal.size[1] = self->last_region_goal.size[1] + xy_diff[1];
            if (self->region_goal.size[0] < 0.0)
                self->region_goal.size[0] = 0.0;
            if (self->region_goal.size[1] < 0.0)
                self->region_goal.size[1] = 0.0;
            viewer_request_redraw(self->viewer);
        }
        return 1;
    }

    if (self->key_operating) {
        if (control) {
            self->region_operating.center[0] = self->last_region_operating.center[0] + xy_diff[0];
            self->region_operating.center[1] = self->last_region_operating.center[1] + xy_diff[1];
            viewer_request_redraw(self->viewer);
        }         
        if (shift) {
            self->region_operating.size[0] = self->last_region_operating.size[0] + xy_diff[0];
            self->region_operating.size[1] = self->last_region_operating.size[1] + xy_diff[1];
            if (self->region_operating.size[0] < 0.0)
                self->region_operating.size[0] = 0.0;
            if (self->region_operating.size[1] < 0.0)
                self->region_operating.size[1] = 0.0;
            viewer_request_redraw(self->viewer);
        }
        return 1;
    }

    if ((self->key_obstacles) && (self->hover_obstacle_region)) {
        if (control) {
            self->hover_obstacle_region->center[0] = self->last_region_obstacles.center[0] + xy_diff[0];
            self->hover_obstacle_region->center[1] = self->last_region_obstacles.center[1] + xy_diff[1];
            viewer_request_redraw(self->viewer);
        }
        if (shift) {
            self->hover_obstacle_region->size[0] = self->last_region_obstacles.size[0] + xy_diff[0];
            self->hover_obstacle_region->size[1] = self->last_region_obstacles.size[1] + xy_diff[1];
            if (self->hover_obstacle_region->size[0] < 0.1)
                self->hover_obstacle_region->size[0] = 0.1;
            if (self->hover_obstacle_region->size[1] < 0.1)
                self->hover_obstacle_region->size[1] = 0.1;
            viewer_request_redraw(self->viewer);
        }
        return 1;
    }

    return 0;

}

static int key_press (Viewer *viewer, EventHandler *ehandler, const GdkEventKey *event)
{
    ot_renderer_t *self = (ot_renderer_t*) ehandler->user;

    if (event->keyval == 'o' || event->keyval == 'O') {
        viewer_request_pick(viewer, ehandler);
        self->key_obstacles = 1;
        self->key_goal = 0;
        self->key_operating = 0;
        return 1;
    }

    if (event->keyval == 'g' || event->keyval == 'G') {
        viewer_request_pick(viewer, ehandler);
        self->key_goal = 1;
        self->key_obstacles = 0;
        self->key_operating = 0;
        return 1;
    }

    if (event->keyval == 'b' || event->keyval == 'B') {
        viewer_request_pick(viewer, ehandler);
        self->key_operating = 1;
        self->key_obstacles = 0;
        self->key_goal = 0;
        return 1;
    }


    if ((event->keyval == GDK_Delete || 
                           event->keyval == GDK_BackSpace)) {

        if (self->key_obstacles) {
            if (self->hover_obstacle_region) {
                self->list_region_obstacles = g_slist_remove (self->list_region_obstacles, self->hover_obstacle_region);
                free (self->hover_obstacle_region);
                self->hover_obstacle_region = NULL;
            }
        }


        viewer_request_redraw(viewer);
        return 1;
    }


    if (event->keyval == GDK_Escape) {
        ehandler->picking = 0;
        self->key_obstacles = 0;
        self->key_goal = 0;
        self->key_operating = 0;
        return 1;
    }

    return 0;
}


void
ot_renderer_draw (Viewer *viewer, Renderer *renderer) {

    ot_renderer_t *self = (ot_renderer_t *)renderer->user;

    glColor3f (1.0, 1.0, 1.0);
    glLineWidth (5.0);
    glBegin (GL_LINE_STRIP);
    glVertex3f (0.0, 0.0, 1.0);
    glVertex3f (0.0, 0.0, 0.0);
    glEnd();


    // Draw the operating region
    glColor3f (1.0, 0.1, 0.1);
    glLineWidth (5.0);
    glBegin (GL_LINE_LOOP);
    glVertex3f (self->region_operating.center[0] - self->region_operating.size[0]/2.0, 
                self->region_operating.center[1] - self->region_operating.size[1]/2.0, 0.0);
    glVertex3f (self->region_operating.center[0] + self->region_operating.size[0]/2.0, 
                self->region_operating.center[1] - self->region_operating.size[1]/2.0, 0.0);
    glVertex3f (self->region_operating.center[0] + self->region_operating.size[0]/2.0, 
                self->region_operating.center[1] + self->region_operating.size[1]/2.0, 0.0);
    glVertex3f (self->region_operating.center[0] - self->region_operating.size[0]/2.0, 
                self->region_operating.center[1] + self->region_operating.size[1]/2.0, 0.0);
    glEnd ();


    // Draw the goal region
    glColor3f (0.1, 1.0, 0.1);
    glLineWidth (5.0);
    glBegin (GL_LINE_LOOP);
    glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] - self->region_goal.size[1]/2.0, 0.0);
    glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] - self->region_goal.size[1]/2.0, 0.0);
    glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] + self->region_goal.size[1]/2.0, 0.0);
    glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] + self->region_goal.size[1]/2.0, 0.0);
    glEnd ();


    // Draw the obstacles
    GSList *list_obstacle_curr = self->list_region_obstacles;
    while (list_obstacle_curr) {
        region_2d_t *region_obstacle_curr = (region_2d_t *) list_obstacle_curr->data;

        if (region_obstacle_curr == self->hover_obstacle_region)
            glColor3f (1.0, 0.5, 0.5);
        else
            glColor3f (1.0, 0.1, 0.1);
        glLineWidth (5.0);
        glBegin (GL_QUADS);
        glVertex3f (region_obstacle_curr->center[0] - region_obstacle_curr->size[0]/2.0, 
                    region_obstacle_curr->center[1] - region_obstacle_curr->size[1]/2.0, 0.0);
        glVertex3f (region_obstacle_curr->center[0] + region_obstacle_curr->size[0]/2.0, 
                    region_obstacle_curr->center[1] - region_obstacle_curr->size[1]/2.0, 0.0);
        glVertex3f (region_obstacle_curr->center[0] + region_obstacle_curr->size[0]/2.0, 
                    region_obstacle_curr->center[1] + region_obstacle_curr->size[1]/2.0, 0.0);
        glVertex3f (region_obstacle_curr->center[0] - region_obstacle_curr->size[0]/2.0, 
                    region_obstacle_curr->center[1] + region_obstacle_curr->size[1]/2.0, 0.0);
        glEnd ();
        
        list_obstacle_curr = g_slist_next (list_obstacle_curr);
    }


    if (self->num_trees == 0) {
        glFlush ();
        return;
    }

    g_mutex_lock (self->mutex);

    // Draw all the nodes    
    lcmtypes_opttree_tree_t *tree = self->trees[(self->index_trees - self->slider_history_no)%MAX_HISTORY];
    glPointSize (1.0);
    glColor3f (0.3,1.0,0.3);
    glBegin (GL_POINTS);    
    for (int i = 0; i < tree->num_nodes; i++) {
        glVertex3f (tree->nodes[i].state.x, tree->nodes[i].state.y, 0.0);
    }
    glEnd ();

    // Draw the edges
    glLineWidth (1.0);
    glColor3f (1.0,0.3,0.3);
    for (int i = 0; i < tree->num_edges; i++) {
        glBegin (GL_LINE_STRIP);
        glVertex3f (tree->nodes[tree->edges[i][0]].state.x, tree->nodes[tree->edges[i][0]].state.y, 0.0);
        glVertex3f (tree->nodes[tree->edges[i][1]].state.x, tree->nodes[tree->edges[i][1]].state.y, 0.0);
        glEnd();
    }

    if (tree->num_nodes <= 0) {
        g_mutex_unlock (self->mutex);
        glFlush ();
        return;
    }

    
    // Draw the clicked node
    double minDistSq = DBL_MAX;
    int minIndex = -1;
    for (int i = 0; i < tree->num_nodes; i++) {
        double distX = tree->nodes[i].state.x - self->mouseNodeXY[0];
        double distY = tree->nodes[i].state.y - self->mouseNodeXY[1];
        double distCurrSq = distX * distX + distY * distY; 
        if (distCurrSq < minDistSq) {
                minDistSq = distCurrSq;
                minIndex = i;
        }
    }
    
    int nodeCount = 0;
    if (minIndex >= 0) {
        // Draw the path from this node back.
        int indexCurr = minIndex;
        int indexParent = -1;
        while (1) {
            for (int i = 0; i < tree->num_edges; i++) {         // Find the parent of the current index
                if (tree->edges[i][0] == indexCurr){
                    indexParent = tree->edges[i][1];
                    break;
                }
            }
            if (indexParent >= 0) {
                glColor3f(1.0, 1.0, 1.0);
                glLineWidth (3.0);
                glBegin (GL_LINE_STRIP);
                glVertex3f (tree->nodes[indexCurr].state.x, tree->nodes[indexCurr].state.y, 0.0);
                glVertex3f (tree->nodes[indexParent].state.x, tree->nodes[indexParent].state.y, 0.0);
                glEnd ();
            }
            indexCurr = indexParent;
            indexParent = -1;
            if ( (indexParent == 0) || (indexParent == minIndex) )
                break;
            nodeCount++;
            if (nodeCount >= tree->num_nodes)
                break;
        }
        char output[80];
        sprintf (output, "Dist: %5.2lf", tree->nodes[minIndex].distance_from_root);
        
        double xyz[3] = {
            tree->nodes[minIndex].state.x,
            tree->nodes[minIndex].state.y,
            0.8
        };

        glColor3f (0.8, 0.8, 0.8);
        bot_gl_draw_text (xyz, GLUT_BITMAP_HELVETICA_12, output, 0);
    }

    int minDistFromRootIndex = -1;
    double minDistFromRoot = DBL_MAX;
    for (int i = 0; i < tree->num_nodes; i++) {
        if ( (tree->nodes[i].state.x >= self->region_goal.center[0] - self->region_goal.size[0]/2.0) && 
             (tree->nodes[i].state.x <= self->region_goal.center[0] + self->region_goal.size[0]/2.0) &&
             (tree->nodes[i].state.y >= self->region_goal.center[1] - self->region_goal.size[1]/2.0) && 
             (tree->nodes[i].state.y <= self->region_goal.center[1] + self->region_goal.size[1]/2.0) ) {
            
            // Calculate the distance from the root. 
            if (tree->nodes[i].distance_from_root < minDistFromRoot) {
                minDistFromRoot = tree->nodes[i].distance_from_root;
                minDistFromRootIndex = i;
            }
        }
    }

    nodeCount = 0;
    if (minDistFromRootIndex != -1) {
        // Draw the path from this node back.
        int indexCurr = minDistFromRootIndex;
        int indexParent = -1;
        while (1) {
            for (int i = 0; i < tree->num_edges; i++) {         // Find the parent of the current index
                if (tree->edges[i][0] == indexCurr){
                    indexParent = tree->edges[i][1];
                    break;
                }
            }
            if (indexParent >= 0) {
                glColor3f(1.0, 1.0, 0.2);
                glLineWidth (3.0);
                glBegin (GL_LINE_STRIP);
                glVertex3f (tree->nodes[indexCurr].state.x, tree->nodes[indexCurr].state.y, 0.0);
                glVertex3f (tree->nodes[indexParent].state.x, tree->nodes[indexParent].state.y, 0.0);
                glEnd ();
            }
            indexCurr = indexParent;
            indexParent = -1;
            if (indexParent == minDistFromRootIndex)
                break;
            nodeCount++;
            if (nodeCount >= tree->num_nodes)
                break;
        }
        char output[80];
        sprintf (output, "Dist: %5.2lf", tree->nodes[minDistFromRootIndex].distance_from_root);
        
        self->cost_last = tree->nodes[minDistFromRootIndex].distance_from_root;

        double xyz[3] = {
            tree->nodes[minDistFromRootIndex].state.x,
            tree->nodes[minDistFromRootIndex].state.y,
            0.8
        };

        glColor3f (0.8, 0.8, 0.8);
        bot_gl_draw_text (xyz, GLUT_BITMAP_HELVETICA_12, output, 0);
    }

    glFlush ();
    
    g_mutex_unlock (self->mutex);
}


void 
setup_renderer_tree (Viewer *viewer, int priority) {

    ot_renderer_t *self = (ot_renderer_t *) malloc (sizeof (ot_renderer_t));

    self->num_trees = 0;

    self->index_trees = 0;
    for (int i = 0; i < MAX_HISTORY; i++)
        self->trees[i] = NULL;

    self->mutex = g_mutex_new ();

    self->mouseNodeXY[0] = 0.0;
    self->mouseNodeXY[1] = 0.0;

    Renderer *renderer = &(self->renderer);

    renderer->draw = ot_renderer_draw;
    renderer->destroy = ot_renderer_destroy;
    renderer->name = RENDERER_NAME;
    renderer->widget = gtk_vbox_new (FALSE, 0);
    renderer->enabled = 1;
    renderer->user = self;
    
    EventHandler *ehandler = &(self->ehandler);
    
    ehandler->name = RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_motion = mouse_motion;
    ehandler->mouse_release = NULL;
    ehandler->pick_query = NULL;
    ehandler->hover_query = NULL;
    ehandler->key_press = key_press;
    ehandler->user = self;

    self->lcm = globals_get_lcm ();
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new());

    self->key_obstacles = 0;
    self->list_region_obstacles = NULL;
    self->hover_obstacle_region = NULL;
    self->last_region_obstacles.center[0] = 0.0;
    self->last_region_obstacles.center[1] = 0.0;
    self->last_region_obstacles.size[0] = 0.0;
    self->last_region_obstacles.size[1] = 0.0;

    self->key_operating = 0;
    self->region_operating.center[0] = 0.0;
    self->region_operating.center[1] = 0.0;
    self->region_operating.size[0] = 20.0;
    self->region_operating.size[1] = 20.0;
    self->last_region_operating.center[0] = self->region_operating.center[0];
    self->last_region_operating.center[1] = self->region_operating.center[1];
    self->last_region_operating.size[0] = self->region_operating.size[0];
    self->last_region_operating.size[1] = self->region_operating.size[1];

    self->key_goal = 0;
    self->region_goal.center[0] = 8.0;
    self->region_goal.center[1] = 8.0;
    self->region_goal.size[0] = 3.0;
    self->region_goal.size[1] = 3.0;
    self->last_region_goal.center[0] = self->region_goal.center[0];
    self->last_region_goal.center[1] = self->region_goal.center[1];
    self->last_region_goal.size[0] = self->region_goal.size[0];
    self->last_region_goal.size[1] = self->region_goal.size[1];

    self->cost_last = 0;

    // Add the history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, SLIDER_HISTORY, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, MAX_HISTORY, 1, 1);
    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_HISTORY, 0);
    self->slider_history_no = 1;

    // Add goal traj check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_RENDER_GOAL_TRAJ, 0, NULL);
    self->render_goal_traj = FALSE;
    

    // Add the number of iterations checkbox
    bot_gtk_param_widget_add_int (self->pw, SLIDER_NUM_ITERATIONS, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 1000, 1, 100);
    self->max_num_iterations = 100000;

    // Add the use_const_nodes button
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_USE_CONST_NODES, 0, NULL);
    self->use_const_nodes = FALSE;
    
    // Add the number of constant nodes
    bot_gtk_param_widget_add_int (self->pw, SLIDER_NUM_CONST_NODES, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 20000, 1, 5000);
    self->num_const_nodes = 5000;


    // Add the message publish interval length 
    bot_gtk_param_widget_add_int (self->pw, SLIDER_MSG_PUB_INTERVAL, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 1000, 1, 500);
    self->msg_pub_interval = 500;


    // Add the RRT/OptRRT button
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_RUN_OPTRRT, 1, NULL);
    
    self->run_optrrt = TRUE;


    // Add the clear history button
    GtkWidget *clear_button = gtk_button_new_with_label ("Clear History");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked",
                     G_CALLBACK(on_clear_button), self);

    // Add the star  button
    GtkWidget *start_button = gtk_button_new_with_label ("Start RRT");
    gtk_box_pack_start(GTK_BOX(renderer->widget), start_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(start_button), "clicked",
                     G_CALLBACK(on_start_button), self);

    // Add the save scene button
    GtkWidget *save_button = gtk_button_new_with_label ("Save Scene");
    gtk_box_pack_start(GTK_BOX(renderer->widget), save_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(save_button), "clicked",
                     G_CALLBACK(on_save_button), self);


    // Add the save history button
    GtkWidget *save_history_button = gtk_button_new_with_label ("Save History");
    gtk_box_pack_start(GTK_BOX(renderer->widget), save_history_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(save_history_button), "clicked",
                     G_CALLBACK(on_save_history_button), self);


    // Add the save environment button
    GtkWidget *save_env_button = gtk_button_new_with_label ("Save Environment");
    gtk_box_pack_start(GTK_BOX(renderer->widget), save_env_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(save_env_button), "clicked",
                     G_CALLBACK(on_save_env_button), self);


    // Add the load environment button
    GtkWidget *load_env_button = gtk_button_new_with_label ("Load Environment");
    gtk_box_pack_start(GTK_BOX(renderer->widget), load_env_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(load_env_button), "clicked",
                     G_CALLBACK(on_load_env_button), self);


    // Show the widgets
    gtk_widget_show_all (renderer->widget);
    g_signal_connect (G_OBJECT (self->pw), "changed", G_CALLBACK (on_param_widget_changed), self);

    // Subscribe to OPTTREE_TREE message
    lcmtypes_opttree_tree_t_subscribe (self->lcm, "OPTTREE_TREE", on_opttree_tree, self);

    viewer_add_renderer (viewer, renderer, priority);
    viewer_add_event_handler(viewer, ehandler, priority);

    return;
}
