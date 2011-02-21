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

#define RENDERER_NAME "SMP"

#define CHECKBOX_SHOW_GRAPHS_VERTICES "Show vertices"
#define CHECKBOX_SHOW_GRAPHS_EDGES "Show edges"
#define GRAPH_SLIDER_HISTORY "Graphs"
#define CHECKBOX_SHOW_TRAJECTORIES "Show trajectories"
#define TRAJECTORY_SLIDER_HISTORY "Trajectories"
#define CHECKBOX_SHOW_ENVIRONMENTS "Show environements"
#define ENVIRONMENT_SLIDER_HISTORY "Environments"

#define SLIDER_OBSTACLE_OPACITY "Obstacle opacity"
#define SLIDER_VERTEX_SIZE "Vertex size"
#define SLIDER_EDGE_THICKNESS "Edge thickness"


#define CHECKBOX_PROJECT "Show projected"

typedef struct _region_2d_t {
    double center[2];
    double size[2];
} region_2d_t;

typedef struct _region_3d_t {
    double center[3];
    double size[3];
} region_3d_t;


typedef struct _smp_renderer_t {

    Renderer renderer;    
    EventHandler ehandler;
    lcm_t *lcm; 
    Viewer *viewer;    
    BotGtkParamWidget *pw;
    
    int num_graphs;
    GList *list_graphs;
    lcmtypes_smp_graph_t *graph_curr;
    gboolean show_graphs_vertices;
    gboolean show_graphs_edges;

    int num_trajectories;
    GList *list_trajectories;
    lcmtypes_smp_trajectory_t *trajectory_curr;
    gboolean show_trajectories;

    int num_environments;
    GList *list_environments;
    lcmtypes_smp_environment_t *environment_curr;
    gboolean show_environments;

    lcmtypes_smp_environment_t environment_default;

    int graph_slider_history_no;
    int trajectory_slider_history_no;
    int environment_slider_history_no;

    gboolean show_projected;

    GMutex *mutex;

    // Parameters used to draw the clicked path
    double mouseNodeXY[2];
    int minIndex;
    
    // Operating region
    int key_operating;
    region_3d_t region_operating;
    region_3d_t last_region_operating;
    
    // Goal region
    int key_goal;
    region_3d_t region_goal;
    region_3d_t last_region_goal;
    
    // Obstacles
    int key_obstacles;
    int key_obstacles_height;
    GSList *list_region_obstacles;
    region_3d_t *hover_obstacle_region; 
    region_3d_t last_region_obstacles;
    
    double last_xy[2];   // Used moving and resizing operating, goal, and obstacle regions

    // Rendering parameters
    int obstacle_opacity;
    int vertex_size;
    int edge_thickness;
    
} smp_renderer_t;


void 
smp_renderer_draw (Viewer *viewer, Renderer *renderer);

void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *name,
                         smp_renderer_t *self) {

    g_mutex_lock (self->mutex);
    
    self->graph_slider_history_no = bot_gtk_param_widget_get_int (self->pw, GRAPH_SLIDER_HISTORY);

    self->obstacle_opacity = bot_gtk_param_widget_get_int (self->pw, SLIDER_OBSTACLE_OPACITY);
    self->vertex_size = bot_gtk_param_widget_get_int (self->pw, SLIDER_VERTEX_SIZE);
    self->edge_thickness = bot_gtk_param_widget_get_int (self->pw, SLIDER_EDGE_THICKNESS);
    
    int graph_index = self->graph_slider_history_no - 1;
    if (graph_index >= 0 && graph_index < self->num_graphs) {
        self->graph_curr = (lcmtypes_smp_graph_t*) g_list_nth_data (self->list_graphs, graph_index);
    }
    
    self->show_projected = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_PROJECT);

    self->show_graphs_vertices = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_SHOW_GRAPHS_VERTICES);

    self->show_graphs_edges = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_SHOW_GRAPHS_EDGES);

    self->show_trajectories = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_SHOW_TRAJECTORIES);

    self->show_environments = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_SHOW_ENVIRONMENTS);

    viewer_request_redraw(self->viewer);    

    g_mutex_unlock (self->mutex);

}

static void
on_clear_graph_button(GtkWidget *button, smp_renderer_t *self)
{

    g_mutex_lock (self->mutex);

    GList *graph_ptr = self->list_graphs;
    while (graph_ptr) {
        lcmtypes_smp_graph_t *graph = graph_ptr->data;
        lcmtypes_smp_graph_t_destroy (graph);
        graph_ptr = g_list_next(graph_ptr);
    }
    g_list_free (self->list_graphs);
    self->list_graphs = NULL;
    self->num_graphs = 0;
    self->graph_curr = NULL;

    self->graph_slider_history_no = 1;

    bot_gtk_param_widget_set_enabled (self->pw, GRAPH_SLIDER_HISTORY, 0);
    

    bot_gtk_param_widget_modify_int (self->pw, GRAPH_SLIDER_HISTORY, 
                                     1, 2,
                                     1, self->graph_slider_history_no);

    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_clear_trajectory_button(GtkWidget *button, smp_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    
    GList *trajectory_ptr = self->list_trajectories;
    while (trajectory_ptr) {
        lcmtypes_smp_trajectory_t *trajectory = trajectory_ptr->data;
        lcmtypes_smp_trajectory_t_destroy (trajectory);
        trajectory_ptr = g_list_next(trajectory_ptr);
    }
    g_list_free (self->list_trajectories);
    self->list_trajectories = NULL;
    self->num_trajectories = 0;
    self->trajectory_curr = NULL;

    self->trajectory_slider_history_no = 1;

    bot_gtk_param_widget_set_enabled (self->pw, TRAJECTORY_SLIDER_HISTORY, 0);

    bot_gtk_param_widget_modify_int (self->pw, TRAJECTORY_SLIDER_HISTORY, 
                                     1, 2,
                                     1, self->trajectory_slider_history_no);


    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_clear_environment_button(GtkWidget *button, smp_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    
    GList *environment_ptr = self->list_environments;
    while (environment_ptr) {
        lcmtypes_smp_environment_t *environment = environment_ptr->data;
        lcmtypes_smp_environment_t_destroy (environment);
        environment_ptr = g_list_next(environment_ptr);
    }
    g_list_free (self->list_environments);
    self->list_environments = NULL;
    

    self->environment_curr = lcmtypes_smp_environment_t_copy (&(self->environment_default));
    self->list_environments = g_list_prepend (self->list_environments, self->environment_curr);
    self->num_environments = 1;


    self->environment_slider_history_no = 1;

    bot_gtk_param_widget_set_enabled (self->pw, ENVIRONMENT_SLIDER_HISTORY, 0);

    bot_gtk_param_widget_modify_int (self->pw, ENVIRONMENT_SLIDER_HISTORY, 
                                     1, 2,
                                     1, self->environment_slider_history_no);


    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_clear_button(GtkWidget *button, smp_renderer_t *self)
{

    on_clear_graph_button (NULL, self);

    on_clear_trajectory_button (NULL, self);

    on_clear_environment_button (NULL, self);

}


static void
on_start_new_environment_button(GtkWidget *button, smp_renderer_t *self)
{
    // TODO: fill in this part.

    // lcmtypes_smp_environment_t *environment_new = lcmtypes_smp_environment_t_copy (self->environment_curr);
    // self->list_environments = g_list_prepend (self->list_environments, environment_new);
    // self->num_environments++;

}



int save_graph_to_file (smp_renderer_t *self, lcmtypes_smp_graph_t *graph, char name_ext[10]) {

    if (!graph)
        return 0;
    
    // Print vertices
    char vertices_name[80] = ""; 
    strcat (vertices_name, "nodes");
    strcat (vertices_name, name_ext);
    strcat (vertices_name, ".txt");
    FILE *f_vertices = fopen (vertices_name, "wt");
    
    for (int i = 0; i < graph->num_vertices; i++) {
        lcmtypes_smp_vertex_t *vertex_curr = &(graph->vertices[i]);
        fprintf (f_vertices, "%d, %3.2lf, %3.2lf, %3.2lf\n", i, 
                 vertex_curr->state.x, vertex_curr->state.y, vertex_curr->state.z);
    }
    
    fclose (f_vertices);

    return 1;
}


static void
on_save_button(GtkWidget *button, smp_renderer_t *self)
{

    g_mutex_lock (self->mutex);

    FILE *f_env = fopen ("env_last_history.txt", "wt");
    
    // Print the operating region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_operating.center[0], self->region_operating.center[1],
             self->region_operating.size[0], self->region_operating.size[1]);

    // Print the goal region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_goal.center[0], self->region_goal.center[1],
             self->region_goal.size[0], self->region_goal.size[1]);
    
    // Print the obstacles
    GSList *obstacle_list_ptr = self->list_region_obstacles;
    while (obstacle_list_ptr) {

        region_3d_t *obstacle_this = (region_3d_t *) (obstacle_list_ptr->data);
        
        fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                 obstacle_this->size[0], obstacle_this->size[1]);

        obstacle_list_ptr = g_slist_next (obstacle_list_ptr);
    }

    fclose (f_env);

    // Save the graph
    save_graph_to_file (self, self->graph_curr, "");
    
    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_save_history_button(GtkWidget *button, smp_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    

    FILE *f_env = fopen ("env_last_history.txt", "wt");
    
    // Print the operating region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_operating.center[0], self->region_operating.center[1],
             self->region_operating.size[0], self->region_operating.size[1]);

    // Print the goal region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_goal.center[0], self->region_goal.center[1],
             self->region_goal.size[0], self->region_goal.size[1]);
    
    // Print the obstacles
    GSList *obstacle_list_ptr = self->list_region_obstacles;
    while (obstacle_list_ptr) {

        region_3d_t *obstacle_this = (region_3d_t *) (obstacle_list_ptr->data);
        
        fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                 obstacle_this->size[0], obstacle_this->size[1]);

        obstacle_list_ptr = g_slist_next (obstacle_list_ptr);
    }

    fclose (f_env);

    // Save all the graphs
    GList *graph_it = self->list_graphs;
    int i = 1;
    while (graph_it) {
        char file_name_ending[10];
        sprintf (file_name_ending, "%d", i++);
        save_graph_to_file (self, graph_it->data, file_name_ending);
        graph_it = g_list_next (graph_it);
    }

    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_save_env_button(GtkWidget *button, smp_renderer_t *self)
{

    g_mutex_lock (self->mutex);
    

    FILE *f_env = fopen ("environment.txt", "wt");
    
    // Print the operating region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_operating.center[0], self->region_operating.center[1],
             self->region_operating.size[0], self->region_operating.size[1]);

    // Print the goal region
    fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", self->region_goal.center[0], self->region_goal.center[1],
             self->region_goal.size[0], self->region_goal.size[1]);
    
    // Print the obstacles
    GSList *obstacle_list_ptr = self->list_region_obstacles;
    while (obstacle_list_ptr) {

        region_3d_t *obstacle_this = (region_3d_t *) (obstacle_list_ptr->data);
        
        fprintf (f_env, "%3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                 obstacle_this->size[0], obstacle_this->size[1]);

        obstacle_list_ptr = g_slist_next (obstacle_list_ptr);
    }

    fclose (f_env);

    g_mutex_unlock (self->mutex);

    viewer_request_redraw(self->viewer);
}


static void
on_load_env_button(GtkWidget *button, smp_renderer_t *self)
{

    g_mutex_lock (self->mutex);

    FILE *f_env = fopen ("env2.txt", "r");
    
    region_3d_t region_operating;
    region_3d_t region_goal;
    GSList *obstacle_list_ptr = NULL;

    fscanf (f_env, "%lf, %lf, %lf, %lf, %lf, %lf\n", 
            &(region_operating.center[0]), &(region_operating.center[1]), &(region_operating.center[2]),
            &(region_operating.size[0]), &(region_operating.size[1]), &(region_operating.size[2]) );


    fscanf (f_env, "%lf, %lf, %lf, %lf, %lf, %lf\n", 
            &(region_goal.center[0]), &(region_goal.center[1]), &(region_goal.center[2]),
            &(region_goal.size[0]), &(region_goal.size[1]), &(region_goal.size[2]) );

    while (!feof (f_env)) {
        printf ("Obstacle... \n");

        region_3d_t *obstacle_this = (region_3d_t *) malloc (sizeof(region_3d_t));

        fscanf (f_env, "%lf, %lf, %lf, %lf, %lf, %lf\n", 
                &(obstacle_this->center[0]), &(obstacle_this->center[1]), &(obstacle_this->center[2]),
                &(obstacle_this->size[0]), &(obstacle_this->size[1]), &(obstacle_this->size[2]) );
        
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
        region_3d_t *obstacle_this = (region_3d_t *) (obstacle_list_this_ptr->data);
        printf ("                 : %3.2lf, %3.2lf, %3.2lf, %3.2lf\n", obstacle_this->center[0], obstacle_this->center[1],
                obstacle_this->size[0], obstacle_this->size[1]);
        obstacle_list_this_ptr = g_slist_next (obstacle_list_this_ptr);
    }


    // Free the current obstacle list
    GSList *region_obstacles_ptr = self->list_region_obstacles;
    while (region_obstacles_ptr) {
        region_3d_t *obstacle_this = (region_3d_t *) (region_obstacles_ptr->data);
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
        region_3d_t *obstacle_this = (region_3d_t *) (obstacle_list_this_ptr->data);
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
on_smp_graph (const lcm_recv_buf_t *buf, const char *channel,
              const lcmtypes_smp_graph_t *msg, void *user) {
    
    smp_renderer_t *self = (smp_renderer_t *) user;
    
    g_mutex_lock (self->mutex);
    
    self->list_graphs = g_list_prepend (self->list_graphs, lcmtypes_smp_graph_t_copy (msg) );
    self->num_graphs++;
    self->graph_curr = self->list_graphs->data;

    if (self->num_graphs > 1) {
        // if (self->graph_slider_history_no > 1) {
            // (self->graph_slider_history_no)++;
        // }
        bot_gtk_param_widget_set_enabled (self->pw, GRAPH_SLIDER_HISTORY, 1);
        bot_gtk_param_widget_modify_int (self->pw, GRAPH_SLIDER_HISTORY, 
                                         1, self->num_graphs,
                                         1, self->graph_slider_history_no);
    }
    
    g_mutex_unlock (self->mutex);

    viewer_request_redraw (self->viewer);
} 


static void
on_smp_trajectory (const lcm_recv_buf_t *buf, const char *channel,
                   const lcmtypes_smp_trajectory_t *msg, void *user) {

    smp_renderer_t *self = (smp_renderer_t *)user;

    g_mutex_lock (self->mutex);
    
    self->list_trajectories = g_list_prepend (self->list_trajectories, lcmtypes_smp_trajectory_t_copy (msg) );
    self->num_trajectories++;


    if (self->num_trajectories > 1) {
        // if (self->trajectory_slider_history_no > 1) {
        //     self->trajectory_slider_history_no++;
        // }
        bot_gtk_param_widget_set_enabled (self->pw, TRAJECTORY_SLIDER_HISTORY, 1);
        bot_gtk_param_widget_modify_int (self->pw, TRAJECTORY_SLIDER_HISTORY, 
                                         1, self->num_trajectories,
                                         1, self->trajectory_slider_history_no);
    }


    g_mutex_unlock (self->mutex);

    viewer_request_redraw (self->viewer);
}



static void 
on_smp_environment (const lcm_recv_buf_t *buf, const char *channel,
                    const lcmtypes_smp_environment_t *msg, void *user) {

    smp_renderer_t *self = (smp_renderer_t *)user;
    
    g_mutex_lock (self->mutex);
        
    // Update the operating region.
    for (int i = 0; i < 3; i++) {
        self->region_operating.center[i] = msg->operating.center[i];
        self->region_operating.size[i] = msg->operating.size[i];
    }
    
    // Update the goal region.
    for (int i = 0; i < 3; i++) {
        self->region_goal.center[i] = msg->goal.center[i];
        self->region_goal.size[i] = msg->goal.size[i];
    }

    //Clear the old obstacles.
    GSList *obstacles_ptr = self->list_region_obstacles;
    while (obstacles_ptr) {
        region_3d_t *obstacle_curr = (region_3d_t *) (obstacles_ptr->data);
        free (obstacle_curr);
        obstacles_ptr = g_slist_next (obstacles_ptr);
    }
    g_slist_free (self->list_region_obstacles);
    self->list_region_obstacles = NULL;
    
    // Add in the new obstacles
    for (int i = 0; i < msg->num_obstacles; i++) {
        region_3d_t *obstacle_curr = malloc (sizeof (region_3d_t));
        for (int j = 0; j < 3; j++) {
            obstacle_curr->center[j] = msg->obstacles[i].center[j];
            obstacle_curr->size[j] = msg->obstacles[i].size[j];
        }
        self->list_region_obstacles = g_slist_prepend (self->list_region_obstacles, obstacle_curr);
    }

    // Add the new environment to the list of environments
    lcmtypes_smp_environment_t *environment_new = lcmtypes_smp_environment_t_copy (msg);
    self->list_environments = g_list_prepend (self->list_environments, environment_new);
    self->num_environments++;

    
    // if (self->environment_slider_history_no > 1) {
    //     self->environment_slider_history_no++;
    // }
    bot_gtk_param_widget_set_enabled (self->pw, ENVIRONMENT_SLIDER_HISTORY, 1);
    bot_gtk_param_widget_modify_int (self->pw, ENVIRONMENT_SLIDER_HISTORY, 
                                     1, self->num_environments,
                                     1, self->environment_slider_history_no);
    
    g_mutex_unlock (self->mutex);

    viewer_request_redraw (self->viewer);

}


void
smp_renderer_destroy (Renderer *renderer) {

    return;
}


static int 
mouse_press (Viewer *viewer, EventHandler *ehandler,
             const double ray_start[3], const double ray_dir[3], 
             const GdkEventButton *event)
{

    smp_renderer_t *self = (smp_renderer_t *)ehandler->user;

    double xy[2];
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), POINT3D(ray_dir), 
            0, POINT2D(xy));

    int control = event->state & GDK_CONTROL_MASK;
    // int shift = event->state & GDK_SHIFT_MASK;

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
        self->last_region_obstacles.center[2] = self->hover_obstacle_region->center[2];
        self->last_region_obstacles.size[0] = self->hover_obstacle_region->size[0];
        self->last_region_obstacles.size[1] = self->hover_obstacle_region->size[1];
        self->last_region_obstacles.size[2] = self->hover_obstacle_region->size[2];
    }

    if (self->key_obstacles && control) {
        if (!(self->hover_obstacle_region)) {
            region_3d_t *region = (region_3d_t *) malloc (sizeof(region_3d_t));
            region->center[0] = xy[0];
            region->center[1] = xy[1];
            region->center[2] = 1.0;
            region->size[0] = 1.0;
            region->size[1] = 1.0;
            region->size[2] = 2.0;
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
    smp_renderer_t *self = (smp_renderer_t*) ehandler->user;

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
        region_3d_t *minDistRegion = NULL;
        GSList *list_obstacle_curr = self->list_region_obstacles;
        while (list_obstacle_curr) {
            ll++;
            region_3d_t *region_obstacle_curr = (region_3d_t *) list_obstacle_curr->data; 
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
        if (self->key_obstacles_height) {
            if (control) {
                self->hover_obstacle_region->center[2] = self->last_region_obstacles.center[2] + xy_diff[0];
            }
            if (shift) {
                self->hover_obstacle_region->size[2] = self->last_region_obstacles.size[2] + xy_diff[0];
                self->hover_obstacle_region->center[2] = self->last_region_obstacles.center[2] + xy_diff[0]/2.0;

            }
            viewer_request_redraw(self->viewer);
            return 1;
        }
        else {
            if (control) {
                self->hover_obstacle_region->center[0] = self->last_region_obstacles.center[0] + xy_diff[0];
                self->hover_obstacle_region->center[1] = self->last_region_obstacles.center[1] + xy_diff[1];
            }
            if (shift) {
                self->hover_obstacle_region->size[0] = self->last_region_obstacles.size[0] + xy_diff[0];
                self->hover_obstacle_region->size[1] = self->last_region_obstacles.size[1] + xy_diff[1];
                if (self->hover_obstacle_region->size[0] < 0.1)
                    self->hover_obstacle_region->size[0] = 0.1;
                if (self->hover_obstacle_region->size[1] < 0.1)
                    self->hover_obstacle_region->size[1] = 0.1;
            }
            viewer_request_redraw(self->viewer);
            return 1;
        }
    }

    return 0;

}

static int key_press (Viewer *viewer, EventHandler *ehandler, const GdkEventKey *event)
{
    smp_renderer_t *self = (smp_renderer_t*) ehandler->user;

    if (event->keyval == 'o' || event->keyval == 'O') {
        viewer_request_pick(viewer, ehandler);
        self->key_obstacles = 1;
        if (event->keyval == 'O') {
            self->key_obstacles_height = 1;
            printf ("Big O\n");
        }
        else { 
            self->key_obstacles_height = 0;
            printf ("Little o\n");
        }
        self->key_goal = 0;
        self->key_operating = 0;
        return 1;
    }

    if (event->keyval == 'g' || event->keyval == 'G') {
        viewer_request_pick(viewer, ehandler);
        self->key_goal = 1;
        self->key_obstacles = 0;
        self->key_obstacles_height = 0;
        self->key_operating = 0;
        return 1;
    }

    if (event->keyval == 'b' || event->keyval == 'B') {
        viewer_request_pick(viewer, ehandler);
        self->key_operating = 1;
        self->key_obstacles = 0;
        self->key_obstacles_height = 0;
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
        self->key_obstacles_height = 0;
        self->key_goal = 0;
        self->key_operating = 0;
        return 1;
    }

    return 0;
}


void
smp_renderer_draw (Viewer *viewer, Renderer *renderer) {

    smp_renderer_t *self = (smp_renderer_t *)renderer->user;

    g_mutex_lock (self->mutex);
    
    glEnable (GL_LIGHTING);
    glEnable (GL_DEPTH_TEST);
    // glEnable (GL_LINE_SMOOTH);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);

    // Draw the graph
    lcmtypes_smp_graph_t *graph = self->graph_curr;

    if (graph) {

        if (self->show_graphs_vertices) {

            // Draw all the vertices            
            double vertex_size = ((double)(self->vertex_size))/10.0;
            glPointSize (vertex_size);
            glBegin (GL_POINTS);    
            for (int i = 0; i < graph->num_vertices; i++) {
                float color_vertex[] = {0.2, 0.2, 1.0, 0.9};
                glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_vertex);
                glVertex3f (graph->vertices[i].state.x, graph->vertices[i].state.y, graph->vertices[i].state.z);
            }

            glEnd ();
        }

        if (self->show_graphs_edges) {

        
            // Draw the edges 
            float color_edge[] = {1.0, 0.3, 0.3, 0.9};
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_edge);
            for (int i = 0; i < graph->num_edges; i++) {
                double edge_thickness = ((double)(self->edge_thickness))/10.0;
                glLineWidth (edge_thickness);
                glBegin (GL_LINE_STRIP);
                glVertex3f (graph->edges[i].vertex_src.state.x, 
                            graph->edges[i].vertex_src.state.y,
                            graph->edges[i].vertex_src.state.z);
                for (int j = 0; j < graph->edges[i].trajectory.num_states; j++) {
                    glVertex3f (graph->edges[i].trajectory.states[j].x, 
                                graph->edges[i].trajectory.states[j].y,
                                graph->edges[i].trajectory.states[j].z);
                }
                glVertex3f (graph->edges[i].vertex_dst.state.x, 
                            graph->edges[i].vertex_dst.state.y,
                            graph->edges[i].vertex_dst.state.z);
                glEnd ();
            }
        }
    }



    if (self->show_environments) {    
        // Draw the operating region

        float color_operating_region[] = { 1.0, 0.1, 0.1, 0.9};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_operating_region);
        glLineWidth (3.0);

        glBegin (GL_LINE_LOOP);
        glVertex3f (self->region_operating.center[0] - self->region_operating.size[0]/2.0, 
                    self->region_operating.center[1] - self->region_operating.size[1]/2.0, 
                    self->region_operating.center[2] - self->region_operating.size[2]/2.0 );

        glVertex3f (self->region_operating.center[0] + self->region_operating.size[0]/2.0, 
                    self->region_operating.center[1] - self->region_operating.size[1]/2.0,
                    self->region_operating.center[2] - self->region_operating.size[2]/2.0 );

        glVertex3f (self->region_operating.center[0] + self->region_operating.size[0]/2.0, 
                    self->region_operating.center[1] + self->region_operating.size[1]/2.0,
                    self->region_operating.center[2] - self->region_operating.size[2]/2.0 );
    
        glVertex3f (self->region_operating.center[0] - self->region_operating.size[0]/2.0, 
                    self->region_operating.center[1] + self->region_operating.size[1]/2.0, 
                    self->region_operating.center[2] - self->region_operating.size[2]/2.0 );
        glEnd ();

        if (!self->show_projected) {
            glBegin (GL_LINE_LOOP);
            glVertex3f (self->region_operating.center[0] - self->region_operating.size[0]/2.0, 
                        self->region_operating.center[1] - self->region_operating.size[1]/2.0, 
                        self->region_operating.center[2] + self->region_operating.size[2]/2.0 );

            glVertex3f (self->region_operating.center[0] + self->region_operating.size[0]/2.0, 
                        self->region_operating.center[1] - self->region_operating.size[1]/2.0,
                        self->region_operating.center[2] + self->region_operating.size[2]/2.0 );

            glVertex3f (self->region_operating.center[0] + self->region_operating.size[0]/2.0, 
                        self->region_operating.center[1] + self->region_operating.size[1]/2.0,
                        self->region_operating.center[2] + self->region_operating.size[2]/2.0 );
    
            glVertex3f (self->region_operating.center[0] - self->region_operating.size[0]/2.0, 
                        self->region_operating.center[1] + self->region_operating.size[1]/2.0, 
                        self->region_operating.center[2] + self->region_operating.size[2]/2.0 );
            glEnd ();
        }    
    
        // Draw the goal region

        float color_goal_region[] = {0.1, 1.0, 0.1, 0.9};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_goal_region);
        glLineWidth (3.0);

        glBegin (GL_LINE_LOOP);
        glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                    self->region_goal.center[1] - self->region_goal.size[1]/2.0, 
                    self->region_goal.center[2] - self->region_goal.size[2]/2.0 );

        glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                    self->region_goal.center[1] - self->region_goal.size[1]/2.0,
                    self->region_goal.center[2] - self->region_goal.size[2]/2.0 );

        glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                    self->region_goal.center[1] + self->region_goal.size[1]/2.0,
                    self->region_goal.center[2] - self->region_goal.size[2]/2.0 );

        glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                    self->region_goal.center[1] + self->region_goal.size[1]/2.0,
                    self->region_goal.center[2] - self->region_goal.size[2]/2.0 );
        glEnd ();

        if (!self->show_projected) {
            glBegin (GL_LINE_LOOP);
            glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                        self->region_goal.center[1] - self->region_goal.size[1]/2.0, 
                        self->region_goal.center[2] + self->region_goal.size[2]/2.0 );

            glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                        self->region_goal.center[1] - self->region_goal.size[1]/2.0,
                        self->region_goal.center[2] + self->region_goal.size[2]/2.0 );

            glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                        self->region_goal.center[1] + self->region_goal.size[1]/2.0,
                        self->region_goal.center[2] + self->region_goal.size[2]/2.0 );

            glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                        self->region_goal.center[1] + self->region_goal.size[1]/2.0,
                        self->region_goal.center[2] + self->region_goal.size[2]/2.0 );
            glEnd ();
        }

    
        // Draw the obstacles
        GSList *list_obstacle_curr = self->list_region_obstacles;
        while (list_obstacle_curr) {
            region_3d_t *region_obstacle_curr = (region_3d_t *) list_obstacle_curr->data;

            if (self->show_projected) {
                if (region_obstacle_curr == self->hover_obstacle_region) {
                    float color_obstacles[] = { 0.5, 0.2, 0.2, 0.5};
                    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
                }
                else {
                    float color_obstacles[] = { 0.3, 0, 0, ((double)(self->obstacle_opacity))/100.0};
                    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
                }
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
            
            }
            else {

                if (region_obstacle_curr == self->hover_obstacle_region) {
                    float color_obstacles[] = { 0.5, 0.2, 0.2, 0.5};
                    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
                }
                else {
                    float color_obstacles[] = { 0.3, 0, 0, ((double)(self->obstacle_opacity))/100.0};
                    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
                }

                glPushMatrix ();
            
                glTranslated (region_obstacle_curr->center[0], region_obstacle_curr->center[1], region_obstacle_curr->center[2]);
                glRotatef (0.0, 0.0, 0.0, 1.0);
            
                glScalef (region_obstacle_curr->size[0], region_obstacle_curr->size[1], region_obstacle_curr->size[2]);
            
                bot_gl_draw_cube ();
            
                glPopMatrix ();
            
            }
            list_obstacle_curr = g_slist_next (list_obstacle_curr);
        }

    }
    


    if (self->show_trajectories) {
        // Draw a trajectory
        lcmtypes_smp_trajectory_t *trajectory = self->trajectory_curr;
        if (trajectory) {

            float color_trajectory[] = {0.9, 0.9, 0.9, 0.9};
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_trajectory);
            glLineWidth (3.0);

            glBegin (GL_LINE_STRIP);        
            for (int i = 0; i < trajectory->num_states - 1; i++) {
                glVertex3f (trajectory->states[i].x, 
                            trajectory->states[i].y,
                            trajectory->states[i].z);
            }
            glEnd ();
        
        }
    }


    glDisable (GL_LIGHTING);
    glDisable (GL_DEPTH_TEST);
    glDisable (GL_LINE_SMOOTH);
    glDisable (GL_BLEND);
        

    glFlush ();    

    g_mutex_unlock (self->mutex);    

}


void 
setup_renderer_smp (Viewer *viewer, int priority) {

    smp_renderer_t *self = (smp_renderer_t *) calloc (sizeof (smp_renderer_t),1);

    self->list_graphs = NULL;
    self->num_graphs = 0;
    self->graph_curr = NULL;

    self->list_trajectories = NULL;
    self->num_trajectories = 0;
    self->trajectory_curr = NULL;

    self->list_environments = NULL;
    self->num_environments = 0;
    self->environment_curr = NULL;

        
    self->mutex = g_mutex_new ();

    self->mouseNodeXY[0] = 0.0;
    self->mouseNodeXY[1] = 0.0;

    Renderer *renderer = &(self->renderer);

    renderer->draw = smp_renderer_draw;
    renderer->destroy = smp_renderer_destroy;
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


    self->region_operating.center[0] = 0.0;
    self->region_operating.center[1] = 0.0;
    self->region_operating.center[2] = 2.5;
    self->region_operating.size[0] = 20.0;
    self->region_operating.size[1] = 20.0;
    self->region_operating.size[2] = 5.0;

    self->region_goal.center[0] = 8.0;
    self->region_goal.center[1] = 8.0;
    self->region_goal.center[2] = 0.5;
    self->region_goal.size[0] = 3.0;
    self->region_goal.size[1] = 3.0;
    self->region_goal.size[2] = 1.0;
    
    
    self->key_obstacles = 0;
    self->key_obstacles_height = 0;
    self->list_region_obstacles = NULL;
    self->hover_obstacle_region = NULL;
    self->last_region_obstacles.center[0] = 0.0;
    self->last_region_obstacles.center[1] = 0.0;
    self->last_region_obstacles.size[0] = 0.0;
    self->last_region_obstacles.size[1] = 0.0;

    self->key_operating = 0;
    self->last_region_operating.center[0] = self->region_operating.center[0];
    self->last_region_operating.center[1] = self->region_operating.center[1];
    self->last_region_operating.center[2] = self->region_operating.center[2];
    self->last_region_operating.size[0] = self->region_operating.size[0];
    self->last_region_operating.size[1] = self->region_operating.size[1];
    self->last_region_operating.size[2] = self->region_operating.size[2];

    self->key_goal = 0;
    self->last_region_goal.center[0] = self->region_goal.center[0];
    self->last_region_goal.center[1] = self->region_goal.center[1];
    self->last_region_goal.center[2] = self->region_goal.center[2];
    self->last_region_goal.size[0] = self->region_goal.size[0];
    self->last_region_goal.size[1] = self->region_goal.size[1];
    self->last_region_goal.size[2] = self->region_goal.size[2];


    // Set up the default environment
    self->environment_default.num_obstacles = 0;
    self->environment_default.operating.center[0] = self->region_operating.center[0];
    self->environment_default.operating.center[1] = self->region_operating.center[1];
    self->environment_default.operating.center[2] = self->region_operating.center[2];
    self->environment_default.operating.size[0] = self->region_operating.size[0];
    self->environment_default.operating.size[1] = self->region_operating.size[1];
    self->environment_default.operating.size[2] = self->region_operating.size[2];
    self->environment_default.goal.center[0] = self->region_goal.center[0];
    self->environment_default.goal.center[1] = self->region_goal.center[1];
    self->environment_default.goal.center[2] = self->region_goal.center[2];
    self->environment_default.goal.size[0] = self->region_goal.size[0];
    self->environment_default.goal.size[1] = self->region_goal.size[1];
    self->environment_default.goal.size[2] = self->region_goal.size[2];
    
    self->environment_default.num_obstacles = 0;
    self->environment_default.obstacles = NULL;

    // Make a copy of the default environment and add it to the list
    self->environment_curr = lcmtypes_smp_environment_t_copy (&(self->environment_default));
    self->list_environments = g_list_prepend (self->list_environments, self->environment_curr);
    self->num_environments++;




    // Add show projected check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_PROJECT, 0, NULL);
    self->show_projected = FALSE;



    // Add show graphs check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_SHOW_GRAPHS_VERTICES, 1, NULL);
    self->show_graphs_vertices = TRUE;

    // Add show graphs check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_SHOW_GRAPHS_EDGES, 1, NULL);
    self->show_graphs_edges = TRUE;

    // Add the graph history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, GRAPH_SLIDER_HISTORY, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 2, 1, 1);
    bot_gtk_param_widget_set_enabled (self->pw, GRAPH_SLIDER_HISTORY, 0);
    self->graph_slider_history_no = 1;


    // Add show trajectories check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_SHOW_TRAJECTORIES, 0, NULL);
    self->show_trajectories = FALSE;

    // Add the trajectory history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, TRAJECTORY_SLIDER_HISTORY, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 2, 1, 1);
    bot_gtk_param_widget_set_enabled (self->pw, TRAJECTORY_SLIDER_HISTORY, 0);
    self->trajectory_slider_history_no = 1;


    // Add show trajectories check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_SHOW_ENVIRONMENTS, 1, NULL);
    self->show_environments = TRUE;

    // Add the environment history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, ENVIRONMENT_SLIDER_HISTORY, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 2, 1, 1);
    bot_gtk_param_widget_set_enabled (self->pw, ENVIRONMENT_SLIDER_HISTORY, 0);
    self->environment_slider_history_no = 1;

    // Add the clear all history button
    GtkWidget *clear_button = gtk_button_new_with_label ("Clear All History");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked",
                     G_CALLBACK(on_clear_button), self);


    // Add the clear graph history button
    GtkWidget *clear_graph_button = gtk_button_new_with_label ("Clear Graph History");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_graph_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_graph_button), "clicked",
                     G_CALLBACK(on_clear_graph_button), self);



    // Add the clear trajectory history button
    GtkWidget *clear_trajectory_button = gtk_button_new_with_label ("Clear Trajectory History");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_trajectory_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_trajectory_button), "clicked",
                     G_CALLBACK(on_clear_trajectory_button), self);


    // Add the clear environment history button
    GtkWidget *clear_environment_button = gtk_button_new_with_label ("Clear Environment History");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_environment_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_environment_button), "clicked",
                     G_CALLBACK(on_clear_environment_button), self);

    
    // Add the clear environment history button
    GtkWidget *start_new_environment_button = gtk_button_new_with_label ("Start New Environment");
    gtk_box_pack_start(GTK_BOX(renderer->widget), start_new_environment_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(start_new_environment_button), "clicked",
                     G_CALLBACK(on_start_new_environment_button), self);


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


    // Add the graph history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, SLIDER_OBSTACLE_OPACITY, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 100, 1, 90);
    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_OBSTACLE_OPACITY, 1);
    self->obstacle_opacity = 90;


    // Add the graph history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, SLIDER_VERTEX_SIZE, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 100, 1, 30);
    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_VERTEX_SIZE, 1);
    self->vertex_size = 30;


    // Add the graph history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, SLIDER_EDGE_THICKNESS, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, 100, 1, 15);
    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_EDGE_THICKNESS, 1);
    self->edge_thickness = 15;


    // Show the widgets
    gtk_widget_show_all (renderer->widget);
    g_signal_connect (G_OBJECT (self->pw), "changed", G_CALLBACK (on_param_widget_changed), self);


    // Subscribe to SMP messages
    lcmtypes_smp_graph_t_subscribe (self->lcm, "SMP_GRAPH", on_smp_graph, self);
    lcmtypes_smp_trajectory_t_subscribe (self->lcm, "SMP_TRAJECTORY", on_smp_trajectory, self);
    lcmtypes_smp_environment_t_subscribe (self->lcm, "SMP_ENVIRONMENT", on_smp_environment, self);


    viewer_add_renderer (viewer, renderer, priority);
    viewer_add_event_handler(viewer, ehandler, priority);

    return;
}

