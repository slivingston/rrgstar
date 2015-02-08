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

#define RENDERER_NAME "Vehicle"

#define PARAM_FOLLOW_POS "Follow position"
#define PARAM_FOLLOW_YAW "Follow yaw"
#define PARAM_MAXPOSES "Max poses"

#define MAX_POSES   10000

#define SQ(a) ((a)*(a))

typedef struct _RendererCar {
    Renderer renderer;
    EventHandler ehandler;

    lcm_t *lc;

    Viewer          *viewer;
    BotGtkParamWidget *pw;
    BotPtrCircular    *path; // elements: double[3] 
    botlcm_pose_t  *last_pose;
    botlcm_pose_t_subscription_t *pose_subscription;
    int             max_draw_poses;

    int          display_detail;
    double       last_xy[2]; // last mouse press

    pointlist2d_t *footprint;
} RendererCar;

enum {
    DETAIL_NONE,
    DETAIL_SPEED,
    DETAIL_RPY,
    NUM_DETAILS
};

// called by bot_ptr_circular when a path sample needs to be freed.
static void free_path_element(void *user, void *p)
{
    free(p);
}

static void on_find_button(GtkWidget *button, RendererCar *self)
{
    if(!self->last_pose)
        return;

    ViewHandler *vhandler = self->viewer->view_handler;

    double eye[3];
    double lookat[3];
    double up[3];

    vhandler->get_eye_look(vhandler, eye, lookat, up);
    double diff[3];
    bot_vector_subtract_3d(eye, lookat, diff);

    double *pos = self->last_pose->pos;

    bot_vector_add_3d(pos, diff, eye);

    vhandler->set_look_at(vhandler, eye, pos, up);

    viewer_request_redraw(self->viewer);
}

static void
on_pose(const lcm_recv_buf_t *rbuf, const char *channel, 
        const botlcm_pose_t *msg, void *user)
{
    RendererCar *self = (RendererCar*) user;
    ViewHandler *vhandler = self->viewer->view_handler;

    double lastpos[3] = {0,0,0};
    if (bot_ptr_circular_size(self->path))
        memcpy(lastpos, bot_ptr_circular_index(self->path, 0),
               3 * sizeof(double));

    double diff[3];
    bot_vector_subtract_3d(msg->pos, lastpos, diff);

    if (bot_vector_magnitude_3d(diff) > 2.0) {
        // clear the buffer if we jump
        bot_ptr_circular_clear(self->path);
    }

    if (bot_vector_magnitude_3d(diff) > 0.1 ||
            bot_ptr_circular_size(self->path)==0) {
        double *p = (double*) calloc(3, sizeof(double));
        memcpy(p, msg->pos, sizeof(double)*3);
        bot_ptr_circular_add(self->path, p);
    }

    if (vhandler && vhandler->update_follow_target) {
        vhandler->update_follow_target(vhandler, msg->pos, msg->orientation);
    }

    on_find_button(NULL, self);

    if(self->last_pose)
        botlcm_pose_t_destroy(self->last_pose);
    self->last_pose = botlcm_pose_t_copy(msg);

    viewer_request_redraw(self->viewer);
}

static void
car_free (Renderer *super)
{
    RendererCar *self = (RendererCar*) super->user;
    free (self);
}

static void
draw_footprint (RendererCar * self)
{
    assert (4 == self->footprint->npoints);

    glPushMatrix ();
    glLineWidth(2);
    glColor4f (1, 1, 1, .3);
    glEnable(GL_BLEND);
    glBegin (GL_QUADS);
    for (int i=0; i<4; i++)
        glVertex2f (self->footprint->points[i].x, 
                    self->footprint->points[i].y);
    glEnd ();

    glColor4f (1, 1, 1, 1);
    glBegin (GL_LINE_LOOP);
    for (int i=0; i<4; i++)
        glVertex2f (self->footprint->points[i].x, 
                    self->footprint->points[i].y);
    glEnd();

    point2d_t fp_centroid = { 0, 0 };
    geom_simple_polygon_centroid_2d (self->footprint, &fp_centroid);
    glTranslatef (fp_centroid.x, fp_centroid.y, .001);

    point2d_t fl = self->footprint->points[0];
    point2d_t fr = self->footprint->points[1];
    point2d_t br = self->footprint->points[2];

    double fp_length = fr.x - br.x;
    double fp_width = fabs (fr.y - fl.y);

    bot_gl_draw_arrow_2d (fp_length, fp_width, fp_length * 0.3, 
            fp_width * 0.5, self->ehandler.hovering);
    glPopMatrix ();
}

static void 
car_draw (Viewer *viewer, Renderer *super)
{
    RendererCar *self = (RendererCar*) super->user;

    if(!self->last_pose)
        return;

    glColor4f(0,1,0,0.75);
    glLineWidth (2);
    glBegin(GL_LINE_STRIP);
    glVertex3dv (self->last_pose->pos);
    for (unsigned int i = 0;
            i < MIN (bot_ptr_circular_size(self->path), self->max_draw_poses);
            i++) {
        glVertex3dv(bot_ptr_circular_index(self->path, i));
    }
    glEnd();

    glPushMatrix();

    // compute the rotation matrix to orient the vehicle in world
    // coordinates
    double body_quat_m[16];
    bot_quat_pos_to_matrix(self->last_pose->orientation, 
            self->last_pose->pos, body_quat_m);

    // opengl expects column-major matrices
    double body_quat_m_opengl[16];
    bot_matrix_transpose_4x4d (body_quat_m, body_quat_m_opengl);

    // rotate and translate the vehicle
    glMultMatrixd (body_quat_m_opengl);

    glEnable (GL_DEPTH_TEST);

    draw_footprint (self);

    glPopMatrix();

    if (self->display_detail) {
        char buf[256];
        switch (self->display_detail) 
        {
        case DETAIL_SPEED:
            sprintf(buf, "%.2f m/s", 
                    bot_vector_magnitude_3d(self->last_pose->vel));
            break;
        case DETAIL_RPY:
        {
            double rpy[3];
            bot_quat_to_roll_pitch_yaw(self->last_pose->orientation, rpy);
            sprintf(buf, "r: %6.2f\np: %6.2f\ny: %6.2f", 
                   bot_to_degrees(rpy[0]), 
                   bot_to_degrees(rpy[1]), 
                   bot_to_degrees(rpy[2]));
            break;
        }
//        case DETAIL_GPS:
//        {
//            double lle[3], q[4];
//            ctrans_gps_pose(self->ctrans, lle, q);
//            double rpy[3];
//            bot_quat_to_roll_pitch_yaw(q, rpy);
//            sprintf(buf, "%15.7f %15.7f\nelev: %10.3f\nhead: %6.2f\n", lle[0], lle[1], lle[2], to_degrees(rpy[2]));
//            break;
//        }
        }
        glColor3f(1,1,1);
        bot_gl_draw_text(self->last_pose->pos, GLUT_BITMAP_HELVETICA_12, buf,
                         BOT_GL_DRAW_TEXT_DROP_SHADOW);
    }
}

static void on_clear_button(GtkWidget *button, RendererCar *self)
{
    bot_ptr_circular_clear(self->path);

    viewer_request_redraw(self->viewer);
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererCar *self = (RendererCar*) user;
    Viewer *viewer = self->viewer;

    viewer->view_handler->follow_mode = 0;
    if (bot_gtk_param_widget_get_bool(pw, PARAM_FOLLOW_POS))
        viewer->view_handler->follow_mode |= FOLLOW_POS;
    if (bot_gtk_param_widget_get_bool(pw, PARAM_FOLLOW_YAW))
        viewer->view_handler->follow_mode |= FOLLOW_YAW;

    self->max_draw_poses = bot_gtk_param_widget_get_int(pw, PARAM_MAXPOSES);

    viewer_request_redraw ( self->viewer);
}

static int mouse_press (Viewer *viewer, EventHandler *ehandler,
                        const double ray_start[3], const double ray_dir[3], 
                        const GdkEventButton *event)
{
    RendererCar *self = (RendererCar*) ehandler->user;

    if (event->type == GDK_2BUTTON_PRESS) {
        self->display_detail = (self->display_detail + 1) % NUM_DETAILS;
        viewer_request_redraw(self->viewer);
    }

    double carpos[3] = { 0, 0, 0 };
    if(self->last_pose)
        memcpy(carpos, self->last_pose->pos, 3 * sizeof(double));
    geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
            POINT3D(ray_dir), carpos[2], POINT2D(self->last_xy));

    return 0;
}

static double pick_query(Viewer *viewer, EventHandler *ehandler, const double ray_start[3], const double ray_dir[3])
{
    RendererCar *self = (RendererCar*) ehandler->user;

    if(!self->last_pose)
        return -1;

    double ray_start_body[3];
    bot_vector_subtract_3d (ray_start, self->last_pose->pos, ray_start_body);
    bot_quat_rotate_rev (self->last_pose->orientation, ray_start_body);

    double ray_dir_body[3] = { ray_dir[0], ray_dir[1], ray_dir[2] };
    bot_quat_rotate_rev (self->last_pose->orientation, ray_dir_body);
    bot_vector_normalize_3d (ray_dir_body);

    point3d_t car_pos_body = { 1.3, 0, 1 };
    point3d_t box_size = { 4.6, 2, 1.4 };
    double t = geom_ray_axis_aligned_box_intersect_3d (POINT3D(ray_start_body), 
            POINT3D (ray_dir_body), &car_pos_body, &box_size, NULL);
    if (isfinite (t)) return t;

    self->ehandler.hovering = 0;
    return -1;
}

static void
on_load_preferences (Viewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererCar *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (Viewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererCar *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

void setup_renderer_car(Viewer *viewer, int render_priority)
{
    RendererCar *self = (RendererCar*) calloc (1, sizeof (RendererCar));

    Renderer *renderer = &self->renderer;

    renderer->draw = car_draw;
    renderer->destroy = car_free;

    renderer->widget = gtk_vbox_new(FALSE, 0);
    renderer->name = RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    EventHandler *ehandler = &self->ehandler;
    ehandler->name = RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = pick_query;
    ehandler->key_press = NULL;
    ehandler->hover_query = pick_query;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = NULL;
    ehandler->mouse_motion = NULL;
    ehandler->user = self;

    self->viewer = viewer;
    self->lc = globals_get_lcm ();
    self->last_pose = NULL;

    self->pose_subscription = botlcm_pose_t_subscribe(self->lc, "POSE", 
            on_pose, self);

    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    self->path = bot_ptr_circular_new (MAX_POSES, free_path_element, NULL);

    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, 
            TRUE, 0);

    bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_FOLLOW_POS, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, 0, PARAM_FOLLOW_YAW, 0, NULL);

    self->max_draw_poses = 1000;
    bot_gtk_param_widget_add_int (self->pw, PARAM_MAXPOSES, 
            BOT_GTK_PARAM_WIDGET_SLIDER, 0, MAX_POSES, 100, 
            self->max_draw_poses);
 
    GtkWidget *find_button = gtk_button_new_with_label("Find");
    gtk_box_pack_start(GTK_BOX(renderer->widget), find_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(find_button), "clicked", 
            G_CALLBACK (on_find_button), self);

    GtkWidget *clear_button = gtk_button_new_with_label("Clear path");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 
            0);
    g_signal_connect(G_OBJECT(clear_button), "clicked", 
            G_CALLBACK (on_clear_button), self);

    gtk_widget_show_all(renderer->widget);

    g_signal_connect (G_OBJECT (self->pw), "changed", 
                      G_CALLBACK (on_param_widget_changed), self);
    on_param_widget_changed(self->pw, "", self);

    viewer_add_renderer(viewer, &self->renderer, render_priority);
    viewer_add_event_handler(viewer, &self->ehandler, render_priority);

    double fp[] = {
         4,  1,
         4, -1,
         1, -1,
         1,  1
    };
    self->footprint = pointlist2d_new_from_double_array(fp, 4);

    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
            G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
            G_CALLBACK (on_save_preferences), self);
}
