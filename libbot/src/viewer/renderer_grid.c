/*
 * renders a grid
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gtk/gtk.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <bot/bot_core.h>
#include <bot/gl/gl_util.h>
#include <bot/gtk/gtk_util.h>
#include <bot/viewer/viewer.h>

#include <common/globals.h>

#define PARAM_AZIMUTH "Rotation"
#define PARAM_BGLIGHT "Light"
#define PARAM_RENDER_GROUND "Ground"

#define RENDERER_NAME "Grid"

typedef struct _RendererGrid RendererGrid;

struct _RendererGrid {
    Renderer renderer;

    lcm_t *lcm;
    botlcm_pose_t      *last_pose;
    botlcm_pose_t_subscription_t *pose_subscription;

    BotGtkParamWidget *pw;
    double             last_meters_per_grid;
    GtkWidget         *label;
    Viewer *viewer;
};

/** round the input number to the next number of the form 1*10^n,
 * 2*10^n, or 5*10^n. */
static double round_to_125(double in)
{
    double v = 0.1;

    while (v < in) {
        if (v < in)
            v *= 2;
        if (v < in)
            v = v/2 * 5;
        if (v < in)
            v *= 2;
    }

    return v;
}

static void
grid_draw (Viewer *viewer, Renderer *renderer)
{
    RendererGrid *self = (RendererGrid*) renderer;

    double eye[3];
    double look[3];
    double up[3];
    viewer->view_handler->get_eye_look(viewer->view_handler, eye, look, up);

    glPushAttrib (GL_ENABLE_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable (GL_DEPTH_TEST);

    glPushMatrix ();
    
    // when looking directly down at the world, about how many grids
    // should appear across the screen?
    double grids_per_screen = 10;

    double eye_dist = bot_vector_dist_3d (eye, look);
    double meters_per_grid = round_to_125(eye_dist / grids_per_screen );

    char txt[64];
    if(meters_per_grid >= 1) {
        snprintf (txt, sizeof (txt), "Spacing: %.0fm", meters_per_grid);
    } else {
        snprintf (txt, sizeof (txt), "Spacing: %.1fm", meters_per_grid);
    }
    gtk_label_set_text (GTK_LABEL (self->label), txt);

    double grid_ox = ceil (look[0] / meters_per_grid) * meters_per_grid;
    double grid_oy = ceil (look[1] / meters_per_grid) * meters_per_grid;
    double grid_oz = look[2];

    if(self->last_pose) {
        grid_oz = self->last_pose->pos[2];
    }
    int num_lines = 300;

    glTranslatef (grid_ox, grid_oy, grid_oz);
    glRotatef (bot_gtk_param_widget_get_double (self->pw, PARAM_AZIMUTH), 
            0, 0, 1);

    glPushMatrix ();
    double box_size = fmax((num_lines/2)*meters_per_grid, 10000);
    glScalef (box_size, box_size, box_size);

    // ground
    if (look[2] < eye[2] && 
            bot_gtk_param_widget_get_bool (self->pw, PARAM_RENDER_GROUND)) { 
        if (bot_gtk_param_widget_get_bool (self->pw, PARAM_BGLIGHT)) {
//             glColor4f(0.8, 0.8, 0.8, 0.6);
            glColor4f(0.0, 0.0, 0.0, 0.6);
        } else {
           glColor4f(0.0, 0.0, 0.0, 1.0);
//             glColor4f (0.2, 0.1, 0, 0.9);
        }

        glEnable (GL_POLYGON_OFFSET_FILL);
        glPolygonOffset (2.0, 2.0);

        glBegin(GL_QUADS);
        glVertex2d ( 1,  1);
        glVertex2d ( 1, -1);
        glVertex2d (-1, -1);
        glVertex2d (-1,  1);

        glEnd ();
        glDisable (GL_POLYGON_OFFSET_FILL);
    }

    glPopMatrix ();

    grid_oz+=.01;
    glLineWidth (1);
    glBegin (GL_LINES);

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_BGLIGHT)) {
        glColor3f (0.8, 0.8, 0.8);
    } else {
        glColor3f (0.2, 0.2, 0.2);
    }

    for (int i=0; i<num_lines; i++) {
        glVertex2f ((-num_lines/2 + i) * meters_per_grid,
                - num_lines/2 * meters_per_grid);
        glVertex2f ((-num_lines/2 + i) * meters_per_grid,
                num_lines/2 * meters_per_grid);

        glVertex2f (- num_lines/2 * meters_per_grid,
                (-num_lines/2 + i) * meters_per_grid);
        glVertex2f (num_lines/2 * meters_per_grid,
                (-num_lines/2 + i) * meters_per_grid);
    }
    glEnd ();

    glPopMatrix ();
    glPopAttrib ();
}

static void
grid_free (Renderer *renderer) 
{
    RendererGrid *self = (RendererGrid*) renderer;
    botlcm_pose_t_unsubscribe(self->lcm, self->pose_subscription);
    globals_release_lcm(self->lcm);
    if(self->last_pose)
        botlcm_pose_t_destroy(self->last_pose);
    free (renderer);
}

static void
on_pose(const lcm_recv_buf_t *rbuf, const char *channel, 
        const botlcm_pose_t *msg, void *user_data)
{
    RendererGrid *self = (RendererGrid*) user_data;
    if(self->last_pose)
        botlcm_pose_t_destroy(self->last_pose);
    self->last_pose = botlcm_pose_t_copy(msg);
}

static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *param,
        void *user_data)
{
    RendererGrid *self = (RendererGrid*) user_data;

    viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (Viewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererGrid *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (Viewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererGrid *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

Renderer *renderer_grid_new (Viewer *viewer)
{
    RendererGrid *self = (RendererGrid*) calloc (1, sizeof (RendererGrid));
    self->viewer = viewer;
    self->renderer.draw = grid_draw;
    self->renderer.destroy = grid_free;
    self->renderer.name = RENDERER_NAME;
    self->renderer.user = self;
    self->renderer.enabled = 1;
    self->lcm = globals_get_lcm();

    self->pose_subscription = botlcm_pose_t_subscribe(self->lcm, "POSE",
            on_pose, self);

    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);

    self->label = gtk_label_new ("Spacing: ???");
    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    gtk_misc_set_alignment (GTK_MISC (self->label), 0.0, 0.5);
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);

    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw), FALSE, TRUE, 0);
    gtk_box_pack_start (GTK_BOX (vbox), self->label, FALSE, TRUE, 0);
    gtk_widget_show (GTK_WIDGET (self->pw));
    gtk_widget_show (self->label);

    bot_gtk_param_widget_add_double (self->pw, PARAM_AZIMUTH,
            BOT_GTK_PARAM_WIDGET_SLIDER,
            0, 90, 0.1, 0);
	bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, 
            PARAM_BGLIGHT, 1, NULL);
	bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, 
            PARAM_RENDER_GROUND, 0, NULL);

    g_signal_connect (G_OBJECT (self->pw), "changed",
            G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences", 
            G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
            G_CALLBACK (on_save_preferences), self);

    return &self->renderer;
}

void setup_renderer_grid(Viewer *viewer, int render_priority)
{
    viewer_add_renderer(viewer, renderer_grid_new(viewer), render_priority);
}
