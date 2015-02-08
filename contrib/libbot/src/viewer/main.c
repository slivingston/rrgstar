#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <assert.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gtk/gtk.h>

#include <bot/bot_core.h>
#include <bot/viewer/viewer.h>

#include "udp_util.h"
#include <common/globals.h>

typedef struct {
    Viewer *viewer;
    lcm_t *lcm;
} state_t;

static int
logplayer_remote_on_key_press(Viewer *viewer, EventHandler *ehandler,
        const GdkEventKey *event)
{
    int keyval = event->keyval;

    switch (keyval)
    {
    case 'P':
    case 'p':
        udp_send_string("127.0.0.1", 53261, "PLAYPAUSETOGGLE");
        break;
    case 'N':
    case 'n':
        udp_send_string("127.0.0.1", 53261, "STEP");
        break;
    case '=':
    case '+':
        udp_send_string("127.0.0.1", 53261, "FASTER");
        break;
    case '_':
    case '-':
        udp_send_string("127.0.0.1", 53261, "SLOWER");
        break;
    case '[':
        udp_send_string("127.0.0.1", 53261, "BACK5");
        break;
    case ']':
        udp_send_string("127.0.0.1", 53261, "FORWARD5");
        break;
    default:
        return 0;
    }

    return 1;
}

/////////////////////////////////////////////////////////////

void setup_renderer_grid(Viewer *viewer, int render_priority);
//void setup_renderer_compass(Viewer *viewer, int priority);
void setup_renderer_car(Viewer *viewer, int render_priority);
void setup_renderer_lcmgl(Viewer *viewer, int render_priority);
void setup_renderer_smp(Viewer *viewer, int render_priority);

int main(int argc, char *argv[])
{
    gtk_init (&argc, &argv);
    glutInit (&argc, argv);
    g_thread_init (NULL);

    setlinebuf (stdout);

    state_t app;
    memset(&app, 0, sizeof(app));

    Viewer *viewer = viewer_new("Viewer");
    app.viewer = viewer;
    app.lcm = globals_get_lcm();

    // setup renderers
    setup_renderer_smp(viewer, 2);
    setup_renderer_grid(viewer, 1);
    setup_renderer_car(viewer, 0);
    setup_renderer_lcmgl(viewer, 0);

    // logplayer controls
    EventHandler *ehandler = (EventHandler*) calloc(1, sizeof(EventHandler));
    ehandler->name = "LogPlayer Remote";
    ehandler->enabled = 1;
    ehandler->key_press = logplayer_remote_on_key_press;
    viewer_add_event_handler(viewer, ehandler, 0);

    char *fname = g_build_filename (g_get_user_config_dir(), ".example-viewerrc",
            NULL);
    viewer_load_preferences (viewer, fname);
    gtk_main ();
    viewer_save_preferences (viewer, fname);
    free (fname);
    viewer_unref (viewer);

    globals_release_lcm(app.lcm);
}
