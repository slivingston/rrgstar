#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm.h>

#include <bot/bot_core.h>
#include <bot/gl/gl_util.h>
#include <bot/gtk/gtk_util.h>
#include <bot/viewer/viewer.h>

#include <common/globals.h>

typedef struct
{
    GPtrArray *backbuffer;
    GPtrArray *frontbuffer;
    int enabled;
} lcgl_channel_t;

typedef struct _RendererLcmgl {
    Renderer renderer;
    BotGtkParamWidget *pw;
    Viewer   *viewer;
    lcm_t     *lcm;

    GHashTable *channels;

} RendererLcmgl;

static void my_free( Renderer *renderer )
{
    RendererLcmgl *self = (RendererLcmgl*) renderer;

    free( self );
}

static void my_draw( Viewer *viewer, Renderer *renderer )
{
    RendererLcmgl *self = (RendererLcmgl*) renderer->user;

    // iterate over each channel
    GList *keys = bot_g_hash_table_get_keys(self->channels);

    for (GList *kiter = keys; kiter; kiter=kiter->next) {
        lcgl_channel_t *chan = g_hash_table_lookup(self->channels,
                                                   kiter->data);
        glPushMatrix();
        glPushAttrib(GL_ENABLE_BIT);

        if (chan->enabled) {
            // iterate over all the messages received for this channel
            for (int i = 0; i < chan->frontbuffer->len; i++) {
                botlcm_lcmgl_data_t *data =
                    g_ptr_array_index(chan->frontbuffer, i);

                bot_lcmgl_decode(data->data, data->datalen);
            }
        }
        glPopAttrib ();
        glPopMatrix();
    }
    g_list_free (keys);
}

static void on_lcgl_data (const lcm_recv_buf_t *rbuf, const char *channel,
        const botlcm_lcmgl_data_t *_msg, void *user_data )
{
    RendererLcmgl *self = (RendererLcmgl*) user_data;

    lcgl_channel_t *chan = g_hash_table_lookup(self->channels, _msg->name);

    if (!chan) {
        chan = (lcgl_channel_t*) calloc(1, sizeof(lcgl_channel_t));
        chan->enabled=1;
        //chan->backbuffer = g_ptr_array_new();
        chan->frontbuffer = g_ptr_array_new();
        g_hash_table_insert(self->channels, strdup(_msg->name), chan);
        bot_gtk_param_widget_add_booleans (self->pw,
                0, strdup(_msg->name), 1, NULL);
    }

#if 0
    int current_scene = -1;
    if (chan->backbuffer->len > 0) {
        botlcm_lcmgl_data_t *ld = g_ptr_array_index(chan->backbuffer, 0);
        current_scene = ld->scene;
    }

    // new scene?
    if (current_scene != _msg->scene) {

        // free objects in foreground buffer
        for (int i = 0; i < chan->frontbuffer->len; i++)
            botlcm_lcmgl_data_t_destroy(g_ptr_array_index(chan->frontbuffer, i));
        g_ptr_array_set_size(chan->frontbuffer, 0);

        // swap front and back buffers
        GPtrArray *tmp = chan->backbuffer;
        chan->backbuffer = chan->frontbuffer;
        chan->frontbuffer = tmp;

        viewer_request_redraw( self->viewer );
    }
#endif

    for (int i = 0; i < chan->frontbuffer->len; i++)
        botlcm_lcmgl_data_t_destroy(g_ptr_array_index(chan->frontbuffer, i));
    g_ptr_array_set_size (chan->frontbuffer, 0);
    g_ptr_array_add(chan->frontbuffer, botlcm_lcmgl_data_t_copy(_msg));
    viewer_request_redraw( self->viewer );
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererLcmgl *self = (RendererLcmgl*) user;

    // iterate over each channel
    GList *keys = bot_g_hash_table_get_keys(self->channels);

    for (GList *kiter=keys; kiter; kiter=kiter->next) {
        lcgl_channel_t *chan = g_hash_table_lookup(self->channels,
                                                   kiter->data);

        chan->enabled = bot_gtk_param_widget_get_bool (pw, kiter->data);
    }
    g_list_free (keys);

    viewer_request_redraw(self->viewer);
}


void setup_renderer_lcmgl(Viewer *viewer, int priority)
{
    RendererLcmgl *self =
        (RendererLcmgl*) calloc(1, sizeof(RendererLcmgl));

    Renderer *renderer = &self->renderer;

    self->lcm = globals_get_lcm();
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    renderer->draw = my_draw;
    renderer->destroy = my_free;
    renderer->name = "LCM GL";
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    self->channels = g_hash_table_new(g_str_hash, g_str_equal);

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);

    botlcm_lcmgl_data_t_subscribe(self->lcm, "LCMGL.*", on_lcgl_data, self);

    viewer_add_renderer(viewer, renderer, priority);
}
