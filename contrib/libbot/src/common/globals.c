#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>

#include <glib.h>

#include <bot/bot_core.h>
#include "globals.h"

//#define dbg(...) fprintf(stderr, __VA_ARGS__)
#define dbg(...) 

#define err(...) fprintf(stderr, __VA_ARGS__)

#define MAX_REFERENCES ((1LL << 60))

static lcm_t *global_lcm;
static int64_t global_lcm_refcount;

static GStaticRecMutex _mutex = G_STATIC_REC_MUTEX_INIT;

lcm_t * 
globals_get_lcm (void)
{
    g_static_rec_mutex_lock (&_mutex);
    lcm_t *result = NULL;

    if (global_lcm_refcount == 0) {
        assert (! global_lcm);

        global_lcm = lcm_create (NULL);
        if (! global_lcm) { goto fail; }

        bot_glib_mainloop_attach_lcm (global_lcm);
    }

    assert (global_lcm);

    if (global_lcm_refcount < MAX_REFERENCES) global_lcm_refcount++;
    result = global_lcm;
    g_static_rec_mutex_unlock (&_mutex);
    return result;
fail:
    g_static_rec_mutex_unlock (&_mutex);
    return NULL;
}

void 
globals_release_lcm (lcm_t *lcm)
{
    g_static_rec_mutex_lock (&_mutex);
    if (global_lcm_refcount == 0) {
        fprintf (stderr, "ERROR: singleton LC refcount already zero!\n");
        g_static_rec_mutex_unlock (&_mutex);
        return;
    }
    if (lcm != global_lcm) {
        fprintf (stderr, "ERROR: %p is not the singleton LC (%p)\n",
                lcm, global_lcm);
    }
    global_lcm_refcount--;

    if (global_lcm_refcount == 0) {
        bot_glib_mainloop_detach_lcm (global_lcm);
        lcm_destroy (global_lcm);
        global_lcm = NULL;
    }
    g_static_rec_mutex_unlock (&_mutex);
}

