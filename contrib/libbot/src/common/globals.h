#ifndef __rl_globals_h__
#define __rl_globals_h__

// file:  globals.h
// desc:  prototypes for accessing global/singleton objects -- objects that
//        will typically be created once throughout the lifespan of a program.

#include <glib.h>
#include <lcm/lcm.h>
#include <bot/bot_core.h>

#ifdef __cplusplus
extern "C" {
#endif

lcm_t * globals_get_lcm (void);
void globals_release_lcm (lcm_t *lcm);

#ifdef __cplusplus
}
#endif

#endif
