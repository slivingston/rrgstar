#ifndef _RRGLIB_REGION_HPP_
#define _RRGLIB_REGION_HPP_


#include <common/region.h>


template <int NUM_DIMENSIONS>
rrglib::region<NUM_DIMENSIONS>
::region () {

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
        this->center[i] = 0.0;
        this->size[i] = 0.0;
    }
}


template <int NUM_DIMENSIONS>
rrglib::region<NUM_DIMENSIONS>
::~region () {


}


template <int NUM_DIMENSIONS>
rrglib::region<NUM_DIMENSIONS>
::region (const rrglib::region<NUM_DIMENSIONS> &region_in) {

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
        this->center[i] = region_in.center[i];
        this->size[i] = region_in.size[i];
    }
}


template <int NUM_DIMENSIONS>
const rrglib::region<NUM_DIMENSIONS> &
rrglib::region<NUM_DIMENSIONS>
::operator=(const rrglib::region<NUM_DIMENSIONS> &region_in) {

    if (&region_in != this) {
        for (int i = 0; i < NUM_DIMENSIONS; i++) {
            this->center[i] = region_in.center[i];
            this->size[i] = region_in.size[i];
        }
    }

    return *this;
}


#endif
