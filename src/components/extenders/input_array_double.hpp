#ifndef _RRGLIB_INPUT_ARRAY_DOUBLE_HPP_
#define _RRGLIB_INPUT_ARRAY_DOUBLE_HPP_


#include <components/extenders/input_array_double.h>

#include <iostream>

using namespace std;

template <int NUM_INPUTS>
rrglib::input_array_double<NUM_INPUTS>
::input_array_double () {

    for (int i = 0; i < NUM_INPUTS; i++)
        input_vars[i] = 0.0;

}


template <int NUM_INPUTS>
rrglib::input_array_double<NUM_INPUTS>
::input_array_double (const rrglib::input_array_double<NUM_INPUTS> &input_in) {

    for (int i = 0; i < NUM_INPUTS; i++)
        input_vars[i] = input_in.input_vars[i];

}


template <int NUM_INPUTS>
rrglib::input_array_double<NUM_INPUTS>
::~input_array_double () {

}


template <int NUM_INPUTS>
const rrglib::input_array_double<NUM_INPUTS> &
rrglib::input_array_double<NUM_INPUTS>
::operator=(const rrglib::input_array_double<NUM_INPUTS> &input_in) {

    if (&input_in != this) {
        for (int i = 0; i < NUM_INPUTS; i++)
            input_vars[i] = input_in.input_vars[i];
    }

    return *this;
}

template <int NUM_INPUTS>
double &
rrglib::input_array_double<NUM_INPUTS>
::operator[] (int index_in) {

    if ( (index_in < NUM_INPUTS) && (index_in >= 0) )
        return input_vars[index_in];

    // TODO: ideally throw an exception also.
    return input_vars[0];
}


#endif
