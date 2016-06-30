/*! \file region.h
  \brief The standard brute-force collision checker

  This file implements the region class, which defines a rectangular region
  in the Euclidean space, the dimension of which is a template argument.
*/

#ifndef _RRGLIB_REGION_H_
#define _RRGLIB_REGION_H_


namespace rrglib {

    //! A rectangular region in an Euclidean space of prespecified dimension.
    /*!
      This class implements a rectangular in an Euclidean space of a certain
      dimension given by a template parameter.
    */
    template <int NUM_DIMENSIONS>
    class region {
    public:

        //! The coordinates of the center of the region.
        double center[NUM_DIMENSIONS];

        //! The size of the region in each dimension.
        double size[NUM_DIMENSIONS];


        region ();
        ~region ();

        /**
         * \brief Copy constructor
         */
        region (const region<NUM_DIMENSIONS> &region_in);


        /** element-wise copy */
        const region<NUM_DIMENSIONS> &operator=(const region<NUM_DIMENSIONS> &region_in);

        inline int numDim() const { return NUM_DIMENSIONS; }

    };

}


#endif
