Rapidly-exploring random graphs library
=======================================

References
----------

We provide here implementations of RRG [1] and the optimal extension [2]. The
latter makes use of important results from [3].

1. S. Karaman and E. Frazzoli (2009). Sampling-based Motion Planning with
  Deterministic $mu$-Calculus Specifications. in Proceedings of the 48th IEEE
  Conference on Decision and Control (CDC), pp. 2222-2229.
  DOI: 10.1109/CDC.2009.5400278

2. S. Karaman and E. Frazzoli (2012). Sampling-based Algorithms for Optimal
  Motion Planning with Deterministic $mu$-Calculus Specifications. in
  Proceedings of the American Control Conference (ACC), pp. 735-74.
  DOI: 10.1109/ACC.2012.6315419

3. S. Karaman and E. Frazzoli (2011). Sampling-based Algorithms for Optimal
  Motion Planning. International Journal of Robotics Research 30(7): 846-894.
  DOI: 10.1177/0278364911406761


Compiling examples
------------------

To build all of the "standalone" examples (i.e., those not depending on libbot),

    make all

The library consists of .h and .hpp header files.  It does not require
compilation a priori.  Alternatively to `make all`, individual examples may be
built by first creating the bin/ directory if it does not exist,

    mkdir -p bin

and then going into the directory

    examples/

where you can find all the examples. Consult notes near the end of this README
concerning examples and usage here of `libbot`. Type `make` in the folder that
contains the particular example you would like to execute. This should compile
the part of the library required by that example, and the resulting binary will
be placed in the bin/ directory at the root of the source tree.


Compiling the documentation
---------------------------

    make doc

requires [Doxygen](http://www.doxygen.org/) to be installed.


Visualizations, interfaces
--------------------------

Note that visualization is not intended to be the focus of this repository, so
Mayavi, Matplotlib, etc. as used in tools/rrgplot.py are regarded as optional
dependencies.  Similarly, building the various interfaces may require external
resources to be available, like ROS.  However, those dependencies are only
necessary if you want to build that particular interface.

* [libbot](https://code.google.com/p/libbot/)
* [LCM](https://lcm-proj.github.io/)


Authors
-------

Scott C. Livingston (Caltech)
Sertac Karaman and Emilio Frazzoli (MIT)


License
-------

This is free software released under the terms of [the MIT License]
(http://opensource.org/licenses/MIT).  There is no warranty; not even for
merchantability or fitness for a particular purpose.  Consult LICENSE for
copying conditions.
