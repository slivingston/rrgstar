Rapidly-exploring random graphs library
=======================================

References
----------

We provide here implementations of RRG [1] and the optimal extension [2] for
motion planning subject to deterministic μ-Calculus constraints. The latter
paper makes use of important results from [3]. The implementation includes
corrections to and missing details from [2].

1. S. Karaman and E. Frazzoli (2009). Sampling-based Motion Planning with
  Deterministic μ-Calculus Specifications. in Proceedings of the 48th IEEE
  Conference on Decision and Control (CDC), pp. 2222-2229.
  DOI: 10.1109/CDC.2009.5400278

2. S. Karaman and E. Frazzoli (2012). Sampling-based Algorithms for Optimal
  Motion Planning with Deterministic μ-Calculus Specifications. in Proceedings
  of the American Control Conference (ACC), pp. 735-74.
  DOI: 10.1109/ACC.2012.6315419

3. S. Karaman and E. Frazzoli (2011). Sampling-based Algorithms for Optimal
  Motion Planning. International Journal of Robotics Research 30(7): 846-894.
  DOI: 10.1177/0278364911406761


Compiling examples, documentation
---------------------------------

The usual [CMake](http://www.cmake.org) idiom,

    mkdir build
    cd build
    cmake ..

On a standard UNIX platform, you can next run `make` to build all examples, the
programs for which will be under bin/.  To instead build a particular example,
call `make` with its name, e.g., to build only the rrg_dubins_car example,

    make rrg_dubins_car

To build the API manual,

    make doc

which requires [Doxygen](http://www.doxygen.org/) to be installed. The generated
files will be under the directory html/


Using rrglib
------------

The rrglib library consists primarily of header files and does not require
compilation a priori.  However, support is provided for installing header files
and libraries in a common location and against which other programs can
link. The default CMake install prefix can be changed by providing a value for
CMAKE_INSTALL_PREFIX. E.g., `cmake -DCMAKE_INSTALL_PREFIX=~/opt ..` will use the
opt/ directory in your home directory instead of the system-wide /usr or
/usr/local.  HPP files will be placed under include/rrglib/ (after the install
prefix), and compiled object and CMake configuration files will be installed
under lib/rrglib/

For example, to use it in a CMakeLists.txt

    find_package (rrglib REQUIRED)
    include_directories (~/opt/include/rrglib)

    add_executable (rrgstar_double_integrator rrgstar_doubleinteg.cpp)
    target_link_libraries (rrgstar_double_integrator kdtree incmumc)

where incmumc is part of rrglib.  kdtree is not a part of rrglib (it originates
elsewhere) but is shipped with rrglib for convenience.


Visualizations, interfaces
--------------------------

Note that visualization is not intended to be the main purpose of this
repository, so Mayavi, Matplotlib, etc. as used in tools/rrgplot.py are regarded
as optional dependencies.  Similarly, building the various interfaces may
require external resources to be available, like ROS.  However, those
dependencies are only necessary if you want to build that particular interface.

* [libbot](https://code.google.com/p/libbot/)
* [LCM](https://lcm-proj.github.io/)


Authors
-------

Scott C. Livingston (Caltech),

and Sertac Karaman and Emilio Frazzoli (MIT), who wrote the code on which the
present work is based and as available from <https://svn.csail.mit.edu/smp>,
which has last changed date of 2011-08-02 05:25:23 -0400. The rrgstar repository
began by importing from there.


Licenses
--------

This is free software released under the terms of [the MIT License]
(http://opensource.org/licenses/MIT).  There is no warranty; not even for
merchantability or fitness for a particular purpose.  Consult LICENSE for
copying conditions.

Copies of several third-party free, open-source softwares are included under the
directory src/external_libraries/.  References to original locations and
licenses can be found there.
