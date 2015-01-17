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


Compiling libbot extensions
---------------------------

**Summary.** The extension and libbot examples are currently broken, due to
out-of-date dependencies.

Once the code is updated for current versions of dependencies, or once we
identify specific old versions required to build, then to compile the libbot
extensions, go into the libbot directory and type `make`.  Dependencies:

* [libbot](https://code.google.com/p/libbot/)
* [LCM](https://lcm-proj.github.io/)
