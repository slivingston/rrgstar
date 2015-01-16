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

where you can find all the examples. Note that those under examples/libbot/
depend on the external library `libbot`. Type `make` in the folder that contains
the particular example you would like to execute. This should compile the part
of the library required by that example, and the resulting binary will be placed
in the bin/ directory at the root of the source tree.


Compiling libbot extensions
---------------------------

To compile the libbot extensions, go into the libbot directory and type `make`.
Dependencies:

* [libbot](https://code.google.com/p/libbot/)
* [LCM](https://lcm-proj.github.io/)


Compiling the documentation
---------------------------

    make doc

requires [Doxygen](http://www.doxygen.org/) to be installed.
