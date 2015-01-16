Compiling examples
------------------

The library consists of .h and .hpp header files.  It does not require
compilation a priori. To compile the example code, go into the following folder:

    examples/

where you can find all the examples. Type `make` in the folder that contains the
particular example you would like to execute. This should compile the part of
the library required by that example. Alternatively, to build all of the
"standalone" examples (i.e., those not depending on libbot),

    make all


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
