Compiling examples
------------------

The library consists of .h and .hpp header files.  It does not require
compilation a priori. To compile the example code, go into the following folder:

    /trunk/examples/

where you can find all the examples. Type `make` in the folder that contains the
particular example you would like to execute. This should compile the part of
the library required by that example.


Compiling libbot extensions
---------------------------

To compile the libbot extensions, go into the /libbot directory and type make.


Compiling the documentation
---------------------------

Make sure that you have doxygen installed. Go into the /trunk folder and type
the following:

    doxygen ./doc_config/smp.doxyconf
