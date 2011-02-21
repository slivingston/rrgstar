#-----------------------------------------------------------------------------
# PATHS  -- the user may need to modify the following paths appropriately.
#-----------------------------------------------------------------------------

# 1. Usual lib directory
LIBDIR = /usr/local/lib

# 2. Directory where the smp trunk/ is located
SMP_ROOT_PATH = $(shell pwd)/../../../../

#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------




#-----------------------------------------------------------------------------
# Standard
#-----------------------------------------------------------------------------

# Compiler and linker
CC := gcc
CXX := g++
LDXX := g++


# Standard flags
CXXFLAGS_STD := -g \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter -Wno-sign-compare -D__STDC_FORMAT_MACROS 

LDFLAGS_STD = -lm -L $(LIBDIR)




#-----------------------------------------------------------------------------
# SMP
#-----------------------------------------------------------------------------

# SMP paths
SMP_SRC_PATH  = $(SMP_ROOT_PATH)/src
SMP_BIN_PATH  = $(SMP_ROOT_PATH)/bin
SMP_LIB_PATH  = $(SMP_ROOT_PATH)/lib

# SMP flags
CXXFLAGS_SMP := -I$(SMP_SRC_PATH)
LDFALGS_SMP := 




#-----------------------------------------------------------------------------
# Flags
#-----------------------------------------------------------------------------

CXXFLAGS = $(CXXFLAGS_STD) $(CXXFLAGS_SMP) 

LDFLAGS	= $(LDFLAGS_STD) $(LDFLAGS_SMP) 



#-----------------------------------------------------------------------------
# Objects
#-----------------------------------------------------------------------------

%.o: %.c
	$(CC) -o $@ -c $(CXXFLAGS) $<

%.o: %.C
	$(CXX) -o $@ -c $(CXXFLAGS) $<

%.o: %.cpp %.h
	$(CXX) -o $@ -c $(CXXFLAGS) $<



#-----------------------------------------------------------------------------
# Program
#-----------------------------------------------------------------------------

SMP = $(SMP_BIN_PATH)/example_standalone_rrt_single_integrator

SMP_OBJ_COMPILE = standalone_rrt_single_integrator.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 

SMP_OBJ = standalone_rrt_single_integrator.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 




#-----------------------------------------------------------------------------
# Make the program
#-----------------------------------------------------------------------------

all: 
	$(MAKE) targets

default: 
	$(MAKE) targets	

targets:  $(SMP) 


$(SMP): $(SMP_OBJ) 
	$(LDXX) $(SMP_OBJ_COMPILE) -o $(SMP) $(LDFLAGS) 




#-----------------------------------------------------------------------------
# Cleaning
#-----------------------------------------------------------------------------

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;
