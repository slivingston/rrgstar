#-----------------------------------------------------------------------------
# PATHS  -- the user may need to modify the following paths appropriately.
#-----------------------------------------------------------------------------

# 1. Usual lib directory
LIBDIR = /usr/local/lib

# 2. Directory where the smp trunk/ is located
SRC_ROOT_PATH = $(shell pwd)/../../../

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

LDFLAGS_STD = -lm -L$(LIBDIR)




#-----------------------------------------------------------------------------
# SMP
#-----------------------------------------------------------------------------

# SMP paths
SRC_PATH  = $(SRC_ROOT_PATH)/src
BIN_PATH  = $(SRC_ROOT_PATH)/bin
LIB_PATH  = $(SRC_ROOT_PATH)/lib

# SMP flags
CXXFLAGS_SMP := -I$(SRC_PATH)
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
