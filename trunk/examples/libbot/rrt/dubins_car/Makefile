#-----------------------------------------------------------------------------
# PATHS  -- the user may need to modify the following paths appropriately.
#-----------------------------------------------------------------------------

# 1. Usual lib directory
LIBDIR = /usr/local/lib

# 2. Directory where the smp trunk/ is located
SMP_ROOT_PATH = $(shell pwd)/../../../../

# 3. Directory where smp libbot extension is located
LIBBOT_ROOT_PATH    = $(shell pwd)/../../../../../libbot

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
# Libbot
#-----------------------------------------------------------------------------

# Libbot paths
LIBBOT_SRC_PATH     = $(LIBBOT_ROOT_PATH)/src
LIBBOT_BIN_PATH     = $(LIBBOT_ROOT_PATH)/bin
LIBBOT_LIB_PATH     = $(LIBBOT_ROOT_PATH)/lib

# gtk
CFLAGS_GTK   :=`pkg-config --cflags gtk+-2.0`
LDFLAGS_GTK  :=`pkg-config --libs gtk+-2.0 gthread-2.0`

# Open GL
CFLAGS_GL    := 
LDFLAGS_GL   := -lGLU -lGLU -lglut

# LCM
CFLAGS_LCM  := `pkg-config --cflags lcm`
LDFLAGS_LCM := `pkg-config --libs lcm`

# libbot
CFLAGS_BOT := -I/usr/local/include/bot/lcmtypes
LDFLAGS_BOT_CORE := -lbot-core


# libbot common library
CFLAGS_COMMON  := -I$(LIBBOT_SRC_PATH) $(CFLAGS_BOT)
LDFLAGS_COMMON := $(LIBBOT_LIB_PATH)/libcommon.a $(LIBBOT_LIB_PATH)/liblcmtypes.a

CFLAGS_LIBBOT :=  $(CFLAGS_COMMON) $(CFLAGS_LCM) $(CFLAGS_GTK) \
		-frounding-math -Wno-deprecated -fpermissive
LDFLAGS_LIBBOT := $(LDFLAGS_COMMON) $(LDFLAGS_LCM) $(LDFLAGS_GTK) $(LDFLAGS_GL) $(LDFLAGS_BOT_CORE)\
		-lblas -lbot-gl -lbot-viewer -lbot-gtk -g 




#-----------------------------------------------------------------------------
# Flags
#-----------------------------------------------------------------------------

CXXFLAGS = $(CXXFLAGS_STD) $(CXXFLAGS_SMP) $(CFLAGS_LIBBOT)

LDFLAGS	= $(LDFLAGS_STD) $(LDFLAGS_SMP) $(LDFLAGS_LIBBOT)



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

SMP = $(SMP_BIN_PATH)/example_libbot_rrt_dubins_car

SMP_OBJ_COMPILE = libbot_rrt_dubins_car.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 

SMP_OBJ = libbot_rrt_dubins_car.o \
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
