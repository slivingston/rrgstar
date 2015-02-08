############################################################
# 
# This file should only contain CFLAGS_XXX and LDFLAGS_XXX directives.
# CFLAGS and LDFLAGS themselves should NOT be set: that is the job
# for the actual Makefiles (which will combine the flags given here)
#
# *** DO NOT SET CFLAGS or LDFLAGS  ***
#
# Our recommended flags for all projects.

# -Wno-format-zero-length: permit printf("");
# -Wno-unused-parameter: permit a function to ignore an argument
CFLAGS_STD   := -g -std=gnu99 \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter 
CXXFLAGS_STD := -g \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter -Wno-sign-compare -D__STDC_FORMAT_MACROS
LDFLAGS_STD  := -lm

ROOT_PATH    = $(shell pwd)/../..
SRC_PATH     = $(ROOT_PATH)/src
BIN_PATH     = $(ROOT_PATH)/bin
LIB_PATH     = $(ROOT_PATH)/lib
CONFIG_DIR   = $(ROOT_PATH)/config
CAMUNITS_DIR   = $(BIN_PATH)/camunits

CC           := gcc
LD           := gcc
CXX          := g++
LDXX         := g++

# dynamic libraries
ifeq "$(shell uname -s)" "Darwin"
	LDSH := -dynamic
	SHEXT := .dylib
	WHOLE_ARCHIVE_START := -all_load
else
	LD := gcc
	LDSH := -shared
	SHEXT := .so
	WHOLE_ARCHIVE_START := -Wl,-whole-archive
	WHOLE_ARCHIVE_STOP := -Wl,-no-whole-archive
endif

############################################################
#
# External libraries
#
# List these in roughly the order of dependency; those with fewest
# dependencies first. Within each LDFLAGS, list the dependencies in in
# decreasing order (e.g., end with LDFLAGS_GLIB)
#
############################################################

# glib
CFLAGS_GLIB  := `pkg-config --cflags glib-2.0 gmodule-2.0`
LDFLAGS_GLIB := `pkg-config --libs glib-2.0 gmodule-2.0 gthread-2.0 gobject-2.0`

# jpeg
LDFLAGS_JPEG := -ljpeg

# gtk
CFLAGS_GTK   :=`pkg-config --cflags gtk+-2.0`
LDFLAGS_GTK  :=`pkg-config --libs gtk+-2.0 gthread-2.0`

# Open GL
CFLAGS_GL    := 
LDFLAGS_GL   := -lGLU -lGLU -lglut

# LCM
CFLAGS_LCM  := `pkg-config --cflags lcm`
LDFLAGS_LCM := `pkg-config --libs lcm`

# camunits
CFLAGS_CAMUNITS  := `pkg-config --cflags camunits camunits-gtk`
LDFLAGS_CAMUNITS := `pkg-config --libs camunits camunits-gtk`

# libbot
CFLAGS_BOT := -I/usr/local/include/bot/lcmtypes
LDFLAGS_BOT_CORE := -lbot-core

# common library
CFLAGS_COMMON  := -I$(SRC_PATH) $(CFLAGS_BOT)
LDFLAGS_COMMON := $(LIB_PATH)/libcommon.a $(LIB_PATH)/liblcmtypes.a

# lcmtypes
CFLAGS_LCMTYPES :=
LDFLAGS_LCMTYPES :=

%.o: %.c %.h
	@echo "    [$@]"
	$(CC) $(CFLAGS) -c $< 

%.o: %.c
	@echo "    [$@]"
	$(CC) $(CFLAGS) -c $< 

%.o: %.cpp %.h
	@echo "    [$@]"
	$(CXX) -c -o $@ $< $(CXXFLAGS)

%.o: %.cpp
	@echo "    [$@]"
	$(CXX) -c -o $@ $< $(CXXFLAGS)
