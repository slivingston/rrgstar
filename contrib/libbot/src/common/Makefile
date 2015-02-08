include ../common.mk

CFLAGS = $(CFLAGS_COMMON) $(CFLAGS_STD) $(CFLAGS_GLIB) \
		 $(CFLAGS_LCM) $(CFLAGS_BOT_CORE) \
		 $(CFLAGS_LCMTYPES) \
		-fPIC 

LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_BOT_CORE) \
		  $(LDFLAGS_LCM) $(LDFLAGS_COMMON) 

COMMON_OBJS = globals.o \
			  geometry.o \
			  convexhull.o

LIB_COMMON = $(LIB_PATH)/libcommon.a

all: $(LIB_COMMON)

$(LIB_COMMON): $(COMMON_OBJS)
	ar rc $@ $(COMMON_OBJS)

clean:
	rm -f *.o *~ *.a $(LIB_COMMON)
