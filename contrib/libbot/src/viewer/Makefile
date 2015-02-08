include ../common.mk

CFLAGS = $(CFLAGS_STD) \
		 $(CFLAGS_COMMON) \
		 $(CFLAGS_LCM) \
		 $(CFLAGS_GTK)

CXXFLAGS = $(CXXFLAGS_STD) \
		 $(CFLAGS_COMMON) \
		 $(CFLAGS_LCM) \
		 $(CFLAGS_GTK)

LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCM) $(LDFLAGS_GTK) \
		  $(LDFLAGS_GL) $(LDFLAGS_BOT_CORE) \
		  -lbot-gl -lbot-viewer -lbot-gtk

VIEWER_OBJS = 	main.o \
		udp_util.o \
		renderer_smp.o \
		renderer_grid.o \
		renderer_lcmgl.o \
		renderer_car.o

VIEWER=$(BIN_PATH)/smp-viewer
VIEWER_DEPENDENCIES:=$(VIEWER_OBJS) $(LDFLAGS_COMMON)

all: $(VIEWER)

$(VIEWER): $(VIEWER_DEPENDENCIES)
	$(CC) -o $@ $(VIEWER_OBJS) $(LDFLAGS)

clean:
	rm -f *.o *.a *~ $(VIEWER)
