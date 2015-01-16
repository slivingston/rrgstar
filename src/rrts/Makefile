CFLAGS_STD   := -g -std=gnu99 \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter 
CXXFLAGS_STD := -g \
	-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE \
	-Wall -Wno-unused-parameter -Wno-sign-compare -D__STDC_FORMAT_MACROS
LDFLAGS_STD  := -lm



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



CFLAGS = $(CFLAGS_STD) 

CXXFLAGS = $(CXXFLAGS_STD) 


LDFLAGS = $(LDFLAGS_STD) 

RRTS_OBJS = 	rrts_main_dubins.o \
		kdtree.o


RRTS=./rrts
RRTS_DEPENDENCIES:=$(RRTS_OBJS) 


all: $(RRTS) 


$(RRTS): $(RRTS_DEPENDENCIES)
	$(CXX) -g -o $@ $(RRTS_OBJS) $(LDFLAGS) $(CXXFLAGS)

clean:
	rm -f *.o *.a *~ $(OPTTREE) $(OPTSYSTEM_TEST) $(RRTS)
