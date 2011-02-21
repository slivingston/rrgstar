include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_LCM) -I.. -O4 -fPIC \
		 -I$(BOT_INCLUDE_DIR)/bot/lcmtypes

lcmtype_dir = ../../lcmtypes
lcm_files = $(shell ls $(lcmtype_dir)/*.lcm)
LCMTYPES = $(lcm_files:$(lcmtype_dir)/%.lcm=%)

## prevent auto-generated lcm .c/.h files from being deleted
.SECONDARY : $(LCMTYPES:%=%.c) $(LCMTYPES:%=%.h)

LCMTYPES_O=$(LCMTYPES:%=%.o)
LCMTYPES_H=$(LCMTYPES:%=%.h)

LCMGEN = lcm-gen
LIBLCMTYPES = $(LIB_PATH)/liblcmtypes.a

all: $(LIBLCMTYPES) lcmtypes.h

lcmtypes.h: Makefile
	@echo >  $@ "#ifndef __lcmtypes_h__"
	@echo >> $@ "#define __lcmtypes_h__"
	@for f in $(LCMTYPES:%=%.h); do \
	    echo >> $@ "#include <lcmtypes/$$f>" || exit 2; done
	@echo >> $@ "#endif"

$(LIBLCMTYPES): $(LCMTYPES_O)
	ar rc $@ $(LCMTYPES_O)

%.o: %.c %.h
%.o: %.c
%.o: %.cpp

%.o: %.c $(LCMTYPES_H)
	@echo "    [$@]"
	gcc $(CFLAGS) -c $< 

%.c %.h: ../../lcmtypes/%.lcm
	@echo "    [$@]"
	$(LCMGEN) -c $<

clean:
	rm -f *.c *.h *.o *~ $(LIBLCMTYPES)
