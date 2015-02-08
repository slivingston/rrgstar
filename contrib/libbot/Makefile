SUBDIRS:=src python java

.PHONY: all clean 
.SILENT:

all: 
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir || exit 2; done

clean:
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir clean || exit 2; done
	rm -f *~ lib/*
