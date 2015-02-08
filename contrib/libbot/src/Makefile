SUBDIRS=lcmtypes common viewer

.PHONY: all clean 

all:
	@for dir in $(SUBDIRS) ; do \
	echo $$dir ; $(MAKE) $(SILENT) -C $$dir all || exit 2; done

clean:
	for dir in $(SUBDIRS); do \
	echo $$dir ; $(MAKE) -C $$dir clean || exit 2; done
	rm -f *~
