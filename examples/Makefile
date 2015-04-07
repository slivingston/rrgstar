SUBDIRS:= rrg/single_integrator_mu_calculus rrg/dubins_car rrg/double_integrator rrg/dubins_double_integrator_airplane \
	rrgstar/single_integrator_mu_calculus rrgstar/dubins_car

.PHONY: all clean 
.SILENT:

all: 
	echo [Make all]
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir || exit 2; done

clean:
	echo [Clean]
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir clean || exit 2; done
	rm -f *~ lib/*
