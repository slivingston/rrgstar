lcm_types:=$(shell ls ../lcmtypes/lcmtypes_*.lcm)
lcm_basenames:=$(basename $(notdir $(lcm_types)))
lcm_py:=$(lcm_basenames:lcmtypes_%=src/lcmtypes/%.py)

botlcm_types:=$(shell ls /usr/local/share/libbot/lcmtypes/*.lcm)
botlcm_basenames:=$(basename $(notdir $(botlcm_types)))
botlcm_py:=$(botlcm_basenames:botlcm_%=src/botlcm/%.py)

lcm_py:=$(lcm_py) $(botlcm_py)

pythonpath:=$(shell pwd)/src
scripts:=smp-pose-stub 
script_targets:=$(scripts:%=../bin/%)

all: $(lcm_py) $(script_targets)

src/botlcm/%.py: /usr/local/share/libbot/lcmtypes/botlcm_%.lcm
	lcm-gen $^ -p --lazy --ppath=src

src/lcmtypes/%.py: ../lcmtypes/lcmtypes_%.lcm
	lcm-gen $^ -p --lazy --ppath=src

../bin/smp-pose-stub:
	@echo "    [$@]"
	@./make-bin-script $@ $(pythonpath) pose_stub

clean:
	rm -f $(lcm_py) $(lcm_py:.py=.pyc) \
		$(script_targets)
	rm -f src/botlcm/__init__.py
	rm -f src/botlcm/__init__.pyc
	rm -f src/lcmtypes/__init__.py
	rm -f src/lcmtypes/__init__.pyc
