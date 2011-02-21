lcm_types:=$(shell ls ../lcmtypes/*.lcm)
lcm_basenames:=$(basename $(notdir $(lcm_types)))
lcm_java:=$(lcm_basenames:lcmtypes_%=src/lcmtypes/%.java)

CLASSPATH_LCM:=`pkg-config --variable=classpath lcm-java`
CLASSPATH_BOT:="/usr/local/share/java/bot.jar"
CLASSPATH:=$(shell pwd)/src:$(CLASSPATH_LCM)

smp_spy=../bin/smp-spy

all: $(lcm_java) $(smp_spy)
	CLASSPATH=$(CLASSPATH):$(CLASSPATH_BOT) ant

src/lcmtypes/%.java: ../lcmtypes/lcmtypes_%.lcm
	lcm-gen $^ -j --jpath=src --lazy

$(smp_spy):
	@echo > $@ "#!/bin/bash"
	@echo >> $@ "CLASSPATH=\"$(CLASSPATH_LCM):$(CLASSPATH_BOT):$(shell pwd)/example.jar\" lcm-spy"
	chmod 755 $@

clean:
	rm -f $(lcm_java) $(smp_spy)
	-rmdir src/lcmtypes
	ant clean
