all:
	@mkdir -p bin
	$(MAKE) -C examples/standalone

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;

clean_tmp:
	-find ./ -name \*~ -exec rm {} \;
	-find ./ -name \*# -exec rm {} \;
