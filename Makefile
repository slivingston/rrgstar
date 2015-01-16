all:
	@mkdir -p bin
	$(MAKE) -C examples/standalone

.PHONY: doc
doc:
	doxygen doc_config/smp.doxyconf

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;
	-find ./ -name \*~ -exec rm {} \;
	-find ./ -name \*# -exec rm {} \;

distclean: clean
	-rm -rf bin doc
