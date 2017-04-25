INCPATH		= /usr/local/include


all:
	make -C lib $@
	make -C cmd $@

clean:
	make -C lib $@
	make -C cmd $@

install:
	make -C lib $@
	make -C cmd $@
	cp -RL include/trackcam $(INCPATH)

uninstall:
	make -C lib $@
	make -C cmd $@
	rm -rf $(INCPATH)/trackcam