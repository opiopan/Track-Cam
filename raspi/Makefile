INCPATH		= /usr/local/include
SHAREPATH	= /usr/local/share


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
	cp -RL service/trackcam $(SHAREPATH)
	cp service/trackcam.service /etc/systemd/system/
	systemctl daemon-reload
	systemctl enable trackcam.service

uninstall:
	make -C lib $@
	make -C cmd $@
	rm -rf $(INCPATH)/trackcam
	rm -rf $(SHAREPATH)/trackcam
	rm -f /etc/systemd/system/trackcam.service
	systemctl daemon-reload
