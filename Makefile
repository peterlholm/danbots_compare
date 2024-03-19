# makefile for compare_pcl

VERSION=1.1.0-2
PKG_NAME=danbots-3d-tools-$(VERSION)
PKG_FOLDER=tmp/package

help:
	@echo "make install to install in /usr/local/bin"
	@echo "make deb-pkg to make an deb installation packet"

install:
	chmod a+x prg/*.py
	cp prg/compare3d.py /usr/local/bin/compare3d
	cp prg/conv3d.py /usr/local/bin/conv3d
	cp prg/info3d.py /usr/local/bin/info3d
	cp prg/mirror3d.py /usr/local/bin/mirror3d
	cp prg/show3d.py /usr/local/bin/show3d
	cp prg/trans3d.py /usr/local/bin/trans3d
	cp prg/stitch3d.py /usr/local/bin/stitch3d
	cp prg/crop3d.py /usr/local/bin/crop3d

uninstall:
	rm -f /usr/local/bin/compare3d /usr/local/bin/conv3d /usr/local/bin/info3d /usr/local/bin/mirror3d /usr/local/bin/show3d 
	rm -f /usr/local/bin/trans3d /usr/local/bin/stitch3d /usr/local/bin/crop3d 

copy-pkg:
	mkdir -p $(PKG_FOLDER)
	mkdir -p $(PKG_FOLDER)/usr/local/bin
	@echo Copy files
	cp prg/compare3d.py $(PKG_FOLDER)/usr/local/bin/compare3d
	cp prg/conv3d.py $(PKG_FOLDER)/usr/local/bin/conv3d
	cp prg/info3d.py $(PKG_FOLDER)/usr/local/bin/info3d
	cp prg/mirror3d.py $(PKG_FOLDER)/usr/local/bin/mirror3d
	cp prg/show3d.py $(PKG_FOLDER)/usr/local/bin/show3d
	cp prg/trans3d.py $(PKG_FOLDER)/usr/local/bin/trans3d
	cp prg/stitch3d.py $(PKG_FOLDER)/usr/local/bin/stitch3d
	cp prg/crop3d.py $(PKG_FOLDER)/usr/local/bin/crop3d

deb-pkg:	copy-pkg
	mkdir -p $(PKG_FOLDER)/DEBIAN
	cp deb_pkg/DEBIAN/* $(PKG_FOLDER)/DEBIAN
	dpkg-deb --build --root-owner-group $(PKG_FOLDER) tmp/$(PKG_NAME).deb


pkg-push:
	rcp tmp/$(PKG_NAME).deb  danbots:/var/www/apt/simple/pool/tools/
	rsh danbots /var/www/apt/simple/scan

clean:
	rm -rf $(PKG_FOLDER)
	rm -f $(PKG_FOLDER).deb
	rm -r tmp
	rm -f out.ply

