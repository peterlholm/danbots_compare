# makefile for compare_pcl

VERSION=1.0.1
PKG_FOLDER=deb_pkg/3d-tools-$(VERSION)
help:
	@echo "make install to install in /usr/local/bin"
	@echo "make deb-pkg to make an deb installation packet"

install:
	cp compare3d.py /usr/local/bin/compare3d
	chmod a+x /usr/local/bin/compare3d
	cp conv3d.py /usr/local/bin/conv3d
	chmod a+x /usr/local/bin/conv3d
	cp info3d.py /usr/local/bin/info3d
	chmod a+x /usr/local/bin/info3d
	cp mirror3d.py /usr/local/bin/mirror3d
	chmod a+x /usr/local/bin/mirror3d
	cp show3d.py /usr/local/bin/show3d
	chmod a+x /usr/local/bin/show3d
	cp trans3d.py /usr/local/bin/trans3d
	chmod a+x /usr/local/bin/trans3d
	cp stitch3d.py /usr/local/bin/stitch3d
	chmod a+x /usr/local/bin/stitch3d
	cp crop3d.py /usr/local/bin/crop3d
	chmod a+x /usr/local/bin/crop3d

copy-pkg:
	mkdir -p $(PKG_FOLDER)
	mkdir -p $(PKG_FOLDER)/usr/local/bin
	@echo Copy files
	cp compare3d.py $(PKG_FOLDER)/usr/local/bin/compare3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/compare3d
	cp conv3d.py $(PKG_FOLDER)/usr/local/bin/conv3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/conv3d
	cp info3d.py $(PKG_FOLDER)/usr/local/bin/info3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/info3d
	cp mirror3d.py $(PKG_FOLDER)/usr/local/bin/mirror3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/mirror3d
	cp show3d.py $(PKG_FOLDER)/usr/local/bin/show3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/show3d
	cp trans3d.py $(PKG_FOLDER)/usr/local/bin/trans3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/trans3d
	cp stitch3d.py $(PKG_FOLDER)/usr/local/bin/stitch3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/stitch3d
	cp crop3d.py $(PKG_FOLDER)/usr/local/bin/crop3d
	chmod a+x $(PKG_FOLDER)/usr/local/bin/crop3d

deb-pkg:	copy-pkg
	mkdir -p $(PKG_FOLDER)/DEBIAN
	cp deb_pkg/DEBIAN/* $(PKG_FOLDER)/DEBIAN
	dpkg-deb --build --root-owner-group $(PKG_FOLDER)

clean:
	rm -rf $(PKG_FOLDER)
	rm -f $(PKG_FOLDER).deb
	rm -f out.ply

