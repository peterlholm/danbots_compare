# makefile for compare_pcl

help:
	@echo "make install to install in /usr/local/bin"

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
	