# makefile for compare_pcl

help:
	@echo "make install to install in /usr/local/bin"

install:
	cp info3d.py /usr/local/bin/info3d
	chmod a+x /usr/local/bin/info3d
	cp show3d.py /usr/local/bin/show3d
	chmod a+x /usr/local/bin/show3d
	cp conv3d.py /usr/local/bin/conv3d
	chmod a+x /usr/local/bin/conv3d
	cp compare3d.py /usr/local/bin/compare3d
	chmod a+x /usr/local/bin/compare3d
	cp scale3d.py /usr/local/bin/scale3d
	chmod a+x /usr/local/bin/scale3d
	