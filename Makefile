# makefile for compare_pcl


help:
	@echo "make install to install in /usr/local/bin"

install:
	cp compare_pcl.py /usr/local/bin/compare3d
	chmod a+x /usr/local/bin/compare3d
	