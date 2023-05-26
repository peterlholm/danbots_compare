# makefile for compare_pcl

help:
	@echo "make install to install in /usr/local/bin"

install:
	cp info3d.py /usr/local/bin
	chmod a+x /usr/local/bin7info3d.py
	cp compare_pcl.py /usr/local/bin/compare3d
	chmod a+x /usr/local/bin/compare3d
	