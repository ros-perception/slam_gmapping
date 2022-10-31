-include ./global.mk

ifeq ($(CARMENSUPPORT),1)
SUBDIRS=utils  sensor log configfile scanmatcher carmenwrapper gridfastslam gui gfs-carmen 
else
ifeq ($(MACOSX),1)
SUBDIRS=utils sensor log configfile scanmatcher gridfastslam 
else
SUBDIRS=utils sensor log configfile scanmatcher gridfastslam gui 
endif
endif

LDFLAGS+=
CPPFLAGS+= -I../sensor

-include ./build_tools/Makefile.subdirs

