OBJS= 
APPS= gfs-carmen

LIBS+= -lcarmenwrapper -lgridfastslam -lconfigfile
CPPFLAGS+= -I ../sensor -I$(CARMEN_HOME)/include 

-include ../global.mk
ifeq ($(CARMENSUPPORT), 0)
APPS=
.PHONY: clean all

all:

clean:

else
	-include ../build_tools/Makefile.app
endif



