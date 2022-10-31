-include ../global.mk

OBJS= gsp_thread.o qparticleviewer.o qgraphpainter.o qmappainter.o 

APPS= gfs_nogui gfs_simplegui gfs2img
LDFLAGS+=  $(QT_LIB) $(KDE_LIB) -lgridfastslam -lscanmatcher -llog -lsensor_range -lsensor_odometry -lsensor_base -lconfigfile -lutils -lpthread

ifeq ($(CARMENSUPPORT),1)
LDFLAGS+= -lcarmenwrapper 
endif

CPPFLAGS+= -I../sensor $(QT_INCLUDE) $(KDE_INCLUDE) -I$(CARMEN_HOME)/include


-include ../build_tools/Makefile.generic-shared-object



