OBJS= configuration.o carmenconfiguration.o sensorlog.o sensorstream.o
APPS= log_test log_plot scanstudio2carmen rdk2carmen

LDFLAGS+=  -lsensor_range -lsensor_odometry -lsensor_base 
CPPFLAGS+= -I../sensor 

-include ../global.mk
-include ../build_tools/Makefile.generic-shared-object

