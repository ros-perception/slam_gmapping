#include "gmapping/sensor/sensor_base/sensorreading.h"

namespace GMapping{

SensorReading::SensorReading(const Sensor* s, double t){
	m_sensor=s;
	m_time=t;
}


SensorReading::~SensorReading(){
}

};

