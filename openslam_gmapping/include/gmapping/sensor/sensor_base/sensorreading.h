#ifndef SENSORREADING_H
#define SENSORREADING_H

#include "gmapping/sensor/sensor_base/sensor.h"
#include <gmapping/sensor/sensor_base/sensor_base_export.h>

namespace GMapping{

class SENSOR_BASE_EXPORT SensorReading{
	public:
		SensorReading(const Sensor* s=0, double time=0);
		virtual ~SensorReading();
		inline double getTime() const {return m_time;}
		inline void setTime(double t) {m_time=t;}
		inline const Sensor* getSensor() const {return m_sensor;}
	protected:
		double m_time;
		const Sensor* m_sensor;

};

}; //end namespace
#endif


