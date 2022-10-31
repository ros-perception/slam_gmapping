#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <istream>
#include <gmapping/sensor/sensor_base/sensor.h>
#include <gmapping/log/log_export.h>

namespace GMapping {

class LOG_EXPORT Configuration{
	public:
		virtual ~Configuration();
		virtual SensorMap computeSensorMap() const=0;
};

};
#endif

