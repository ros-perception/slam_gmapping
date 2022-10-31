#ifndef CARMENCONFIGURATION_H
#define CARMENCONFIGURATION_H

#include <string>
#include <map>
#include <vector>
#include <istream>
#include <gmapping/sensor/sensor_base/sensor.h>
#include "gmapping/log/configuration.h"
#include <gmapping/log/log_export.h>

namespace GMapping {

class LOG_EXPORT CarmenConfiguration: public Configuration, public std::map<std::string, std::vector<std::string> >{
	public:
		virtual std::istream& load(std::istream& is);
		virtual SensorMap computeSensorMap() const;
};

};

#endif

