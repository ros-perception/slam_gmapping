#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include <map>
#include <gmapping/sensor/sensor_base/sensor_base_export.h>

namespace GMapping{

class SENSOR_BASE_EXPORT Sensor{
	public:
		Sensor(const std::string& name="");
		virtual ~Sensor();
		inline std::string getName() const {return m_name;}
		inline void setName(const std::string& name) {m_name=name;}
	protected:
		std::string m_name;
};

typedef std::map<std::string, Sensor*> SensorMap;

}; //end namespace

#endif

