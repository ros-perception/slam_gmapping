#ifndef SENSORLOG_H
#define SENSORLOG_H

#include <list>
#include <istream>
#include <gmapping/sensor/sensor_base/sensorreading.h>
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_odometry/odometryreading.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
#include "gmapping/log/configuration.h"
#include <gmapping/log/log_export.h>

namespace GMapping {

class LOG_EXPORT SensorLog : public std::list<SensorReading*>{
	public:
		SensorLog(const SensorMap&);
		~SensorLog();
		std::istream& load(std::istream& is);
		OrientedPoint boundingBox(double& xmin, double& ymin, double& xmax, double& ymax) const;
	protected:
		const SensorMap& m_sensorMap;
		OdometryReading* parseOdometry(std::istream& is, const OdometrySensor* ) const;
		RangeReading* parseRange(std::istream& is, const RangeSensor* ) const;
};

};

#endif
