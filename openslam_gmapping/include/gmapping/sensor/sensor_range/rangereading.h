#ifndef RANGEREADING_H
#define RANGEREADING_H

#include <vector>
#include <gmapping/sensor/sensor_base/sensorreading.h>
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include <gmapping/sensor/sensor_range/sensor_range_export.h>

#ifdef _MSC_VER
namespace std {
	extern template class __declspec(dllexport) vector<double>;
};
#endif

namespace GMapping{

class SENSOR_RANGE_EXPORT RangeReading: public SensorReading, public std::vector<double>{
	public:
		RangeReading(const RangeSensor* rs, double time=0);
		RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time=0);
		virtual ~RangeReading();
		inline const OrientedPoint& getPose() const {return m_pose;}
		inline void setPose(const OrientedPoint& pose) {m_pose=pose;}
		unsigned int rawView(double* v, double density=0.) const;
		std::vector<Point> cartesianForm(double maxRange=1e6) const;
		unsigned int activeBeams(double density=0.) const;
	protected:
		OrientedPoint m_pose;
};

};

#endif
