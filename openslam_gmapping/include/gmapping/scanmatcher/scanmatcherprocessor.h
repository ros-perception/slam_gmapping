#ifndef SCANMATCHERPROCESSOR_H
#define SCANMATCHERPROCESSOR_H

#include <gmapping/log/sensorlog.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
//#include <gsl/gsl_eigen.h>
#include "gmapping/scanmatcher/scanmatcher.h"
#include <gmapping/scanmatcher/scanmatcher_export.h>

namespace GMapping {

class SCANMATCHER_EXPORT ScanMatcherProcessor{
	public:
  ScanMatcherProcessor(const ScanMatcherMap& m);
  ScanMatcherProcessor (double xmin, double ymin, double xmax, double ymax, double delta, double patchdelta);
		virtual ~ScanMatcherProcessor ();
		virtual void processScan(const RangeReading & reading);
		void setSensorMap(const SensorMap& smap, std::string sensorName="FLASER");
		void init();
		void setMatchingParameters
			(double min_range, double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, bool computeCovariance=false);
		void setRegistrationParameters(double regScore, double critScore);
		OrientedPoint getPose() const;
		inline const ScanMatcherMap& getMap() const {return m_map;}
		inline ScanMatcher& matcher() {return m_matcher;}
		inline void setmaxMove(double mmove){m_maxMove=mmove;}
		bool useICP;
	protected:
		ScanMatcher m_matcher;
		bool m_computeCovariance;
		bool m_first;
		SensorMap m_sensorMap;
		double m_regScore, m_critScore;
		unsigned int m_beams;
		double m_maxMove;
		//state
		ScanMatcherMap m_map;
		OrientedPoint m_pose;
		OrientedPoint m_odoPose;
		int  m_count;
		//gsl_eigen_symmv_workspace * m_eigenspace;
};

};

#endif


