#include <assert.h>
#include <sstream>
#include "gmapping/log/sensorstream.h"
//#define LINEBUFFER_SIZE 1000000 //for not Cyrill to unbless me, it is better to exagerate :-))
// Can't declare a buffer that big on the stack.  So we'll risk Cyrill's
// unblessing, and make it smaller.
#define LINEBUFFER_SIZE 8192

namespace GMapping {

using namespace std;

//SensorStream
SensorStream::SensorStream(const SensorMap& sensorMap) :m_sensorMap(sensorMap){}

SensorStream::~SensorStream(){}

SensorReading* SensorStream::parseReading(std::istream& is, const SensorMap& smap){
	SensorReading* reading=0;
	if (is){
		char buf[LINEBUFFER_SIZE];
		is.getline(buf, LINEBUFFER_SIZE);
		istringstream lis(buf);
		
		string sensorname;
		
		if (lis){
			lis >>sensorname; 
		} else 
			return 0;
		
		SensorMap::const_iterator it=smap.find(sensorname);
		if (it==smap.end()){
			return 0;
		}
		
		Sensor* sensor=it->second;
		
		OdometrySensor* odometry=dynamic_cast<OdometrySensor*>(sensor);
		if (odometry)
			reading=parseOdometry(lis, odometry);
		
		RangeSensor* range=dynamic_cast<RangeSensor*>(sensor);
		if (range)
			reading=parseRange(lis, range);
	}
	return reading;
}

OdometryReading* SensorStream::parseOdometry(std::istream& is, const OdometrySensor* osen ){
	OdometryReading* reading=new OdometryReading(osen);
	OrientedPoint pose;
	OrientedPoint speed;
	OrientedPoint accel;
	is >> pose.x >> pose.y >> pose.theta;
	is >> speed.x >>speed.theta;
	speed.y=0;
	is >> accel.x;
	accel.y=accel.theta=0;
	reading->setPose(pose); reading->setSpeed(speed); reading->setAcceleration(accel);
	double timestamp, reltimestamp;
	string s;
	is >> timestamp >>s >> reltimestamp;
	reading->setTime(timestamp);
	return reading;
}

RangeReading* SensorStream::parseRange(std::istream& is, const RangeSensor* rs){
	//cerr << __func__ << endl;
	if(rs->newFormat){
		string laser_type, start_angle, field_of_view, angular_resolution, maximum_range, accuracy, remission_mode;
		is >> laser_type>> start_angle>> field_of_view>> angular_resolution>> maximum_range>> accuracy>> remission_mode;
		//cerr << " New format laser msg" << endl;
	}
	unsigned int size;
	is >> size;
	assert(size==rs->beams().size());
	RangeReading* reading=new RangeReading(rs);
	reading->resize(size);
	for (unsigned int i=0; i<size; i++){
		is >> (*reading)[i];
	}
	if (rs->newFormat){
		int reflectionBeams;
		is >> reflectionBeams;
		double reflection;
		for (int i=0; i<reflectionBeams; i++)
			is >> reflection;
	}
	OrientedPoint laserPose;
	is >> laserPose.x >> laserPose.y >> laserPose.theta;
	OrientedPoint pose;
	is >> pose.x >> pose.y >> pose.theta;
	reading->setPose(pose);

	if (rs->newFormat){
		string laser_tv, laser_rv, forward_safety_dist, side_safty_dist, turn_axis;
		is >> laser_tv >> laser_rv >> forward_safety_dist >> side_safty_dist >> turn_axis;
	} 
// 	else {
// 	  double a,b,c;
// 		is >> a >> b >> c;
// 	}
	double timestamp, reltimestamp;
	string s;
	is >> timestamp >>s >> reltimestamp;
	reading->setTime(timestamp);
	return reading;

}

//LogSensorStream
LogSensorStream::LogSensorStream(const SensorMap& sensorMap, const SensorLog* log):
	SensorStream(sensorMap){
	m_log=log;
	assert(m_log);
	m_cursor=log->begin();
}

LogSensorStream::operator bool() const{
	return m_cursor==m_log->end();
}

bool LogSensorStream::rewind(){
	m_cursor=m_log->begin();
	return true;
}

SensorStream& LogSensorStream::operator >>(const SensorReading*& rd){
	rd=*m_cursor;
	m_cursor++;
	return *this;
}

//InputSensorStream
InputSensorStream::InputSensorStream(const SensorMap& sensorMap, std::istream& is):
	SensorStream(sensorMap), m_inputStream(is){
}

InputSensorStream::operator bool() const{
	return (bool) m_inputStream;
}

bool InputSensorStream::rewind(){
	//m_inputStream.rewind();
	return false;
}

SensorStream& InputSensorStream::operator >>(const SensorReading*& reading){
	reading=parseReading(m_inputStream, m_sensorMap);
	return *this;
}

};

