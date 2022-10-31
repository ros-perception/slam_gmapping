#include "gmapping/log/sensorlog.h"

#include <iostream>
#include <sstream>
#include <assert.h>
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>

#define LINEBUFFER_SIZE 100000

namespace GMapping {

using namespace std;

SensorLog::SensorLog(const SensorMap& sm): m_sensorMap(sm){
}

SensorLog::~SensorLog(){
	for (iterator it=begin(); it!=end(); it++)
		if (*it) delete (*it);
}

istream& SensorLog::load(istream& is){
	for (iterator it=begin(); it!=end(); it++)
		if (*it) delete (*it);
	clear();
	
	char buf[LINEBUFFER_SIZE];
	while (is){
		is.getline(buf, LINEBUFFER_SIZE);
		istringstream lis(buf);
		
		string sensorname;
		
		if (lis) 
			lis >>sensorname; 
		else 
			continue;
		
		
			
		SensorMap::const_iterator it=m_sensorMap.find(sensorname);
		if (it==m_sensorMap.end()){
			continue;
		}
		
		Sensor* sensor=it->second;
		
		SensorReading* reading=0;
		OdometrySensor* odometry=dynamic_cast<OdometrySensor*>(sensor);
		if (odometry)
			reading=parseOdometry(lis, odometry);
		
		RangeSensor* range=dynamic_cast<RangeSensor*>(sensor);
		if (range)
			reading=parseRange(lis, range);
		if (reading)
			push_back(reading);
	}
	return is;
	
}

OdometryReading* SensorLog::parseOdometry(istream& is, const OdometrySensor* osen) const{
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
	return reading;
}

RangeReading* SensorLog::parseRange(istream& is, const RangeSensor* rs) const{
	if(rs->newFormat){
		string laser_type, start_angle, field_of_view, angular_resolution, maximum_range, accuracy, remission_mode;
		is >> laser_type>> start_angle>> field_of_view>> angular_resolution>> maximum_range>> accuracy >> remission_mode;
	}
	
	unsigned int size;
	is >> size;
	assert(size==rs->beams().size());
	
	RangeReading* reading=new RangeReading(rs);
	//cerr << "#R=" << size << endl;
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
	//FIXME XXX
	OrientedPoint laserPose;
	is >> laserPose.x >> laserPose.y >> laserPose.theta;
	OrientedPoint pose;
	is >> pose.x >> pose.y >> pose.theta; 
	reading->setPose(pose);
	double a,b,c;
	if (rs->newFormat){
		string laser_tv, laser_rv, forward_safety_dist, side_safty_dist, turn_axis;
		is >> laser_tv >> laser_rv >> forward_safety_dist >> side_safty_dist >> turn_axis;
	} else {
		is >> a >> b >> c;
	}
	string s;
	is >> a >> s;
	is >> a;
	reading->setTime(a);
	return reading;
}

OrientedPoint SensorLog::boundingBox(double& xmin, double& ymin, double& xmax, double& ymax) const {
	xmin=ymin=1e6;
	xmax=ymax=-1e6;
	bool first=true;
	OrientedPoint start;
	for (const_iterator it=begin(); it!=end(); it++){
		double lxmin=0., lxmax=0., lymin=0., lymax=0.;
		const SensorReading* reading=*it;
		const OdometryReading* odometry=dynamic_cast<const OdometryReading*> (reading);
		if (odometry){
			lxmin=lxmax=odometry->getPose().x;
			lymin=lymax=odometry->getPose().y;
		}
		
		const RangeReading* rangeReading=dynamic_cast<const RangeReading*> (reading);
		if (rangeReading){
			lxmin=lxmax=rangeReading->getPose().x;
			lymin=lymax=rangeReading->getPose().y;
			if (first){
				first=false;
				start=rangeReading->getPose();	
			}
		}
		xmin=xmin<lxmin?xmin:lxmin;
		xmax=xmax>lxmax?xmax:lxmax;
		ymin=ymin<lymin?lymin:lymin;
		ymax=ymax>lymax?ymax:lymax;
	}
	return start;
}

};

