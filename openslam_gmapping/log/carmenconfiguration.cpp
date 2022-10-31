#include <cstdlib>
#include "gmapping/log/carmenconfiguration.h"
#include <iostream>
#include <sstream>
#include <assert.h>
#include <sys/types.h>
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>


#define LINEBUFFER_SIZE 10000

namespace GMapping {

using namespace std;

istream& CarmenConfiguration::load(istream& is){
	clear();
	char buf[LINEBUFFER_SIZE];
	bool laseron=false;
	bool rlaseron=false;
	bool rlaser1=false;
	bool rlaser2=false;

	string beams;
	string rbeams;

	while (is){
		is.getline(buf, LINEBUFFER_SIZE);
		istringstream lis(buf);
		
		string qualifier;
		string name;
		
		if (lis) 
			lis >> qualifier; 
		else 
			continue;
		//this is a workaround for carmen log files
		//the number lf laser beams should be specofoed in the config
		//part of the log
		if (qualifier=="FLASER"){
			laseron=true;
			lis >> beams;
		}
		if (qualifier=="RLASER"){
			rlaseron=true;
			lis >> rbeams;
		}
		if (qualifier=="ROBOTLASER1"){
			string laser_type, start_angle, field_of_view, angular_resolution, maximum_range, accuracy, remission_mode;
			lis >> laser_type>> start_angle>> field_of_view>> angular_resolution>> maximum_range>> accuracy>> remission_mode>> beams;
			rlaser1=true;
		}
		if (qualifier=="ROBOTLASER2"){
			string laser_type, start_angle, field_of_view, angular_resolution, maximum_range, accuracy, remission_mode;
			lis >> laser_type>> start_angle>> field_of_view>> angular_resolution>> maximum_range>> accuracy>> remission_mode>> rbeams;
			rlaser2=true;
		}
		if (qualifier!="PARAM") 
			continue;
		if (lis) 
			lis >> name; 
		else continue;
		

		vector<string> v;
		while (lis){
			string cparm;
			lis >> cparm;
			if (lis)
				v.push_back(cparm);
		}
		insert(make_pair(name, v));
	}
	if (laseron || rlaser1){
		vector<string> v;
		v.push_back(beams);
		insert(make_pair("laser_beams", v));
		cerr << "FRONT LASER BEAMS FROM LOG: " << beams << endl;
		v.clear();
		v.push_back("on");
		insert(make_pair("robot_use_laser", v));
	}
	if (rlaseron || rlaser2){
		vector<string> v;
		v.push_back(rbeams);
		insert(make_pair("rear_laser_beams", v));
		cerr << "REAR LASER BEAMS FROM LOG: " << beams << endl;
		v.clear();
		v.push_back("on");
		insert(make_pair("robot_use_rear_laser", v));
	}
	return is;
}

SensorMap CarmenConfiguration::computeSensorMap() const{
	//this boring stuff is for retrieving the parameters from the loaded tokens
	
	SensorMap smap;
	//odometry
	OdometrySensor* odometry=new OdometrySensor("ODOM");
	OdometrySensor* truepos=new OdometrySensor("TRUEPOS", true);
	
	smap.insert(make_pair(odometry->getName(), odometry));
	smap.insert(make_pair(truepos->getName(), truepos));
	//sonars
	const_iterator key=find("robot_use_sonar");
	if (key!=end() && key->second.front()=="on"){
		RangeSensor* sonar=new RangeSensor("SONAR");

		//the center of the sonar is the center of the base
		sonar->m_pose.x=sonar->m_pose.y=sonar->m_pose.theta=0;

		double maxrange=10.;
		key=find("robot_max_sonar");
		if (key!=end()){
			maxrange=atof(key->second.front().c_str());
			cerr << "max sonar:" << maxrange << endl;
		}

		unsigned int sonar_num=0;
		key=find("robot_num_sonars");
		if (key!=end()){
			sonar_num=atoi(key->second.front().c_str());
			cerr << "robot_num_sonars" << sonar_num << endl;
		}

		key=find("robot_sonar_offsets");
		if (key!=end()){
			const vector<string> & soff=key->second;

			if( (soff.size()/3<sonar_num)){
				cerr << __func__ << ": Error " << soff.size()
				<< " parameters for defining the sonar offsets"
				<< " while the specified number of sonars requires "
				<< sonar_num*3 << " parameters at least" << endl;
			} else {
				cerr << __func__ << ": Ok " << soff.size() << " parameters for defining the sonar offsets of " << sonar_num << " devices" << endl;
			}


			RangeSensor::Beam beam;

			for (unsigned int i=0; i<sonar_num*3; i+=3){
				beam.span=M_PI/180.*7.5;
				beam.pose.x=atof(soff[i].c_str());
				beam.pose.y=atof(soff[i+1].c_str());
				beam.pose.theta=atof(soff[i+2].c_str());
				beam.maxRange=maxrange;
				sonar->m_beams.push_back(beam);
				cerr << "beam_x" << beam.pose.x;
				cerr << " beam_y" << beam.pose.y;
				cerr << " beam_theta" << beam.pose.theta << endl;;
			}
		}
		sonar->updateBeamsLookup();
		smap.insert(make_pair(sonar->getName(), sonar));
	}
	
	//laser
	key=find("robot_use_laser");

	if (key!=end() && key->second.front()=="on"){
		RangeSensor* laser=new RangeSensor("FLASER");
		laser->newFormat=false;
		//by default the center of the robot is the center of the laser
		laser->m_pose.x=laser->m_pose.y=laser->m_pose.theta=0;
		key=find("robot_frontlaser_offset");
		if (key!=end()){
			laser->m_pose.x=atof(key->second.front().c_str());
			cerr << "FRONT OFFSET= " << laser->m_pose.x << endl;
		}
		
		
		
		RangeSensor::Beam beam;
			
		//double angle=-.5*M_PI;
		unsigned int beam_no=180;
	
		key=find("laser_beams");
		if (key!=end()){
			beam_no=atoi(key->second.front().c_str());
			cerr << "FRONT BEAMS="<< beam_no << endl;
		}
		
		double maxrange=50;
		double resolution=1.;
				
		
		if (beam_no==180 || beam_no==181)
		  resolution =1.;
		else if (beam_no==360 || beam_no==361)
		  resolution =.5;
		else if (beam_no==540 || beam_no==541)
		  resolution =.5;
		else if (beam_no==769) {
		  resolution =360./1024.;
		  maxrange = 4.1;
		}
		else if (beam_no==682) {
		  resolution =360./1024.;
		  maxrange = 4.1;
		}
		else if (beam_no==683) {
		  resolution =360./1024.;
		  maxrange = 5.5;
		}
		else {
			key=find("laser_front_laser_resolution");
			if (key!=end()){
				resolution=atof(key->second.front().c_str());
				cerr << "FRONT RES " << resolution << endl;
			}
		}
		
		laser->m_beams.resize(beam_no);
		double center_beam=(double)beam_no/2.;
		uint low_index=(uint)floor(center_beam);
		uint up_index=(uint)ceil(center_beam);
		double step=resolution*M_PI/180.;
		double angle=beam_no%2?0:step;
		unsigned int i=beam_no%2?0:1;
		for (; i<low_index+1; i++, angle+=step){
			beam.span=0;
			beam.pose.x=0;
			beam.pose.y=0;
			beam.s = 1;
			beam.c = 1;
			beam.pose.theta=-angle;
			beam.maxRange=maxrange;
			laser->m_beams[low_index-i]=beam;
			beam.pose.theta=angle;
			laser->m_beams[up_index+i-1]=beam;
		}
		laser->updateBeamsLookup();
		smap.insert(make_pair(laser->getName(), laser));
		cerr << "front beams " << beam_no << endl;
		cerr << "maxrange " << maxrange << endl;
	}


	key=find("robot_use_laser");
	if (key!=end() && key->second.front()=="on"){
		RangeSensor* laser=new RangeSensor("ROBOTLASER1");
		laser->newFormat=true;
		cerr << "ROBOTLASER1 inserted" << endl; 
		//by default the center of the robot is the center of the laser
		laser->m_pose.x=laser->m_pose.y=laser->m_pose.theta=0;
		key=find("robot_frontlaser_offset");
		if (key!=end()){
			laser->m_pose.x=atof(key->second.front().c_str());
			cerr << "FRONT OFFSET=" << laser->m_pose.x << endl;
		}
		
		RangeSensor::Beam beam;
			
		//double angle=-.5*M_PI;
		unsigned int beam_no=180;
	
		key=find("laser_beams");
		if (key!=end()){
			beam_no=atoi(key->second.front().c_str());
			cerr << "FRONT BEAMS="<< beam_no << endl;
		}
		
		double maxrange=50;
		double resolution=1.;
				
		
		if (beam_no==180 || beam_no==181)
		  resolution =1.;
		else if (beam_no==360 || beam_no==361)
		  resolution =.5;
		else if (beam_no==540 || beam_no==541)
		  resolution =.5;
		else if (beam_no==769)
		  resolution =360./1024.;
		else if (beam_no==683) {
		  resolution =360./1024.;
		   maxrange=5.50;
		}
		else {
			key=find("laser_front_laser_resolution");
			if (key!=end()){
				resolution=atof(key->second.front().c_str());
				cerr << "FRONT RES" << resolution << endl;
			}
		}
		
		laser->m_beams.resize(beam_no);
		double center_beam=(double)beam_no/2.;
		uint low_index=(uint)floor(center_beam);
		uint up_index=(uint)ceil(center_beam);
		double step=resolution*M_PI/180.;
		double angle=beam_no%2?0:step;
		unsigned int i=beam_no%2?0:1;
		for (; i<low_index+1; i++, angle+=step){
			beam.span=0;
			beam.pose.x=0;
			beam.pose.y=0;
			beam.s=0;
			beam.c=1;
			beam.pose.theta=-angle;
			beam.maxRange=maxrange;
			laser->m_beams[low_index-i]=beam;
			beam.pose.theta=angle;
			laser->m_beams[up_index+i-1]=beam;
		}
		laser->updateBeamsLookup();
		smap.insert(make_pair(laser->getName(), laser));
		cerr << "front beams" << beam_no << endl;
	}

	
	//vertical laser
	key=find("robot_use_rear_laser");

	if (key!=end() && key->second.front()=="on"){
		RangeSensor* laser=new RangeSensor("RLASER");
		
		//by default the center of the robot is the center of the laser
		laser->m_pose.x=laser->m_pose.y=laser->m_pose.theta=0;
		laser->m_pose.theta=M_PI;
		key=find("robot_rearlaser_offset");
		if (key!=end()){
			laser->m_pose.x=atof(key->second.front().c_str());
			cerr << "REAR OFFSET = " << laser->m_pose.x << endl;
		}
		
		
		
		RangeSensor::Beam beam;
			
		//double angle=-.5*M_PI;
		unsigned int beam_no=180;
	
		key=find("rear_laser_beams");
		if (key!=end()){
			beam_no=atoi(key->second.front().c_str());
			cerr << "REAR BEAMS="<< beam_no << endl;
		}
		
		double maxrange=89;
		double resolution=1.;
				
		
		if (beam_no==180 || beam_no==181)
			resolution =1.;
		else if (beam_no==360 || beam_no==361)
			resolution =.5;
		else if (beam_no==540 || beam_no==541)
		  resolution =.5;
		else if (beam_no==769)
			resolution =360./1024.;
		else {
			key=find("laser_rear_laser_resolution");
			if (key!=end()){
				resolution=atof(key->second.front().c_str());
				cerr << "REAR RES" << resolution << endl;
			}
		}
		
		laser->m_beams.resize(beam_no);
		double center_beam=(double)beam_no/2.;
		uint low_index=(uint)floor(center_beam);
		uint up_index=(uint)ceil(center_beam);
		double step=resolution*M_PI/180.;
		double angle=beam_no%2?0:step;
		unsigned int i=beam_no%2?0:1;
		for (; i<low_index+1; i++, angle+=step){
			beam.span=0;
			beam.pose.x=0;
			beam.pose.y=0;
			beam.s=0;
			beam.c=1;
			beam.pose.theta=-angle;
			beam.maxRange=maxrange;
			laser->m_beams[low_index-i]=beam;
			beam.pose.theta=angle;
			laser->m_beams[up_index+i-1]=beam;
		}
		laser->updateBeamsLookup();
		smap.insert(make_pair(laser->getName(), laser));
		cerr<< "rear beams" << beam_no << endl;
	}

	key=find("robot_use_rear_laser");
	if (key!=end() && key->second.front()=="on"){
		RangeSensor* laser=new RangeSensor("ROBOTLASER2");
		laser->newFormat=true;
		cerr << "ROBOTLASER2 inserted" << endl; 
		//by default the center of the robot is the center of the laser
		laser->m_pose.x=laser->m_pose.y=0;
		laser->m_pose.theta=M_PI;
		key=find("robot_rearlaser_offset");
		if (key!=end()){
			// laser->m_pose.x==atof(key->second.front().c_str());
			cerr << "REAR OFFSET not used" << laser->m_pose.x << endl;
		}
		
		RangeSensor::Beam beam;
			
		//double angle=-.5*M_PI;
		unsigned int beam_no=180;
	
		key=find("rear_laser_beams");
		if (key!=end()){
			beam_no=atoi(key->second.front().c_str());
			cerr << "REAR BEAMS="<< beam_no << endl;
		}
		
		double maxrange=50;
		double resolution=1.;
				
		
		if (beam_no==180 || beam_no==181)
			resolution =1.;
		else if (beam_no==360 || beam_no==361)
			resolution =.5;
		else if (beam_no==540 || beam_no==541)
		  resolution =.5;
		else if (beam_no==769)
			resolution =360./1024.;
		else {
			key=find("laser_rear_laser_resolution");
			if (key!=end()){
				resolution=atof(key->second.front().c_str());
				cerr << "REAR RES" << resolution << endl;
			}
		}
		
		laser->m_beams.resize(beam_no);
		double center_beam=(double)beam_no/2.;
		uint low_index=(uint)floor(center_beam);
		uint up_index=(uint)ceil(center_beam);
		double step=resolution*M_PI/180.;
		double angle=beam_no%2?0:step;
		unsigned int i=beam_no%2?0:1;
		for (; i<low_index+1; i++, angle+=step){
			beam.span=0;
			beam.s=0;
			beam.c=1;
			beam.pose.x=0;
			beam.pose.y=0;
			beam.pose.theta=-angle;
			beam.maxRange=maxrange;
			laser->m_beams[low_index-i]=beam;
			beam.pose.theta=angle;
			laser->m_beams[up_index+i-1]=beam;
		}
		laser->updateBeamsLookup();
		smap.insert(make_pair(laser->getName(), laser));
		cerr << "rear beams" << beam_no << endl;
	}


	return smap;
}

};

