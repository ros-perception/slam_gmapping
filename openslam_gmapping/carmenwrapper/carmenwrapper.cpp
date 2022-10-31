/*****************************************************************
 *
 * This file is part of the GMAPPING project
 *
 * GMAPPING Copyright (c) 2004 Giorgio Grisetti, 
 * Cyrill Stachniss, and Wolfram Burgard
 *
 * This software is licensed under the 3-Clause BSD License
 * and is copyrighted by Giorgio Grisetti, Cyrill Stachniss, 
 * and Wolfram Burgard.
 * 
 * Further information on this license can be found at:
 * https://opensource.org/licenses/BSD-3-Clause
 * 
 * GMAPPING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/


#include "gmapping/carmenwrapper/carmenwrapper.h"

using namespace GMapping;
using namespace std;

//static vars for the carmenwrapper
SensorMap CarmenWrapper::m_sensorMap;
deque<RangeReading> CarmenWrapper::m_rangeDeque;
pthread_mutex_t CarmenWrapper::m_mutex;  
sem_t CarmenWrapper::m_dequeSem;  
pthread_mutex_t CarmenWrapper::m_lock;  
pthread_t CarmenWrapper::m_readingThread;
RangeSensor* CarmenWrapper::m_frontLaser=0;
RangeSensor* CarmenWrapper::m_rearLaser=0;
bool CarmenWrapper::m_threadRunning=false;
OrientedPoint CarmenWrapper::m_truepos;
bool CarmenWrapper::stopped=true;


void CarmenWrapper::initializeIPC(const char* name) {
  carmen_ipc_initialize(1,(char **)&name);
}




int CarmenWrapper::registerLocalizationMessages(){
  lock();
  IPC_RETURN_TYPE err;

  /* register globalpos message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_LOCALIZE_GLOBALPOS_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_GLOBALPOS_NAME);

  /* register robot particle message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_PARTICLE_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_LOCALIZE_PARTICLE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_PARTICLE_NAME);

/*
  carmen_localize_subscribe_initialize_placename_message(NULL, 
							 (carmen_handler_t)
							 carmen_localize_initialize_placename_handler,
							 CARMEN_SUBSCRIBE_LATEST);

  // register map request message 
  err = IPC_defineMsg(CARMEN_LOCALIZE_MAP_QUERY_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_MAP_QUERY_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_LOCALIZE_MAP_QUERY_NAME);

  err = IPC_defineMsg(CARMEN_LOCALIZE_MAP_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_MAP_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_MAP_NAME);

  // subscribe to map request message 
  err = IPC_subscribe(CARMEN_LOCALIZE_MAP_QUERY_NAME, map_query_handler, NULL);
  carmen_test_ipc(err, "Could not subscribe", CARMEN_LOCALIZE_MAP_QUERY_NAME);
  IPC_setMsgQueueLength(CARMEN_LOCALIZE_MAP_QUERY_NAME, 1);


  // register globalpos request message 
  err = IPC_defineMsg(CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME, 
		      IPC_VARIABLE_LENGTH,
		      CARMEN_DEFAULT_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		       CARMEN_LOCALIZE_MAP_QUERY_NAME);

  // subscribe to globalpos request message 
  err = IPC_subscribe(CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME, 
		      globalpos_query_handler, NULL);
  carmen_test_ipc(err, "Could not subscribe", 
		  CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME);
  IPC_setMsgQueueLength(CARMEN_LOCALIZE_GLOBALPOS_QUERY_NAME, 1);
*/
  unlock();
  return 0;
}

bool CarmenWrapper::start(const char* name){
	if (m_threadRunning)
		return false;
	carmen_robot_subscribe_frontlaser_message(NULL, (carmen_handler_t)robot_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_robot_subscribe_rearlaser_message(NULL, (carmen_handler_t)robot_rearlaser_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_simulator_subscribe_truepos_message(NULL,(carmen_handler_t) simulator_truepos_handler, CARMEN_SUBSCRIBE_LATEST);

	IPC_RETURN_TYPE err;

	err = IPC_subscribe(CARMEN_NAVIGATOR_GO_NAME, navigator_go_handler,  NULL);
	carmen_test_ipc_exit(err, "Could not subscribe", 
			     CARMEN_NAVIGATOR_GO_NAME);
	IPC_setMsgQueueLength(CARMEN_NAVIGATOR_GO_NAME, 1);

	err = IPC_subscribe(CARMEN_NAVIGATOR_STOP_NAME, navigator_stop_handler,  NULL);
	carmen_test_ipc_exit(err, "Could not subscribe", 
			     CARMEN_NAVIGATOR_STOP_NAME);
	IPC_setMsgQueueLength(CARMEN_NAVIGATOR_STOP_NAME, 1);



	signal(SIGINT, shutdown_module);
	pthread_mutex_init(&m_mutex, 0);
	pthread_mutex_init(&m_lock, 0);
	sem_init(&m_dequeSem, 0, 0);
	m_threadRunning=true;
	pthread_create (&m_readingThread,0,m_reading_function,0);
	return true; 
}

void CarmenWrapper::lock(){
	//cerr <<"LOCK" << endl;
	pthread_mutex_lock(&m_lock);
}

void CarmenWrapper::unlock(){
	//cerr <<"UNLOCK" << endl;
	pthread_mutex_unlock(&m_lock);
}


bool CarmenWrapper::sensorMapComputed(){
	pthread_mutex_lock(&m_mutex);
	bool smok=m_frontLaser;
	pthread_mutex_unlock(&m_mutex);
	return smok;
}

const SensorMap& CarmenWrapper::sensorMap(){
	return m_sensorMap;
}
 
bool CarmenWrapper::isRunning(){
	return m_threadRunning;
}

bool CarmenWrapper::isStopped(){
	return stopped;
}

int CarmenWrapper::queueLength(){
	int ql=0;
	pthread_mutex_lock(&m_mutex);
	ql=m_rangeDeque.size();
	pthread_mutex_unlock(&m_mutex);
	return ql;
}

OrientedPoint CarmenWrapper::getTruePos(){
	return m_truepos;
}

bool CarmenWrapper::getReading(RangeReading& reading){
	bool present=false;
	sem_wait(&m_dequeSem);
	pthread_mutex_lock(&m_mutex);
	if (!m_rangeDeque.empty()){
//		cerr << __func__ << ": queue size=" <<m_rangeDeque.size() << endl;
		reading=m_rangeDeque.front();
		m_rangeDeque.pop_front();
		present=true;
	}
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
//	cerr << "fetch. elements= "<< m_rangeDeque.size() << " sval=" << sval <<endl;
	pthread_mutex_unlock(&m_mutex);
	return present;
}

void CarmenWrapper::addReading(RangeReading& reading){
	pthread_mutex_lock(&m_mutex);
	m_rangeDeque.push_back(reading);
	pthread_mutex_unlock(&m_mutex);
	sem_post(&m_dequeSem);
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
//	cerr << "post. elements= "<< m_rangeDeque.size() << " sval=" << sval <<endl;
}


//RangeSensor::RangeSensor(std::string name, unsigned int beams_num, unsigned int res, const OrientedPoint& position, double span, double maxrange): 

void CarmenWrapper::robot_frontlaser_handler(carmen_robot_laser_message* frontlaser) {
/*	if (! m_rangeSensor){
		double res=0;
		if (frontlaser->num_readings==180 || frontlaser->num_readings==181)
			res=M_PI/180;
		if (frontlaser->num_readings==360 || frontlaser->num_readings==361)
			res=M_PI/360;
		assert(res>0);
		m_rangeSensor=new RangeSensor("FLASER",frontlaser->num_readings, res, OrientedPoint(0,0,0), 0, 89.9);
		m_sensorMap.insert(make_pair(string("FLASER"), m_rangeSensor));
		
		cout << __func__ 
		     << ": FrontLaser configured." 
		     << " Readings " << m_rangeSensor->beams().size() 
		     << " Resolution " << res << endl;
	}
	
	RangeReading reading(m_rangeSensor, frontlaser->timestamp);
	reading.resize(m_rangeSensor->beams().size());
	for (unsigned int i=0; i< (unsigned int)frontlaser->num_readings; i++){
		reading[i]=(double)frontlaser->range[i];
	}
	reading.setPose(OrientedPoint(frontlaser->x, frontlaser->y, frontlaser->theta));
*/	
	RangeReading reading=carmen2reading(*frontlaser);
	addReading(reading);
}

void CarmenWrapper::robot_rearlaser_handler(carmen_robot_laser_message* rearlaser) {
/*	if (! m_rangeSensor){
		double res=0;
		if (frontlaser->num_readings==180 || frontlaser->num_readings==181)
			res=M_PI/180;
		if (frontlaser->num_readings==360 || frontlaser->num_readings==361)
			res=M_PI/360;
		assert(res>0);
		m_rangeSensor=new RangeSensor("FLASER",frontlaser->num_readings, res, OrientedPoint(0,0,0), 0, 89.9);
		m_sensorMap.insert(make_pair(string("FLASER"), m_rangeSensor));
		
		cout << __func__ 
		     << ": FrontLaser configured." 
		     << " Readings " << m_rangeSensor->beams().size() 
		     << " Resolution " << res << endl;
	}
	
	RangeReading reading(m_rangeSensor, frontlaser->timestamp);
	reading.resize(m_rangeSensor->beams().size());
	for (unsigned int i=0; i< (unsigned int)frontlaser->num_readings; i++){
		reading[i]=(double)frontlaser->range[i];
	}
	reading.setPose(OrientedPoint(frontlaser->x, frontlaser->y, frontlaser->theta));
*/	
	RangeReading reading=carmen2reading(*rearlaser);
	addReading(reading);
}




void CarmenWrapper:: navigator_go_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  carmen_navigator_go_message msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_navigator_go_message));  
  IPC_freeByteArray(callData);

  carmen_test_ipc_return
    (err, "Could not unmarshall", IPC_msgInstanceName(msgRef));
  cerr<<"go"<<endl;
  stopped=false;
}


void CarmenWrapper:: navigator_stop_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  carmen_navigator_stop_message msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_navigator_stop_message));  
  IPC_freeByteArray(callData);

  carmen_test_ipc_return
    (err, "Could not unmarshall", IPC_msgInstanceName(msgRef));
  cerr<<"stop"<<endl;
  stopped=true;
}



void CarmenWrapper::simulator_truepos_handler(carmen_simulator_truepos_message* truepos){
	m_truepos.x=truepos->truepose.x;
	m_truepos.y=truepos->truepose.y;
	m_truepos.theta=truepos->truepose.theta;
}

RangeReading CarmenWrapper::carmen2reading(const carmen_robot_laser_message& msg){
	//either front laser or rear laser
	double dth=msg.laser_pose.theta-msg.robot_pose.theta;
	dth=atan2(sin(dth), cos(dth));

	if (msg.laser_pose.theta==msg.robot_pose.theta && !m_frontLaser){
		double res=0;
		res = msg.config.angular_resolution;
// 		if (msg.num_readings==180 || msg.num_readings==181)
// 			res=M_PI/180;
// 		if (msg.num_readings==360 || msg.num_readings==361)
// 			res=M_PI/360;
		assert(res>0);
		string sensorName="FLASER";
		OrientedPoint rpose(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta);
		OrientedPoint lpose(msg.laser_pose.x, msg.laser_pose.y, msg.laser_pose.theta);
		OrientedPoint dp=absoluteDifference(lpose, rpose);
		m_frontLaser=new RangeSensor(sensorName,msg.num_readings, res, OrientedPoint(0,0,msg.laser_pose.theta-msg.robot_pose.theta), 0, 
					      msg.config.maximum_range);
		m_frontLaser->updateBeamsLookup();
		m_sensorMap.insert(make_pair(sensorName, m_frontLaser));
		
		cout << __func__ 
		     << ": " << sensorName <<" configured." 
		     << " Readings " << m_frontLaser->beams().size() 
		     << " Resolution " << res << endl;
	}
	if (msg.laser_pose.theta!=msg.robot_pose.theta && !m_rearLaser){
		double res=0;
		res = msg.config.angular_resolution;
// 		if (msg.num_readings==180 || msg.num_readings==181)
// 			res=M_PI/180;
// 		if (msg.num_readings==360 || msg.num_readings==361)
// 			res=M_PI/360;
		assert(res>0);
		OrientedPoint rpose(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta);
		OrientedPoint lpose(msg.laser_pose.x, msg.laser_pose.y, msg.laser_pose.theta);
		OrientedPoint dp=absoluteDifference(lpose, rpose);
		string sensorName="RLASER";
		m_rearLaser=new RangeSensor(sensorName,msg.num_readings, res, OrientedPoint(0,0,msg.laser_pose.theta-msg.robot_pose.theta), 0, 
					      msg.config.maximum_range);
		m_rearLaser->updateBeamsLookup();
		m_sensorMap.insert(make_pair(sensorName, m_rearLaser));
		
		cout << __func__ 
		     << ": " << sensorName <<" configured." 
		     << " Readings " << m_rearLaser->beams().size() 
		     << " Resolution " << res << endl;
	}	

	const RangeSensor * rs=(msg.laser_pose.theta==msg.robot_pose.theta)?m_frontLaser:m_rearLaser;
	RangeReading reading(rs, msg.timestamp);
	reading.resize(rs->beams().size());
	for (unsigned int i=0; i< (unsigned int)msg.num_readings; i++){
		reading[i]=(double)msg.range[i];
	}
	reading.setPose(OrientedPoint(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta));
	return reading;
}

void CarmenWrapper::publish_globalpos(carmen_localize_summary_p summary)
{
  lock();
  static carmen_localize_globalpos_message globalpos;
  IPC_RETURN_TYPE err;
  
  globalpos.timestamp = carmen_get_time();
  globalpos.host = carmen_get_host();
  globalpos.globalpos = summary->mean;
  globalpos.globalpos_std = summary->std;
  globalpos.globalpos_xy_cov = summary->xy_cov;
  globalpos.odometrypos = summary->odometry_pos;
  globalpos.converged = summary->converged;
  err = IPC_publishData(CARMEN_LOCALIZE_GLOBALPOS_NAME, &globalpos);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_LOCALIZE_GLOBALPOS_NAME);  
  unlock();
}

/* publish a particle message */

void CarmenWrapper::publish_particles(carmen_localize_particle_filter_p filter, 
		       carmen_localize_summary_p summary)
{
  lock();
  static carmen_localize_particle_message pmsg;
  IPC_RETURN_TYPE err;

  pmsg.timestamp = carmen_get_time();
  pmsg.host = carmen_get_host();
  pmsg.globalpos = summary->mean;
  pmsg.globalpos_std = summary->mean;
  pmsg.num_particles = filter->param->num_particles;
  pmsg.particles = (carmen_localize_particle_ipc_p)filter->particles;
  err = IPC_publishData(CARMEN_LOCALIZE_PARTICLE_NAME, &pmsg);
  carmen_test_ipc_exit(err, "Could not publish", 
		       CARMEN_LOCALIZE_PARTICLE_NAME);  
  fprintf(stderr, "P");
  unlock();
}





void * CarmenWrapper::m_reading_function(void*){
	while (true) {
		lock();
		IPC_listen(100);
		unlock();
		usleep(20000);
	}    
	return 0;
}

void CarmenWrapper::shutdown_module(int sig){
  if(sig == SIGINT) {
      carmen_ipc_disconnect();

      fprintf(stderr, "\nDisconnecting (shutdown_module(%d) called).\n",sig);
      exit(0);
  }
}
/*
typedef struct {
  int num_readings;
  float *range;
  char *tooclose;
  double x, y, theta;//position of the laser on the robot
  double odom_x, odom_y, odom_theta; //position of the center of the robot
  double tv, rv;
  double forward_safety_dist, side_safety_dist;
  double turn_axis;
  double timestamp;
  char host[10];
} carmen_robot_laser_message;
*/

carmen_robot_laser_message CarmenWrapper::reading2carmen(const RangeReading& reading){
	carmen_robot_laser_message frontlaser;
	frontlaser.num_readings=reading.size();
	frontlaser.range = new float[frontlaser.num_readings];
	frontlaser.tooclose=0;
	frontlaser.laser_pose.x=frontlaser.robot_pose.x=reading.getPose().x;
	frontlaser.laser_pose.y=frontlaser.robot_pose.y=reading.getPose().y;
	frontlaser.laser_pose.theta=frontlaser.robot_pose.theta=reading.getPose().theta;
	frontlaser.tv=frontlaser.rv=0;
	frontlaser.forward_safety_dist=frontlaser.side_safety_dist=0;
	frontlaser.turn_axis=0;
	frontlaser.timestamp=reading.getTime();
	for (unsigned int i=0; i< reading.size(); i++){
		frontlaser.range[i]=(float)reading[i];
	}
	return frontlaser;
}

carmen_point_t CarmenWrapper::point2carmen (const OrientedPoint& p){
	return (carmen_point_t){p.x,p.y,p.theta};
}

OrientedPoint CarmenWrapper::carmen2point (const carmen_point_t& p){
	return OrientedPoint(p.x, p.y, p.theta);
}


/*
int main (int argc, char** argv) {
	CarmenWrapper::start(argc, argv);
	while(1){
		sleep(2);
		RangeReading reading(0,0);
		while(CarmenWrapper::getReading(reading)){
			cout << "FLASER " <<  reading.size();
			for (int i=0; i<reading.size(); i++)
				cout << " " << reading[i];
			cout << reading.getPose().x << " "
			     << reading.getPose().y << " "
			     << reading.getPose().theta << " 0 cazzo 0" << endl;
		}
		cout << endl;
	}
	return 1;
}
*/

