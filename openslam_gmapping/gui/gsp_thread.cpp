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


#include "gmapping/gui/gsp_thread.h"
#include <gmapping/utils/commandline.h>
#include <gmapping/utils/stat.h>
#include <gmapping/configfile/configfile.h>

#ifdef CARMEN_SUPPORT
	#include <gmapping/carmenwrapper/carmenwrapper.h>
#endif

#define DEBUG cout << __func__

using namespace std;

int GridSlamProcessorThread::init(int argc, const char * const * argv){
	m_argc=argc;
	m_argv=argv;
	std::string configfilename;
	std::string ebuf="not_set";

	CMD_PARSE_BEGIN_SILENT(1,argc);
		parseStringSilent("-cfg",configfilename);
	CMD_PARSE_END_SILENT;

	if (configfilename.length()>0){
	  ConfigFile cfg(configfilename);

	  filename = (std::string) cfg.value("gfs","filename",filename);
	  outfilename = (std::string) cfg.value("gfs","outfilename",outfilename);
	  xmin = cfg.value("gfs","xmin", xmin);
	  xmax = cfg.value("gfs","xmax",xmax);
	  ymin = cfg.value("gfs","ymin",ymin);
	  ymax = cfg.value("gfs","ymax",ymax);
	  delta =  cfg.value("gfs","delta",delta);
	  maxrange = cfg.value("gfs","maxrange",maxrange);
	  maxUrange = cfg.value("gfs","maxUrange",maxUrange);
	  regscore = cfg.value("gfs","regscore",regscore);
	  critscore = cfg.value("gfs","critscore",critscore);
	  kernelSize = cfg.value("gfs","kernelSize",kernelSize);
	  sigma = cfg.value("gfs","sigma",sigma);
	  iterations = cfg.value("gfs","iterations",iterations);
	  lstep = cfg.value("gfs","lstep",lstep);
	  astep = cfg.value("gfs","astep",astep);
	  maxMove = cfg.value("gfs","maxMove",maxMove);
	  srr = cfg.value("gfs","srr", srr);
	  srt = cfg.value("gfs","srt", srt);
	  str = cfg.value("gfs","str", str);
	  stt = cfg.value("gfs","stt", stt);
	  particles = cfg.value("gfs","particles",particles);
	  angularUpdate = cfg.value("gfs","angularUpdate", angularUpdate);
	  linearUpdate = cfg.value("gfs","linearUpdate", linearUpdate);
	  lsigma = cfg.value("gfs","lsigma", lsigma);
	  ogain = cfg.value("gfs","lobsGain", ogain);
	  lskip = (int)cfg.value("gfs","lskip", lskip);
	  mapUpdateTime = cfg.value("gfs","mapUpdate", mapUpdateTime);
	  randseed = cfg.value("gfs","randseed", randseed);
	  autosize = cfg.value("gfs","autosize", autosize);
	  readFromStdin = cfg.value("gfs","stdin", readFromStdin);
	  resampleThreshold = cfg.value("gfs","resampleThreshold", resampleThreshold);
	  skipMatching = cfg.value("gfs","skipMatching", skipMatching);
	  onLine = cfg.value("gfs","onLine", onLine);
	  generateMap = cfg.value("gfs","generateMap", generateMap);
	  m_minimumScore = cfg.value("gfs","minimumScore", m_minimumScore);
	  llsamplerange = cfg.value("gfs","llsamplerange", llsamplerange);
	  lasamplerange = cfg.value("gfs","lasamplerange",lasamplerange );
	  llsamplestep = cfg.value("gfs","llsamplestep", llsamplestep);
	  lasamplestep = cfg.value("gfs","lasamplestep", lasamplestep);
	  linearOdometryReliability = cfg.value("gfs","linearOdometryReliability",linearOdometryReliability);
	  angularOdometryReliability = cfg.value("gfs","angularOdometryReliability",angularOdometryReliability);
	  ebuf = (std::string) cfg.value("gfs","estrategy", ebuf);
	  considerOdometryCovariance = cfg.value("gfs","considerOdometryCovariance",considerOdometryCovariance);
	  
	}


	CMD_PARSE_BEGIN(1,argc);
          	parseString("-cfg",configfilename);     /* to avoid the warning*/
		parseString("-filename",filename);
		parseString("-outfilename",outfilename);
		parseDouble("-xmin",xmin);
		parseDouble("-xmax",xmax);
		parseDouble("-ymin",ymin);
		parseDouble("-ymax",ymax);
		parseDouble("-delta",delta);
		parseDouble("-maxrange",maxrange);
		parseDouble("-maxUrange",maxUrange);
		parseDouble("-regscore",regscore);
		parseDouble("-critscore",critscore);
		parseInt("-kernelSize",kernelSize);
		parseDouble("-sigma",sigma);
		parseInt("-iterations",iterations);
		parseDouble("-lstep",lstep);
		parseDouble("-astep",astep);
		parseDouble("-maxMove",maxMove);
		parseDouble("-srr", srr);
		parseDouble("-srt", srt);
		parseDouble("-str", str);
		parseDouble("-stt", stt);
		parseInt("-particles",particles);
		parseDouble("-angularUpdate", angularUpdate);
		parseDouble("-linearUpdate", linearUpdate);
		parseDouble("-lsigma", lsigma);
		parseDouble("-lobsGain", ogain);
		parseInt("-lskip", lskip);
		parseInt("-mapUpdate", mapUpdateTime);
		parseInt("-randseed", randseed);
		parseFlag("-autosize", autosize);
		parseFlag("-stdin", readFromStdin);
		parseDouble("-resampleThreshold", resampleThreshold);
		parseFlag("-skipMatching", skipMatching);
		parseFlag("-onLine", onLine);
		parseFlag("-generateMap", generateMap);
		parseDouble("-minimumScore", m_minimumScore);
		parseDouble("-llsamplerange", llsamplerange);
		parseDouble("-lasamplerange", lasamplerange);
		parseDouble("-llsamplestep", llsamplestep);
		parseDouble("-lasamplestep", lasamplestep);
		parseDouble("-linearOdometryReliability",linearOdometryReliability);
		parseDouble("-angularOdometryReliability",angularOdometryReliability);
		parseString("-estrategy", ebuf);
		
		parseFlag("-considerOdometryCovariance",considerOdometryCovariance);
	CMD_PARSE_END;
	
	if (filename.length() <=0){
		cout << "no filename specified" << endl;
		return -1;
	}
	return 0;
}

int GridSlamProcessorThread::loadFiles(const char * fn){
	if (onLine){
		cout << " onLineProcessing" << endl;
		return 0;
	}	
	ifstream is;
	if (fn)
	  is.open(fn);
	else
	  is.open(filename.c_str());
	if (! is){
		cout << "no file found" << endl;
		return -1;
	}

	CarmenConfiguration conf;
	conf.load(is);
	is.close();

	sensorMap=conf.computeSensorMap();
	
	if (input)
		delete input;
	
	if (! readFromStdin){
		plainStream.open(filename.c_str());
		input=new InputSensorStream(sensorMap, plainStream);
		cout << "Plain Stream opened="<< (bool) plainStream << endl;
	} else {
		input=new InputSensorStream(sensorMap, cin);
		cout << "Plain Stream opened on stdin" << endl;
	}
	return 0;
}	

GridSlamProcessorThread::GridSlamProcessorThread(): GridSlamProcessor(cerr){
	//This are the processor parameters
	filename="";
	outfilename="";
	xmin=-100.;
	ymin=-100.;
	xmax=100.;
	ymax=100.;
	delta=0.05;
	
	//scan matching parameters
	sigma=0.05;
	maxrange=80.;
	maxUrange=80.;
	regscore=1e4;
	lstep=.05;
	astep=.05;
	kernelSize=1;
	iterations=5;
	critscore=0.;
	maxMove=1.;
	lsigma=.075;
	ogain=3;
	lskip=0;
	autosize=false;
	skipMatching=false;

	//motion model parameters
	srr=0.1, srt=0.1, str=0.1, stt=0.1;
	//particle parameters
	particles=30;
	randseed=0;
	
	//gfs parameters
	angularUpdate=0.5;
	linearUpdate=1;
	resampleThreshold=0.5;
	
	input=0;
	
	pthread_mutex_init(&hp_mutex,0);
	pthread_mutex_init(&ind_mutex,0);
	pthread_mutex_init(&hist_mutex,0);
	running=false;
	eventBufferLength=0;
	mapUpdateTime=5;
	mapTimer=0;
	readFromStdin=false;
	onLine=false;
	generateMap=false;

	// This  are the dafault settings for a grid map of 5 cm
	llsamplerange=0.01;
	llsamplestep=0.01;
	lasamplerange=0.005;
	lasamplestep=0.005;
	linearOdometryReliability=0.;
	angularOdometryReliability=0.;
	
	considerOdometryCovariance=false;
/*	
	// This  are the dafault settings for a grid map of 10 cm
	m_llsamplerange=0.1;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
*/	
	// This  are the dafault settings for a grid map of 20/25 cm
/*
	m_llsamplerange=0.2;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
	m_generateMap=false;
*/

			
}

GridSlamProcessorThread::~GridSlamProcessorThread(){
	pthread_mutex_destroy(&hp_mutex);
	pthread_mutex_destroy(&ind_mutex);
	pthread_mutex_destroy(&hist_mutex);
	
	for (deque<Event*>::const_iterator it=eventBuffer.begin(); it!=eventBuffer.end(); it++)
		delete *it;
}
	

void * GridSlamProcessorThread::fastslamthread(GridSlamProcessorThread* gpt){
	if (! gpt->input && ! gpt->onLine)
		return 0;
		
	
	//if started online retrieve the settings from the connection
#ifdef CARMEN_SUPPORT
	if (gpt->onLine){
		cout << "starting the process:" << endl;
		CarmenWrapper::initializeIPC(gpt->m_argv[0]);
		CarmenWrapper::start(gpt->m_argv[0]);
		cout << "Waiting for retrieving the sensor map:" << endl;
		while (! CarmenWrapper::sensorMapComputed()){
			usleep(500000);
			cout << "." << flush;
		}
		gpt->sensorMap=CarmenWrapper::sensorMap();
		cout << "Connected " << endl;
	}
#else
	if (gpt->onLine){
		cout << "FATAL ERROR: cannot run online without the carmen support" << endl;
		DoneEvent *done=new DoneEvent;
		gpt->addEvent(done);
		return 0;
	}
#endif
	
	gpt->setSensorMap(gpt->sensorMap);
	gpt->setMatchingParameters(gpt->maxUrange, gpt->maxrange, gpt->sigma, gpt->kernelSize, gpt->lstep, gpt->astep, gpt->iterations, gpt->lsigma, gpt->ogain, gpt->lskip);
	
	double xmin=gpt->xmin, 
	       ymin=gpt->ymin, 
	       xmax=gpt->xmax, 
	       ymax=gpt->ymax;
	
	OrientedPoint initialPose(0,0,0);
	
	if (gpt->autosize){
		if (gpt->readFromStdin || gpt->onLine)
		cout << "Error, cant autosize form stdin" << endl;
		SensorLog * log=new SensorLog(gpt->sensorMap);
		ifstream is(gpt->filename.c_str());
		log->load(is);
		is.close();
		initialPose=gpt->boundingBox(log, xmin, ymin, xmax, ymax);
		delete log;
	}
	
	if( gpt->infoStream()){
		gpt->infoStream() << " initialPose=" << initialPose.x << " " << initialPose.y << " " << initialPose.theta
				<< cout << " xmin=" << xmin <<" ymin=" << ymin <<" xmax=" << xmax <<" ymax=" << ymax << endl;
	}
	gpt->setMotionModelParameters(gpt->srr, gpt->srt, gpt->str, gpt->stt);
	gpt->setUpdateDistances(gpt->linearUpdate, gpt->angularUpdate, gpt->resampleThreshold);
	gpt->setgenerateMap(gpt->generateMap);
	gpt->GridSlamProcessor::init(gpt->particles, xmin, ymin, xmax, ymax, gpt->delta, initialPose);
	gpt->setllsamplerange(gpt->llsamplerange);
	gpt->setllsamplestep(gpt->llsamplestep);
	gpt->setlasamplerange(gpt->llsamplerange);
	gpt->setlasamplestep(gpt->llsamplestep);
	
#define printParam(n)\
	 gpt->outputStream() \
	 << "PARAM " << \
	 #n \
	 << " " << gpt->n << endl
	
	if (gpt->outfilename.length()>0 ){
		gpt->outputStream().open(gpt->outfilename.c_str());
		printParam(filename);
		printParam(outfilename);
		printParam(xmin);
		printParam(ymin);
		printParam(xmax);
		printParam(ymax);
		printParam(delta);
		
		//scan matching parameters
		printParam(sigma);
		printParam(maxrange);
		printParam(maxUrange);
		printParam(regscore);
		printParam(lstep);
		printParam(astep);
		printParam(kernelSize);
		printParam(iterations);
		printParam(critscore);
		printParam(maxMove);
		printParam(lsigma);
		printParam(ogain);
		printParam(lskip);
		printParam(autosize);
		printParam(skipMatching);
	
		//motion model parameters
		printParam(srr);
		printParam(srt); 
		printParam(str);
		printParam(stt);
		//particle parameters
		printParam(particles);
		printParam(randseed);
		
		//gfs parameters
		printParam(angularUpdate);
		printParam(linearUpdate);
		printParam(resampleThreshold);
		
		printParam(llsamplerange);
		printParam(lasamplerange);
		printParam(llsamplestep);
		printParam(lasamplestep);
	}
	#undef printParam
	
	if (gpt->randseed!=0)
		sampleGaussian(1,gpt->randseed);
	if (!gpt->infoStream()){
		cerr << "cant open info stream for writing by unuseful debug messages" << endl;
	} else {
		gpt->infoStream() << "setting randseed" << gpt->randseed<< endl;
	}
	
	
#ifdef CARMEN_SUPPORT
	list<RangeReading*> rrlist;
	if (gpt->onLine){
		RangeReading rr(0,0);
		while (1){
			while (CarmenWrapper::getReading(rr)){
				RangeReading* nr=new RangeReading(rr);
				rrlist.push_back(nr);
				gpt->processScan(*nr);
			}
		}
	}
#endif
	ofstream rawpath("rawpath.dat");
	if (!gpt->onLine){
		while(*(gpt->input) && gpt->running){
			const SensorReading* r;
			(*(gpt->input)) >> r;
			if (! r)
				continue;
			const RangeReading* rr=dynamic_cast<const RangeReading*>(r);
			if (rr && gpt->running){
				const RangeSensor* rs=dynamic_cast<const RangeSensor*>(rr->getSensor());
				assert (rs && rs->beams().size()==rr->size());
				
				bool processed=gpt->processScan(*rr);
				rawpath << rr->getPose().x << " " << rr->getPose().y << " " << rr->getPose().theta << endl;
				if (0 && processed){
				cerr << "Retrieving state .. ";
					TNodeVector trajetories=gpt->getTrajectories();
					cerr << "Done" <<  endl;
					cerr << "Deleting Tree state .. ";
					for (TNodeVector::iterator it=trajetories.begin(); it!=trajetories.end(); it++)
						delete *it;
					cerr << "Done" << endl;
				}
// 				if (0 && processed){
// 					cerr << "generating copy" << endl;;
// 					GridSlamProcessor* m_gsp=gpt->clone();
// 					Map<double, DoubleArray2D, false>*  pmap=m_gsp->getParticles()[0].map.toDoubleMap() ;
// 					cerr << "deleting" << endl;
// 					delete m_gsp;
// 					delete pmap;
// 				}
			}
			const OdometryReading* o=dynamic_cast<const OdometryReading*>(r);
			if (o && gpt->running){
				gpt->processTruePos(*o);
				TruePosEvent* truepos=new TruePosEvent;
				truepos->pose=o->getPose();
			}
		}
	}
	rawpath.close();

	TNodeVector trajetories=gpt->getTrajectories();
	cerr << "WRITING WEIGHTS" << endl;
	int pnumber=0;
	for (TNodeVector::iterator it=trajetories.begin(); it!=trajetories.end(); it++){
		char buf[10];
		sprintf(buf, "w-%03d.dat",pnumber);
		ofstream weightsStream(buf);
		GridSlamProcessor::TNode* n=*it;
		double oldWeight=0, oldgWeight=0;
		while (n!=0){
			double w=n->weight-oldWeight;
			double gw=n->gweight-oldgWeight;
			oldWeight=n->weight;
			oldgWeight=n->gweight;
			weightsStream << w << " " << gw << endl;
			n=n->parent;
		}
		weightsStream.close();
		pnumber++;
		cerr << buf << endl;
	}
		
	DoneEvent *done=new DoneEvent;
	gpt->addEvent(done);
	gpt->infoStream() << "Hallo, I am the gsp thread. I have finished. Do you think it is the case of checking for unlocked mutexes." << endl;
	return 0;
}

std::vector<OrientedPoint> GridSlamProcessorThread::getHypotheses(){
	pthread_mutex_lock(&hp_mutex);
	std::vector<OrientedPoint> retval(hypotheses);
	pthread_mutex_unlock(&hp_mutex);
	return retval;
}

std::vector<unsigned int> GridSlamProcessorThread::getIndexes(){
	pthread_mutex_lock(&ind_mutex);
	std::vector<unsigned int> retval(indexes);
	pthread_mutex_unlock(&ind_mutex);
	return retval;
}

void GridSlamProcessorThread::start(){
	if (running)
		return;
	running=true;
	pthread_create(&gfs_thread, 0, (void * (*)(void *))fastslamthread, (void *) this);
}

void GridSlamProcessorThread::stop(){
	if (! running){
		cout << "PORCO CAZZO" << endl;
		return;
	}
	running=false;
	void * retval;
	pthread_join(gfs_thread, &retval);
}

void GridSlamProcessorThread::onOdometryUpdate(){
	pthread_mutex_lock(&hp_mutex);
	hypotheses.clear();
	weightSums.clear();
	for (GridSlamProcessor::ParticleVector::const_iterator part=getParticles().begin(); part!=getParticles().end(); part++ ){
		hypotheses.push_back(part->pose);
		weightSums.push_back(part->weightSum);
	}
	
	ParticleMoveEvent* event=new ParticleMoveEvent;
	event->scanmatched=false;
	event->hypotheses=hypotheses;
	event->weightSums=weightSums;
	event->neff=m_neff;
	pthread_mutex_unlock(&hp_mutex);
	
	addEvent(event);
	
	syncOdometryUpdate();
}

void GridSlamProcessorThread::onResampleUpdate(){
	pthread_mutex_lock(&ind_mutex);
	pthread_mutex_lock(&hp_mutex);
	
	indexes=GridSlamProcessor::getIndexes();
	
	assert (indexes.size()==getParticles().size());
	ResampleEvent* event=new ResampleEvent;
	event->indexes=indexes;
	
	pthread_mutex_unlock(&hp_mutex);
	pthread_mutex_unlock(&ind_mutex);
	
	addEvent(event);
	
	syncResampleUpdate();
}

void GridSlamProcessorThread::onScanmatchUpdate(){
	pthread_mutex_lock(&hp_mutex);
	hypotheses.clear();
	weightSums.clear();
	unsigned int bestIdx=0;
	double bestWeight=-1e1000;
	unsigned int idx=0;
	for (GridSlamProcessor::ParticleVector::const_iterator part=getParticles().begin(); part!=getParticles().end(); part++ ){
		hypotheses.push_back(part->pose);
		weightSums.push_back(part->weightSum);
		if(part->weightSum>bestWeight){
			bestIdx=idx;
			bestWeight=part->weightSum;
		}
		idx++;
	}
	
	ParticleMoveEvent* event=new ParticleMoveEvent;
	event->scanmatched=true;
	event->hypotheses=hypotheses;
	event->weightSums=weightSums;
	event->neff=m_neff;
	addEvent(event);
	
	if (! mapTimer){
		MapEvent* event=new MapEvent;
		event->index=bestIdx;
		event->pmap=new ScanMatcherMap(getParticles()[bestIdx].map);
		event->pose=getParticles()[bestIdx].pose;
		addEvent(event);
	}
	
	mapTimer++;
	mapTimer=mapTimer%mapUpdateTime;
	
	pthread_mutex_unlock(&hp_mutex);

	syncOdometryUpdate();
}

void GridSlamProcessorThread::syncOdometryUpdate(){
}

void GridSlamProcessorThread::syncResampleUpdate(){
}

void GridSlamProcessorThread::syncScanmatchUpdate(){
}

void GridSlamProcessorThread::addEvent(GridSlamProcessorThread::Event * e){
	pthread_mutex_lock(&hist_mutex);
	while (eventBuffer.size()>eventBufferLength){
		Event* event=eventBuffer.front();
		delete event;
		eventBuffer.pop_front();
	}
	eventBuffer.push_back(e);
	pthread_mutex_unlock(&hist_mutex);
}

GridSlamProcessorThread::EventDeque GridSlamProcessorThread::getEvents(){
	pthread_mutex_lock(&hist_mutex);
	EventDeque copy(eventBuffer);
	eventBuffer.clear();
	pthread_mutex_unlock(&hist_mutex);
	return copy;
}

GridSlamProcessorThread::Event::~Event(){}

GridSlamProcessorThread::MapEvent::~MapEvent(){
	if (pmap)
		delete pmap;
}

void GridSlamProcessorThread::setEventBufferSize(unsigned int length){
	eventBufferLength=length;
}

OrientedPoint GridSlamProcessorThread::boundingBox(SensorLog* log, double& xmin, double& ymin, double& xmax, double& ymax) const{
	OrientedPoint initialPose(0,0,0);
	initialPose=log->boundingBox(xmin, ymin, xmax, ymax);
	xmin-=3*maxrange;
	ymin-=3*maxrange;
	xmax+=3*maxrange;
	ymax+=3*maxrange;
	return initialPose;
}
