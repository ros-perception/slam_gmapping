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


#ifndef GSP_THREAD_H
#define GSP_THREAD_H

#ifndef _WIN32
  #include <unistd.h>
  #include <pthread.h>
#endif
#include <deque>
#include <fstream>
#include <iostream>
#include <gmapping/log/carmenconfiguration.h>
#include <gmapping/log/sensorstream.h>
#include <gmapping/gridfastslam/gridslamprocessor.h>

using namespace std;
using namespace GMapping;


#define MAX_STRING_LENGTH 1024


struct GridSlamProcessorThread : public GridSlamProcessor {
		struct Event{
			virtual ~Event();
		};
		
		struct ParticleMoveEvent: public Event{
			bool scanmatched;
			double neff;
			std::vector<OrientedPoint> hypotheses;
			std::vector<double> weightSums;
		};
		
		struct TruePosEvent : public Event{
			OrientedPoint pose;
		};
		
		struct ResampleEvent: public Event{
			std::vector<unsigned int> indexes;
		};
		
		struct MapEvent: public Event{
			ScanMatcherMap* pmap;
			unsigned int index;
			OrientedPoint pose;
			virtual ~MapEvent();
		};
		
		struct DoneEvent: public Event{
		};
		
		typedef deque<Event*> EventDeque;
		
		GridSlamProcessorThread();
		~GridSlamProcessorThread();	
		int init(int argc, const char * const * argv);
		int loadFiles(const char * fn=0);
		static void * fastslamthread(GridSlamProcessorThread* gpt);
		std::vector<OrientedPoint> getHypotheses();
		std::vector<unsigned int> getIndexes();
		
		EventDeque getEvents();
		
		void start();
		void stop();
		
		virtual void onOdometryUpdate();
		virtual void onResampleUpdate();
		virtual void onScanmatchUpdate();
		
		virtual void syncOdometryUpdate();
		virtual void syncResampleUpdate();
		virtual void syncScanmatchUpdate();
		
		void setEventBufferSize(unsigned int length);
		inline void setMapUpdateTime(unsigned int ut) {mapUpdateTime=ut;}
		inline bool isRunning() const {return running;}
		OrientedPoint boundingBox(SensorLog* log, double& xmin, double& ymin, double& xmax, double& ymax) const;
	private:
		
		void addEvent(Event *);
		EventDeque eventBuffer;
		
		unsigned int eventBufferLength;
		unsigned int mapUpdateTime;
		unsigned int mapTimer;
		
		//thread interaction stuff
		std::vector<OrientedPoint> hypotheses;
		std::vector<unsigned int> indexes;
		std::vector<double> weightSums;
		pthread_mutex_t hp_mutex, ind_mutex, hist_mutex;
		pthread_t gfs_thread;
		bool running;
		
		//This are the processor parameters
		std::string filename;
		std::string outfilename;

		double xmin;
		double ymin;
		double xmax;
		double ymax;
		bool autosize;
		double delta;
		double resampleThreshold;
		
		//scan matching parameters
		double sigma;
		double maxrange;
		double maxUrange;
		double regscore;
		double lstep;
		double astep;
		int kernelSize;
		int iterations;
		double critscore;
		double maxMove;
		unsigned int lskip;

		//likelihood
		double lsigma;
		double ogain;
		double llsamplerange, lasamplerange;
		double llsamplestep, lasamplestep;
		double linearOdometryReliability;
		double angularOdometryReliability;
		
	
		//motion model parameters
		double srr, srt, str, stt;
		//particle parameters
		int particles;
		bool skipMatching;
		
		//gfs parameters
		double angularUpdate;
		double linearUpdate;
	
		//robot config
		SensorMap sensorMap;
		//input stream
		InputSensorStream* input;
		std::ifstream plainStream;
		bool readFromStdin;
		bool onLine;
		bool generateMap;
		bool considerOdometryCovariance;
		unsigned int randseed;
		
		//dirty carmen interface
		const char* const * m_argv;
		unsigned int m_argc;
		
};
#endif
