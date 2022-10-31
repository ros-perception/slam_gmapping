#include <cstdlib>
#include <fstream>
#include <iostream>
#include <gmapping/log/carmenconfiguration.h>
#include <gmapping/log/sensorlog.h>
#ifndef _WIN32
  #include <unistd.h>
#endif
#include <gmapping/utils/commandline.h>
#include <gmapping/log/sensorstream.h>
#include "gmapping/scanmatcher/scanmatcherprocessor.h"

using namespace std;
using namespace GMapping;

#define DEBUG cout << __func__
#define MAX_STRING_LENGTH 1024

int main(int argc, const char * const * argv){
	string filename;
	string outfilename;
	double xmin=-100.;
	double ymin=-100.;
	double xmax=100.;
	double ymax=100.;
	double delta=1.;
	double patchDelta=0.1;
	double sigma=0.02;
	double maxrange=81.9;
	double maxUrange=81.9;
	double regscore=1e4;
	double lstep=.05;
	double astep=.05;
	int kernelSize=0;
	int iterations=4;
	double critscore=0.;
	double maxMove=1.;
	bool computeCovariance=false;
	bool readFromStdin=false;
	bool useICP=false;
	double laserx=.0,lasery=.0,lasertheta=.0;
// 	bool headingOnly=false;


	if (argc<2){
		cout << "usage main {arglist}" << endl;
		cout << "where the arguments are: " << endl;
		cout << "\t -xmin     <value>" << endl;
		cout << "\t -xmax     <value>" << endl;
		cout << "\t -ymin     <value>" << endl;
		cout << "\t -ymax     <value>" << endl;
		cout << "\t -maxrange <value>   : maxmimum preception range" << endl;
		cout << "\t -delta    <value>   : patch size" << endl;
		cout << "\t -patchDelta <value> : patch cell size" << endl;
		cout << "\t -lstep    <value> : linear serach step" << endl;
		cout << "\t -astep    <value> : ìangular search step" << endl;
		cout << "\t -regscore <value> : registration scan score" << endl;
		cout << "\t -filename <value> : log filename in carmen format" << endl;
		cout << "\t -sigma    <value> : convolution kernel size" << endl;
		cout << "Look the code for discovering another thousand of unuseful parameters" << endl;
		return -1;
	}

	CMD_PARSE_BEGIN(1,argc);
		parseString("-filename",filename);
		parseString("-outfilename",outfilename);
		parseDouble("-xmin",xmin);
		parseDouble("-xmax",xmax);
		parseDouble("-ymin",ymin);
		parseDouble("-ymax",ymax);
		parseDouble("-delta",delta);
		parseDouble("-patchDelta",patchDelta);
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
		parseFlag("-computeCovariance",computeCovariance);
		parseFlag("-stdin", readFromStdin);
		parseFlag("-useICP", useICP);
		parseDouble("-laserx",laserx);
		parseDouble("-lasery",lasery);
		parseDouble("-lasertheta",lasertheta);
	CMD_PARSE_END;
	
	if (!filename.size()){
		cout << "no filename specified" << endl;
		return -1;
	}

	ifstream is;
	is.open(filename.c_str());
	if (! is){
		cout << "no file found" << endl;
		return -1;
	}

	
	DEBUG << "scanmatcher processor construction" << endl;
	ScanMatcherProcessor scanmatcher(xmin, ymin, xmax, ymax, delta, patchDelta);
	
	//double range, double sigma, int kernsize, double lopt, double aopt, int iterations
	scanmatcher.setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, computeCovariance);
	scanmatcher.setRegistrationParameters(regscore, critscore);
	scanmatcher.setmaxMove(maxMove);
	scanmatcher.useICP=useICP;
	scanmatcher.matcher().setlaserPose(OrientedPoint(laserx,lasery,lasertheta));
	
	CarmenConfiguration conf;
	conf.load(is);
	is.close();

	SensorMap sensorMap=conf.computeSensorMap();
	scanmatcher.setSensorMap(sensorMap);
	
	InputSensorStream* input=0;
	
	ifstream plainStream;
	if (! readFromStdin){
		plainStream.open(filename.c_str());
		input=new InputSensorStream(sensorMap, plainStream);
		cout << "Plain Stream opened="<< (bool) plainStream << endl;
	} else {
		input=new InputSensorStream(sensorMap, cin);
		cout << "Plain Stream opened on stdin" << endl;
	}

/*	
	SensorLog log(sensorMap);
	ifstream logstream(filename);
	log.load(logstream);
	logstream.close();
	cout << "Log loaded " << log.size() << " records" << endl; 
*/
	ostream* output;
	ofstream poseStream;
	if (! readFromStdin){
		if (! outfilename.size()){
			outfilename=string("scanmatched")+filename;
		}
		poseStream.open(outfilename.c_str());
		output=&poseStream;
	} else {
		output=&cout;
	}
	scanmatcher.init();
	ofstream odopathStream("odopath.dat");
	while (*input){
		const SensorReading* r;
		(*input) >> r;
		if (! r)
			continue;
		const RangeReading* rr=dynamic_cast<const RangeReading*>(r);
		if (rr){
			const RangeSensor* s=dynamic_cast<const RangeSensor*>(r->getSensor());
			bool isFront= s->getPose().theta==0;
				
			if (! readFromStdin){
				cout << "." << flush;
			}
			const RangeSensor* rs=dynamic_cast<const RangeSensor*>(rr->getSensor());
			assert (rs && rs->beams().size()==rr->size());
			odopathStream << rr->getPose().x << " " << rr->getPose().y << endl;
			scanmatcher.processScan(*rr);
			OrientedPoint p=scanmatcher.getPose();
			if (isFront)
				*output << "FLASER "<< rr->size() << " ";
			else
				*output << "RLASER "<< rr->size() << " ";
			for (RangeReading::const_iterator b=rr->begin(); b!=rr->end(); b++){
				*output << *b << " ";
			}
			*output << p.x << " " << p.y << " " << p.theta << " ";
			//p=rr->getPose();
			double t=rr->getTime(); //FIXME
			*output << p.x << " " << p.y << " " << p.theta << " ";
			*output << t << " nohost " << t <<  endl;
		}
	}
	if (! readFromStdin){
		poseStream.close();
	}
}
