#include <cstdlib>
#include <fstream>
#include <iostream>
#include <gmapping/log/carmenconfiguration.h>
#include <gmapping/log/sensorlog.h>


using namespace std;
using namespace GMapping;

int main(int argc, char ** argv){
	if (argc<2){
		cout << "usage log_test <filename>" << endl;
		exit (-1);
	}
	ifstream is(argv[1]);
	if (! is){
		cout << "no file " << argv[1] << " found" << endl;
		exit (-1);
	}
	CarmenConfiguration conf;
	conf.load(is);
	
	SensorMap m=conf.computeSensorMap();
	
	//for (SensorMap::const_iterator it=m.begin(); it!=m.end(); it++)
	//	cout << it->first << " " << it->second->getName() << endl;
	
	SensorLog log(m);
	is.close();
	
	ifstream ls(argv[1]);
	log.load(ls);
	ls.close();
	cerr << "log size" << log.size() << endl;
	for (SensorLog::iterator it=log.begin(); it!=log.end(); it++){
		RangeReading* rr=dynamic_cast<RangeReading*>(*it);
		if (rr){
			//cerr << rr->getSensor()->getName() << " ";
			//cerr << rr->size()<< " ";
			//for (RangeReading::const_iterator it=rr->begin(); it!=rr->end(); it++){
			//	cerr << *it << " ";
			//}
			cout<< rr->getPose().x << " " << rr->getPose().y << " " << rr->getPose().theta << " " << rr->getTime()  << endl;
		}
	}
}
