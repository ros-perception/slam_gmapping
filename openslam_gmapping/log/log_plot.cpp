#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <gmapping/log/carmenconfiguration.h>
#include <gmapping/log/sensorlog.h>


using namespace std;
using namespace GMapping;

int main(int argc, char ** argv){
  double maxrange=2.;
	if (argc<2){
		cout << "usage log_plot <filename> | gnuplot" << endl;
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
	int count=0;
	int frame=0;
	cerr << "log size" << log.size() << endl;
	for (SensorLog::iterator it=log.begin(); it!=log.end(); it++){
		RangeReading* rr=dynamic_cast<RangeReading*>(*it);
		if (rr){
		  count++;
		  if (count%3)
		    continue;
		  std::vector<Point> points(rr->size());
		  uint j=0;
		  for (uint i=0; i<rr->size(); i++){
		    const RangeSensor * rs=dynamic_cast<const RangeSensor*>(rr->getSensor());
		    double c=rs->beams()[i].c, s=rs->beams()[i].s;
		    double r=(*rr)[i];
		    if (r>maxrange)
		      continue;
		    points[j++]=Point(r*c,r*s);
		  }
		  if (j){
		    char buf[1024];
		    sprintf(buf,"frame-%05d.gif",frame);
		    frame++;
		    cout << "set terminal gif" << endl;
		    cout << "set output \"" << buf << "\"" << endl;
		    cout << "set size ratio -1" << endl;
		    cout << "plot [-3:3][0:3] '-' w p ps 1" << endl;
		    for (uint i=0; i<j; i++){
		      cout << points[i].y << " " << points[i].x << endl;
		  }
		  cout << "e" << endl;
		  }
		}
	}
}
