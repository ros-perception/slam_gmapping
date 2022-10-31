#include <cstdlib>
#include <iostream>
#include <fstream>

#include <list>
#include "gmapping/scanmatcher/icp.h"

using namespace GMapping;
using namespace std;

typedef std::list<PointPair> PointPairList;

PointPairList generateRandomPointPairs(int size, OrientedPoint t, double noise=0.){
	PointPairList ppl;
	double s=sin(t.theta), c=cos(t.theta);
	for (int i=0; i<size; i++){
		Point noiseDraw(noise*(drand48()-.5),noise*(drand48()-.5));
		PointPair pp;
		pp.first.x=100.*(drand48()-.5)+200;
		pp.first.y=10.*(drand48()-.5);
		pp.second.x= c*pp.first.x-s*pp.first.y;
		pp.second.y= s*pp.first.x+c*pp.first.y;
		pp.second=pp.second+t+noiseDraw;
		//cerr << "p1=" << pp.first.x << " " << pp.first.y << endl;
		//cerr << "p2=" << pp.second.x << " " << pp.second.y << endl;
		ppl.push_back(pp);
	}
	return ppl;
}


int main(int argc, const char ** argv){
	while (1){
		OrientedPoint t;
		int size;
		cerr << "Insert size, t.x, t.y, t.theta" << endl;
		cin >> size >> t.x >> t.y >> t.theta;
		PointPairList ppl=generateRandomPointPairs(size, t, 3);
		OrientedPoint tc;
		OrientedPoint ttot(0.,0.,0.);
		bool method=true;
		while(1){
			char buf[10];
			cerr << "iterate?" << endl;
			cin.getline(buf,10);
			if (buf[0]=='n')
				method=false;
			else if (buf[0]=='l')
				method=true;
			else if (buf[0]!=char(0))
				break;
			cout << "plot '-' w l, '-' w p, '-' w p" << endl;
			for(PointPairList::iterator it=ppl.begin(); it!=ppl.end(); it++){
				cout << it->first.x << " " << it->first.y<< endl;
				cout << it->second.x << " " << it->second.y<< endl;
				cout << endl;
			}
			cout << "e" << endl;
			for(PointPairList::iterator it=ppl.begin(); it!=ppl.end(); it++){
				cout << it->first.x << " " << it->first.y<< endl;
			}
			cout << "e" << endl;
			for(PointPairList::iterator it=ppl.begin(); it!=ppl.end(); it++){
				cout << it->second.x << " " << it->second.y<< endl;
			}
			cout << "e" << endl;
			
			double error;
			if (!method){
				cerr << "Nonlinear Optimization" << endl;
				error=icpNonlinearStep(tc,ppl);
			}else {
				cerr << "Linear Optimization" << endl;
				error=icpStep(tc,ppl);
			}
			cerr << "ICP err=" << error << " t.x=" << tc.x << " t.y=" << tc.y << " t.theta=" << tc.theta << endl;
			cerr << "\t" << error << " ttot.x=" << ttot.x << " ttot.y=" << ttot.y << " ttot.theta=" << ttot.theta << endl;
			double s=sin(tc.theta), c=cos(tc.theta);
			for(PointPairList::iterator it=ppl.begin(); it!=ppl.end(); it++){
				Point p1(c*it->first.x-s*it->first.y+tc.x,
					 s*it->first.x+c*it->first.y+tc.y);
				it->first=p1;
			}
			ttot.x+=tc.x;
			ttot.y+=tc.y;
			ttot.theta+=tc.theta;
			ttot.theta=atan2(sin(ttot.theta), cos(ttot.theta));
		}
	}
	return 0;
}
