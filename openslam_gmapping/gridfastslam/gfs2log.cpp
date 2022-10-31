#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <gmapping/utils/point.h>
#include "gmapping/gridfastslam/gfsreader.h"

#define MAX_LINE_LENGHT (1000000)

using namespace std;
using namespace GMapping;
using namespace GMapping::GFSReader;

int main (int argc, const char * const * argv){
	if (argc<3){
		cout << "usage gfs2log [-err] [-neff] [-part] [-odom] <infilename> <outfilename>" << endl;
		cout << "  -odom : dump raw odometry in ODOM message instead of inpolated corrected one" << endl;
		return -1;
	}
	bool err=0;
	bool neff=0;
	bool part=0;
	bool odom=0;
	//	int particle_num;
	unsigned int  c=1;
	if (!strcmp(argv[c],"-err")){
		err=true;
		c++;
	}
	if (!strcmp(argv[c],"-neff")){
		neff=true;
		c++;
	}
	if (!strcmp(argv[c],"-part")){
		part=true;
		c++;
	}
	if (!strcmp(argv[c],"-odom")){
		odom=true;
		c++;
	}
	ifstream is(argv[c]);
	if (!is){
		cout << "could read file "<< endl;
		return -1;
	}
		c++;
	RecordList rl;
	rl.read(is);
	unsigned int bestidx=rl.getBestIdx();
	cout << endl << "best index = " <<  bestidx<< endl;
	ofstream os(argv[c]);
	if (! os){
		cout << "could write file "<< endl;
		return -1;
	}
	rl.printPath(os,bestidx,err,odom);
	if(part)
	  rl.printLastParticles(os);
	os.close();
	return 0;
}
