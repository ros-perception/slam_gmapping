#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <gmapping/utils/point.h>
#include <gmapping/utils/commandline.h>
#include "gmapping/gridfastslam/gfsreader.h"
#define MAX_LINE_LENGHT (1000000)

using namespace std;
using namespace GMapping;
using namespace GMapping::GFSReader;

computeBoundingBox()


int main (unsigned int argc, const char * const * argv)
	double delta = 0.1;
	double skip  = 2;
	double rotate = 0;
	double maxrange = 0;
	
	if (argc<3){
		cout << "usage gfs2stream [-step Number]  <outfilename>" << endl;
		return -1;
	}
	
	
	CMD_PARSE_BEGIN(1,argc-2);
	
	CMD_PARSE_END;

	if (argc<3){
		cout << "usage gfs2stream [-step Number]  <outfilename>" << endl;
		return -1;
	}
	bool err=0;
	bool neff=0;
	bool part=0;
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
	rl.printPath(os,bestidx,err);
	if(part)
		rl.printLastParticles(os);
	os.close();
	return 0;
}
