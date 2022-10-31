#include "gmapping/utils/printmemusage.h"

namespace GMapping{

using namespace std;
void printmemusage(){
	pid_t pid=getpid();
	char procfilename[1000];
	sprintf(procfilename, "/proc/%d/status", pid);
	ifstream is(procfilename);
	string line;
	while (is){
		is >> line;
		if (line=="VmData:"){
			is >> line;
			cerr << "#VmData:\t" << line << endl; 
		}
		if (line=="VmSize:"){
			is >> line;
			cerr << "#VmSize:\t" << line << endl; 
		}
			
	}
}

};

