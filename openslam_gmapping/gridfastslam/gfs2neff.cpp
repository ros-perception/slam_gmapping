#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

using namespace std;

int main(int argc, char**argv){
	if (argc<3){
		cout << "usage gfs2neff <infilename> <nefffilename>" << endl;
		return -1;
	}
	ifstream is(argv[1]);
	if (!is){
		cout << "could read file "<< endl;
		return -1;
	}
	ofstream os(argv[2]);
	if (! os){
		cout << "could write file "<< endl;
		return -1;
	}
	unsigned int frame=0;
	double neff=0;
	while(is){
			char buf[8192];
			is.getline(buf, 8192);
			istringstream lineStream(buf);
			string recordType;
			lineStream >> recordType;
			if (recordType=="FRAME"){
				lineStream>> frame;
			}
			if (recordType=="NEFF"){
				lineStream>> neff;
				os << frame << " " << neff << endl;
			}
	}
	os.close();
}
