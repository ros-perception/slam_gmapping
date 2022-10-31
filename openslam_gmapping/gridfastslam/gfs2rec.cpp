#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <gmapping/utils/point.h>

#define MAX_LINE_LENGHT (1000000)

using namespace GMapping;
using namespace std;

struct Record{
	unsigned int dim;
	double time;
	virtual ~Record(){}
	virtual void read(istream& is)=0;
	virtual void write(ostream& os){};
};

struct CommentRecord: public Record{
	string text;
	virtual void read(istream& is){
		char buf[MAX_LINE_LENGHT];
		memset(buf,0, MAX_LINE_LENGHT*sizeof(char));
		is.getline(buf, MAX_LINE_LENGHT);
		text=string(buf);
	}
	virtual void write(ostream& os){
		os << "#GFS_COMMENT: " << text << endl;
	}
};

struct PoseRecord: public Record{
	PoseRecord(bool ideal=false){
		truePos=ideal;
	}
	bool truePos;
	OrientedPoint pose;
	void read(istream& is){
		is >> pose.x >> pose.y >> pose.theta;
		time = 0;
		if (is)
			is >> time;
	}
	virtual void write(ostream& os){
		if (truePos) 
			os << "POS-CORR";
		else
			os << "POS ";
		// FIXME os << floor(time) << " " << (int) (time-floor(time)*1e6) << ": ";
		os << "0 0: ";
		os << pose.x*100 << " " << pose.y*100 << " " << 180/M_PI*pose.theta <<  endl;
	}
};

struct NeffRecord: public Record{
	double neff;
	void read(istream& is){
		is >> neff;
	}
	virtual void write(ostream& os){
		os << "NEFF " << neff << endl;
	}
};


struct OdometryRecord: public Record{
	vector<OrientedPoint> poses;
	virtual void read(istream& is){
		is >> dim;
		for (unsigned int i=0; i< dim; i++){
			OrientedPoint p;
			double w;
			is >> p.x;
			is >> p.y;
			is >> p.theta;
			is >> w;
			poses.push_back(p);
		}
		time = 0;
		if (is)
			is >> time;
	}
};


struct ScanMatchRecord: public Record{
	vector<OrientedPoint> poses;
	vector<double> weights;
	virtual void read(istream& is){
		is >> dim;
		for (unsigned int i=0; i< dim; i++){
			OrientedPoint p;
			double w;
			is >> p.x;
			is >> p.y;
			is >> p.theta;
			is >> w;
			poses.push_back(p);
			weights.push_back(w);
		}
	}
};

struct LaserRecord: public Record{
	vector<double> readings;
	OrientedPoint pose;
	virtual void read(istream& is){
		is >> dim;
		for (unsigned int i=0; i< dim; i++){
			double r;
			is >> r;
			readings.push_back(r);
		}
		is >> pose.x;
		is >> pose.y;
		is >> pose.theta;
		time = 0;
		if (is)
			is >> time;
	}
	
	//	dummy, &sec, &usec, &nLas, &nVal, &range ) == EOF) {

	virtual void write(ostream& os){
		os << "POS ";
		// FIXME os << floor(time) << " " << (int) (time-floor(time)*1e6) << ": ";
		os << "0 0: ";
		os << pose.x*100 << " " << pose.y*100 << " " << 180/M_PI*pose.theta <<  endl;
		
		os << "LASER-RANGE ";
		// FIXME os << floor(time) << " " << (int) (time-floor(time)*1e6) << ": ";
		os << " 0 0 0 " << dim << " 180. : ";
		for (unsigned int i=0; i< dim; i++){
			os <<" "<< readings[i]*100 ;
		}
		os << endl;
	};
};

struct ResampleRecord: public Record{
	vector<unsigned int> indexes;
	virtual void read(istream& is){
		is >> dim;
		for (unsigned int i=0; i< dim; i++){
			unsigned int j;
			is >> j;
			indexes.push_back(j);
		}
	}
};

struct RecordList: public list<Record*>{
	mutable int sampleSize;
	
	istream& read(istream& is){
		while(is){
			char buf[8192];
			is.getline(buf, 8192);
			istringstream lineStream(buf);
			string recordType;
			lineStream >> recordType;
			Record* rec=0;
			if (recordType=="LASER_READING"){
				rec=new LaserRecord;
				cout << "l" << flush;
			}
			if (recordType=="ODO_UPDATE"){
				rec=new OdometryRecord;
				cout << "o" << flush;
			}
			if (recordType=="SM_UPDATE"){
				rec=new ScanMatchRecord;
				cout << "m" << flush;
			}
			if (recordType=="SIMULATOR_POS"){
				rec=new PoseRecord(true);
				cout << "t" << flush;
			}
			if (recordType=="RESAMPLE"){
				rec=new ResampleRecord;
				cout << "r" << flush;
			}
			if (recordType=="NEFF"){
				rec=new NeffRecord;
				cout << "n" << flush;
			}
			if (recordType=="COMMENT"){
				rec=new CommentRecord;
				cout << "c" << flush;
			}
			if (rec){
				rec->read(lineStream);
				push_back(rec);
			}
		}
		return is;
	}
	
	double getLogWeight(unsigned int i) const{
		double weight=0;
		unsigned int currentIndex=i;
		for(RecordList::const_reverse_iterator it=rbegin(); it!=rend(); it++){
			ScanMatchRecord* scanmatch=dynamic_cast<ScanMatchRecord*>(*it); 
			if (scanmatch){
				weight+=scanmatch->weights[currentIndex];
			}
			ResampleRecord* resample=dynamic_cast<ResampleRecord*>(*it); 
			if (resample){
				currentIndex=resample->indexes[currentIndex];
			}
		}
		return weight;
	}
	unsigned int getBestIdx() const {
		if (empty())
			return 0;
		const ScanMatchRecord* scanmatch=0;
		const_reverse_iterator it=rbegin();
		while(!scanmatch){
			scanmatch=dynamic_cast<const ScanMatchRecord*>(*it); 
			it++;
		}
		unsigned int dim=scanmatch->dim;
		sampleSize=(int)dim;
		double bestw=-1e200;
		unsigned int best=scanmatch->dim+1;
		for (unsigned i=0; i<dim; i++){
			double w=getLogWeight(i);
			if (w>bestw){
				best=i;
				bestw=w;
			}
		}
		return best;
	}
	
	void printPath(ostream& os, unsigned int i, bool err=false) const{
		unsigned int currentIndex=i;
		OrientedPoint p(0,0,0);
	
		RecordList rl;
		
		//reconstruct a  path
		for(RecordList::const_reverse_iterator it=rbegin(); it!=rend(); it++){
			const NeffRecord* neff=dynamic_cast<const NeffRecord*>(*it);
			if (neff){
				NeffRecord* n=new NeffRecord(*neff);
				rl.push_front(n);
			}
			const ScanMatchRecord* scanmatch=dynamic_cast<const ScanMatchRecord*>(*it); 
			if (scanmatch){
				PoseRecord* pose=new PoseRecord;
				pose->dim=0;
				p=pose->pose=scanmatch->poses[currentIndex];
				rl.push_front(pose);
			}
			const OdometryRecord* odometry=dynamic_cast<const OdometryRecord*>(*it);
			if (odometry){
				PoseRecord* pose=new PoseRecord;
				pose->dim=0;
				p=pose->pose=odometry->poses[currentIndex];
				pose->time=odometry->time;
				rl.push_front(pose);
			}
			const PoseRecord* tpose=dynamic_cast<const PoseRecord*>(*it);
			if (tpose){
				PoseRecord* pose=new PoseRecord(*tpose);
				rl.push_front(pose);
			}
			const LaserRecord* laser=dynamic_cast<const LaserRecord*>(*it); 
			if (laser){
				LaserRecord* claser=new LaserRecord(*laser);
				claser->pose=p;
				rl.push_front(claser);
			}
			const CommentRecord* comment=dynamic_cast<const CommentRecord*>(*it); 
			if (comment){
				CommentRecord* ccomment=new CommentRecord(*comment);
				rl.push_front(ccomment);
			}
			const ResampleRecord* resample=dynamic_cast<const ResampleRecord*>(*it); 
			if (resample){
				currentIndex=resample->indexes[currentIndex];
				ResampleRecord* r= new ResampleRecord(*resample);
				rl.push_front(r);
			}
		}
		bool started=false;
		double ox=0, oy=0, rxx=0, rxy=0, ryx=0, ryy=0, rth=0;
		bool computedTransformation=false;
		bool truePosFound=false;
		OrientedPoint truePose(0,0,0);
		OrientedPoint currPose(0,0,0);
		bool tpf=false;
		double neff=0;
		unsigned int count=0;
		for(RecordList::iterator it=rl.begin(); it!=rl.end(); it++){
			NeffRecord* neffr=dynamic_cast<NeffRecord*>(*it);
			if (neffr)
				neff=neffr->neff/(double)sampleSize;
			started=started || dynamic_cast<const LaserRecord*>(*it)?true:false;
			if (started && ! truePosFound){
				PoseRecord* tpose=dynamic_cast<PoseRecord*>(*it);
				if (tpose && tpose->truePos){
					truePosFound=true;
					tpf=true;
					truePose=tpose->pose;
					os << "# ";
					(*it)->write(os);
				}
			}
			if (started && truePosFound && ! computedTransformation){
				PoseRecord* pos=dynamic_cast<PoseRecord*>(*it);
				if (pos && !pos->truePos){
					OrientedPoint pose=pos->pose;
					rth=truePose.theta-pose.theta;
					double s=sin(rth), c=cos(rth);
					rxx=ryy=c;
					rxy=-s; ryx=s;
					ox=truePose.x-(rxx*pose.x+rxy*pose.y);
					oy=truePose.y-(ryx*pose.x+ryy*pose.y);
					computedTransformation=true;
					os << "# ";
					(*it)->write(os);
					
				}
			}
			ResampleRecord* resample=dynamic_cast<ResampleRecord*>(*it);
			if(resample){
				os << "MARK-POS 0 0: " <<currPose.x*100 << " " << currPose.y*100 << " 0 " << count++ << endl;
			} 
			if (computedTransformation){
				PoseRecord* pos=dynamic_cast<PoseRecord*>(*it);
				if (pos){
					if (pos->truePos){
						tpf=true;
						truePose=pos->pose;
					} else {
						if (tpf){
							tpf=false;
							OrientedPoint pose=pos->pose;
							double ex, ey, eth=truePose.theta-pose.theta-rth;
							ex=truePose.x-(ox+rxx*pose.x+rxy*pose.y);
							ey=truePose.y-(oy+ryx*pose.x+ryy*pose.y);
							eth=atan2(sin(eth), cos(eth));
							if (! err)
								os << "# ERROR ";
							os << neff << " "
							   << ex << " " << ey << " " << eth 
							   << " " << sqrt(ex*ex+ey*ey) << " " << fabs(eth) << endl;
						}
					}
				}
				
			}
			PoseRecord* pos=dynamic_cast<PoseRecord*>(*it);
			if (pos)
				currPose=pos->pose;
			
			if (! err)
				(*it)->write(os);
			delete *it;
		}
	}
};



int main (int argc, const char * const * argv){
	if (argc<3){
		cout << "usage gfs2rec [-err] <infilename> <outfilename>" << endl;
		return -1;
	}
	bool err=0;
	bool neff=0;
	unsigned int  c=1;
	if (!strcmp(argv[c],"-err")){
		err=true;
		c++;
	}
	if (!strcmp(argv[c],"-neff")){
		neff=true;
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
	os.close();
	return 0;
}
