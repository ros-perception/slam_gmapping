#include <cstring>
#include "gmapping/gridfastslam/gfsreader.h"
#include <iomanip>
#include <limits>

namespace  GMapping { 

namespace GFSReader{

Record::~Record(){}
void Record::write(ostream& os){};

void CommentRecord::read(istream& is){
	char buf[MAX_LINE_LENGHT];
	memset(buf,0, MAX_LINE_LENGHT*sizeof(char));
	is.getline(buf, MAX_LINE_LENGHT);
	text=string(buf);
}

void CommentRecord::write(ostream& os){
	os << "#GFS_COMMENT: " << text << endl;
}

PoseRecord::PoseRecord(bool ideal){
	truePos=ideal;
}
void PoseRecord::read(istream& is){
	is >> pose.x >> pose.y >> pose.theta;
	time = 0;
	if (is)
		is >> time;
}
void PoseRecord::write(ostream& os){
	if (truePos)
		os << "TRUEPOS ";
	else
		os << "ODOM ";
	os << setiosflags(ios::fixed) << setprecision(6);
	os << pose.x << " " << pose.y << " " << pose.theta << " 0 0 0 ";
	os << time << " pippo " << time << endl;
}

void NeffRecord::read(istream& is){
        is >> neff;
        time =0;
	if (is)
	  is >> time;

}

void NeffRecord::write(ostream& os){
	os << "NEFF " << neff ;
	os << setiosflags(ios::fixed) << setprecision(6);
	os << " " << time << " pippo " << time << endl;
}


void OdometryRecord::read(istream& is){
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

void RawOdometryRecord::read(istream& is){
  is >> pose.x;
  is >> pose.y;
  is >> pose.theta;
  time = 0;
  assert(is);
    is >> time;
 
}


void EntropyRecord::read(istream& is){
  is >> poseEntropy >> trajectoryEntropy >> mapEntropy;
  time =0;
  if (is)
    is >> time;
}

void EntropyRecord::write(ostream& os){
  os << setiosflags(ios::fixed) << setprecision(6) << "ENTROPY " << poseEntropy << " " << trajectoryEntropy << " " << mapEntropy;
  os << " " << time << " pippo " << time << endl;
}

void ScanMatchRecord::read(istream& is){
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

void LaserRecord::read(istream& is){
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

void LaserRecord::write(ostream& os){
	os << "WEIGHT " << weight << endl;
	os << "ROBOTLASER1 ";
	
        
        if ((dim == 541)||(dim == 540)) { // S300
          os <<" 4";  // laser type
          os <<" -2.351831";  // start_angle
          os <<" 4.712389";  // fov
          os <<" 0.008727";  // angular res
          os <<" 30.0" ;  // maxrange
        }
        else if ((dim == 180)||(dim == 181)) { // PLS
          os <<" 0";  // laser type
          os <<" -1.570796";  // start_angle
          os <<" 3.141593";  // fov
          os <<" 0.017453";  // angular res
          os <<" 81.9" ;  // maxrange
        }
        else if ((dim == 360)||(dim == 361)) { // LMS
          os <<" 0";  // laser type
          os <<" -1.570796";  // start_angle
          os <<" 3.141593";  // fov
          os <<" 0.008726";  // angular res
          os <<" 81.9" ;  // maxrange
        }
        else if ((dim == 682)||(dim == 683)) { // URG
          os <<" 0";  // laser type
          os <<" -2.094395";  // start_angle
          os <<" 4.1887902";  // fov
          os << " " <<   360.0/1024.0/180.0*M_PI;  // angular res
          os <<" 5.5" ;  // maxrange
        }
        else {     // PLS
          os <<" 0";  // laser type
          os <<" -1.570796";  // start_angle
          os <<" 3.141593";  // fov
          os <<" 0.017453";  // angular res
          os <<" 81.9" ;  // maxrange
        }
	os <<" 0.01"; // accuracy	
	os <<" 0" ;  // remission mode
	os <<" "<< dim; // num readings
        os << setiosflags(ios::fixed) << setprecision(2);
	for (unsigned int i=0; i< dim; i++){
		os <<" "<< readings[i] ;
	}
	os << setiosflags(ios::fixed) << setprecision(6);
	os <<" 0"; // num remession values
	os <<" "<< pose.x;
	os <<" "<< pose.y;
	os <<" "<< pose.theta;
	os <<" "<< pose.x;
	os <<" "<< pose.y;
	os <<" "<< pose.theta;
	os <<" 0" ;  // tv
	os <<" 0" ;  // rv
	os <<" 0.55" ;  // forward_safety_dist
	os <<" 0.375" ;     // sideward_safety_dist
	os <<" 1000000.0" ; // turn_axis
	os <<" "<< time <<  " localhost " << time << endl;
};

void ResampleRecord::read(istream& is){
	is >> dim;
	for (unsigned int i=0; i< dim; i++){
		unsigned int j;
		is >> j;
		indexes.push_back(j);
	}
}

istream& RecordList::read(istream& is){
	while(is){
		char buf[MAX_LINE_LENGHT];
		is.getline(buf, MAX_LINE_LENGHT);
		istringstream lineStream(buf);
		string recordType;
		lineStream >> recordType;
		Record* rec=0;
		if (recordType=="LASER_READING"){
			rec=new LaserRecord;
//			cout << "l" << flush;
		}
		else if (recordType=="ODO_UPDATE"){
			rec=new OdometryRecord;
//			cout << "o" << flush;
		}
		else if (recordType=="ODOM"){
			rec=new RawOdometryRecord;
//			cout << "O" << flush;
		}
		else if (recordType=="SM_UPDATE"){
			rec=new ScanMatchRecord;
//			cout << "m" << flush;
		}
		else if (recordType=="SIMULATOR_POS"){
			rec=new PoseRecord(true);
//			cout << "t" << flush;
		}
		else if (recordType=="RESAMPLE"){
			rec=new ResampleRecord;
//			cout << "r" << flush;
		}
		else if (recordType=="NEFF"){
			rec=new NeffRecord;
//			cout << "n" << flush;
		}
		else if (recordType=="COMMENT" || recordType=="#COMMENT"){
			rec=new CommentRecord;
//			cout << "c" << flush;
		}
		else if (recordType=="ENTROPY"){
			rec=new EntropyRecord;
//			cout << "c" << flush;
		}
		
		if (rec){
			rec->read(lineStream);
			push_back(rec);
		}
	}
	return is;
}

double RecordList::getLogWeight(unsigned int i) const{
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

double RecordList::getLogWeight(unsigned int i, RecordList::const_iterator frame) const{
	double weight=0;
	unsigned int currentIndex=i;
	for(RecordList::const_reverse_iterator it(frame); it!=rend(); it++){
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

unsigned int RecordList::getBestIdx() const {
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
	double bestw=-std::numeric_limits<double>::max();
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

void RecordList::printLastParticles(ostream& os) const {
	if (empty())
		return;
	const ScanMatchRecord* scanmatch=0;
	const_reverse_iterator it=rbegin();
	while(!scanmatch){
		scanmatch=dynamic_cast<const ScanMatchRecord*>(*it); 
		it++;
	}
	if (! scanmatch)
		return;
	for (vector<OrientedPoint>::const_iterator it=scanmatch->poses.begin(); it!=scanmatch->poses.end(); it++){
		os << "MARKER [color=black; circle=" << it->x*100 << "," << it->y*100 << ",10] 0 pippo 0" << endl;
	}
}

void RecordList::destroyReferences(){
	for(RecordList::iterator it=begin(); it!=end(); it++)
		delete (*it);
	
}

RecordList RecordList::computePath(unsigned int i, RecordList::const_iterator frame) const{
	unsigned int currentIndex=i;
	OrientedPoint p(0,0,0);
	RecordList rl;
	
	//reconstruct a  path
	bool first=true;
	for(RecordList::const_reverse_iterator it(frame); it!=rend(); it++){
		const ScanMatchRecord* scanmatch=dynamic_cast<const ScanMatchRecord*>(*it); 
		if (scanmatch){
			p=scanmatch->poses[currentIndex];
			first=false;
		}
		const LaserRecord* laser=dynamic_cast<const LaserRecord*>(*it); 
		if (laser && !first){
			LaserRecord* claser=new LaserRecord(*laser);
			claser->pose=p;
			rl.push_front(claser);
		}
		const ResampleRecord* resample=dynamic_cast<const ResampleRecord*>(*it); 
		if (resample){
			currentIndex=resample->indexes[currentIndex];
		}
	}
	return rl;
}

	
void RecordList::printPath(ostream& os, unsigned int i, bool err, bool rawodom) const{
	unsigned int currentIndex=i;
	OrientedPoint p(0,0,0);
	RecordList rl;
	double oldWeight=0;
	double w=0;
	//reconstruct a  path
	for(RecordList::const_reverse_iterator it=rbegin(); it!=rend(); it++){
		const NeffRecord* neff=dynamic_cast<const NeffRecord*>(*it);
		if (neff){
			NeffRecord* n=new NeffRecord(*neff);
			rl.push_front(n);
		}
		const EntropyRecord* entropy=dynamic_cast<const EntropyRecord*>(*it);
		if (entropy){
		    EntropyRecord* n=new EntropyRecord(*entropy);
		    rl.push_front(n);
		}
		const ScanMatchRecord* scanmatch=dynamic_cast<const ScanMatchRecord*>(*it); 
		if (scanmatch){
			PoseRecord* pose=new PoseRecord;
			pose->dim=0;
			p=pose->pose=scanmatch->poses[currentIndex];
			w=scanmatch->weights[currentIndex]-oldWeight;
			oldWeight=scanmatch->weights[currentIndex];
			
			if (!rawodom) {
			  rl.push_front(pose);
			}
		}
		const OdometryRecord* odometry=dynamic_cast<const OdometryRecord*>(*it);
		if (odometry){
		  PoseRecord* pose=new PoseRecord;
		  pose->dim=0;
		  p=pose->pose=odometry->poses[currentIndex];
		  pose->time=odometry->time;
		  if (!rawodom) {
		    rl.push_front(pose);
		  }
		}
		const RawOdometryRecord* rawodometry=dynamic_cast<const RawOdometryRecord*>(*it);
		if (rawodometry){
		  PoseRecord* pose=new PoseRecord;
		  pose->dim=0;
		  pose->pose=rawodometry->pose;
		  pose->time=rawodometry->time;
		  if (rawodom) {
		    rl.push_front(pose);
		  }
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
			claser->weight=w;
			rl.push_front(claser);
		}
		const CommentRecord* comment=dynamic_cast<const CommentRecord*>(*it); 
		if (comment){
			CommentRecord* ccomment=new CommentRecord(*comment);
			rl.push_front(ccomment);
		}
		const ResampleRecord* resample=dynamic_cast<const ResampleRecord*>(*it); 
		if (resample){
			rl.push_front(new ResampleRecord(*resample));
			currentIndex=resample->indexes[currentIndex];
		}
		
	}
	bool started=false;
	bool computedTransformation=false;
	bool truePosFound=false;
	OrientedPoint truePose;
	OrientedPoint oldPose;
	OrientedPoint trueStart, realStart;
	bool tpf=false;
	double neff=0;
	double totalError=0;
	int count=0;
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
				trueStart=truePose;
				realStart=pos->pose;
				os << "# ";
				(*it)->write(os);
				computedTransformation=true;
			}
		}
		if (computedTransformation){
		  os << setiosflags(ios::fixed) << setprecision(6);
			PoseRecord* pos=dynamic_cast<PoseRecord*>(*it);
			if (pos){
				if (pos->truePos){
					tpf=true;
					truePose=pos->pose;
				} else {
					if (tpf){
						tpf=false;
						OrientedPoint realDelta=absoluteDifference(pos->pose,realStart);
						OrientedPoint trueDelta=absoluteDifference(truePose,trueStart);
						double ex=realDelta.x-trueDelta.x;
						double ey=realDelta.y-trueDelta.y;
						double eth=realDelta.theta-trueDelta.theta;
						eth=atan2(sin(eth), cos(eth));
						if (! err)
							os << "# ERROR ";
						os << neff << " "
							<< ex << " " << ey << " " << eth 
							<< " " << sqrt(ex*ex+ey*ey) << " " << fabs(eth) << endl;
						totalError+=sqrt(ex*ex+ey*ey);
						count++;
					}
				}
			}
			
		}
		PoseRecord* pos=dynamic_cast<PoseRecord*>(*it);
		if (pos)
			oldPose=pos->pose;
		if (! err)
			(*it)->write(os);
		delete *it;
	}
	if (err)
		cout << "average error" << totalError/count << endl;
}

}; //gfsreader

}; //GMapping;
