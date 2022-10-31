#include <cstdlib>
#include <limits.h>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/gridfastslam/gfsreader.h>
#include <qpixmap.h>
#include <qpainter.h>
#include <qimage.h>
#include <qapplication.h>
#include <gmapping/utils/commandline.h>

#define MAX_LASER_BEAMS 1024
#define MAX_FILENAME 1024

using namespace std;
using namespace GMapping;
using namespace GMapping::GFSReader;

inline double min(double a, double b){
	return (a<b)?a:b;
}

inline double max(double a, double b){
	return (a>b)?a:b;
}

void computeBoundingBox(double& xmin, double& ymin, double& xmax, double& ymax, const LaserRecord& laser, const OrientedPoint& pose, double maxrange){
	double theta=-M_PI/2+pose.theta;
	double theta_step=(laser.readings.size()==180||laser.readings.size()==181)?M_PI/180:M_PI/360;
	for (std::vector<double>::const_iterator it=laser.readings.begin(); it!=laser.readings.end(); it++){
		if (*it<maxrange){
			xmin=min(xmin,pose.x+*it*cos(theta));
			ymin=min(ymin,pose.y+*it*sin(theta));
			xmax=max(xmax,pose.x+*it*cos(theta));
			ymax=max(ymax,pose.y+*it*sin(theta));
		}
		theta+=theta_step;
	}
}

void computeBoundingBox(double& xmin, double& ymin, double& xmax, double& ymax, const RecordList& rl, double maxrange){
	xmin = ymin = MAXDOUBLE;
	xmax = ymax =-MAXDOUBLE;
	const LaserRecord* lastLaser=0;
	for (RecordList::const_iterator it=rl.begin(); it!=rl.end(); it++){
		const LaserRecord* lr= dynamic_cast<const LaserRecord*>(*it);
		if (lr){
			lastLaser=lr;
			continue;
		}
		const ScanMatchRecord* smr= dynamic_cast<const ScanMatchRecord*>(*it);
		if (smr && lastLaser){
			for (std::vector<OrientedPoint>::const_iterator pit=smr->poses.begin(); pit!=smr->poses.end(); pit++){
				computeBoundingBox(xmin, ymin, xmax, ymax, *lastLaser, *pit, maxrange);
			}
		}
	}
}
int main(int argc, char** argv){
	QApplication app(argc, argv);
	double maxrange=50;
	double delta=0.1;
	int scanSkip=5;
	const char* filename=0;
	const char* format="PNG";
	CMD_PARSE_BEGIN(1, argc)
		parseDouble("-maxrange", maxrange);
		parseDouble("-delta", delta);
		parseInt("-skip", scanSkip);
		parseString("-filename",filename);
		parseString("-format",format);
	CMD_PARSE_END
	
	double maxUrange=maxrange;
	if (! filename){
		cout << " supply a gfs file, please" << endl;
		cout << " usage gfs2img [options] -filename <gfs_file>" << endl;
		cout << " [options]:" << endl;
		cout << " -maxrange <range>" << endl;
		cout << " -delta    <map cell size>" << endl;
		cout << " -skip     <frames to skip among images>" << endl;
		cout << " -format   <image format in capital letters>" << endl;
		return -1;
	}
	ifstream is(filename);
	if (!is){
		cout << " supply an EXISTING gfs file, please" << endl;
		return -1;
	}
	RecordList rl;
	rl.read(is);
	
	int particles=0;
	int beams=0;
	for (RecordList::const_iterator it=rl.begin(); it!=rl.end(); it++){
		const OdometryRecord* odometry=dynamic_cast<const OdometryRecord*>(*it);
		if (odometry){
			particles=odometry->dim;
		}
		const LaserRecord* s=dynamic_cast<const LaserRecord*>(*it);
		if (s){
			beams=s->readings.size();
		}
		if (particles && beams)
			break;
	}
	cout << "Particles from gfs=" << particles << endl;
	if (! particles){
		cout << "no particles found, terminating" << endl;
		return -1;
	}
	cout << "Laser beams from gfs=" << beams << endl;
	if (! beams){
		cout << "0 beams found, terminating" << endl;
		return -1;
	}
	
	
	double laserBeamStep=0;
	if (beams==180||beams==181){
		laserBeamStep=M_PI/180;
	} else if (beams==360||beams==361){
		laserBeamStep=M_PI/360;
	}
	cout << "Laser beam step" << laserBeamStep << endl;
	if (laserBeamStep==0){
		cout << "Invalid Beam Step, terminating" << endl;
		return -1;
	}
	double laserAngles[MAX_LASER_BEAMS];
	double theta=-M_PI/2;
	for (int i=0; i<beams; i++){
		laserAngles[i]=theta;
		theta+=laserBeamStep;
	}

	ScanMatcher matcher;
	matcher.setLaserParameters(beams, laserAngles, OrientedPoint(0,0,0));
	matcher.setlaserMaxRange(maxrange);
	matcher.setusableRange(maxUrange);
	matcher.setgenerateMap(true);
	
	double xmin, ymin, xmax, ymax;
	cout << "computing bounding box" << endl;
	computeBoundingBox(xmin, ymin, xmax, ymax, rl, maxrange);
	cout << "DONE" << endl << "BBOX= " << xmin << " " << ymin << " " << xmax << " " << ymax << endl;

	Point center;
	center.x=(xmin+xmax)/2.;
	center.y=(ymin+ymax)/2.;
		
	cout << "computing paths" << endl;
	unsigned int frame=0;
	int scanCount=0;
	
	for (RecordList::const_iterator it=rl.begin(); it!=rl.end(); it++){
		const ScanMatchRecord* s=dynamic_cast<const ScanMatchRecord*>(*it);
		if (!s) 
			continue;
		scanCount++;
		if (scanCount%scanSkip)
			continue;
		cout << "Frame " << frame << " ";
		std::vector<RecordList> paths(particles);
		int bestIdx=0;
		double bestWeight=-MAXDOUBLE;
		for (int p=0; p<particles; p++){
			paths[p]=rl.computePath(p,it);
			double w=rl.getLogWeight(p,it);
			if (w>bestWeight){
				bestWeight=w;
				bestIdx=p;
			}
		}
		cout << "bestIdx=" << bestIdx << " bestWeight=" << bestWeight << endl;
		
		cout << "computing best map" << endl;
		ScanMatcherMap smap(center, xmin, ymin, xmax, ymax, delta);
		int count=0;
		for (RecordList::const_iterator mt=paths[bestIdx].begin(); mt!=paths[bestIdx].end(); mt++){
			const LaserRecord* s=dynamic_cast<const LaserRecord*>(*mt);
			if (s){
				double rawreadings[MAX_LASER_BEAMS];
				for (uint i=0; i<s->readings.size(); i++)
					rawreadings[i]=s->readings[i];
				matcher.invalidateActiveArea();
				matcher.computeActiveArea(smap, s->pose, rawreadings);
//				matcher.allocActiveArea(smap, s->pose, rawreadings);
				matcher.registerScan(smap, s->pose, rawreadings);
				count++;
			}
		}
		cout << "DONE " << count <<endl;

		QPixmap pixmap(smap.getMapSizeX(), smap.getMapSizeY());
		pixmap.fill(QColor(200, 200, 255));
		QPainter painter(&pixmap);
		for (int x=0; x<smap.getMapSizeX(); x++)
			for (int y=0; y<smap.getMapSizeY(); y++){
				double v=smap.cell(x,y);
				if (v>=0){
					int grayValue=255-(int)(255.*v);
					painter.setPen(QColor(grayValue, grayValue, grayValue));
					painter.drawPoint(x,smap.getMapSizeY()-y-1);
				}
			}
		
		/*
		cout << "painting trajectories" << endl;
		for (int p=0; p<particles; p++){
			painter.setPen(QColor(Qt::red));
			if (p==bestIdx)
				continue;
			bool first=true;
			IntPoint oldPoint(0,0);
			for (RecordList::const_iterator mt=paths[p].begin(); mt!=paths[p].end(); mt++){
				const LaserRecord* s=dynamic_cast<const LaserRecord*>(*mt);
				if (s){
					IntPoint ip=smap.world2map(s->pose);
					ip.y=smap.getMapSizeY()-ip.y-1;
					if (!first){
						painter.drawLine( oldPoint.x, oldPoint.y, ip.x, ip.y);
					}
					oldPoint=ip;
					first=false;
				}
			}	
			paths[p].destroyReferences();;
		}
		painter.setPen(QColor(Qt::black));
		bool first=true;
		IntPoint oldPoint(0,0);
		for (RecordList::const_iterator mt=paths[bestIdx].begin(); mt!=paths[bestIdx].end(); mt++){
			const LaserRecord* s=dynamic_cast<const LaserRecord*>(*mt);
			if (s){
				IntPoint ip=smap.world2map(s->pose);
				ip.y=smap.getMapSizeY()-ip.y-1;
				if (!first){
					painter.drawLine( oldPoint.x, oldPoint.y, ip.x, ip.y);
				} 
				oldPoint=ip;
				first=false;
			}
		}	
		paths[bestIdx].destroyReferences();;
		*/
		cout << " DONE" << endl;
		cout << "writing image" << endl;
		QImage img=pixmap.convertToImage();
		char ofilename[MAX_FILENAME];
		sprintf(ofilename,"%s-%.4d.%s",filename, frame, format);
		cout << ofilename << endl;
		img.save(QString(ofilename), format,0);
		frame++;
		
	}
	cout << "For Cyrill: \"The Evil is Outside\"" << endl;
}

