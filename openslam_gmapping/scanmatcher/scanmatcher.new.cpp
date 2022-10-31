#include <list>
#include <iostream>

#include "gmapping/scanmatcher/scanmatcher.h"
#include "gmapping/scanmatcher/gridlinetraversal.h"
//#define GENERATE_MAPS

using namespace std;
namespace GMapping {

const double ScanMatcher::nullLikelihood=-1.;

ScanMatcher::ScanMatcher(): m_laserPose(0,0,0){
	m_laserAngles=0;
	m_laserBeams=0;
	m_optRecursiveIterations=3;
	m_activeAreaComputed=false;

	// This  are the dafault settings for a grid map of 5 cm
	m_llsamplerange=0.01;
	m_llsamplestep=0.01;
	m_lasamplerange=0.005;
	m_lasamplestep=0.005;
/*	
	// This  are the dafault settings for a grid map of 10 cm
	m_llsamplerange=0.1;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
*/	
	// This  are the dafault settings for a grid map of 20/25 cm
/*
	m_llsamplerange=0.2;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
	m_generateMap=false;
*/
}

ScanMatcher::~ScanMatcher(){
	if (m_laserAngles)
		delete [] m_laserAngles;
}

void ScanMatcher::invalidateActiveArea(){
	m_activeAreaComputed=false;
}

void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings){
	if (m_activeAreaComputed)
		return;
	HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	IntPoint p0=map.world2map(lp);
	const double * angle=m_laserAngles;
	for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
		if (m_generateMap){
			double d=*r;
			if (d>m_laserMaxRange)
				continue;
			if (d>m_usableRange)
				d=m_usableRange;
			
			Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
			IntPoint p1=map.world2map(phit);
			
			d+=map.getDelta();
			//Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
			//IntPoint p2=map.world2map(phit2);
			IntPoint linePoints[20000] ;
			GridLineTraversalLine line;
			line.points=linePoints;
			//GridLineTraversal::gridLine(p0, p2, &line);
			GridLineTraversal::gridLine(p0, p1, &line);
			for (int i=0; i<line.num_points-1; i++){
				activeArea.insert(map.storage().patchIndexes(linePoints[i]));
			}
			if (d<=m_usableRange){
				activeArea.insert(map.storage().patchIndexes(p1));
				//activeArea.insert(map.storage().patchIndexes(p2));
			}
		} else {
			if (*r>m_laserMaxRange||*r>m_usableRange) continue;
			Point phit=lp;
			phit.x+=*r*cos(lp.theta+*angle);
			phit.y+=*r*sin(lp.theta+*angle);
			IntPoint p1=map.world2map(phit);
			assert(p1.x>=0 && p1.y>=0);
			IntPoint cp=map.storage().patchIndexes(p1);
			assert(cp.x>=0 && cp.y>=0);
			activeArea.insert(cp);
			
		}
	//this allocates the unallocated cells in the active area of the map
	//cout << "activeArea::size() " << activeArea.size() << endl;
	map.storage().setActiveArea(activeArea, true);
	m_activeAreaComputed=true;
}

void ScanMatcher::registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings){
	if (!m_activeAreaComputed)
		computeActiveArea(map, p, readings);
		
	//this operation replicates the cells that will be changed in the registration operation
	map.storage().allocActiveArea();
	
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	IntPoint p0=map.world2map(lp);
	const double * angle=m_laserAngles;
	for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
		if (m_generateMap){	
			double d=*r;
			if (d>m_laserMaxRange)
				continue;
			if (d>m_usableRange)
				d=m_usableRange;
			Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
			IntPoint p1=map.world2map(phit);
			
			d+=map.getDelta();
			//Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
			//IntPoint p2=map.world2map(phit2);
			IntPoint linePoints[20000] ;
			GridLineTraversalLine line;
			line.points=linePoints;
			//GridLineTraversal::gridLine(p0, p2, &line);
			GridLineTraversal::gridLine(p0, p1, &line);
			for (int i=0; i<line.num_points-1; i++){
				map.cell(line.points[i]).update(false, Point(0,0));
			}
			if (d<=m_usableRange){
				map.cell(p1).update(true,phit);
			//	map.cell(p2).update(true,phit);
			}
		} else {
			if (*r>m_laserMaxRange||*r>m_usableRange) continue;
			Point phit=lp;
			phit.x+=*r*cos(lp.theta+*angle);
			phit.y+=*r*sin(lp.theta+*angle);
			map.cell(phit).update(true,phit);
		}
}



double ScanMatcher::optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const{
	double bestScore=-1;
	OrientedPoint currentPose=init;
	double currentScore=score(map, currentPose, readings);
	double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
	unsigned int refinement=0;
	enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
	int c_iterations=0;
	do{
		if (bestScore>=currentScore){
			refinement++;
			adelta*=.5;
			ldelta*=.5;
		}
		bestScore=currentScore;
//		cout <<"score="<< currentScore << " refinement=" << refinement;
//		cout <<  "pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
		OrientedPoint bestLocalPose=currentPose;
		OrientedPoint localPose=currentPose;

		Move move=Front;
		do {
			localPose=currentPose;
			switch(move){
				case Front:
					localPose.x+=ldelta;
					move=Back;
					break;
				case Back:
					localPose.x-=ldelta;
					move=Left;
					break;
				case Left:
					localPose.y-=ldelta;
					move=Right;
					break;
				case Right:
					localPose.y+=ldelta;
					move=TurnLeft;
					break;
				case TurnLeft:
					localPose.theta+=adelta;
					move=TurnRight;
					break;
				case TurnRight:
					localPose.theta-=adelta;
					move=Done;
					break;
				default:;
			}
			double localScore=score(map, localPose, readings);
			if (localScore>currentScore){
				currentScore=localScore;
				bestLocalPose=localPose;
			}
			c_iterations++;
		} while(move!=Done);
		currentPose=bestLocalPose;
		//cout << __func__ << "currentScore=" << currentScore<< endl;
		//here we look for the best move;
	}while (currentScore>bestScore || refinement<m_optRecursiveIterations);
	//cout << __func__ << "bestScore=" << bestScore<< endl;
	//cout << __func__ << "iterations=" << c_iterations<< endl;
	pnew=currentPose;
	return bestScore;
}

struct ScoredMove{
	OrientedPoint pose;
	double score;
	double likelihood;
};

typedef std::list<ScoredMove> ScoredMoveList;

double ScanMatcher::optimize(OrientedPoint& _mean, ScanMatcher::CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const{
	ScoredMoveList moveList;
	double bestScore=-1;
	OrientedPoint currentPose=init;
	ScoredMove sm={currentPose,0,0};
	unsigned int matched=likelihoodAndScore(sm.score, sm.likelihood, map, currentPose, readings);
	double currentScore=sm.score;
	moveList.push_back(sm);
	double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
	unsigned int refinement=0;
	enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
	do{
		if (bestScore>=currentScore){
			refinement++;
			adelta*=.5;
			ldelta*=.5;
		}
		bestScore=currentScore;
//		cout <<"score="<< currentScore << " refinement=" << refinement;
//		cout <<  "pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
		OrientedPoint bestLocalPose=currentPose;
		OrientedPoint localPose=currentPose;

		Move move=Front;
		do {
			localPose=currentPose;
			switch(move){
				case Front:
					localPose.x+=ldelta;
					move=Back;
					break;
				case Back:
					localPose.x-=ldelta;
					move=Left;
					break;
				case Left:
					localPose.y-=ldelta;
					move=Right;
					break;
				case Right:
					localPose.y+=ldelta;
					move=TurnLeft;
					break;
				case TurnLeft:
					localPose.theta+=adelta;
					move=TurnRight;
					break;
				case TurnRight:
					localPose.theta-=adelta;
					move=Done;
					break;
				default:;
			}
			double localScore, localLikelihood;
			//update the score
			matched=likelihoodAndScore(localScore, localLikelihood, map, localPose, readings);
			if (localScore>currentScore){
				currentScore=localScore;
				bestLocalPose=localPose;
			}
			sm.score=localScore;
			sm.likelihood=localLikelihood;
			sm.pose=localPose;
			moveList.push_back(sm);
			//update the move list
		} while(move!=Done);
		currentPose=bestLocalPose;
		//cout << __func__ << "currentScore=" << currentScore<< endl;
		//here we look for the best move;
	}while (currentScore>bestScore || refinement<m_optRecursiveIterations);
	//cout << __func__ << "bestScore=" << bestScore<< endl;
	
	//normalize the likelihood
	double lmin=1e9;
	double lmax=-1e9;
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++){
		lmin=it->likelihood<lmin?it->likelihood:lmin;
		lmax=it->likelihood>lmax?it->likelihood:lmax;
	}
	//cout << "lmin=" << lmin << " lmax=" << lmax<< endl;
	for (ScoredMoveList::iterator it=moveList.begin(); it!=moveList.end(); it++){
		it->likelihood=exp(it->likelihood-lmax);
		//cout << "l=" << it->likelihood << endl;
	}
	//compute the mean
	OrientedPoint mean(0,0,0);
	double lacc=0;
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++){
		mean=mean+it->pose*it->likelihood;
		lacc+=it->likelihood;
	}
	mean=mean*(1./lacc);
	//OrientedPoint delta=mean-currentPose;
	//cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
	CovarianceMatrix cov={0.,0.,0.,0.,0.,0.};
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++){
		OrientedPoint delta=it->pose-mean;
		delta.theta=atan2(sin(delta.theta), cos(delta.theta));
		cov.xx+=delta.x*delta.x*it->likelihood;
		cov.yy+=delta.y*delta.y*it->likelihood;
		cov.tt+=delta.theta*delta.theta*it->likelihood;
		cov.xy+=delta.x*delta.y*it->likelihood;
		cov.xt+=delta.x*delta.theta*it->likelihood;
		cov.yt+=delta.y*delta.theta*it->likelihood;
	}
	cov.xx/=lacc, cov.xy/=lacc, cov.xt/=lacc, cov.yy/=lacc, cov.yt/=lacc, cov.tt/=lacc;
	
	_mean=currentPose;
	_cov=cov;
	return bestScore;
}

	void ScanMatcher::setLaserParameters
	(unsigned int beams, double* angles, const OrientedPoint& lpose){
	if (m_laserAngles)
		delete [] m_laserAngles;
	m_laserPose=lpose;
	m_laserBeams=beams;
	m_laserAngles=new double[beams];
	memcpy(m_laserAngles, angles, sizeof(double)*m_laserBeams);	
}
	

double ScanMatcher::likelihood
	(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings){
	ScoredMoveList moveList;
	
	
	for (double xx=-m_llsamplerange; xx<=m_llsamplerange; xx+=m_llsamplestep)
	for (double yy=-m_llsamplerange; yy<=m_llsamplerange; yy+=m_llsamplestep)
	for (double tt=-m_lasamplerange; tt<=m_lasamplerange; tt+=m_lasamplestep){
		
		OrientedPoint rp=p;
		rp.x+=xx;
		rp.y+=yy;
		rp.theta+=tt;
		
		ScoredMove sm;
		sm.pose=rp;
		
		likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
		moveList.push_back(sm);
	}
	
	//OrientedPoint delta=mean-currentPose;
	//cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
	//normalize the likelihood
	double lmax=-1e9;
	double lcum=0;
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++){
		lmax=it->likelihood>lmax?it->likelihood:lmax;
	}
	for (ScoredMoveList::iterator it=moveList.begin(); it!=moveList.end(); it++){
		//it->likelihood=exp(it->likelihood-lmax);
		lcum+=exp(it->likelihood-lmax);
		it->likelihood=exp(it->likelihood-lmax);
		//cout << "l=" << it->likelihood << endl;
	}
	
	OrientedPoint mean(0,0,0);
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++){
		mean=mean+it->pose*it->likelihood;
	}
	mean=mean*(1./lcum);
	
	CovarianceMatrix cov={0.,0.,0.,0.,0.,0.};
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++){
		OrientedPoint delta=it->pose-mean;
		delta.theta=atan2(sin(delta.theta), cos(delta.theta));
		cov.xx+=delta.x*delta.x*it->likelihood;
		cov.yy+=delta.y*delta.y*it->likelihood;
		cov.tt+=delta.theta*delta.theta*it->likelihood;
		cov.xy+=delta.x*delta.y*it->likelihood;
		cov.xt+=delta.x*delta.theta*it->likelihood;
		cov.yt+=delta.y*delta.theta*it->likelihood;
	}
	cov.xx/=lcum, cov.xy/=lcum, cov.xt/=lcum, cov.yy/=lcum, cov.yt/=lcum, cov.tt/=lcum;
	
	_mean=mean;
	_cov=cov;
	_lmax=lmax;
	return log(lcum)+lmax;
}

void ScanMatcher::setMatchingParameters
	(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations,  double likelihoodSigma, unsigned int likelihoodSkip){	
	m_usableRange=urange;
	m_laserMaxRange=range;
	m_kernelSize=kernsize;
	m_optLinearDelta=lopt;
	m_optAngularDelta=aopt;
	m_optRecursiveIterations=iterations;
	m_gaussianSigma=sigma;
	m_likelihoodSigma=likelihoodSigma;
	m_likelihoodSkip=likelihoodSkip;
}

};

