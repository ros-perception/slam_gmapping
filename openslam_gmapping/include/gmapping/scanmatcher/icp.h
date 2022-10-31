#ifndef _ICP_H_
#define _ICP_H_

#include <gmapping/utils/point.h>
#include <utility>
#include <list>
#include <vector>

namespace GMapping{
typedef std::pair<Point,Point> PointPair;

template <typename PointPairContainer>
double icpStep(OrientedPoint & retval, const PointPairContainer& container){
	typedef typename PointPairContainer::const_iterator ContainerIterator;
	PointPair mean=std::make_pair(Point(0.,0.), Point(0.,0.));
	int size=0;
	for (ContainerIterator it=container.begin(); it!=container.end(); it++){
		mean.first=mean.first+it->first;
		mean.second=mean.second+it->second;
		size++;
	}
	mean.first=mean.first*(1./size);
	mean.second=mean.second*(1./size);
	double sxx=0, sxy=0, syx=0, syy=0;
	
	for (ContainerIterator it=container.begin(); it!=container.end(); it++){
		PointPair mf=std::make_pair(it->first-mean.first, it->second-mean.second);
		sxx+=mf.first.x*mf.second.x;
		sxy+=mf.first.x*mf.second.y;
		syx+=mf.first.y*mf.second.x;
		syy+=mf.first.y*mf.second.y;
	}
	retval.theta=atan2(sxy-syx, sxx+sxy);
	double s=sin(retval.theta), c=cos(retval.theta);
	retval.x=mean.second.x-(c*mean.first.x-s*mean.first.y);
	retval.y=mean.second.y-(s*mean.first.x+c*mean.first.y);
	
	double error=0;
	for (ContainerIterator it=container.begin(); it!=container.end(); it++){
		Point delta(
			c*it->first.x-s*it->first.y+retval.x-it->second.x, s*it->first.x+c*it->first.y+retval.y-it->second.y);
		error+=delta*delta;
	}
	return error;	
}

template <typename PointPairContainer>
double icpNonlinearStep(OrientedPoint & retval, const PointPairContainer& container){
	typedef typename PointPairContainer::const_iterator ContainerIterator;
	PointPair mean=std::make_pair(Point(0.,0.), Point(0.,0.));
	int size=0;
	for (ContainerIterator it=container.begin(); it!=container.end(); it++){
		mean.first=mean.first+it->first;
		mean.second=mean.second+it->second;
		size++;
	}
	
	mean.first=mean.first*(1./size);
	mean.second=mean.second*(1./size);
	
	double ms=0,mc=0;
	for (ContainerIterator it=container.begin(); it!=container.end(); it++){
		PointPair mf=std::make_pair(it->first-mean.first, it->second-mean.second);
		double  dalpha=atan2(mf.second.y, mf.second.x) - atan2(mf.first.y, mf.first.x);
		double gain=sqrt(mean.first*mean.first);
		ms+=gain*sin(dalpha);
		mc+=gain*cos(dalpha);
	}
	retval.theta=atan2(ms, mc);
	double s=sin(retval.theta), c=cos(retval.theta);
	retval.x=mean.second.x-(c*mean.first.x-s*mean.first.y);
	retval.y=mean.second.y-(s*mean.first.x+c*mean.first.y);
	
	double error=0;
	for (ContainerIterator it=container.begin(); it!=container.end(); it++){
		Point delta(
			c*it->first.x-s*it->first.y+retval.x-it->second.x, s*it->first.x+c*it->first.y+retval.y-it->second.y);
		error+=delta*delta;
	}
	return error;	
}

}//end namespace

#endif
