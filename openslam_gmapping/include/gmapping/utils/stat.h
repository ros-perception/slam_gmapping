#ifndef STAT_H
#define STAT_H
#include "gmapping/utils/point.h"
#include <vector>
#include "gmapping/utils/gvalues.h"
#include <gmapping/utils/utils_export.h>

namespace GMapping {

/**stupid utility function for drawing particles form a zero mean, sigma variance normal distribution
probably it should not go there*/
double UTILS_EXPORT sampleGaussian(double sigma,unsigned long int S=0);

double UTILS_EXPORT evalGaussian(double sigmaSquare, double delta);
double UTILS_EXPORT evalLogGaussian(double sigmaSquare, double delta);
int UTILS_EXPORT sampleUniformInt(int max);
double UTILS_EXPORT sampleUniformDouble(double min, double max);

struct Covariance3{
	Covariance3 operator + (const Covariance3 & cov) const;
	static Covariance3 zero;
	double xx, yy, tt, xy, xt, yt;
};

struct EigenCovariance3{
	EigenCovariance3();
	EigenCovariance3(const Covariance3& c);
	EigenCovariance3 rotate(double angle) const;
	OrientedPoint sample() const;
	double eval[3];
	double evec[3][3];
};

struct Gaussian3{
	OrientedPoint mean;
	EigenCovariance3 covariance;
	Covariance3 cov;
	double UTILS_EXPORT eval(const OrientedPoint& p) const;
	void computeFromSamples(const std::vector<OrientedPoint> & poses);
	void computeFromSamples(const std::vector<OrientedPoint> & poses, const std::vector<double>& weights );
};

template<typename PointIterator, typename WeightIterator>
Gaussian3 computeGaussianFromSamples(PointIterator& pointBegin, PointIterator& pointEnd, WeightIterator& weightBegin, WeightIterator& weightEnd){
	Gaussian3 gaussian;
	OrientedPoint mean=OrientedPoint(0,0,0);
	double wcum=0;
	double s=0, c=0;
	WeightIterator wt=weightBegin;
	double *w=new double();
	OrientedPoint *p=new OrientedPoint();
	for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
		*w=*wt;
		*p=*pt;
		s+=*w*sin(p->theta);
		c+=*w*cos(p->theta);
		mean.x+=*w*p->x;
		mean.y+=*w*p->y;
		wcum+=*w;
		wt++;
	}
	mean.x/=wcum;
	mean.y/=wcum;
	s/=wcum;
	c/=wcum;
	mean.theta=atan2(s,c);

	Covariance3 cov=Covariance3::zero;
	wt=weightBegin;
	for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
		*w=*wt;
		*p=*pt;
		OrientedPoint delta=(*p)-mean;
		delta.theta=atan2(sin(delta.theta),cos(delta.theta));
		cov.xx+=*w*delta.x*delta.x;
		cov.yy+=*w*delta.y*delta.y;
		cov.tt+=*w*delta.theta*delta.theta;
		cov.xy+=*w*delta.x*delta.y;
		cov.yt+=*w*delta.y*delta.theta;
		cov.xt+=*w*delta.x*delta.theta;
		wt++;
	}
	cov.xx/=wcum;
	cov.yy/=wcum;
	cov.tt/=wcum;
	cov.xy/=wcum;
	cov.yt/=wcum;
	cov.xt/=wcum;
	EigenCovariance3 ecov(cov);
	gaussian.mean=mean;
	gaussian.covariance=ecov;
	gaussian.cov=cov;
	delete w;
	delete p;
	return gaussian;
}

template<typename PointIterator>
Gaussian3 computeGaussianFromSamples(PointIterator& pointBegin, PointIterator& pointEnd){
	Gaussian3 gaussian;
	OrientedPoint mean=OrientedPoint(0,0,0);
	double wcum=1;
	double s=0, c=0;
	OrientedPoint *p=new OrientedPoint();
	for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
		*p=*pt;
		s+=sin(p->theta);
		c+=cos(p->theta);
		mean.x+=p->x;
		mean.y+=p->y;
		wcum+=1.;
	}
	mean.x/=wcum;
	mean.y/=wcum;
	s/=wcum;
	c/=wcum;
	mean.theta=atan2(s,c);

	Covariance3 cov=Covariance3::zero;
	for (PointIterator pt=pointBegin; pt!=pointEnd; pt++){
		*p=*pt;
		OrientedPoint delta=(*p)-mean;
		delta.theta=atan2(sin(delta.theta),cos(delta.theta));
		cov.xx+=delta.x*delta.x;
		cov.yy+=delta.y*delta.y;
		cov.tt+=delta.theta*delta.theta;
		cov.xy+=delta.x*delta.y;
		cov.yt+=delta.y*delta.theta;
		cov.xt+=delta.x*delta.theta;
	}
	cov.xx/=wcum;
	cov.yy/=wcum;
	cov.tt/=wcum;
	cov.xy/=wcum;
	cov.yt/=wcum;
	cov.xt/=wcum;
	EigenCovariance3 ecov(cov);
	gaussian.mean=mean;
	gaussian.covariance=ecov;
	gaussian.cov=cov;
	delete p;
	return gaussian;
}


}; //end namespace
#endif

