#include <stdlib.h>

//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist.h>
//#include <gsl/gsl_eigen.h>
//#include <gsl/gsl_blas.h>
#include <math.h>
#include "gmapping/utils/stat.h"

namespace GMapping {

#if 0

int sampleUniformInt(int max)
{
  return (int)(max*(rand()/(RAND_MAX+1.0)));
}

double sampleUniformDouble(double min, double max)
{
  return min + (rand() / (double)RAND_MAX) * (max - min);
}


#endif

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double sigma)
{
  double x1, x2, w;
  double r;

  do
  {
    do { r = drand48(); } while (r == 0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r == 0.0);
    x2 = 2.0 * drand48() - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}

double sampleGaussian(double sigma, unsigned long int S) {
  /*
	static gsl_rng * r = NULL;
	if(r==NULL) {
		gsl_rng_env_setup();
		r = gsl_rng_alloc (gsl_rng_default);
	}
        */
	if (S!=0)
        {
		//gsl_rng_set(r, S);
                srand48(S);
        }
	if (sigma==0)
		return 0;
	//return gsl_ran_gaussian (r,sigma);
	return pf_ran_gaussian (sigma);
}
#if 0

double evalGaussian(double sigmaSquare, double delta){
	if (sigmaSquare<=0)
		sigmaSquare=1e-4;
	return exp(-.5*delta*delta/sigmaSquare)/sqrt(2*M_PI*sigmaSquare);
}

#endif
double evalLogGaussian(double sigmaSquare, double delta){
	if (sigmaSquare<=0)
		sigmaSquare=1e-4;
	return -.5*delta*delta/sigmaSquare-.5*log(2*M_PI*sigmaSquare);
}
#if 0


Covariance3 Covariance3::zero={0.,0.,0.,0.,0.,0.};

Covariance3 Covariance3::operator + (const Covariance3 & cov) const{
	Covariance3 r(*this);
	r.xx+=cov.xx;
	r.yy+=cov.yy;
	r.tt+=cov.tt;
	r.xy+=cov.xy;
	r.yt+=cov.yt;
	r.xt+=cov.xt;
	return r;
}

EigenCovariance3::EigenCovariance3(){}

EigenCovariance3::EigenCovariance3(const Covariance3& cov){
	static gsl_eigen_symmv_workspace * m_eigenspace=NULL;
	static gsl_matrix * m_cmat=NULL;
	static gsl_matrix * m_evec=NULL;
	static gsl_vector * m_eval=NULL;
	static gsl_vector * m_noise=NULL;
	static gsl_vector * m_pnoise=NULL;
	
	if (m_eigenspace==NULL){
		m_eigenspace=gsl_eigen_symmv_alloc(3);
		m_cmat=gsl_matrix_alloc(3,3);
		m_evec=gsl_matrix_alloc(3,3);
		m_eval=gsl_vector_alloc(3);
		m_noise=gsl_vector_alloc(3);
		m_pnoise=gsl_vector_alloc(3);
	}

	gsl_matrix_set(m_cmat,0,0,cov.xx); gsl_matrix_set(m_cmat,0,1,cov.xy); gsl_matrix_set(m_cmat,0,2,cov.xt);
	gsl_matrix_set(m_cmat,1,0,cov.xy); gsl_matrix_set(m_cmat,1,1,cov.yy); gsl_matrix_set(m_cmat,1,2,cov.yt);
	gsl_matrix_set(m_cmat,2,0,cov.xt); gsl_matrix_set(m_cmat,2,1,cov.yt); gsl_matrix_set(m_cmat,2,2,cov.tt);
	gsl_eigen_symmv (m_cmat, m_eval,  m_evec, m_eigenspace);
	for (int i=0; i<3; i++){
		eval[i]=gsl_vector_get(m_eval,i);
		for (int j=0; j<3; j++)
			evec[i][j]=gsl_matrix_get(m_evec,i,j);
	}
}

EigenCovariance3 EigenCovariance3::rotate(double angle) const{
	static gsl_matrix * m_rmat=NULL;
	static gsl_matrix * m_vmat=NULL;
	static gsl_matrix * m_result=NULL;
	if (m_rmat==NULL){
		m_rmat=gsl_matrix_alloc(3,3);
		m_vmat=gsl_matrix_alloc(3,3);
		m_result=gsl_matrix_alloc(3,3);
	}
	
	double c=cos(angle);
	double s=sin(angle);
	gsl_matrix_set(m_rmat,0,0, c ); gsl_matrix_set(m_rmat,0,1, -s); gsl_matrix_set(m_rmat,0,2, 0.);
	gsl_matrix_set(m_rmat,1,0, s ); gsl_matrix_set(m_rmat,1,1,  c); gsl_matrix_set(m_rmat,1,2, 0.);
	gsl_matrix_set(m_rmat,2,0, 0.); gsl_matrix_set(m_rmat,2,1, 0.); gsl_matrix_set(m_rmat,2,2, 1.);
	
	for (unsigned int i=0; i<3; i++)
		for (unsigned int j=0; j<3; j++)
			gsl_matrix_set(m_vmat,i,j,evec[i][j]);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1., m_rmat, m_vmat, 0., m_result);
	EigenCovariance3 ecov(*this);
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++)
			ecov.evec[i][j]=gsl_matrix_get(m_result,i,j);
	}
	return ecov;
}

OrientedPoint EigenCovariance3::sample() const{
	static gsl_matrix * m_evec=NULL;
	static gsl_vector * m_noise=NULL;
	static gsl_vector * m_pnoise=NULL;
	if (m_evec==NULL){
		m_evec=gsl_matrix_alloc(3,3);
		m_noise=gsl_vector_alloc(3);
		m_pnoise=gsl_vector_alloc(3);
	}
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++)
			gsl_matrix_set(m_evec,i,j, evec[i][j]);
	}
	for (int i=0; i<3; i++){
		double v=sampleGaussian(sqrt(eval[i]));
		if(isnan(v))
			v=0;
		gsl_vector_set(m_pnoise,i, v);
	}
	gsl_blas_dgemv (CblasNoTrans, 1., m_evec, m_pnoise, 0, m_noise);
	OrientedPoint ret(gsl_vector_get(m_noise,0),gsl_vector_get(m_noise,1),gsl_vector_get(m_noise,2));
	ret.theta=atan2(sin(ret.theta), cos(ret.theta));
	return ret;
}

#endif

double Gaussian3::eval(const OrientedPoint& p) const{
	OrientedPoint q=p-mean;
	q.theta=atan2(sin(p.theta-mean.theta),cos(p.theta-mean.theta));
	double v1,v2,v3;
	v1 = covariance.evec[0][0]*q.x+covariance.evec[1][0]*q.y+covariance.evec[2][0]*q.theta;
	v2 = covariance.evec[0][1]*q.x+covariance.evec[1][1]*q.y+covariance.evec[2][1]*q.theta;
	v3 = covariance.evec[0][2]*q.x+covariance.evec[1][2]*q.y+covariance.evec[2][2]*q.theta;
	return evalLogGaussian(covariance.eval[0], v1)+evalLogGaussian(covariance.eval[1], v2)+evalLogGaussian(covariance.eval[2], v3);
}

#if 0
void Gaussian3::computeFromSamples(const std::vector<OrientedPoint> & poses, const std::vector<double>& weights ){
	OrientedPoint mean=OrientedPoint(0,0,0);
	double wcum=0;
	double s=0, c=0;
	std::vector<double>::const_iterator w=weights.begin();
	for (std::vector<OrientedPoint>::const_iterator p=poses.begin(); p!=poses.end(); p++){
		s+=*w*sin(p->theta);
		c+=*w*cos(p->theta);
		mean.x+=*w*p->x;
		mean.y+=*w*p->y;
		wcum+=*w;
		w++;
	}
	mean.x/=wcum;
	mean.y/=wcum;
	s/=wcum;
	c/=wcum;
	mean.theta=atan2(s,c);

	Covariance3 cov=Covariance3::zero;
	w=weights.begin();
	for (std::vector<OrientedPoint>::const_iterator p=poses.begin(); p!=poses.end(); p++){
		OrientedPoint delta=(*p)-mean;
		delta.theta=atan2(sin(delta.theta),cos(delta.theta));
		cov.xx+=*w*delta.x*delta.x;
		cov.yy+=*w*delta.y*delta.y;
		cov.tt+=*w*delta.theta*delta.theta;
		cov.xy+=*w*delta.x*delta.y;
		cov.yt+=*w*delta.y*delta.theta;
		cov.xt+=*w*delta.x*delta.theta;
		w++;
	}
	cov.xx/=wcum;
	cov.yy/=wcum;
	cov.tt/=wcum;
	cov.xy/=wcum;
	cov.yt/=wcum;
	cov.xt/=wcum;
	EigenCovariance3 ecov(cov);
	this->mean=mean;
	this->covariance=ecov;
	this->cov=cov;
}

void Gaussian3::computeFromSamples(const std::vector<OrientedPoint> & poses){
	OrientedPoint mean=OrientedPoint(0,0,0);
	double wcum=1;
	double s=0, c=0;
	for (std::vector<OrientedPoint>::const_iterator p=poses.begin(); p!=poses.end(); p++){
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
	for (std::vector<OrientedPoint>::const_iterator p=poses.begin(); p!=poses.end(); p++){
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
	this->mean=mean;
	this->covariance=ecov;
	this->cov=cov;
}
#endif

}// end namespace

