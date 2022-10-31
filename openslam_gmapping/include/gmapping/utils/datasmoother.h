#ifndef DATASMOOTHER_H
#define DATASMOOTHER_H

#include <list>
#include <stdio.h>
#include <math.h>
#include <values.h>
#include "gmapping/utils/stat.h"
#include <assert.h>

namespace GMapping {

class DataSmoother {
 public: 
  struct DataPoint {
    DataPoint(double _x=0.0, double _y=0.0) { x=_x;y=_y;}
    double x;
    double y;
  };
  
  typedef std::vector<DataPoint> Data;

  DataSmoother(double parzenWindow) {
    init(parzenWindow);
  };

  virtual ~DataSmoother() {
    m_data.clear();
    m_cummulated.clear();
  };

  void init(double parzenWindow) {
    m_data.clear();
    m_cummulated.clear();
    m_int=-1; 
    m_parzenWindow = parzenWindow;
    m_from = MAXDOUBLE;
    m_to = -MAXDOUBLE;
    m_lastStep = 0.001;
  };


  double sqr(double x) {
    return x*x;
  }


  void setMinToZero() {
    double minval=MAXDOUBLE;

    for (Data::const_iterator it = m_data.begin(); it != m_data.end(); it++) {
      const DataPoint& d = *it;
      if (minval > d.y)
	minval = d.y;
    }

    for (Data::iterator it = m_data.begin(); it != m_data.end(); it++) {
      DataPoint& d = *it;
      d.y = d.y - minval;
    }

    m_cummulated.clear();
  }

  void add(double x, double p) {
    m_data.push_back(DataPoint(x,p));
    m_int=-1;
    
    if (x-3.0*m_parzenWindow < m_from)
      m_from = x - 3.0*m_parzenWindow;

    if (x+3.0*m_parzenWindow > m_to)
      m_to = x + 3.0*m_parzenWindow;

    m_cummulated.clear();
  }
   
  void integrate(double step) {
    m_lastStep = step;
    double sum=0;
    for (double x=m_from; x<=m_to; x+=step)
      sum += smoothedData(x)*step;
    m_int = sum;
  }

  double integral(double step, double xTo) {
    double sum=0;
    for (double x=m_from; x<=xTo; x+=step)
      sum += smoothedData(x)*step;
    return sum;
  }


  double smoothedData(double x) {
    assert( m_data.size() > 0 );

    double p=0;
    double sum_y=0;
    for (Data::const_iterator it = m_data.begin(); it != m_data.end(); it++) {
      const DataPoint& d = *it;
      double dist = fabs(x - d.x);
      p +=  d.y * exp( -0.5 * sqr ( dist/m_parzenWindow ) );
      sum_y += d.y;
    }
    double denom = sqrt(2.0 * M_PI) * (sum_y) * m_parzenWindow;
    p *= 1./denom; 

    return p;
  }

  double sampleNumeric(double step) {

    assert( m_data.size() > 0 );

    if (m_int <0 || step != m_lastStep)
      integrate(step);
    
    double r = sampleUniformDouble(0.0, m_int);
    double sum2=0;
    for (double x=m_from; x<=m_to; x+=step) {
      sum2 += smoothedData(x)*step;
      if (sum2 > r)
	return x-0.5*step;
    }
    return m_to;
  }

  void computeCummuated() {
    assert( m_data.size() > 0 );
    m_cummulated.resize(m_data.size());
    std::vector<double>::iterator cit = m_cummulated.begin();
    double sum=0;
    for (Data::const_iterator it = m_data.begin(); it != m_data.end(); ++it) {
      sum += it->y;
      (*cit) = sum;
      ++cit;
    }
  }
  
  double sample() {
    
    assert( m_data.size() > 0 );
    
    if (m_cummulated.size() == 0) {
      computeCummuated();
    }
    double maxval = m_cummulated.back();
    
    double random = sampleUniformDouble(0.0, maxval);
    int nCum = (int) m_cummulated.size();
    double sum=0;
    int i=0;
    while (i<nCum) {
      sum += m_cummulated[i];
      
      if  (sum >= random) {
	return m_data[i].x + sampleGaussian(m_parzenWindow);
      }
      i++;
    }
    assert(0);
  }


  void sampleMultiple(std::vector<double>& samples, int num) {
    
    assert( m_data.size() > 0 );
    samples.clear();
    
    if (m_cummulated.size() == 0) {
      computeCummuated();
    }
    double maxval = m_cummulated.back();
    
    std::vector<double> randoms(num);
    for (int i=0; i<num; i++)
      randoms[i] = sampleUniformDouble(0.0, maxval);
    
    std::sort(randoms.begin(), randoms.end());
    
    int nCum = (int) m_cummulated.size();
    
    double sum=0;
    int i=0;
    int j=0;
    while (i<nCum && j < num) {
      sum += m_cummulated[i];
      
      while (sum >= randoms[j] && j < num) {
	samples.push_back( m_data[i].x + sampleGaussian(m_parzenWindow) );
	j++;
      }
      i++;
    }
  }



  void approxGauss(double step, double* mean, double* sigma) {

    assert( m_data.size() > 0 );

    double sum=0;
    double d=0;

    *mean=0;
    for (double x=m_from; x<=m_to; x+=step) {
      d = smoothedData(x);
      sum += d;
      *mean += x*d;
    }
    *mean /= sum;

    double var=0;
    for (double x=m_from; x<=m_to; x+=step) {
      d = smoothedData(x);
      var += sqr(x-*mean) * d;
    }
    var /= sum;

    *sigma = sqrt(var);
  }

  double gauss(double x, double mean, double sigma) {
    return 1.0/(sqrt(2.0*M_PI)*sigma) * exp(-0.5 * sqr( (x-mean)/sigma));
  }

  double cramerVonMisesToGauss(double step, double mean, double sigma) {
    
    double p=0;
    double s=0;
    double g=0;
    double sint=0;
    double gint=0;
    
    for (double x=m_from; x<=m_to; x+=step) {
      s = smoothedData(x);
      sint += s * step;

      g = gauss(x, mean, sigma);
      gint += g * step;

      p += sqr( (sint - gint) );
    }
    
    return p;
  }

  double kldToGauss(double step, double mean, double sigma) {
    
    double p=0;
    double d=0;
    double g=0;

    double sd=0;
    double sg=0;

    for (double x=m_from; x<=m_to; x+=step) {

      d = 1e-10 + smoothedData(x);
      g = 1e-10 + gauss(x, mean, sigma);

      sd += d;
      sg += g;
      
      p += d * log(d/g);
    }
    
    sd *= step;
    sg *= step;

    if (fabs(sd-sg) > 0.1)
      assert(0);

    p *= step;
    return p;
  }
  
  
  void gnuplotDumpData(FILE* fp) {
    for (Data::const_iterator it = m_data.begin(); it != m_data.end(); it++) {
      const DataPoint& d = *it;
      fprintf(fp, "%f %f\n", d.x, d.y);
    }
  }

  void gnuplotDumpSmoothedData(FILE* fp, double step) {
    for (double x=m_from; x<=m_to; x+=step)
      fprintf(fp, "%f %f\n", x, smoothedData(x));
  }
  
 protected:
  Data m_data;
  std::vector<double> m_cummulated;
  double m_int;
  double m_lastStep;

  double m_parzenWindow;
  double m_from;
  double m_to;

};




/* class DataSmoother3D { */
/*  public:  */
/*   struct InputPoint { */
/*     InputPoint(double _x=0.0, double _y=0.0, double _t=0.0) { x=_x;y=_y;t=_t;} */
/*     double x; */
/*     double y; */
/*     double t; */
/*   }; */

/*   struct DataPoint { */
/*     DataPoint(const InputPoint& _p, double _val=0.0;)  { p=_p;val=_val;} */
/*     InputPoint p; */
/*     double val; */
/*   }; */
  
/*   typedef std::list<DataPoint> Data; */

/*   DataSmoother(double parzenWindow) { */
/*     m_int=-1;  */
/*     m_parzenWindow = parzenWindow; */
/*     m_from = InputPoint(MAXDOUBLE,MAXDOUBLE,MAXDOUBLE); */
/*     m_to = InputPoint(-MAXDOUBLE,-MAXDOUBLE,-MAXDOUBLE); */
/*   }; */

/*   virtual ~DataSmoother() { */
/*     m_data.clear(); */
/*   }; */

/*   double sqr(double x) { */
/*     return x*x; */
/*   } */


/*   void setMinToZero() { */
/*     double minval=MAXDOUBLE; */
/*     for (Data::const_iterator it = m_data.begin(); it != m_data.end(); it++) { */
/*       const DataPoint& d = *it; */
/*       if (minval > d.val) */
/* 	minval = d.val; */
/*     } */

/*     for (Data::iterator it = m_data.begin(); it != m_data.end(); it++) { */
/*       DataPoint& d = *it; */
/*       d.val = d.val - minval; */
/*     } */

/*   } */

/*   void add(double x, double y, double t, double v) { */
/*     m_data.push_back(DataPoint(InputPoint(x,y,t),v)); */
/*     m_int=-1; */
    
/*     if (x-3.0*m_parzenWindow < m_from.x) */
/*       m_from.x = x - 3.0*m_parzenWindow.x; */
/*     if (x+3.0*m_parzenWindow.x > m_to.x) */
/*       m_to.x = x + 3.0*m_parzenWindow.x; */

/*     if (y-3.0*m_parzenWindow < m_from.y) */
/*       m_from.y = y - 3.0*m_parzenWindow.y; */
/*     if (y+3.0*m_parzenWindow.y > m_to.y) */
/*       m_to.y = y + 3.0*m_parzenWindow.y; */

/*     if (t-3.0*m_parzenWindow < m_from.t) */
/*       m_from.t = t - 3.0*m_parzenWindow.t; */
/*     if (t+3.0*m_parzenWindow.t > m_to.t) */
/*       m_to.t = t + 3.0*m_parzenWindow.t; */
/*   } */
   
/*   void integrate(InputPoint step) { */
/*     m_lastStep = step; */
/*     double sum=0; */
/*     for (double x=m_from.x; x<=m_to.x; x+=step.x) { */
/*       for (double y=m_from.y; x<=m_to.y; y+=step.y) { */
/* 	for (double t=m_from.t; t<=m_to.t; t+=step.t) { */
/* 	  sum += smoothedData(InputPoint(x,y,t)) * step.x * step.y * step.t; */
/* 	} */
/*       } */
/*     } */
/*     m_int = sum; */
/*   } */


/*   double smoothedData(InputPoint pnt) { */
/*     assert( m_data.size() > 0 ); */
/*     double p=0; */
/*     double sum_y=0; */
/*     for (Data::const_iterator it = m_data.begin(); it != m_data.end(); it++) { */
/*       const DataPoint& d = *it; */
/*       double u = sqr( (pnt.x-d.x)/m_parzenWindow.x) +  */
/* 	sqr((pnt.y-d.y)/m_parzenWindow.y) +  */
/* 	sqr((pnt.t-d.t)/m_parzenWindow.t); */
/*       p +=  d.val * exp( -0.5 * u); */
/*       sum_y += d.y; */
/*     } */
/*     double denom = sqr(m_parzenWindow.x)*sqr(m_parzenWindow.x)*sqr(m_parzenWindow.x) * (sum_y) *  */
/*       sqrt(sqr(m_parzenWindow.x) + sqr(m_parzenWindow.y) + sqr(m_parzenWindow.t)); */
/*     p *= 1./denom;  */
    
/*     return p; */
/*   } */

/*   double sample(const InputPoint& step) { */

/*     assert( m_data.size() > 0 ); */

/*     if (m_int <0 || step != m_lastStep) */
/*       integrate(step); */
    
/*     double r = sampleUniformDouble(0.0, m_int); */
/*     double sum2=0; */
/*     for (double x=m_from; x<=m_to; x+=step) { */
/*       sum2 += smoothedData(x)*step; */
/*       if (sum2 > r) */
/* 	return x-0.5*step; */
/*     } */
/*     return m_to; */
/*   } */
  
/*   void gnuplotDumpData(FILE* fp) { */
/*     for (Data::const_iterator it = m_data.begin(); it != m_data.end(); it++) { */
/*       const DataPoint& d = *it; */
/*       fprintf(fp, "%f %f %f %f\n", d.x, d.y, d.t, d.val); */
/*     } */
/*   } */

/*   void gnuplotDumpSmoothedData(FILE* fp, double step) { */
/*     for (double x=m_from; x<=m_to; x+=step) */
/*       fprintf(fp, "%f %f %f %f\n", x, ,y, t, smoothedData(x,y,t)); */
/*   } */

/*  protected: */
/*   Data m_data; */
/*   vector<double> m_intdata; */
/*   double m_int; */
/*   double m_lastStep; */

/*   double m_parzenWindow; */
/*   double m_from; */
/*   double m_to; */

/* }; */

}

#endif
