#ifndef _GVALUES_H_
#define _GVALUES_H_

#ifdef LINUX
	#include <values.h>
#endif
#ifdef MACOSX
	#include <limits.h>
	#include <math.h>
	#define MAXDOUBLE 1e1000
	//#define isnan(x) (x==FP_NAN)
#endif
#ifdef _WIN32
  #include <limits>
  #ifndef __DRAND48_DEFINED__
     #define __DRAND48_DEFINED__
     inline double drand48() { return double(rand()) / RAND_MAX;}
     inline void srand48(unsigned int seed) { srand(seed); }
  #endif
  #ifndef M_PI
    #define M_PI 3.1415926535897932384626433832795
  #endif
  #define round(d) (floor((d) + 0.5))
  typedef unsigned int uint;
  #define isnan(x) (_isnan(x))
#endif

#endif 

