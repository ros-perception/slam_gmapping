#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H
#include <stdlib.h>
#include<vector>
#include<utility>
#include<gmapping/utils/gvalues.h>



/**
the particle class has to be convertible into numeric data type;
That means that a particle must define the Numeric conversion operator;
	operator Numeric() const.
that returns the weight, and the method
	setWeight(Numeric)
that sets the weight.

*/

typedef std::pair<uint,uint> UIntPair;

template <class OutputIterator, class Iterator>
double toNormalForm(OutputIterator& out, const Iterator & begin, const Iterator & end){
	//determine the maximum
	double lmax=-MAXDOUBLE;
	for (Iterator it=begin; it!=end; it++){
		lmax=lmax>((double)(*it))? lmax: (double)(*it);
	}
	//convert to raw form
	for (Iterator it=begin; it!=end; it++){
		*out=exp((double)(*it)-lmax);
		out++;
	}
	return lmax;
}

template <class OutputIterator, class Iterator,	class Numeric>
void toLogForm(OutputIterator& out, const Iterator & begin, const Iterator & end, Numeric lmax){
	//determine the maximum
	for (Iterator it=begin; it!=end; it++){
		*out=log((Numeric)(*it))-lmax;
		out++;
	}
	return lmax;
}

template <class WeightVector>
void resample(std::vector<int>& indexes, const WeightVector& weights, unsigned int nparticles=0){
	double cweight=0;
	
	//compute the cumulative weights
	unsigned int n=0;
	for (typename WeightVector::const_iterator it=weights.begin(); it!=weights.end(); ++it){
		cweight+=(double)*it;
		n++;
	}

	if (nparticles>0)
		n=nparticles;
	
	//compute the interval
	double interval=cweight/n;

	//compute the initial target weight
	double target=interval*::drand48();
	//compute the resampled indexes

	cweight=0;
	indexes.resize(n);
	
	n=0;
	unsigned int i=0;
	for (typename WeightVector::const_iterator it=weights.begin(); it!=weights.end(); ++it, ++i){
		cweight+=(double)* it;
		while(cweight>target){
			indexes[n++]=i;
			target+=interval;
		}
	}
}

template <typename WeightVector>
void normalizeWeights(WeightVector& weights, unsigned int size, double minWeight){
	double wmin=MAXDOUBLE;
	double wmax=-MAXDOUBLE;
	for (uint i=0; i<size; i++){
		wmin=wmin<weights[i]?wmin:weights[i];
		wmax=wmax>weights[i]?wmax:weights[i];
	}
	double min_normalized_value=log(minWeight);
	double max_normalized_value=log(1.);
	double dn=max_normalized_value-min_normalized_value;
	double dw=wmax-wmin;
	if (dw==0) dw=1;
	double scale=dn/dw;
	double offset=-wmax*scale;
	for (uint i=0; i<size; i++){
		double w=weights[i];
		w=scale*w+offset;
		weights[i]=exp(w);
	}
}

template <typename Vector>
void repeatIndexes(Vector& dest, const std::vector<int>& indexes, const Vector& particles){
/*<<<<<<< .mine
	assert(indexes.size()==particles.size());
	if (dest.size()!=particles.size())
		dest.resize(particles.size());
=======*/
	//assert(indexes.size()==particles.size()); //DIEGO non ne vedo il senso, anzi è sbagliata
	//dest.resize(particles.size());	    //      è sbagliato anche questo
	dest.resize(indexes.size());
// >>>>>>> .r2534
	unsigned int i=0;
	for (std::vector<int>::const_iterator it=indexes.begin(); it!=indexes.end(); ++it){
		dest[i]=particles[*it];
		i++;
	}
}

template <typename Vector>
void repeatIndexes(Vector& dest, const std::vector<int>& indexes2, const Vector& particles, const std::vector<int>& indexes){
  //	assert(indexes.size()==indexes2.size());
	dest=particles;
	unsigned int i=0;
	for (std::vector<int>::const_iterator it=indexes2.begin(); it!=indexes2.end(); ++it){
		dest[indexes[i]]=particles[*it];
		i++;
	}
}


template <class Iterator>
double neff(const Iterator& begin, const Iterator& end){
	double sum=0;
	for (Iterator it=begin; it!=end; ++it){
		sum+=*it;
	}
	double cum=0;
	for (Iterator it=begin; it!=end; ++it){
		double w=*it/sum;
		cum+=w*w;
	}
	return 1./cum;
}



template <class OutputIterator, class Iterator>
void rle(OutputIterator& out, const Iterator & begin, const Iterator & end){
	unsigned int current=0;
	unsigned int count=0;
	for (Iterator it=begin; it!=end; it++){
		if (it==begin){
			current=*it;
			count=1;
			continue;
		}
		if (((uint)*it) ==current)
			count++;
		if (((uint)*it)!=current){
			*out=std::make_pair(current,count);
			out++;
			current=*it;
			count=1;
		}
	}
	if (count>0)
		*out=std::make_pair(current,count);
		out++;
}

#endif

