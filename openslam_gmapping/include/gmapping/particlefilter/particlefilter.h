#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H
#include <stdlib.h>
#include <float.h>
#include <sys/types.h>
#include<vector>
#include<utility>
#include<cmath>
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
	//double lmax=-MAXDOUBLE;
    double lmax=-DBL_MAX;
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

template <typename Vector>
void repeatIndexes(Vector& dest, const std::vector<int>& indexes, const Vector& particles){
	assert(indexes.size()==particles.size());
	dest.resize(particles.size());
	unsigned int i=0;
	for (std::vector<int>::const_iterator it=indexes.begin(); it!=indexes.end(); ++it){
		dest[i]=particles[*it];
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

template <class Iterator>
void normalize(const Iterator& begin, const Iterator& end){
	double sum=0;
	for (Iterator it=begin; it!=end; ++it){
		sum+=*it;
	}
	for (Iterator it=begin; it!=end; ++it){
		*it=*it/sum;
	}
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

//BEGIN legacy
template <class Particle, class Numeric>
struct uniform_resampler{
	std::vector<unsigned int> resampleIndexes(const std::vector<Particle> & particles, int nparticles=0) const;
	std::vector<Particle> resample(const std::vector<Particle> & particles, int nparticles=0) const;
	Numeric neff(const std::vector<Particle> & particles) const;
};

/*Implementation of the above stuff*/
template <class Particle, class Numeric>
std::vector<unsigned int> uniform_resampler<Particle, Numeric>:: resampleIndexes(const std::vector<Particle>& particles, int nparticles) const{
	Numeric cweight=0;

	//compute the cumulative weights
	unsigned int n=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it){
		cweight+=(Numeric)*it;
		n++;
	}

	if (nparticles>0)
		n=nparticles;
	
	//compute the interval
	Numeric interval=cweight/n;

	//compute the initial target weight
	Numeric target=interval*::drand48();
	//compute the resampled indexes

	cweight=0;
	std::vector<unsigned int> indexes(n);
	n=0;
	unsigned int i=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it, ++i){
		cweight+=(Numeric)* it;
		while(cweight>target){
			indexes[n++]=i;
			target+=interval;
		}
	}
	return indexes;
}

template <class Particle, class Numeric>
std::vector<Particle> uniform_resampler<Particle,Numeric>::resample
  (const typename std::vector<Particle>& particles, int nparticles) const{
	Numeric cweight=0;

	//compute the cumulative weights
	unsigned int n=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it){
		cweight+=(Numeric)*it;
		n++;
	}

	if (nparticles>0)
		n=nparticles;
	
	//weight of the particles after resampling
	double uw=1./n;

	//compute the interval
	Numeric interval=cweight/n;

	//compute the initial target weight
	Numeric target=interval*::drand48();
	//compute the resampled indexes

	cweight=0;
	std::vector<Particle> resampled;
	n=0;
	unsigned int i=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it, ++i){
		cweight+=(Numeric)*it;
		while(cweight>target){
			resampled.push_back(*it);
			resampled.back().setWeight(uw);
			target+=interval;
		}
	}
	return resampled;
}

template <class Particle, class Numeric>
Numeric uniform_resampler<Particle,Numeric>::neff(const std::vector<Particle> & particles) const{
	double cum=0;
	double sum=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it){
		Numeric w=(Numeric)*it;
		cum+=w*w;
		sum+=w;
	}
	return sum*sum/cum;
}


/*

The following are patterns for the evolution and the observation classes
The user should implement classes having the specified meaning

template <class State, class Numeric, class Observation>
struct observer{
	Observation& observation
	Numeric observe(const class State&) const;
};

template <class State, class Numeric, class Input>
struct evolver{
	Input& input;
	State& evolve(const State& s);
};
*/


template <class Particle, class EvolutionModel>
struct evolver{
	EvolutionModel evolutionModel;
	void evolve(std::vector<Particle>& particles);
	void evolve(std::vector<Particle>& dest, const std::vector<Particle>& src);
};

template <class Particle, class EvolutionModel>
void evolver<Particle, EvolutionModel>::evolve(std::vector<Particle>& particles){
	for (typename std::vector<Particle>::iterator it=particles.begin(); it!=particles.end(); ++it){
		*it=evolutionModel.evolve(*it);
	}
}

template <class Particle, class EvolutionModel>
void evolver<Particle, EvolutionModel>::evolve(std::vector<Particle>& dest, const std::vector<Particle>& src){
	dest.clear();
	for (typename std::vector<Particle>::const_iterator it=src.begin(); it!=src.end(); ++it)
		dest.push_back(evolutionModel.evolve(*it));
}


template <class Particle, class Numeric, class QualificationModel, class EvolutionModel, class LikelyhoodModel>
struct auxiliary_evolver{
	EvolutionModel evolutionModel;
	QualificationModel qualificationModel;
	LikelyhoodModel likelyhoodModel;
	void evolve(std::vector<Particle>& particles);
	void evolve(std::vector<Particle>& dest, const std::vector<Particle>& src);
};

template <class Particle, class Numeric, class QualificationModel, class EvolutionModel, class LikelyhoodModel>
void auxiliary_evolver<Particle, Numeric, QualificationModel, EvolutionModel, LikelyhoodModel>::evolve
  (std::vector<Particle>&particles){
	std::vector<Numeric> observationWeights(particles.size());
	unsigned int i=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it, i++){
		observationWeights[i]=likelyhoodModel.likelyhood(qualificationModel.evolve(*it));
	}
	uniform_resampler<Numeric, Numeric> resampler;
	std::vector<unsigned int> indexes(resampler.resampleIndexes(observationWeights));
	for (typename std::vector<unsigned int>::const_iterator it=indexes.begin(); it!=indexes.end(); it++){
		Particle & particle=particles[*it];
		particle=evolutionModel.evolve(particle);
		particle.setWeight(likelyhoodModel.likelyhood(particle)/observationWeights[*it]);
	}
}

template <class Particle, class Numeric, class QualificationModel, class EvolutionModel, class LikelyhoodModel>
void auxiliary_evolver<Particle, Numeric, QualificationModel, EvolutionModel, LikelyhoodModel>::evolve
  (std::vector<Particle>& dest, const std::vector<Particle>& src){
	dest.clear();
	std::vector<Numeric> observationWeights(src.size());
	unsigned int i=0;
	for (typename std::vector<Particle>::const_iterator it=src.begin(); it!=src.end(); ++it, i++){
		observationWeights[i]=likelyhoodModel.likelyhood(qualificationModel.evolve(*it));
	}
	uniform_resampler<Numeric, Numeric> resampler;
	std::vector<unsigned int> indexes(resampler.resampleIndexes(observationWeights));
	for (typename std::vector<unsigned int>::const_iterator it=indexes.begin(); it!=indexes.end(); it++){
		Particle & particle=src[*it];
		dest.push_back(evolutionModel.evolve(particle));
		dest.back().weight*=likelyhoodModel.likelyhood(particle)/observationWeights[*it];
	}
}
//END legacy

#endif
