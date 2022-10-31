std::vector<unsigned int> sistematicResampler<State,Numeric>::resample(const vector<Particle>& particles) const{
	Numeric cweight=0;

	//compute the cumulative weights
	unsigned int n=0;
	for (vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it){
		cweight+=it->weight;
		n++;
	}

	//compute the interval
	Numeric interval=cweight/n;

	//compute the initial target weight
	Numeric target=
	//compute the resampled indexes

	cweight=0;
	std::vector<int> indexes(n);
	n=0;
	unsigned int i=0;
	for (vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it, ++i){
		cweight+=it->weight;
		while(cweight>target){
			indexes[n++]=i;
			target+=interval;
		}
	}
	return indexes;
	}

template <class Numeric>
std::vector<unsigned int> indexResampler<Numeric>::resample(const vector<Numeric> >& weights) const{
	Numeric cweight=0;

	//compute the cumulative weights
	unsigned int n=0;
	for (vector<Numeric>::const_iterator it=weights.begin(); it!=weights.end(); ++it){
		cweight+=*it;
		n++;
	}

	//compute the interval
	Numeric interval=cweight/n;

	//compute the initial target weight
	Numeric target=
	//compute the resampled indexes

	cweight=0;
	std::vector<int> indexes(n);
	n=0;
	unsigned int i=0;
	for (vector<Numeric>::const_iterator it=weights.begin(); it!=weights.end(); ++it, ++i){
		cweight+=it->weight;
		while(cweight>target){
			indexes[n++]=i;
			target+=interval;
		}
	}
	return indexes;
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

template <class State, class Numeric, class EvolutionModel>
void evolver<State, Numeric, EvolutionModel>::evolve(std::vector<evolver::Particle>& particles) const{
	for (std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it)
		*it=evolutionModel.evolve(*it);
}

void evolver<State, Numeric, EvolutionModel>::evolve(std::vector<evolver::Particle>& dest, const std::vector<evolver::Particle>& src) const{
	dest.clear();
	for (std::vector<Particle>::const_iterator it=src.begin(); it!=src.end(); ++it)
		dest.push_back(evolutionModel.evolve(*it));
}

template <class State, class Numeric, class QualificationModel, class EvolutionModel, class LikelyhoodModel>
struct auxiliaryEvolver{
	typedef particle<State, Numeric> Particle;

	EvolutionModel evolutionModel;
	QualificationModel qualificationModel;
	LikelyhoodModel likelyhoodModel;
	indexResampler<Numeric> resampler;

void auxiliaryEvolver<State, Numeric, QualificationModel, EvolutionModel, LikelyhoodModel>::evolve
  (std::vector<auxiliaryEvolver::Particle>&particles){
	std::vector<Numeric> observationWeights(particles.size());
	unsigned int i=0;
	for (std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it, i++){
		observationWeights[i]=likelyhoodModel.likelyhood(qualificationModel.evolve(*it));
	}
	std::vector<unsigned int> indexes(indexResampler.resample(observationWeights));
	for (std::vector<unsigned int>::const_iterator it=indexes.begin(); it!=indexes.end(); it++){
		Particle & particle=particles[*it];
		particle=evolutionModel.evolve(particle);
		particle.weight*=lykelyhoodModel.lykelyhood(particle)/observationWeights[*it];
	}
}

void auxiliaryEvolver<State, Numeric, QualificationModel, EvolutionModel, LikelyhoodModel>::evolve
  (std::vector<Particle>& dest, const std::vector<Particle>& src){
	dest.clear();
	std::vector<Numeric> observationWeights(particles.size());
	unsigned int i=0;
	for (std::vector<Particle>::const_iterator it=src.begin(); it!=src.end(); ++it, i++){
		observationWeights[i]=likelyhoodModel.likelyhood(qualificationModel.evolve(*it));
	}
	std::vector<unsigned int> indexes(indexResampler.resample(observationWeights));
	for (std::vector<unsigned int>::const_iterator it=indexes.begin(); it!=indexes.end(); it++){
		Particle & particle=src[*it];
		dest.push_back(evolutionModel.evolve(particle));
		dest.back().weight*=likelyhoodModel.lykelyhood(particle)/observationWeights[*it];
	}
	return dest();
}
