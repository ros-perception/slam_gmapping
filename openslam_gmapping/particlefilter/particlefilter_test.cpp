#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "gmapping/particlefilter/particlefilter.h"

using namespace std;

#define test(s) {cout << s <<  " " << flush;}
#define testOk() {cout << "OK" << endl;}

struct Particle{
	double p;
	double w;
	inline operator double() const {return w;}
	inline void setWeight(double _w) {w=_w;}
};

ostream& printParticles(ostream& os, const vector<Particle>& p)
{
	for (vector<Particle>::const_iterator it=p.begin(); it!=p.end(); ++it) {
		os << it->p<< " " << (double)*it << endl;
	}
	return os;
}

struct EvolutionModel{
	Particle evolve(const Particle& p){
		Particle pn(p);
		pn.p+=.5*(drand48()-.5);
		return pn;
	}
};

struct QualificationModel{
	Particle evolve(const Particle& p){
		return p;
	}
};

struct LikelyhoodModel{
	double likelyhood(const Particle& p) const{
		double v = 1./(0.1+10*(p.p-2)*(p.p-2))+0.5/(0.1+10*(p.p-8)*(p.p-8));
		return v;
	}
};

int main (unsigned int argc, const char * const * argv){
	int nparticles=100;
	if (argc>1)
		nparticles=atoi(argv[1]);
	vector<Particle> particles(nparticles);
	LikelyhoodModel likelyhoodModel;
	uniform_resampler<Particle, double> resampler;
	auxiliary_evolver <Particle, double, QualificationModel, EvolutionModel, LikelyhoodModel> auxevolver;
	evolver <Particle, EvolutionModel> evolver;

	for (vector<Particle>::iterator it=particles.begin(); it!=particles.end(); it++){
		it->w=1;
		it->p=10*(drand48());
	}

	vector<Particle> sirparticles(particles);
	vector<Particle> auxparticles(particles);

	/*sir step*/
	while (1){
		char buf[2];
		cin.getline(buf,2);
		vector<Particle> newgeneration;

		cout << "# SIR step" << endl;
		evolver.evolve(sirparticles);
		for (vector<Particle>::iterator it=sirparticles.begin(); it!=sirparticles.end(); it++){
			it->setWeight(likelyhoodModel.likelyhood(*it));
		}
		ofstream os("sir.dat");
		printParticles(os, sirparticles);
		os.close();
		newgeneration=resampler.resample(sirparticles);
		sirparticles=newgeneration;

		cout << "# AUX step" << endl;
		auxevolver.evolve(auxparticles);
		for (vector<Particle>::iterator it=auxparticles.begin(); it!=auxparticles.end(); it++){
			it->setWeight(likelyhoodModel.likelyhood(*it));
		}
		os.open("aux.dat");
		printParticles(os, auxparticles);
		os.close();
		newgeneration=resampler.resample(auxparticles);
		auxparticles=newgeneration;
		cout << "plot [0:10][0:10]\"sir.dat\" w impulses" << endl;
		cout << "replot 1./(0.1+10*(x-2)*(x-2))+0.5/(0.1+10*(x-8)*(x-8))" << endl;
		
//		cout << "replot \"aux.dat\" w p" << endl;
	}
}

