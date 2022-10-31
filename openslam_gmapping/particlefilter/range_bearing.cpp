#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <gmapping/utils/point.h>
#include "gmapping/particlefilter/particlefilter.h"

using namespace std;
using namespace GMapping;

#define test(s) {cout << s <<  " " << flush;}
#define testOk() {cout << "OK" << endl;}

struct Particle{
	Particle(): p(0,0), w(0){}
	Point p;
	double w;
	operator double() const {return w; }
	void setWeight(double _w) {w=_w;}
};

ostream& printParticles(ostream& os, const vector<Particle>& p)
{
	for (vector<Particle>::const_iterator it=p.begin(); it!=p.end(); ++it) {
		os << it->p.x << " " << it->p.y << endl;
	}
	return os;
}

struct EvolutionModel{
	Particle evolve(const Particle& p){
		Particle pn(p);
		pn.p.x+=10*(drand48()-.5);
		pn.p.y+=10*(drand48()-.5);
		return pn;
	}
};


struct LikelyhoodModel{
	std::vector<Point> observerVector;
	std::vector<double> observations;
	double sigma;
	double likelyhood(const Particle& p) const{
		double v=1;
		std::vector<double>::const_iterator oit=observations.begin();
		for (std::vector<Point>::const_iterator it=observerVector.begin(); it!=observerVector.end();it++){
			v*=exp(-pow(((p.p-*it)*(p.p-*it)-*oit*(*oit))/sigma, 2));
			oit++;
		}
		cout << "#v=" << v << endl;
		return v;
	}
};

int main (unsigned int argc, const char * const * argv){
	vector<Particle> particles(1000);
	LikelyhoodModel likelyhoodModel;
	uniform_resampler<Particle, double> resampler;
	evolver <Particle, EvolutionModel> evolver;

	for (vector<Particle>::iterator it=particles.begin(); it!=particles.end(); it++){
		it->w=1;
		it->p.x=400*(drand48()-.5);
		it->p.y=400*(drand48()-.5);
	}
	
	vector<Point> sensors;
	
	sensors.push_back(Point(-50,0));
	sensors.push_back(Point(50,0));
	sensors.push_back(Point(0,100));
	
	likelyhoodModel.sigma=1000;
	likelyhoodModel.observations.push_back(70);
	likelyhoodModel.observations.push_back(70);
	likelyhoodModel.observations.push_back(70);
	
	likelyhoodModel.observerVector=sensors;
	while (1){
		char buf[2];
		cin.getline(buf,2);
		vector<Particle> newgeneration;

		cout << "# SIR step" << endl;
		evolver.evolve(particles);
		for (vector<Particle>::iterator it=particles.begin(); it!=particles.end(); it++){
			it->w*=likelyhoodModel.likelyhood(*it);
		}
		
		ofstream os("sir.dat");
		printParticles(os, particles);
		os.close();
		vector<Particle> newpart=resampler.resample(particles);
		particles=newpart;

		cout << "plot [-200:200][-200:200]\"sir.dat\" w p" << endl;
	}
}

