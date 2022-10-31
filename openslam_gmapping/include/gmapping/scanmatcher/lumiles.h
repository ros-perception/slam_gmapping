#ifndef LUMILESPROCESSOR
#define LUMILESPROCESSOR

namespace GMapping{

class LuMilesProcessor{
	typedef std:vector<Point> PointVector;
	static OrientedPoint step(const PointVector& src, const PointVector& dest);
};

OrientedPoint LuMilesProcessors::step(const PointVector& src, const PointVector& dest){
	assert(src.size()==dest.size());
	unsigned int size=dest.size();
	double smx=0, smy=0, dmx=0, dmy=0;
	for (PointVector::const_iterator it=src.begin(); it!=src.end(); it++){
		smx+=it->x;
		smy+=it->y;
	}
	smx/=src.size();
	smy/=src.size();
	
	for (PointVector::const_iterator it=dest.begin(); it!=dest.end(); it++){
		dmx+=it->x;
		dmy+=it->y;
	}
	dmx/=src.size();
	dmy/=src.size();
	
	double sxx=0, sxy=0;
	double syx=0, syy=0;
	for (unsigned int i=0; i<size(); i++){
		sxx+=(src[i].x-smx)*(dest[i].x-dmx);
		sxy+=(src[i].x-smx)*(dest[i].y-dmy);
		syx+=(src[i].y-smy)*(dest[i].x-dmx);
		syy+=(src[i].y-smy)*(dest[i].y-dmy);
	}
	double omega=atan2(sxy-syx,sxx+syy);
	return OrientedPoint(
		dmx-smx*cos(omega)+smx*sin(omega)),
		dmy-smx*sin(omega)-smy*cos(omega)),
		omega
	)
};

int main(int argc, conat char ** argv){
}

};

#endif