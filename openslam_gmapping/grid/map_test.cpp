#include <iostream>
#include "gmapping/grid/map.h"
#include "gmapping/grid/harray2d.h"

using namespace std;
using namespace GMapping;

struct SimpleCell{
	int value;
	SimpleCell(int v=0){value=v;}
	static const SimpleCell& Unknown();
	static SimpleCell* address;
};

SimpleCell* SimpleCell::address=0;

const SimpleCell& SimpleCell::Unknown(){
	if (address)
		return *address;
	address=new SimpleCell(-1);
	return *address;
}

typedef Map< SimpleCell, HierarchicalArray2D<SimpleCell> > CGrid;

int main (int argc, char ** argv){
	CGrid g1(Point(0.,0.), 200, 200, 0.1);
	CGrid g2(Point(10.,10.), 200, 200, 0.1);
	{
		HierarchicalArray2D<SimpleCell>::PointSet ps;
		IntPoint pp=g1.world2map(Point(5.1,5.1));
		cout << pp.x << " " << pp.y << endl;
		ps.insert(pp);
		g1.storage().setActiveArea(ps,false);
		g1.storage().allocActiveArea();
		g1.cell(Point(5.1,5.1)).value=5;
		cout << "cell value" << (int) g1.cell(Point(5.1,5.1)).value << endl;
		g1.resize(-150, -150, 150, 150);
		cout << "cell value" << (int) g1.cell(Point(5.1,5.1)).value << endl;
		CGrid g3(g1);
		g1=g2;
	}
	cerr << "copy and modify test" << endl;
	CGrid *ap,* gp1=new CGrid(Point(0,0), 200, 200, 0.1);
        CGrid* gp0=new CGrid(*gp1);
	for (int i=1; i<10; i++){
	  ap=new CGrid(*gp1);
	  delete gp1;
	  gp1=gp0;
	  gp0=ap;
	  IntPoint pp=gp0->world2map(Point(5.1,5.1));
	  HierarchicalArray2D<SimpleCell>::PointSet ps;
	  ps.insert(pp);
          gp1->storage().setActiveArea(ps,false);	
          gp1->storage().allocActiveArea();
	  gp1->cell(Point(5.1,5.1)).value=i;
          cout << "cell value" << (int) gp1->cell(Point(5.1,5.1)).value << endl;
	}
	delete gp0;
	delete gp1;
	return 0;
}
