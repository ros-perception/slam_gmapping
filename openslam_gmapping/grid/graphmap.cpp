#ifndef GRAPHMAP_H
#define GRAPHMAP_H
#include <list>
#include <gmapping/utils/point.h>
#include <utils/graph.h>
#include <gmapping/grid/map.h>

namespace GMapping {

class RasterMap;

struct GraphMapPatch{
	typedef typename std::list<IntPoint> PointList; 
	/**Renders the map relatively to the center of the patch*/
	//void render(RenderMap rmap);
	/**returns the lower left corner of the patch, relative to the center*/
	//Point minBoundary() const; 
	/**returns the upper right corner of the patch, relative to the center*/
	//Point maxBoundary() const; // 

	OrientedPoint center;
	PointList m_points;
};

struct Covariance3{
	double sxx, sxy, sxt, syy, syt ,stt;
};

struct GraphMapEdge{
	Covariance3 covariance;
	GraphMapPatch* first, *second;
	inline operator double() const{
		return sqrt((first->center-second->center)*(first->center-second->center));
	}
};


struct GraphPatchGraph: public Graph<GraphMapPatch, Covariance3>{
	void addEdge(Vertex* v1, Vertex* v2, const Covariance3& covariance);
};

void GraphPatchGraph::addEdge(GraphPatchGraph::Vertex* v1, GraphPatchGraph::VertexVertex* v2, 
	const Covariance3& cov){
	GraphMapEdge gme;
	gme.covariance=cov;
	gme.first=v1;
	gme.second=v2;
	return Graph<GraphMapPatch, Covariance3>::addEdge(v1,v2,gme);
}

struct GraphPatchDirectoryCell: public std::set<GraphMapPatch::Vertex*> {
	GraphPatchDirectoryCell(double);
};

typedef Map<GraphPatchDirectoryCell>, Array2D::set<GraphPatchDirectoryCell> >

};

#endif