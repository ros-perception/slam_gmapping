#ifndef GRIDLINETRAVERSAL_H
#define GRIDLINETRAVERSAL_H

#include <cstdlib>
#include <gmapping/utils/point.h>

namespace GMapping {

typedef struct {
  int     num_points;
  IntPoint*  points;
} GridLineTraversalLine;

struct GridLineTraversal {
  inline static void gridLine( IntPoint start, IntPoint end, GridLineTraversalLine *line ) ;
  inline static void gridLineCore( IntPoint start, IntPoint end, GridLineTraversalLine *line ) ;

};

void GridLineTraversal::gridLineCore( IntPoint start, IntPoint end, GridLineTraversalLine *line )
{
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
  int cnt = 0;

  dx = abs(end.x-start.x); dy = abs(end.y-start.y);
  
  if (dy <= dx) {
    d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x; y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x; y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    line->points[cnt].x=x;
    line->points[cnt].y=y;
    cnt++;
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y++; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    } else {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y--; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    }		
  } else {
    d = 2*dx - dy;
    incr1 = 2*dx; incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y; x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y; x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    line->points[cnt].x=x;
    line->points[cnt].y=y;
    cnt++;
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x++; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    } else {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x--; d+=incr2;
	}
	line->points[cnt].x=x;
	line->points[cnt].y=y;
	cnt++;
      }
    }
  }
  line->num_points = cnt;
}

void GridLineTraversal::gridLine( IntPoint start, IntPoint end, GridLineTraversalLine *line ) {
  int i,j;
  int half;
  IntPoint v;
  gridLineCore( start, end, line );
  if ( start.x!=line->points[0].x ||
       start.y!=line->points[0].y ) {
    half = line->num_points/2;
    for (i=0,j=line->num_points - 1;i<half; i++,j--) {
      v = line->points[i];
      line->points[i] = line->points[j];
      line->points[j] = v;
    }
  }
}

};

#endif
