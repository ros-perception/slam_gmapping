template <class NUMERIC>
double OrientedBoundingBox<NUMERIC>::area() {
  return  sqrt((ul.x - ll.x)*(ul.x - ll.x) + (ul.y - ll.y)*(ul.y - ll.y)) *
    sqrt((ul.x - ur.x)*(ul.x - ur.x) + (ul.y - ur.y)*(ul.y - ur.y)) ;
}

template<class NUMERIC>
OrientedBoundingBox<NUMERIC>::OrientedBoundingBox(std::vector< point<NUMERIC> > p) {
  
  int nOfPoints = (int)  p.size();
  
  // calculate the center of all points (schwerpunkt)
  // -------------------------------------------------
  double centerx = 0;
  double centery = 0;
  for (int i=0; i < nOfPoints; i++) {
    centerx += p[i].x;
    centery += p[i].y;
  }    
  centerx /= (double) nOfPoints;
  centery /= (double) nOfPoints;
  

  
  // calcutae the covariance matrix
  // -------------------------------
  // covariance matrix (x1 x2, x3 x4) 
  double x1 = 0.0;
  double x2 = 0.0;
  double x3 = 0.0;
  double x4 = 0.0;

  for (int i=0; i < nOfPoints; i++) {
    double cix = p[i].x - centerx;
    double ciy = p[i].y - centery;
    
    x1 += cix*cix;
    x2 += cix*ciy;  
    x4 += ciy*ciy;
  }
  x1 /= (double) nOfPoints;
  x2 /= (double) nOfPoints;
  x3 = x2;
  x4 /= (double) nOfPoints;
  // covariance & center  done


  // calculate the eigenvectors
  // ---------------------------
  // catch 1/0 or sqrt(<0)
  if ((x3 == 0) || (x2 == 0)|| (x4*x4-2*x1*x4+x1*x1+4*x2*x3 < 0 )) {
    fprintf(stderr,"error computing the Eigenvectors (%s, line %d)\nx3=%lf, x2=%lf, term=%lf\n\n",
	    __FILE__, __LINE__,   x3,x2,  (x4*x4-2*x1*x4+x1*x1+4*x2*x3)  );
  
    ul.x = 0;
    ul.y = 0;
    ur.x = 0;
    ur.y = 0;
    ll.x = 0;
    ll.y = 0;
    lr.x = 0;
    lr.y = 0;
  }
 
 // eigenvalues
  double lamda1 = 0.5* (x4 + x1 + sqrt(x4*x4 - 2.0*x1*x4 + x1*x1 + 4.0*x2*x3));
  double lamda2 = 0.5* (x4 + x1 - sqrt(x4*x4 - 2.0*x1*x4 + x1*x1 + 4.0*x2*x3));
  
  // eigenvector 1  with  (x,y)
  double v1x = - (x4-lamda1) * (x4-lamda1) * (x1-lamda1) / (x2 * x3 * x3);
  double v1y = (x4-lamda1) * (x1-lamda1) / (x2 * x3);
  // eigenvector 2 with  (x,y)
  double v2x = - (x4-lamda2) * (x4-lamda2) * (x1-lamda2) / (x2 * x3 * x3);
  double v2y = (x4-lamda2) * (x1-lamda2) / (x2 * x3);

  // norm the eigenvectors
  double lv1 = sqrt ( (v1x*v1x) + (v1y*v1y) );
  double lv2 = sqrt ( (v2x*v2x) + (v2y*v2y) );
  v1x /= lv1;
  v1y /= lv1;
  v2x /= lv2;
  v2y /= lv2;
  // eigenvectors done

  // get the points with maximal dot-product 
  double x = 0.0;
  double y = 0.0;
  double xmin = 1e20;
  double xmax = -1e20;
  double ymin = 1e20;
  double ymax = -1e20;
  for(int i = 0; i< nOfPoints; i++) {
    // dot-product of relativ coordinates of every point
    x = (p[i].x - centerx) * v1x +  (p[i].y - centery) * v1y;
    y = (p[i].x - centerx) * v2x +  (p[i].y - centery) * v2y;

    if( x > xmax) xmax = x;
    if( x < xmin) xmin = x;
    if( y > ymax) ymax = y;
    if( y < ymin) ymin = y;
  }

  // now we can compute the corners of the bounding box
  ul.x = centerx + xmin * v1x + ymin * v2x;
  ul.y = centery + xmin * v1y + ymin * v2y;

  ur.x = centerx + xmax * v1x + ymin * v2x;
  ur.y = centery + xmax * v1y + ymin * v2y;

  ll.x = centerx + xmin * v1x + ymax * v2x;
  ll.y = centery + xmin * v1y + ymax * v2y;

  lr.x = centerx + xmax * v1x + ymax * v2x;
  lr.y = centery + xmax * v1y + ymax * v2y;

}
