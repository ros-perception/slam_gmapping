#include "gmapping/utils/movement.h"
#include "gmapping/utils/gvalues.h"

namespace GMapping {


FSRMovement::FSRMovement(double f, double s, double r) {
  this->f = f;     
  this->s = s;       
  this->r = r;
}

FSRMovement::FSRMovement(const FSRMovement& src) {
  *this = src;
}

FSRMovement::FSRMovement(const OrientedPoint& pt1, const OrientedPoint& pt2) {
  *this = moveBetweenPoints(pt1, pt2);
}


FSRMovement::FSRMovement(const FSRMovement& move1, const FSRMovement& move2) {
  *this = composeMoves(move1, move2);
}

void FSRMovement::normalize()
{
  if (r >= -M_PI && r < M_PI)
    return;
  
  int multiplier = (int)(r / (2*M_PI));
  r = r - multiplier*2*M_PI;
  if (r >= M_PI)
    r -= 2*M_PI;
  if (r < -M_PI)
    r += 2*M_PI;
}

OrientedPoint FSRMovement::move(const OrientedPoint& pt) const {
  return movePoint(pt, *this);
}

void FSRMovement::invert() {
  *this = invertMove(*this);
}

void FSRMovement::compose(const FSRMovement& move2) {
  *this = composeMoves(*this, move2);
}


FSRMovement FSRMovement::composeMoves(const FSRMovement& move1, 
				      const FSRMovement& move2) {
  FSRMovement comp;
  comp.f = cos(move1.r) * move2.f - sin(move1.r) * move2.s + move1.f;
  comp.s = sin(move1.r) * move2.f + cos(move1.r) * move2.s + move1.s;
  comp.r = (move1.r  + move2.r);
  comp.normalize();
  return comp;
}

OrientedPoint FSRMovement::movePoint(const OrientedPoint& pt, const FSRMovement& move1) {
  OrientedPoint pt2(pt);
  pt2.x += move1.f * cos(pt.theta) - move1.s * sin(pt.theta);
  pt2.y += move1.f * sin(pt.theta) + move1.s * cos(pt.theta);
  pt2.theta = (move1.r + pt.theta);
  pt2.normalize();
  return pt2;
}

FSRMovement FSRMovement::moveBetweenPoints(const OrientedPoint& pt1, 
					   const OrientedPoint& pt2) {
  FSRMovement move;
  move.f =   (pt2.y - pt1.y) * sin(pt1.theta) + (pt2.x - pt1.x) * cos(pt1.theta);
  move.s = + (pt2.y - pt1.y) * cos(pt1.theta) - (pt2.x - pt1.x) * sin(pt1.theta);
  move.r = (pt2.theta - pt1.theta);
  move.normalize();
  return move;
  
}

FSRMovement FSRMovement::invertMove(const FSRMovement& move1) {
  FSRMovement p_inv;
  p_inv.f = - cos(move1.r) * move1.f - sin(move1.r) * move1.s;
  p_inv.s =   sin(move1.r) * move1.f - cos(move1.r) * move1.s;
  p_inv.r = (-move1.r);
  p_inv.normalize();
  return p_inv;
}


OrientedPoint FSRMovement::frameTransformation(const OrientedPoint& reference_pt_frame1, 
					       const OrientedPoint& reference_pt_frame2,
					       const OrientedPoint& pt_frame1) {
  OrientedPoint zero;

  FSRMovement itrans_refp1(zero, reference_pt_frame1);
  itrans_refp1.invert();

  FSRMovement trans_refp2(zero, reference_pt_frame2);
  FSRMovement trans_pt(zero, pt_frame1);

  FSRMovement tmp = composeMoves( composeMoves(trans_refp2, itrans_refp1), trans_pt);
  return tmp.move(zero);
}
 

}
