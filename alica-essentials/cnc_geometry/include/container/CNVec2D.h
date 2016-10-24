/*
 * CNVec2D.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNVEC2D_H_
#define CNC_GEOMETRY_CONTAINER_CNVEC2D_H_

#include "geometry_msgs/Point.h"

using namespace std;

namespace geometry
{

class CNPositionBase;

class CNVec2D : public geometry_msgs::Point
{
  public:
    CNVec2D(double x, double y);
    CNVec2D() : CNVec2D(0, 0) {}
    virtual ~CNVec2D();

    string toString();
    shared_ptr<CNVec2D> clone();

    double length();
    shared_ptr<CNVec2D> rotate(double radian);
    double angleTo();
    double angleToPoint(shared_ptr<CNVec2D> point);
    double distanceTo(shared_ptr<CNVec2D> point);
    double distanceTo(shared_ptr<CNPositionBase> pos);
    shared_ptr<CNVec2D> normalize();

    shared_ptr<CNVec2D> operator*(const double &right);
    shared_ptr<CNVec2D> operator/(const double &right);
    shared_ptr<CNVec2D> operator+(const shared_ptr<CNVec2D> &right);
    shared_ptr<CNVec2D> operator-(const shared_ptr<CNVec2D> &right);
    shared_ptr<CNVec2D> operator+(const shared_ptr<CNPositionBase> &right);
    shared_ptr<CNVec2D> operator-(const shared_ptr<CNPositionBase> &right);
};

shared_ptr<CNVec2D> operator+(const shared_ptr<CNVec2D> &left, const shared_ptr<CNVec2D> &right);
shared_ptr<CNVec2D> operator-(const shared_ptr<CNVec2D> &left, const shared_ptr<CNVec2D> &right);
shared_ptr<CNVec2D> operator+(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPositionBase> &right);
shared_ptr<CNVec2D> operator-(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPositionBase> &right);
shared_ptr<CNVec2D> operator+(const shared_ptr<CNPositionBase> &left, const shared_ptr<CNVec2D> &right);
shared_ptr<CNVec2D> operator-(const shared_ptr<CNPositionBase> &left, const shared_ptr<CNVec2D> &right);
shared_ptr<CNVec2D> operator*(const shared_ptr<CNVec2D> &left, const double &right);
shared_ptr<CNVec2D> operator/(const shared_ptr<CNVec2D> &left, const double &right);

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNVEC2D_H_ */
