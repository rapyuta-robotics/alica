/*
 * CNPositionEgo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONEGO_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONEGO_H_

#include "container/CNPositionBase.h"
#include "container/CNVec2D.h"

namespace geometry
{

class CNPositionAllo;

class CNPositionEgo : public CNPositionBase
{
  public:
    CNPositionEgo() : CNPositionEgo(0, 0, 0) {}
    CNPositionEgo(double x, double y, double theta) : CNPositionEgo(x, y, theta) {};
    virtual ~CNPositionEgo();

    string toString();

    double distanceTo(shared_ptr<CNPositionEgo> egoPos);
    shared_ptr<CNPositionAllo> toAllo(CNPositionAllo &origin);

    /* Operators */

    // CNPositionEgo
    shared_ptr<CNPositionEgo> operator+(const shared_ptr<CNPositionEgo> &right);
    shared_ptr<CNPositionEgo> operator-(const shared_ptr<CNPositionEgo> &right);

    // CNVec2D
    shared_ptr<CNPositionEgo> operator+(const shared_ptr<CNVec2D> &right);
    shared_ptr<CNPositionEgo> operator-(const shared_ptr<CNVec2D> &right);
};

/* Right handed operators */

// CNVec2D
shared_ptr<CNPositionEgo> operator+(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPositionEgo> &right);
shared_ptr<CNPositionEgo> operator-(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPositionEgo> &right);

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONEGO_H_ */
