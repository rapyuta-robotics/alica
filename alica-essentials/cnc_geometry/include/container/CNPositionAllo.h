/*
 * CNPositionAllo.h
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_

#include "container/CNPositionBase.h"
#include "container/CNVec2D.h"

namespace geometry
{

class CNPositionEgo;

class CNPositionAllo : public CNPositionBase
{
  public:
    CNPositionAllo() : CNPositionAllo(0, 0, 0) {};
    CNPositionAllo(double x, double y, double theta) : CNPositionBase(x, y, theta) {};
    virtual ~CNPositionAllo();

    string toString();

    double distanceTo(shared_ptr<CNPositionAllo> alloPos);
    shared_ptr<CNPositionEgo> toEgo(CNPositionAllo &origin);

    /* Operators */

    // CNPositionAllo
    shared_ptr<CNPositionAllo> operator+(const shared_ptr<CNPositionAllo> &right);
    shared_ptr<CNPositionAllo> operator-(const shared_ptr<CNPositionAllo> &right);

    // CNVec2D
    shared_ptr<CNPositionAllo> operator+(const shared_ptr<CNVec2D> &right);
    shared_ptr<CNPositionAllo> operator-(const shared_ptr<CNVec2D> &right);
};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_ */
