#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_

#include "container/CNPositionBase.h"

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

    shared_ptr<CNPositionAllo> operator+(const shared_ptr<CNPositionAllo> &right);
    shared_ptr<CNPositionAllo> operator-(const shared_ptr<CNPositionAllo> &right);
};

shared_ptr<CNPositionAllo> operator+(const shared_ptr<CNPositionAllo> &left, const shared_ptr<CNPositionAllo> &right);
shared_ptr<CNPositionAllo> operator-(const shared_ptr<CNPositionAllo> &left, const shared_ptr<CNPositionAllo> &right);

} /* namespace */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONALLO_H_ */
