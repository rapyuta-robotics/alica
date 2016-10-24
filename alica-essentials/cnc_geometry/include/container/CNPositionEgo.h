#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITION_EGO_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITION_EGO_H_

#include "container/CNPositionBase.h"

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

    shared_ptr<CNPositionEgo> operator+(const shared_ptr<CNPositionEgo> &right);
    shared_ptr<CNPositionEgo> operator-(const shared_ptr<CNPositionEgo> &right);
};

shared_ptr<CNPositionEgo> operator+(const shared_ptr<CNPositionEgo> &left, const shared_ptr<CNPositionEgo> &right);
shared_ptr<CNPositionEgo> operator-(const shared_ptr<CNPositionEgo> &left, const shared_ptr<CNPositionEgo> &right);

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITION_EGO_H_ */
