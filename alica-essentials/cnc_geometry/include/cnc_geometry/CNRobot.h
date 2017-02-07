#pragma once

#include <cnc_geometry/CNVecAllo.h>
#include "CNPositionAllo.h"

namespace geometry
{

class CNRobot : public CNPositionAllo
{
  public:
    CNRobot();
    virtual ~CNRobot();
    std::string toString();

    int id;               /**< The ID of this robot. Its either the robots id in our team, or some specific value for artificial robots or real opponents.*/
    double radius;        /**< Radius of the disc this robot fits into.*/
    CNVecAllo velocity; /**< Allocentric velocity of this robot.  */
    double rotation;      /**< Rotation velocity of this robot. */

    std::shared_ptr<std::vector<int>> opposer;   /**< List of robots (identified by their id) that are not seeing this robot, although they should.*/
    std::shared_ptr<std::vector<int>> supporter; /**< List of robots (identified by their id) that should be able to see this robot and also do so.*/
    double certainty;                  /**< Some certainty value about the trueness of the data about this robot.*/
};

}
