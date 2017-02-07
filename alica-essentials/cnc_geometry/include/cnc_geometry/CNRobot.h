/*
 * CNRobot.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_
#define SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_

#include "cnc_geometry/container/CNPositionAllo.h"
#include "cnc_geometry/container/CNVec2DAllo.h"

#include <memory>
#include <string>
#include <vector>

using std::shared_ptr;
using std::vector;
using std::string;

namespace geometry
{
class CNRobot : public CNPositionAllo
{
  public:
    CNRobot();
    virtual ~CNRobot();
    string toString();

    int id;               /**< The ID of this robot. Its either the robots id in our team, or some specific value for artificial robots or real opponents.*/
    double radius;        /**< Radius of the disc this robot fits into.*/
    CNVec2DAllo velocity; /**< Allocentric velocity of this robot.  */
    double rotation;      /**< Rotation velocity of this robot. */

    shared_ptr<vector<int>> opposer;   /**< List of robots (identified by their id) that are not seeing this robot, although they should.*/
    shared_ptr<vector<int>> supporter; /**< List of robots (identified by their id) that should be able to see this robot and also do so.*/
    double certainty;                  /**< Some certainty value about the trueness of the data about this robot.*/
};
}

#endif /* SUPPLEMENTARY_CNC_GEOMETRY_SRC_CONTAINER_CNROBOT_H_ */
