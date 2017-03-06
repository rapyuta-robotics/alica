#pragma once

#include "CNPositionAllo.h"
#include "CNVecAllo.h"

namespace geometry
{

class CNRobotAllo : public CNPositionAllo
{
  public:
    CNRobotAllo();
    virtual ~CNRobotAllo();
    virtual std::string toString() const;

    /**
     * The ID of this robot. Its either the robots id in our team, or some specific value for artificial robots or real
     * opponents.
     */
    int id;

    /**
     * Radius of the disc this robot fits into.
     */
    double radius;

    /**
     * Allocentric velocity of this robot.
     */
    CNVecAllo velocity;

    /**
     * Rotation velocity of this robot.
     */
    double rotationVel;

    /**
     * List of robots (identified by their id) that are not seeing this robot, although they should.
     */
    std::shared_ptr<std::vector<int>> opposer;

    /**
     * List of robots (identified by their id) that should be able to see this robot and also do so.
     */
    std::shared_ptr<std::vector<int>> supporter;

    /**
     * Some certainty value about the trueness of the data about this robot.
     */
    double certainty;
};
} /* namespace geometry */
