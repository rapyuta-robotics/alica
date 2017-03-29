#pragma once

#include "CNVecAllo.h"
#include "CNPositionAllo.h"

namespace geometry
{

class CNRobotEgo;

class CNRobotAllo
{
  public:
    CNRobotAllo();
    virtual ~CNRobotAllo();

    /**
     * Creates a string representation of this robot.
     * @return the string representing the robot.
     */
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
     * Allocentric position of this robot.
     */
    CNPositionAllo position;

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

    /**
     * Creates the egocentric representation of this Robot.
     */
    CNRobotEgo toEgo(CNPositionAllo ownPos);
};

} /* namespace geometry */
