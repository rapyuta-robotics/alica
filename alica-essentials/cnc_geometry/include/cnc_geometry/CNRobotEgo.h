#pragma once

#include "CNPositionEgo.h"
#include "CNVecEgo.h"

namespace geometry
{

class CNRobotEgo : public CNPositionEgo
{
  public:
    CNRobotEgo();
    virtual ~CNRobotEgo();
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
     * Egocentric velocity of this robot.
     */
    CNVecEgo velocity; // TODO: allo or ego?

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
