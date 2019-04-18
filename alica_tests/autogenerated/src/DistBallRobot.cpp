#include "DistBallRobot.h"
#include "TestWorldModel.h"
#include <essentials/AgentIDConstPtr.h>
#include <engine/model/EntryPoint.h>
#include <engine/planselector/IAssignment.h>

#include <essentials/AgentIDManager.h>

namespace alica
{

DistBallRobot::DistBallRobot(double weight)
        : USummand(weight)
        , angleBallOpp(0)
        , velAngle(0)
        , robotId(nullptr)
        , sb(0)
        , manager(new essentials::AgentIDManager(new essentials::AgentIDFactory()))
{
}

DistBallRobot::~DistBallRobot()
{
    delete manager;
}

UtilityInterval DistBallRobot::eval(IAssignment ass) const
{
    UtilityInterval ui(0.0, 1.0);

    int numAssignedRobots = 0;

    long x8 = 8;
    std::vector<uint8_t> id8(reinterpret_cast<const uint8_t*>(&x8), (reinterpret_cast<const uint8_t*>(&x8) + sizeof(x8)));
    essentials::AgentIDConstPtr agentID8 = this->manager->getIDFromBytes(id8);

    long x9 = 9;
    std::vector<uint8_t> id9(reinterpret_cast<const uint8_t*>(&x9), (reinterpret_cast<const uint8_t*>(&x9) + sizeof(x9)));
    essentials::AgentIDConstPtr agentID9 = this->manager->getIDFromBytes(id9);

    double curPosition;
    for (essentials::AgentIDConstPtr id : ass.getAgentsWorking(_relevantEntryPoints[0])) {
        int pos = 0;
        if (*id == *agentID9) {
            pos = 1;
        }

        if (*(this->robotId) == *agentID8) {
            curPosition = alicaTests::TestWorldModel::getOne()->robotsXPos[pos];
        } else {
            curPosition = alicaTests::TestWorldModel::getTwo()->robotsXPos[pos];
        }
        // if no opp is near ball
        ui.setMin(std::max(ui.getMin(), 1 - std::abs(sb - curPosition) / 18000));
        ++numAssignedRobots;
    }
    ui.setMax(ui.getMin());
    if (_relevantEntryPoints[0]->getMaxCardinality() > numAssignedRobots && ass.getUnAssignedAgentCount() > 0) {
        for (int i = 0; i < ass.getUnAssignedAgentCount(); ++i) {
            // curPosition = this.playerPositions.GetValue(ass.UnAssignedRobots[i]);
            if (*(this->robotId) == *agentID8) {
                curPosition = alicaTests::TestWorldModel::getOne()->robotsXPos.at(i);
            } else {
                curPosition = alicaTests::TestWorldModel::getTwo()->robotsXPos.at(i);
            }
            ui.setMax(std::max(ui.getMax(), 1 - std::abs(sb - curPosition) / 18000));
        }
    }

    ui.setMin(std::max(0.0, ui.getMin()));
    ui.setMax(std::max(0.0, ui.getMax()));

    return ui;
}

} /* namespace alica */
