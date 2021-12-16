#include <alica_tests/DistBallRobot.h>
#include <alica_tests/TestWorldModel.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/model/EntryPoint.h>
#include <engine/planselector/IAssignment.h>

namespace alica
{

DistBallRobot::DistBallRobot(double weight)
        : USummand(weight)
        , angleBallOpp(0)
        , velAngle(0)
        , robotId(0)
        , sb(0)
{
}

DistBallRobot::~DistBallRobot() {}

UtilityInterval DistBallRobot::eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel* wm) const
{
    UtilityInterval ui(0.0, 1.0);

    int numAssignedRobots = 0;

    AgentId agentID8 = 8;

    AgentId agentID9 = 9;

    double curPosition;
    for (AgentId id : ass.getAgentsWorking(_relevantEntryPoints[0])) {
        int pos = 0;
        if (id == agentID9) {
            pos = 1;
        }

        if (this->robotId == agentID8) {
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
            if (this->robotId == agentID8) {
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
