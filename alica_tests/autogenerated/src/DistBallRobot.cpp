#include "DistBallRobot.h"
#include "engine/IAssignment.h"
#include "engine/model/EntryPoint.h"
#include <TestWorldModel.h>
#include <supplementary/AgentID.h>
#include <supplementary/AgentIDManager.h>

namespace alica
{

DistBallRobot::DistBallRobot(double weight, string name, long id, vector<long>& relevantEntryPointIds)
{
    this->weight = weight;
    this->name = name;
    this->id = id;
    this->relevantEntryPointIds = relevantEntryPointIds;
    this->angleBallOpp = 0;
    this->velAngle = 0;
    this->robotId = nullptr;
    this->sb = 0;
    this->manager = new supplementary::AgentIDManager(new supplementary::AgentIDFactory());
}

DistBallRobot::~DistBallRobot()
{
    delete manager;
}

void DistBallRobot::cacheEvalData() {}

UtilityInterval DistBallRobot::eval(IAssignment* ass)
{
    ui.setMin(0.0);
    ui.setMax(1.0);
    int numAssignedRobots = 0;

    long x8 = 8;
    std::vector<uint8_t> id8(reinterpret_cast<const uint8_t*>(&x8), (reinterpret_cast<const uint8_t*>(&x8) + sizeof(x8)));
    const supplementary::AgentID* agentID8 = this->manager->getIDFromBytes(id8);

    long x9 = 9;
    std::vector<uint8_t> id9(reinterpret_cast<const uint8_t*>(&x9), (reinterpret_cast<const uint8_t*>(&x9) + sizeof(x9)));
    const supplementary::AgentID* agentID9 = this->manager->getIDFromBytes(id9);

    const AgentGrp* relevantRobots = ass->getRobotsWorking(this->relevantEntryPoints[0]);

    double curPosition;
    for (int i = 0; i < static_cast<int>(relevantRobots->size()); ++i) {
        int pos = 0;
        if (*(relevantRobots->at(i)) == *agentID9) {
            pos = 1;
        }

        if (*(this->robotId) == *agentID8) {
            curPosition = alicaTests::TestWorldModel::getOne()->robotsXPos[pos];
        } else {
            curPosition = alicaTests::TestWorldModel::getTwo()->robotsXPos[pos];
        }
        // if no opp is near ball
        ui.setMin(std::max(ui.getMin(), 1 - fabs(sb - curPosition) / 18000));
        numAssignedRobots++;
    }
    ui.setMax(ui.getMin());
    if (this->relevantEntryPoints[0]->getMaxCardinality() > numAssignedRobots && ass->getNumUnAssignedRobotIds() > 0) {
        for (int i = 0; i < ass->getNumUnAssignedRobotIds(); ++i) {
            // curPosition = this.playerPositions.GetValue(ass.UnAssignedRobots[i]);
            if (*(this->robotId) == *agentID8) {
                curPosition = alicaTests::TestWorldModel::getOne()->robotsXPos.at(i);
            } else {
                curPosition = alicaTests::TestWorldModel::getTwo()->robotsXPos.at(i);
            }
            ui.setMax(std::max(ui.getMax(), 1 - fabs(sb - curPosition) / 18000));
        }
    }
    //			Console.WriteLine("DistBallRobot: UI is " + retUI.Min + ".." + retUI.Max); // DEBUG OUTPUT
    ui.setMin(std::max(0.0, ui.getMin()));
    ui.setMax(std::max(0.0, ui.getMax()));

    return ui;
}

} /* namespace alica */
