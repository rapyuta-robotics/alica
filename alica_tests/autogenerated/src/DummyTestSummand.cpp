/*
 * DummyTestSummand.cpp
 *
 *  Created on: Oct 29, 2014
 *      Author: Stefan Jakob
 */

#include "DummyTestSummand.h"
#include "engine/model/EntryPoint.h"
#include "engine/planselector/IAssignment.h"
#include "essentials/AgentID.h"
#include <TestWorldModel.h>

namespace alica
{

DummyTestSummand::DummyTestSummand(double weight)
        : USummand(weight)
{
    this->angleBallOpp = 0;
    this->velAngle = 0;
    this->robotId = nullptr;
    this->sb = 0;
}

DummyTestSummand::~DummyTestSummand() {}

UtilityInterval DummyTestSummand::eval(IAssignment ass) const
{
    UtilityInterval ui(0.0, 1.0);

    for (essentials::AgentIDConstPtr agent : ass.getAgentsWorking(_relevantEntryPoints[0])) {
        if (agent == this->robotId) {
            ui.setMin(0.5);
        } else {
            ui.setMin(0.0);
        }
    }
    if (_relevantEntryPoints.size() > 1) {
        for (essentials::AgentIDConstPtr agent : ass.getAgentsWorking(_relevantEntryPoints[1])) {
            if (agent != this->robotId) {
                ui.setMin(ui.getMin() + 0.5);
            }
        }
    }
    ui.setMax(ui.getMin());
    return ui;
}

} /* namespace alica */
