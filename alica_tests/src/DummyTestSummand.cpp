/*
 * DummyTestSummand.cpp
 *
 *  Created on: Oct 29, 2014
 *      Author: Stefan Jakob
 */

#include <alica_tests/DummyTestSummand.h>
#include "engine/model/EntryPoint.h"
#include "engine/planselector/IAssignment.h"
#include <alica_tests/TestWorldModel.h>

namespace alica
{

DummyTestSummand::DummyTestSummand(double weight)
        : USummand(weight)
{
    this->angleBallOpp = 0;
    this->velAngle = 0;
    this->robotId = 0;
    this->sb = 0;
}

DummyTestSummand::~DummyTestSummand() {}

UtilityInterval DummyTestSummand::eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel* wm) const
{
    UtilityInterval ui(0.0, 1.0);

    for (AgentId agent : ass.getAgentsWorking(_relevantEntryPoints[0])) {
        if (agent == this->robotId) {
            ui.setMin(0.5);
        } else {
            ui.setMin(0.0);
        }
    }
    if (_relevantEntryPoints.size() > 1) {
        for (AgentId agent : ass.getAgentsWorking(_relevantEntryPoints[1])) {
            if (agent != this->robotId) {
                ui.setMin(ui.getMin() + 0.5);
            }
        }
    }
    ui.setMax(ui.getMin());
    return ui;
}

} /* namespace alica */
