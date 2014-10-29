/*
 * DummyTestSummand.cpp
 *
 *  Created on: Oct 29, 2014
 *      Author: Stefan Jakob
 */

#include "DummyTestSummand.h"
#include "engine/IAssignment.h"
#include "engine/model/EntryPoint.h"
#include <TestWorldModel.h>

namespace alica
{

	DummyTestSummand::DummyTestSummand(double weight, string name, long id, vector<long>& relevantEntryPointIds)
	{
		this->weight = weight;
		this->name = name;
		this->id = id;
		this->relevantEntryPointIds = relevantEntryPointIds;
		this->angleBallOpp = 0;
		this->velAngle = 0;
		this->robotId = 0;
		this->sb = 0;
	}

	DummyTestSummand::~DummyTestSummand()
	{
		// TODO Auto-generated destructor stub
	}

	void DummyTestSummand::cacheEvalData()
	{
	}

	UtilityInterval* DummyTestSummand::eval(IAssignment* ass)
	{
		ui->setMin(0.0);
		ui->setMax(1.0);
		std::shared_ptr<vector<int>> relevantRobots = ass->getRobotsWorking(this->relevantEntryPoints[0]);

		for (int i = 0; i < relevantRobots->size(); ++i)
		{
			int pos = 0;
			if (relevantRobots->at(i) == this->robotId)
			{
				ui->setMin(1.0);
			}
			else
			{
				ui->setMin(0.0);
			}

		}

		ui->setMax(ui->getMin());
		return ui;
	}

} /* namespace alica */
