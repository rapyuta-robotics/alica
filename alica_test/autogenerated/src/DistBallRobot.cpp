/*
 * DistBallRobot.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Stefan Jakob
 */

#include "DistBallRobot.h"
#include "engine/IAssignment.h"
#include "engine/model/EntryPoint.h"
#include <TestWorldModel.h>

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
		this->robotId = 0;
		this->sb = 0;
	}

	DistBallRobot::~DistBallRobot()
	{
		// TODO Auto-generated destructor stub
	}

	void DistBallRobot::cacheEvalData()
	{
	}

	UtilityInterval* DistBallRobot::eval(IAssignment* ass)
	{
		ui->setMin(0.0);
		ui->setMax(1.0);
		int numAssignedRobots = 0;
		std::shared_ptr<vector<int>> relevantRobots = ass->getRobotsWorking(this->relevantEntryPoints[0]);

		double curPosition;
		for (int i = 0; i < relevantRobots->size(); ++i)
		{
			int pos = 0;
			if (relevantRobots->at(i) == 9)
			{
				pos = 1;
			}

			if (this->robotId == 8)
			{
				curPosition = alicaTests::TestWorldModel::getOne()->robotsXPos[pos];
			}
			else
			{
				curPosition = alicaTests::TestWorldModel::getTwo()->robotsXPos[pos];
			}
			//if no opp is near ball
			ui->setMin(std::max(ui->getMin(), 1 - fabs(sb - curPosition) / 18000));
			numAssignedRobots++;

		}
		ui->setMax(ui->getMin());
		if (this->relevantEntryPoints[0]->getMaxCardinality() > numAssignedRobots && ass->getNumUnAssignedRobots() > 0)
		{
			for (int i = 0; i < ass->getNumUnAssignedRobots(); ++i)
			{
				//curPosition = this.playerPositions.GetValue(ass.UnAssignedRobots[i]);
				if (this->robotId == 8)
				{
					curPosition = alicaTests::TestWorldModel::getOne()->robotsXPos.at(i);
				}
				else
				{
					curPosition = alicaTests::TestWorldModel::getTwo()->robotsXPos.at(i);
				}
				ui->setMax(std::max(ui->getMax(), 1 - fabs(sb - curPosition) / 18000));
			}
		}
		//			Console.WriteLine("DistBallRobot: UI is " + retUI.Min + ".." + retUI.Max); // DEBUG OUTPUT
		ui->setMin(std::max(0.0, ui->getMin()));
		ui->setMax(std::max(0.0, ui->getMax()));

		return ui;
	}

} /* namespace alica */
