/*
 * DistBallRobot.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Stefan Jakob
 */

#include "DistBallRobot.h"

namespace alica
{

	DistBallRobot::DistBallRobot(double weight, string name, long id, vector<long> relevantEntryPointIds)
	{
		this->weight = weight;
		this->name = name;
		this->id = id;
		this->relevantEntryPointIds = relevantEntryPointIds;
		this->validAngle = false;
		this->angleBallOpp = 0;
		this->velAngle = 0;
	}

	DistBallRobot::~DistBallRobot()
	{
		// TODO Auto-generated destructor stub
	}

	void DistBallRobot::cacheEvalData()
	{
	}

	UtilityInterval DistBallRobot::Eval(IAssignment* ass)
	{
//		ui.setMin(0.0);
//		ui.setMax(1.0);
//		int numAssignedRobots = 0;
//		shared_ptr<vector<int>> relevantRobots = ass->getRobotsWorking(this->relevantEntryPoints[0]);
//
//		pair<double, double> curPosition;
//		for (int i = 0; i < relevantRobots->size(); ++i)
//		{
//			//curPosition = this.playerPositions.GetValue(relevantRobotsList[i]);
//			curPosition = this.shwm.GetRobotDataByID(relevantRobotsList[i]).playerPosition;
//			if (curPosition == null)
//				continue; // This player was not 'positionReceived'
//			Velocity v = this.shwm.GetRobotDataByID(relevantRobotsList[i]).ballVelocity;
//			Point2D alloPoint = this.shwm.GetRobotDataByID(relevantRobotsList[i]).ballPosition;
//
//			double util = 0.05;
//			if (v != null && alloPoint != null && v.Length() > 350.0)
//			{
//				Point2D hisBall = WorldHelper.Allo2Ego(alloPoint, curPosition);
//				util = hisBall.X * v.Vx + hisBall.Y * v.Vy / (hisBall.Distance() * v.Length());
//				util = Math.Max(0.2, (-util + 1.0) / 2.0);
//			}
//			// SET UI.MIN
//			if (!validAngle)
//			{
//				//if no opp is near ball
//				ui.Min = Math.Max(ui.Min, 1 - sb.DistanceTo(curPosition) / field.MaxDistance);
//			}
//			else
//			{
//				//if an opp is near ball
//				double curAngleOppToUs = Math.Atan2(closest.Pos.Y - curPosition.Y, closest.Pos.X - curPosition.X);
//				double scale = Math.Abs(angleBallOpp - curAngleOppToUs);
//				//double curAngleToBall = Math.Atan2(sb.Y-curPosition.Y, sb.X-curPosition.X);
//				//double scale = Math.Abs(angleBallOpp+curAngleToBall);
//				//Normalize
//				while (scale >= Math.PI)
//					scale -= Math.PI;
//				while (scale <= -Math.PI)
//					scale += Math.PI;
//				scale /= Math.PI;
//				scale = scale * 0.8 + 0.2;
//				scale = 1.0 - scale;
//				//	Console.WriteLine("scale: "+scale+" angleBallOpp "+angleBallOpp+" curAngleOppToUs "+curAngleOppToUs);
//
//				ui.Min = Math.Max(ui.Min, (1 - sb.DistanceTo(curPosition) / field.MaxDistance) * scale);
//			}
//			//ui.Min *= util;
//			numAssignedRobots++;
//
//		}
//		ui.setMax(ui.getMin());
//		// Calculate the best possible robot for this job (ignoring every other summand...)
//		if (this->relevantEntryPoints[0]->getMaxCardinality > numAssignedRobots && ass->getNumUnAssignedRobots() > 0)
//		{
//			for (int i = 0; i < ass->getNumUnAssignedRobots(); ++i)
//			{
//				//curPosition = this.playerPositions.GetValue(ass.UnAssignedRobots[i]);
//				curPosition = this.shwm.GetRobotDataByID(ass.UnAssignedRobots[i]).playerPosition;
//				if (curPosition == null)
//					continue;
//				ui.setMax(std::max(ui.getMax(), 1 - curPosition.DistanceTo(sb) / field.MaxDistance));
//			}
//		}
//		//			Console.WriteLine("DistBallRobot: UI is " + retUI.Min + ".." + retUI.Max); // DEBUG OUTPUT
//		ui.setMin(std::max(0, ui.getMin()));
//		ui.setMax(std::max(0, ui.getMax()));
//		return ui;
	}

} /* namespace alica */
