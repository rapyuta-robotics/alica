/*
 * MSLConstraintBuilder.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: psp
 */

#include "MSLConstraintBuilder.h"

namespace carpenoctem
{
	namespace base
	{
		double MSLConstraintBuilder::AREA_TOL = 100.0;
		double MSLConstraintBuilder::ON_LINE_TOL = 50.0;
		double MSLConstraintBuilder::BLOCK_PASS_WIDTH_TOL = 100.0;
		double MSLConstraintBuilder::BLOCK_MIN_RADIUS = 600.0;
		double MSLConstraintBuilder::MAX_GOAL_DEFEND_DIST = 3000.0;
		double MSLConstraintBuilder::MIN_CORRIDOR_WIDTH = 700.0;
		double MSLConstraintBuilder::MIN_POSITION_DIST = 650.0;

		// INTERN
		msl::MSLFootballField* MSLConstraintBuilder::field = msl::MSLFootballField::getInstance();

		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownRightSurCornerP = field->posLRSurrounding();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLeftSurCornerP = field->posULSurrounding();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownRightCornerP = field->posRightOwnCorner();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLeftCornerP = field->posLeftOppCorner();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRHalfP = field->posLROppHalf();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULHalfP = field->posULOwnHalf();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRPenaltyAreaP = field->posLROppPenaltyArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppULPenaltyAreaP = field->posULOppPenaltyArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownLRPenaltyAreaP = field->posLROwnPenaltyArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULPenaltyAreaP = field->posULOwnPenaltyArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownLRGoalAreaP = field->posLROwnGoalArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULGoalAreaP = field->posULOwnGoalArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRGoalAreaP = field->posLROppGoalArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppULGoalAreaP = field->posULOppGoalArea();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownGoalMidP = field->posOwnGoalMid();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppGoalMidP = field->posOppGoalMid();
		shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::centreMarkP = field->posCenterMarker();

//		shared_ptr<TVec> MSLConstraintBuilder::ownRightSurCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLeftSurCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownRightCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLeftCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLRHalfT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownULHalfT;
		shared_ptr<TVec> MSLConstraintBuilder::oppLRPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
				oppLRPenaltyAreaP->x, oppLRPenaltyAreaP->y});
		shared_ptr<TVec> MSLConstraintBuilder::oppULPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
				oppULPenaltyAreaP->x, oppULPenaltyAreaP->y});
		shared_ptr<TVec> MSLConstraintBuilder::ownLRPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
				ownLRPenaltyAreaP->x, ownLRPenaltyAreaP->y});
		shared_ptr<TVec> MSLConstraintBuilder::ownULPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
				ownULPenaltyAreaP->x, ownULPenaltyAreaP->x});
//		shared_ptr<TVec> MSLConstraintBuilder::ownLRGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownULGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLRGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppULGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownGoalMidT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppGoalMidT;
//		shared_ptr<TVec> MSLConstraintBuilder::centreMarkT;

		shared_ptr<Term> MSLConstraintBuilder::outsideRectangle(shared_ptr<TVec> lowerRightCorner,
																shared_ptr<TVec> upperLeftCorner,
																vector<shared_ptr<TVec>> points)
		{
			shared_ptr<Term> c = !TermBuilder::boundedRectangle(points[0], lowerRightCorner, upperLeftCorner,
																Term::getConstraintSteepness());
			;
			for (int i = 1; i < points.size(); ++i)
			{
				c = c
						& !TermBuilder::boundedRectangle(points[i], lowerRightCorner, upperLeftCorner,
															Term::getConstraintSteepness());
			}
			return c;
		}

		shared_ptr<Term> MSLConstraintBuilder::outsideArea(Areas area, shared_ptr<TVec> point)
		{
			vector<shared_ptr<TVec>> points;
			points.push_back(point);
			return outsideArea(area, points);
		}

		shared_ptr<Term> MSLConstraintBuilder::outsideArea(Areas area, vector<shared_ptr<TVec>> points)
		{
			shared_ptr<geometry::CNPoint2D> lowerRightCornerP;
			shared_ptr<geometry::CNPoint2D> upperLeftCornerP;
			resolveArea(area, &lowerRightCornerP, &upperLeftCornerP);
			shared_ptr<TVec> lowerRightCorner =
					make_shared<TVec>(
							initializer_list<double> {lowerRightCornerP->x - AREA_TOL, lowerRightCornerP->y
																- AREA_TOL});
			shared_ptr<TVec> upperLeftCorner =
					make_shared<TVec>(
							initializer_list<double> {upperLeftCornerP->x + AREA_TOL, upperLeftCornerP->y
																+ AREA_TOL});
			return outsideRectangle(lowerRightCorner, upperLeftCorner, points);
		}

		void MSLConstraintBuilder::resolveArea(Areas area, shared_ptr<geometry::CNPoint2D> *lowerRightCorner,
												shared_ptr<geometry::CNPoint2D> *upperLeftCorner)
		{
			switch (area)
			{
				case Areas::Surrounding:

					break;
				case Areas::Field:

					break;
				case Areas::OppHalf:

					break;
				case Areas::OwnHalf:

					break;
				case Areas::OppPenaltyArea:
					*lowerRightCorner = oppLRPenaltyAreaP;
					*upperLeftCorner = oppULPenaltyAreaP;
					break;
				case Areas::OwnPenaltyArea:
					*lowerRightCorner = ownLRPenaltyAreaP;
					*upperLeftCorner = ownULPenaltyAreaP;
					break;
				case Areas::OwnGoalArea:

					break;
				case Areas::OppGoalArea:

					break;
				default:
					throw "Unknown Area!";
			}
		}
	} /* namespace base */
} /* namespace carpenoctem */
