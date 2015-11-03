/*
 * MSLConstraintBuilder.h
 *
 *  Created on: Sep 2, 2014
 *      Author: psp
 */

#ifndef MSLCONSTRAINTBUILDER_H_
#define MSLCONSTRAINTBUILDER_H_

#include <AutoDiff.h>
#include "Point2D.h"
#include <MSLFootballField.h>
#include "GeometryCalculator.h"

#include <memory>
#include <vector>

using namespace std;
using namespace autodiff;
using namespace carpenoctem::containers;



namespace carpenoctem
{
	namespace base
	{
		// should be in /home/psp/impera/MSLCN/MSLWorldModel/src/Areas.cs
		enum Areas
		{
			Surrounding, Field, OwnHalf, OwnPenaltyArea, OwnGoalArea, OppHalf, OppPenaltyArea, OppGoalArea
		};

		class MSLConstraintBuilder
		{
		public:
			static double AREA_TOL;
			static double ON_LINE_TOL;
			static double BLOCK_PASS_WIDTH_TOL;
			static double BLOCK_MIN_RADIUS;
			static double MAX_GOAL_DEFEND_DIST;
			static double MIN_CORRIDOR_WIDTH;
			static double MIN_POSITION_DIST;

			static shared_ptr<Term> outsideRectangle(shared_ptr<TVec> lowerRightCorner,
																shared_ptr<TVec> upperLeftCorner,
																vector<shared_ptr<TVec>> points);
			static shared_ptr<Term> outsideArea(Areas area, shared_ptr<TVec> point);
			static shared_ptr<Term> outsideArea(Areas area, vector<shared_ptr<TVec>> points);

		private:
			static msl::MSLFootballField* field;
			static shared_ptr<geometry::CNPoint2D> ownRightSurCornerP;
			static shared_ptr<geometry::CNPoint2D> oppLeftSurCornerP;
			static shared_ptr<geometry::CNPoint2D> ownRightCornerP;
			static shared_ptr<geometry::CNPoint2D> oppLeftCornerP;
			static shared_ptr<geometry::CNPoint2D> oppLRHalfP;
			static shared_ptr<geometry::CNPoint2D> ownULHalfP;
			static shared_ptr<geometry::CNPoint2D> oppLRPenaltyAreaP;
			static shared_ptr<geometry::CNPoint2D> oppULPenaltyAreaP;
			static shared_ptr<geometry::CNPoint2D> ownLRPenaltyAreaP;
			static shared_ptr<geometry::CNPoint2D> ownULPenaltyAreaP;
			static shared_ptr<geometry::CNPoint2D> ownLRGoalAreaP;
			static shared_ptr<geometry::CNPoint2D> ownULGoalAreaP;
			static shared_ptr<geometry::CNPoint2D> oppLRGoalAreaP;
			static shared_ptr<geometry::CNPoint2D> oppULGoalAreaP;
			static shared_ptr<geometry::CNPoint2D> ownGoalMidP;
			static shared_ptr<geometry::CNPoint2D> oppGoalMidP;
			static shared_ptr<geometry::CNPoint2D> centreMarkP;

			static shared_ptr<TVec> ownRightSurCornerT;
			static shared_ptr<TVec> oppLeftSurCornerT;
			static shared_ptr<TVec> ownRightCornerT;
			static shared_ptr<TVec> oppLeftCornerT;
			static shared_ptr<TVec> oppLRHalfT;
			static shared_ptr<TVec> ownULHalfT;
			static shared_ptr<TVec> oppLRPenaltyAreaT;
			static shared_ptr<TVec> oppULPenaltyAreaT;
			static shared_ptr<TVec> ownLRPenaltyAreaT;
			static shared_ptr<TVec> ownULPenaltyAreaT;
			static shared_ptr<TVec> ownLRGoalAreaT;
			static shared_ptr<TVec> ownULGoalAreaT;
			static shared_ptr<TVec> oppLRGoalAreaT;
			static shared_ptr<TVec> oppULGoalAreaT;
			static shared_ptr<TVec> ownGoalMidT;
			static shared_ptr<TVec> oppGoalMidT;
			static shared_ptr<TVec> centreMarkT;

			static void resolveArea(Areas area, shared_ptr<Point2D> *lowerRightCorner,
									shared_ptr<Point2D> *upperLeftCorner);
		};

	} /* namespace base */
} /* namespace carpeNoctem */

#endif /* MSLCONSTRAINTBUILDER_H_ */
