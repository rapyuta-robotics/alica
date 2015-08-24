/*
 * GeometryCalculator.h
 *
 *  Created on: 15.11.2014
 *      Author: Tobias Schellien
 */

#ifndef CNC_GEOMETRY_GEOMETRY_GEOMETRYCALCULATOR_H_
#define CNC_GEOMETRY_GEOMETRY_GEOMETRYCALCULATOR_H_

#include <tuple>

#include "container/CNPoint2D.h"

using namespace std;

namespace geometry {

	class GeometryCalculator {
	public:

		virtual ~GeometryCalculator(){};
		static double deltaAngle(double angle1, double angle2);
		static bool isInsideRectangle(shared_ptr<CNPoint2D> rectPointA, shared_ptr<CNPoint2D> rectPointB, shared_ptr<CNPoint2D> point);
		static bool isInsidePolygon(vector<shared_ptr<CNPoint2D>> polygon, int n, shared_ptr<CNPoint2D> point);
		static double distancePointToLineSegment (double x, double y, shared_ptr<CNPoint2D> a, shared_ptr<CNPoint2D> b);
		static double absDeltaAngle(double angle1, double angle2);

		//Sign function --> VORZEICHENFUNKTION
		template <typename T> static int sgn(T val) {
			return (T(0) < val) - (val < T(0));
		}


	private:
		GeometryCalculator();
		static bool onSegment(shared_ptr<CNPoint2D> p, shared_ptr<CNPoint2D> q, shared_ptr<CNPoint2D> r);
		static int orientation(shared_ptr<CNPoint2D> p, shared_ptr<CNPoint2D> q, shared_ptr<CNPoint2D> r);
		static bool doIntersect(shared_ptr<CNPoint2D> p1, shared_ptr<CNPoint2D> q1, shared_ptr<CNPoint2D> p2, shared_ptr<CNPoint2D> q2);
	};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_GEOMETRY_GEOMETRYCALCULATOR_H_ */
