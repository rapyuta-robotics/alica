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
		static bool isInsideRectangle(CNPoint2D rectPointA, CNPoint2D rectPointB, CNPoint2D point);
		static bool isInsidePolygon(vector<CNPoint2D> polygon, int n, CNPoint2D point);

	private:
		GeometryCalculator();
		static bool onSegment(CNPoint2D p, CNPoint2D q, CNPoint2D r);
		static int orientation(CNPoint2D p, CNPoint2D q, CNPoint2D r);
		static bool doIntersect(CNPoint2D p1, CNPoint2D q1, CNPoint2D p2, CNPoint2D q2);
	};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_GEOMETRY_GEOMETRYCALCULATOR_H_ */
