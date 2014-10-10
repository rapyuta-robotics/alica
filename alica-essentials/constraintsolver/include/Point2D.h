/*
 * Point2D.h
 *
 *  Created on: Sep 2, 2014
 *      Author: psp
 */

#ifndef POINT2D_H_
#define POINT2D_H_

namespace CarpeNoctem
{
	namespace Containers
	{

		class Point2D
		{
		public:
			Point2D(double x, double y);

			double getX();
			void setX(double x);
			double getY();
			void setY(double y);

		private:
			double _x;
			double _y;
		};

	} /* namespace Containers */
} /* namespace CarpeNoctem */

#endif /* POINT2D_H_ */
