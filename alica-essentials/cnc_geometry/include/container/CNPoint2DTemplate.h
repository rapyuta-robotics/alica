/*
 * CNVec2DBase.h
 *
 *  Created on: Nov 7, 2016
 *      Author: cn
 */

#ifndef INCLUDE_CONTAINER_CNVEC2DTEMPLATE_H_
#define INCLUDE_CONTAINER_CNVEC2DTEMPLATE_H_

#include "geometry_msgs/Point.h"
#include "container/CNPosition.h" // TODO: remove

using namespace std;

namespace geometry {

template <class T> class CNPoint2DTemplate : public geometry_msgs::Point
{
  public:
	shared_ptr<T> clone()
	{
		return make_shared<T>(this->x, this->y);
	}

    shared_ptr<T> rotate(double radian)
    {
    	return make_shared<T>(
    			this->x * cos(radian) - this->y * sin(radian),
				this->x * sin(radian) + this->y * cos(radian));
    }

    double angle()
    {
    	return atan2(y, x);
    }

    double angleToPoint(shared_ptr<T> point)
    {
    	return atan2(point->y - this->y, point->x - this->x);
    }

    shared_ptr<T> normalize()
	{
        shared_ptr<T> norm = make_shared<T>();
        double length = this->length();

        if (length > 0)
        {
            norm->x = this->x / length;
            norm->y = this->y / length;
        }
        else
        {
            cerr << "CNVec2D: Trying to normalize (0, 0)!" << endl;
        }

        return norm;
	}

    double distanceTo(shared_ptr<T> pos)
    {
    	T delta = this - pos;
    	return delta->length();
    }

    double length() {
    	return sqrt(x * x + y * y);
    }

    // Operators

    // Self

    shared_ptr<T> operator+(const shared_ptr<T> &right)
    {
        return make_shared<T>(
        		this->x + right->x,
				this->y + right->y);
    }

    shared_ptr<T> operator-(const shared_ptr<T> &right)
    {
        return make_shared<T>(
        		this->x - right->x,
				this->y - right->y);
    }

    // Scalar

    shared_ptr<T> operator/(const double &right)
	{
		return make_shared<T>(
				this->x / right,
				this->y / right);
	}

    shared_ptr<T> operator*(const double &right)
	{
		return make_shared<T>(
				this->x * right,
				this->y * right);
	}

    static shared_ptr<T> fromOld(shared_ptr<CNPoint2D> oldPos) {
    	//TODO: remove function
		return make_shared<T>(
				oldPos->x,
				oldPos->y);
	}

    shared_ptr<CNPoint2D> toOld() {
		//TODO: remove function
		return make_shared<CNPoint2D>(
				this->x,
				this->y);
	}

};

} /* namespace geometry */

#endif /* INCLUDE_CONTAINER_CNVEC2DTEMPLATE_H_ */
