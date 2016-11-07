/*
 * CNVec2DTemplate.h
 *
 *  Created on: Nov 7, 2016
 *      Author: cn
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNVEC2DTEMPLATE_H_
#define CNC_GEOMETRY_CONTAINER_CNVEC2DTEMPLATE_H_

#include "geometry_msgs/Point.h"

using namespace std;

namespace geometry {

template <class T> class CNVec2DTemplate : public geometry_msgs::Point
{
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

    double angleTo(shared_ptr<T> point)
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

};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNVEC2DTEMPLATE_H_ */
