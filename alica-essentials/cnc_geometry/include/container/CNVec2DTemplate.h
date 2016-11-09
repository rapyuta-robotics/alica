/*
 * CNVec2DTemplate.h
 *
 *  Created on: Nov 7, 2016
 *      Author: cn
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNVEC2DTEMPLATE_H_
#define CNC_GEOMETRY_CONTAINER_CNVEC2DTEMPLATE_H_

#include "geometry_msgs/Point.h"

namespace geometry {

template <class T> class CNVec2DTemplate : public geometry_msgs::Point
{
  public:
	std::shared_ptr<T> clone()
	{
		return std::make_shared<T>(this->x, this->y);
	}

    std::shared_ptr<T> rotate(double radian)
    {
    	return std::make_shared<T>(
    			this->x * cos(radian) - this->y * sin(radian),
				this->x * sin(radian) + this->y * cos(radian));
    }

    double angle()
    {
    	return atan2(y, x);
    }

    double angleTo(std::shared_ptr<T> point)
    {
    	return atan2(point->y - this->y, point->x - this->x);
    }

    std::shared_ptr<T> normalize()
	{
        std::shared_ptr<T> norm = std::make_shared<T>();
        double length = this->length();

        if (length > 0)
        {
            norm->x = this->x / length;
            norm->y = this->y / length;
        }
        else
        {
            std::cerr << "CNVec2D: Trying to normalize (0, 0)!" << std::endl;
        }

        return norm;
	}

    double distanceTo(std::shared_ptr<T> pos)
    {
    	T delta = this - pos;
    	return delta->length();
    }

    double length() {
    	return sqrt(x * x + y * y);
    }

    // Operators

    // Self

    std::shared_ptr<T> operator+(const std::shared_ptr<T> &right)
    {
        return std::make_shared<T>(
        		this->x + right->x,
				this->y + right->y);
    }

    std::shared_ptr<T> operator-(const std::shared_ptr<T> &right)
    {
        return std::make_shared<T>(
        		this->x - right->x,
				this->y - right->y);
    }

    // Scalar

    std::shared_ptr<T> operator/(const double &right)
	{
		return std::make_shared<T>(
				this->x / right,
				this->y / right);
	}

    std::shared_ptr<T> operator*(const double &right)
	{
		return std::make_shared<T>(
				this->x * right,
				this->y * right);
	}

};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNVEC2DTEMPLATE_H_ */
