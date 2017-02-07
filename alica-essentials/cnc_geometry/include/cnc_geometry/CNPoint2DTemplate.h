/*
 * CNVec2DBase.h
 *
 *  Created on: Nov 7, 2016
 *      Author: cn
 */

#pragma once

#include <geometry_msgs/Point.h>

namespace geometry {

template <class T> class CNPoint2DTemplate : public geometry_msgs::Point
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

    double angleToPoint(std::shared_ptr<T> point)
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
    	T delta = this - pos; // TODO
    	return delta->length();
    }

    double length() {
    	return sqrt(x * x + y * y);
    }

};

// Operators

// Self

template <typename T>
typename std::enable_if<std::is_base_of<CNPoint2DTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator+(const std::shared_ptr<T> &left, const std::shared_ptr<T> &right)
{
    return std::make_shared<T>(
    		left->x + right->x,
			left->y + right->y);
}

template <typename T>
typename std::enable_if<std::is_base_of<CNPoint2DTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator-(const std::shared_ptr<T> &left, const std::shared_ptr<T> &right)
{
    return std::make_shared<T>(
    		left->x - right->x,
			left->y - right->y);
}

// Scalar

template <typename T>
typename std::enable_if<std::is_base_of<CNPoint2DTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator/(const std::shared_ptr<T> &left, const double &right)
{
	return std::make_shared<T>(
			left->x / right,
			left->y / right);
}

template <typename T>
typename std::enable_if<std::is_base_of<CNPoint2DTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator*(const std::shared_ptr<T> &left, const double &right)
{
	return std::make_shared<T>(
			left->x * right,
			left->y * right);
}

} /* namespace geometry */
