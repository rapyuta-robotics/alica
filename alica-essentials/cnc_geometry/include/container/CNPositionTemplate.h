/*
 * CNPositionTemplate.h
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#ifndef CNC_GEOMETRY_CONTAINER_CNPOSITIONTEMPLATE_H_
#define CNC_GEOMETRY_CONTAINER_CNPOSITIONTEMPLATE_H_

#include "geometry_msgs/Pose2D.h"
#include "container/CNVec2DTemplate.h"

namespace geometry
{

template <class T> class CNPositionTemplate : public geometry_msgs::Pose2D
{
  public:
	std::shared_ptr<T> clone()
	{
		return std::make_shared<T>(this->x, this->y, this->theta);
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
				this->y + right->y,
				this->theta + right->theta);
    }

    std::shared_ptr<T> operator-(const std::shared_ptr<T> &right)
    {
        return std::make_shared<T>(
        		this->x - right->x,
				this->y - right->y,
				this->theta - right->theta);
    }

    // Scalar

    std::shared_ptr<T> operator/(const double &right)
	{
		return std::make_shared<T>(
				this->x / right,
				this->y / right,
				this->theta);
	}

    std::shared_ptr<T> operator*(const double &right)
	{
		return std::make_shared<T>(
				this->x * right,
				this->y * right,
				this->theta);
	}

};

} /* namespace geometry */

#endif /* CNC_GEOMETRY_CONTAINER_CNPOSITIONTEMPLATE_H_ */
