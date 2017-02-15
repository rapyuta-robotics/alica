#pragma once

#include <geometry_msgs/Pose2D.h>

namespace geometry
{

template <class T> class CNPositionTemplate : public geometry_msgs::Pose2D
{
  public:

    double length()
	{
    	return sqrt(x*x + y*y);
    }


    // Self
    T operator+(const T &right)
    {
        return T(
        		this->x + right.x,
				this->y + right.y,
				this->theta + right.theta);
    }

    T operator-(const T &right)
    {
        return T(
        		this->x - right.x,
				this->y - right.y,
				this->theta - right.theta);
    }

    // Scalar
    T operator/(const double &right)
    {
    	return T(
    			this->x / right,
				this->y / right,
				this->theta);
    }

    T operator*(const double &right)
    {
    	return T(
    			this->x * right,
				this->y * right,
				this->theta);
    }
};

} /* namespace geometry */
