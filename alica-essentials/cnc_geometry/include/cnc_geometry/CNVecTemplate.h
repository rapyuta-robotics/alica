#pragma once

#include <geometry_msgs/Point.h>

namespace geometry
{

template <class T> class CNVecTemplate : public geometry_msgs::Point
{
  public:
	std::shared_ptr<T> clone()
	{
		return std::make_shared<T>(x, y, z);
	}

    std::shared_ptr<T> rotate(double radian)
    {
    	// TODO: fix
    	return std::make_shared<T>(
    			this->x * cos(radian) - this->y * sin(radian),
				this->x * sin(radian) + this->y * cos(radian));
    }

    double angle()
    {
    	// TODO: fix
    	return atan2(y, x);
    }

    double angleTo(std::shared_ptr<T> point)
    {
    	// TODO: fix
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
            norm->z = this->z / length;
        }
        else
        {
            std::cerr << "CNVec: Trying to normalize (0, 0, 0)!" << std::endl;
        }

        return norm;
	}

    double distanceTo(std::shared_ptr<T> pos)
    {
    	// TODO: fix
    	T delta = this - pos;
    	return delta->length();
    }

    double length() {
    	return sqrt(x*x + y*y + z*z);
    }

    std::shared_ptr<T> operator*(const double &right)
    {
    	return std::make_shared<T>(
    			this->x * right,
				this->y * right);
    }

};

// Operators

// Self

template <typename T>
typename std::enable_if<std::is_base_of<CNVecTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator+(const std::shared_ptr<T> &left, const std::shared_ptr<T> &right)
{
    return std::make_shared<T>(
    		left->x + right->x,
			left->y + right->y,
			left->z + right->z);
}

template <typename T>
typename std::enable_if<std::is_base_of<CNVecTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator-(const std::shared_ptr<T> &left, const std::shared_ptr<T> &right)
{
    return std::make_shared<T>(
    		left->x - right->x,
			left->y - right->y,
			left->z - right->z);
}

// Scalar

template <typename T>
typename std::enable_if<std::is_base_of<CNVecTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator/(const std::shared_ptr<T> &left, const double &right)
{
	return std::make_shared<T>(
			left->x / right,
			left->y / right,
			left->z / right);
}

template <typename T>
typename std::enable_if<std::is_base_of<CNVecTemplate<T>, T>::value, std::shared_ptr<T>>::type
operator*(const std::shared_ptr<T> &left, const double &right)
{
	return std::make_shared<T>(
			left->x * right,
			left->y * right,
			left->z * right);
}

} /* namespace geometry */
