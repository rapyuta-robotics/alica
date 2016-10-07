
#include "container/CNPositionBaseBase.h"
#include <sstream>
namespace geometry
{

	CNPositionBase::CNPositionBase(double x, double y, double theta)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	CNPositionBase::~CNPositionBase() { }

	string CNPositionBase::toString()
	{
		stringstream ss;
		ss << "CNPositionBase: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
		return ss.str();
	}
	
}
