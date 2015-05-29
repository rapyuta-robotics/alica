/*
 * CNPosition.cpp
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#include "container/CNPosition.h"
#include <sstream>
namespace geometry {


	CNPosition::CNPosition(double x, double y, double theta) {
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	CNPosition::~CNPosition() {
	}

	string CNPosition::toString()
	{
		stringstream ss;
		ss << "CNPosition: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
		return ss.str();
	}
}

