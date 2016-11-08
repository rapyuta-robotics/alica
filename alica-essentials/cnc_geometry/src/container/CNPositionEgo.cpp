/*
 * CNPositionEgo.cpp
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPositionEgo.h"

#include <sstream>
#include "container/CNPositionAllo.h"

using namespace std;

namespace geometry
{

CNPositionEgo::~CNPositionEgo()
{
}

string CNPositionEgo::toString()
{
	stringstream ss;
	ss << "CNPositionEgo: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
	return ss.str();
}

shared_ptr<CNPositionAllo> CNPositionEgo::toAllo(CNPositionAllo &me)
{
	shared_ptr<CNPositionAllo> allo = make_shared<CNPositionAllo>();

	// rotate rel point around origin -> rel point with allo orientation
	double s = sin(me.theta);
	double c = cos(me.theta);

	double x = c * this->x - s * this->y;
	double y = s * this->x - c * this->y;

	// sum me pos and rel pos -> allo pos with allo rotaion
	allo->x = x + me.x;
	allo->y = y + me.y;

	// rotate theta
	allo->theta = this->theta + me.theta;

	return allo;
}

shared_ptr<CNPositionEgo> CNPositionEgo::operator+(const shared_ptr<CNVec2DEgo> &right)
{
	return make_shared<CNPositionEgo>(
			this->x + right->x,
			this->y + right->y,
			this->theta);
}

shared_ptr<CNPositionEgo> CNPositionEgo::operator-(const shared_ptr<CNVec2DEgo> &right)
{
	return make_shared<CNPositionEgo>(
			this->x - right->x,
			this->y - right->y,
			this->theta);
}

} /* namespace geometry */
