/*
 * CNPositionEgo.cpp
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#include <cnc_geometry/CNVecEgo.h>
#include "cnc_geometry/CNPositionEgo.h"

#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"

using namespace std;

namespace geometry
{

CNPositionEgo::CNPositionEgo(double x, double y, double theta)
{
	this->x = x;
	this->y = y;
	this->theta = theta;
}

CNPositionEgo::CNPositionEgo(const CNPositionEgo &obj)
{
	this->x = obj.x;
	this->y = obj.y;
	this->theta = obj.theta;
}

CNPositionEgo::~CNPositionEgo() {}

string CNPositionEgo::toString() const
{
	stringstream ss;
	ss << "CNPositionEgo: X: " << this->x << " Y: " << this->y << " Orientation: " << this->theta << endl;
	return ss.str();
}

CNPositionAllo CNPositionEgo::toAllo(CNPositionAllo &me) const
{
	auto allo = CNPositionAllo();

	// rotate rel point around origin -> rel point with allo orientation
	double s = sin(me.theta);
	double c = cos(me.theta);

	double x = c * this->x - s * this->y;
	double y = s * this->x - c * this->y; // TODO: fix

	// sum me pos and rel pos -> allo pos with allo rotaion
	allo.x = x + me.x;
	allo.y = y + me.y;

	// rotate theta
	allo.theta = this->theta + me.theta;

	return allo;
}

CNPositionEgo CNPositionEgo::operator+(const CNVecEgo &right) const
{
	return CNPositionEgo(
			this->x + right.x,
			this->y + right.y,
			this->theta);
}

CNPositionEgo CNPositionEgo::operator-(const CNVecEgo &right) const
{
	return CNPositionEgo(
			this->x - right.x,
			this->y - right.y,
			this->theta);
}

} /* namespace geometry */
