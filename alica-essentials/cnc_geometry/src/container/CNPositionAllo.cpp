/*
 * CNPositionAllo.cpp
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPositionAllo.h"

#include <sstream>
#include "container/CNPositionEgo.h"
#include "container/CNVec2DAllo.h"

using namespace std;

namespace geometry
{

CNPositionAllo::~CNPositionAllo() {}

string CNPositionAllo::toString()
{
    stringstream ss;
    ss << "CNPositionAllo: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
    return ss.str();
}

shared_ptr<CNPositionEgo> CNPositionAllo::toEgo(CNPositionAllo &me)
{
    shared_ptr<CNPositionEgo> ego = make_shared<CNPositionEgo>();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - me.x;
    double relY = this->y - me.y;

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-me.theta);
    double c = cos(-me.theta);

    ego->x = c * relX - s * relY;
    ego->y = s * relX - c * relY;

    // rotate theta
    ego->theta = this->theta - me.theta;

    return ego;
}

shared_ptr<CNPositionAllo> operator+(const shared_ptr<CNPositionAllo> &left, const shared_ptr<CNVec2DAllo> &right)
{
	return make_shared<CNPositionAllo>(
			left->x + right->x,
			left->y + right->y,
			left->theta);
}

shared_ptr<CNPositionAllo> operator-(const shared_ptr<CNPositionAllo> &left, const shared_ptr<CNVec2DAllo> &right)
{
	return make_shared<CNPositionAllo>(
			left->x - right->x,
			left->y - right->y,
			left->theta);
}

} /* namespace geometry */
