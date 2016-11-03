/*
 * CNPositionAllo.cpp
 *
 *  Created on: 24.10.2016
 *      Author: Philipp Mandler
 */

#include "container/CNPositionAllo.h"

#include <sstream>
#include "container/CNPositionEgo.h"

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

double CNPositionAllo::distanceTo(shared_ptr<CNPositionAllo> alloPos)
{
    return sqrt(pow(this->x - alloPos->x, 2) + pow(this->y - alloPos->y, 2));
}

shared_ptr<CNPositionEgo> CNPosition::toEgo(CNPositionAllo &me)
{
    shared_ptr<CNPositionEgo> ego = make_shared<CNPositionEgo>();

    // sub me pos from allo pos -> rel pos with allo orientation
    double relX = this->x - me.x;
    double relY = this->y - me.y;

    // rotate rel point arround origin -> rel point with ego orientation
    double sin = sin(-me.theta);
    double cos = cos(-me.theta);

    ego->x = cos * relX - sin * relY;
    ego->y = sin * relX - cos * relY;

    // rotate theta
    ego->theta = this->theta - me->theta;

    return ego;
}

/* Operators */

// CNPositionAllo

shared_ptr<CNPositionAllo> CNPositionAllo::operator+(const shared_ptr<CNPositionAllo> &right)
{
    auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    ret->theta += right->theta;
    return ret;
}

shared_ptr<CNPositionAllo> CNPositionAllo::operator-(const shared_ptr<CNPositionAllo> &right)
{
    auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
    ret->x -= right->x;
    ret->y -= right->y;
    ret->theta -= right->theta;
    return ret;
}

// CNVec2D

shared_ptr<CNPositionAllo> CNPositionAllo::operator+(const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNPositionAllo> CNPositionAllo::operator-(const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

} /* namespace geometry */
