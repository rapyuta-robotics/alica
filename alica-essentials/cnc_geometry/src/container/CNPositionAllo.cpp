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
    //TODO: rotate ego
    shared_ptr<CNPositionEgo> ego = make_shared<CNPositionEgo>();

    double x = this->x - me.x;
    double y = this->y - me.y;

    double angle = atan2(y, x) - me.theta;
    double dist = sqrt(x * x + y * y);

    ego->x = cos(angle) * dist;
    ego->y = sin(angle) * dist;

    return ego;
}

/* Operators */

// CNPositionAllo

shared_ptr<CNPositionAllo> CNPositionAllo::operator+(const shared_ptr<CNPositionAllo> &right)
{
    auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    // TODO
    return ret;
}

shared_ptr<CNPositionAllo> CNPositionAllo::operator-(const shared_ptr<CNPositionAllo> &right)
{
    auto ret = make_shared<CNPositionAllo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    // TODO
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
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

/* Right handed operators */

// CNVec2D

shared_ptr<CNPositionAllo> operator+(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPositionAllo> &right)
{
    auto ret = make_shared<CNPositionAllo>(right->x, right->y, right->theta);
    ret->x += left->x;
    ret->y += left->y;
    return ret;
}

shared_ptr<CNPositionAllo> operator-(const shared_ptr<CNPositionAllo> &left, const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNPositionAllo>(right->x, right->y, right->theta);
    ret->x -= left->x;
    ret->y -= left->y;
    return ret;
}

} /* namespace geometry */
