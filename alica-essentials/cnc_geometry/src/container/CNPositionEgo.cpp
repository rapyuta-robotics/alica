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

CNPositionEgo::~CNPositionEgo() {}

string CNPositionEgo::toString()
{
    stringstream ss;
    ss << "CNPositionEgo: x: " << this->x << " y: " << this->y << " theta: " << this->theta << endl;
    return ss.str();
}

double CNPositionEgo::distanceTo(shared_ptr<CNPositionEgo> egoPos)
{
    return sqrt(pow(this->x - egoPos->x, 2) + pow(this->y - egoPos->y, 2));
}

shared_ptr<CNPositionAllo> CNPositionEgo::toAllo(CNPositionAllo &me)
{
    // TODO: rotate ego
    shared_ptr<CNPositionAllo> allo = make_shared<CNPositionAllo>();
    allo->x = me.x;
    allo->y = me.y;

    allo->x += cos(me.theta) * x - sin(me.theta) * y;
    allo->y += sin(me.theta) * x + cos(me.theta) * y;

    return allo;
}

/* Operators */

// CNPositionEgo

shared_ptr<CNPositionEgo> CNPositionEgo::operator+(const shared_ptr<CNPositionEgo> &right)
{
    auto ret = make_shared<CNPositionEgo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    // TODO
    return ret;
}

shared_ptr<CNPositionEgo> CNPositionEgo::operator-(const shared_ptr<CNPositionEgo> &right)
{
    auto ret = make_shared<CNPositionEgo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    // TODO
    return ret;
}

// CNVec2D

shared_ptr<CNPositionEgo> CNPositionEgo::operator+(const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNPositionEgo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNPositionEgo> CNPositionEgo::operator-(const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNPositionEgo>(this->x, this->y, this->theta);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

/* Right handed operators */

// CNVec2D

shared_ptr<CNPositionEgo> operator+(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPositionEgo> &right)
{
    auto ret = make_shared<CNPositionEgo>(right->x, right->y, right->theta);
    ret->x += left->x;
    ret->y += left->y;
    return ret;
}

shared_ptr<CNPositionEgo> operator-(const shared_ptr<CNPositionEgo> &left, const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNPositionEgo>(right->x, right->y, right->theta);
    ret->x -= left->x;
    ret->y -= left->y;
    return ret;
}

} /* namespace geometry */
