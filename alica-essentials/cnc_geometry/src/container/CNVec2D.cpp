/*
 * CNVec2D.cpp
 *
 *  Created on: 24.12.2016
 *      Author: Philipp Mandler
 */

#include "container/CNVec2D.h"
#include "container/CNPosition.h"
#include <sstream>

using namespace std;

namespace geometry
{

CNVec2D::CNVec2D(double x, double y)
{
    this->x = x;
    this->y = y;
}

double CNVec2D::length()
{
    return sqrt(x * x + y * y);
}

shared_ptr<CNVec2D> CNVec2D::rotate(double radian)
{
    return make_shared<CNVec2D>(this->x * cos(radian) - this->y * sin(radian), this->x * sin(radian) + this->y * cos(radian));
}

double CNVec2D::angleTo()
{
    return atan2(y, x);
}

// shared_ptr<CNVec2D> CNVec2D::alloToEgo(CNPosition &me)
// {
//     shared_ptr<CNVec2D> ego = make_shared<CNVec2D>();

//     double x = this->x - me.x;
//     double y = this->y - me.y;

//     double angle = atan2(y, x) - me.theta;
//     double dist = sqrt(x * x + y * y);

//     ego->x = cos(angle) * dist;
//     ego->y = sin(angle) * dist;

//     return ego;
// }

// shared_ptr<CNVec2D> CNVec2D::egoToAllo(CNPosition &me)
// {
//     shared_ptr<CNVec2D> allo = make_shared<CNVec2D>();
//     allo->x = me.x;
//     allo->y = me.y;

//     allo->x += cos(me.theta) * x - sin(me.theta) * y;
//     allo->y += sin(me.theta) * x + cos(me.theta) * y;

//     return allo;
// }

CNVec2D::~CNVec2D() {}

shared_ptr<CNVec2D> CNVec2D::normalize()
{
    shared_ptr<CNVec2D> norm = make_shared<CNVec2D>();
    double length = this->length();

    if (length > 0)
    {
        norm->x = this->x / length;
        norm->y = this->y / length;
    }
    else
    {
        cerr << "CNVec2D: Trying to normalize 0.0!" << endl;
    }

    return norm;
}

shared_ptr<CNVec2D> CNVec2D::clone()
{
    return make_shared<CNVec2D>(this->x, this->y);
}

shared_ptr<CNVec2D> CNVec2D::operator*(const double &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x *= right;
    ret->y *= right;
    return ret;
}

shared_ptr<CNVec2D> operator*(const shared_ptr<CNVec2D> &left, const double &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x *= right;
    ret->y *= right;
    return ret;
}

shared_ptr<CNVec2D> CNVec2D::operator+(const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNVec2D> CNVec2D::operator-(const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

shared_ptr<CNVec2D> CNVec2D::operator+(const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNVec2D> CNVec2D::operator-(const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

shared_ptr<CNVec2D> CNVec2D::operator/(const double &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x /= right;
    ret->y /= right;
    return ret;
}

shared_ptr<CNVec2D> operator+(const shared_ptr<CNVec2D> &left, const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNVec2D> operator-(const shared_ptr<CNVec2D> &left, const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

//TODO
shared_ptr<CNVec2D> operator+(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNVec2D> operator-(const shared_ptr<CNVec2D> &left, const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

shared_ptr<CNVec2D> operator/(const shared_ptr<CNVec2D> &left, const double &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x /= right;
    ret->y /= right;
    return ret;
}

shared_ptr<CNVec2D> operator+(const shared_ptr<CNPosition> &left, const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNVec2D> operator-(const shared_ptr<CNPosition> &left, const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNVec2D>(left->x, left->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

string CNVec2D::toString()
{
    stringstream ss;
    ss << "CNVec2D: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

double CNVec2D::distanceTo(shared_ptr<CNVec2D> point)
{
    return sqrt(pow(this->x - point->x, 2) + pow(this->y - point->y, 2));
}

double CNVec2D::distanceTo(shared_ptr<CNPosition> pos)
{
    return sqrt(pow(this->x - pos->x, 2) + pow(this->y - pos->y, 2));
}

double geometry::CNVec2D::angleToPoint(shared_ptr<CNVec2D> point)
{
    return atan2(point->y - this->y, point->x - this->x);
}

} /* namespace */
