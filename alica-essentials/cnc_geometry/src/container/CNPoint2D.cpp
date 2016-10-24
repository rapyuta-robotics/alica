/*
 * Point2D.cpp
 *
 *  Created on: 19.11.2014
 *      Author: Tobias Schellien
 */

#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include <sstream>

using namespace std;

namespace geometry
{

CNPoint2D::CNPoint2D(double x, double y)
{
    this->x = x;
    this->y = y;
}

double CNPoint2D::length()
{
    return sqrt(x * x + y * y);
}

shared_ptr<CNPoint2D> CNPoint2D::rotate(double radian)
{
    return make_shared<CNPoint2D>(this->x * cos(radian) - this->y * sin(radian), this->x * sin(radian) + this->y * cos(radian));
}

double CNPoint2D::angleTo()
{
    return atan2(y, x);
}

shared_ptr<CNPoint2D> CNPoint2D::alloToEgo(CNPosition &me)
{
    shared_ptr<CNPoint2D> ego = make_shared<CNPoint2D>();

    double x = this->x - me.x;
    double y = this->y - me.y;

    double angle = atan2(y, x) - me.theta;
    double dist = sqrt(x * x + y * y);

    ego->x = cos(angle) * dist;
    ego->y = sin(angle) * dist;

    return ego;
}

shared_ptr<CNPoint2D> CNPoint2D::egoToAllo(CNPosition &me)
{
    shared_ptr<CNPoint2D> allo = make_shared<CNPoint2D>();
    allo->x = me.x;
    allo->y = me.y;

    allo->x += cos(me.theta) * x - sin(me.theta) * y;
    allo->y += sin(me.theta) * x + cos(me.theta) * y;

    return allo;
}

CNPoint2D::~CNPoint2D() {}

shared_ptr<CNPoint2D> CNPoint2D::normalize()
{
    shared_ptr<CNPoint2D> norm = make_shared<CNPoint2D>();
    double length = this->length();

    if (length > 0)
    {
	norm->x = this->x / length;
	norm->y = this->y / length;
    }
    else
    {
	cerr << "CNPoint2D: Trying to normalize 0.0!" << endl;
    }

    return norm;
}

shared_ptr<CNPoint2D> CNPoint2D::clone()
{
    return make_shared<CNPoint2D>(this->x, this->y);
}

shared_ptr<CNPoint2D> CNPoint2D::operator*(const double &right)
{
    auto ret = make_shared<CNPoint2D>(this->x, this->y);
    ret->x *= right;
    ret->y *= right;
    return ret;
}

shared_ptr<CNPoint2D> operator*(const shared_ptr<CNPoint2D> &left, const double &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x *= right;
    ret->y *= right;
    return ret;
}

shared_ptr<CNPoint2D> CNPoint2D::operator+(const shared_ptr<CNPoint2D> &right)
{
    auto ret = make_shared<CNPoint2D>(this->x, this->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNPoint2D> CNPoint2D::operator-(const shared_ptr<CNPoint2D> &right)
{
    auto ret = make_shared<CNPoint2D>(this->x, this->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

shared_ptr<CNPoint2D> CNPoint2D::operator+(const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNPoint2D>(this->x, this->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNPoint2D> CNPoint2D::operator-(const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNPoint2D>(this->x, this->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

shared_ptr<CNPoint2D> CNPoint2D::operator/(const double &right)
{
    auto ret = make_shared<CNPoint2D>(this->x, this->y);
    ret->x /= right;
    ret->y /= right;
    return ret;
}

shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPoint2D> &left, const shared_ptr<CNPoint2D> &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPoint2D> &left, const shared_ptr<CNPoint2D> &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

//TODO
shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPoint2D> &left, const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPoint2D> &left, const shared_ptr<CNPosition> &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

shared_ptr<CNPoint2D> operator/(const shared_ptr<CNPoint2D> &left, const double &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x /= right;
    ret->y /= right;
    return ret;
}

shared_ptr<CNPoint2D> operator+(const shared_ptr<CNPosition> &left, const shared_ptr<CNPoint2D> &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x += right->x;
    ret->y += right->y;
    return ret;
}

shared_ptr<CNPoint2D> operator-(const shared_ptr<CNPosition> &left, const shared_ptr<CNPoint2D> &right)
{
    auto ret = make_shared<CNPoint2D>(left->x, left->y);
    ret->x -= right->x;
    ret->y -= right->y;
    return ret;
}

string CNPoint2D::toString()
{
    stringstream ss;
    ss << "CNPoint2D: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

double CNPoint2D::distanceTo(shared_ptr<CNPoint2D> point)
{
    return sqrt(pow(this->x - point->x, 2) + pow(this->y - point->y, 2));
}

double CNPoint2D::distanceTo(shared_ptr<CNPosition> pos)
{
    return sqrt(pow(this->x - pos->x, 2) + pow(this->y - pos->y, 2));
}

double geometry::CNPoint2D::angleToPoint(shared_ptr<CNPoint2D> point)
{
    return atan2(point->y - this->y, point->x - this->x);
}

} /* namespace */
