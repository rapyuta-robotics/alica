/**
 * @file   CNVec2D.cpp
 * @author Philipp Mandler (info@philipp-mandler.com)
 * @date   2016/12/24
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

CNVec2D::~CNVec2D() {}

string CNVec2D::toString()
{
    stringstream ss;
    ss << "CNVec2D: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

shared_ptr<CNVec2D> CNVec2D::clone()
{
    return make_shared<CNVec2D>(this->x, this->y);
}

/**
 * Calculates the length of this vector.
 *
 * @returns The length of the vector.
 */
double CNVec2D::length()
{
    return sqrt(x * x + y * y);
}

/**
 * Rotates the vector arround the origin by the given radians.
 *
 * @returns The rotated vector.
 */
shared_ptr<CNVec2D> CNVec2D::rotate(double radian)
{
    return make_shared<CNVec2D>(this->x * cos(radian) - this->y * sin(radian), this->x * sin(radian) + this->y * cos(radian));
}

/**
 * Calculates the direction of this vector.
 *
 * @returns The angle in radians.
 */
double CNVec2D::angle()
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

shared_ptr<CNVec2D> CNPositionEgo::egoToAllo(CNPositionAllo &me)
{
    shared_ptr<CNPositionAllo> alloVec = make_shared<CNPositionAllo>();

    // rotate rel point arround origin -> rel point with allo orientation
    double sin = sin(me.theta);
    double cos = cos(me.theta);

    double x = cos * this->x - sin * this->y;
    double y = sin * this->x - cos * this->y;

    // sum me pos and rel pos -> allo pos with allo rotaion
    alloVec->x = x + me.x;
    alloVec->y = y + me.y;

    return alloVec;
}

/**
 * Calculates the direction from this point to another point.
 *
 * @returns The angle in radians
 */
double geometry::CNVec2D::angleToPoint(shared_ptr<CNVec2D> point)
{
    return atan2(point->y - this->y, point->x - this->x);
}

/**
 * Calculates the distance from this point to another point.
 *
 * @returns The distance.
 */
double CNVec2D::distanceTo(shared_ptr<CNVec2D> point)
{
    return sqrt(pow(this->x - point->x, 2) + pow(this->y - point->y, 2));
}

/**
 * Calculates the distance from this point to a CNPositionBase.
 *
 * @returns The distance.
 */
double CNVec2D::distanceTo(shared_ptr<CNPositionBase> pos)
{
    return sqrt(pow(this->x - pos->x, 2) + pow(this->y - pos->y, 2));
}

/**
 * Calculates a normailzed vector.
 *
 * @returns The normalized vector.
 */
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
        cerr << "CNVec2D: Trying to normalize (0, 0)!" << endl;
    }

    return norm;
}

/* Operators */

// Scalar
shared_ptr<CNVec2D> CNVec2D::operator*(const double &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x *= right;
    ret->y *= right;
    return ret;
}

shared_ptr<CNVec2D> CNVec2D::operator/(const double &right)
{
    auto ret = make_shared<CNVec2D>(this->x, this->y);
    ret->x /= right;
    ret->y /= right;
    return ret;
}

// CNVec2D

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

/* Right handed operators */

// Scalar

shared_ptr<CNVec2D> operator*(const double &left, const shared_ptr<CNVec2D> &right)
{
    auto ret = make_shared<CNVec2D>(right->x, right->y);
    ret->x *= left;
    ret->y *= left;
    return ret;
}

} /* namespace geometry */
