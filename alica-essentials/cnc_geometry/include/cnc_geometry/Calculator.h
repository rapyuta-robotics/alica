/*
 * Calculator.h
 *
 *  Created on: 07.02.2017
 *      Author: Stephan Opfer
 */
#pragma once

#include "CNPointTemplate.h"
#include "CNPointAllo.h"
#include "CNPointEgo.h"

#include <vector>

using std::vector;

namespace geometry
{

/**
 * Normalizes the given angle between M_PI and -M_PI.
 * @param angle to be normalized
 * @return angle normalized between M_PI and -M_PI
 */
inline double normalizeAngle(double angle)
{
    if (angle > M_PI)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
    }
    else if (angle < -M_PI)
    {
        while (angle < -M_PI)
            angle += 2 * M_PI;
    }
    return angle;
}

/**
 * Calculates the normalized difference between the two given angles.
 * @param angle1
 * @param angle2
 * @return Normalized difference angle
 */
inline double deltaAngle(double angle1, double angle2)
{
    return normalizeAngle(angle2 - angle1);
}

/**
 * Calculates the normalized absolute difference between the two given angles.
 * @param angle1
 * @param angle2
 * @return Normalized absolute difference angle
 */
inline double absDeltaAngle(double angle1, double angle2)
{
    return abs(deltaAngle(angle1, angle2));
}

/**
 * Calculates the distance between two 2d points represented by 2 doubles, each.
 * @param x1 X coordinate of point 1
 * @param y1 Y coordinate of point 1
 * @param x2 X coordinate of point 2
 * @param y2 Y coordinate of point 2
 * @return Distance between the two points.
 */
inline double distance(double x1, double y1, double x2, double y2)
{
    double dx = (x1 - x2);
    double dy = (y1 - y2);

    return sqrt(dx * dx + dy * dy);
}

/**
 * Checks whether the given point is inside or on the edge of the rectangle described by two rectangle points.
 * The rectangle points have to be diagonal opposing corner points of the rectangle.
 * @param rectPointA Rectangle corner point
 * @param rectPointB Rectangle corner point
 * @param point Point to be checked
 * @return True if the given point is inside or on the edge of the rectangle.
 */
template <class T>
bool isInsideRectangle(const CNPointTemplate<T> &rectPointA, const CNPointTemplate<T> &rectPointB, const CNPointTemplate<T> &point)
{
    double minX = min(rectPointA->x, rectPointB->x);
    double maxX = max(rectPointA->x, rectPointB->x);
    double minY = min(rectPointA->y, rectPointB->y);
    double maxY = max(rectPointA->y, rectPointB->y);

    return point->x >= minX && point->x <= maxX && point->y >= minY && point->y <= maxY;
}

/**
 * Given three collinear points p, q, r, the function checks if point q lies on line segment 'pr'
 * @param p point determining the line
 * @param q point determining the line
 * @param r point to check
 * @return True if point q lies on line segment 'pr'.
 */
template <class T>
bool onSegment(const CNPointTemplate<T> &p, const CNPointTemplate<T> &q, const CNPointTemplate<T> &r)
{
    if (q->x <= max(p->x, r->x) && q->x >= min(p->x, r->x) && q->y <= max(p->y, r->y) && q->y >= min(p->y, r->y))
    {
        return true;
    }
    return false;
}

/**
 * Finds the orientation of an ordered triplet (p, q, r).
 * @param p
 * @param q
 * @param r
 * @return 0 --> p, q and r are colinear; 1 --> Clockwise; 2 --> Counterclockwise
 */
template <class T>
int orientation(const CNPointTemplate<T> &p, const CNPointTemplate<T> &q, const CNPointTemplate<T> &r)
{
    int val = (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);

    if (val == 0)
        return 0;             // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

/**
 * Determines whether line segment 'p1q1' and 'p2q2' do intersect.
 * @param p1 first point of first line segment
 * @param q1 second point of first line segment
 * @param p2 first point of second line segment
 * @param q2 second point of second line segment
 * @return True if line segment 'p1q1' and 'p2q2' intersect
 */
template <class T>
bool doIntersect(const CNPointTemplate<T> &p1, const CNPointTemplate<T> &q1, const CNPointTemplate<T> &p2, const CNPointTemplate<T> &q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and p2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}

/**
 * Distance between the point p and the line described by point a and b.
 * @param a first point on the line
 * @param b second point on the line
 * @param p point not on the line
 * @return Distance between the p and line ab
 */
template <class T>
double distancePointToLine(const CNPointTemplate<T> &a, const CNPointTemplate<T> &b, const CNPointTemplate<T> &p)
{
    auto a2p = p - a;
    auto a2b = b - a;
    return (a2p.x * a2b.y - a2p.y * a2b.x) / a2p.length();
}

/**
 * Distance between the point (x,y) and the line segment described by point a and b.
 * @param x X coordinate of the point to check
 * @param y Y coordinate of the point to check
 * @param a first point of the line segment
 * @param b second point of the line segment
 * @return Distance between (x,y) and line segment ab
 */
template <class T>
double distancePointToLineSegment(double x, double y, const CNPointTemplate<T> &a, const CNPointTemplate<T> &b)
{
    return distancePointToLineSegment(x, y, a.x, a.y, b.x, b.y);
}

/**
 * Distance between the point (px,py) and the line segment described by point (lx1, ly1) and (lx2, ly2).
 * @param px
 * @param py
 * @param lx1
 * @param ly1
 * @param lx2
 * @param ly2
 * @return Distance between (x,y) and line segment (lx1, ly1) and (lx2, ly2).
 */
inline double distancePointToLineSegment(double px, double py, double lx1, double ly1, double lx2, double ly2)
{
    double abx = lx2 - lx1;
    double aby = ly2 - ly1;
    double apx = px - lx1;
    double apy = py - ly1;

    double angle1 = atan2(apy, apx);
    double angle2 = atan2(aby, abx);

    double alpha = angle1 - angle2;
    if (alpha < -M_PI)
    {
        alpha += 2.0 * M_PI;
    }
    else if (alpha > M_PI)
    {
        alpha -= 2.0 * M_PI;
    }
    double distAtoP = sqrt(apx * apx + apy * apy);
    if (alpha > M_PI / 2 || alpha < -M_PI / 2)
    {
        return distAtoP;
    }

    double dist1 = cos(alpha) * distAtoP;
    if (dist1 > sqrt(abx * abx + aby * aby))
    {
        return sqrt(pow(px - lx2, 2) + pow(py - ly2, 2));
    }
    else
    {
        return abs(sin(alpha)) * distAtoP;
    }
}

/**
 * Determines whether the given point is inside the given polygon.
 * @param polygon vector of points describing the polygon
 * @param p point to check
 * @return True if the point p lies inside the polygon
 */
template <class T>
bool isInsidePolygon(const vector<CNPointTemplate<T>> &polygon, const CNPointTemplate<T> &p)
{
    // There must be at least 3 points to build a polygon
    if (polygon.size() < 3)
        return false;

    // Create a point for line segment from p to infinite
    CNPointTemplate<T> extreme = CNPointTemplate<T>(30000, p->y);

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i + 1) % polygon.size();

        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is collinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], p, polygon[next]) == 0)
                return !onSegment(polygon[i], p, polygon[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count & 1; // Same as (count%2 == 1)
}

/**
 * Determines whether the given points are all outside the triangle (a,b,c) with respect to the given tolerance.
 * @param a
 * @param b
 * @param c
 * @param tolerance
 * @param points
 * @return True if all points are outside the triangle (a,b,c).
 */
template <class T>
bool outsideTriangle(const CNPointTemplate<T> &a, const CNPointTemplate<T> &b, const CNPointTemplate<T> &c, double tolerance,
                     const vector<CNPointTemplate<T>> &points)
{
    T a2b = b - a;
    T b2c = c - b;
    T c2a = a - c;

    T a2p;
    T b2p;
    T c2p;
    T p;

    for (int i = 0; i < points->size(); i++)
    {
        p = points->at(i);
        a2p = p - a;
        b2p = p - b;
        c2p = p - c;

        if ((a2p->x * a2b->y - a2p->y * a2b->x) / a2p->normalize()->length() < tolerance &&
            (b2p->x * b2c->y - b2p->y * b2c->x) / b2p->normalize()->length() < tolerance &&
            (c2p->x * c2a->y - c2p->y * c2a->x) / c2p->normalize()->length() < tolerance)
        {
            return false;
        }
    }
    return true;
}

/**
 * Determines whether the point a is left of vector b
 * @param a point
 * @param b vector
 * @return True is a is left of b.
 */
template <class T>
bool leftOf(const CNPointTemplate<T> &a, const CNPointTemplate<T> &b)
{
    return (a->x * b->y - a->y * b->x) < 0;
}

/**
 * Calculates the mean of all given points.
 * @param points
 * @return The mean of all given points.
 */
template <typename T>
typename std::enable_if<std::is_base_of<CNPointTemplate<T>, T>::value, T>::type
calculateMean(const vector<T> &points)
{
    if (points.empty())
    {
        return T();
    }
    T ret;
    for (auto &iter : points)
    {
        ret.x += iter.x;
        ret.y += iter.y;
        ret.z += iter.z;
    }
    ret.x /= points.size();
    ret.y /= points.size();
    ret.z /= points.size();
    return ret;
}
/**
 *  Sign function (Vorzeichenfunktion)
 */
template <typename T>
static int sgn(const T val)
{
    return (T(0) < val) - (val < T(0));
}

} /* namespace geometry */
