/*
 * GeometryCalculator.cpp
 *
 *  Created on: 15.11.2014
 *      Author: Tobias Schellien
 */

#include <GeometryCalculator.h>

#include <limits>

namespace geometry
{

/**
 * @return angle normalized between M_PI and -M_PI.
 */
double normalizeAngle(double angle)
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

double deltaAngle(double angle1, double angle2)
{
    return normalizeAngle(angle2 - angle1);
}

double distance(double x1, double y1, double x2, double y2)
{
    double dx = (x1 - x2);
    double dy = (y1 - y2);

    return sqrt(dx * dx + dy * dy);
}

bool isInsideRectangle(shared_ptr<CNPoint2D> &rectPointA, shared_ptr<CNPoint2D> &rectPointB, shared_ptr<CNPoint2D> &point)
{
    double minX = min(rectPointA->x, rectPointB->x);
    double maxX = max(rectPointA->x, rectPointB->x);
    double minY = min(rectPointA->y, rectPointB->y);
    double maxY = max(rectPointA->y, rectPointB->y);

    return point->x >= minX && point->x <= maxX && point->y >= minY && point->y <= maxY;
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(shared_ptr<CNPoint2D> &p, shared_ptr<CNPoint2D> &q, shared_ptr<CNPoint2D> &r)
{
    if (q->x <= max(p->x, r->x) && q->x >= min(p->x, r->x) && q->y <= max(p->y, r->y) && q->y >= min(p->y, r->y))
    {
        return true;
    }
    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(shared_ptr<CNPoint2D> &p, shared_ptr<CNPoint2D> &q, shared_ptr<CNPoint2D> &r)
{
    int val = (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);

    if (val == 0)
        return 0;             // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(shared_ptr<CNPoint2D> &p1, shared_ptr<CNPoint2D> &q1, shared_ptr<CNPoint2D> &p2, shared_ptr<CNPoint2D> &q2)
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
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}

double distancePointToLine(shared_ptr<CNPoint2D> &a, shared_ptr<CNPoint2D> &b, shared_ptr<CNPoint2D> &p)
{
    auto a2p = p - a;
    auto a2b = b - a;
    //	cout << "DistToLine: " << (a2p->x * a2b->y - a2p->y * a2b->x) / a2p->length() << "\n A:" << a->toString() << " B: " <<  b->toString() << " P: " <<
    //p->toString()  << endl;
    return (a2p->x * a2b->y - a2p->y * a2b->x) / a2p->length();
}

// Returns true if the point p lies inside the polygon
bool isInsidePolygon(vector<shared_ptr<CNPoint2D>> &polygon, shared_ptr<CNPoint2D> &point)
{
    // There must be at least 3 points to build a polygon
    if (polygon.size() < 3)
        return false;

    // Create a point for line segment from p to infinite
    shared_ptr<CNPoint2D> extreme = make_shared<CNPoint2D>(30000, point->y);

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i + 1) % polygon.size();

        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], point, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], point, polygon[next]) == 0)
                return !onSegment(polygon[i], point, polygon[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count & 1; // Same as (count%2 == 1)
}

double distancePointToLineSegment(double x, double y, shared_ptr<CNPoint2D> &a, shared_ptr<CNPoint2D> &b)
{
    return distancePointToLineSegment(x, y, a->x, a->y, b->x, b->y);
}

double distancePointToLineSegment(double px, double py, double lx1, double ly1, double lx2, double ly2)
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

double absDeltaAngle(double angle1, double angle2)
{
    double ret = abs(angle1 - angle2);
    if (ret > M_PI)
    {
        return M_PI * 2 - ret;
    }
    else
    {
        return ret;
    }
}

bool outsideTriangle(shared_ptr<CNPoint2D> &a, shared_ptr<CNPoint2D> &b, shared_ptr<CNPoint2D> &c, double tolerance,
                     shared_ptr<vector<shared_ptr<CNPoint2D>>> &points)
{
    shared_ptr<CNPoint2D> a2b = b - a;
    shared_ptr<CNPoint2D> b2c = c - b;
    shared_ptr<CNPoint2D> c2a = a - c;

    shared_ptr<CNPoint2D> a2p;
    shared_ptr<CNPoint2D> b2p;
    shared_ptr<CNPoint2D> c2p;
    shared_ptr<CNPoint2D> p;

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

bool outsideCorridore(shared_ptr<geometry::CNPoint2D> &start, shared_ptr<geometry::CNPoint2D> &end, double passCorridorWidth,
                      shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> &points)
{
    for (int i = 0; i < points->size(); i++)
    {
        if (distancePointToLineSegment(points->at(i)->x, points->at(i)->y, start, end) < passCorridorWidth)
        {
            return false;
        }
    }
    return true;
}


//TODO LMA: find solution for this (fitting name or remove and deal with  passpoint in behaviour)
bool outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                               shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
{
    for (int i = 0; i < points->size(); i++)
    {
        if (geometry::distancePointToLineSegment(points->at(i)->x, points->at(i)->y, ball, passPoint) < passCorridorWidth &&
            ball->distanceTo(points->at(i)) < ball->distanceTo(passPoint) - 100)
        {
            return false;
        }
    }
    return true;
}

bool leftOf(shared_ptr<CNPoint2D> &a, shared_ptr<CNPoint2D> &b)
{
    return (a->x * b->y - a->y * b->x) < 0;
}

shared_ptr<CNPoint3D> calculateMean(vector<shared_ptr<CNPoint3D>> values)
{
    if (values.size() == 0)
    {
        return nullptr;
    }
    shared_ptr<CNPoint3D> ret = make_shared<CNPoint3D>(0.0, 0.0, 0.0);
    for (shared_ptr<CNPoint3D> iter : values)
    {
        ret->x += iter->x;
        ret->y += iter->y;
        ret->z += iter->z;
    }
    ret->x /= values.size();
    ret->y /= values.size();
    ret->z /= values.size();
    return ret;
}
} /* namespace geometry */
