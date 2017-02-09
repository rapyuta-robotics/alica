/*
 * Calculator.h
 *
 *  Created on: 07.02.2017
 *      Author: Stephan Opfer
 */
#pragma once

#include "CNPointTemplate.h"
#include "CNPointAllo.h"

#include <vector>

using std::vector;

namespace geometry
{

double normalizeAngle(double angle);
double deltaAngle(double angle1, double angle2);
double absDeltaAngle(double angle1, double angle2);
double distance(double x1, double y1, double x2, double y2);
double distancePointToLineSegment(double px, double py, double lx1, double ly1, double lx2, double ly2);

template <class T>
bool isInsideRectangle(const CNPointTemplate<T> &rectPointA, const CNPointTemplate<T> &rectPointB, const CNPointTemplate<T> &point);
template <class T>
bool isInsidePolygon(const vector<CNPointTemplate<T>> &polygon, const CNPointTemplate<T> &point);
template <class T>
double distancePointToLineSegment(double x, double y, const CNPointTemplate<T> &a, const CNPointTemplate<T> &b);
template <class T>
double distancePointToLine(const CNPointTemplate<T> &a, const CNPointTemplate<T> &b, const CNPointTemplate<T> &p);
template <class T>
bool outsideTriangle(const CNPointTemplate<T> &a, const CNPointTemplate<T> &b, const CNPointTemplate<T> &c, double tolerance,
                     const vector<CNPointTemplate<T>> &points);
template <class T>
bool leftOf(const CNPointTemplate<T> &a, const CNPointTemplate<T> &b);
template <class T>
T calculateMean(const vector<CNPointTemplate<T>> &values);
template <class T>
bool onSegment(const CNPointTemplate<T> &p, const CNPointTemplate<T> &q, const CNPointTemplate<T> &r);
template <class T>
int orientation(const CNPointTemplate<T> &p, const CNPointTemplate<T> &q, const CNPointTemplate<T> &r);
template <class T>
bool doIntersect(const CNPointTemplate<T> &p1, const CNPointTemplate<T> &q1, const CNPointTemplate<T> &p2, const CNPointTemplate<T> &q2);

/**
 *  Sign function (Vorzeichenfunktion)
 */
template <typename T>
static int sgn(const T val)
{
    return (T(0) < val) - (val < T(0));
}

} /* namespace geometry */
