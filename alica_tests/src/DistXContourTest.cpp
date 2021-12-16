/*
 * DistXContourTest.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Stefan Jakob
 */
#include <alica_tests/DistXContourTest.h>
#include <alica_tests/TestWorldModel.h>

#include <engine/planselector/IAssignment.h>

namespace alica
{

DistXContourTest::DistXContourTest(double weight, const std::vector<std::pair<double, double>>& ContourPoints, double xMaxVal, double xMinVal)
        : USummand(weight)
{
    this->xMaxVal = xMaxVal;
    this->xMinVal = xMinVal;
    this->contourPoints = ContourPoints;
    this->xAlloBall = 0;
}

DistXContourTest::~DistXContourTest() {}

void DistXContourTest::cacheEvalData(const IAlicaWorldModel* wm)
{
    xAlloBall = alicaTests::TestWorldModel::getOne()->x;
    xAlloBall = alicaTests::TestWorldModel::getTwo()->x;
}

double DistXContourTest::interpolate2D(double X1, double Y1, double X2, double Y2, double xPoint) const
{
    return ((Y2 - Y1) / (X2 - X1) * (xPoint - X1) + Y1);
}

UtilityInterval DistXContourTest::eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel* wm) const
{
    UtilityInterval ui(0.0, 0.0);

    std::pair<double, double> lastpoint;
    lastpoint.first = -18000 / 2;
    lastpoint.second = xMinVal;

    std::pair<double, double> nextpoint;
    nextpoint.first = 18000 / 2;
    nextpoint.second = xMaxVal;

    double val = 0;
    if (contourPoints.empty()) {
        return ui;
    }
    for (int i = 0; i < static_cast<int>(contourPoints.size()); ++i) {
        if (contourPoints[i].first < xAlloBall && contourPoints[i].first > lastpoint.first) {
            lastpoint = contourPoints[i];
        }

        if (contourPoints[i].first > xAlloBall && contourPoints[i].first < nextpoint.first) {
            nextpoint = contourPoints[i];
        }
    }
    if (xAlloBall > 18000 / 2)
        val = xMaxVal;
    else if (xAlloBall < -18000 / 2)
        val = xMinVal;
    else
        val = interpolate2D(lastpoint.first, lastpoint.second, nextpoint.first, nextpoint.second, xAlloBall);

    ui.setMin(val);
    ui.setMax(val);
    return ui;
}

} /* namespace alica */
