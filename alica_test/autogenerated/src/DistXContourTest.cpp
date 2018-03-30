/*
 * DistXContourTest.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Stefan Jakob
 */

#include <DistXContourTest.h>
#include "TestWorldModel.h"

namespace alica {

DistXContourTest::DistXContourTest(double weight, string name, long id, vector<long>& relevantEntryPointIds,
        vector<pair<double, double>>& ContourPoints, double xMaxVal, double xMinVal, int ownId) {
    this->ownId = ownId;
    this->weight = weight;
    this->name = name;
    this->id = id;
    this->relevantEntryPointIds = relevantEntryPointIds;

    this->xMaxVal = xMaxVal;
    this->xMinVal = xMinVal;
    this->contourPoints = ContourPoints;
    this->xAlloBall = 0;
}

DistXContourTest::~DistXContourTest() {}

void DistXContourTest::cacheEvalData() {
    xAlloBall = alicaTests::TestWorldModel::getOne()->x;
    xAlloBall = alicaTests::TestWorldModel::getTwo()->x;
}

double DistXContourTest::interpolate2D(double X1, double Y1, double X2, double Y2, double xPoint) {
    return ((Y2 - Y1) / (X2 - X1) * (xPoint - X1) + Y1);
}

UtilityInterval DistXContourTest::eval(IAssignment* ass) {
    ui.setMin(0.0);
    ui.setMax(0.0);

    pair<double, double> lastpoint;
    lastpoint.first = -18000 / 2;
    lastpoint.second = xMinVal;

    pair<double, double> nextpoint;
    nextpoint.first = 18000 / 2;
    nextpoint.second = xMaxVal;

    double val = 0;
    if (this->contourPoints.size() == 0) {
        ui.setMax(0.0);
        return ui;
    }
    for (int i = 0; i < this->contourPoints.size(); ++i) {
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
