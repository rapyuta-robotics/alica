#pragma once
#include <engine/USummand.h>
#include <string>
#include <vector>

namespace alica
{

class IAssignment;

class DistXContourTest : public USummand
{
public:
    DistXContourTest(double weight, const std::vector<std::pair<double, double>>& ContourPoints, double xMaxVal, double xMinVal);
    virtual ~DistXContourTest();
    virtual void cacheEvalData() override;
    double interpolate2D(double X1, double Y1, double X2, double Y2, double xPoint) const;
    virtual UtilityInterval eval(IAssignment ass) const override;

protected:
    std::vector<std::pair<double, double>> contourPoints;
    double xAlloBall;
    double xMaxVal;
    double xMinVal;
};

} /* namespace alica */
