#pragma once
#include <engine/USummand.h>
#include <string>
#include <vector>

namespace alica
{

class IAssignment;
class Blackboard;

class DistXContourTest : public USummand
{
public:
    DistXContourTest(double weight, const std::vector<std::pair<double, double>>& ContourPoints, double xMaxVal, double xMinVal);
    virtual ~DistXContourTest();
    void cacheEvalData(const Blackboard* worldModels) override;
    double interpolate2D(double X1, double Y1, double X2, double Y2, double xPoint) const;
    UtilityInterval eval(IAssignment ass, const Assignment* oldAss, const Blackboard* worldModels) const override;

protected:
    std::vector<std::pair<double, double>> contourPoints;
    double xAlloBall;
    double xMaxVal;
    double xMinVal;
};

} /* namespace alica */
