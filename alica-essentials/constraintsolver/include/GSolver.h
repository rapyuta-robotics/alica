/*
 * GSolver.h
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#ifndef GSOLVER_H_
#define GSOLVER_H_

#include <AutoDiff.h>
#include <engine/AlicaClock.h>

#include <memory>
#include <vector>

#include <fstream>

using namespace std;

namespace alica {

namespace reasoner {
class GSolver {
protected:
    class RpropResult;

public:
    GSolver();
    ~GSolver();

    shared_ptr<vector<double>> solve(shared_ptr<Term> equation, shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
            shared_ptr<vector<shared_ptr<vector<double>>>> limits, double* util);
    bool solveSimple(shared_ptr<Term> equation, shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
            shared_ptr<vector<shared_ptr<vector<double>>>> limits);
    shared_ptr<vector<double>> solve(shared_ptr<Term> equation, shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
            shared_ptr<vector<shared_ptr<vector<double>>>> limits, shared_ptr<vector<shared_ptr<vector<double>>>> seeds,
            double sufficientUtility, double* util);
    bool solveSimple(shared_ptr<Term> equation, shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
            shared_ptr<vector<shared_ptr<vector<double>>>> limits,
            shared_ptr<vector<shared_ptr<vector<double>>>> seeds);
    shared_ptr<vector<double>> solveTest(shared_ptr<Term> equation,
            shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
            shared_ptr<vector<shared_ptr<vector<double>>>> limits);
    shared_ptr<vector<double>> solveTest(shared_ptr<Term> equation,
            shared_ptr<vector<shared_ptr<autodiff::Variable>>> args,
            shared_ptr<vector<shared_ptr<vector<double>>>> limits, int maxRuns, bool* found);

    long getRuns();
    void setRuns(long runs);
    long getFEvals();
    void setFEvals(long fevals);
    long getMaxFEvals();
    void setMaxFEvals(long maxfevals);
    double getRPropConvergenceStepSize();
    void setRPropConvergenceStepSize(double rPropConvergenceStepSize);
    void setUtilitySignificanceThreshold(double utilitySignificanceThreshold);

    AlicaClock* getAlicaClock();
    void setAlicaClock(AlicaClock* clock);

protected:
    static int _fcounter;
    bool _seedWithUtilOptimum;

    AlicaClock* _alicaClock;

    void initLog();
    void log(double util, shared_ptr<vector<double>>& val);
    void logStep();
    void closeLog();
    shared_ptr<vector<double>> initialPointFromSeed(shared_ptr<RpropResult>& res, shared_ptr<vector<double>>& seed);
    shared_ptr<vector<double>> initialPoint(shared_ptr<RpropResult>& res);
    shared_ptr<RpropResult> rPropLoop(shared_ptr<vector<double>> seed);
    shared_ptr<RpropResult> rPropLoop(shared_ptr<vector<double>> seed, bool precise);
    shared_ptr<RpropResult> rPropLoopSimple(shared_ptr<vector<double>> seed);
    void initialStepSize();
    bool evalResults();

    class RpropResult : public enable_shared_from_this<RpropResult> {
    public:
        shared_ptr<vector<double>> _initialValue;
        shared_ptr<vector<double>> _finalValue;
        double _initialUtil;
        double _finalUtil;
        bool _aborted;

        int compareTo(shared_ptr<RpropResult> other);
        double distanceTraveled();
        double distanceTraveledNormed(vector<double> ranges);

        //				friend bool operator<(const shared_ptr<RpropResult>& left, const
        // shared_ptr<RpropResult>& right);
        //				friend bool operator>(const shared_ptr<RpropResult>& left, const
        // shared_ptr<RpropResult>& right);
    };

protected:
    double _utilitySignificanceThreshold = 1E-22;
    // Random rand;
    int _dim;
    shared_ptr<vector<shared_ptr<vector<double>>>> _limits;
    vector<double> _ranges;
    vector<double> _rpropStepWidth;
    vector<double> _rpropStepConvergenceThreshold;
    double _utilityThreshold;
    AlicaTime _maxSolveTime;
    // //vector<double>[] seeds;
    shared_ptr<ICompiledTerm> _term;

    ofstream sw;

    vector<shared_ptr<RpropResult>> _rResults;

    long _runs;
    long _fevals;
    long _maxfevals;
    double _initialStepSize = 0.005;
    double _rPropConvergenceStepSize;
};
} /* namespace reasoner */
} /* namespace alica */

#endif /* GSOLVER_H_ */
