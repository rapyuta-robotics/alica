#pragma once

#include <alica_solver_interface/Interval.h>
#include <engine/AlicaClock.h>

#include <autodiff/TermHolder.h>
#include <autodiff/TermPtr.h>

#include <memory>
#include <vector>

//#define GSOLVER_LOG

#ifdef GSOLVER_LOG
#include <fstream>
#endif

namespace alica
{

namespace reasoner
{
class GSolver
{
    class RpropResult;

  public:
    GSolver();
    ~GSolver();
    bool solve(autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits, double& out_util,
               std::vector<double>& o_solution);
    bool solveSimple(autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits);

    bool solve(autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits, const std::vector<double>& seeds,
               double sufficientUtility, double& out_util, std::vector<double>& o_solution);

    bool solveSimple(autodiff::TermPtr equation, autodiff::TermHolder& holder, const std::vector<Interval<double>>& limits, const std::vector<double>& seeds);

    int64_t getRuns() const { return _runs; }
    int64_t getFEvals() const { return _fevals; }
    int64_t getMaxFEvals() const { return _maxfevals; }
    void setMaxFEvals(int64_t maxfevals) { _maxfevals = maxfevals; }
    void setRPropConvergenceStepSize(double rPropConvergenceStepSize) { _rPropConvergenceStepSize = rPropConvergenceStepSize; }
    void setUtilitySignificanceThreshold(double utilitySignificanceThreshold) { _utilityThreshold = utilitySignificanceThreshold; }

  private:
    class ResultView
    {
      public:
        // resulting point, value and status info
        constexpr static int getRequiredSize(int dim) { return dim + 2; }
        ResultView(double* mem_loc, int dim)
            : _location(mem_loc)
            , _size(getRequiredSize(dim))
        {
        }
        double getUtil() const { return _location[_size - 2]; }
        void setUtil(double util) { _location[_size - 2] = util; }
        const double* getPoint() const { return _location; }

        double* editPoint() { return _location; }
        bool isAborted() const { return _location[_size - 1] < 0.0; }
        void setAborted() { _location[_size - 1] = -1.0; }
        void setOk() { _location[_size - 1] = 1.0; }
        int size() const { return _size; }
        int dim() const { return _size - 2; }

      private:
        double* _location;
        int _size;
    };

#ifdef GSOLVER_LOG
    void initLog();
    void log(double util, const double* point);
    void logStep();
    void closeLog();
#endif

    void ensureResultSpace(int count, int dim);
    ResultView getResultView(int count, int dim);
    void writeSolution(ResultView result, std::vector<double>& o_solution) const;

    void initialPointFromSeed(const autodiff::Tape& tape, const double* seed, ResultView o_res, const std::vector<Interval<double>>& limits,
                              double* o_value) const;
    void initialPoint(const autodiff::Tape& tape, ResultView o_res, const std::vector<Interval<double>>& limits, double* o_value);

    void rPropLoop(const autodiff::Tape& tape, const double* seed, const std::vector<Interval<double>>& limits, ResultView o_result);
    void rPropLoop(const autodiff::Tape& tape, const double* seed, const std::vector<Interval<double>>& limits, ResultView o_result, bool precise);

    int movePoint(int dim, double minStep, double* pointBuffer, const double* curGradient, const double* oldGradient,
                  const std::vector<Interval<double>>& limits);
    void initialStepSize(int dim, const std::vector<Interval<double>>& limits);
    bool evalResults(int numResults, int dim, const std::vector<Interval<double>>& limits);

    std::vector<double> _results;

    static int _fcounter;
    bool _seedWithUtilOptimum;

    AlicaClock _alicaClock;

    double _utilitySignificanceThreshold;

    std::vector<double> _rpropStepWidth;
    std::vector<double> _rpropStepConvergenceThreshold;

    double _utilityThreshold;
    AlicaTime _maxSolveTime;

    double _initialStepSize;
    double _rPropConvergenceStepSize;

    int64_t _runs;
    int64_t _fevals;
    int64_t _maxfevals;
#ifdef GSOLVER_LOG
    std::ofstream _sw;
#endif
};
} /* namespace reasoner */
} /* namespace alica */
