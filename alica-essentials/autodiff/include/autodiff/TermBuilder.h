#pragma once

#include "TVec.h"
#include "Types.h"
#include <cmath>
namespace autodiff
{

class TermBuilder
{
  public:
    static TermPtr quadform(const TermPtr x1, const TermPtr x2, const TermPtr a11, const TermPtr a12, const TermPtr a21, const TermPtr a22);

    template <int DIM>
    static TermPtr normalDistribution(const TVec<DIM>& args, const TVec<DIM>& mean, double variance);
    template <int DIM>
    static TermPtr gaussian(const TVec<DIM>& args, const TVec<DIM>& mean, double variance);
    static TermPtr sigmoid(const TermPtr arg, const TermPtr upperBound, const TermPtr lowerBound, const TermPtr mid, double steepness);
    static TermPtr boundedValue(const TermPtr arg, const TermPtr leftBound, const TermPtr rightBound);

    static TermPtr boundedRectangle(const TVec<2>& arg, const TVec<2>& leftLower, const TVec<2>& rightUpper);

    template <int DIM>
    static TermPtr polynom(const TVec<DIM>& input, int degree, const TVec<DIM>& param);
};

template <int DIM>
TermPtr TermBuilder::normalDistribution(const TVec<DIM>& args, const TVec<DIM>& mean, double variance)
{
    return args[0]->getOwner()->exp((args - mean)->normSquared() * (-0.5 / variance)) * (1 / sqrt(2.0 * M_PI * variance));
}

template <int DIM>
TermPtr TermBuilder::gaussian(const TVec<DIM>& args, const TVec<DIM>& mean, double variance)
{
    return args[0]->getOwner()->exp((args - mean)->normSquared() * (-0.5 / variance));
}

template <int DIM>
TermPtr TermBuilder::polynom(const TVec<DIM>& input, int degree, const TVec<DIM>& param)
{
    TermPtr ret = input[0]->getOwner()->zeroConstant();
    for (int i = 0; i < DIM; ++i) {
        TermPtr t = input[i]->getOwner()->constPower(input[i], degree);
        ret = ret + t * param[i];
    }
    return ret;
}

} /* namespace autodiff */
