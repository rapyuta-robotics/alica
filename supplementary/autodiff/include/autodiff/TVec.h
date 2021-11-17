#pragma once

#include "Term.h"
#include "TermHolder.h"
#include <array>
#include <type_traits>

namespace autodiff
{

template <int DIM>
class TVec
{
    static_assert(DIM > 0, "Vector dimensionality must be positive.");

  public:
    TVec(const std::array<TermPtr, DIM>& terms)
        : _terms(terms)
    {
    }
    template <class... Args>
    TVec(Args... args)
        : _terms{{args...}}
    {
    }

    inline TermPtr normSquared() const;
    inline TVec<DIM> normalized() const;
    int dimension() const { return DIM; }

    TermPtr getX() const { return _terms[0]; }
    template <int D = DIM>
    typename std::enable_if<(D > 1), TermPtr>::type getY() const
    {
        return _terms[1];
    }
    template <int D = DIM>
    typename std::enable_if<(D > 2), TermPtr>::type getZ() const
    {
        return _terms[2];
    }

    TermPtr innerProduct(const TVec<DIM>& o) const;

    TermPtr operator[](int index) const { return _terms[index]; }
    TermPtr& operator[](int index) { return _terms[index]; }

  private:
    std::array<TermPtr, DIM> _terms;
};
// Operators:
template <int DIM>
TVec<DIM> operator+(const TVec<DIM>& left, const TVec<DIM>& o);
template <int DIM>
TVec<DIM> operator-(const TVec<DIM>& left, const TVec<DIM>& o);
template <int DIM>
TVec<DIM> operator-(const TVec<DIM>& vector);
template <int DIM>
TVec<DIM> operator*(const TVec<DIM>& vector, TermPtr scalar);
template <int DIM>
TVec<DIM> operator*(const TVec<DIM>& vector, const double scalar);
template <int DIM>
TVec<DIM> operator*(TermPtr scalar, const TVec<DIM>& vector);
template <int DIM>
TVec<DIM> operator*(const double scalar, const TVec<DIM>& vector);
template <int DIM>
TermPtr operator*(const TVec<DIM>& left, const TVec<DIM>& o);

// Helpers:
template <int DIM>
inline TVec<DIM> crossProduct(const TVec<DIM>& t1, const TVec<DIM>& t2);
template <int DIM>
inline TermPtr distance(const TVec<DIM>& t1, const TVec<DIM>& t2);
template <int DIM>
inline TermPtr distanceSqr(const TVec<DIM>& t1, const TVec<DIM>& t2);
template <int DIM>
inline TVec<DIM> rotate(const TVec<DIM>& vec, double alpha);
template <int DIM>
inline TermPtr projectVectorOntoX(const TVec<DIM>& origin, const TVec<DIM>& dir, TermPtr x);
template <int DIM>
inline TVec<DIM> inCoordsOf(const TVec<DIM>& point, const TVec<DIM>& vec);
template <int DIM>
inline TermPtr leftOf(const TVec<DIM>& vec, const TVec<DIM>& toCheck);
template <int DIM>
inline TermPtr rightOf(const TVec<DIM>& vec, const TVec<DIM>& toCheck);
template <int DIM>
inline TermPtr equals(const TVec<DIM>& t1, const TVec<DIM>& t2, double tolerance);

/// Implementation:

template <>
inline TermPtr TVec<1>::normSquared() const
{
    return _terms[0]->getOwner()->constPower(_terms[0], 2);
}

template <int DIM>
inline TermPtr TVec<DIM>::normSquared() const
{
    TermHolder* h = _terms[0]->getOwner();
    TermPtr ret = h->constPower(_terms[0], 2.0);
    for (int i = 1; i < DIM; ++i) {
        ret = ret + h->constPower(_terms[i], 2.0);
    }
    return ret;
}

template <int DIM>
TVec<DIM> TVec<DIM>::normalized() const
{
    TermPtr a = normSquared();
    a = a->getOwner()->constPower(a, 0.5);
    TVec<DIM> ret;
    for (int i = 0; i < DIM; ++i) {
        ret[i] = (_terms[i] / a);
    }
    return ret;
}

template <int DIM>
TermPtr TVec<DIM>::innerProduct(const TVec<DIM>& o) const
{
    TermPtr ret = _terms[0] * o._terms[0];
    for (int i = 1; i < DIM; ++i) {
        ret = ret + (_terms[i] * o._terms[i]);
    }
    return ret;
}

template <int DIM>
TVec<DIM> operator+(const TVec<DIM>& left, const TVec<DIM>& right)
{
    TVec<DIM> ret;
    for (int i = 0; i < DIM; ++i) {
        ret[i] = left[i] + right[i];
    }
    return ret;
}

template <int DIM>
TVec<DIM> operator-(const TVec<DIM>& left, const TVec<DIM>& right)
{
    TVec<DIM> ret;
    for (int i = 0; i < DIM; ++i) {
        ret[i] = left[i] - right[i];
    }
    return ret;
}

template <int DIM>
TVec<DIM> operator-(const TVec<DIM>& vector)
{
    return vector * -1;
}

template <int DIM>
TVec<DIM> operator*(const TVec<DIM>& vector, TermPtr scalar)
{
    TVec<DIM> ret;
    for (int i = 0; i < DIM; ++i) {
        ret[i] = vector[i] * scalar;
    }
    return ret;
}

template <int DIM>
TVec<DIM> operator*(const TVec<DIM>& vector, const double scalar)
{
    return vector * vector[0]->getOwner()->constant(scalar);
}

template <int DIM>
TVec<DIM> operator*(TermPtr scalar, const TVec<DIM>& vector)
{
    return vector * scalar;
}

template <int DIM>
TVec<DIM> operator*(const double scalar, const TVec<DIM>& vector)
{
    return vector[0]->getOwner()->constant(scalar) * vector;
}

template <int DIM>
TermPtr operator*(const TVec<DIM>& left, const TVec<DIM>& right)
{
    return left.innerProduct(right);
}

template <>
inline TVec<3> crossProduct(const TVec<3>& t1, const TVec<3>& t2)
{
    return TVec<3>(t1.getY() * t2.getZ() - t1.getZ() * t2.getY(), t1.getZ() * t2.getX() - t1.getX() * t2.getZ(), t1.getX() * t2.getY() - t1.getY() * t2.getX());
}

template <int DIM>
inline TermPtr distanceSqr(const TVec<DIM>& one, const TVec<DIM>& two)
{
    return (one - two).normSquared();
}

template <int DIM>
inline TermPtr distance(const TVec<DIM>& one, const TVec<DIM>& two)
{
    return one[0]->getOwner()->constPower(distanceSqr(one, two), 0.5);
}

template <>
inline TVec<2> rotate(const TVec<2>& vec, double alpha)
{
    TermHolder* h = vec[0]->getOwner();
    TermPtr at = h->constant(alpha);
    TermPtr cosA = h->cos(at);
    TermPtr sinA = h->sin(at);

    return TVec<2>(cosA * vec.getX() - sinA * vec.getY(), sinA * vec.getX() + cosA * vec.getY());
}
template <>
inline TermPtr projectVectorOntoX(const TVec<2>& origin, const TVec<2>& dir, TermPtr x)
{
    return origin.getY() + dir.getY() * (x - origin.getX()) * x->getOwner()->constPower(dir.getX(), -1);
}

/**
 * Returns point in the coordinate system defined by vec and its rectangular.
 *
 * @param point A two-dimensional TVec
 * @param vec A two-dimensional TVec
 *
 * @return A two-dimensional TVec
 */
template <>
inline TVec<2> inCoordsOf(const TVec<2>& point, const TVec<2>& vec)
{
    TermPtr quo = point[0]->getOwner()->constPower(vec.normSquared(), -1.0);
    return TVec<2>((point.getX() * vec.getX() + point.getY() * vec.getY()) * quo, (point.getX() * vec.getY() - point.getY() * vec.getX()) * quo);
}

/**
 * Two dimensional geometry:
 * Returns if toCheck is left of vec.
 *
 * @param vec A TVec
 * @param toCheck A TVec
 *
 * @result A Term
 */
template <>
inline TermPtr leftOf(const TVec<2>& vec, const TVec<2>& toCheck)
{
    return ((toCheck.getX() * vec.getY()) - (toCheck.getY() * vec.getX())) < vec[0]->getOwner()->zeroConstant();
}

/**
 * Two dimensional geometry:
 * Returns if toCheck is right of vec.
 *
 * @param vec A TVec
 * @param toCheck A TVec
 *
 * @return A Term
 */
template <>
inline TermPtr rightOf(const TVec<2>& vec, const TVec<2>& toCheck)
{
    return ((toCheck.getX() * vec.getY()) - (toCheck.getY() * vec.getX())) > vec[0]->getOwner()->zeroConstant();
}

/**
 * Returns wether the distance between two n-dimensional vectors is less than tolerance.
 *
 * @param t1 A TVec
 * @param t2 A TVec
 * @param tolerance A double
 *
 * @return A Term
 */
template <int DIM>
inline TermPtr equals(const TVec<DIM>& t1, const TVec<DIM>& t2, double tolerance)
{
    return distanceSqr(t1, t2) < t1[0]->getOwner()->constant(tolerance * tolerance);
}

} /* namespace autodiff */
