/*
 * TermBuilder.h
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#ifndef TERMBUILDER_H_
#define TERMBUILDER_H_

#include <vector>
#include <memory>

using namespace std;

namespace AutoDiff
{
	class Constant;
	class ConstPower;
	class Cos;
	class Exp;
	class Log;
	class Product;
	class Sin;
	class Sum;
	class Term;
	class TermPower;
	class TVec;

	class TermBuilder
	{
	public:
		static shared_ptr<Term> constant(double value);
		static shared_ptr<Term> sum(vector<shared_ptr<Term>> terms);
		static shared_ptr<Term> sum(shared_ptr<Term> v1, shared_ptr<Term> v2,
									vector<shared_ptr<Term>> rest = vector<shared_ptr<Term>>());
		static shared_ptr<Term> product(shared_ptr<Term> v1, shared_ptr<Term> v2, vector<shared_ptr<Term>> rest =
												vector<shared_ptr<Term>>());
		static shared_ptr<Term> power(shared_ptr<Term> t, double power);
		static shared_ptr<Term> power(shared_ptr<Term> baseTerm, shared_ptr<Term> exponent);
		static shared_ptr<Term> exp(shared_ptr<Term> arg);
		static shared_ptr<Term> log(shared_ptr<Term> arg);
		static shared_ptr<Term> sin(shared_ptr<Term> arg);
		static shared_ptr<Term> cos(shared_ptr<Term> arg);
		static shared_ptr<Term> quadform(shared_ptr<Term> x1, shared_ptr<Term> x2, shared_ptr<Term> a11,
											shared_ptr<Term> a12, shared_ptr<Term> a21, shared_ptr<Term> a22);
		static shared_ptr<Term> normalDistribution(shared_ptr<TVec> args, shared_ptr<TVec> mean, double variance);
		static shared_ptr<Term> gaussian(shared_ptr<TVec> args, shared_ptr<TVec> mean, double variance);
		static shared_ptr<Term> sigmoid(shared_ptr<Term> arg, shared_ptr<Term> upperBound, shared_ptr<Term> lowerBound,
										shared_ptr<Term> mid, double steepness);
		static shared_ptr<Term> boundedValue(shared_ptr<Term> arg, shared_ptr<Term> leftBound,
												shared_ptr<Term> rightBound, double steepness);
		static shared_ptr<Term> boundedRectangle(shared_ptr<TVec> arg, shared_ptr<TVec> leftLower,
													shared_ptr<TVec> rightUpper, double steepness);
		static shared_ptr<Term> euclidianDistanceSqr(shared_ptr<TVec> one, shared_ptr<TVec> two);
		static shared_ptr<Term> euclidianDistance(shared_ptr<TVec> one, shared_ptr<TVec> two);
		static shared_ptr<Term> polynom(vector<shared_ptr<Term>> input, int degree, vector<shared_ptr<Term>> param);
	};

} /* namespace AutoDiff */

#endif /* TERMBUILDER_H_ */
