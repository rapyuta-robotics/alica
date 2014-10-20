/*
 * ConstraintBuilder.h
 *
 *  Created on: Sep 5, 2014
 *      Author: psp
 */

#ifndef CONSTRAINTBUILDER_H_
#define CONSTRAINTBUILDER_H_

#include <memory>

namespace autodiff {
	class Term;
	class TVec;
}

using namespace std;
using namespace autodiff;

namespace alica
{
	class ConstraintBuilder
	{
	public:
		const double steepnessWide = 0.005;
		const double steepnessDefault = 0.01;
		const double steepnessSteep = 10.0;

		static const shared_ptr<Term> TRUE;
		static const shared_ptr<Term> FALSE;

		static void setSteepness(double s);

		static shared_ptr<Term> distance(shared_ptr<TVec> t1, shared_ptr<TVec> t2);
		static shared_ptr<Term> distanceSqr(shared_ptr<TVec> t1, shared_ptr<TVec> t2);
		static shared_ptr<TVec> rotate(shared_ptr<TVec> vec, double alpha);
		static shared_ptr<Term> projectVectorOntoX(shared_ptr<TVec> origin, shared_ptr<TVec> dir, shared_ptr<Term> x);
		static shared_ptr<TVec> inCoordsOf(shared_ptr<TVec> point, shared_ptr<TVec> vec);
		static shared_ptr<Term> leftOf(shared_ptr<TVec> vec, shared_ptr<TVec> toCheck);
		static shared_ptr<Term> rightOf(shared_ptr<TVec> vec, shared_ptr<TVec> toCheck);
		static shared_ptr<Term> equals(shared_ptr<Term> t1, shared_ptr<Term> t2, shared_ptr<Term> tolerance);
		static shared_ptr<Term> equals(shared_ptr<TVec> t1, shared_ptr<TVec> t2, double tolerance);
		static shared_ptr<Term> not_(shared_ptr<Term> t);
		static shared_ptr<Term> and_(shared_ptr<Term> t1, shared_ptr<Term> t2);
		static shared_ptr<Term> or_(shared_ptr<Term> t1, shared_ptr<Term> t2);
		static shared_ptr<Term> ifThen(shared_ptr<Term> tif, shared_ptr<Term> tthen);
		static shared_ptr<Term> ifThenElse(shared_ptr<Term> tif, shared_ptr<Term> tthen, shared_ptr<Term> telse);
		static shared_ptr<Term> equiv(shared_ptr<Term> a, shared_ptr<Term> b);
		static shared_ptr<Term> constraintApply(shared_ptr<Term> constraint, shared_ptr<Term> utility);
	};

} /* namespace alica */

#endif /* CONSTRAINTBUILDER_H_ */
