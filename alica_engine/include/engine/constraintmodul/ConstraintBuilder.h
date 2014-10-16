/*
 * ConstraintBuilder.h
 *
 *  Created on: Sep 5, 2014
 *      Author: psp
 */

#ifndef CONSTRAINTBUILDER_H_
#define CONSTRAINTBUILDER_H_

#include <AutoDiff.h>

#include <memory>

using namespace std;
using namespace autodiff;

namespace alica
{
	// should be in impera/cn-alica-ros-pkg/AlicaEngine/src/Engine/ConstraintBuilder.cs
	class ConstraintBuilder
	{
	public:
		const double steepnessWide = 0.005;
		const double steepnessDefault = 0.01;
		const double steepnessSteep = 10.0;

		static const shared_ptr<Term> TRUE;
		static const shared_ptr<Term> FALSE;

		static shared_ptr<Term> distance(shared_ptr<TVec> t1, shared_ptr<TVec> t2);
	};

} /* namespace alica */

#endif /* CONSTRAINTBUILDER_H_ */
