/*
 * ConstraintUsingBehaviour.h
 *
 *  Created on: Oct 23, 2014
 *      Author: Philipp
 */

#ifndef CONSTRAINTUSINGBEHAVIOUR_H_
#define CONSTRAINTUSINGBEHAVIOUR_H_

#include <vector>

#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/ConstraintQuery.h>

using namespace std;

namespace alicaTests
{

	class ConstraintUsingBehaviour : public alica::BasicBehaviour
	{
	public:
		ConstraintUsingBehaviour();
		virtual ~ConstraintUsingBehaviour();
		virtual void run(void* msg);

		int getCallCounter();
		static vector<string> result;
	protected:
		int callCounter;
		virtual void initialiseParameters();
		shared_ptr<alica::ConstraintQuery> query;

	};

} /* namespace alica */

#endif /* CONSTRAINTUSINGBEHAVIOUR_H_ */
