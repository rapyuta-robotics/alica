/*
 * Assignment.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef ASSIGNMENT_H_
#define ASSIGNMENT_H_

using namespace std;

#include "IAssignment.h"

namespace alica
{

	class Plan;
	class StateCollection;

	class Assignment : public IAssignment
	{
	public:
		Assignment();
		virtual ~Assignment();
		Plan* getPlan();
		void setPlan(Plan* plan);
		StateCollection* getRobotStateMapping();

	protected:
		Plan* plan;
		StateCollection* robotStateMapping;
	};

} /* namespace alica */

#endif /* ASSIGNMENT_H_ */
