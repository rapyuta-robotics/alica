/*
 * PreCondition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PRECONDITION_H_
#define PRECONDITION_H_


#include <string>
#include <sstream>

#include "Condition.h"

using namespace std;
namespace alica
{

	class RunningPlan;

	/**
	 * A precondition guards a Plan or a Transition.
	 */
	class PreCondition : public Condition
	{
	public:
		PreCondition(long id = 0);
		virtual ~PreCondition();

		string toString();

		bool isEnabled() const;
		void setEnabled(bool enabled);

	private:
		bool enabled;
	};

} /* namespace Alica */

#endif /* PRECONDITION_H_ */
