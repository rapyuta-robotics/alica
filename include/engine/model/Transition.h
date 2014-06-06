/*
 * Transition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TRANSITION_H_
#define TRANSITION_H_

#include "AlicaElement.h"
#include "PreCondition.h"
#include "State.h"

namespace alica
{

	/*
	 *
	 */
	class State;
	class Transition : public AlicaElement
	{
	public:
		Transition();
		virtual ~Transition();
		const PreCondition* getPreCondition() ;
		void setPreCondition(const PreCondition* preCondition);
		State* getOutState();
		State* getInState() ;
		void setInState(State* inState);
		void setOutState(State* outState);

	private:
		const PreCondition* preCondition;
		State* inState;
		State* outState;
	};

} /* namespace Alica */

#endif /* TRANSITION_H_ */
