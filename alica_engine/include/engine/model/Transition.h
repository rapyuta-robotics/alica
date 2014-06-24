/*
 * Transition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TRANSITION_H_
#define TRANSITION_H_

#include "AlicaElement.h"

namespace alica
{

	class State;
	class SyncTransition;
	class PreCondition;

	class Transition : public AlicaElement
	{
	public:
		Transition();
		virtual ~Transition();
		PreCondition* getPreCondition() ;
		void setPreCondition(PreCondition* preCondition);
		State* getOutState();
		State* getInState() ;
		void setInState(State* inState);
		void setOutState(State* outState);
		SyncTransition* getSyncTransition();
		void setSyncTransition(SyncTransition* syncTransition);

	private:
		PreCondition* preCondition;
		State* inState;
		State* outState;
		SyncTransition* syncTransition;
	};

} /* namespace Alica */

#endif /* TRANSITION_H_ */
