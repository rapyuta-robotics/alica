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
#include "SyncTransition.h"

namespace alica
{

	/*
	 *
	 */
	class State;
	class SyncTransition;
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
		const SyncTransition* getSyncTransition() const;
		void setSyncTransition(const SyncTransition* syncTransition);

	private:
		const PreCondition* preCondition;
		State* inState;
		State* outState;
		const SyncTransition* syncTransition;
	};

} /* namespace Alica */

#endif /* TRANSITION_H_ */
