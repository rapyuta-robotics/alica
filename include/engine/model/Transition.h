/*
 * Transition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TRANSITION_H_
#define TRANSITION_H_

#include "AlicaElement.h"
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
		State* getInState() const;
		void setInState(State* inState);
		State* getOutState() const;
		void setOutState(State* outState);

	private:
		State* inState;
		State* outState;
	};

} /* namespace Alica */

#endif /* TRANSITION_H_ */
