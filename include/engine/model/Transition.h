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

namespace alica
{

	/*
	 *
	 */
	class Transition : public AlicaElement
	{
	public:
		Transition();
		virtual ~Transition();
		const PreCondition* getPreCondition() ;
		void setPreCondition(const PreCondition* preCondition);

	private:
		const PreCondition* preCondition;
	};

} /* namespace Alica */

#endif /* TRANSITION_H_ */
