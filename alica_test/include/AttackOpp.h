/*
 * AttackOpp.h
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#ifndef ATTACKOPP_H_
#define ATTACKOPP_H_

#include <engine/BasicBehaviour.h>

namespace alica
{

	class AttackOpp : public BasicBehaviour
	{
	public:
		AttackOpp();
		virtual ~AttackOpp();
		virtual void run(void* msg);
	protected:
		int callCounter;
		virtual void initialiseParameters();
	};

} /* namespace alica */

#endif /* ATTACKOPP_H_ */
