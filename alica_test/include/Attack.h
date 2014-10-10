/*
 * Attack.h
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#ifndef ATTACK_H_
#define ATTACK_H_

#include <engine/BasicBehaviour.h>

namespace alica
{


	class Attack : public BasicBehaviour
	{
	public:
		Attack();
		virtual ~Attack();
		virtual void run(void* msg);
		int callCounter;
		int initCounter;

	protected:
		virtual void initialiseParameters();
	};

} /* namespace alica */

#endif /* ATTACK_H_ */
