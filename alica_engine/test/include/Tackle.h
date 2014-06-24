/*
 * Tackle.h
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#ifndef TACKLE_H_
#define TACKLE_H_

#include <engine/BasicBehaviour.h>

namespace alica
{

	class Tackle : public BasicBehaviour
	{
	public:
		Tackle();
		virtual ~Tackle();
		virtual void run(void* msg);
	protected:
		int callCounter;
	};

} /* namespace alica */

#endif /* TACKLE_H_ */
