/*
 * Tackle.h
 *
 *  Created on: Jun 20, 2014
 *      Author: Stephan Opfer
 */

#ifndef TACKLE_H_
#define TACKLE_H_

#include <engine/BasicBehaviour.h>

namespace alicaTests
{

	class Tackle : public alica::BasicBehaviour
	{
	public:
		Tackle();
		virtual ~Tackle();
		virtual void run(void* msg);
	protected:
		int callCounter;
		virtual void initialiseParameters();
	};

} /* namespace alica */

#endif /* TACKLE_H_ */
