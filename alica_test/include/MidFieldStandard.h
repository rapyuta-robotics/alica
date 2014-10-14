/*
 * MidFieldStandard.h
 *
 *  Created on: Jun 20, 2014
 *      Author: emmeda
 */

#ifndef MIDFIELDSTANDARD_H_
#define MIDFIELDSTANDARD_H_

#include "engine/BasicBehaviour.h"

namespace alicaTests
{


	class MidFieldStandard : public alica::BasicBehaviour
	{
	public:
		MidFieldStandard();
		virtual ~MidFieldStandard();
		virtual void run(void* msg);
		int callCounter;
	protected:
		virtual void initialiseParameters();
	};

} /* namespace alica */

#endif /* MIDFIELDSTANDARD_H_ */
