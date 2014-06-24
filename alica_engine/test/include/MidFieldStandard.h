/*
 * MidFieldStandard.h
 *
 *  Created on: Jun 20, 2014
 *      Author: emmeda
 */

#ifndef MIDFIELDSTANDARD_H_
#define MIDFIELDSTANDARD_H_

#include "engine/BasicBehaviour.h"

namespace alica
{


	class MidFieldStandard : public BasicBehaviour
	{
	public:
		MidFieldStandard();
		virtual ~MidFieldStandard();
		virtual void run(void* msg);
	protected:
		int callCounter;
	};

} /* namespace alica */

#endif /* MIDFIELDSTANDARD_H_ */
