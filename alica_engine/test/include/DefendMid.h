/*
 * DefendMid.h
 *
 *  Created on: Jun 20, 2014
 *      Author: emmeda
 */

#ifndef DEFENDMID_H_
#define DEFENDMID_H_

#include "engine/BasicBehaviour.h"

namespace alica
{

	class DefendMid : public BasicBehaviour
	{
	public:
		DefendMid();
		virtual ~DefendMid();
		virtual void run(void* msg);
	protected:
		int callCounter;
	};

} /* namespace alica */

#endif /* DEFENDMID_H_ */
