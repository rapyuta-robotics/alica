/*
 * IEngineModule.h
 *
 *  Created on: Jul 7, 2014
 *      Author: snook
 */

#ifndef IENGINEMODULE_H_
#define IENGINEMODULE_H_

#include "AlicaEngine.h"
#include "IEngineModule.h"

namespace alica
{

	class IEngineModule
	{
	public:
		virtual ~IEngineModule()
		{
		}
		virtual void init(AlicaEngine* ae) = 0;
		virtual void close() = 0;
		virtual void tick(RunningPlan* root) = 0;
	};
}

#endif /* IENGINEMODULE_H_ */
