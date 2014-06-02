/*
 * ModelFactory.h
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef MODELFACTORY_H_
#define MODELFACTORY_H_

using namespace std;

#include <memory>

#include "../PlanRepository.h"

namespace alica
{
	class PlanParser;

	class ModelFactory
	{
	public:
		ModelFactory(shared_ptr<PlanParser> p, shared_ptr<PlanRepository> rep);
		virtual ~ModelFactory();

		bool ignoreMasterPlanId;
		bool getIgnoreMasterPlanId();
		void setIgnoreMasterPlanId(bool value);

	protected:
		shared_ptr<PlanParser> p;
		shared_ptr<PlanRepository> rep;
	};

} /* namespace Alica */

#endif /* MODELFACTORY_H_ */
