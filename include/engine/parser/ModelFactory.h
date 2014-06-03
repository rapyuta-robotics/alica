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
#include "tinyxml2.h"

#include "../PlanRepository.h"
#include "../model/Plan.h"
namespace alica
{
	class PlanParser;

	class ModelFactory
	{
	public:
		ModelFactory(PlanParser *p, shared_ptr<PlanRepository> rep);
		virtual ~ModelFactory();

		bool ignoreMasterPlanId;
		bool getIgnoreMasterPlanId();
		void setIgnoreMasterPlanId(bool value);
		Plan createPlan(tinyxml2::XMLDocument node);

	protected:
		PlanParser* parser;
		shared_ptr<PlanRepository> rep;
	};
} /* namespace Alica */

#endif /* MODELFACTORY_H_ */
