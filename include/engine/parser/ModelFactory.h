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
#include "PlanParser.h"
#include "../model/AlicaElement.h"
namespace alica
{
	class PlanParser;
	class ModelFactory
	{
	public:
		ModelFactory(PlanParser *p, shared_ptr<PlanRepository> rep);
		virtual ~ModelFactory();

		bool ignoreMasterPlanId;bool getIgnoreMasterPlanId();
		void setIgnoreMasterPlanId(bool value);
		Plan createPlan(tinyxml2::XMLDocument* node);
		const map<long, AlicaElement>& getElements() const;
		void setElements(const map<long, AlicaElement>& elements);

	private:
		PlanParser* parser;
		shared_ptr<PlanRepository> rep;
		map<long,AlicaElement> elements;
		void setAlicaElementAttributes(AlicaElement& ae, tinyxml2::XMLElement& ele);
		void addElement(const AlicaElement ae);
	};
} /* namespace Alica */

#endif /* MODELFACTORY_H_ */
