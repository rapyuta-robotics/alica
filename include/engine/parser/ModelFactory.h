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
#include "../model/Parametrisation.h"
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
		Plan* createPlan(tinyxml2::XMLDocument* node);
		const map<long, AlicaElement*>& getElements() const;
		void setElements(const map<long, AlicaElement*>& elements);
		string getNameOfNode(tinyxml2::XMLElement* node);

	private:
		static const string entryPoints;
		static const string states;
		static const string transitions;
		static const string conditions;
		static const string vars;
		static const string synchronisations;
		static const string rating;
		static const string state;
		static const string task;
		static const string inTransitions;
		static const string outTransitions;
		static const string plans;
		static const string parametrisation;
		static const string subplan;
		static const string subvar;
		static const string var;

		PlanParser* parser;
		shared_ptr<PlanRepository> rep;
		map<long, AlicaElement*> elements;
		list<pair<long, long>> stateInTransitionReferences;
		list<pair<long, long>> stateOutTransitionReferences;
		list<pair<long, long>> statePlanReferences;
		list<pair<long, long>> paramSubPlanReferences;
		list<pair<long, long>> paramSubVarReferences;
		list<pair<long, long>> paramVarReferences;

		void setAlicaElementAttributes(AlicaElement* ae, tinyxml2::XMLElement* ele);
		EntryPoint* createEntryPoint(tinyxml2::XMLElement* element);
		State* createState(tinyxml2::XMLElement* element);
		Parametrisation* createParametrisation(tinyxml2::XMLElement* element);
		bool isReferenceNode(tinyxml2::XMLElement* node);
		void addElement(AlicaElement* ae);

	protected:

	};
} /* namespace Alica */

#endif /* MODELFACTORY_H_ */
