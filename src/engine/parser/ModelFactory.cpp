/*
 * ModelFactory.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#include "engine/parser/ModelFactory.h"
namespace alica
{

	ModelFactory::ModelFactory(PlanParser* p, std::shared_ptr<PlanRepository> rep)
	{
		this->parser = p;
		this->rep = rep;
		this->ignoreMasterPlanId = false;
	}

	ModelFactory::~ModelFactory()
	{
	}

	void ModelFactory::setIgnoreMasterPlanId(bool value)
	{
		this->ignoreMasterPlanId = value;
	}

	bool ModelFactory::getIgnoreMasterPlanId()
	{
		return this->ignoreMasterPlanId;
	}

	Plan* ModelFactory::createPlan(tinyxml2::XMLDocument* node)
	{
		tinyxml2::XMLElement* element = node->FirstChildElement("alica:Plan");

		long id = this->parser->parserId(element);
		Plan* plan = new Plan(id);
		plan->setFilename(this->parser->getCurrentFile());
		setAlicaElementAttributes(plan, *element);

		string isMasterPlanAttr = element->Attribute("masterPlan");

		if (!isMasterPlanAttr.empty())
		{
			std::transform(isMasterPlanAttr.begin(), isMasterPlanAttr.end(), isMasterPlanAttr.begin(), ::tolower);

			if (isMasterPlanAttr.compare("true") == 0)
			{
				plan->setMasterPlan(true);
			}
		}

		string attr = element->Attribute("minCardinality");
		if (!attr.empty())
		{

		}
		attr = element->Attribute("maxCardinality");
		if (!attr.empty())
		{

		}
		attr = element->Attribute("utilityThreshold");
		if (!attr.empty())
		{

		}


		// insert into elements map
		addElement(plan);
		// insert into planrepository map
		this->rep.get()->getPlans().insert(pair<long, Plan*>(plan->getId(), plan));
		return plan;
	}

	void ModelFactory::setAlicaElementAttributes(AlicaElement* ae, tinyxml2::XMLElement& ele)
	{
		string name = ele.Attribute("name");
		string comment = ele.Attribute("comment");

		if (!name.empty())
		{
			ae->setName(name);
		}
		else
			ae->setName("MISSING_NAME");
		if (!comment.empty())
		{
			ae->setComment(comment);
		}
		else
			ae->setComment("");
	}

	const map<long, AlicaElement*>& ModelFactory::getElements() const
	{
		return this->elements;
	}

	void ModelFactory::setElements(const map<long, AlicaElement*>& elements)
	{
		this->elements = elements;
	}

	void ModelFactory::addElement(AlicaElement* ae)
	{
		if (this->elements.find(ae->getId()) != this->elements.end())
		{
			AlicaEngine::getInstance()->abort("PP: ERROR Double IDs: " + ae->getId());
		}
		elements.insert(pair<long, AlicaElement*>(ae->getId(), ae));
	}

} /* namespace Alica */

