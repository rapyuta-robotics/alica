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
		// TODO Auto-generated constructor stub
		this->parser = p;
		this->rep = rep;
		this->ignoreMasterPlanId = false;


	}

	ModelFactory::~ModelFactory()
	{
		// TODO Auto-generated destructor stub
	}

	void ModelFactory::setIgnoreMasterPlanId(bool value)
	{
		this->ignoreMasterPlanId = value;
	}

	bool ModelFactory::getIgnoreMasterPlanId()
	{
		return this->ignoreMasterPlanId;
	}

	std::shared_ptr<Plan> ModelFactory::createPlan(tinyxml2::XMLDocument* node){
		tinyxml2::XMLElement* element = node->FirstChildElement("alica:Plan");
		long id = this->parser->parserId(element);
		cout << "ID " << id << endl;

//		Plan* plan = new Plan(id);
		std::shared_ptr<Plan> plan  = shared_ptr<Plan>(new Plan(id));
		plan->setFilename(this->parser->getCurrentFile());
		setAlicaElementAttributes(*plan, *element);

		string masterPlan = element->Attribute("masterPlan");

		if(!masterPlan.empty()){
			//TODO: PAUL c#  Zeile 468
			//Hier geht es morgen weiter
		}
		return plan;

	}

	void ModelFactory::setAlicaElementAttributes(AlicaElement& ae, tinyxml2::XMLElement& ele){
		string name = ele.Attribute("name");
		string comment = ele.Attribute("comment");

		if(!name.empty()){
			ae.setName(name);
		}else ae.setName("MISSING_NAME");
		if(!comment.empty()){
			ae.setComment(comment);
		}else ae.setComment("");
	}

} /* namespace Alica */
