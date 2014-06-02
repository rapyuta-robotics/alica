/*
 * ModelFactory.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#include "engine/parser/ModelFactory.h"
namespace alica
{

ModelFactory::ModelFactory(shared_ptr<PlanParser> p, shared_ptr<PlanRepository> rep)
{
	// TODO Auto-generated constructor stub
	this->p = p;
	this->ignoreMasterPlanId = false;

}

ModelFactory::~ModelFactory()
{
	// TODO Auto-generated destructor stub
}

void ModelFactory::setIgnoreMasterPlanId(bool value){
	this->ignoreMasterPlanId = value;
}

bool ModelFactory::getIgnoreMasterPlanId(){
	return this->ignoreMasterPlanId;
}

} /* namespace Alica */
