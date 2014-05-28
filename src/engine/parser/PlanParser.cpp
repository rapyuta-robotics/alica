/*
 * PlanParser.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#include "engine/parser/PlanParser.h"
#include "SystemConfig.h"

namespace alica
{

PlanParser::PlanParser(std::shared_ptr<PlanRepository> rep)
{

//	this->mf = new ModelFactory(this, rep);
	this->rep = rep;

	this->sc = supplementary::SystemConfig::getInstance();
	this->esConfigRoot = this->sc->getConfigPath();

	std::string planDir = (*this->sc)["Alica"]->get<std::string>("Alica.PlanDir");
	std::string roleDir = (*this->sc)["Alica"]->get<std::string>("Alica.RoleDir");



}

PlanParser::~PlanParser()
{
	// TODO Auto-generated destructor stub
}

std::shared_ptr<Plan> PlanParser::ParsePlanTree(std::string masterplan)
{

	std::shared_ptr<Plan> p;
	return p;
}
std::shared_ptr<RoleSet> PlanParser::ParseRoleSet(std::string roleSetName, std::string roleSetDir)
{

	std::shared_ptr<RoleSet> r;
	return r;
}
std::shared_ptr<std::map<long, alica::AlicaElement> > PlanParser::GetParsedElements()
{

	std::shared_ptr<std::map<long, alica::AlicaElement> > map;
	return map;
}
void PlanParser::IgnoreMasterPlanId(bool val)
{
	this->mf->setIgnoreMasterPlanId(val);
}
std::string PlanParser::getCurrentFile()
{
	return this->currentFile;
}


void PlanParser::setCurrentFile(std::string currentFile)
{
	if (currentFile.compare(0, basePlanPath.size(), basePlanPath))
	{
		this->currentFile = currentFile.substr(basePlanPath.size());
	}
	else if (currentFile.compare(0, baseRolePath.size(), baseRolePath))
	{
		this->currentFile = currentFile.substr(baseRolePath.size());
	}
	else
	{
		this->currentFile = currentFile;
	}
}

} /* namespace Alica */
