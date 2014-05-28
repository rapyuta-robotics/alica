/*
 * PlanParser.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#include "PlanParser.h"

namespace alica
{

PlanParser::PlanParser()
{
	// TODO Auto-generated constructor stub

}

PlanParser::~PlanParser()
{
	// TODO Auto-generated destructor stub
}

virtual alica::Plan PlanParser::ParsePlanTree(std::string masterplan)
{

	return 0;
}
virtual alica::RoleSet PlanParser::ParseRoleSet(std::string roleSetName, std::string roleSetDir)
{

	return 0;
}
virtual std::map<long, alica::AlicaElement> PlanParser::GetParsedElements()
{

	return 0;
}
virtual void PlanParser::IgnoreMasterPlanId(bool val)
{

}

} /* namespace Alica */
