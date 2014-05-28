/*
 * PlanParser.h
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLANPARSER_H_
#define PLANPARSER_H_

#include "SystemConfig.h";
#include "ModelFactory.h";
#include ""

namespace alica
{

class PlanParser : public IPlanParser
{
public:
	PlanParser();
	virtual ~PlanParser();

	virtual alica::Plan ParsePlanTree(std::string masterplan);
	virtual alica::RoleSet ParseRoleSet(std::string roleSetName, std::string roleSetDir);
	virtual void IgnoreMasterPlanId(bool val);
	virtual std::map<long, alica::AlicaElement> GetParsedElements();

private:
	SystemConfig sc;
	ModelFactory mf;
	PlanRepository rep;


};

} /* namespace Alica */

#endif /* PLANPARSER_H_ */
