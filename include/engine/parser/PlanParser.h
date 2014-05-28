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
#include "PlanRepository.h"

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
	supplementary::SystemConfig sc;
	ModelFactory mf;
	PlanRepository rep;

	std::list<std::string> filesToParse;
	std::list<std::string> filesParsed;

	std::string basePlanPath;
	std::string baseRolePath;
	std::string currentDirectory;
	std::string esConfigRoot;
	std::string currentFile;


};

} /* namespace Alica */

#endif /* PLANPARSER_H_ */
