/*
 * PlanParser.h
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLANPARSER_H_
#define PLANPARSER_H_

class ModelFactory;

#include <SystemConfig.h>
#include "../PlanRepository.h"
#include "../IPlanParser.h"
#include <list>
#include "ModelFactory.h"

namespace alica
{
class PlanParser : public IPlanParser
{
public:
	PlanParser(std::shared_ptr<PlanRepository> rep);
	virtual ~PlanParser();

	virtual std::shared_ptr<Plan> ParsePlanTree(std::string masterplan);
	virtual std::shared_ptr<RoleSet> ParseRoleSet(std::string roleSetName, std::string roleSetDir);
	virtual void IgnoreMasterPlanId(bool val);
	virtual std::shared_ptr<std::map<long, alica::AlicaElement> > GetParsedElements();

	std::string getCurrentFile();
	void setCurrentFile(std::string currentFile);

private:
	//std::shared_ptr<supplementary::SystemConfig> sc;
	supplementary::SystemConfig* sc;
	std::shared_ptr<ModelFactory> mf;
	std::shared_ptr<PlanRepository> rep;
	Plan masterPlan;


	std::list<std::string> filesToParse();
	std::list<std::string> filesParsed();

	std::string basePlanPath;
	std::string baseRolePath;
	std::string currentDirectory;
	std::string esConfigRoot;
	std::string currentFile;


};

} /* namespace Alica */

#endif /* PLANPARSER_H_ */
