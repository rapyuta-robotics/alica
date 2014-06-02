/*
 * PlanParser.h
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLANPARSER_H_
#define PLANPARSER_H_

using namespace std;

#include <iostream>
#include <list>
#include <stdio.h>

#include <SystemConfig.h>
#include "../PlanRepository.h"
#include "../IPlanParser.h"

namespace alica
{
	class ModelFactory;
	class PlanParser : public IPlanParser
	{
	public:
		PlanParser(shared_ptr<PlanRepository> rep);
		virtual ~PlanParser();

		virtual shared_ptr<Plan> ParsePlanTree(string masterplan);
		virtual shared_ptr<RoleSet> ParseRoleSet(string roleSetName, string roleSetDir);
		virtual void IgnoreMasterPlanId(bool val);
		virtual shared_ptr<map<long, alica::AlicaElement> > GetParsedElements();

		string getCurrentFile();
		void setCurrentFile(string currentFile);

	private:
		//shared_ptr<supplementary::SystemConfig> sc;
		supplementary::SystemConfig* sc;
		shared_ptr<ModelFactory> mf;
		shared_ptr<PlanRepository> rep;
		Plan masterPlan;
		string planDir;
		string roleDir;

		list<string> filesToParse();
		list<string> filesParsed();

		string basePlanPath;
		string baseRolePath;
		string currentDirectory;
		string esConfigRoot;
		string currentFile;

		string findPmlFile(string path, string plan);

	};
} /* namespace Alica */

#endif /* PLANPARSER_H_ */
