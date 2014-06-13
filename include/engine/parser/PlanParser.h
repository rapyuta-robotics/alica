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
#include <exception>
#include <algorithm>
#include <string>

#include <SystemConfig.h>
#include <FileSystem.h>
#include "../IPlanParser.h"
#include "../model/RoleSet.h"

namespace tinyxml2 {
	class XMLElement;
}

namespace alica
{

	class ModelFactory;
	class PlanRepository;
	class Plan;
	class RoleSet;
	class AlicaElement;

	class PlanParser : public IPlanParser
	{
	public:
		PlanParser(shared_ptr<PlanRepository> rep);
		virtual ~PlanParser();

		virtual Plan* parsePlanTree(string masterplan);
		virtual void ignoreMasterPlanId(bool val);
		virtual shared_ptr<map<long, AlicaElement> > getParsedElements();

		string getCurrentFile();
		void setCurrentFile(string currentFile);
		void parseFileLoop();
		RoleSet* parseRoleSet(string roleSetName, string roleSetDir);
		long parserId(tinyxml2::XMLElement* node);

	private:
		supplementary::SystemConfig* sc;
		shared_ptr<ModelFactory> mf;
		shared_ptr<PlanRepository> rep;
		Plan* masterPlan;
		string planDir;
		string roleDir;
		string basePlanPath;
		string baseRolePath;
		string currentDirectory;
		string domainConfigFolder;
		string currentFile;
		void parseTaskFile(string currentFile);
		void parseRoleDefFile(string currentFile);
		void parseCapabilityDefFile(string currentFile);
		void parsePlanTypeFile(string currentFile);
		void parseBehaviourFile(string currentFile);
		Plan* parsePlanFile(string& planFile);
		long fetchId(const string& idString, long id);
		string findDefaultRoleSet(string dir);

		list<string> filesToParse;
		list<string> filesParsed;

	};
} /* namespace Alica */

#endif /* PLANPARSER_H_ */
