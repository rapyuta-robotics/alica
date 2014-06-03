/*
 * PlanParser.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */
#include "engine/parser/PlanParser.h"
#include "engine/parser/ModelFactory.h"
#define PP_DEBUG
namespace alica
{

	PlanParser::PlanParser(shared_ptr<PlanRepository> rep)
	{
		this->rep = rep;
		this->mf = shared_ptr<ModelFactory>(new ModelFactory(this, rep));
		this->sc = supplementary::SystemConfig::getInstance();
		this->domainConfigFolder = this->sc->getConfigPath();

		this->planDir = (*this->sc)["Alica"]->get<string>("Alica.PlanDir", NULL);
		this->roleDir = (*this->sc)["Alica"]->get<string>("Alica.RoleDir", NULL);

		if (domainConfigFolder.find_last_of("/") != domainConfigFolder.length() - 1)
		{
			domainConfigFolder = domainConfigFolder + "/";
		}
		if (planDir.find_last_of("/") != planDir.length() - 1)
		{
			planDir = planDir + "/";
		}
		if (roleDir.find_last_of("/") != roleDir.length() - 1)
		{
			roleDir = roleDir + "/";
		}
		if (!(supplementary::FileSystem::isPathRooted(this->planDir)))
		{
			basePlanPath = domainConfigFolder + planDir;
		}
		else
		{
			basePlanPath = planDir;
		}
		if (!(supplementary::FileSystem::isPathRooted(this->roleDir)))
		{
			baseRolePath = domainConfigFolder + roleDir;
		}
		else
		{
			baseRolePath = roleDir;
		}
		cout << "PP: basePlanPath: " << basePlanPath << endl;
		cout << "PP: baseRolePath: " << baseRolePath << endl;
		if (!(supplementary::FileSystem::fileExists(basePlanPath)))
		{
			//TODO: Abort method (c#) for the AlicaEngine
			throw new exception();
		}
		if (!(supplementary::FileSystem::fileExists(baseRolePath)))
		{
			//TODO: Abort method (c#) for the AlicaEngine
			throw new exception();
		}
	}

	PlanParser::~PlanParser()
	{
		// TODO Auto-generated destructor stub
	}

	shared_ptr<Plan> PlanParser::ParsePlanTree(string masterplan)
	{
		string masterPlanPath;
		bool found = supplementary::FileSystem::findFile(this->basePlanPath, masterplan + ".pml", masterPlanPath);
		cout << "PP: masterPlanPath: " << masterPlanPath << endl;

		if (!found)
		{
			cerr << "PP: Cannot find MasterPlan '" << masterplan << "' in '" << this->basePlanPath << "'" << endl;
			throw new exception();
		}

		this->currentFile = masterPlanPath;
		this->currentDirectory = supplementary::FileSystem::getParent(masterPlanPath);

		cout << "PP: CurFile: " << this->currentFile << " CurDir: " << this->currentDirectory << endl;

		this->masterPlan = parsePlanFile(masterPlanPath);
		//ParseFileLoop();
		//this.mf.ComputeReachabilities();
		//return this.masterPlan;
	}

	Plan PlanParser::parsePlanFile(string& planFile)
	{
#ifdef PP_DEBUG
		cout << "PP: parsing Plan file: " << planFile << endl;
#endif

		tinyxml2::XMLDocument doc;
		doc.LoadFile(planFile.c_str());
		if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
			cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
			throw new exception ();
		}

		// Structure of the XML file:
		// - Element "PLAY"      the root Element, which is the
		//                       FirstChildElement of the Document
		// - - Element "TITLE"   child of the root PLAY Element
		// - - - Text            child of the TITLE Element

		// Navigate to the title, using the convenience function,
		// with a dangerous lack of error checking.
		const char* title =
				doc.FirstChildElement("alica:Plan")->FirstChildElement("states")->FirstChildElement("plans")->GetText();
		cout << "Text (1): " << title << endl;

		// Text is just another Node to TinyXML-2. The more
		// general way to get to the XMLText:
		tinyxml2::XMLText* textNode =
				doc.FirstChildElement("alica:Plan")->FirstChildElement("states")->FirstChildElement("plans")->FirstChild()->ToText();
		title = textNode->Value();
		cout << "Text (2): " << title << endl;

		return Plan();
	}

	shared_ptr<RoleSet> PlanParser::ParseRoleSet(string roleSetName, string roleSetDir)
	{

		shared_ptr<RoleSet> r;
		return r;
	}

	shared_ptr<map<long, alica::AlicaElement> > PlanParser::GetParsedElements()
	{

		shared_ptr<map<long, alica::AlicaElement> > map;
		return map;
	}

	void PlanParser::IgnoreMasterPlanId(bool val)
	{
		this->mf->setIgnoreMasterPlanId(val);
	}

	string PlanParser::getCurrentFile()
	{
		return this->currentFile;
	}

	void PlanParser::setCurrentFile(string currentFile)
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
}
/* namespace Alica */
