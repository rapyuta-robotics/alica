/*
 * PlanParser.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */
#include "engine/parser/PlanParser.h"
#include "engine/parser/ModelFactory.h"
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
			basePlanPath = domainConfigFolder + "/" + planDir;
		}
		else
		{
			basePlanPath = planDir;
		}
		if (!(supplementary::FileSystem::isPathRooted(this->roleDir)))
		{
			baseRolePath = domainConfigFolder + "/" + roleDir;
		}
		else
		{
			baseRolePath = roleDir;
		}
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
		string topFile = supplementary::FileSystem::findFile(this->basePlanPath, masterplan, ".pml");
		cout << "PP: topFile: " << topFile << endl;

		if (topFile.compare("") == 0)
		{
//			AlicaEngine.Get().Abort(String.Format("PP: Cannot find Masterplan {0} in {1}",masterplan,this.basePlanPath));
		}
//		this.currentDirectory =  Directory.GetParent(topFile).FullName+Path.DirectorySeparatorChar;
//		this->currentDirectory =
//		cout << "PP: Using Masterplan " << topFile << endl;
		//this.CurrentFile = topFile;
		//this.masterPlan = ParsePlanFile(topFile);
		//ParseFileLoop();
		//this.mf.ComputeReachabilities();
		//return this.masterPlan;

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

	string PlanParser::findPmlFile(string path, string plan)
	{
		string fname = plan + ".pml";
//		foreach(string file in Directory.GetFiles(path,"*.pml"))
//		{
//			if (file.EndsWith(Path.DirectorySeparatorChar+fname))
//			{
//				return file;
//			}
//		}
//		foreach(string subfolder in Directory.GetDirectories(path))
//		{
//			string file = FindPmlFile(subfolder,plan);
//			if (!file.Equals("")) return file;
//		}
		return "";
	}

}
/* namespace Alica */
