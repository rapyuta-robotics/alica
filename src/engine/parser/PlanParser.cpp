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
		this->esConfigRoot = this->sc->getConfigPath();

		this->planDir = (*this->sc)["Alica"]->get<string>("Alica.PlanDir", NULL);
		this->roleDir = (*this->sc)["Alica"]->get<string>("Alica.RoleDir", NULL);

		if (esConfigRoot.find_last_of("/") != esConfigRoot.length() - 1)
		{
			esConfigRoot = esConfigRoot + "/";
		}
		if (planDir.find_last_of("/") != planDir.length() - 1)
		{
			planDir = planDir + "/";
		}
		if (roleDir.find_last_of("/") != roleDir.length() - 1)
		{
			roleDir = roleDir + "/";
		}
		//Hier gehts bei mir weiter...
	}

	PlanParser::~PlanParser()
	{
		// TODO Auto-generated destructor stub
	}

	shared_ptr<Plan> PlanParser::ParsePlanTree(string masterplan)
	{

		string topFile = supplementary::FileSystem::findFile(this->planDir, masterplan, ".pml");
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
