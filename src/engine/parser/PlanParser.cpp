/*
 * PlanParser.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */
using namespace std;
#include "engine/parser/PlanParser.h"

namespace alica
{

	PlanParser::PlanParser(shared_ptr<PlanRepository> rep)
	{
		this->rep = rep;
		std::shared_ptr<PlanParser> ppp = shared_from_this();
		cout << "HAÖÖÖP " << ppp << endl;
		this->mf = std::shared_ptr<ModelFactory>(new ModelFactory(ppp, this->rep));

		this->sc = supplementary::SystemConfig::getInstance();
		this->esConfigRoot = this->sc->getConfigPath();

		string planDir = (*this->sc)["Alica"]->get<string>("Alica.PlanDir", NULL);
		string roleDir = (*this->sc)["Alica"]->get<string>("Alica.RoleDir", NULL);

	}

	PlanParser::~PlanParser()
	{
		// TODO Auto-generated destructor stub
	}

	shared_ptr<Plan> PlanParser::ParsePlanTree(string masterplan)
	{
		string topFile = PlanParser::findPmlFile("basePlanPath", masterplan);

		//if (topFile.Equals("")) {
//			AlicaEngine.Get().Abort(String.Format("PP: Cannot find Masterplan {0} in {1}",masterplan,this.basePlanPath));
		//}
		//this.currentDirectory =  Directory.GetParent(topFile).FullName+Path.DirectorySeparatorChar;
		cout << "PP: Using Masterplan " << topFile << endl;
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
