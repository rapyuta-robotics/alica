/*
 * PlanWriter.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: Stefan Jakob
 */

#include <engine/parser/PlanWriter.h>
#include <SystemConfig.h>
#include <FileSystem.h>
#include "engine/PlanRepository.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/Plan.h"
#include "engine/parser/tinyxml2.h"
#include "engine/model/PreCondition.h"

namespace alica
{

	PlanWriter::PlanWriter(PlanRepository* rep)
	{
		string path = supplementary::SystemConfig::getInstance()->getConfigPath();
		this->tempPlanDir = supplementary::FileSystem::combinePaths(path, "plans/tmp");
		this->rep = rep;
		this->plansToSave = vector<AlicaElement*>();
		this->plansSaved = vector<AlicaElement*>();
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->configPath = sc->getConfigPath();
	}

	PlanWriter::~PlanWriter()
	{

	}

	string PlanWriter::getTempPlanDir()
	{
		return tempPlanDir;
	}

	void PlanWriter::setTempPlanDir(string directory)
	{
		this->tempPlanDir = directory;
	}

	string PlanWriter::getConfigPath()
	{
		return configPath;
	}

	vector<AlicaElement*>& PlanWriter::getPlansToSave()
	{
		return plansToSave;
	}

	void PlanWriter::saveAllPlans()
	{
		this->plansToSave.clear();
		for (auto pair : this->rep->getPlans())
		{
			this->plansToSave.push_back(pair.second);
		}
		saveFileLoop();
	}

	void PlanWriter::saveSinglePlan(Plan* p)
	{
		this->currentFile = supplementary::FileSystem::combinePaths(this->tempPlanDir, p->getFileName());
		tinyxml2::XMLDocument* doc = createPlanXMLDocument(p);

		//Console.WriteLine(p.FileName);
		string dir = supplementary::FileSystem::getParent(this->currentFile);
		if (!supplementary::FileSystem::isDirectory(dir))
		{
			supplementary::FileSystem::createDirectory(dir, 777);
		}
		//Console.WriteLine("Save file to : " + this.currentFile);
		doc->SaveFile(this->currentFile.c_str(), false);
	}

	void PlanWriter::saveSinglePlan(string directory, Plan* p)
	{
		this->currentFile = supplementary::FileSystem::combinePaths(directory, p->getFileName());
		tinyxml2::XMLDocument* doc = createPlanXMLDocument(p);

		//Console.WriteLine(p.FileName);
		string dir = supplementary::FileSystem::getParent(this->currentFile);
		if (!supplementary::FileSystem::isDirectory(dir))
		{
			supplementary::FileSystem::createDirectory(dir, 777);
		}
		//Console.WriteLine("Save file to : " + this.currentFile);
		doc->SaveFile(this->currentFile.c_str(), false);
	}

	void PlanWriter::saveFileLoop()
	{
		while (plansToSave.size() > 0)
		{
			AlicaElement* ae = plansToSave[plansToSave.size() - 1];
			plansToSave.erase(plansToSave.begin() + (plansToSave.size() - 1));
			if (dynamic_cast<Plan*>(ae) != nullptr)
			{
				saveSinglePlan(dynamic_cast<Plan*>(ae));
			}
			else
			{
				cout << "Saving of type " << typeid(ae).name() << " is not implemented." << endl;
				throw new exception();
			}
			plansSaved.push_back(ae);
		}
		this->plansToSave.clear();
		this->plansSaved.clear();
	}

	tinyxml2::XMLDocument* PlanWriter::createPlanXMLDocument(Plan* p)
	{
		tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

		tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("version=\"1.0\" encoding=\"ASCII\"");
		doc->InsertEndChild(decl);

		createPlanXMLNode(p, doc);

		return doc;
	}

	void PlanWriter::createPlanXMLNode(Plan* p, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xp = doc->NewElement("alica:Plan");
		xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
		doc->InsertEndChild(xp);
		//TODO perhaps it works in tinyxml2
//		tinyxml2::XMLAttribute* xsiType = doc->CreateAttribute("xmi", "version", XmlSchema.InstanceNamespace);
//		xsiType->Value = "2.0";
//		xp->SetAttributeNode(xsiType);
		xp->SetAttribute("xmi:version", "2.0");
		xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
		xp->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");

		addPlanElementAttributes(p, xp);

		xp->SetAttribute("masterPlan", to_string(p->isMasterPlan()).c_str());
		xp->SetAttribute("utilityFunction", "");
		xp->SetAttribute("utilityThreshold", to_string(p->getUtilityThreshold()).c_str());
		xp->SetAttribute("minCardinality", to_string(p->getMinCardinality()).c_str());
		xp->SetAttribute("maxCardinality", to_string(p->getMaxCardinality()).c_str());

		if (p->getPreCondition() != nullptr)
		{
			tinyxml2::XMLElement* xc = doc->NewElement("conditions");
			xp->InsertEndChild(xc);
			xc->SetAttribute("xsi:type","alica:PreCondition");
			addPlanElementAttributes(p->getPreCondition(), xc);
			xc->SetAttribute("conditionString", p->getPreCondition()->getConditionString().c_str());
			xc->SetAttribute("enabled", to_string(p->getPreCondition()->isEnabled()).c_str());
			addConditionChildren(p->getPreCondition(), xc, doc);
		}
//		if (p.RuntimeCondition != null)
//		{
//			XmlElement xc = doc.CreateElement("conditions");
//			xp.AppendChild(xc);
//			xc.SetAttributeNode(GetXsiTypeAttribute("alica:RuntimeCondition", doc));
//			AddPlanElementAttributes(p.RuntimeCondition, xc);
//			xc.SetAttribute("conditionString", p.RuntimeCondition.ConditionString);
//			AddConditionChildren(p.RuntimeCondition, xc, doc);
//		}
//		if (p.PostCondition != null)
//		{
//			XmlElement xc = doc.CreateElement("conditions");
//			xp.AppendChild(xc);
//			xc.SetAttributeNode(GetXsiTypeAttribute("alica:Result", doc));
//			AddPlanElementAttributes(p.PostCondition, xc);
//			xc.SetAttribute("conditionString", p.PostCondition.ConditionString);
//			AddConditionChildren(p.PostCondition, xc, doc);
//		}
//	foreach(Variable v in p.Variables)
//	{
//		XmlElement xc = doc.CreateElement("vars");
//		AddPlanElementAttributes(v,xc);
//		if(!String.IsNullOrEmpty(v.Type))
//		{
//			xc.SetAttribute("Type",v.Type);
//		}
//		xp.AppendChild(xc);
//	}
//	foreach(EntryPoint e in p.EntryPoints.Values)
//	{
//		xp.AppendChild(CreateEntryPointXMLNode(e,doc));
//	}
//	foreach(State s in p.States)
//	{
//		xp.AppendChild(CreateStateXMLNode(s,doc));
//	}
//	foreach(Transition t in p.Transitions)
//	{
//		xp.AppendChild(CreateTransitionXMLNode(t,doc));
//	}
//	foreach(SyncTransition s in p.SyncTransitions)
//	{
//		xp.AppendChild(CreateSynchronisationXMLNode(s,doc));
//	}

	}

	tinyxml2::XMLDocument* PlanWriter::createRoleSetXMLDocument(RoleSet* r)
	{
	}

	void PlanWriter::saveRoleSet(RoleSet* r, string name)
	{
	}

	void PlanWriter::saveRoleSet(RoleSet* r, string directory, string name)
	{
	}

	tinyxml2::XMLDocument* PlanWriter::createTaskRepositoryXMLDocument(TaskRepository* tr)
	{
	}

	void PlanWriter::saveTaskRepository(TaskRepository* tr, string name)
	{
	}

	void PlanWriter::saveTaskRepository(TaskRepository* tr, string directory, string name)
	{
	}

	void PlanWriter::setPlansToSave(vector<AlicaElement*>& plansToSave)
	{
		this->plansToSave = plansToSave;
	}

	void PlanWriter::addConditionChildren(Condition* c, tinyxml2::XMLElement* xn, tinyxml2::XMLDocument* doc)
	{
	}

	tinyxml2::XMLElement* PlanWriter::createStateXMLNode(State* s, tinyxml2::XMLDocument* doc)
	{
	}

	tinyxml2::XMLElement* PlanWriter::createParametrisationXMLNode(Parametrisation* p, tinyxml2::XMLDocument* doc)
	{
	}

	tinyxml2::XMLElement* PlanWriter::createResultXMLNode(PostCondition* r, tinyxml2::XMLDocument* doc)
	{
	}

	tinyxml2::XMLElement* PlanWriter::createPreConditionXMLNode(PreCondition* c, tinyxml2::XMLDocument* doc)
	{
	}

	tinyxml2::XMLElement* PlanWriter::createSynchronisationXMLNode(SyncTransition* s, tinyxml2::XMLDocument* doc)
	{
	}

	tinyxml2::XMLElement* PlanWriter::createTransitionXMLNode(Transition* t, tinyxml2::XMLDocument* doc)
	{
	}

	tinyxml2::XMLElement* PlanWriter::createEntryPointXMLNode(EntryPoint* e, tinyxml2::XMLDocument* doc)
	{
	}

	void PlanWriter::addPlanElementAttributes(AlicaElement* p, tinyxml2::XMLElement* x)
	{
	}

	tinyxml2::XMLAttribute* PlanWriter::getXsiTypeAttribute(string type, tinyxml2::XMLDocument* doc)
	{
	}

	string PlanWriter::getRelativeFileName(string file)
	{
	}

	string PlanWriter::getRelativeFileName(AbstractPlan* p)
	{
	}

	void PlanWriter::createRoleSet(RoleSet* r, tinyxml2::XMLDocument* doc)
	{
	}

	void PlanWriter::createTaskRepository(TaskRepository* tr, tinyxml2::XMLDocument* doc)
	{
	}

} /* namespace alica */
