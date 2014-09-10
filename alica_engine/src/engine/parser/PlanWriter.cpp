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
#include "engine/model/PostCondition.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/Variable.h"
#include "engine/model/Quantifier.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/SuccessState.h"
#include "engine/model/FailureState.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/Transition.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/PlanType.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/PlanningProblem.h"
#include "engine/model/SyncTransition.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"
#include "engine/model/TaskRepository.h"
#include "engine/model/RoleSet.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaClock.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Role.h"
#include "engine/model/RoleDefinitionSet.h"

namespace alica
{

	int PlanWriter::objectCounter = 0;

	PlanWriter::PlanWriter(PlanRepository* rep)
	{
		string path = supplementary::SystemConfig::getInstance()->getConfigPath();
		this->tempPlanDir = supplementary::FileSystem::combinePaths(path, "plans/tmp/");
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
			xc->SetAttribute("xsi:type", "alica:PreCondition");
			addPlanElementAttributes(p->getPreCondition(), xc);
			xc->SetAttribute("conditionString", p->getPreCondition()->getConditionString().c_str());
			xc->SetAttribute("enabled", to_string(p->getPreCondition()->isEnabled()).c_str());
			addConditionChildren(p->getPreCondition(), xc, doc);
		}
		if (p->getRuntimeCondition() != nullptr)
		{
			tinyxml2::XMLElement* xc = doc->NewElement("conditions");
			xp->InsertEndChild(xc);
			xc->SetAttribute("xsi:type", "alica:RuntimeCondition");
			addPlanElementAttributes(p->getRuntimeCondition(), xc);
			xc->SetAttribute("conditionString", p->getRuntimeCondition()->getConditionString().c_str());
			addConditionChildren(p->getRuntimeCondition(), xc, doc);
		}
		if (p->getPostCondition() != nullptr)
		{
			tinyxml2::XMLElement* xc = doc->NewElement("conditions");
			xp->InsertEndChild(xc);
			xc->SetAttribute("xsi:type", "alica:Result");
			addPlanElementAttributes(p->getPostCondition(), xc);
			xc->SetAttribute("conditionString", p->getPostCondition()->getConditionString().c_str());
			addConditionChildren(p->getPostCondition(), xc, doc);
		}
		for (Variable* v : (*p->getVariables()))
		{
			tinyxml2::XMLElement* xc = doc->NewElement("vars");
			addPlanElementAttributes(v, xc);
			if (!(v->getType().empty()))
			{
				xc->SetAttribute("Type", v->getType().c_str());
			}
			xp->InsertEndChild(xc);
		}
		for (auto e : p->getEntryPoints())
		{
			xp->InsertEndChild(createEntryPointXMLNode(e.second, doc));
		}
		for (State* s : p->getStates())
		{
			xp->InsertEndChild(createStateXMLNode(s, doc));
		}
		for (Transition* t : p->getTransitions())
		{
			xp->InsertEndChild(createTransitionXMLNode(t, doc));
		}
		for (SyncTransition* s : p->getSyncTransitions())
		{
			xp->InsertEndChild(createSynchronisationXMLNode(s, doc));
		}

	}

	tinyxml2::XMLDocument* PlanWriter::createRoleSetXMLDocument(RoleSet* r)
	{
		tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

		tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("version=\"1.0\" encoding=\"ASCII\"");
		doc->InsertEndChild(decl);

		createRoleSet(r, doc);

		return doc;
	}

	void PlanWriter::saveRoleSet(RoleSet* r, string name)
	{
		this->currentFile = supplementary::FileSystem::combinePaths(this->tempPlanDir, name);
		tinyxml2::XMLDocument* doc = createRoleSetXMLDocument(r);

		//Console.WriteLine(p.FileName);
		string dir = supplementary::FileSystem::getParent(this->currentFile);
		if (!supplementary::FileSystem::isDirectory(dir))
		{
			supplementary::FileSystem::createDirectory(dir, 777);
		}
		//Console.WriteLine("Save file to : " + this.currentFile);
		doc->SaveFile(this->currentFile.c_str(), false);
	}

	void PlanWriter::saveRoleSet(RoleSet* r, string directory, string name)
	{
		this->currentFile = supplementary::FileSystem::combinePaths(directory, name);
		tinyxml2::XMLDocument* doc = createRoleSetXMLDocument(r);

		//Console.WriteLine(p.FileName);
		string dir = supplementary::FileSystem::getParent(this->currentFile);
		if (!supplementary::FileSystem::isDirectory(dir))
		{
			supplementary::FileSystem::createDirectory(dir, 777);
		}
		//Console.WriteLine("Save file to : " + this.currentFile);
		doc->SaveFile(this->currentFile.c_str(), false);
	}

	tinyxml2::XMLDocument* PlanWriter::createTaskRepositoryXMLDocument(TaskRepository* tr)
	{
		tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

		tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("version=\"1.0\" encoding=\"ASCII\"");
		doc->InsertEndChild(decl);

		createTaskRepository(tr,doc);

		return doc;
	}

	void PlanWriter::saveTaskRepository(TaskRepository* tr, string name)
	{
		this->currentFile = supplementary::FileSystem::combinePaths(this->tempPlanDir, name);
		tinyxml2::XMLDocument* doc = createTaskRepositoryXMLDocument(tr);

		//Console.WriteLine(p.FileName);
		string dir = supplementary::FileSystem::getParent(this->currentFile);
		if (!supplementary::FileSystem::isDirectory(dir))
		{
			supplementary::FileSystem::createDirectory(dir, 777);
		}
		//Console.WriteLine("Save file to : " + this.currentFile);
		doc->SaveFile(this->currentFile.c_str(), false);
	}

	void PlanWriter::saveTaskRepository(TaskRepository* tr, string directory, string name)
	{
		this->currentFile = supplementary::FileSystem::combinePaths(directory, name);
		tinyxml2::XMLDocument* doc = createTaskRepositoryXMLDocument(tr);

		//Console.WriteLine(p.FileName);
		string dir = supplementary::FileSystem::getParent(this->currentFile);
		if (!supplementary::FileSystem::isDirectory(dir))
		{
			supplementary::FileSystem::createDirectory(dir, 777);
		}
		//Console.WriteLine("Save file to : " + this.currentFile);
		doc->SaveFile(this->currentFile.c_str(), false);
	}

	void PlanWriter::setPlansToSave(vector<AlicaElement*>& plansToSave)
	{
		this->plansToSave = plansToSave;
	}

	void PlanWriter::addConditionChildren(Condition* c, tinyxml2::XMLElement* xn, tinyxml2::XMLDocument* doc)
	{
		for (Variable* v : c->getVariables())
		{
			tinyxml2::XMLElement* xc = doc->NewElement("vars");
			xc->InsertEndChild(doc->NewText("#" + v->getId()));
			xn->InsertEndChild(xc);
		}
		for (Quantifier* q : c->getQuantifiers())
		{
			tinyxml2::XMLElement* xc = doc->NewElement("quantifiers");
			addPlanElementAttributes(q, xc);
			if (dynamic_cast<ForallAgents*>(q) != nullptr)
			{
				xc->SetAttribute("xsi:type", "alica:ForallAgents");
			}
			else
			{
				cout << "Unknown Quantifier: " << q->toString() << endl;
				throw new exception();
			}
			xc->SetAttribute("scope", to_string(q->getScope()->getId()).c_str());
			for (string sort : q->getDomainIdentifiers())
			{
				tinyxml2::XMLElement* xcc = doc->NewElement("sorts");
				xc->InsertEndChild(xcc);
				xcc->InsertEndChild(doc->NewText(sort.c_str()));
			}
			xn->InsertEndChild(xc);
		}
	}

	tinyxml2::XMLElement* PlanWriter::createStateXMLNode(State* s, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xs = doc->NewElement("states");
		if (dynamic_cast<SuccessState*>(s) != nullptr)
		{
			xs->SetAttribute("xsi:type", "alica:SuccessState");
			SuccessState* su = dynamic_cast<SuccessState*>(s);
			if (su->getPostCondition() != nullptr)
			{
				xs->InsertEndChild(createResultXMLNode(su->getPostCondition(), doc));
			}

		}
		else if (dynamic_cast<FailureState*>(s) != nullptr)
		{
			xs->SetAttribute("xsi:type", "alica:FailureState");
			FailureState* fu = dynamic_cast<FailureState*>(s);
			if (fu->getPostCondition() != nullptr)
			{
				xs->InsertEndChild(createResultXMLNode(fu->getPostCondition(), doc));
			}
		}
		else
		{

		}
		addPlanElementAttributes(s, xs);

		for (Parametrisation* p : s->getParametrisation())
		{
			xs->InsertEndChild(createParametrisationXMLNode(p, doc));
		}

		for (Transition* t : s->getInTransitions())
		{
			tinyxml2::XMLElement* xc = doc->NewElement("inTransitions");
			xs->InsertEndChild(xc);
			xc->InsertEndChild(doc->NewText("#" + t->getId()));
		}

		for (Transition* t : s->getOutTransitions())
		{
			tinyxml2::XMLElement* xc = doc->NewElement("outTransitions");
			xs->InsertEndChild(xc);
			xc->InsertEndChild(doc->NewText("#" + t->getId()));
		}
		for (AbstractPlan* p : s->getPlans())
		{
			tinyxml2::XMLElement* xc = doc->NewElement("plans");
			xs->InsertEndChild(xc);
			if (dynamic_cast<Plan*>(p) != nullptr)
			{
				xc->SetAttribute("xsi:type", "alica:Plan");
			}
			else if (dynamic_cast<PlanType*>(p) != nullptr)
			{
				xc->SetAttribute("xsi:type", "alica:PlanType");
			}
			else if (dynamic_cast<BehaviourConfiguration*>(p) != nullptr)
			{
				xc->SetAttribute("xsi:type", "alica:BehaviourConfiguration");
			}
			else if (dynamic_cast<PlanningProblem*>(p) != nullptr)
			{
				xc->SetAttribute("xsi:type", "alica:PlanningProblem");
			}
			xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
		}

		return xs;
	}

	tinyxml2::XMLElement* PlanWriter::createParametrisationXMLNode(Parametrisation* p, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xr = doc->NewElement("parametrisation");
		addPlanElementAttributes(p, xr);
		tinyxml2::XMLElement* xc = doc->NewElement("subplan");
		if (dynamic_cast<Plan*>(p->getSubPlan()) != nullptr)
		{
			xc->SetAttribute("xsi:type", "alica:Plan");
		}
		else if (dynamic_cast<PlanType*>(p->getSubPlan()) != nullptr)
		{
			xc->SetAttribute("xsi:type", "alica:PlanType");
		}
		else if (dynamic_cast<BehaviourConfiguration*>(p->getSubPlan()) != nullptr)
		{
			xc->SetAttribute("xsi:type", "alica:BehaviourConfiguration");
		}
		xc->InsertEndChild(
				doc->NewText(
						(getRelativeFileName(p->getSubPlan()) + "#" + to_string(p->getSubPlan()->getId())).c_str()));
		xr->InsertEndChild(xc);

		xc = doc->NewElement("subvar");
		xc->InsertEndChild(
				doc->NewText(
						(getRelativeFileName(p->getSubPlan()) + "#" + to_string(p->getSubVar()->getId())).c_str()));
		xr->InsertEndChild(xc);

		xc = doc->NewElement("var");
		xc->InsertEndChild(doc->NewText("#" + p->getVar()->getId()));
		xr->InsertEndChild(xc);
		return xr;
	}

	tinyxml2::XMLElement* PlanWriter::createResultXMLNode(PostCondition* r, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xr = doc->NewElement("result");
		addPlanElementAttributes(r, xr);
		xr->SetAttribute("conditionString", r->getConditionString().c_str());
		addConditionChildren(r, xr, doc);
		return xr;
	}

	tinyxml2::XMLElement* PlanWriter::createPreConditionXMLNode(PreCondition* c, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xr = doc->NewElement("preCondition");
		addPlanElementAttributes(c, xr);
		xr->SetAttribute("conditionString", c->getConditionString().c_str());
		xr->SetAttribute("enabled", to_string(c->isEnabled()).c_str());
		addConditionChildren(c, xr, doc);
		return xr;
	}

	tinyxml2::XMLElement* PlanWriter::createSynchronisationXMLNode(SyncTransition* s, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xr = doc->NewElement("synchronisations");
		addPlanElementAttributes(s, xr);
		xr->SetAttribute("talkTimeout", to_string(s->getTalkTimeOut()).c_str());
		xr->SetAttribute("syncTimeout", to_string(s->getSyncTimeOut()).c_str());
		xr->SetAttribute("failOnSyncTimeOut", "false");
		string synched = "";
		for (Transition* t : s->getInSync())
		{
			synched += t->getId() + " ";
		}
		xr->SetAttribute("synchedTransitions", supplementary::Configuration::trim(synched).c_str());
		return xr;
	}

	tinyxml2::XMLElement* PlanWriter::createTransitionXMLNode(Transition* t, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xt = doc->NewElement("transitions");
		addPlanElementAttributes(t, xt);
		xt->SetAttribute("msg", "");
		if (t->getPreCondition() != nullptr)
		{
			xt->InsertEndChild(createPreConditionXMLNode(t->getPreCondition(), doc));
		}
		tinyxml2::XMLElement* xc = doc->NewElement("inState");
		xt->InsertEndChild(xc);
		xc->InsertEndChild(doc->NewText("#" + t->getInState()->getId()));
		xc = doc->NewElement("outState");
		xt->InsertEndChild(xc);
		xc->InsertEndChild(doc->NewText("#" + t->getOutState()->getId()));
		if (t->getSyncTransition() != nullptr)
		{
			xc = doc->NewElement("synchronisation");
			xt->InsertEndChild(xc);
			xc->InsertEndChild(doc->NewText("#" + t->getSyncTransition()->getId()));
		}
		return xt;
	}

	tinyxml2::XMLElement* PlanWriter::createEntryPointXMLNode(EntryPoint* e, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xe = doc->NewElement("entryPoints");
		addPlanElementAttributes(e, xe);
		xe->SetAttribute("minCardinality", e->getMinCardinality());
		xe->SetAttribute("maxCardinality", e->getMaxCardinality());
		xe->SetAttribute("successRequired", e->isSuccessRequired());
		tinyxml2::XMLElement* xc = doc->NewElement("state");
		xe->InsertEndChild(xc);
		xc->InsertEndChild(doc->NewText("#" + e->getState()->getId()));
		xc = doc->NewElement("task");
		xe->InsertEndChild(xc);
		xc->InsertEndChild(
				doc->NewText(
						(getRelativeFileName(
								this->rep->getTaskRepositorys()[e->getTask()->getTaskRepository()->getId()]->getFileName())
								+ "#" + to_string(e->getTask()->getId())).c_str()));
		return xe;
	}

	void PlanWriter::addPlanElementAttributes(AlicaElement* p, tinyxml2::XMLElement* x)
	{
		x->SetAttribute("id", to_string(p->getId()).c_str());
		x->SetAttribute("name", p->getName().c_str());
		x->SetAttribute("comment", p->getComment().c_str());
	}

	//TODO perhaps not needed
//	tinyxml2::XMLAttribute* PlanWriter::getXsiTypeAttribute(string type, tinyxml2::XMLDocument* doc)
//	{
//		tinyxml2::XMLAttribute* xsiType = doc->CreateAttribute("xsi", "type",XmlSchema.InstanceNamespace);
//		xsiType.Value = type;
//		return xsiType;
//	}

	string PlanWriter::getRelativeFileName(string file)
	{
		string curdir = this->currentFile;
		string ufile = "";
		if (supplementary::FileSystem::isPathRooted(file))
		{
			ufile = file;
		}
		else
		{
			if (file.substr(file.size() - 4, 4) == ".beh" || file.substr(file.size() - 4, 4) == ".pty"
					|| file.substr(file.size() - 4, 4) == ".pml" || file.substr(file.size() - 3, 3) == ".pp")
			{
				supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
				string tfile = (*sc)["Alica"]->get<string>("Alica.PlanDir");
				supplementary::FileSystem::combinePaths(tfile, file);
				if (!supplementary::FileSystem::isPathRooted(tfile))
				{
					tfile = supplementary::FileSystem::combinePaths(this->configPath, tfile);
				}
				ufile = tfile;
			}
			else
			{
				cout << "File reference not implemented: " << file << "(occurred in file " << this->currentFile << ")"
						<< endl;
				throw new exception();
			}

		}
		//TODO maybe think about it
		string ret = "";
		int pos = 0;
		if (ufile.size() < curdir.size())
		{
			for (int i = 0; i < ufile.size(); i++)
			{
				if (curdir.at(i) != ufile.at(i))
				{
					break;
				}
				pos++;
			}
			curdir.erase(curdir.begin(), curdir.begin() + pos);
			ret = curdir;
		}
		else
		{
			for (int i = 0; i < curdir.size(); i++)
			{
				if (curdir.at(i) != ufile.at(i))
				{
					break;
				}
				pos++;
			}
			ufile.erase(ufile.begin(), ufile.begin() + pos);
			ret = ufile;
		}
		return ret;
	}

	string PlanWriter::getRelativeFileName(AbstractPlan* p)
	{
		if (find(this->plansToSave.begin(), this->plansToSave.end(), p) != this->plansToSave.end()
				|| find(this->plansSaved.begin(), this->plansSaved.end(), p) != this->plansSaved.end())
		{
			string dirfile = this->tempPlanDir;
			//TODO should have been done by fsystem
//			if (!dirfile.EndsWith("" + Path.DirectorySeparatorChar))
//			{
//				dirfile += Path.DirectorySeparatorChar;
//			}
			dirfile += p->getFileName();
			return getRelativeFileName(dirfile);
		}
		else
		{
			return getRelativeFileName(p->getFileName());
		}
	}

	void PlanWriter::createRoleSet(RoleSet* r, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xp = doc->NewElement("alica:RoleSet");
		xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
		doc->InsertEndChild(xp);
//		XmlAttribute xsiType = doc.CreateAttribute("xmi", "version", XmlSchema.InstanceNamespace);
//		xsiType.Value = "2.0";
		xp->SetAttribute("xmi:version", "2.0");
		xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
		xp->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
//		xp->SetAttributeNode(xsiType);
		xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
		xp->SetAttribute("id",
							to_string(AlicaEngine::getInstance()->getIAlicaClock()->now() + objectCounter++).c_str());
		xp->SetAttribute("name", r->getName().c_str());
		xp->SetAttribute("comment", r->getComment().c_str());
		xp->SetAttribute("usableWithPlanID", to_string(r->getUsableWithPlanId()).c_str());
		xp->SetAttribute("default", to_string(r->isIsDefault()).c_str());

		for (RoleTaskMapping* rtm : r->getRoleTaskMappings())
		{
			tinyxml2::XMLElement* xc = doc->NewElement("mappings");
			xp->InsertEndChild(xc);
			xc->SetAttribute(
					"id", to_string(AlicaEngine::getInstance()->getIAlicaClock()->now() + objectCounter++).c_str());
			xc->SetAttribute("name", rtm->getName().c_str());
			xc->SetAttribute("comment", rtm->getComment().c_str());
			for (auto mapping : rtm->getTaskPriorities())
			{
				tinyxml2::XMLElement* xd = doc->NewElement("taskPriorities");
				xc->InsertEndChild(xd);
				xd->SetAttribute(
						"id", to_string(AlicaEngine::getInstance()->getIAlicaClock()->now() + objectCounter++).c_str());
				xd->SetAttribute("name", "");
				xd->SetAttribute("comment", "");
				xd->SetAttribute("key", to_string(mapping.first).c_str());
				xd->SetAttribute("value", to_string(mapping.second).c_str());
			}
			tinyxml2::XMLElement* xe = doc->NewElement("role");
			xc->InsertEndChild(xe);
			xe->InsertEndChild(
					doc->NewText(
							(rtm->getRole()->getRoleDefinitionSet()->getFileName() + "#"
									+ to_string(rtm->getRole()->getId())).c_str()));
		}
	}

	void PlanWriter::createTaskRepository(TaskRepository* tr, tinyxml2::XMLDocument* doc)
	{
		tinyxml2::XMLElement* xp = doc->NewElement("alica:TaskRepository");
		xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
		doc->InsertEndChild(xp);
//		XmlAttribute xsiType = doc.CreateAttribute("xmi", "version",XmlSchema.InstanceNamespace);
//		xsiType.Value = "2.0";
//		xp.SetAttributeNode(xsiType);
		xp->SetAttribute("xmi:version", "2.0");
		xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
		xp->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
		xp->SetAttribute("xmlns:xmi","http://www.omg.org/XMI");
		xp->SetAttribute("id",to_string(AlicaEngine::getInstance()->getIAlicaClock()->now() + objectCounter++).c_str());
		xp->SetAttribute("name",tr->getName().c_str());
		xp->SetAttribute("comment",tr->getComment().c_str());
		xp->SetAttribute("defaultTask",to_string(tr->getDefaultTask()).c_str());

		for(Task* t : tr->getTasks())
		{
			tinyxml2::XMLElement* xc = doc->NewElement("tasks");
			xp->InsertEndChild(xc);
			xc->SetAttribute("id",to_string(t->getId()).c_str());
			xc->SetAttribute("name",t->getName().c_str());
			xc->SetAttribute("comment",t->getComment().c_str());
			xc->SetAttribute("description",t->getDescription().c_str());
		}
	}

} /* namespace alica */
