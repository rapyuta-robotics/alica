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
#include "engine/AlicaClock.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Role.h"
#include "engine/model/RoleDefinitionSet.h"
#include "engine/model/Parameter.h"

namespace alica {

int PlanWriter::objectCounter = 0;

PlanWriter::PlanWriter(AlicaEngine* ae, PlanRepository* rep) {
    this->ae = ae;
    string path = supplementary::SystemConfig::getInstance()->getConfigPath();
    this->tempPlanDir = supplementary::FileSystem::combinePaths(path, "plans/tmp/");
    this->rep = rep;
    this->plansToSave = vector<AlicaElement*>();
    this->plansSaved = vector<AlicaElement*>();
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    this->configPath = sc->getConfigPath();
}

PlanWriter::~PlanWriter() {}

/**
 * Gets or sets the directory to save to.
 */
string PlanWriter::getTempPlanDir() {
    return tempPlanDir;
}
void PlanWriter::setTempPlanDir(string directory) {
    this->tempPlanDir = directory;
}

string PlanWriter::getConfigPath() {
    return configPath;
}

/**
 *  Gets or sets the plans to save.
 */
vector<AlicaElement*>& PlanWriter::getPlansToSave() {
    return plansToSave;
}
void PlanWriter::setPlansToSave(vector<AlicaElement*>& plansToSave) {
    this->plansToSave = plansToSave;
}

/**
 * Save all plans in the repository.
 */
void PlanWriter::saveAllPlans() {
    this->plansToSave.clear();
    for (auto pair : this->rep->getPlans()) {
        this->plansToSave.push_back(pair.second);
    }
    saveFileLoop();
}

/**
 * Saves a single plan.
 * @param p The plan to save.
 */
void PlanWriter::saveSinglePlan(Plan* p) {
    this->currentFile = supplementary::FileSystem::combinePaths(this->tempPlanDir, p->getName() + string(".pml"));
    tinyxml2::XMLDocument* doc = createPlanXMLDocument(p);

    if (!supplementary::FileSystem::pathExists(this->tempPlanDir)) {
        supplementary::FileSystem::createDirectory(this->tempPlanDir, 777);
    }
    doc->SaveFile(this->currentFile.c_str(), false);
}

void PlanWriter::saveSinglePlan(string directory, Plan* p) {
    this->currentFile = supplementary::FileSystem::combinePaths(directory, p->getFileName());
    tinyxml2::XMLDocument* doc = createPlanXMLDocument(p);

    if (!supplementary::FileSystem::pathExists(directory)) {
        supplementary::FileSystem::createDirectory(directory, 777);
    }
    doc->SaveFile(this->currentFile.c_str(), false);
}

void PlanWriter::saveFileLoop() {
    while (plansToSave.size() > 0) {
        AlicaElement* ae = plansToSave[plansToSave.size() - 1];
        plansToSave.erase(plansToSave.begin() + (plansToSave.size() - 1));
        if (dynamic_cast<Plan*>(ae) != nullptr) {
            saveSinglePlan(dynamic_cast<Plan*>(ae));
        } else {
            cout << "Saving of type " << typeid(ae).name() << " is not implemented." << endl;
            throw new exception();
        }
        plansSaved.push_back(ae);
    }
    this->plansToSave.clear();
    this->plansSaved.clear();
}

tinyxml2::XMLDocument* PlanWriter::createPlanXMLDocument(Plan* p) {
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

    tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("xml version=\"1.0\" encoding=\"ASCII\"");
    doc->InsertEndChild(decl);

    createPlanXMLNode(p, doc);

    return doc;
}

void PlanWriter::createPlanXMLNode(Plan* p, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xp = doc->NewElement("alica:Plan");
    xp->SetAttribute("xmi:version", "2.0");
    xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
    xp->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
    doc->InsertEndChild(xp);

    addPlanElementAttributes(p, xp);

    if (p->isMasterPlan()) {
        xp->SetAttribute("masterPlan", "true");
    } else {
        xp->SetAttribute("masterPlan", "false");
    }

    xp->SetAttribute("utilityFunction", "");
    stringstream ss;
    ss << p->getUtilityThreshold();
    xp->SetAttribute("utilityThreshold", ss.str().c_str());
    xp->SetAttribute("destinationPath", p->getDestinationPath().c_str());
    stringstream ss2;
    ss2 << "0.0";
    xp->SetAttribute("priority", ss2.str().c_str());
    xp->SetAttribute("minCardinality", to_string(p->getMinCardinality()).c_str());
    xp->SetAttribute("maxCardinality", to_string(p->getMaxCardinality()).c_str());

    if (p->getPreCondition() != nullptr) {
        tinyxml2::XMLElement* xc = doc->NewElement("conditions");
        xp->InsertEndChild(xc);
        xc->SetAttribute("xsi:type", "alica:PreCondition");
        addPlanElementAttributes(p->getPreCondition(), xc);
        xc->SetAttribute("conditionString", p->getPreCondition()->getConditionString().c_str());
        xc->SetAttribute("pluginName", p->getPreCondition()->getPlugInName().c_str());
        if (p->getPreCondition()->isEnabled()) {
            xc->SetAttribute("enabled", "true");
        } else {
            xc->SetAttribute("enabled", "false");
        }

        addConditionChildren(p->getPreCondition(), xc, doc);
    }
    if (p->getRuntimeCondition() != nullptr) {
        tinyxml2::XMLElement* xc = doc->NewElement("conditions");
        xp->InsertEndChild(xc);
        xc->SetAttribute("xsi:type", "alica:RuntimeCondition");
        addPlanElementAttributes(p->getRuntimeCondition(), xc);
        xc->SetAttribute("conditionString", p->getRuntimeCondition()->getConditionString().c_str());
        xc->SetAttribute("pluginName", p->getRuntimeCondition()->getPlugInName().c_str());
        addConditionChildren(p->getRuntimeCondition(), xc, doc);
    }
    if (p->getPostCondition() != nullptr) {
        tinyxml2::XMLElement* xc = doc->NewElement("conditions");
        xp->InsertEndChild(xc);
        xc->SetAttribute("xsi:type", "alica:postCondition");
        addPlanElementAttributes(p->getPostCondition(), xc);
        xc->SetAttribute("conditionString", p->getPostCondition()->getConditionString().c_str());
        xc->SetAttribute("pluginName", p->getPostCondition()->getPlugInName().c_str());
        addConditionChildren(p->getPostCondition(), xc, doc);
    }
    for (Variable* v : (*p->getVariables())) {
        tinyxml2::XMLElement* xc = doc->NewElement("vars");
        addPlanElementAttributes(v, xc);
        if (!(v->getType().empty())) {
            xc->SetAttribute("Type", v->getType().c_str());
        } else {
            xc->SetAttribute("Type", "");
        }
        xp->InsertEndChild(xc);
    }
    for (State* s : p->getStates()) {
        xp->InsertEndChild(createStateXMLNode(s, doc));
    }
    for (Transition* t : p->getTransitions()) {
        xp->InsertEndChild(createTransitionXMLNode(t, doc));
    }
    for (SyncTransition* s : p->getSyncTransitions()) {
        xp->InsertEndChild(createSynchronisationXMLNode(s, doc));
    }
    for (auto e : p->getEntryPoints()) {
        xp->InsertEndChild(createEntryPointXMLNode(e.second, doc));
    }
}

tinyxml2::XMLDocument* PlanWriter::createRoleSetXMLDocument(RoleSet* r) {
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

    tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("version=\"1.0\" encoding=\"ASCII\"");
    doc->InsertEndChild(decl);

    createRoleSet(r, doc);

    return doc;
}

void PlanWriter::saveRoleSet(RoleSet* r, string name) {
    this->currentFile = supplementary::FileSystem::combinePaths(this->tempPlanDir, name);
    tinyxml2::XMLDocument* doc = createRoleSetXMLDocument(r);

    if (!supplementary::FileSystem::pathExists(this->tempPlanDir)) {
        supplementary::FileSystem::createDirectory(this->tempPlanDir, 777);
    }
    doc->SaveFile(this->currentFile.c_str(), false);
}

void PlanWriter::saveRoleSet(RoleSet* r, string directory, string name) {
    this->currentFile = supplementary::FileSystem::combinePaths(directory, name);
    tinyxml2::XMLDocument* doc = createRoleSetXMLDocument(r);

    if (!supplementary::FileSystem::pathExists(directory)) {
        supplementary::FileSystem::createDirectory(directory, 777);
    }
    doc->SaveFile(this->currentFile.c_str(), false);
}

tinyxml2::XMLDocument* PlanWriter::createTaskRepositoryXMLDocument(TaskRepository* tr) {
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

    tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("version=\"1.0\" encoding=\"ASCII\"");
    doc->InsertEndChild(decl);

    createTaskRepository(tr, doc);

    return doc;
}

void PlanWriter::saveTaskRepository(TaskRepository* tr, string name) {
    this->currentFile = supplementary::FileSystem::combinePaths(this->tempPlanDir, name);
    tinyxml2::XMLDocument* doc = createTaskRepositoryXMLDocument(tr);

    if (!supplementary::FileSystem::pathExists(this->tempPlanDir)) {
        supplementary::FileSystem::createDirectory(this->tempPlanDir, 777);
    }
    doc->SaveFile(this->currentFile.c_str(), false);
}

void PlanWriter::saveTaskRepository(TaskRepository* tr, string directory, string name) {
    this->currentFile = supplementary::FileSystem::combinePaths(directory, name);
    tinyxml2::XMLDocument* doc = createTaskRepositoryXMLDocument(tr);

    if (!supplementary::FileSystem::pathExists(directory)) {
        supplementary::FileSystem::createDirectory(directory, 777);
    }
    doc->SaveFile(this->currentFile.c_str(), false);
}

void PlanWriter::addConditionChildren(Condition* c, tinyxml2::XMLElement* xn, tinyxml2::XMLDocument* doc) {
    for (Quantifier* q : c->getQuantifiers()) {
        tinyxml2::XMLElement* xc = doc->NewElement("quantifiers");
        if (dynamic_cast<ForallAgents*>(q) != nullptr) {
            xc->SetAttribute("xsi:type", "alica:ForallAgents");
        } else {
            cout << "Unknown Quantifier: " << q->toString() << endl;
            throw new exception();
        }
        addPlanElementAttributes(q, xc);
        xc->SetAttribute("scope", to_string(q->getScope()->getId()).c_str());
        for (string sort : q->getDomainIdentifiers()) {
            tinyxml2::XMLElement* xcc = doc->NewElement("sorts");
            xc->InsertEndChild(xcc);
            xcc->InsertEndChild(doc->NewText(sort.c_str()));
        }
        xn->InsertEndChild(xc);
    }
    for (Parameter* p : c->getParameters()) {
        tinyxml2::XMLElement* xc = doc->NewElement("parameters");
        addPlanElementAttributes(p, xc);
        xc->SetAttribute("key", p->getKey().c_str());
        xc->SetAttribute("value", p->getValue().c_str());
        xn->InsertEndChild(xc);
    }
    for (Variable* v : c->getVariables()) {
        tinyxml2::XMLElement* xc = doc->NewElement("vars");
        xc->InsertEndChild(doc->NewText((string("#") + to_string(v->getId())).c_str()));
        xn->InsertEndChild(xc);
    }
}

tinyxml2::XMLElement* PlanWriter::createStateXMLNode(State* s, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xs = doc->NewElement("states");
    if (dynamic_cast<SuccessState*>(s) != nullptr) {
        xs->SetAttribute("xsi:type", "alica:SuccessState");

    } else if (dynamic_cast<FailureState*>(s) != nullptr) {
        xs->SetAttribute("xsi:type", "alica:FailureState");
    } else {
    }
    addPlanElementAttributes(s, xs);
    if (s->getEntryPoint() != nullptr) {
        xs->SetAttribute("entryPoint", to_string(s->getEntryPoint()->getId()).c_str());
    }

    for (Parametrisation* p : s->getParametrisation()) {
        xs->InsertEndChild(createParametrisationXMLNode(p, doc));
    }

    for (AbstractPlan* p : s->getPlans()) {
        if (dynamic_cast<PlanType*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:PlanType");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }
        if (dynamic_cast<BehaviourConfiguration*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:BehaviourConfiguration");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }
        if (dynamic_cast<Plan*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:Plan");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }

        if (dynamic_cast<PlanningProblem*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:PlanningProblem");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }
    }

    for (Transition* t : s->getInTransitions()) {
        tinyxml2::XMLElement* xc = doc->NewElement("inTransitions");
        xs->InsertEndChild(xc);
        xc->InsertEndChild(doc->NewText((string("#") + to_string(t->getId())).c_str()));
    }

    for (Transition* t : s->getOutTransitions()) {
        tinyxml2::XMLElement* xc = doc->NewElement("outTransitions");
        xs->InsertEndChild(xc);
        xc->InsertEndChild(doc->NewText((string("#") + to_string(t->getId())).c_str()));
    }
    if (dynamic_cast<SuccessState*>(s) != nullptr) {
        SuccessState* su = dynamic_cast<SuccessState*>(s);
        if (su->getPostCondition() != nullptr) {
            xs->InsertEndChild(createResultXMLNode(su->getPostCondition(), doc));
        }

    } else if (dynamic_cast<FailureState*>(s) != nullptr) {
        FailureState* fu = dynamic_cast<FailureState*>(s);
        if (fu->getPostCondition() != nullptr) {
            xs->InsertEndChild(createResultXMLNode(fu->getPostCondition(), doc));
        }
    } else {
    }
    return xs;
}

tinyxml2::XMLElement* PlanWriter::createParametrisationXMLNode(Parametrisation* p, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xr = doc->NewElement("parametrisation");
    addPlanElementAttributes(p, xr);
    tinyxml2::XMLElement* xc = doc->NewElement("subplan");
    if (dynamic_cast<Plan*>(p->getSubPlan()) != nullptr) {
        xc->SetAttribute("xsi:type", "alica:Plan");
    } else if (dynamic_cast<PlanType*>(p->getSubPlan()) != nullptr) {
        xc->SetAttribute("xsi:type", "alica:PlanType");
    } else if (dynamic_cast<BehaviourConfiguration*>(p->getSubPlan()) != nullptr) {
        xc->SetAttribute("xsi:type", "alica:BehaviourConfiguration");
    }
    xc->InsertEndChild(
            doc->NewText((getRelativeFileName(p->getSubPlan()) + "#" + to_string(p->getSubPlan()->getId())).c_str()));
    xr->InsertEndChild(xc);

    xc = doc->NewElement("subvar");
    xc->InsertEndChild(
            doc->NewText((getRelativeFileName(p->getSubPlan()) + "#" + to_string(p->getSubVar()->getId())).c_str()));
    xr->InsertEndChild(xc);

    xc = doc->NewElement("var");
    xc->InsertEndChild(doc->NewText((string("#") + to_string(p->getVar()->getId())).c_str()));
    xr->InsertEndChild(xc);
    return xr;
}

tinyxml2::XMLElement* PlanWriter::createResultXMLNode(PostCondition* r, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xr = doc->NewElement("postCondition");
    addPlanElementAttributes(r, xr);
    xr->SetAttribute("conditionString", r->getConditionString().c_str());
    xr->SetAttribute("pluginName", r->getPlugInName().c_str());
    addConditionChildren(r, xr, doc);
    return xr;
}

tinyxml2::XMLElement* PlanWriter::createPreConditionXMLNode(PreCondition* c, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xr = doc->NewElement("preCondition");
    addPlanElementAttributes(c, xr);
    xr->SetAttribute("conditionString", c->getConditionString().c_str());
    xr->SetAttribute("pluginName", c->getPlugInName().c_str());
    if (c->isEnabled()) {
        xr->SetAttribute("enabled", "true");
    } else {
        xr->SetAttribute("enabled", "false");
    }

    addConditionChildren(c, xr, doc);
    return xr;
}

tinyxml2::XMLElement* PlanWriter::createSynchronisationXMLNode(SyncTransition* s, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xr = doc->NewElement("synchronisations");
    addPlanElementAttributes(s, xr);
    string synched = "";
    auto iter = s->getInSync().end();
    while (s->getInSync().size() > 0 && iter != s->getInSync().begin()) {
        iter = prev(iter);
        synched += to_string((*iter)->getId()) + " ";
    }
    xr->SetAttribute("synchedTransitions", supplementary::Configuration::trim(synched).c_str());
    xr->SetAttribute("talkTimeout", to_string(s->getTalkTimeOut()).c_str());
    xr->SetAttribute("syncTimeout", to_string(s->getSyncTimeOut()).c_str());
    xr->SetAttribute("failOnSyncTimeOut", "false");

    return xr;
}

tinyxml2::XMLElement* PlanWriter::createTransitionXMLNode(Transition* t, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xt = doc->NewElement("transitions");
    addPlanElementAttributes(t, xt);
    xt->SetAttribute("msg", "");
    if (t->getPreCondition() != nullptr) {
        xt->InsertEndChild(createPreConditionXMLNode(t->getPreCondition(), doc));
    }
    tinyxml2::XMLElement* xc = doc->NewElement("inState");
    xt->InsertEndChild(xc);
    xc->InsertEndChild(doc->NewText((string("#") + to_string(t->getInState()->getId())).c_str()));
    xc = doc->NewElement("outState");
    xt->InsertEndChild(xc);
    xc->InsertEndChild(doc->NewText((string("#") + to_string(t->getOutState()->getId())).c_str()));
    if (t->getSyncTransition() != nullptr) {
        xc = doc->NewElement("synchronisation");
        xt->InsertEndChild(xc);
        xc->InsertEndChild(doc->NewText((string("#") + to_string(t->getSyncTransition()->getId())).c_str()));
    }
    return xt;
}

tinyxml2::XMLElement* PlanWriter::createEntryPointXMLNode(EntryPoint* e, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xe = doc->NewElement("entryPoints");
    addPlanElementAttributes(e, xe);
    if (e->isSuccessRequired()) {
        xe->SetAttribute("successRequired", "true");
    } else {
        xe->SetAttribute("successRequired", "false");
    }
    xe->SetAttribute("minCardinality", e->getMinCardinality());
    xe->SetAttribute("maxCardinality", e->getMaxCardinality());
    tinyxml2::XMLElement* xc = doc->NewElement("task");
    xc->InsertEndChild(doc->NewText(
            (getRelativeFileName(
                     this->rep->getTaskRepositorys()[e->getTask()->getTaskRepository()->getId()]->getFileName()) +
                    "#" + to_string(e->getTask()->getId()))
                    .c_str()));
    xe->InsertEndChild(xc);
    xc = doc->NewElement("state");
    xc->InsertEndChild(doc->NewText((string("#") + to_string(e->getState()->getId())).c_str()));
    xe->InsertEndChild(xc);
    return xe;
}

void PlanWriter::addPlanElementAttributes(AlicaElement* p, tinyxml2::XMLElement* x) {
    x->SetAttribute("id", to_string(p->getId()).c_str());
    x->SetAttribute("name", p->getName().c_str());
    x->SetAttribute("comment", p->getComment().c_str());
}

string PlanWriter::getRelativeFileName(string file) {
    string curdir = this->currentFile;
    string ufile = "";
    if (supplementary::FileSystem::isPathRooted(file)) {
        ufile = file;
    } else {
        if (file.substr(file.size() - 4, 4) == ".beh" || file.substr(file.size() - 4, 4) == ".pty" ||
                file.substr(file.size() - 4, 4) == ".pml" || file.substr(file.size() - 3, 3) == ".pp") {
            supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
            string tfile = (*sc)["Alica"]->get<string>("Alica.PlanDir");
            supplementary::FileSystem::combinePaths(tfile, file);
            if (!supplementary::FileSystem::isPathRooted(tfile)) {
                tfile = supplementary::FileSystem::combinePaths(this->configPath, tfile);
            }
            ufile = tfile;
        } else {
            cout << "File reference not implemented: " << file << "(occurred in file " << this->currentFile << ")"
                 << endl;
            throw new exception();
        }
    }
    string ret = "";
    int pos = 0;
    for (int i = 0; i < curdir.size(); i++) {
        if (curdir.at(i) != ufile.at(i)) {
            break;
        }
        pos++;
    }
    ufile.erase(ufile.begin(), ufile.begin() + pos);
    ret = ufile;
    return ret;
}

string PlanWriter::getRelativeFileName(AbstractPlan* p) {
    if (find(this->plansToSave.begin(), this->plansToSave.end(), p) != this->plansToSave.end() ||
            find(this->plansSaved.begin(), this->plansSaved.end(), p) != this->plansSaved.end()) {
        string dirfile = this->tempPlanDir;
        dirfile += p->getFileName();
        return getRelativeFileName(dirfile);
    } else {
        return getRelativeFileName(p->getFileName());
    }
}

void PlanWriter::createRoleSet(RoleSet* r, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xp = doc->NewElement("alica:RoleSet");
    doc->InsertEndChild(xp);
    xp->SetAttribute("xmi:version", "2.0");
    xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
    xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
    xp->SetAttribute("id", to_string(ae->getAlicaClock()->now() + objectCounter++).c_str());
    xp->SetAttribute("name", r->getName().c_str());
    xp->SetAttribute("comment", r->getComment().c_str());
    xp->SetAttribute("usableWithPlanID", to_string(r->getUsableWithPlanId()).c_str());
    if (r->isIsDefault()) {
        xp->SetAttribute("default", "true");
    } else {
        xp->SetAttribute("default", "false");
    }

    for (RoleTaskMapping* rtm : r->getRoleTaskMappings()) {
        tinyxml2::XMLElement* xc = doc->NewElement("mappings");
        xp->InsertEndChild(xc);
        xc->SetAttribute("id", to_string(ae->getAlicaClock()->now() + objectCounter++).c_str());
        xc->SetAttribute("name", rtm->getName().c_str());
        xc->SetAttribute("comment", rtm->getComment().c_str());
        for (auto mapping : rtm->getTaskPriorities()) {
            tinyxml2::XMLElement* xd = doc->NewElement("taskPriorities");
            xc->InsertEndChild(xd);
            xd->SetAttribute("id", to_string(ae->getAlicaClock()->now() + objectCounter++).c_str());
            xd->SetAttribute("name", "");
            xd->SetAttribute("comment", "");
            xd->SetAttribute("key", to_string(mapping.first).c_str());
            xd->SetAttribute("value", to_string(mapping.second).c_str());
        }
        tinyxml2::XMLElement* xe = doc->NewElement("role");
        xc->InsertEndChild(xe);
        xe->InsertEndChild(doc->NewText(
                (rtm->getRole()->getRoleDefinitionSet()->getFileName() + "#" + to_string(rtm->getRole()->getId()))
                        .c_str()));
    }
}

void PlanWriter::createTaskRepository(TaskRepository* tr, tinyxml2::XMLDocument* doc) {
    tinyxml2::XMLElement* xp = doc->NewElement("alica:TaskRepository");
    doc->InsertEndChild(xp);
    xp->SetAttribute("xmi:version", "2.0");
    xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
    xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
    xp->SetAttribute("id", to_string(ae->getAlicaClock()->now() + objectCounter++).c_str());
    xp->SetAttribute("name", tr->getName().c_str());
    xp->SetAttribute("comment", tr->getComment().c_str());
    xp->SetAttribute("defaultTask", to_string(tr->getDefaultTask()).c_str());

    for (Task* t : tr->getTasks()) {
        tinyxml2::XMLElement* xc = doc->NewElement("tasks");
        xp->InsertEndChild(xc);
        xc->SetAttribute("id", to_string(t->getId()).c_str());
        xc->SetAttribute("name", t->getName().c_str());
        xc->SetAttribute("comment", t->getComment().c_str());
        xc->SetAttribute("description", t->getDescription().c_str());
    }
}

} /* namespace alica */
