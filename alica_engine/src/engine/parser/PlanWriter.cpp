/*
 * PlanWriter.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: Stefan Jakob
 */

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/FailureState.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Parameter.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/PlanningProblem.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/Quantifier.h"
#include "engine/model/Role.h"
#include "engine/model/RoleDefinitionSet.h"
#include "engine/model/RoleSet.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/SuccessState.h"
#include "engine/model/SyncTransition.h"
#include "engine/model/Task.h"
#include "engine/model/TaskRepository.h"
#include "engine/model/Transition.h"
#include "engine/model/Variable.h"
#include "engine/parser/tinyxml2.h"
#include <FileSystem.h>
#include <SystemConfig.h>
#include <engine/parser/PlanWriter.h>

namespace alica
{
using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using std::to_string;

int PlanWriter::s_objectCounter = 0;

PlanWriter::PlanWriter(AlicaEngine* ae, PlanRepository* rep)
{
    this->ae = ae;
    string path = essentials::SystemConfig::getInstance().getConfigPath();
    this->tempPlanDir = essentials::FileSystem::combinePaths(path, "plans/tmp/");
    this->_rep = rep;
}

PlanWriter::~PlanWriter() {}

/**
 * Gets or sets the directory to save to.
 */
const std::string& PlanWriter::getTempPlanDir() const
{
    return tempPlanDir;
}
void PlanWriter::setTempPlanDir(const std::string& directory)
{
    this->tempPlanDir = directory;
}

/**
 *  Gets or sets the plans to save.
 */
const AlicaElementGrp& PlanWriter::getPlansToSave() const
{
    return plansToSave;
}
void PlanWriter::setPlansToSave(const AlicaElementGrp& plansToSave)
{
    this->plansToSave = plansToSave;
}

/**
 * Save all plans in the repository.
 */
void PlanWriter::saveAllPlans()
{
    this->plansToSave.clear();
    for (const Plan* p : this->_rep->getPlans()) {
        this->plansToSave.push_back(p);
    }
    saveFileLoop();
}

/**
 * Saves a single plan.
 * @param p The plan to save.
 */
void PlanWriter::saveSinglePlan(const Plan* p)
{
    this->_currentFile = essentials::FileSystem::combinePaths(this->tempPlanDir, p->getName() + string(".pml"));
    tinyxml2::XMLDocument* doc = createPlanXMLDocument(p);

    if (!essentials::FileSystem::pathExists(this->tempPlanDir)) {
        essentials::FileSystem::createDirectory(this->tempPlanDir, 777);
    }
    doc->SaveFile(this->_currentFile.c_str(), false);
}

void PlanWriter::saveSinglePlan(std::string directory, const Plan* p)
{
    this->_currentFile = essentials::FileSystem::combinePaths(directory, p->getFileName());
    tinyxml2::XMLDocument* doc = createPlanXMLDocument(p);

    if (!essentials::FileSystem::pathExists(directory)) {
        essentials::FileSystem::createDirectory(directory, 777);
    }
    doc->SaveFile(this->_currentFile.c_str(), false);
}

void PlanWriter::saveFileLoop()
{
    while (plansToSave.size() > 0) {
        const AlicaElement* ae = plansToSave[plansToSave.size() - 1];
        plansToSave.erase(plansToSave.begin() + (plansToSave.size() - 1));
        const Plan* ap = dynamic_cast<const Plan*>(ae);
        if (ap != nullptr) {
            saveSinglePlan(ap);
        } else {
            cout << "Saving of type " << typeid(ae).name() << " is not implemented." << endl;
            throw std::exception();
        }
        plansSaved.push_back(ae);
    }
    this->plansToSave.clear();
    this->plansSaved.clear();
}

tinyxml2::XMLDocument* PlanWriter::createPlanXMLDocument(const Plan* p)
{
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

    tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("xml version=\"1.0\" encoding=\"ASCII\"");
    doc->InsertEndChild(decl);

    createPlanXMLNode(p, doc);

    return doc;
}

void PlanWriter::createPlanXMLNode(const Plan* p, tinyxml2::XMLDocument* doc)
{
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
    for (const Variable* v : p->getVariables()) {
        tinyxml2::XMLElement* xc = doc->NewElement("vars");
        addPlanElementAttributes(v, xc);
        if (!(v->getType().empty())) {
            xc->SetAttribute("Type", v->getType().c_str());
        } else {
            xc->SetAttribute("Type", "");
        }
        xp->InsertEndChild(xc);
    }
    for (const State* s : p->getStates()) {
        xp->InsertEndChild(createStateXMLNode(s, doc));
    }
    for (const Transition* t : p->getTransitions()) {
        xp->InsertEndChild(createTransitionXMLNode(t, doc));
    }
    for (const SyncTransition* s : p->getSyncTransitions()) {
        xp->InsertEndChild(createSynchronisationXMLNode(s, doc));
    }
    for (const EntryPoint* e : p->getEntryPoints()) {
        xp->InsertEndChild(createEntryPointXMLNode(e, doc));
    }
}

tinyxml2::XMLDocument* PlanWriter::createRoleSetXMLDocument(const RoleSet* r)
{
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

    tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("version=\"1.0\" encoding=\"ASCII\"");
    doc->InsertEndChild(decl);

    createRoleSet(r, doc);

    return doc;
}

void PlanWriter::saveRoleSet(const RoleSet* r, string name)
{
    this->_currentFile = essentials::FileSystem::combinePaths(this->tempPlanDir, name);
    tinyxml2::XMLDocument* doc = createRoleSetXMLDocument(r);

    if (!essentials::FileSystem::pathExists(this->tempPlanDir)) {
        essentials::FileSystem::createDirectory(this->tempPlanDir, 777);
    }
    doc->SaveFile(this->_currentFile.c_str(), false);
}

void PlanWriter::saveRoleSet(const RoleSet* r, string directory, string name)
{
    this->_currentFile = essentials::FileSystem::combinePaths(directory, name);
    tinyxml2::XMLDocument* doc = createRoleSetXMLDocument(r);

    if (!essentials::FileSystem::pathExists(directory)) {
        essentials::FileSystem::createDirectory(directory, 777);
    }
    doc->SaveFile(this->_currentFile.c_str(), false);
}

tinyxml2::XMLDocument* PlanWriter::createTaskRepositoryXMLDocument(const TaskRepository* tr)
{
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

    tinyxml2::XMLDeclaration* decl = doc->NewDeclaration("version=\"1.0\" encoding=\"ASCII\"");
    doc->InsertEndChild(decl);

    createTaskRepository(tr, doc);

    return doc;
}

void PlanWriter::saveTaskRepository(const TaskRepository* tr, string name)
{
    this->_currentFile = essentials::FileSystem::combinePaths(this->tempPlanDir, name);
    tinyxml2::XMLDocument* doc = createTaskRepositoryXMLDocument(tr);

    if (!essentials::FileSystem::pathExists(this->tempPlanDir)) {
        essentials::FileSystem::createDirectory(this->tempPlanDir, 777);
    }
    doc->SaveFile(this->_currentFile.c_str(), false);
}

void PlanWriter::saveTaskRepository(const TaskRepository* tr, string directory, string name)
{
    this->_currentFile = essentials::FileSystem::combinePaths(directory, name);
    tinyxml2::XMLDocument* doc = createTaskRepositoryXMLDocument(tr);

    if (!essentials::FileSystem::pathExists(directory)) {
        essentials::FileSystem::createDirectory(directory, 777);
    }
    doc->SaveFile(this->_currentFile.c_str(), false);
}

void PlanWriter::addConditionChildren(const Condition* c, tinyxml2::XMLElement* xn, tinyxml2::XMLDocument* doc)
{
    for (const Quantifier* q : c->getQuantifiers()) {
        tinyxml2::XMLElement* xc = doc->NewElement("quantifiers");
        if (dynamic_cast<const ForallAgents*>(q) != nullptr) {
            xc->SetAttribute("xsi:type", "alica:ForallAgents");
        } else {
            cout << "Unknown Quantifier: " << q->toString() << endl;
            throw std::exception();
        }
        addPlanElementAttributes(q, xc);
        xc->SetAttribute("scope", to_string(q->getScope()->getId()).c_str());
        for (const std::string& sort : q->getDomainIdentifiers()) {
            tinyxml2::XMLElement* xcc = doc->NewElement("sorts");
            xc->InsertEndChild(xcc);
            xcc->InsertEndChild(doc->NewText(sort.c_str()));
        }
        xn->InsertEndChild(xc);
    }
    for (const Parameter* p : c->getParameters()) {
        tinyxml2::XMLElement* xc = doc->NewElement("parameters");
        addPlanElementAttributes(p, xc);
        xc->SetAttribute("key", p->getKey().c_str());
        xc->SetAttribute("value", p->getValue().c_str());
        xn->InsertEndChild(xc);
    }
    for (const Variable* v : c->getVariables()) {
        tinyxml2::XMLElement* xc = doc->NewElement("vars");
        xc->InsertEndChild(doc->NewText((string("#") + to_string(v->getId())).c_str()));
        xn->InsertEndChild(xc);
    }
}

tinyxml2::XMLElement* PlanWriter::createStateXMLNode(const State* s, tinyxml2::XMLDocument* doc)
{
    tinyxml2::XMLElement* xs = doc->NewElement("states");
    if (s->isSuccessState()) {
        xs->SetAttribute("xsi:type", "alica:SuccessState");

    } else if (s->isFailureState()) {
        xs->SetAttribute("xsi:type", "alica:FailureState");
    } else {
    }
    addPlanElementAttributes(s, xs);
    if (s->getEntryPoint() != nullptr && s->getEntryPoint()->getState() == s) {
        xs->SetAttribute("entryPoint", to_string(s->getEntryPoint()->getId()).c_str());
    }

    for (const Parametrisation* p : s->getParametrisation()) {
        xs->InsertEndChild(createParametrisationXMLNode(p, doc));
    }

    for (const AbstractPlan* p : s->getPlans()) {
        if (dynamic_cast<const PlanType*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:PlanType");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }
        if (dynamic_cast<const BehaviourConfiguration*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:BehaviourConfiguration");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }
        if (dynamic_cast<const Plan*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:Plan");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }

        if (dynamic_cast<const PlanningProblem*>(p) != nullptr) {
            tinyxml2::XMLElement* xc = doc->NewElement("plans");
            xs->InsertEndChild(xc);
            xc->SetAttribute("xsi:type", "alica:PlanningProblem");
            xc->InsertEndChild(doc->NewText((getRelativeFileName(p) + "#" + to_string(p->getId())).c_str()));
            continue;
        }
    }

    for (const Transition* t : s->getInTransitions()) {
        tinyxml2::XMLElement* xc = doc->NewElement("inTransitions");
        xs->InsertEndChild(xc);
        xc->InsertEndChild(doc->NewText((string("#") + to_string(t->getId())).c_str()));
    }

    for (const Transition* t : s->getOutTransitions()) {
        tinyxml2::XMLElement* xc = doc->NewElement("outTransitions");
        xs->InsertEndChild(xc);
        xc->InsertEndChild(doc->NewText((string("#") + to_string(t->getId())).c_str()));
    }
    if (s->isSuccessState()) {
        const SuccessState* su = static_cast<const SuccessState*>(s);
        if (su->getPostCondition() != nullptr) {
            xs->InsertEndChild(createResultXMLNode(su->getPostCondition(), doc));
        }

    } else if (s->isFailureState()) {
        const FailureState* fu = static_cast<const FailureState*>(s);
        if (fu->getPostCondition() != nullptr) {
            xs->InsertEndChild(createResultXMLNode(fu->getPostCondition(), doc));
        }
    } else {
    }
    return xs;
}

tinyxml2::XMLElement* PlanWriter::createParametrisationXMLNode(const Parametrisation* p, tinyxml2::XMLDocument* doc)
{
    tinyxml2::XMLElement* xr = doc->NewElement("parametrisation");
    addPlanElementAttributes(p, xr);
    tinyxml2::XMLElement* xc = doc->NewElement("subplan");
    if (dynamic_cast<const Plan*>(p->getSubPlan()) != nullptr) {
        xc->SetAttribute("xsi:type", "alica:Plan");
    } else if (dynamic_cast<const PlanType*>(p->getSubPlan()) != nullptr) {
        xc->SetAttribute("xsi:type", "alica:PlanType");
    } else if (dynamic_cast<const BehaviourConfiguration*>(p->getSubPlan()) != nullptr) {
        xc->SetAttribute("xsi:type", "alica:BehaviourConfiguration");
    }
    xc->InsertEndChild(doc->NewText((getRelativeFileName(p->getSubPlan()) + "#" + to_string(p->getSubPlan()->getId())).c_str()));
    xr->InsertEndChild(xc);

    xc = doc->NewElement("subvar");
    xc->InsertEndChild(doc->NewText((getRelativeFileName(p->getSubPlan()) + "#" + to_string(p->getSubVar()->getId())).c_str()));
    xr->InsertEndChild(xc);

    xc = doc->NewElement("var");
    xc->InsertEndChild(doc->NewText((string("#") + to_string(p->getVar()->getId())).c_str()));
    xr->InsertEndChild(xc);
    return xr;
}

tinyxml2::XMLElement* PlanWriter::createResultXMLNode(const PostCondition* r, tinyxml2::XMLDocument* doc)
{
    tinyxml2::XMLElement* xr = doc->NewElement("postCondition");
    addPlanElementAttributes(r, xr);
    xr->SetAttribute("conditionString", r->getConditionString().c_str());
    xr->SetAttribute("pluginName", r->getPlugInName().c_str());
    addConditionChildren(r, xr, doc);
    return xr;
}

tinyxml2::XMLElement* PlanWriter::createPreConditionXMLNode(const PreCondition* c, tinyxml2::XMLDocument* doc)
{
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

tinyxml2::XMLElement* PlanWriter::createSynchronisationXMLNode(const SyncTransition* s, tinyxml2::XMLDocument* doc)
{
    tinyxml2::XMLElement* xr = doc->NewElement("synchronisations");
    addPlanElementAttributes(s, xr);
    std::string synched = "";
    for (int i = s->getInSync().size() - 1; i >= 0; --i) {
        synched.append(to_string(s->getInSync()[i]->getId()));
        synched.append(" ");
    }
    xr->SetAttribute("synchedTransitions", essentials::Configuration::trim(synched).c_str());
    xr->SetAttribute("talkTimeout", to_string(s->getTalkTimeOut().inMilliseconds()).c_str());
    xr->SetAttribute("syncTimeout", to_string(s->getSyncTimeOut().inMilliseconds()).c_str());
    xr->SetAttribute("failOnSyncTimeOut", "false");

    return xr;
}

tinyxml2::XMLElement* PlanWriter::createTransitionXMLNode(const Transition* t, tinyxml2::XMLDocument* doc)
{
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

tinyxml2::XMLElement* PlanWriter::createEntryPointXMLNode(const EntryPoint* e, tinyxml2::XMLDocument* doc)
{
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
    xc->InsertEndChild(doc->NewText((getRelativeFileName(this->_rep->getTaskRepositorys()[e->getTask()->getTaskRepository()->getId()]->getFileName()) + "#" +
                                     to_string(e->getTask()->getId()))
                                            .c_str()));
    xe->InsertEndChild(xc);
    xc = doc->NewElement("state");
    xc->InsertEndChild(doc->NewText((string("#") + to_string(e->getState()->getId())).c_str()));
    xe->InsertEndChild(xc);
    return xe;
}

void PlanWriter::addPlanElementAttributes(const AlicaElement* p, tinyxml2::XMLElement* x)
{
    x->SetAttribute("id", to_string(p->getId()).c_str());
    x->SetAttribute("name", p->getName().c_str());
    x->SetAttribute("comment", "");
}

std::string PlanWriter::getRelativeFileName(const std::string& file)
{
    std::string curdir = this->_currentFile;
    std::string ufile = "";
    if (essentials::FileSystem::isPathRooted(file)) {
        ufile = file;
    } else {
        essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
        std::string configPath = sc.getConfigPath();
        if (file.substr(file.size() - 4, 4) == ".beh" || file.substr(file.size() - 4, 4) == ".pty" || file.substr(file.size() - 4, 4) == ".pml" ||
                file.substr(file.size() - 3, 3) == ".pp") {

            std::string tfile = sc["Alica"]->get<string>("Alica.PlanDir", NULL);
            // tfile = essentials::FileSystem::combinePaths(tfile, file);
            if (!essentials::FileSystem::isPathRooted(tfile)) {
                tfile = essentials::FileSystem::combinePaths(configPath, tfile);
            }
            ufile = tfile;
        } else if (file.substr(file.size() - 4, 4) == ".tsk") {
            std::string tfile = sc["Alica"]->get<string>("Alica.MiscDir", NULL);
            // tfile = essentials::FileSystem::combinePaths(tfile, file);
            if (!essentials::FileSystem::isPathRooted(tfile)) {
                tfile = essentials::FileSystem::combinePaths(configPath, tfile);
            }
            ufile = tfile;
        } else {
            cout << "File reference not implemented: " << file << "(occurred in file " << this->_currentFile << ")" << endl;
            throw std::exception();
        }
    }
    string ret = "";
    int pos = 0;
    for (int i = 0; i < static_cast<int>(curdir.size()); ++i) {
        if (static_cast<int>(ufile.size()) <= i || curdir.at(i) != ufile.at(i)) {
            break;
        }
        ++pos;
    }
    ufile.erase(ufile.begin(), ufile.begin() + pos);
    return ufile;
}

string PlanWriter::getRelativeFileName(const AbstractPlan* p)
{
    if (find(this->plansToSave.begin(), this->plansToSave.end(), p) != this->plansToSave.end() ||
            find(this->plansSaved.begin(), this->plansSaved.end(), p) != this->plansSaved.end()) {
        std::string dirfile = this->tempPlanDir;
        dirfile += p->getFileName();
        return getRelativeFileName(dirfile);
    } else {
        return getRelativeFileName(p->getFileName());
    }
}

void PlanWriter::createRoleSet(const RoleSet* r, tinyxml2::XMLDocument* doc)
{
    tinyxml2::XMLElement* xp = doc->NewElement("alica:RoleSet");
    doc->InsertEndChild(xp);
    xp->SetAttribute("xmi:version", "2.0");
    xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
    xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
    xp->SetAttribute("id", to_string(ae->getAlicaClock()->now().inNanoseconds() + s_objectCounter++).c_str());
    xp->SetAttribute("name", r->getName().c_str());
    xp->SetAttribute("usableWithPlanID", to_string(r->getUsableWithPlanId()).c_str());
    if (r->isDefault()) {
        xp->SetAttribute("default", "true");
    } else {
        xp->SetAttribute("default", "false");
    }

    for (const RoleTaskMapping* rtm : r->getRoleTaskMappings()) {
        tinyxml2::XMLElement* xc = doc->NewElement("mappings");
        xp->InsertEndChild(xc);
        xc->SetAttribute("id", to_string(ae->getAlicaClock()->now().inNanoseconds() + s_objectCounter++).c_str());
        xc->SetAttribute("name", rtm->getName().c_str());
        for (auto mapping : rtm->getTaskPriorities()) {
            tinyxml2::XMLElement* xd = doc->NewElement("taskPriorities");
            xc->InsertEndChild(xd);
            xd->SetAttribute("id", to_string(ae->getAlicaClock()->now().inNanoseconds() + s_objectCounter++).c_str());
            xd->SetAttribute("name", "");
            xd->SetAttribute("key", to_string(mapping.first).c_str());
            xd->SetAttribute("value", to_string(mapping.second).c_str());
        }
        tinyxml2::XMLElement* xe = doc->NewElement("role");
        xc->InsertEndChild(xe);
        xe->InsertEndChild(doc->NewText((rtm->getRole()->getRoleDefinitionSet()->getFileName() + "#" + to_string(rtm->getRole()->getId())).c_str()));
    }
}

void PlanWriter::createTaskRepository(const TaskRepository* tr, tinyxml2::XMLDocument* doc)
{
    tinyxml2::XMLElement* xp = doc->NewElement("alica:TaskRepository");
    doc->InsertEndChild(xp);
    xp->SetAttribute("xmi:version", "2.0");
    xp->SetAttribute("xmlns:xmi", "http://www.omg.org/XMI");
    xp->SetAttribute("xmlns:alica", "http:///de.uni_kassel.vs.cn");
    xp->SetAttribute("id", to_string(ae->getAlicaClock()->now().inNanoseconds() + s_objectCounter++).c_str());
    xp->SetAttribute("name", tr->getName().c_str());
    xp->SetAttribute("defaultTask", to_string(tr->getDefaultTask()).c_str());

    for (const Task* t : tr->getTasks()) {
        tinyxml2::XMLElement* xc = doc->NewElement("tasks");
        xp->InsertEndChild(xc);
        xc->SetAttribute("id", to_string(t->getId()).c_str());
        xc->SetAttribute("name", t->getName().c_str());
        xc->SetAttribute("description", t->getDescription().c_str());
    }
}

} /* namespace alica */
