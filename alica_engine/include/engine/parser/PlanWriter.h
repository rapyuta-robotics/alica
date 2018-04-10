/*
 * PlanWriter.h
 *
 *  Created on: Sep 5, 2014
 *      Author: Stefan Jakob
 */

#pragma once

#include "engine/Types.h"

#include <vector>
#include <string>
#include <typeinfo>

namespace tinyxml2 {
class XMLElement;
class XMLDocument;
class XMLAttribute;
}  // namespace tinyxml2

namespace alica {

class PlanRepository;
class Condition;
class Parametrisation;
class TaskRepository;
class PostCondition;
class PreCondition;
class EntryPoint;
class RoleSet;
class AlicaEngine;

/**
 * The PlanWriter can be used to store generated plans.
 */
class PlanWriter {
public:
    PlanWriter(AlicaEngine* ae, PlanRepository* rep);
    virtual ~PlanWriter();

    const std::string& getTempPlanDir() const;
    const std::string& getConfigPath() const;
    const AlicaElementSet& getPlansToSave() const;

    void saveAllPlans();
    void saveFileLoop();
    void saveSinglePlan(const Plan* p);
    void saveSinglePlan(std::string directory, const Plan* p);
    void saveRoleSet(const RoleSet* r, std::string name);
    void saveRoleSet(const RoleSet* r, std::string directory, std::string name);
    void saveTaskRepository(const TaskRepository* tr, std::string name);
    void saveTaskRepository(const TaskRepository* tr, std::string directory, std::string name);
    void setTempPlanDir(const std::string& directory);
    void createPlanXMLNode(const Plan* p, tinyxml2::XMLDocument* doc);

    tinyxml2::XMLDocument* createPlanXMLDocument(const Plan* p);
    tinyxml2::XMLDocument* createRoleSetXMLDocument(const RoleSet* r);
    tinyxml2::XMLDocument* createTaskRepositoryXMLDocument(const TaskRepository* tr);

private:
    PlanRepository* rep;
    std::string currentFile;
    static int objectCounter;

    std::string getRelativeFileName(std::string file);
    std::string getRelativeFileName(const AbstractPlan* p);

    void setPlansToSave(const AlicaElementSet& plansToSave);
    void addConditionChildren(const Condition* c, tinyxml2::XMLElement* xn, tinyxml2::XMLDocument* doc);
    void addPlanElementAttributes(const AlicaElement* p, tinyxml2::XMLElement* x);
    //		tinyxml2::XMLAttribute* getXsiTypeAttribute(string type, tinyxml2::XMLDocument* doc);
    void createRoleSet(const RoleSet* r, tinyxml2::XMLDocument* doc);
    void createTaskRepository(const TaskRepository* tr, tinyxml2::XMLDocument* doc);

    tinyxml2::XMLElement* createStateXMLNode(const State* s, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createParametrisationXMLNode(const Parametrisation* p, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createResultXMLNode(const PostCondition* r, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createPreConditionXMLNode(const PreCondition* c, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createSynchronisationXMLNode(const SyncTransition* s, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createTransitionXMLNode(const Transition* t, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createEntryPointXMLNode(const EntryPoint* e, tinyxml2::XMLDocument* doc);

protected:
    AlicaEngine* ae;
    std::string tempPlanDir;
    std::string configPath;
    AlicaElementSet plansToSave;
    AlicaElementSet plansSaved;
};

} /* namespace alica */