/*
 * PlanWriter.h
 *
 *  Created on: Sep 5, 2014
 *      Author: Stefan Jakob
 */

#ifndef PLANWRITER_H_
#define PLANWRITER_H_

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
class AlicaElement;
class Plan;
class State;
class Condition;
class Parametrisation;
class TaskRepository;
class PostCondition;
class PreCondition;
class SyncTransition;
class Transition;
class EntryPoint;
class RoleSet;
class AbstractPlan;
class AlicaEngine;

/**
 * The PlanWriter can be used to store generated plans.
 */
class PlanWriter {
public:
    PlanWriter(AlicaEngine* ae, PlanRepository* rep);
    virtual ~PlanWriter();
    void saveAllPlans();
    void saveSinglePlan(const Plan* p);
    void saveSinglePlan(string directory,const  Plan* p);
    void saveFileLoop();
    tinyxml2::XMLDocument* createPlanXMLDocument(const Plan* p);
    void createPlanXMLNode(const Plan* p, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLDocument* createRoleSetXMLDocument(const RoleSet* r);
    void saveRoleSet(const RoleSet* r, string name);
    void saveRoleSet(const RoleSet* r, string directory, string name);
    tinyxml2::XMLDocument* createTaskRepositoryXMLDocument(const TaskRepository* tr);
    void saveTaskRepository(const TaskRepository* tr, string name);
    void saveTaskRepository(const TaskRepository* tr, string directory, string name);
    std::string getTempPlanDir();
    void setTempPlanDir(string directory);
    std::string getConfigPath() const;
    std::vector<AlicaElement*>& getPlansToSave() const;

private:
    PlanRepository* rep;
    void setPlansToSave(std::vector<AlicaElement*>& plansToSave);
    std::string currentFile;
    void addConditionChildren(const Condition* c, tinyxml2::XMLElement* xn, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createStateXMLNode(const State* s, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createParametrisationXMLNode(const Parametrisation* p, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createResultXMLNode(const PostCondition* r, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createPreConditionXMLNode(const PreCondition* c, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createSynchronisationXMLNode(const SyncTransition* s, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createTransitionXMLNode(const Transition* t, tinyxml2::XMLDocument* doc);
    tinyxml2::XMLElement* createEntryPointXMLNode(const EntryPoint* e, tinyxml2::XMLDocument* doc);
    void addPlanElementAttributes(const AlicaElement* p, tinyxml2::XMLElement* x);
    //		tinyxml2::XMLAttribute* getXsiTypeAttribute(string type, tinyxml2::XMLDocument* doc);
    std::string getRelativeFileName(std::string file);
    std::string getRelativeFileName(const AbstractPlan* p);
    void createRoleSet(const RoleSet* r, tinyxml2::XMLDocument* doc);
    void createTaskRepository(const TaskRepository* tr, tinyxml2::XMLDocument* doc);
    static int objectCounter;

protected:
    AlicaEngine* ae;
    std::string tempPlanDir;
    std::string configPath;
    std::vector<const AlicaElement*> plansToSave;
    std::vector<const AlicaElement*> plansSaved;
};

} /* namespace alica */

#endif /* PLANWRITER_H_ */
