/*
 * PlanWriter.h
 *
 *  Created on: Sep 5, 2014
 *      Author: Stefan Jakob
 */

#ifndef PLANWRITER_H_
#define PLANWRITER_H_

using namespace std;
#include <vector>
#include <string>
#include <typeinfo>

namespace tinyxml2 {
	class XMLElement;
	class XMLDocument;
	class XMLAttribute;
}


namespace alica
{

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
	class PlanWriter
	{
	public:
		PlanWriter(AlicaEngine* ae, PlanRepository* rep);
		virtual ~PlanWriter();
		void saveAllPlans();
		void saveSinglePlan(Plan* p);
		void saveSinglePlan(string directory, Plan* p);
		void saveFileLoop();
		tinyxml2::XMLDocument* createPlanXMLDocument(Plan* p);
		void createPlanXMLNode(Plan* p, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLDocument* createRoleSetXMLDocument(RoleSet* r);
		void saveRoleSet(RoleSet* r, string name);
		void saveRoleSet(RoleSet* r, string directory, string name);
		tinyxml2::XMLDocument* createTaskRepositoryXMLDocument(TaskRepository* tr);
		void saveTaskRepository(TaskRepository* tr, string name);
		void saveTaskRepository(TaskRepository* tr, string directory, string name);
		string getTempPlanDir() ;
		void setTempPlanDir(string directory);
		string getConfigPath();
		vector<AlicaElement*>& getPlansToSave();


	private:
		PlanRepository* rep;
		void setPlansToSave(vector<AlicaElement*>& plansToSave);
		string currentFile;
		void addConditionChildren(Condition* c,tinyxml2::XMLElement* xn, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLElement* createStateXMLNode(State* s, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLElement* createParametrisationXMLNode(Parametrisation* p, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLElement* createResultXMLNode(PostCondition* r, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLElement* createPreConditionXMLNode(PreCondition* c, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLElement* createSynchronisationXMLNode(SyncTransition* s, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLElement* createTransitionXMLNode(Transition* t, tinyxml2::XMLDocument* doc);
		tinyxml2::XMLElement* createEntryPointXMLNode(EntryPoint* e, tinyxml2::XMLDocument* doc);
		void addPlanElementAttributes(AlicaElement* p, tinyxml2::XMLElement* x);
//		tinyxml2::XMLAttribute* getXsiTypeAttribute(string type, tinyxml2::XMLDocument* doc);
		string getRelativeFileName(string file);
		string getRelativeFileName(AbstractPlan* p);
		void createRoleSet(RoleSet* r, tinyxml2::XMLDocument* doc);
		void createTaskRepository(TaskRepository* tr, tinyxml2::XMLDocument* doc);
		static int objectCounter;


	protected:
		AlicaEngine* ae;
		string tempPlanDir;
		string configPath;
		vector<AlicaElement*> plansToSave;
		vector<AlicaElement*> plansSaved;
	};

} /* namespace alica */

#endif /* PLANWRITER_H_ */
