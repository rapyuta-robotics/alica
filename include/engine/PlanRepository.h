/*
 * PlanRepository.h
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#ifndef PLANREPOSITORY_H_
#define PLANREPOSITORY_H_

using namespace std;

#include <stdio.h>
#include <iostream>
#include <map>

namespace alica
{
	class BehaviourConfiguration;
	class Behaviour;
	class Capability;
	class Characteristic;
	class EntryPoint;
	class Plan;
	class PlanType;
	class Quantifier;
	class RoleDefinitionSet;
	class Role;
	class State;
	class SyncTransition;
	class TaskRepository;
	class Task;
	class Transition;
	class Variable;

	class PlanRepository
	{
	public:
		PlanRepository();
		virtual ~PlanRepository();

		map<long, BehaviourConfiguration*>& getBehaviourConfigurations();
		map<long, Behaviour*>& getBehaviours();
		map<long, Capability*>& getCapabilities();
		map<long, Characteristic*>& getCharacteristics();
		map<long, EntryPoint*>& getEntryPoints();
		map<long, Plan*>& getPlans();
		map<long, PlanType*>& getPlanTypes();
		map<long, Quantifier*>& getQuantifiers();
		map<long, RoleDefinitionSet*>& getRoleDefinitionSets();
		map<long, Role*>& getRoles();
		map<long, State*>& getStates();
		map<long, SyncTransition*>& getSyncTransitions();
		map<long, TaskRepository*>& getTaskRepositorys();
		map<long, Task*>& getTasks();
		map<long, Transition*>& getTransitions();
		map<long, Variable*>& getVariables();

	private:
		map<long, Plan*> plans;
		map<long, Task*> tasks;
		map<long, Behaviour*> behaviours;
		map<long, BehaviourConfiguration*> behaviourConfigurations;
		map<long, PlanType*> planTypes;
		map<long, Role*> roles;
		map<long, Characteristic*> characteristics;
		map<long, Capability*> capabilities;
		map<long, State*> states;
		map<long, EntryPoint*> entryPoints;
		map<long, Transition*> transitions;
		map<long, SyncTransition*> syncTransitions;
		map<long, Quantifier*> quantifiers;
		map<long, Variable*> variables;
		map<long, RoleDefinitionSet*> roleDefinitionSets;
		map<long, TaskRepository*> taskRepositorys;

		void setBehaviourConfigurations(const map<long, BehaviourConfiguration*>& behaviourConfigurations);
		void setBehaviours(const map<long, Behaviour*>& behaviours);
		void setCapabilities(const map<long, Capability*>& capabilities);
		void setCharacteristics(const map<long, Characteristic*>& characteristics);
		void setEntryPoints(const map<long, EntryPoint*>& entryPoints);
		void setPlans(const map<long, Plan*>& plans);
		void setPlanTypes(const map<long, PlanType*>& planTypes);
		void setQuantifiers(const map<long, Quantifier*>& quantifiers);
		void setRoleDefinitionSets(const map<long, RoleDefinitionSet*>& roleDefinitionSets);
		void setRoles(const map<long, Role*>& roles);
		void setStates(const map<long, State*>& states);
		void setSyncTransitions(const map<long, SyncTransition*>& syncTransitions);
		void setTaskRepositorys(const map<long, TaskRepository*>& taskRepositorys);
		void setTasks(const map<long, Task*>& tasks);
		void setTransitions(const map<long, Transition*>& transitions);
		void setVariables(const map<long, Variable*>& variables);
	};
}
#endif /* PLANREPOSITORY_H_ */
