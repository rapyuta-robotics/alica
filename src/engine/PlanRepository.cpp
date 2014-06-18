/*
 * PlanRepository.cpp
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#include "engine/PlanRepository.h"
#include "engine/model/Quantifier.h"

namespace alica
{
	PlanRepository::PlanRepository()
	{

	}
	PlanRepository::~PlanRepository()
	{

	}
	map<long, BehaviourConfiguration*>& PlanRepository::getBehaviourConfigurations()
	{
		return behaviourConfigurations;
	}

	void PlanRepository::setBehaviourConfigurations(const map<long, BehaviourConfiguration*>& behaviourConfigurations)
	{
		this->behaviourConfigurations = behaviourConfigurations;
	}

	map<long, Behaviour*>& PlanRepository::getBehaviours()
	{
		return behaviours;
	}

	void PlanRepository::setBehaviours(const map<long, Behaviour*>& behaviours)
	{
		this->behaviours = behaviours;
	}

	map<long, Capability*>& PlanRepository::getCapabilities()
	{
		return capabilities;
	}

	void PlanRepository::setCapabilities(const map<long, Capability*>& capabilities)
	{
		this->capabilities = capabilities;
	}

	map<long, Characteristic*>& PlanRepository::getCharacteristics()
	{
		return characteristics;
	}

	void PlanRepository::setCharacteristics(const map<long, Characteristic*>& characteristics)
	{
		this->characteristics = characteristics;
	}

	map<long, EntryPoint*>& PlanRepository::getEntryPoints()
	{
		return entryPoints;
	}

	void PlanRepository::setEntryPoints(const map<long, EntryPoint*>& entryPoints)
	{
		this->entryPoints = entryPoints;
	}

	map<long, Plan*>& PlanRepository::getPlans()
	{
		return plans;
	}

	void PlanRepository::setPlans(const map<long, Plan*>& plans)
	{
		this->plans = plans;
	}

	map<long, PlanType*>& PlanRepository::getPlanTypes()
	{
		return planTypes;
	}

	void PlanRepository::setPlanTypes(const map<long, PlanType*>& planTypes)
	{
		this->planTypes = planTypes;
	}

	map<long, Quantifier*>& PlanRepository::getQuantifiers()
	{
		return quantifiers;
	}

	void PlanRepository::setQuantifiers(const map<long, Quantifier*>& quantifiers)
	{
		this->quantifiers = quantifiers;
	}

	map<long, RoleDefinitionSet*>& PlanRepository::getRoleDefinitionSets()
	{
		return roleDefinitionSets;
	}

	void PlanRepository::setRoleDefinitionSets(const map<long, RoleDefinitionSet*>& roleDefinitionSets)
	{
		this->roleDefinitionSets = roleDefinitionSets;
	}

	map<long, Role*>& PlanRepository::getRoles()
	{
		return roles;
	}

	void PlanRepository::setRoles(const map<long, Role*>& roles)
	{
		this->roles = roles;
	}

	map<long, State*>& PlanRepository::getStates()
	{
		return states;
	}

	void PlanRepository::setStates(const map<long, State*>& states)
	{
		this->states = states;
	}

	map<long, SyncTransition*>& PlanRepository::getSyncTransitions()
	{
		return syncTransitions;
	}

	void PlanRepository::setSyncTransitions(const map<long, SyncTransition*>& syncTransitions)
	{
		this->syncTransitions = syncTransitions;
	}

	map<long, TaskRepository*>& PlanRepository::getTaskRepositorys()
	{
		return taskRepositorys;
	}

	void PlanRepository::setTaskRepositorys(const map<long, TaskRepository*>& taskRepositorys)
	{
		this->taskRepositorys = taskRepositorys;
	}

	map<long, Task*>& PlanRepository::getTasks()
	{
		return tasks;
	}

	void PlanRepository::setTasks(const map<long, Task*>& tasks)
	{
		this->tasks = tasks;
	}

	map<long, Transition*>& PlanRepository::getTransitions()
	{
		return transitions;
	}

	void PlanRepository::setTransitions(const map<long, Transition*>& transitions)
	{
		this->transitions = transitions;
	}

	map<long, Variable*>& PlanRepository::getVariables()
	{
		return variables;
	}

	void PlanRepository::setVariables(const map<long, Variable*>& variables)
	{
		this->variables = variables;
	}
}

