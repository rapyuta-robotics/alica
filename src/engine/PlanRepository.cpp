/*
 * PlanRepository.cpp
 *
 *  Created on: May 28, 2014
 *      Author: snook
 */

#include "engine/PlanRepository.h"

namespace alica
{
	PlanRepository::PlanRepository()
	{

	}
	PlanRepository::~PlanRepository()
	{

	}
	const map<long, BehaviourConfiguration>& PlanRepository::getBehaviourConfigurations() const
	{
		return behaviourConfigurations;
	}

	void PlanRepository::setBehaviourConfigurations(const map<long, BehaviourConfiguration>& behaviourConfigurations)
	{
		this->behaviourConfigurations = behaviourConfigurations;
	}

	const map<long, Behaviour>& PlanRepository::getBehaviours() const
	{
		return behaviours;
	}

	void PlanRepository::setBehaviours(const map<long, Behaviour>& behaviours)
	{
		this->behaviours = behaviours;
	}

	const map<long, Capability>& PlanRepository::getCapabilities() const
	{
		return capabilities;
	}

	void PlanRepository::setCapabilities(const map<long, Capability>& capabilities)
	{
		this->capabilities = capabilities;
	}

	const map<long, Characteristic>& PlanRepository::getCharacteristics() const
	{
		return characteristics;
	}

	void PlanRepository::setCharacteristics(const map<long, Characteristic>& characteristics)
	{
		this->characteristics = characteristics;
	}

	const map<long, EntryPoint>& PlanRepository::getEntryPoints() const
	{
		return entryPoints;
	}

	void PlanRepository::setEntryPoints(const map<long, EntryPoint>& entryPoints)
	{
		this->entryPoints = entryPoints;
	}

	map<long, Plan> PlanRepository::getPlans() const
	{
		return plans;
	}

	void PlanRepository::setPlans(const map<long, Plan>& plans)
	{
		this->plans = plans;
	}

	const map<long, PlanType>& PlanRepository::getPlanTypes() const
	{
		return planTypes;
	}

	void PlanRepository::setPlanTypes(const map<long, PlanType>& planTypes)
	{
		this->planTypes = planTypes;
	}

	const map<long, Quantifier>& PlanRepository::getQuantifiers() const
	{
		return quantifiers;
	}

	void PlanRepository::setQuantifiers(const map<long, Quantifier>& quantifiers)
	{
		this->quantifiers = quantifiers;
	}

	const map<long, RoleDefinitionSet>& PlanRepository::getRoleDefinitionSets() const
	{
		return roleDefinitionSets;
	}

	void PlanRepository::setRoleDefinitionSets(const map<long, RoleDefinitionSet>& roleDefinitionSets)
	{
		this->roleDefinitionSets = roleDefinitionSets;
	}

	const map<long, Role>& PlanRepository::getRoles() const
	{
		return roles;
	}

	void PlanRepository::setRoles(const map<long, Role>& roles)
	{
		this->roles = roles;
	}

	const map<long, State>& PlanRepository::getStates() const
	{
		return states;
	}

	void PlanRepository::setStates(const map<long, State>& states)
	{
		this->states = states;
	}

	const map<long, SyncTransition>& PlanRepository::getSyncTransitions() const
	{
		return syncTransitions;
	}

	void PlanRepository::setSyncTransitions(const map<long, SyncTransition>& syncTransitions)
	{
		this->syncTransitions = syncTransitions;
	}

	const map<long, TaskRepository>& PlanRepository::getTaskRepositorys() const
	{
		return taskRepositorys;
	}

	void PlanRepository::setTaskRepositorys(const map<long, TaskRepository>& taskRepositorys)
	{
		this->taskRepositorys = taskRepositorys;
	}

	const map<long, Task>& PlanRepository::getTasks() const
	{
		return tasks;
	}

	void PlanRepository::setTasks(const map<long, Task>& tasks)
	{
		this->tasks = tasks;
	}

	const map<long, Transition>& PlanRepository::getTransitions() const
	{
		return transitions;
	}

	void PlanRepository::setTransitions(const map<long, Transition>& transitions)
	{
		this->transitions = transitions;
	}

	const map<long, Variable>& PlanRepository::getVariables() const
	{
		return variables;
	}

	void PlanRepository::setVariables(const map<long, Variable>& variables)
	{
		this->variables = variables;
	}
}

