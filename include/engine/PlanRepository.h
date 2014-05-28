/*
 * PlanRepository.h
 *
 *  Created on: May 28, 2014
 *      Author: snook
 */

#ifndef PLANREPOSITORY_H_
#define PLANREPOSITORY_H_
#include <stdio.h>
#include <map>
#include <Plan.h>
#include <Task.h>
#include <Behaviour.h>
#include <BehaviourConfiguration.h>
#include <PlanType.h>
#include <Role.h>
#include <Characteristic.h>
#include <Capability.h>
#include <State.h>
#include <EntryPoint.h>
#include <Transition.h>
#include <SyncTransition.h>
#include <Quantifier.h>
#include <Variable.h>
#include <RoleDefinitionSet.h>
#include <TaskRepository.h>

namespace alica
{
class PlanRepository
{
public:
	PlanRepository();
	virtual ~PlanRepository();

	std::map<long, alica::Plan> Plans();
	std::map<long, alica::Task> Tasks();
	std::map<long, alica::Behaviour> Behaviours();
	std::map<long, alica::BehaviourConfiguration> BehaviourConfigurations();
	std::map<long, alica::PlanType> PlanTypes();
	std::map<long, alica::Role> Roles();
	std::map<long, alica::Characteristic> Characteristics();
	std::map<long, alica::Capability> Capabilities();
	std::map<long, alica::State> States();
	std::map<long, alica::EntryPoint> EntryPoints();
	std::map<long, alica::Transition> Transitions();
	std::map<long, alica::SyncTransition> SyncTransitions();
	std::map<long, alica::Quantifier> Quantifiers();
	std::map<long, alica::Variable> Variables();
	std::map<long, alica::RoleDefinitionSet> RoleDefinitionSets();
	std::map<long, alica::TaskRepository> TaskRepositorys();

};
}
#endif /* PLANREPOSITORY_H_ */
