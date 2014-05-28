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

namespace Alica
{
class PlanRepository
{
public:
	PlanRepository();
	virtual ~PlanRepository();

	std::map<long, Alica::Plan> Plans ();
	std::map<long, Alica::Task> Tasks ();
	std::map<long, Alica::Behaviour> Behaviours ();
	std::map<long, Alica::BehaviourConfiguration> BehaviourConfigurations ();
	std::map<long, Alica::PlanType> PlanTypes ();
	std::map<long, Alica::Role> Roles ();
	std::map<long, Alica::Characteristic> Characteristics ();
	std::map<long, Alica::Capability> Capabilities ();
	std::map<long, Alica::State> States ();
	std::map<long, Alica::EntryPoint> EntryPoints ();
	std::map<long, Alica::Transition> Transitions ();
	std::map<long, Alica::SyncTransition> SyncTransitions ();
	std::map<long, Alica::Quantifier> Quantifiers ();
	std::map<long, Alica::Variable> Variables ();
	std::map<long, Alica::RoleDefinitionSet> RoleDefinitionSets ();
	std::map<long, Alica::TaskRepository> TaskRepositorys ();

};
}
#endif /* PLANREPOSITORY_H_ */
