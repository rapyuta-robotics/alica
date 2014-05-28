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
#include "model/Plan.h"
#include "model/Task.h"
#include "model/Behaviour.h"
#include "model/BehaviourConfiguration.h"
#include "model/PlanType.h"
#include "model/Role.h"
#include "model/Characteristic.h"
#include "model/Capability.h"
#include "model/State.h"
#include "model/EntryPoint.h"
#include "model/Transition.h"
#include "model/SyncTransition.h"
#include "model/Quantifier.h"
#include "model/Variable.h"
#include "model/RoleDefinitionSet.h"
#include "model/TaskRepository.h"

namespace alica
{
class PlanRepository
{
public:
	PlanRepository();
	virtual ~PlanRepository();

	std::map<long, alica::Plan> plans();
	std::map<long, alica::Task> tasks();
	std::map<long, alica::Behaviour> behaviours();
	std::map<long, alica::BehaviourConfiguration> behaviourConfigurations();
	std::map<long, alica::PlanType> planTypes();
	std::map<long, alica::Role> roles();
	std::map<long, alica::Characteristic> characteristics();
	std::map<long, alica::Capability> capabilities();
	std::map<long, alica::State> states();
	std::map<long, alica::EntryPoint> entryPoints();
	std::map<long, alica::Transition> transitions();
	std::map<long, alica::SyncTransition> syncTransitions();
	std::map<long, alica::Quantifier> quantifiers();
	std::map<long, alica::Variable> variables();
	std::map<long, alica::RoleDefinitionSet> roleDefinitionSets();
	std::map<long, alica::TaskRepository> taskRepositorys();

};
}
#endif /* PLANREPOSITORY_H_ */
