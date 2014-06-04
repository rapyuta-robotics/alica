/*
 * Plan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Plan.h"

namespace alica
{
	Plan::Plan() :
			AbstractPlan()
	{

	}
	Plan::Plan(long id)
	{
		this->id = id;
	}

	Plan::~Plan()
	{
	}
	void Plan::setFilename(string filename)
	{
		this->filename = filename;
	}
	string Plan::getFilename()
	{
		return filename;
	}
	string Plan::toString() const
	{
		stringstream ss;
		ss << AbstractPlan::toString();
		ss << "Filename: " << this->filename << endl;
		return ss.str();
	}
	EntryPoint* Plan::getEntryPointTaskID(long taskID)
	{
		for (map<long, alica::EntryPoint*>::const_iterator iter = entryPoints.begin(); iter != entryPoints.end(); iter++)
		{
			const Task* task = iter->second->getTask();
			if(task != nullptr){
				if(task->getId() == taskID){
					return iter->second;
				}
			}
			else{
				cout << "Model: Class Plan: Entrypoint with ID " << iter->second->getId() << " does not have a Task" << endl;
				throw new exception();
			}
		}
		return nullptr;
	}

	map<long, EntryPoint*>& Plan::getEntryPoints()
	{
		return entryPoints;
	}

	void Plan::setEntryPoints(const map<long, EntryPoint*>& entryPoints)
	{
		this->entryPoints = entryPoints;
	}

	list<FailureState>& Plan::getFailureStates()
	{
		return failureStates;
	}

	void Plan::setFailureStates(const list<FailureState>& failureStates)
	{
		this->failureStates = failureStates;
	}

	int Plan::getMaxCardinality()
	{
		return maxCardinality;
	}

	void Plan::setMaxCardinality(int maxCardinality)
	{
		this->maxCardinality = maxCardinality;
	}

	int Plan::getMinCardinality()
	{
		return minCardinality;
	}

	void Plan::setMinCardinality(int minCardinality)
	{
		this->minCardinality = minCardinality;
	}

	PostCondition& Plan::getPostCondition()
	{
		return postCondition;
	}

	void Plan::setPostCondition(const PostCondition& postCondition)
	{
		this->postCondition = postCondition;
	}

	list<State>& Plan::getStates()
	{
		return states;
	}

	void Plan::setStates(const list<State>& states)
	{
		this->states = states;
	}

	list<SuccessState>& Plan::getSuccessStates()
	{
		return successStates;
	}

	void Plan::setSuccessStates(const list<SuccessState>& successStates)
	{
		this->successStates = successStates;
	}

	list<SyncTransition>& Plan::getSyncTransitions()
	{
		return syncTransitions;
	}

	void Plan::setSyncTransitions(const list<SyncTransition>& syncTransitions)
	{
		this->syncTransitions = syncTransitions;
	}

	list<Transition>& Plan::getTransitions()
	{
		return transitions;
	}

	void Plan::setTransitions(const list<Transition>& transitions)
	{
		this->transitions = transitions;
	}

} /* namespace Alica */

