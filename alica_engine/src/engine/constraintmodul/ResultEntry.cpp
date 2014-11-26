/*
 * ResultEntry.cpp
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */
#include "engine/constraintmodul/ResultEntry.h"

#include <engine/model/Variable.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include <engine/containers/SolverVar.h>
#include <engine/containers/SolverResult.h>
#include <limits>

using namespace std;

namespace alica
{

	ResultEntry::ResultEntry(int robotId, AlicaEngine* ae)
	{
		this->ae = ae;
		this->id = robotId;
	}

	ResultEntry::~ResultEntry()
	{
		// TODO Auto-generated destructor stub
	}

	int ResultEntry::getId()
	{
		return id;
	}

	void ResultEntry::addValue(long vid, double val)
	{
		long now = ae->getIAlicaClock()->now();
		shared_ptr<VarValue> vv;
		auto it = this->values.find(vid);
		lock_guard<std::mutex> lock(valueLock);
		if (it != values.end())
		{
			vv = it->second;
			vv->val = val;
			vv->lastUpdate = now;
		}
		else
		{
			//is this needed? see C# code!
			it = this->values.find(vid);
			if (it != values.end())
			{
				vv = it->second;
				vv->val = val;
				vv->lastUpdate = now;
			}
			else
			{
				vv = make_shared<VarValue>(vid, val, now);
				this->values[vid] = vv;
			}

		}
	}

	void ResultEntry::clear()
	{
		lock_guard<std::mutex> lock(valueLock);
		this->values.clear();
	}

	shared_ptr<vector<SolverVar*>> ResultEntry::getCommunicatableResults(long ttl4Communication)
	{
		lock_guard<std::mutex> lock(valueLock);
		shared_ptr<vector<SolverVar*>> lv = make_shared<vector<SolverVar*>>();
		long now = ae->getIAlicaClock()->now();
		for(auto iterator = values.begin(); iterator != values.end(); iterator++)
		{
			if(iterator->second->lastUpdate + ttl4Communication > now)
			{
				SolverVar* sv = new SolverVar();
				sv->id = iterator->second->id;
				sv->value = iterator->second->val;
				lv->push_back(sv);
			}
		}
		return lv;
	}

	double ResultEntry::getValue(long vid, long ttl4Usage)
	{
		long now = ae->getIAlicaClock()->now();
		lock_guard<std::mutex> lock(valueLock);
		auto it = this->values.find(vid);
		if(it != values.end()) {
			if(it->second->lastUpdate > now) {
				return it->second->val;
			}
		}
		return std::numeric_limits<double>::max();
	}

	shared_ptr<vector<double>> ResultEntry::getValues(shared_ptr<vector<Variable*>> query, long ttl4Usage)
	{
		shared_ptr<vector<double>> ret = make_shared<vector<double>>(query->size());
		int i = 0;
		for(auto it = query->begin(); it != query->end(); it++, i++) {
			ret->at(i) = getValue((*it)->getId(), ttl4Usage);
		}
		return ret;
	}

} /* namespace alica */
