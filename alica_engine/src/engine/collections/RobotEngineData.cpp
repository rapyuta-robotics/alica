/*
 * RobotEngineData.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */


#include <engine/collections/RobotEngineData.h>

#include <engine/model/Quantifier.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanRepository.h>
#include <engine/model/ForallAgents.h>
#include <engine/model/Variable.h>
#include <engine/collections/RobotProperties.h>
#include <engine/IPlanParser.h>
#include <engine/model/AlicaElement.h>
#include "engine/collections/SuccessMarks.h"

namespace alica
{

	/**
	 * Basic constructor
	 * @param properties This robot's RobotProperties
	 */
	RobotEngineData::RobotEngineData(AlicaEngine* ae, shared_ptr<RobotProperties> properties)
	{
		this->ae = ae;
		this->active = false;
		this->lastMessageTime = 0;
		this->properties = properties;
		this->initDomainVariables();
		this->successMarks = make_shared<SuccessMarks>(ae);
		this->lastRole = nullptr;

	}

	RobotEngineData::~RobotEngineData()
	{
		for(auto x : this->domainVariables)
		{
			delete x.second;
		}
	}

	bool RobotEngineData::isActive()
	{
		return active;
	}

	void RobotEngineData::setActive(bool active)
	{
		this->active = active;
	}

	shared_ptr<RobotProperties> RobotEngineData::getProperties()
	{
		return properties;
	}

	void RobotEngineData::setProperties(shared_ptr<RobotProperties> properties)
	{
		this->properties = properties;
	}
	shared_ptr<SuccessMarks> RobotEngineData::getSuccessMarks()
	{
		return successMarks;
	}

	void RobotEngineData::setSuccessMarks(shared_ptr<SuccessMarks> successMarks)
	{
		this->successMarks = successMarks;
	}

	unsigned long RobotEngineData::getLastMessageTime()
	{
		return lastMessageTime;
	}

	void RobotEngineData::setLastMessageTime(unsigned long lastMessageTime)
	{
		this->lastMessageTime = lastMessageTime;
	}

	void RobotEngineData::initDomainVariables()
	{
		auto qs = ae->getPlanRepository()->getQuantifiers();
		//for (map<long, Quantifier*>::const_iterator iter = qs.begin(); iter != qs.end(); iter++)
		for (auto quantifierPair : qs)
		{
			if (dynamic_cast<ForallAgents*>(quantifierPair.second) != nullptr)
			{
				for (string s : quantifierPair.second->getDomainIdentifiers())
				{
					Variable* v = new Variable(makeUniqueId(s), this->getProperties()->getName() + "." + s,"");
					this->domainVariables.insert(pair<string, Variable*>(s,v));
				}
			}
		}
	}

	Variable* RobotEngineData::getDomainVariable(string ident)
	{
		auto iterator = this->domainVariables.find(ident);
		if(iterator != this->domainVariables.end())
		{
			return iterator->second;
		}
		else
		{
//			cout << "RobotEngineData: DomainVarible not found returning nullptr" << endl;
			return nullptr;
		}
	}

	Role* RobotEngineData::getLastRole()
	{
		return lastRole;
	}

	void RobotEngineData::setLastRole(Role* lastRole)
	{
		this->lastRole = lastRole;
	}

	long RobotEngineData::makeUniqueId(string s)
	{
		long ret = (long) (std::hash<alica::IRobotID>()(this->getProperties()->getId()) + std::hash<string>()(s));
		if(this->ae->getPlanParser()->getParsedElements()->find(ret) != ae->getPlanParser()->getParsedElements()->end())
		{
			ae->abort("TO: Hash Collision in generating unique ID: ", ret);
		}
		return ret;
	}

} /* namespace alica */

