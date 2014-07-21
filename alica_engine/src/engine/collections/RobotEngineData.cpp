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

namespace alica
{

	RobotEngineData::RobotEngineData()
	{

	}

	RobotEngineData::RobotEngineData(RobotProperties* properties)
	{
		this->active = false;
		this->lastMessageTime = 0;
		this->properties = properties;
		this->initSortedTerms();

	}

	RobotEngineData::~RobotEngineData()
	{
	}

	bool RobotEngineData::isActive() const
	{
		return active;
	}

	void RobotEngineData::setActive(bool active)
	{
		this->active = active;
	}

	RobotProperties* RobotEngineData::getProperties() const
	{
		return properties;
	}

	void RobotEngineData::setProperties(RobotProperties* properties)
	{
		this->properties = properties;
	}
	SuccessMarks* RobotEngineData::getSuccessMarks() const
	{
		return successMarks;
	}

	void RobotEngineData::setSuccessMarks(SuccessMarks* successMarks)
	{
		this->successMarks = successMarks;
	}

	unsigned long RobotEngineData::getLastMessageTime() const
	{
		return lastMessageTime;
	}

	void RobotEngineData::setLastMessageTime(unsigned long lastMessageTime)
	{
		this->lastMessageTime = lastMessageTime;
	}

	void RobotEngineData::initSortedTerms()
	{
		map<long, Quantifier*> qs = AlicaEngine::getInstance()->getPlanRepository()->getQuantifiers();
		for (map<long, Quantifier*>::const_iterator iter = qs.begin(); iter != qs.end(); iter++)
		{
			if (typeid(iter->second) == typeid(ForallAgents))
			{
				for (string s : iter->second->getDomainIdentifiers())
				{
					Variable* v = new Variable(makeUniqueId(s), this->getProperties()->getName() + "." + s,"");
					this->sortedVariables.insert(pair<string, Variable*>(s,v));
				}
			}
		}
	}

	Variable* RobotEngineData::getSortedVariable(string sort)
	{
		auto iterator = this->sortedVariables.find(sort);
		if(iterator != this->sortedVariables.end())
		{
			return iterator->second;
		}
		else
		{
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
		long ret = (long)this->getProperties()->getId() << 32;
		ret += (unsigned int) hash<string>()(s);
		auto iterator = AlicaEngine::getInstance()->getPlanParser()->getParsedElements()->find(ret);
		if(iterator != AlicaEngine::getInstance()->getPlanParser()->getParsedElements()->end())
		{
			AlicaEngine::getInstance()->abort("TO: Hash Collision in generating unique ID: ", ret);
		}
		return ret;
	}

} /* namespace alica */

