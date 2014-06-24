/*
 * Quantifier.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Quantifier.h"
#include "engine/AlicaEngine.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

	Quantifier::Quantifier(long id)
	{
		this->id = id;

	}

	Quantifier::~Quantifier()
	{
	}

	bool Quantifier::isScopeIsEntryPoint() const
	{
		return scopeIsEntryPoint;
	}

	void Quantifier::setScopeIsEntryPoint(bool scopeIsEntryPoint)
	{
		this->scopeIsEntryPoint = scopeIsEntryPoint;
	}

	bool Quantifier::isScopeIsPlan() const
	{
		return scopeIsPlan;
	}

	void Quantifier::setScopeIsPlan(bool scopeIsPlan)
	{
		this->scopeIsPlan = scopeIsPlan;
	}

	bool Quantifier::isScopeIsState() const
	{
		return scopeIsState;
	}

	list<string>& Quantifier::getDomainIdentifiers()
	{
		return domainIdentifiers;
	}

	void Quantifier::setDomainIdentifiers(const list<string>& domainIdentifiers)
	{
		this->domainIdentifiers = domainIdentifiers;
	}
	State* Quantifier::getScopedState()
	{
		return this->state;
	}

	EntryPoint* Quantifier::getScopedEntryPoint()
	{
		return this->entryPoint;
	}

	Plan* Quantifier::getScopedPlan()
	{
		return this->plan;
	}

	void Quantifier::setScope(AlicaElement* ae)
	{
		scopeIsEntryPoint = (typeid(ae) == typeid(EntryPoint*));
		scopeIsPlan = (typeid(ae) == typeid(Plan*));
		scopeIsState = (typeid(ae) == typeid(State*));

		if (scopeIsPlan)
		{
			this->plan = (Plan*)ae;
		}
		else if (scopeIsEntryPoint)
		{
			this->entryPoint = (EntryPoint*)ae;
		}
		else if (scopeIsState)
		{
			this->state = (State*)ae;
		}
		else
		{
			AlicaEngine::getInstance()->abort("Scope of Quantifier is not an entrypoint, plan, or state: ", ae);
		}
	}

	AlicaElement* Quantifier::getScope()
	{
		if (scopeIsPlan)
		{
			return this->plan;
		}
		if (scopeIsState)
		{
			return this->state;
		}
		if (scopeIsEntryPoint)
		{
			return this->entryPoint;
		}
		return NULL;
	}

	void Quantifier::setScopeIsState(bool scopeIsState)
	{
		this->scopeIsState = scopeIsState;
	}

} /* namespace Alica */


