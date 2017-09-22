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
		this->plan = nullptr;
		this->entryPoint = nullptr;
		this->scopeIsEntryPoint = false;
		this->scopeIsPlan = false;
		this->scopeIsState = false;
		this->state = nullptr;

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

	/**
	 * Returns the scope of this quantifier, returns null, if the scope is not a state.
	 */
	State* Quantifier::getScopedState()
	{
		return this->state;
	}

	/**
	 * Returns the scope of this quantifier, returns null, if the scope is not an EntryPoint.
	 */
	EntryPoint* Quantifier::getScopedEntryPoint()
	{
		return this->entryPoint;
	}

	/**
	 * Returns the scope of this quantifier, returns null, if the scope is not a Plan.
	 */
	Plan* Quantifier::getScopedPlan()
	{
		return this->plan;
	}

	/**
	 * Set the scope of this quantifier, called by the ModelFactory
	 * @param ae An AlicaElement
	 */
	void Quantifier::setScope(AlicaEngine* engine, AlicaElement* element)
	{
		scopeIsEntryPoint = (dynamic_cast<EntryPoint*>(element) !=0);
		scopeIsPlan = (dynamic_cast<Plan*>(element) !=0 );
		scopeIsState = (dynamic_cast<State*>(element) !=0);

		if (scopeIsPlan)
		{
			this->plan = (Plan*)element;
		}
		else if (scopeIsEntryPoint)
		{
			this->entryPoint = (EntryPoint*)element;
		}
		else if (scopeIsState)
		{
			this->state = (State*)element;
		}
		else
		{
			engine->abort("Scope of Quantifier is not an entrypoint, plan, or state", element->getId());
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


