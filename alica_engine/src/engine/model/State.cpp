/*
 * State.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/State.h"
namespace alica
{

	/**
	 * Basic constructor
	 */
	State::State()
	{
		this->terminal = false;
		this->successState = false;
		this->failureState = false;
		this->inPlan = nullptr;
		this->entryPoint = nullptr;

	}

	/**
	 * Constructor which accepts a unique id.
	 * @param id A int
	 */
	State::State(long id)
	{
		this->id = id;
		this->terminal = false;
		this->successState = false;
		this->failureState = false;
		this->inPlan = nullptr;
		this->entryPoint = nullptr;
	}

	State::~State()
	{
	}

	bool State::isFailureState() const
	{
		return failureState;
	}

	void State::setFailureState(bool failureState)
	{
		this->failureState = failureState;
	}

	Plan* State::getInPlan() const
	{
		return inPlan;
	}

	void State::setInPlan(Plan* inPlan)
	{
		this->inPlan = inPlan;
	}

	list<Transition*>& State::getInTransitions()
	{
		return inTransitions;
	}

	void State::setInTransitions(const list<Transition*>& inTransitions)
	{
		this->inTransitions = inTransitions;
	}

	 list<Transition*>& State::getOutTransitions()
	{
		return outTransitions;
	}

	void State::setOutTransitions(list<Transition*> outTransition)
	{
		this->outTransitions = outTransition;
	}

	 list<Parametrisation*>& State::getParametrisation()
	{
		return parametrisation;
	}

	void State::setParametrisation(const list<Parametrisation*>& parametrisation)
	{
		this->parametrisation = parametrisation;
	}

	list<AbstractPlan*>& State::getPlans()
	{
		return plans;
	}

	void State::setPlans(const list<AbstractPlan*>& plans)
	{
		this->plans = plans;
	}

	bool State::isSuccessState() const
	{
		return successState;
	}

	void State::setSuccessState(bool successState)
	{
		this->successState = successState;
	}

	bool State::isTerminal() const
	{
		return terminal;
	}

	void State::setTerminal(bool terminal)
	{
		this->terminal = terminal;
	}
	 EntryPoint* alica::State::getEntryPoint()
	{
		return entryPoint;
	}

	void alica::State::setEntryPoint( EntryPoint* entryPoint)
	{
		this->entryPoint = entryPoint;
	}
	string State::toString() const
	{
		stringstream ss;
		ss << AlicaElement::toString();
		return ss.str();
	}

} /* namespace Alica */


