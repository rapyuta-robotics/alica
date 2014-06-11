/*
 * State.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/State.h"
namespace alica
{

	State::State()
	{
		this->terminal = false;
		this->successState = false;
		this->failureState = false;
		// TODO Auto-generated constructor stub

	}
	State::State(long id)
	{
		this->id = id;

	}

	State::~State()
	{
		// TODO Auto-generated destructor stub
	}

	bool State::isFailureState() const
	{
		return failureState;
	}

	void State::setFailureState(bool failureState)
	{
		this->failureState = failureState;
	}

	const Plan* State::getInPlan() const
	{
		return inPlan;
	}

	void State::setInPlan(Plan* inPlan)
	{
		this->inPlan = inPlan;
	}

	const list<Transition*>& State::getInTransitions() const
	{
		return inTransitions;
	}

	void State::setInTransitions(const list<Transition*>& inTransitions)
	{
		this->inTransitions = inTransitions;
	}

	const list<Transition*>& State::getOutTransitions() const
	{
		return outTransitions;
	}

	void State::setOutTransitions(const list<Transition*>& outTransition)
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

	const list<AbstractPlan*>& State::getPlans() const
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
	string State::toString() const
	{
		stringstream ss;
		ss << AlicaElement::toString();
		ss << "Filename: " << this << endl;
		return ss.str();
	}

} /* namespace Alica */
