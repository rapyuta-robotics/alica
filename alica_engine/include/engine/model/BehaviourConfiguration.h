/*
 * BehaviourConfiguration.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOURCONFIGURATION_H_
#define BEHAVIOURCONFIGURATION_H_

using namespace std;

#include <map>
#include <string>

#include "AbstractPlan.h"

namespace alica
{
	class Behaviour;

	/**
	 * A Behaviour Configuration encapsulates a set of static parameters and a set of variables for a (Basic)Behaviour.
	 *
	 * The BehaviourConfiguration is indirectly derived from AlicaElement, therefore it
	 * is owned by the PlanRepository and should never be deleted or changed by anybody else.
	 */
	class BehaviourConfiguration : public AbstractPlan
	{
	public:
		BehaviourConfiguration();
		BehaviourConfiguration(long id);
		virtual ~BehaviourConfiguration();

		string toString();

		int getDeferring() const;
		void setDeferring(int deferring);bool isEventDriven() const;
		void setEventDriven(bool eventDriven);
		int getFrequency() const;
		void setFrequency(int frequency);
		shared_ptr<map<string,string>> getParameters();
		void setParameters(const shared_ptr<map<string,string>> parameters);
		const Behaviour* getBehaviour() const;
		void setBehaviour(const Behaviour* behaviour);

	private:
		bool eventDriven;
		int frequency;
		int deferring;
		shared_ptr<map<string,string>> parameters;
		const Behaviour* behaviour;
	};

} /* namespace Alica */

#endif /* BEHAVIOURCONFIGURATION_H_ */
