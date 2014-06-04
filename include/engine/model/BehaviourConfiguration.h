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
#include "Behaviour.h"

namespace alica
{
	class Behaviour;
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
		const map<string, string>& getParameters() const;
		void setParameters(const map<string, string>& parameters);
		const Behaviour* getBehaviour() const;
		void setBehaviour(const Behaviour* behaviour);

	private:
		bool eventDriven;
		int frequency;
		int deferring;
		map<string,string> parameters;
		const Behaviour* behaviour;
	};

} /* namespace Alica */

#endif /* BEHAVIOURCONFIGURATION_H_ */
