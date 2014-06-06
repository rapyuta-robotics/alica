/*
 * Behaviour.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOUR_H_
#define BEHAVIOUR_H_

using namespace std;

#include <list>
#include <string>

#include "AlicaElement.h"
#include "../BasicBehaviour.h"
#include "BehaviourConfiguration.h"

namespace alica
{

	class BehaviourConfiguration;
	class Behaviour : public AlicaElement
	{
	public:
		Behaviour();
		Behaviour(string name);
		virtual ~Behaviour();

		string toString();

		const list<BehaviourConfiguration*>& getConfigurations() const;
		void setConfigurations(const list<BehaviourConfiguration*>& configurations);
		const string& getFileName() const;
		void setFileName(const string& fileName);
		const BasicBehaviour& getImplementation() const;
		void setImplementation(const BasicBehaviour& implementation);

	private:
		list<BehaviourConfiguration*> configurations;
		string fileName;
		BasicBehaviour implementation;
	};

} /* namespace Alica */

#endif /* BEHAVIOUR_H_ */
