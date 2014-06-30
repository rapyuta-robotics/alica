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

namespace alica
{

	class BehaviourConfiguration;
	class BasicBehaviour;

	class Behaviour : public AlicaElement
	{
	public:
		Behaviour();
		Behaviour(string name);
		virtual ~Behaviour();

		string toString();

		list<BehaviourConfiguration*>& getConfigurations();
		void setConfigurations(const list<BehaviourConfiguration*>& configurations);
		const string& getFileName() const;
		void setFileName(const string& fileName);

	private:
		list<BehaviourConfiguration*> configurations;
		string fileName;
	};

} /* namespace Alica */

#endif /* BEHAVIOUR_H_ */
