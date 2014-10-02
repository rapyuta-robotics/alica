/*
 * Parametrisation.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PARAMETRISATION_H_
#define PARAMETRISATION_H_

using namespace std;

#include <string>
#include <sstream>

#include "AlicaElement.h"

namespace alica
{
	class Variable;
	class AbstractPlan;

	class Parametrisation : public AlicaElement
	{
	public:
		Parametrisation();
		virtual ~Parametrisation();

		string ToString();

		AbstractPlan* getSubPlan();
		void setSubPlan(AbstractPlan* subPlan);
		Variable* getSubVar();
		void setSubVar(Variable* subVar);
		Variable* getVar();
		void setVar(Variable* var);

	protected:
		Variable* var;
		Variable* subVar;
		AbstractPlan* subPlan;

	};

} /* namespace Alica */

#endif /* PARAMETRISATION_H_ */
