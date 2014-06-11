/*
 * Condition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef CONDITION_H_
#define CONDITION_H_

using namespace std;

#include <string>
#include <list>

#include "AlicaElement.h"

namespace alica
{
	class Variable;
	class Quantifier;
	class AbstractPlan;

	class Condition : public AlicaElement
	{
	public:
		Condition();
		Condition(long id);
		virtual ~Condition();

		const string& getConditionString() const;
		void setConditionString(const string& conditionString);
		list<Quantifier*>& getQuantifiers() ;
		const list<Variable*>& getVariables() const;
		void setVariables(const list<Variable*>& variables);
		AbstractPlan* getAbstractPlan() const;
		void setAbstractPlan(AbstractPlan* abstractPlan);

	private:
		void setQuantifiers(const list<Quantifier*>& quantifiers);
	protected:
		string conditionString;
		list<Variable*> variables;
		list<Quantifier*> quantifiers;
		AbstractPlan* abstractPlan;

	};

} /* namespace Alica */

#endif /* CONDITION_H_ */
