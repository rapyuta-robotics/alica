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
#include <memory>

#include "AlicaElement.h"

namespace alica
{
	class Variable;
	class Quantifier;
	class AbstractPlan;
	class BasicCondition;
	class RunningPlan;

	/**
	 * A condition encapsulates expressions and constraint specific to a AlicaElement, e.g., a Transition, or a Plan.
	 */
	class Condition : public AlicaElement
	{
	public:
		Condition();
		Condition(long id);
		virtual ~Condition();

		/**
		 * The delegate type used to attach constraints to plans.
		 */
		//public delegate void GetConstraint(ConstraintDescriptor cd,RunningPlan rp);

		const string& getConditionString() const;
		void setConditionString(const string& conditionString);
		list<Quantifier*>& getQuantifiers() ;
		 list<Variable*>& getVariables() ;
		void setVariables(const list<Variable*>& variables);
		AbstractPlan* getAbstractPlan() const;
		void setAbstractPlan(AbstractPlan* abstractPlan);
		const string& getPlugInName() const;
		void setPlugInName(const string& plugInName);
		bool evaluate(shared_ptr<RunningPlan> rp);
		shared_ptr<BasicCondition> getBasicCondition();
		void setBasicCondition(shared_ptr<BasicCondition> basicCondition);

	private:
		void setQuantifiers(const list<Quantifier*>& quantifiers);
	protected:
		string conditionString;
		/**
		 * The static variables used in the constraint of this condition.
		 */
		list<Variable*> variables;
		/**
		 * The quantifiers used in the constraint of this condition.
		 */
		list<Quantifier*> quantifiers;
		/**
		 * The Abstract Plan in which this condition occurs.
		 */
		AbstractPlan* abstractPlan;
		shared_ptr<BasicCondition> basicCondition;
		string plugInName;

	};

} /* namespace Alica */

#endif /* CONDITION_H_ */
