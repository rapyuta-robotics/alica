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
#include <vector>
#include <list>
#include <memory>

#include "AlicaElement.h"

namespace alica
{
	class Variable;
	class Quantifier;
	class AbstractPlan;
	class BasicCondition;
	class BasicConstraint;
	class RunningPlan;
	class Parameter;
	class ConstraintDescriptor;

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
		void getConstraint(shared_ptr<ConstraintDescriptor> cd, shared_ptr<RunningPlan> rp);

		const string& getConditionString() const;
		void setConditionString(const string& conditionString);
		list<Quantifier*>& getQuantifiers() ;
		vector<Variable*>& getVariables() ;
		void setVariables(const vector<Variable*>& variables);
		AbstractPlan* getAbstractPlan() const;
		void setAbstractPlan(AbstractPlan* abstractPlan);
		const string& getPlugInName() const;
		void setPlugInName(const string& plugInName);
		bool evaluate(shared_ptr<RunningPlan> rp);
		shared_ptr<BasicCondition> getBasicCondition();
		void setBasicCondition(shared_ptr<BasicCondition> basicCondition);
		list<Parameter*>& getParameters();
		void setParameters(list<Parameter*> parameters);

		void setBasicConstraint(shared_ptr<BasicConstraint> basicConstraint);

	private:
		void setQuantifiers(const list<Quantifier*>& quantifiers);
	protected:
		string conditionString;
		/**
		 * The static variables used in the constraint of this condition.
		 */
		vector<Variable*> variables;
		/**
		 * The quantifiers used in the constraint of this condition.
		 */
		list<Quantifier*> quantifiers;
		/**
		 * The Abstract Plan in which this condition occurs.
		 */
		AbstractPlan* abstractPlan;
		list<Parameter*> parameters;
		shared_ptr<BasicCondition> basicCondition;
		shared_ptr<BasicConstraint> basicConstraint;
		string plugInName;

	};

} /* namespace Alica */

#endif /* CONDITION_H_ */
