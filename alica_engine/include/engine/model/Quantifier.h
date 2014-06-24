/*
 * Quantifier.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef QUANTIFIER_H_
#define QUANTIFIER_H_

using namespace std;

#include <list>
#include <string>
#include <typeinfo>

#include "AlicaElement.h"

namespace alica
{
	class State;
	class EntryPoint;
	class Plan;

	class Quantifier : public AlicaElement
	{
	public:
		Quantifier(long id = 0);
		virtual ~Quantifier();
		list<string>& getDomainIdentifiers() ;
		void setDomainIdentifiers(const list<string>& domainIdentifiers);
		bool isScopeIsEntryPoint() const;
		bool isScopeIsPlan() const;
		bool isScopeIsState() const;
		State* getScopedState();
		EntryPoint* getScopedEntryPoint();
		Plan* getScopedPlan();
		void setScope(AlicaElement* ae);
		AlicaElement* getScope();

	private:
		list<string> domainIdentifiers;
		void setScopeIsEntryPoint(bool scopeIsEntryPoint);
		void setScopeIsPlan(bool scopeIsPlan);
		void setScopeIsState(bool scopeIsState);

	protected:
		bool scopeIsEntryPoint;
		bool scopeIsPlan;
		bool scopeIsState;
		EntryPoint* entryPoint;
		State* state;
		Plan* plan;
	};

} /* namespace Alica */

#endif /* QUANTIFIER_H_ */
