/*
 * RuleBook.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef RULEBOOK_H_
#define RULEBOOK_H_

namespace alica
{
	class ISyncModul;
	class IPlanSelector;
	class Logger;
	class ITeamObserver;
	class RunningPlan;
	class Plan;

	class RuleBook
	{
	public:
		RuleBook();
		virtual ~RuleBook();
		bool isChangeOccured() const;
		void setChangeOccured(bool changeOccured);

		bool changeOccured;

		RunningPlan* initialisationRule(Plan* masterPlan);
	protected:
		ITeamObserver* to;
		ISyncModul* sm;
		int maxConsecutiveChanges;
		IPlanSelector* ps;
		Logger* log;


	};

#endif /* RULEBOOK_H_ */
}
