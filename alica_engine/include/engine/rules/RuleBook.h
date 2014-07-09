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
	class ITeamObserver;
	class ISyncModul;
	class IPlanSelector;
	class Logger;

	class RuleBook
	{
	public:
		RuleBook();
		virtual ~RuleBook();
		bool isChangeOccured() const;
		void setChangeOccured(bool changeOccured);

		bool changeOccured;
	protected:
		ITeamObserver* to;
		ISyncModul* sm;
		int maxConsecutiveChanges;
		IPlanSelector* ps;
		Logger* log;


	};

#endif /* RULEBOOK_H_ */
}
