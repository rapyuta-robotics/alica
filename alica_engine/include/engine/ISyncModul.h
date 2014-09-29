/*
 * ISyncModul.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef ISYNCMODUL_H_
#define ISYNCMODUL_H_

#include <memory>

using namespace std;

namespace alica
{
	class Transition;
	struct SyncTalk;
	struct SyncReady;

	/**
	 * Handles synchronizations between agents, that is tightly synchronized transitions.
	 */
	class ISyncModul
	{
	public:
		virtual ~ISyncModul()
		{
		}
		/**
		 * Starts this module
		 */
		virtual void init() = 0;
		/**
		 * Closes the module for good.
		 */
		virtual void close() = 0;
		/**
		 * Regularly called by the PlanBase.
		 */
		virtual void tick() = 0;
		/**
		 * Called by the RuleBook to indicate that a synchronization may happen.
		 * @param trans The Transition belonging to a synchronized set
		 * @param holds A bool indicating if the condition guarding trans holds.
		 */
		virtual void setSynchronisation(Transition* trans, bool holds) = 0;
		/**
		 * Indicates that a synchronized transition is to be followed now.
		 * If true is returned, the transition must be followed immediately, as the synchronization is completed with the call of this method.
		 * @param trans A Transition
		 * @return A bool
		 */
		virtual bool followSyncTransition(Transition* trans) = 0;
		virtual void onSyncTalk(shared_ptr<SyncTalk> st) = 0;
		virtual void onSyncReady(shared_ptr<SyncReady> sr) = 0;
	};
}
#endif /* ISYNCMODUL_H_ */
