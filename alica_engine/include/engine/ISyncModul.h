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

	class ISyncModul
	{
	public:
		virtual ~ISyncModul()
		{
		}
		virtual void init() = 0;
		virtual void close() = 0;
		virtual void tick() = 0;
		virtual void setSynchronisation(Transition* trans, bool holds) = 0;
		virtual bool followSyncTransition(Transition* trans) = 0;
		virtual void onSyncTalk(shared_ptr<SyncTalk> st) = 0;
		virtual void onSyncReady(shared_ptr<SyncReady> sr) = 0;
	};
}
#endif /* ISYNCMODUL_H_ */
