/*
 * SyncModul.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Paul Panin
 */

#ifndef SYNCMODUL_H_
#define SYNCMODUL_H_

namespace alica
{

	class SyncModul : public ISyncModul
	{
	public:
		SyncModul();
		virtual ~SyncModul();
		virtual void init();
		virtual void close();
		virtual void tick();
		virtual void setSynchronisation(Transition* trans, bool holds);
		virtual bool followSyncTransition(Transition* trans);
	};

} /* namespace supplementary */

#endif /* SYNCMODUL_H_ */
