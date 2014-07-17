/*
 * ISyncModul.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef ISYNCMODUL_H_
#define ISYNCMODUL_H_


namespace alica
{
	class Transition;

	class ISyncModul
	{
	public:
		virtual ~ISyncModul() {}
		virtual void init() = 0;
		virtual void close() = 0;
		virtual void tick() = 0;
		virtual void setSynchronisation(Transition trans, bool holds) = 0;
		virtual bool followSyncTransition(Transition trans) = 0;
	};
}
#endif /* ISYNCMODUL_H_ */
