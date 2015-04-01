/*
 * EventTrigger.h
 *
 *  Created on: Apr 1, 2015
 *      Author: Stefan Jakob
 */

#ifndef SUPPLEMENTARY_EVENT_HANDLING_SRC_EVENTTRIGGER_H_
#define SUPPLEMENTARY_EVENT_HANDLING_SRC_EVENTTRIGGER_H_

#include <vector>
#include <mutex>
#include <condition_variable>

using namespace std;

namespace supplementary
{
	class EventTrigger
	{
	public:
		EventTrigger();
		virtual ~EventTrigger();
		void registerCV(condition_variable* condVar);
		void trigger(bool notifyAll = false);

	private:
		mutex cv_mtx;
		condition_variable cv;
		vector<condition_variable*> registeredCVs;
	};
}

#endif /* SUPPLEMENTARY_EVENT_HANDLING_SRC_EVENTTRIGGER_H_ */
