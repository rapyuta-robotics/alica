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
#include "ITrigger.h"

using namespace std;

namespace supplementary
{
	class EventTrigger : public virtual ITrigger
	{
	public:
		EventTrigger();
		virtual ~EventTrigger();
		void run(bool notifyAll = true);
	};
}

#endif /* SUPPLEMENTARY_EVENT_HANDLING_SRC_EVENTTRIGGER_H_ */
