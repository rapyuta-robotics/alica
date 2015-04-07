/*
 * ITrigger.h
 *
 *  Created on: Apr 7, 2015
 *      Author: Stefan Jakob
 */

#ifndef SUPPLEMENTARY_EVENT_HANDLING_SRC_ITRIGGER_H_
#define SUPPLEMENTARY_EVENT_HANDLING_SRC_ITRIGGER_H_

#include <vector>
#include <mutex>
#include <condition_variable>

using namespace std;

namespace supplementary
{

	class ITrigger
	{
	public:
		virtual ~ITrigger() {}
		virtual void registerCV(condition_variable* condVar) = 0;
		virtual void run(bool notifyAll = false) = 0;
		mutex cv_mtx;
		condition_variable cv;
		vector<condition_variable*> registeredCVs;
	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_EVENT_HANDLING_SRC_ITRIGGER_H_ */
