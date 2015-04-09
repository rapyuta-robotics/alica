/*
 * ITrigger.h
 *
 *  Created on: Apr 7, 2015
 *      Author: Stefan Jakob
 */

#ifndef SUPPLEMENTARY_EVENT_HANDLING_SRC_ITRIGGER_H_
#define SUPPLEMENTARY_EVENT_HANDLING_SRC_ITRIGGER_H_

#include <map>
#include <mutex>
#include <condition_variable>
#include <iostream>

using namespace std;

namespace supplementary
{

	class ITrigger
	{
	public:
		virtual ~ITrigger()
		{
		}
		void registerCV(condition_variable* condVar)
		{
			lock_guard<mutex> lock(cvVec_mtx);
			registeredCVs[condVar] = false;
		}
		virtual void run(bool notifyAll = true) = 0;
		bool isNotifyCalled(condition_variable* cv)
		{
			return registeredCVs.find(cv) != registeredCVs.end() && registeredCVs[cv];
		}
		void setNotifyCalled(bool called, condition_variable* cv)
		{
			if (registeredCVs.find(cv) != registeredCVs.end())
			{
				registeredCVs[cv] = called;
			}
		}

	protected:
		void notifyAll(bool notifyAll)
		{
			for (auto& pair : registeredCVs)
			{
				pair.second = true;
				if (notifyAll)
				{
					pair.first->notify_all();
				}
				else
				{
					pair.first->notify_one();
				}

			}
		}
		mutex cv_mtx;
		mutex cvVec_mtx;
		map<condition_variable*, bool> registeredCVs;
	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_EVENT_HANDLING_SRC_ITRIGGER_H_ */
