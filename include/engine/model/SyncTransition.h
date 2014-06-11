/*
 * SyncTransition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef SYNCTRANSITION_H_
#define SYNCTRANSITION_H_

using namespace std;

#include <list>
#include <string>
#include <sstream>

#include "AlicaElement.h"

namespace alica
{

	class Plan;
	class Transition;

	class SyncTransition : public AlicaElement
	{
	public:
		SyncTransition();
		virtual ~SyncTransition();

		string toString();

		bool isFailOnSyncTimeOut() const;
		void setFailOnSyncTimeOut(bool failOnSyncTimeOut);
		unsigned long getSyncTimeOut() const;
		void setSyncTimeOut(unsigned long syncTimeOut);
		unsigned long getTalkTimeOut() const;
		void setTalkTimeOut(unsigned long talkTimeOut);
		const Plan* getPlan() const;
		void setPlan(Plan* plan);
		const list<Transition*>& getInSync() const;
		void setInSync(const list<Transition*>& inSync);

	private:
		unsigned long talkTimeOut ;
		unsigned long syncTimeOut;
		bool failOnSyncTimeOut;
		Plan* plan;
		list<Transition*> inSync;
	};

} /* namespace Alica */

#endif /* SYNCTRANSITION_H_ */
