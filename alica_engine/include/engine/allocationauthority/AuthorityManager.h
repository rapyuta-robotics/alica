/*
 * AuthorityManager.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef AUTHORITYMANAGER_H_
#define AUTHORITYMANAGER_H_

#include "../containers/AllocationAuthorityInfo.h"
#include "../IAlicaCommunication.h"
#include "../AlicaEngine.h"
#include "../ITeamObserver.h"
#include "../RunningPlan.h"

#include <memory>

using namespace std;

namespace alica
{
	class IAlicaCommunication;

	class AuthorityManager
	{
	public:
		AuthorityManager();
		virtual ~AuthorityManager();
		void init();
		void close();
		void handleIncomingAuthorityMessage(shared_ptr<AllocationAuthorityInfo> aai);
		void tick(RunningPlan* p);
		void sendAllocation(RunningPlan* p);

	protected:
		IAlicaCommunication* authorityPub;
		list<shared_ptr<AllocationAuthorityInfo>> queue;
		AlicaEngine* ae;
		int ownID;
		mutex mu;
		void processPlan(RunningPlan* p);
		bool authorityMatchesPlan(AllocationAuthorityInfo aai, RunningPlan* p);


	};
}
#endif /* AUTHORITYMANAGER_H_ */
