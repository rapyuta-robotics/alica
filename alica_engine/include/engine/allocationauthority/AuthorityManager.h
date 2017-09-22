#pragma once

//#define AM_DEBUG

#include "../containers/AllocationAuthorityInfo.h"
#include "../IAlicaCommunication.h"
#include "../AlicaEngine.h"
#include "../ITeamObserver.h"
#include "../RunningPlan.h"

#include <memory>
#include <vector>

namespace alica
{
	class IAlicaCommunication;

	/**
	 * Manages communication wrt. conflict resolution.
	 */
	class AuthorityManager
	{
	public:
		AuthorityManager(AlicaEngine* ae);
		virtual ~AuthorityManager();
		void init();
		void close();
		void handleIncomingAuthorityMessage(shared_ptr<AllocationAuthorityInfo> aai);
		void tick(shared_ptr<RunningPlan> p);
		void sendAllocation(shared_ptr<RunningPlan> p);

	protected:
		vector<shared_ptr<AllocationAuthorityInfo>> queue;
		const AlicaEngine* engine;
		const alica::IRobotID* localAgentID;
		mutex mu;
		void processPlan(shared_ptr<RunningPlan> p);
		bool authorityMatchesPlan(shared_ptr<AllocationAuthorityInfo> aai, shared_ptr<RunningPlan> p);
	};
} /* namespace alica */
