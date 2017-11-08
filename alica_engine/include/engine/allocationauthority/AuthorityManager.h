#pragma once

//#define AM_DEBUG

#include "../containers/AllocationAuthorityInfo.h"
#include "../IAlicaCommunication.h"
#include "../AlicaEngine.h"
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
		void handleIncomingAuthorityMessage(std::shared_ptr<AllocationAuthorityInfo> aai);
		void tick(std::shared_ptr<RunningPlan> p);
		void sendAllocation(std::shared_ptr<RunningPlan> p);

	protected:
		vector<std::shared_ptr<AllocationAuthorityInfo>> queue;
		const AlicaEngine* engine;
		const supplementary::IAgentID* localAgentID;
		mutex mu;
		void processPlan(std::shared_ptr<RunningPlan> p);
		bool authorityMatchesPlan(std::shared_ptr<AllocationAuthorityInfo> aai, std::shared_ptr<RunningPlan> p);
	};
} /* namespace alica */
