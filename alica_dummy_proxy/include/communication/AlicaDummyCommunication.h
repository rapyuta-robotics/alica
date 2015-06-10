/*
 * AlicaDummyCommunication.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#ifndef ALICADUMMYCOMMUNICATION_H_
#define ALICADUMMYCOMMUNICATION_H_

#include <engine/IAlicaCommunication.h>

namespace alica_dummy_proxy {

	class AlicaDummyCommunication : public alica::IAlicaCommunication
	{
		public:
			AlicaDummyCommunication(alica::AlicaEngine* ae);
			virtual ~AlicaDummyCommunication();

			virtual void tick();

			virtual void sendAllocationAuthority(alica::AllocationAuthorityInfo& aai);
			virtual void sendAlicaEngineInfo(alica::AlicaEngineInfo& bi);
			virtual void sendPlanTreeInfo(alica::PlanTreeInfo& pti);
			virtual void sendRoleSwitch(alica::RoleSwitch& rs);
			virtual void sendSyncReady(alica::SyncReady& sr);
			virtual void sendSyncTalk(alica::SyncTalk& st);
			virtual void sendSolverResult(alica::SolverResult& sr);


			virtual void startCommunication();
			virtual void stopCommunication();
		};

} /* namespace alica */

#endif /* ALICA_ALICA_DUMMY_PROXY_INCLUDE_COMMUNICATION_ALICADUMMYCOMMUNICATION_H_ */
