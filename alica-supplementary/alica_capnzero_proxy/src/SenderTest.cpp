#include <alica_capnzero_proxy/Communication.h>
#include <engine/AlicaEngine.h>
#include <engine/containers/AllocationAuthorityInfo.h>
#include <engine/containers/EntryPointRobots.h>
#include <essentials/Identifier.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <thread>
#include <engine/containers/AlicaEngineInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/containers/RoleSwitch.h>
#include <engine/containers/SyncReady.h>
#include <engine/containers/SyncTalk.h>


int main(int argc, char* argv[])
{
    alica::AlicaEngine *ae = NULL;
    alica_capnzero_proxy::Communication *com = new alica_capnzero_proxy::Communication(ae);
    com->startCommunication();
    com->sendLogMessage(1, "Test");
    essentials::IdentifierConstPtr own = essentials::IDManager().generateID(16);
    alica::AllocationAuthorityInfo authorityInfo;
    authorityInfo.senderID = own;
    authorityInfo.authority = own;
    authorityInfo.planId = 11;
    authorityInfo.planType = 11;
    authorityInfo.parentState = 11;

    alica::AlicaEngineInfo aei;
    aei.senderID = own;
    aei.currentTask = "test";
    aei.currentState = "test";
    aei.currentRole = "test";
    aei.currentPlan = "test";
    aei.masterPlan = "test";

    alica::PlanTreeInfo pti;
    pti.senderID = own;
    pti.stateIDs.push_back(12);
    pti.stateIDs.push_back(12);
    pti.succeededEPs.push_back(12);
    pti.succeededEPs.push_back(12);

    alica::RoleSwitch rs;
    rs.roleID = 13;
    rs.senderID = own;

    alica::SyncReady sr;
    sr.senderID = own;
    sr.synchronisationID = 14;

    alica::SyncTalk st;
    st.senderID = own;
    st.syncData.push_back(alica::SyncData());
    st.syncData.at(0).robotID = own;
    st.syncData.at(0).transitionID = 15;
    st.syncData.at(0).conditionHolds = true;
    st.syncData.at(0).ack = false;

    std::cout << "\033[91mSending testdata:\033[0m\n";
    com->sendAllocationAuthority(authorityInfo);
    com->sendAlicaEngineInfo(aei);
    com->sendPlanTreeInfo(pti);
    com->sendRoleSwitch(rs);
    com->sendSyncReady(sr);
    com->sendSyncTalk(st);


    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}