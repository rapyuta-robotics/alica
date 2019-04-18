#include <communication/AlicaCapnzeroCommunication.h>
#include <engine/AlicaEngine.h>
#include <essentials/AgentIDFactory.h>
#include <engine/containers/AllocationAuthorityInfo.h>
#include <engine/containers/EntryPointRobots.h>
#include <essentials/AgentID.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <thread>


int main(int argc, char* argv[])
{
    alica::AlicaEngine *ae = NULL;
    alicaCapnzeroProxy::AlicaCapnzeroCommunication *com = new alicaCapnzeroProxy::AlicaCapnzeroCommunication(ae);
    com->startCommunication();
    com->sendLogMessage(1, "Test");
    alica::AllocationAuthorityInfo authorityInfo;
    authorityInfo.senderID = essentials::AgentIDFactory().generateID(16);
    authorityInfo.authority = essentials::AgentIDFactory().generateID(16);
    authorityInfo.planId = 12345;
    authorityInfo.planType = 6789;
    authorityInfo.parentState = 456;
    std::cout << "\033[91mSending testdata:\033[0m\n";
    com->sendAllocationAuthority(authorityInfo);
    std::cout << "\033[91mSending testdata:\033[0m\n";
    com->sendAllocationAuthority(authorityInfo);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}