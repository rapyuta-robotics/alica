#include "test_alica.h"

#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/Behaviour/MidFieldStandard.h>
#include <alica_tests/CounterClass.h>
#include <alica_tests/TestLogger.h>

#include <alica/test/Util.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>

#include <gtest/gtest.h>
#include <string>

namespace alica
{
namespace
{

class AlicaLoggerTest : public AlicaTestFixtureWithLogger
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SimpleTestPlan"; }
    bool stepEngine() const override { return false; }
};

/**
 * Tests whether logs in the engine are correct.
 */
TEST_F(AlicaLoggerTest, testLogging)
{
    ASSERT_NO_SIGNAL
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::seconds(1));

    alicaTests::TestLogger& logger = getLogger();

    ASSERT_NE(logger.logs.at(0).second.find("MM: Config key 'PlanDir' maps to"), std::string::npos);
    ASSERT_EQ(logger.logs.at(0).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(1).second.find("MM: Config key 'RoleDir' maps to"), std::string::npos);
    ASSERT_EQ(logger.logs.at(1).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(2).second.find("MM: Config key 'TaskDir' maps to"), std::string::npos);
    ASSERT_EQ(logger.logs.at(2).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(3).second.find("MM: fileToParse:"), std::string::npos);
    ASSERT_EQ(logger.logs.at(3).first, Verbosity::DEBUG);

    ASSERT_NE(logger.logs.at(7).second.find("MM: #Behaviour"), std::string::npos);
    ASSERT_EQ(logger.logs.at(7).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(8).second.find("MM: #Behaviour: MidFieldStandard"), std::string::npos);
    ASSERT_EQ(logger.logs.at(8).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(9).second.find("MM: Parsed the following role set:"), std::string::npos);
    ASSERT_EQ(logger.logs.at(9).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(10).second.find("[TeamManager] Own ID is"), std::string::npos);
    ASSERT_EQ(logger.logs.at(10).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(11).second.find("PB: Engine loop time is"), std::string::npos);
    ASSERT_EQ(logger.logs.at(11).first, Verbosity::INFO);

    ASSERT_NE(logger.logs.at(12).second.find("AE: Constructor finished!"), std::string::npos);
    ASSERT_EQ(logger.logs.at(12).first, Verbosity::DEBUG);

    ASSERT_NE(logger.logs.at(13).second.find("RA: Setting Role"), std::string::npos);
    ASSERT_EQ(logger.logs.at(13).first, Verbosity::DEBUG);

    ASSERT_NE(logger.logs.at(14).second.find("TM: Announcing presence"), std::string::npos);
    ASSERT_EQ(logger.logs.at(14).first, Verbosity::DEBUG);

    ASSERT_NE(logger.logs.at(15).second.find("AE: Engine started!"), std::string::npos);
    ASSERT_EQ(logger.logs.at(15).first, Verbosity::DEBUG);
}
} // namespace
} // namespace alica
