#include <supplementary/AgentID.h>
#include <supplementary/AgentIDFactory.h>
#include <supplementary/AgentIDManager.h>
#include <supplementary/BroadcastID.h>

#include <gtest/gtest.h>
#include <vector>

TEST(AgentID, ConstructorCopiesBytes)
{
    std::vector<uint8_t> bytes1;
    for (int i = 0; i < 20; i++)
    {
        bytes1.push_back(i);
    }
    supplementary::AgentID *id1 = new supplementary::AgentID(bytes1.data(), bytes1.size());

    ASSERT_FALSE(id1->getRaw() == bytes1.data());

    delete id1;
}

TEST(AgentID, ConstructionOfHugeID)
{
    std::vector<uint8_t> bytes1;
    int size = 100;
    for (int i = 0; i < size; i++)
    {
        bytes1.push_back(i);
    }
    supplementary::AgentID *id1 = new supplementary::AgentID(bytes1.data(), bytes1.size());

    ASSERT_EQ(id1->getSize(), size);

    delete id1;
}

TEST(AgentID, ToByteVectorReturnsCopy)
{
    std::vector<uint8_t> bytes1;
    int size = 100;
    for (int i = 0; i < size; i++)
    {
        bytes1.push_back(i);
    }
    supplementary::AgentID *id1 = new supplementary::AgentID(bytes1.data(), bytes1.size());
    auto byteVector = id1->toByteVector();

    ASSERT_NE(byteVector.data(), id1->getRaw());

    delete id1;
}

TEST(AgentID, HashEqualForSameIDs)
{
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++)
    {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    supplementary::AgentID *id1 = new supplementary::AgentID(bytes1.data(), bytes1.size());
    supplementary::AgentID *id2 = new supplementary::AgentID(bytes2.data(), bytes2.size());

    ASSERT_EQ(id1->hash(), id2->hash());

    delete id1;
    delete id2;
}

TEST(AgentID, EqualWithSameID)
{
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++)
    {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    supplementary::AgentID *id1 = new supplementary::AgentID(bytes1.data(), bytes1.size());
    supplementary::AgentID *id2 = new supplementary::AgentID(bytes2.data(), bytes2.size());

    ASSERT_TRUE(*id1 == *id2);

    delete id1;
    delete id2;
}

TEST(AgentID, NotEqualWithDifferentID)
{
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++)
    {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    bytes2.push_back(2);
    supplementary::AgentID *id1 = new supplementary::AgentID(bytes1.data(), bytes1.size());
    supplementary::AgentID *id2 = new supplementary::AgentID(bytes2.data(), bytes2.size());

    ASSERT_FALSE(*id1 == *id2);

    delete id1;
    delete id2;
}

TEST(BroadCastID, NotEqualWithNormalID)
{
    std::vector<uint8_t> bytes1;
    for (int i = 0; i < 20; i++)
    {
        bytes1.push_back(i);
    }
    supplementary::AgentID *normalID = new supplementary::AgentID(bytes1.data(), bytes1.size());
    std::vector<uint8_t> bytesBroadcast;
    supplementary::AgentID *broadcastID = new supplementary::BroadcastID(bytesBroadcast.data(), bytesBroadcast.size());

    ASSERT_FALSE(*broadcastID == *normalID);

    delete normalID;
    delete broadcastID;
}

TEST(BroadCastID, EqualWithBroadcastID)
{
    std::vector<uint8_t> bytesBroadcast1;
    bytesBroadcast1.push_back(1);
    supplementary::AgentID *broadcastID1 =
        new supplementary::BroadcastID(bytesBroadcast1.data(), bytesBroadcast1.size());
    std::vector<uint8_t> bytesBroadcast2;
    supplementary::AgentID *broadcastID2 =
        new supplementary::BroadcastID(bytesBroadcast2.data(), bytesBroadcast2.size());

    ASSERT_TRUE(*broadcastID1 == *broadcastID2);

    delete broadcastID1;
    delete broadcastID2;
}

TEST(AgentIDFactory, GenerateIDsOfVariousLength)
{
    supplementary::AgentIDFactory factory;

    auto id1 = factory.generateID(1);
    ASSERT_EQ(id1->getSize(), 1);

    auto id4 = factory.generateID(4);
    ASSERT_EQ(id4->getSize(), 4);

    auto id15 = factory.generateID(15);
    ASSERT_EQ(id15->getSize(), 15);

    auto id18 = factory.generateID(18);
    ASSERT_EQ(id18->getSize(), 18);
}

TEST(AgentIDFactory, DuplicateIDs)
{
    supplementary::AgentIDFactory factory;
    auto id18 = factory.generateID(18);
    auto id18Copy = factory.create(id18->toByteVector());
    ASSERT_TRUE(*id18 == *id18Copy);
}

TEST(AgentIDManager, CreateIDsFromIntegralTypes)
{
    supplementary::AgentIDFactory *factory = new supplementary::AgentIDFactory();
    supplementary::AgentIDManager idManager(factory);
    int idInt = 5;
    auto intId5 = idManager.getID<int>(idInt);

    std::vector<uint8_t> idBytes;
    idBytes.push_back(5);
    idBytes.push_back(0);
    idBytes.push_back(0);
    idBytes.push_back(0);
    supplementary::AgentID *referenceId5 = new supplementary::AgentID(idBytes.data(), idBytes.size());

    ASSERT_TRUE(*intId5 == *referenceId5);
}

TEST(AgentIDManager, GuarenteeSingleEntities)
{
    supplementary::AgentIDFactory *factory = new supplementary::AgentIDFactory();
    supplementary::AgentIDManager idManager(factory);
    int idInt = 5;
    auto intId1 = idManager.getID<int>(idInt);
    auto intId2 = idManager.getID<int>(idInt);

    ASSERT_EQ(intId1, intId2);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
