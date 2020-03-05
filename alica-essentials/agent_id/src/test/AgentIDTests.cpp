#include <essentials/AgentID.h>
#include <essentials/AgentIDFactory.h>
#include <essentials/AgentIDManager.h>
#include <essentials/BroadcastID.h>

#include <gtest/gtest.h>
#include <vector>

TEST(AgentID, ConstructorCopiesBytes) {
    std::vector<uint8_t> bytes1;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
    }
    essentials::AgentID* id1 = new essentials::AgentID(bytes1.data(), bytes1.size());

    ASSERT_FALSE(id1->getRaw() == bytes1.data());

    delete id1;
}

TEST(AgentID, ConstructionOfHugeID) {
    std::vector<uint8_t> bytes1;
    int size = 100;
    for (int i = 0; i < size; i++) {
        bytes1.push_back(i);
    }
    essentials::AgentID* id1 = new essentials::AgentID(bytes1.data(), bytes1.size());

    ASSERT_EQ(id1->getSize(), size);

    delete id1;
}

TEST(AgentID, ToByteVectorReturnsCopy) {
    std::vector<uint8_t> bytes1;
    int size = 100;
    for (int i = 0; i < size; i++) {
        bytes1.push_back(i);
    }
    essentials::AgentID* id1 = new essentials::AgentID(bytes1.data(), bytes1.size());
    auto byteVector = id1->toByteVector();

    ASSERT_NE(byteVector.data(), id1->getRaw());

    delete id1;
}

TEST(AgentID, HashEqualForSameIDs) {
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    essentials::AgentID* id1 = new essentials::AgentID(bytes1.data(), bytes1.size());
    essentials::AgentID* id2 = new essentials::AgentID(bytes2.data(), bytes2.size());

    ASSERT_EQ(id1->hash(), id2->hash());

    delete id1;
    delete id2;
}

TEST(AgentID, EqualWithSameID) {
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    essentials::AgentID* id1 = new essentials::AgentID(bytes1.data(), bytes1.size());
    essentials::AgentID* id2 = new essentials::AgentID(bytes2.data(), bytes2.size());

    ASSERT_TRUE(*id1 == *id2);

    delete id1;
    delete id2;
}

TEST(AgentID, NotEqualWithDifferentID) {
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    bytes2.push_back(2);
    essentials::AgentID* id1 = new essentials::AgentID(bytes1.data(), bytes1.size());
    essentials::AgentID* id2 = new essentials::AgentID(bytes2.data(), bytes2.size());

    ASSERT_FALSE(*id1 == *id2);

    delete id1;
    delete id2;
}

TEST(BroadCastID, NotEqualWithNormalID) {
    std::vector<uint8_t> bytes1;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
    }
    essentials::AgentID* normalID = new essentials::AgentID(bytes1.data(), bytes1.size());
    std::vector<uint8_t> bytesBroadcast;
    essentials::AgentID* broadcastID = new essentials::BroadcastID(bytesBroadcast.data(), bytesBroadcast.size());

    ASSERT_FALSE(*broadcastID == *normalID);

    delete normalID;
    delete broadcastID;
}

TEST(BroadCastID, EqualWithBroadcastID) {
    std::vector<uint8_t> bytesBroadcast1;
    bytesBroadcast1.push_back(1);
    essentials::AgentID* broadcastID1 =
            new essentials::BroadcastID(bytesBroadcast1.data(), bytesBroadcast1.size());
    std::vector<uint8_t> bytesBroadcast2;
    essentials::AgentID* broadcastID2 =
            new essentials::BroadcastID(bytesBroadcast2.data(), bytesBroadcast2.size());

    ASSERT_TRUE(*broadcastID1 == *broadcastID2);

    delete broadcastID1;
    delete broadcastID2;
}

TEST(AgentIDFactory, GenerateIDsOfVariousLength) {
    essentials::AgentIDFactory factory;

    auto id1 = factory.generateID(1);
    ASSERT_EQ(id1->getSize(), 1);

    auto id4 = factory.generateID(4);
    ASSERT_EQ(id4->getSize(), 4);

    auto id15 = factory.generateID(15);
    ASSERT_EQ(id15->getSize(), 15);

    auto id18 = factory.generateID(18);
    ASSERT_EQ(id18->getSize(), 18);
}

TEST(AgentIDFactory, DuplicateIDs) {
    essentials::AgentIDFactory factory;
    auto id18 = factory.generateID(18);
    auto id18Copy = factory.create(id18->toByteVector());
    ASSERT_TRUE(*id18 == *id18Copy);
}

TEST(AgentIDManager, CreateIDsFromIntegralTypes) {
    essentials::AgentIDFactory* factory = new essentials::AgentIDFactory();
    essentials::AgentIDManager idManager(factory);
    int idInt = 5;
    auto intId5 = idManager.getID<int>(idInt);

    std::vector<uint8_t> idBytes;
    idBytes.push_back(5);
    idBytes.push_back(0);
    idBytes.push_back(0);
    idBytes.push_back(0);
    essentials::AgentID* referenceId5 = new essentials::AgentID(idBytes.data(), idBytes.size());

    ASSERT_TRUE(*intId5 == *referenceId5);
}

TEST(AgentIDManager, GuarenteeSingleEntities) {
    essentials::AgentIDFactory* factory = new essentials::AgentIDFactory();
    essentials::AgentIDManager idManager(factory);
    int idInt = 5;
    auto intId1 = idManager.getID<int>(idInt);
    auto intId2 = idManager.getID<int>(idInt);

    ASSERT_EQ(intId1, intId2);
}

TEST(AgentIDManager, GenerateIDsOfVariousLength) {
    essentials::AgentIDFactory* factory = new essentials::AgentIDFactory();
    essentials::AgentIDManager idManager(factory);

    auto id1 = idManager.generateID(1);
    ASSERT_EQ(id1->getSize(), 1);

    auto id4 = idManager.generateID(4);
    ASSERT_EQ(id4->getSize(), 4);

    auto id15 = idManager.generateID(15);
    ASSERT_EQ(id15->getSize(), 15);

    auto id18 = idManager.generateID(18);
    ASSERT_EQ(id18->getSize(), 18);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
