#include <essentials/Identifier.h>
#include <essentials/IDManager.h>
#include <essentials/WildcardID.h>

#include <gtest/gtest.h>
#include <vector>

TEST(Identifier, ConstructorCopiesBytes)
{
    std::vector<uint8_t> bytes1;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
    }
    essentials::Identifier* id1 = new essentials::Identifier(bytes1.data(), bytes1.size());

    ASSERT_FALSE(id1->getRaw() == bytes1.data());

    delete id1;
}

TEST(Identifier, ConstructionOfHugeID)
{
    std::vector<uint8_t> bytes1;
    int size = 100;
    for (int i = 0; i < size; i++) {
        bytes1.push_back(i);
    }
    essentials::Identifier* id1 = new essentials::Identifier(bytes1.data(), bytes1.size());

    ASSERT_EQ(id1->getSize(), size);

    delete id1;
}

TEST(Identifier, ToByteVectorReturnsCopy)
{
    std::vector<uint8_t> bytes1;
    int size = 100;
    for (int i = 0; i < size; i++) {
        bytes1.push_back(i);
    }
    essentials::Identifier* id1 = new essentials::Identifier(bytes1.data(), bytes1.size());
    auto byteVector = id1->toByteVector();

    ASSERT_NE(byteVector.data(), id1->getRaw());

    delete id1;
}

TEST(Identifier, HashEqualForSameIDs)
{
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    essentials::Identifier* id1 = new essentials::Identifier(bytes1.data(), bytes1.size());
    essentials::Identifier* id2 = new essentials::Identifier(bytes2.data(), bytes2.size());

    ASSERT_EQ(id1->hash(), id2->hash());

    delete id1;
    delete id2;
}

TEST(Identifier, EqualWithSameID)
{
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    essentials::Identifier* id1 = new essentials::Identifier(bytes1.data(), bytes1.size());
    essentials::Identifier* id2 = new essentials::Identifier(bytes2.data(), bytes2.size());

    ASSERT_TRUE(*id1 == *id2);

    delete id1;
    delete id2;
}

TEST(Identifier, NotEqualWithDifferentID)
{
    std::vector<uint8_t> bytes1;
    std::vector<uint8_t> bytes2;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
        bytes2.push_back(i);
    }
    bytes2.push_back(2);
    essentials::Identifier* id1 = new essentials::Identifier(bytes1.data(), bytes1.size());
    essentials::Identifier* id2 = new essentials::Identifier(bytes2.data(), bytes2.size());

    ASSERT_FALSE(*id1 == *id2);

    delete id1;
    delete id2;
}

TEST(BroadCastID, NotEqualWithNormalID)
{
    std::vector<uint8_t> bytes1;
    for (int i = 0; i < 20; i++) {
        bytes1.push_back(i);
    }
    essentials::Identifier* normalID = new essentials::Identifier(bytes1.data(), bytes1.size());
    std::vector<uint8_t> bytesBroadcast;
    essentials::Identifier* broadcastID = new essentials::WildcardID(bytesBroadcast.data(), bytesBroadcast.size());

    ASSERT_FALSE(*broadcastID == *normalID);

    delete normalID;
    delete broadcastID;
}

TEST(BroadCastID, EqualWithBroadcastID)
{
    std::vector<uint8_t> bytesBroadcast1;
    bytesBroadcast1.push_back(1);
    essentials::Identifier* broadcastID1 = new essentials::WildcardID(bytesBroadcast1.data(), bytesBroadcast1.size());
    std::vector<uint8_t> bytesBroadcast2;
    essentials::Identifier* broadcastID2 = new essentials::WildcardID(bytesBroadcast2.data(), bytesBroadcast2.size());

    ASSERT_TRUE(*broadcastID1 == *broadcastID2);

    delete broadcastID1;
    delete broadcastID2;
}

TEST(IdentifierFactory, GenerateIDsOfVariousLength)
{
    essentials::IDManager factory;

    auto id1 = factory.generateID(1);
    ASSERT_EQ(id1->getSize(), 1);

    auto id4 = factory.generateID(4);
    ASSERT_EQ(id4->getSize(), 4);

    auto id15 = factory.generateID(15);
    ASSERT_EQ(id15->getSize(), 15);

    auto id18 = factory.generateID(18);
    ASSERT_EQ(id18->getSize(), 18);
}

TEST(IdentifierFactory, DuplicateIDs)
{
    essentials::IDManager idManager;
    auto id18 = idManager.generateID(18);
    auto id18Copy = idManager.getIDFromBytes(id18->toByteVector().data(), id18->toByteVector().size());
    ASSERT_TRUE(*id18 == *id18Copy);
}

TEST(IdentifierManager, CreateIDsFromIntegralTypes)
{
    essentials::IDManager* idManager = new essentials::IDManager();
    int idInt = 5;
    auto intId5 = idManager->getID<int>(idInt);

    std::vector<uint8_t> idBytes;
    idBytes.push_back(5);
    idBytes.push_back(0);
    idBytes.push_back(0);
    idBytes.push_back(0);
    essentials::Identifier* referenceId5 = new essentials::Identifier(idBytes.data(), idBytes.size());

    ASSERT_TRUE(*intId5 == *referenceId5);
}

TEST(IdentifierManager, GuarenteeSingleEntities)
{
    essentials::IDManager* idManager = new essentials::IDManager();
    int idInt = 5;
    auto intId1 = idManager->getID<int>(idInt);
    auto intId2 = idManager->getID<int>(idInt);

    ASSERT_EQ(intId1, intId2);
}

TEST(IdentifierManager, GenerateIDsOfVariousLength)
{
    essentials::IDManager* idManager = new essentials::IDManager();

    auto id1 = idManager->generateID(1);
    ASSERT_EQ(id1->getSize(), 1);

    auto id4 = idManager->generateID(4);
    ASSERT_EQ(id4->getSize(), 4);

    auto id15 = idManager->generateID(15);
    ASSERT_EQ(id15->getSize(), 15);

    auto id18 = idManager->generateID(18);
    ASSERT_EQ(id18->getSize(), 18);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
