#include <gtest/gtest.h>


#include <engine/blackboard/BlackBoard.h>


using alica::BlackBoard;
using alica::BBIdent;
using alica::ByteArray;


TEST(BlackBoard, AddRemoveEntry) {
    BlackBoard bb;
    EXPECT_TRUE(bb.empty());
    EXPECT_EQ(0,bb.size());
    
    std::string s1("Hello");
    std::string s2("My name is Inigo Montoya.");
    BBIdent hi = bb.registerValue(s1.c_str(),s1.size());
    EXPECT_FALSE(bb.empty());
    EXPECT_EQ(1,bb.size());
    EXPECT_TRUE(bb.hasValue(hi));
    BBIdent monty = bb.registerValue(s2.c_str(),s2.size());
    EXPECT_FALSE(bb.empty());
    EXPECT_EQ(2,bb.size());

    BBIdent monty2 = bb.registerValue(s2.begin(),s2.end());

    EXPECT_EQ(monty,monty2);
    EXPECT_EQ(2,bb.size());

    EXPECT_TRUE(bb.hasValue(hi));
    EXPECT_TRUE(bb.hasValue(monty));
    const ByteArray& ba = bb.getValue(monty);
    std::string s3(reinterpret_cast<const char*>(ba.begin()),ba.size());
    EXPECT_EQ(s3,s2);

    bb.removeValue(monty);
    EXPECT_EQ(1,bb.size());
    EXPECT_TRUE(bb.hasValue(hi));
    EXPECT_FALSE(bb.hasValue(monty));

}