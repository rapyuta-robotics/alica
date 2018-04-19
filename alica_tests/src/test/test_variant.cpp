#include <gtest/gtest.h>

#include "engine/collections/Variant.h"

using alica::Variant;

TEST(Variant, works) {

    int64_t val = 5;
    Variant v1;
    Variant v2(val);

    EXPECT_FALSE(v1.isSet());
    EXPECT_TRUE(v2.isSet());
    EXPECT_TRUE(v2.isInt());
    EXPECT_FALSE(v2.isPtr());
    EXPECT_FALSE(v2.isDouble());
    EXPECT_FALSE(v2.isIdent());
    EXPECT_FALSE(v2.isBool());
    EXPECT_FALSE(v2.isFloat());

    EXPECT_EQ(5, v2.getInt());
    v2.setInt(6);
    EXPECT_EQ(6, v2.getInt());

    EXPECT_FALSE(v1.isInt());
    EXPECT_FALSE(v1.isBool());
    EXPECT_FALSE(v1.isDouble());
    EXPECT_FALSE(v1.isIdent());
    EXPECT_FALSE(v1.isFloat());
    EXPECT_FALSE(v1.isPtr());

    v2.setFloat(2.3f);
    EXPECT_EQ(2.3f,v2.getFloat());

    v1.setBool(false);
    EXPECT_TRUE(v1.isSet());
    EXPECT_TRUE(v1.isBool());
    EXPECT_FALSE(v1.getBool());
    EXPECT_FALSE(v1.isInt());
    EXPECT_FALSE(v1.isFloat());
    EXPECT_FALSE(v1.isDouble());
    EXPECT_FALSE(v1.isPtr());
    EXPECT_FALSE(v1.isIdent());

    uint8_t arr[Variant::kVariantSize];

    int ret = v2.serializeTo(arr);
    EXPECT_EQ(Variant::kVariantSize,ret);
    ret = v1.loadFrom(arr);
    EXPECT_EQ(Variant::kVariantSize,ret);
    EXPECT_TRUE(v1.isFloat());
    EXPECT_EQ(2.3f,v2.getFloat());

    Variant v3;
    v3.serializeTo(arr);
    v2.loadFrom(arr);

    EXPECT_FALSE(v2.isSet());
}