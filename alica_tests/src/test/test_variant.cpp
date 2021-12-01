#include <gtest/gtest.h>

#include "engine/collections/Variant.h"

using alica::Variant;

namespace alica {

TEST(Variant, works)
{
    int64_t val = 5;
    Variant v1;
    Variant v2(val);

    EXPECT_FALSE(variant::isSet(v1));
    EXPECT_TRUE(variant::isSet(v2));

    uint8_t arr[variant::kVariantSize];

    v2 = 2.3f;
    uint32_t ret = variant::serializeTo(arr, v2);
    EXPECT_EQ(variant::kVariantSize, ret);
    ret = variant::loadFrom(arr, v1);
    EXPECT_EQ(variant::kVariantSize, ret);
    EXPECT_TRUE(std::holds_alternative<float>(v1));
    EXPECT_EQ(2.3f, std::get<float>(v2));

    Variant v3;
    variant::serializeTo(arr, v3);
    variant::loadFrom(arr, v2);

    EXPECT_FALSE(variant::isSet(v2));
}

} // namespace alica
