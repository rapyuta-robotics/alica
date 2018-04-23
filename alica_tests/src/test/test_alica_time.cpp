#include <gtest/gtest.h>
#include <engine/AlicaClock.h>

using alica::AlicaTime;
using alica::AlicaClock;

TEST(AlicaTime, AlicaClock) {

    AlicaTime t;
    EXPECT_EQ(t.inNanoseconds(), 0);
    EXPECT_EQ(t.inMicroseconds(), 0);
    EXPECT_EQ(t.inMilliseconds(), 0);
    EXPECT_EQ(t.inSeconds(), 0);
    EXPECT_EQ(t.inMinutes(), 0);
    EXPECT_EQ(t.inHours(), 0);
    EXPECT_EQ(t, AlicaTime::zero());

    AlicaTime i = AlicaTime::hours(2);
    EXPECT_EQ(i.inHours(), 2LL);
    EXPECT_EQ(i.inMinutes(), 2LL * 60);
    EXPECT_EQ(i.inSeconds(), 2LL * 60 * 60);
    EXPECT_EQ(i.inMilliseconds(), 2LL * 60 * 60 * 1000);
    EXPECT_EQ(i.inMicroseconds(), 2LL * 60 * 60 * 1000000);
    EXPECT_EQ(i.inNanoseconds(), 2LL * 60 * 60 * 1000000000);

    EXPECT_EQ(i + AlicaTime::minutes(120), AlicaTime::hours(4));
    EXPECT_EQ(i - AlicaTime::minutes(60), AlicaTime::minutes(60));

    i -= AlicaTime::minutes(30);
    EXPECT_EQ(i, AlicaTime::minutes(90));

    i += AlicaTime::minutes(30);
    EXPECT_EQ(i, AlicaTime::minutes(120));
    
    i *= 2;
    EXPECT_EQ(i, AlicaTime::minutes(240));

    i /= 2;
    EXPECT_EQ(i, AlicaTime::minutes(120));

    EXPECT_EQ(i / 2, AlicaTime::minutes(60));
    EXPECT_EQ(i / 2.0, AlicaTime::minutes(60));

    EXPECT_EQ(i * 2, AlicaTime::minutes(240));
    EXPECT_EQ(i * 2.0, AlicaTime::minutes(240));
    EXPECT_TRUE(t < i);
    EXPECT_TRUE(t <= i);
    EXPECT_TRUE(t <= t);
    EXPECT_TRUE(AlicaTime::nanoseconds(i.inNanoseconds() + 1) > i);
    EXPECT_TRUE(AlicaTime::nanoseconds(i.inNanoseconds() + 1) >= i);
    EXPECT_TRUE(i >= i);
    EXPECT_TRUE(i == i);
    EXPECT_TRUE(i != i / 2);

    AlicaTime m = AlicaTime::seconds(1.05);
    EXPECT_EQ(m.inMilliseconds(), AlicaTime::milliseconds(1050).inMilliseconds());

    AlicaClock e;
    AlicaTime start = e.now();
    AlicaTime result = e.now() - start;
    EXPECT_GE(result, AlicaTime::zero());
}