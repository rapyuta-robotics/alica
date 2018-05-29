#include "alica_solver_interface/Interval.h"

#include <gtest/gtest.h>

using namespace alica;

TEST(IntervalTest, Double)
{
    Interval<double> a(0, 1);
    ASSERT_TRUE(a.isValid());
    Interval<double> b(1, 0);
    ASSERT_FALSE(b.isValid());

    ASSERT_DOUBLE_EQ(a.size(), 1.0);
    ASSERT_DOUBLE_EQ(a.getMidPoint(), 0.5);
    ASSERT_DOUBLE_EQ(a.getMin(), 0.0);
    ASSERT_DOUBLE_EQ(a.getMax(), 1.0);

    Interval<double> c = a;
    ASSERT_EQ(a, c);
    ASSERT_NE(b, c);

    c.intersect(-2.0, 5.0);
    ASSERT_EQ(a, c);

    c.intersect(Interval<double>(0.3, 0.6));
    ASSERT_NE(a, c);
    ASSERT_DOUBLE_EQ(c.getMidPoint(), 0.45);
    ASSERT_DOUBLE_EQ(c.getMin(), 0.3);
    ASSERT_DOUBLE_EQ(c.getMax(), 0.6);

    ASSERT_DOUBLE_EQ(a.clamp(5), 1);
    ASSERT_DOUBLE_EQ(a.clamp(-5), 0);

    c.intersect(9, 10);
    ASSERT_FALSE(c.isValid());
}
