#include "AutoDiff.h"

#include <gtest/gtest.h>

using namespace autodiff;

TEST(AutoDiffTest, Distance)
{
    TermHolder h;
    TermPtr constraint = h.trueConstant();
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);
    TVec<2> pos(x, y);
    TVec<2> ball = TVec<2>(h.constant(-3000), h.constant(-2000));

    constraint = constraint & (distanceSqr(ball, pos) > h.constant(2000 * 2000));

    h.compile(constraint);
    std::array<double, 2> point{-3900, -1770.5};
    std::array<double, 3> result;
    h.evaluate(&point[0], &result[0]);
    ASSERT_LE(result[0], 0.0);
    ASSERT_LT(result[1], 0.0);
    ASSERT_GT(result[2], 0.0);
}

TEST(AutoDiffTest, ABS)
{
    TermHolder h;
    VarPtr x = h.createVariable(1);

    TermPtr func = h.abs(x);

    std::vector<double> point{-13};

    h.compile(func);

    std::vector<double> gradientAndValue(2);
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(13, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(-1, gradientAndValue[1]);

    point[0] = 4.2;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(4.2, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[1]);
}

TEST(AutoDiffTest, AND)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.and_(x > h.zeroConstant(), y > h.zeroConstant());

    std::vector<double> point{13, 37};

    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(1, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);

    point[0] = 0.0;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(0, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0.01, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);

    point[1] = -10.0;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(-0.1, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0.01, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0.01, gradientAndValue[2]);
}

TEST(AutoDiffTest, ATAN2)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.atan2(x, y);

    std::vector<double> point{13, 37};
    h.compile(func);

    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    double eval = gradientAndValue[0];

    ASSERT_NEAR(0.337878, eval, 10E-6);
    ASSERT_NEAR(-0.024057, gradientAndValue[1], 10E-6);
    ASSERT_NEAR(0.008453, gradientAndValue[2], 10E-6);
}

TEST(AutoDiffTest, CONSTPOWER)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);

    TermPtr func = h.constPower(x, 2);

    std::vector<double> point{13};
    h.compile(func);
    std::vector<double> gradientAndValue(2);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(169, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(26, gradientAndValue[1]);

    point[0] = 0;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(0, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
}

TEST(AutoDiffTest, CONSTRAINTUTILITY)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.constraintUtility(x < h.constant(42), y);

    std::vector<double> point{13, 37};

    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(37, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[2]);

    point[0] = 44;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(-0.02, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(-0.01, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);
}

TEST(AutoDiffTest, COS)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);

    TermPtr func = h.cos(x);

    std::vector<double> point{13};

    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(0.907447, gradientAndValue[0], 10E-6);
    ASSERT_NEAR(-0.420167, gradientAndValue[1], 10E-6);

    point[0] = 0.0;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(1.0, gradientAndValue[0], 10E-12);
    ASSERT_NEAR(0.0, gradientAndValue[1], 10E-12);
}

TEST(AutoDiffTest, EXP)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);

    TermPtr func = h.exp(x);

    std::vector<double> point{13};

    h.compile(func);
    std::vector<double> gradientAndValue(2);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(442413, gradientAndValue[0], 10E-1);
    ASSERT_NEAR(442413, gradientAndValue[1], 10E-1);

    point[0] = 0;

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(1.0, gradientAndValue[0], 10E-1);
    ASSERT_NEAR(1.0, gradientAndValue[1], 10E-1);
}

TEST(AutoDiffTest, LINSIGMOID)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);

    TermPtr func = h.linSigmoid(x);

    std::vector<double> point{0};
    h.compile(func);
    std::vector<double> gradientAndValue(2);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(0.5, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(1.0, gradientAndValue[1]);

    point[0] = 1.0;
    h.evaluate(&point[0], &gradientAndValue[0]);
    ASSERT_NEAR(0.7310586, gradientAndValue[0], 1e-6);
    ASSERT_DOUBLE_EQ(1.0, gradientAndValue[1]);
}

TEST(AutoDiffTest, LOG)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);

    TermPtr func = h.log(x);

    std::vector<double> point{13};

    h.compile(func);
    std::vector<double> gradientAndValue(2);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(2.56495, gradientAndValue[0], 10E-5);
    ASSERT_NEAR(0, gradientAndValue[1], 10E-1);

    point[0] = 1.0;

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(0, gradientAndValue[0], 10E-12);
    ASSERT_NEAR(1.0, gradientAndValue[1], 10E-12);
}

TEST(AutoDiffTest, LTCONSTRAINT)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.lessThan(x, y);
    std::vector<double> point{13, 37};
    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(1, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);

    std::swap(point[0], point[1]);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(-0.24, gradientAndValue[0], 10E-2);
    ASSERT_DOUBLE_EQ(-0.01, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0.01, gradientAndValue[2]);
}

TEST(AutoDiffTest, LTECONSTRAINT)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.lessThanEqual(x, y);

    std::vector<double> point{13, 37};
    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(1, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);

    std::swap(point[0], point[1]);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(-0.24, gradientAndValue[0], 10E-2);
    ASSERT_DOUBLE_EQ(-0.01, gradientAndValue[1]);
}

TEST(AutoDiffTest, MAX)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.max(x, y);

    std::vector<double> point{13, 37};

    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(37, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[2]);

    std::swap(point[0], point[1]);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(37, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);
}

TEST(AutoDiffTest, MIN)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.min(x, y);

    std::vector<double> point{13, 37};

    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(13, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);

    std::swap(point[0], point[1]);

    h.evaluate(&point[0], &gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(13, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[2]);
}

TEST(AutoDiffTest, OR)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.or_(x, y);

    std::vector<double> point{13, 37};

    h.compile(func);
    std::vector<double> gradientAndValue(3);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(1, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);

    point[0] = 0.0;
    h.evaluate(&point[0], &gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[2]);

    point[1] = 0.0;
    h.evaluate(&point[0], &gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[1]);
    ASSERT_DOUBLE_EQ(1, gradientAndValue[2]);
}

TEST(AutoDiffTest, REIFICATION)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);

    TermPtr func = h.reify(x > h.constant(3));

    std::vector<double> point{13};

    h.compile(func);
    std::vector<double> gradientAndValue(2);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(1, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0.01, gradientAndValue[1]);

    point[0] = 2.0;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_DOUBLE_EQ(0.0, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(0.01, gradientAndValue[1]);
}

TEST(AutoDiffTest, SIN)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);

    TermPtr func = h.sin(x);

    std::vector<double> point{13};

    h.compile(func);
    std::vector<double> gradientAndValue(2);

    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(0.420167, gradientAndValue[0], 10E-6);
    ASSERT_NEAR(0.907447, gradientAndValue[1], 10E-6);

    point[0] = 0.0;
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(0.0, gradientAndValue[0], 10E-12);
    ASSERT_NEAR(1.0, gradientAndValue[1], 10E-12);
}

TEST(AutoDiffTest, TERMPOWER)
{
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    TermPtr func = h.power(x, y);

    std::vector<double> point{13, 2};
    h.compile(func);
    std::vector<double> gradientAndValue(3);
    h.evaluate(point.begin(), point.end(), gradientAndValue.begin(), gradientAndValue.end());

    ASSERT_DOUBLE_EQ(169, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(26, gradientAndValue[1]);
    ASSERT_NEAR(433.476441, gradientAndValue[2], 10E-6);

    point[0] = 2.0;
    point[1] = -1;
    h.evaluate(point.begin(), point.end(), gradientAndValue.begin(), gradientAndValue.end());

    ASSERT_DOUBLE_EQ(0.5, gradientAndValue[0]);
    ASSERT_DOUBLE_EQ(-0.25, gradientAndValue[1]);
    ASSERT_NEAR(0.34657359, gradientAndValue[2], 10E-6);
}

TEST(AutoDiffTest, EQUALITY)
{
    TermHolder h;
    TermPtr one = h.constant(1);

    ASSERT_TRUE(one == one);
}

TEST(AutoDiffTest, COMPILED)
{
    // we will use a function of two variables
    TermHolder h;
    TermPtr x = h.createVariable(1);
    TermPtr y = h.createVariable(2);

    // Define our function: func(x, y) = (x + y) * exp(x - y)
    TermPtr func = (x + y) * h.exp(x - y);

    // point where the function will be evaluated/differentiated
    std::vector<double> point{1, -2};

    // create a compiled version of func.
    h.compile(func);
    std::vector<double> gradientAndValue(3);

    // calculate value and gradient at (x, y) = (1, -2)
    h.evaluate(&point[0], &gradientAndValue[0]);

    ASSERT_NEAR(-20.0855369231877, gradientAndValue[0], 10E-10);
    ASSERT_NEAR(0, gradientAndValue[1], 10E-10);
    ASSERT_NEAR(40.1710738463753, gradientAndValue[2], 10E-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
