#include <gtest/gtest.h>

#include "constraintsolver/GSolver.h"
#include "constraintsolver/CNSMTGSolver.h"
#include "autodiff/AutoDiff.h"

#include <autodiff/ConstraintBuilder.h>
#include <essentials/SystemConfig.h>
#include <engine/AlicaClock.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <ctime>
#include <iostream>

#include <cmath>

using namespace std;
using namespace autodiff;
using namespace alica;
using namespace alica::reasoner;

TermPtr outsideDummyAreas(TVec<2> pos)
{
    TermHolder* h = pos.getX()->getOwner();
    return ((pos.getX() > h->constant(500)) | (pos.getX() < h->constant(-500)) | (pos.getY() > h->constant(9000)) | (pos.getY() < h->constant(8000))) &
           ((pos.getX() > h->constant(500)) | (pos.getX() < h->constant(-500)) | (pos.getY() < h->constant(-9000)) | (pos.getY() > h->constant(-8000)));
}

TEST(AutoDiffTest, GSOLVER)
{
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test";
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    sc.setRootPath(path);
    sc.setConfigPath(path + "/etc");

    // 12000 * 18000
    const double FIELDLENGTH = 18000;
    const double FIELDWIDTH = 12000;

    GSolver g;

    Term::setAnd(AndType::AND);
    Term::setOr(OrType::MAX);

    TermHolder h;
    TVec<2> ball = TVec<2>(h.constant(-3000), h.constant(-2000));

    TermPtr equation;
    int posCount = 30;
    // int oppCount = 3;

    std::vector<Interval<double>> limits(posCount * 2);

    TermPtr constraint = h.trueConstant();
    std::vector<TVec<2>> poses;
    std::vector<TVec<2>> opps;
    opps.push_back(TVec<2>(h.constant(5000), h.constant(0)));
    opps.push_back(TVec<2>(h.constant(500), h.constant(-2000)));
    opps.push_back(TVec<2>(h.constant(2000), h.constant(3300)));

    for (int i = 0; i < posCount; ++i) {
        TermPtr x = h.createVariable(2 * i);
        TermPtr y = h.createVariable(2 * i + 1);
        poses.push_back(TVec<2>(x, y));

        constraint = constraint & outsideDummyAreas(poses[i]);
        constraint = constraint & (distanceSqr(ball, poses[i]) > h.constant(2000 * 2000));
        constraint = constraint & (distanceSqr(opps[0], poses[i]) > h.constant(800 * 800));
        constraint = constraint & (distanceSqr(opps[1], poses[i]) > h.constant(800 * 800));
        constraint = constraint & (distanceSqr(opps[2], poses[i]) > h.constant(800 * 800));

        limits[2 * i] = Interval<double>(-FIELDLENGTH / 2, FIELDLENGTH / 2);
        limits[2 * i + 1] = Interval<double>(-FIELDWIDTH / 2, FIELDWIDTH / 2);
    }

    double util = 0;
    TermPtr csu = h.constraintUtility(constraint, h.constant(1));

    // cout << constraint << endl;
    long gt = time(NULL);

    std::vector<double> res;
    int gsolved = 0;
    int count = 3;
    for (int i = 0; i < count; ++i) {
        g.solve(csu, h, limits, util, res);
        cout << ".";
        if (util > 0.5) {
            ++gsolved;
        }
        EXPECT_GT(util, 0.5);
        for (int i = 0; i < posCount; ++i) {

            EXPECT_LT(2000 * 2000, (res[2 * i] + 3000) * (res[2 * i] + 3000) + (res[2 * i + 1] + 2000) * (res[2 * i + 1] + 2000));
            EXPECT_LT(800 * 800, (res[2 * i] - 5000) * (res[2 * i] - 5000) + (res[2 * i + 1]) * (res[2 * i + 1]));
            EXPECT_LT(800 * 800, (res[2 * i] - 500) * (res[2 * i] - 500) + (res[2 * i + 1] + 2000) * (res[2 * i + 1] + 2000));
            EXPECT_LT(800 * 800, (res[2 * i] - 2000) * (res[2 * i] - 2000) + (res[2 * i + 1] - 3300) * (res[2 * i + 1] - 3300));
            EXPECT_LE(-FIELDLENGTH / 2, res[2 * i]);
            EXPECT_GE(FIELDLENGTH / 2, res[2 * i]);
            EXPECT_LE(-FIELDWIDTH / 2, res[2 * i + 1]);
            EXPECT_GE(FIELDWIDTH / 2, res[2 * i + 1]);
        }
    }
    cout << endl;
    gt = time(NULL) - gt;

    cout << "GSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
    cout << "GSolver Solved: " << gsolved << " times" << endl;
}

TEST(AutoDiffTest, GSOLVER_UTIL)
{
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test";
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    sc.setRootPath(path);
    sc.setConfigPath(path + "/etc");
    // 12000 * 18000
    const double FIELDLENGTH = 18000;
    const double FIELDWIDTH = 12000;

    GSolver g;
    TermHolder h;
    Term::setAnd(AndType::AND);
    Term::setOr(OrType::MAX);

    TVec<2> ball = TVec<2>(h.constant(-3000), h.constant(-2000));

    TermPtr equation;
    int posCount = 30;
    std::vector<Interval<double>> limits(posCount * 2);

    TermPtr constraint = h.trueConstant();

    std::vector<TVec<2>> opps;
    opps.push_back(TVec<2>(h.constant(5000), h.constant(0)));
    opps.push_back(TVec<2>(h.constant(500), h.constant(-2000)));
    opps.push_back(TVec<2>(h.constant(2000), h.constant(3300)));

    std::vector<TVec<2>> poses;
    for (int i = 0; i < posCount; ++i) {
        VarPtr x = h.createVariable(i * 2);
        VarPtr y = h.createVariable(i * 2 + 1);
        poses.push_back(TVec<2>(x, y));

        constraint = constraint & outsideDummyAreas(poses[i]);
        constraint = constraint & outsideDummyAreas(poses[i]);
        constraint = constraint & (distance(ball, poses[i]) > h.constant(2000));
        constraint = constraint & (distanceSqr(opps[0], poses[i]) > h.constant(800 * 800));
        constraint = constraint & (distanceSqr(opps[1], poses[i]) > h.constant(800 * 800));
        constraint = constraint & (distanceSqr(opps[2], poses[i]) > h.constant(800 * 800));

        limits[2 * i] = Interval<double>(-FIELDLENGTH / 2, FIELDLENGTH / 2);
        limits[2 * i + 1] = Interval<double>(-FIELDWIDTH / 2, FIELDWIDTH / 2);
    }

    TermPtr utilCsu = h.zeroConstant();
    for (int i = 0; i < posCount; ++i) {
        TVec<2> ownpos = poses[i];
        utilCsu = utilCsu + h.constant(2 * FIELDLENGTH) - distance(ball, ownpos);
    }

    double util = 0;
    TermPtr csu = h.constraintUtility(constraint, utilCsu);

    // cout << constraint << endl;
    long gt = time(NULL);

    std::vector<double> res;
    int gsolved = 0;
    int count = 2;
    for (int i = 0; i < count; ++i) {
        g.solve(csu, h, limits, util, res);
        if (util > 0.5) {
            ++gsolved;
        }
        EXPECT_GT(util, 0.5);
        for (int i = 0; i < posCount; ++i) {
            EXPECT_GT(2500 * 2500, (res[2 * i] + 3000) * (res[2 * i] + 3000) + (res[2 * i + 1] + 2000) * (res[2 * i + 1] + 2000));
            EXPECT_LT(2000 * 2000, (res[2 * i] + 3000) * (res[2 * i] + 3000) + (res[2 * i + 1] + 2000) * (res[2 * i + 1] + 2000));
            EXPECT_LT(800 * 800, (res[2 * i] - 5000) * (res[2 * i] - 5000) + (res[2 * i + 1]) * (res[2 * i + 1]));
            EXPECT_LT(800 * 800, (res[2 * i] - 500) * (res[2 * i] - 500) + (res[2 * i + 1] + 2000) * (res[2 * i + 1] + 2000));
            EXPECT_LT(800 * 800, (res[2 * i] - 2000) * (res[2 * i] - 2000) + (res[2 * i + 1] - 3300) * (res[2 * i + 1] - 3300));
            EXPECT_LE(-FIELDLENGTH / 2, res[2 * i]);
            EXPECT_GE(FIELDLENGTH / 2, res[2 * i]);
            EXPECT_LE(-FIELDWIDTH / 2, res[2 * i + 1]);
            EXPECT_GE(FIELDWIDTH / 2, res[2 * i + 1]);
        }
    }
    cout << endl;
    gt = time(NULL) - gt;

    cout << "GSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
    cout << "GSolver Solved: " << gsolved << " times" << endl;
}
/*
TEST(AutoDiffTest, CNSMTGSOLVER)
{
        // 12000 * 18000
        const double FIELDLENGTH = 18000;
        const double FIELDWIDTH = 12000;

        shared_ptr<CNSMTGSolver> g = make_shared<CNSMTGSolver>();
//	g->useIntervalProp = false;

        Term::setAnd(AndType::AND);
        Term::setOr(OrType::MAX);

        shared_ptr<TVec> ball = make_shared<TVec>(initializer_list<double>{-3000, -2000});

        shared_ptr<Term> equation;
        int posCount = 30;
        int oppCount = 3;
        auto vars = make_shared<vector<shared_ptr<Variable>>>(posCount * 2);
        auto limits = make_shared<vector<shared_ptr<vector<double>>>>(posCount * 2);
        for (int i = 0; i < posCount * 2; ++i)
                limits->at(i) = make_shared<vector<double>>(2);

        shared_ptr<Term> constraint = LTConstraint::TRUE;
        vector<shared_ptr<TVec>> poses;
        for (int i = 0; i < posCount; ++i)
        {
                vars->at(i * 2) = make_shared<Variable>();
                vars->at(i * 2 + 1) = make_shared<Variable>();
                poses.push_back(make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(i * 2), vars->at(i * 2 +
1)}));

                constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OwnPenaltyArea, poses[i]);
                constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OppPenaltyArea, poses[i]);
                constraint = constraint & (ConstraintBuilder::distance(ball, poses[i]) > TermBuilder::constant(2000));

                limits->at(i * 2)->at(0) = -FIELDLENGTH / 2;
                limits->at(i * 2)->at(1) = FIELDLENGTH / 2;
                limits->at(i * 2 + 1)->at(0) = -FIELDWIDTH / 2;
                limits->at(i * 2 + 1)->at(1) = FIELDWIDTH / 2;
        }

        vector<shared_ptr<TVec>> opps;
        opps.push_back(make_shared<TVec>(initializer_list<double>{5000, 0}));
        opps.push_back(make_shared<TVec>(initializer_list<double>{500, -2000}));
        opps.push_back(make_shared<TVec>(initializer_list<double>{2000, 3300}));
        // XXX: random pos

        double util = 0;
        shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, TermBuilder::constant(1));


        //cout << constraint << endl;
        long gt = time(NULL);

        shared_ptr<vector<double>> res;
        int gsolved = 0;
        int count = 1;
        for (int i = 0; i < count; ++i)
        {
                res = g->solve(csu, vars, limits, util);
                cout << ".";
                if (util > 0.5)
                        gsolved++;
        }
        cout << endl;
        gt = time(NULL) - gt;

        cout << "CNSMTGSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
        cout << "CNSMTGSolver Solved: " << gsolved << " times" << endl;
        cout << "Result:" << res->at(0) << " " << res->at(1) << " with Utility " << util << endl;
        double actual = TermUtils::evaluate(csu, vars, res);
        cout << "Result:" << actual << endl;
}

TEST(AutoDiffTest, CNSMTGSOLVER_UTIL)
{
        // 12000 * 18000
        const double FIELDLENGTH = 18000;
        const double FIELDWIDTH = 12000;

        shared_ptr<CNSMTGSolver> g = make_shared<CNSMTGSolver>();
//	g->useIntervalProp = false;

        Term::setAnd(AndType::AND);
        Term::setOr(OrType::MAX);

        shared_ptr<TVec> ball = make_shared<TVec>(initializer_list<double>{-3000, -2000});

        shared_ptr<Term> equation;
        int posCount = 30;
        int oppCount = 3;
        auto vars = make_shared<vector<shared_ptr<Variable>>>(posCount * 2);
        auto limits = make_shared<vector<shared_ptr<vector<double>>>>(posCount * 2);
        for (int i = 0; i < posCount * 2; ++i)
                limits->at(i) = make_shared<vector<double>>(2);

        shared_ptr<Term> constraint = LTConstraint::TRUE;
        vector<shared_ptr<TVec>> poses;
        for (int i = 0; i < posCount; ++i)
        {
                vars->at(i * 2) = make_shared<Variable>();
                vars->at(i * 2 + 1) = make_shared<Variable>();
                poses.push_back(make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(i * 2), vars->at(i * 2 +
1)}));

                constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OwnPenaltyArea, poses[i]);
                constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OppPenaltyArea, poses[i]);
                constraint = constraint & (ConstraintBuilder::distance(ball, poses[i]) > TermBuilder::constant(2000));

                limits->at(i * 2)->at(0) = -FIELDLENGTH / 2;
                limits->at(i * 2)->at(1) = FIELDLENGTH / 2;
                limits->at(i * 2 + 1)->at(0) = -FIELDWIDTH / 2;
                limits->at(i * 2 + 1)->at(1) = FIELDWIDTH / 2;
        }

        vector<shared_ptr<TVec>> opps;
        opps.push_back(make_shared<TVec>(initializer_list<double>{5000, 0}));
        opps.push_back(make_shared<TVec>(initializer_list<double>{500, -2000}));
        opps.push_back(make_shared<TVec>(initializer_list<double>{2000, 3300}));
        // XXX: random pos

        shared_ptr<Term> utilCsu = TermBuilder::constant(0);
        for (int i = 0; i < posCount; ++i) {
//		TVec ownpos = new TVec(vars[0], vars[1]);
//	    Term util = 2*FIELDLENGTH - ConstraintBuilder::distance(ball, ownpos);
                shared_ptr<TVec> ownpos = make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(0),
vars->at(1)}); utilCsu = utilCsu + TermBuilder::constant(2*FIELDLENGTH) - ConstraintBuilder::distance(ball, ownpos);
        }

        double util = 0;
        shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, utilCsu);


        //cout << constraint << endl;
        long gt = time(NULL);

        shared_ptr<vector<double>> res;
        int gsolved = 0;
        int count = 1;
        for (int i = 0; i < count; ++i)
        {
                res = g->solve(csu, vars, limits, util);
                cout << ".";
                if (util > 0.5)
                        gsolved++;
        }
        cout << endl;
        gt = time(NULL) - gt;

        cout << "CNSMTGSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
        cout << "CNSMTGSolver Solved: " << gsolved << " times" << endl;
        cout << "Result:" << res->at(0) << " " << res->at(1) << " with Utility " << util << endl;
        double actual = TermUtils::evaluate(csu, vars, res);
        cout << "Result:" << actual << endl;
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
