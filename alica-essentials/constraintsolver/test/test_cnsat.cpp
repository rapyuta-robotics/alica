#include <gtest/gtest.h>

#include "CNSat.h"
#include "FileSystem.h"
#include "types/Clause.h"
#include "types/Lit.h"
#include "types/Var.h"
#include <engine/AlicaClock.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>

using namespace std;
using namespace alica::reasoner;
using namespace alica::reasoner::cnsat;
using alica::AlicaTime;

TEST(CNSatTest, CNSAT0)
{
    shared_ptr<CNSat> cns = make_shared<CNSat>();

    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test/cnf/testen.cnf";
    std::cout << path << std::endl;
    {
        ifstream test(path);
        ASSERT_TRUE(test.good()) << "Cannot find file " << path << std::endl;
        ;
    }
    cns->readFromCNFFile(path);

    /*foreach(Alica.Reasoner.CNSAT.Clause c in cns->clauses) {
     c->print();
     }*/
    alica::AlicaClock clock;
    cns->init();
    bool sf = cns->solve(clock.now() + AlicaTime::minutes(1));
    EXPECT_TRUE(sf);

    // cout << endl;

    //    if (sf)
    //    {
    //        cout << "Satisfiable" << endl;
    //        for (shared_ptr<Var> v : *(cns->variables))
    //        {
    //            v->print();
    //            cout << " ";
    //        }
    //        cout << endl;
    //    }
    //    else
    //        cout << "Unsatisfiable" << endl;

    //    cout << "------------------\n\nChecking Solution:" << endl;
    bool isSolution = true;
    for (shared_ptr<Clause> c : *(cns->clauses)) {
        if (!c->checkSatisfied()) {
            isSolution = false;
            /*c->print();
             foreach(Lit l in c.Literals) {
             Console.Write(l.Var.WatchList.Count+"\t");
             }

             cout << endl;
             c.watcher[0].Lit.Var->print();
             Console.Write("\t");
             c.watcher[1].Lit.Var->print();
             Console.Write("\n");*/
        }
    }
    EXPECT_TRUE(isSolution);
    cout << "Is Solution: " << isSolution << endl;
    // cns->printStatistics();
    //
    //    cout << endl;
}

TEST(CNSatTest, CNSATaim_50_1_6_yes1_4)
{
    shared_ptr<CNSat> cns = make_shared<CNSat>();
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test/cnf/aim-50-1_6-yes1-4.cnf";
    std::cout << path << std::endl;
    {
        ifstream test(path);
        ASSERT_TRUE(test.good()) << "Cannot find file " << path << std::endl;
        ;
    }
    cns->readFromCNFFile(path);

    alica::AlicaClock clock;
    cns->init();
    bool sf = cns->solve(clock.now() + AlicaTime::minutes(1));

    EXPECT_TRUE(sf);

    bool isSolution = true;
    for (shared_ptr<Clause> c : *(cns->clauses)) {
        if (!c->checkSatisfied()) {
            isSolution = false;
        }
    }
    EXPECT_TRUE(isSolution);
}

TEST(CNSatTest, CNSATpar8_1_c_cnf)
{
    shared_ptr<CNSat> cns = make_shared<CNSat>();
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test/cnf/par8-1-c.cnf";
    std::cout << path << std::endl;
    {
        ifstream test(path);
        ASSERT_TRUE(test.good()) << "Cannot find file " << path << std::endl;
        ;
    }

    cns->readFromCNFFile(path);

    alica::AlicaClock clock;
    cns->init();
    bool sf = cns->solve(clock.now() + AlicaTime::minutes(1));

    EXPECT_TRUE(sf);

    bool isSolution = true;
    for (shared_ptr<Clause> c : *(cns->clauses)) {
        if (!c->checkSatisfied()) {
            isSolution = false;
        }
    }
    EXPECT_TRUE(isSolution);
}

TEST(CNSatTest, CNSAT1_aim_100_1_6_no_1cnf)
{
    shared_ptr<CNSat> cns = make_shared<CNSat>();
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test/cnf/aim-100-1_6-no-1.cnf";
    std::cout << path << std::endl;
    {
        ifstream test(path);
        ASSERT_TRUE(test.good()) << "Cannot find file " << path << std::endl;
        ;
    }

    cns->readFromCNFFile(path);

    alica::AlicaClock clock;
    cns->init();
    bool sf = cns->solve(clock.now() + AlicaTime::minutes(1));

    EXPECT_FALSE(sf);
}

TEST(CNSatTest, CNSAT1dubois22)
{
    shared_ptr<CNSat> cns = make_shared<CNSat>();
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test/cnf/dubois22.cnf";
    std::cout << path << std::endl;
    {
        ifstream test(path);
        ASSERT_TRUE(test.good()) << "Cannot find file " << path << std::endl;
        ;
    }

    cns->readFromCNFFile(path);

    alica::AlicaClock clock;
    cns->init();
    bool sf = cns->solve(clock.now() + AlicaTime::minutes(1));

    EXPECT_FALSE(sf);
}

TEST(CNSatTest, CNSAThole6)
{
    shared_ptr<CNSat> cns = make_shared<CNSat>();
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test/cnf/hole6.cnf";
    std::cout << path << std::endl;
    {
        ifstream test(path);
        ASSERT_TRUE(test.good()) << "Cannot find file " << path << std::endl;
        ;
    }

    cns->readFromCNFFile(path);

    alica::AlicaClock clock;
    cns->init();
    bool sf = cns->solve(clock.now() + AlicaTime::minutes(1));

    EXPECT_FALSE(sf);
}

TEST(CNSatTest, CNSAT1_dubois20)
{
    shared_ptr<CNSat> cns = make_shared<CNSat>();
    std::string path = ros::package::getPath("constraintsolver");
    path = path + "/test/cnf/dubois20.cnf";
    std::cout << path << std::endl;
    {
        ifstream test(path);
        ASSERT_TRUE(test.good()) << "Cannot find file " << path << std::endl;
        ;
    }

    cns->readFromCNFFile(path);

    alica::AlicaClock clock;
    cns->init();
    bool sf = cns->solve(clock.now() + AlicaTime::minutes(1));

    EXPECT_FALSE(sf);
}
