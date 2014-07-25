// {{{ GPL License 

// This file is part of gringo - a grounder for logic programs.
// Copyright (C) 2013  Roland Kaminski

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// }}}

#include "gringo/safetycheck.hh"

#include "tests/tests.hh"
#include "tests/term_helper.hh"

#include <map>

namespace Gringo { namespace Test {

// {{{ declaration of TestSafetyCheck

class TestSafetyCheck : public CppUnit::TestFixture {
    CPPUNIT_TEST_SUITE(TestSafetyCheck);
        CPPUNIT_TEST(test_safety);
    CPPUNIT_TEST_SUITE_END();

public:
    virtual void setUp();
    virtual void tearDown();

    void test_safety();

    virtual ~TestSafetyCheck();
};

// }}}

using namespace Gringo::IO;

// {{{ definition of auxiliary functions

namespace {

typedef SafetyChecker<std::string, std::string> C;
typedef std::tuple<std::string, std::initializer_list<std::string>, std::initializer_list<std::string>> T;

std::string check(std::initializer_list<T> list) {
    C dep;
    std::unordered_map<std::string, C::VarNode*> vars;
    auto iv = [&vars, &dep](std::string const &y) -> C::VarNode& {
        auto &pVar = vars[y];
        if (!pVar) { pVar = &dep.insertVar(y); }
        return *pVar;
    };
    for (auto &x : list) {
        auto &ent(dep.insertEnt(std::get<0>(x)));
        for (auto &y : std::get<1>(x)) { dep.insertEdge(iv(y), ent); }
        for (auto &y : std::get<2>(x)) { dep.insertEdge(ent, iv(y)); }
    }
    std::vector<std::string> order, open;
    for (auto &x : dep.order()) { order.push_back(x->data); }
    for (auto &x : dep.open()) { open.push_back(x->data); }
    std::sort(open.begin(), open.end());
    return to_string(std::make_pair(std::move(order), std::move(open)));
}

} // namespace

// }}}
// {{{ definition of TestSafetyCheck

void TestSafetyCheck::setUp() { }

void TestSafetyCheck::tearDown() { }

void TestSafetyCheck::test_safety() {
    CPPUNIT_ASSERT_EQUAL(std::string("([Y=1,X=Y],[])"), check({ T{"X=Y",{"Y"},{"X"}}, T{"Y=1",{},{"Y"}} }));
    CPPUNIT_ASSERT_EQUAL(std::string("([],[X,Y])"), check({ T{"0=X+Y",{"X","Y"}, {}} }));
    CPPUNIT_ASSERT_EQUAL(std::string("([A=1,B=A,C=B,D=C],[])"), check({ T{"A=1",{}, {"A"}}, T{"B=A",{"A"}, {"B"}}, T{"C=B",{"B"}, {"C"}}, T{"D=C",{"C"}, {"D"}} }));
    CPPUNIT_ASSERT_EQUAL(std::string("([],[A,B])"), check({ T{"A=B",{"B"}, {"A"}}, T{"B=A",{"A"}, {"B"}} }));
    CPPUNIT_ASSERT_EQUAL(std::string("([A=1,B=A,A=B],[])"), check({ T{"A=B",{"B"}, {"A"}}, T{"B=A",{"A"}, {"B"}}, T{"A=1",{}, {"A"}} }));
    CPPUNIT_ASSERT_EQUAL(std::string("([(X,Y)=(1,1),2=X+Y],[])"), check({ T{"(X,Y)=(1,1)",{}, {"X","Y"}}, T{"2=X+Y",{"X", "Y"}, {}} }));
}

TestSafetyCheck::~TestSafetyCheck() { }

// }}}

CPPUNIT_TEST_SUITE_REGISTRATION(TestSafetyCheck);

} } // namespace Test Gringo
