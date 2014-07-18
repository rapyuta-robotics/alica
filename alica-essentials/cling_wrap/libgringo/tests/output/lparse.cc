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

#include "tests/tests.hh"
#include "tests/term_helper.hh"
#include "tests/output/solver_helper.hh"

namespace Gringo { namespace Output { namespace Test {

// {{{ declaration of TestLparse

class TestLparse : public CppUnit::TestFixture {
    CPPUNIT_TEST_SUITE(TestLparse);
        CPPUNIT_TEST(test_empty);
        CPPUNIT_TEST(test_projectionBug);
        CPPUNIT_TEST(test_recAntiAggr);
        CPPUNIT_TEST(test_aggregateBug);
        CPPUNIT_TEST(test_aggregateRecBug);
        CPPUNIT_TEST(test_headAggregateBug);
        CPPUNIT_TEST(test_symTabBug);
        CPPUNIT_TEST(test_mutexBug);
        CPPUNIT_TEST(test_head);
        CPPUNIT_TEST(test_assign);
        CPPUNIT_TEST(test_conjunction);
        CPPUNIT_TEST(test_disjunction);
        CPPUNIT_TEST(test_show);
        CPPUNIT_TEST(test_aggregates);
        CPPUNIT_TEST(test_minimize);
        CPPUNIT_TEST(test_csp);
        CPPUNIT_TEST(test_cspbound);
        CPPUNIT_TEST(test_disjoint);
        CPPUNIT_TEST(test_queens);
        CPPUNIT_TEST(test_python);
        CPPUNIT_TEST(test_lua);
    CPPUNIT_TEST_SUITE_END();
    using S = std::string;

public:
    virtual void setUp();
    virtual void tearDown();

    void test_empty();
    void test_projectionBug();
    void test_recAntiAggr();
    void test_aggregateBug();
    void test_aggregateRecBug();
    void test_headAggregateBug();
    void test_symTabBug();
    void test_mutexBug();
    void test_head();
    void test_assign();
    void test_conjunction();
    void test_disjunction();
    void test_show();
    void test_aggregates();
    void test_csp();
    void test_cspbound();
    void test_disjoint();
    void test_queens();
    void test_python();
    void test_lua();
    void test_minimize();
    virtual ~TestLparse();
};

// }}}

// {{{ definition of TestLparse

void TestLparse::setUp() {
}

void TestLparse::tearDown() {
}

void TestLparse::test_empty() {
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("")));
}

void TestLparse::test_projectionBug() {
    CPPUNIT_ASSERT_EQUAL(S("[[p(1),p(2)]]"), IO::to_string(solve(
        "q((1,x),2).\n"
        "p(A) :- q((A,_),_).\n"
        "p(B) :- q((A,_),B).\n"
        , {"p("})));
}
void TestLparse::test_aggregateBug() {
    CPPUNIT_ASSERT_EQUAL(
        S("[[a(1),a(2),b(1)],[a(1),a(2),b(1),b(2)]]"), 
        IO::to_string(solve(
        "a(1)."
        "a(2)."
        "b(1)."
        "{ b(X) } :- a(X).")));
}
void TestLparse::test_aggregateRecBug() {
    Gringo::Test::Messages msg;
    CPPUNIT_ASSERT_EQUAL(
        S("[]"), 
        IO::to_string(solve(
        "a :- {a}!=1."
        )));
    CPPUNIT_ASSERT_EQUAL(
        S("[]"), 
        IO::to_string(solve(
        "a :- #sum {1:a}!=1."
        )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[b]]"), 
        IO::to_string(solve(
            "b :- 0  #sum+ { 1: b }."
        )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[b]]"), 
        IO::to_string(solve(
            "b :- 0  #sum { 1: b }."
        )));
}
void TestLparse::test_symTabBug() {
    Gringo::Test::Messages msg;
    CPPUNIT_ASSERT_EQUAL(
        S("[[does(a,0),does(a,1)],[does(a,0),does(b,1)]]"),
        IO::to_string(solve(
            "time(0..1).\n"
            "1 { does(M,T) : legal(M,T) } 1 :- time(T).\n"
            "legal(a,T) :- time(T).\n"
            "legal(b,T) :- does(a,0), time(T).\n", 
            {"does"})));
}
void TestLparse::test_recAntiAggr() {
    CPPUNIT_ASSERT_EQUAL(
        S("[[p],[r]]"),
        IO::to_string(solve(
            "r :- #sum { 1:p } < 1.\n"
            "p :- not r.")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[p]]"),
        IO::to_string(solve("p :- #sum { 1:not p } < 1.")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[p]]"),
        IO::to_string(solve("p :- not #sum { 1:p } < 1.")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[p]]"),
        IO::to_string(solve("p :- not #sum { 1:not p } > 0.")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[p]]"),
        IO::to_string(solve("p :- not not #sum { 1:p } > 0.")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[p]]"),
        IO::to_string(solve("p :- not not #sum { 1:not p } < 1.")));
}
void TestLparse::test_mutexBug() {
#ifdef WITH_LUA
    CPPUNIT_ASSERT_EQUAL(
        S("[[]]"), 
        IO::to_string(solve(
            "#script (lua) \n"
            "function main(prg)\n"
            "	prg:ground(\"base\", {})\n"
            "	prg:ground(\"plan_graph_base\", {})\n"
            "\n"
            "	for step = 1,3,1 do\n"
            "		prg:ground(\"plan_graph_step\", {step})\n"
            "		prg:solve()\n"
            "	end\n"
            "--	Comment line 8 and uncomment line 11\n"
            "--	prg:solve()\n"
            "end\n"
            "#end.\n"
            "\n"
            "#program plan_graph_base.\n"
            "valid_f(FLUENT, 1) :- init(FLUENT).\n"
            "valid_f1(FLUENT, 1) :- init(FLUENT).\n"
            "\n"
            "#program plan_graph_step(time).\n"
            "\n"
            "%Encoding 1\n"
            "\n"
            "valid_a(ACT, time) :- action(ACT); valid_f(FLUENT, time) : pre(ACT, FLUENT);\n"
            "			not mutex(F1, F2, time) : req_both(ACT, F1, F2).\n"
            "\n"
            "valid_pre(ACT, FLUENT, time) :- valid_a(ACT, time), pre(ACT, FLUENT).\n"
            "valid_add(ACT, FLUENT, time) :- valid_a(ACT, time), addadd(ACT, FLUENT).\n"
            "valid_f(FLUENT, time + 1) :- valid_add(_, FLUENT, time).\n"
            "\n"
            "mutex_actions(A1, A2, time) :- valid_a(A1, time), valid_a(A2, time), conflicting(A1, A2).\n"
            "\n"
            "mutex_a_with_f(A1, P2, time) :- mutex(P1, P2, time), valid_pre(A1, P1, time).\n"
            "mutex_actions(A1, A2, time) :- mutex_a_with_f(A1, P2, time), valid_pre(A2, P2, time).\n"
            "%mutex_actions(A1, A2, time) :- mutex(P1, P2, time), valid_pre(A1, P1, time), valid_pre(A2, P2, time).\n"
            "\n"
            "mutex_a_sym(A1, A2, time; A2, A1, time) :- mutex_actions(A1, A2, time).\n"
            "\n"
            "closes(A2, F1, time) :- valid_a(A2, time), valid_f(F1, time+1); mutex_a_sym(A1, A2, time) : valid_add(A1, F1, time).\n"
            "mutex(F1, F2, time+1) :- F1 < F2, valid_f(F1, time+1), valid_f(F2, time+1);\n"
            "			closes(A2, F1, time) : valid_add(A2, F2, time).\n"
            "\n"
            "% Encoding 2:\n"
            "% Identical to encoding 1 except for lines 32 and 33 vs. line 57.\n"
            "% However these should generate identical groundings, but they don't for some reason\n"
            "\n"
            "valid_a1(ACT, time) :- action(ACT); valid_f1(FLUENT, time) : pre(ACT, FLUENT);\n"
            "			not mutex1(F1, F2, time) : req_both(ACT, F1, F2).\n"
            "\n"
            "valid_pre1(ACT, FLUENT, time) :- valid_a1(ACT, time), pre(ACT, FLUENT).\n"
            "valid_add1(ACT, FLUENT, time) :- valid_a1(ACT, time), addadd(ACT, FLUENT).\n"
            "valid_f1(FLUENT, time + 1) :- valid_add1(_, FLUENT, time).\n"
            "\n"
            "mutex_actions1(A1, A2, time) :- valid_a1(A1, time), valid_a1(A2, time), conflicting(A1, A2).\n"
            "\n"
            "%mutex_a_with_f1(A1, P2, time) :- mutex1(P1, P2, time), valid_pre1(A1, P1, time).\n"
            "%mutex_actions1(A1, A2, time) :- mutex_a_with_f1(A1, P2, time), valid_pre1(A2, P2, time).\n"
            "mutex_actions1(A1, A2, time) :- mutex1(P1, P2, time), valid_pre1(A1, P1, time), valid_pre1(A2, P2, time).\n"
            "\n"
            "mutex_a_sym1(A1, A2, time; A2, A1, time) :- mutex_actions1(A1, A2, time).\n"
            "\n"
            "closes1(A2, F1, time) :- valid_a1(A2, time), valid_f1(F1, time+1); mutex_a_sym1(A1, A2, time) : valid_add1(A1, F1, time).\n"
            "mutex1(F1, F2, time+1) :- F1 < F2, valid_f1(F1, time+1), valid_f1(F2, time+1);\n"
            "			closes1(A2, F1, time) : valid_add1(A2, F2, time).\n"
            "\n"
            "% diff reports if the two encodings are not the same\n"
            "\n"
            "diff(A1, A2, time) :- mutex_actions1(A1, A2, time), not mutex_actions(A1, A2, time).\n"
            "diff(A1, A2, time) :- mutex_actions(A1, A2, time), not mutex_actions1(A1, A2, time).\n"
            "\n"
            "#program base.\n"
            "\n"
            "#show diff/3.\n"
            "\n"
            "preserve_action(preserve(F)) :- fact(F).\n"
            "action(A) :- preserve_action(A).\n"
            "pre(preserve(F), F) :- fact(F).\n"
            "addadd(preserve(F), F) :- fact(F).\n"
            "\n"
            "prepre(A, F) :- pre(A, F), not del(A, F).\n"
            "deldel(A, F) :- del(A, F), not pre(A, F).\n"
            "predel(A, F) :- pre(A, F), del(A, F).\n"
            "addadd(A, F) :- add(A, F), not pre(A, F), not del(A, F).\n"
            "\n"
            "conflicting(A1, A2) :- pre(A1, F), del(A2, F), A1 != A2.\n"
            "\n"
            "req_both(ACT, F1, F2) :- pre(ACT, F1), pre(ACT, F2), F1 < F2.\n"
            "\n"
            "type(object).\n"
            "type(ferry, object).\n"
            "fact(at_ferry(X)) :- type(X, object).\n"
            "fact(at(X, Y)) :- type(X, object), type(Y, object).\n"
            "fact(on(X, Y)) :- type(X, object), type(Y, object).\n"
            "fact(empty_ferry).\n"
            "action(debark(X, Y)) :- type(X, object), type(Y, object), auto__(X), place__(Y).\n"
            "pre(debark(X, Y), on(X, ferry)) :- action(debark(X, Y)).\n"
            "pre(debark(X, Y), at_ferry(Y)) :- action(debark(X, Y)).\n"
            "add(debark(X, Y), at(X, Y)) :- action(debark(X, Y)).\n"
            "add(debark(X, Y), empty_ferry) :- action(debark(X, Y)).\n"
            "del(debark(X, Y), on(X, ferry)) :- action(debark(X, Y)).\n"
            "action(sail(X, Y)) :- type(X, object), type(Y, object), place__(X), place__(Y).\n"
            "pre(sail(X, Y), at_ferry(X)) :- action(sail(X, Y)).\n"
            "add(sail(X, Y), at_ferry(Y)) :- action(sail(X, Y)).\n"
            "del(sail(X, Y), at_ferry(X)) :- action(sail(X, Y)).\n"
            "action(board(X, Y)) :- type(X, object), type(Y, object), place__(Y), auto__(X).\n"
            "pre(board(X, Y), at(X, Y)) :- action(board(X, Y)).\n"
            "pre(board(X, Y), at_ferry(Y)) :- action(board(X, Y)).\n"
            "pre(board(X, Y), empty_ferry) :- action(board(X, Y)).\n"
            "add(board(X, Y), on(X, ferry)) :- action(board(X, Y)).\n"
            "del(board(X, Y), at(X, Y)) :- action(board(X, Y)).\n"
            "del(board(X, Y), empty_ferry) :- action(board(X, Y)).\n"
            "type(c1, object).\n"
            "type(b, object).\n"
            "type(c2, object).\n"
            "type(a, object).\n"
            "place__(a).\n"
            "place__(b).\n"
            "auto__(c1).\n"
            "auto__(c2).\n"
            "init(at(c1, a)).\n"
            "init(at(c2, a)).\n"
            "init(at_ferry(a)).\n"
            "init(empty_ferry).\n"
            "goal(at(c1, b)).\n"
            "goal(at(c2, b)).\n"
            )));
#endif // WITH_LUA
}
void TestLparse::test_headAggregateBug() {
    Gringo::Test::Messages msg;
    CPPUNIT_ASSERT_EQUAL(S("[[q(a),r(a)]]"), IO::to_string(solve("1 { q(a); p(X) : z(X) }. r(X) :- q(X).")));
    CPPUNIT_ASSERT_EQUAL(S("[[q(a),r(a)]]"), IO::to_string(solve("1 { p(X) : z(X); q(a) }. r(X) :- q(X).")));
}
void TestLparse::test_head() {
    CPPUNIT_ASSERT_EQUAL(S("[[],[a],[a,b],[b]]"), IO::to_string(solve("{a;b}.")));
    CPPUNIT_ASSERT_EQUAL(S("[[a],[b]]"), IO::to_string(solve("1{a;b}1.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p(1)],[p(1),p(2)],[p(1),p(3)],[p(1),p(4)],[p(2)],[p(2),p(3)],[p(2),p(4)],[p(3)],[p(3),p(4)],[p(4)]]"), IO::to_string(solve("1#count{X:p(X):X=1..4}2.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p(1)],[p(2)]]"), IO::to_string(solve("1#sum+{X:p(X):X=1..4}2.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p(1)],[p(2)]]"), IO::to_string(solve("1#sum {X:p(X):X=1..4}2.")));
    CPPUNIT_ASSERT_EQUAL(S(
        "[[p(1)],[p(1),p(2)],[p(1),p(2),p(3)],[p(1),p(2),p(3),p(4)],[p(1),p(2),p(4)],[p(1),p(3)],[p(1),p(3),p(4)],[p(1),p(4)],"
        "[p(2)],[p(2),p(3)],[p(2),p(3),p(4)],[p(2),p(4)]]"),
        IO::to_string(solve("1#min{X:p(X):X=1..4}2.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p(1)],[p(1),p(2)],[p(2)]]"), IO::to_string(solve("1#max{X:p(X):X=1..4}2.")));
    CPPUNIT_ASSERT_EQUAL(S("[[c,p]]"), IO::to_string(solve("{p}. 1 {c:p}.")));
}

void TestLparse::test_assign() {
    CPPUNIT_ASSERT_EQUAL(S("[[p,q(1)],[q(0)]]"),IO::to_string(solve("{p}. q(M):-M=#count{1:p}.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p,q(1)],[q(0)]]"),IO::to_string(solve("{p}. q(M):-M=#sum+{1:p}.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p,q(1)],[q(0)]]"),IO::to_string(solve("{p}. q(M):-M=#sum{1:p}.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p,q(p)],[q(#sup)]]"),IO::to_string(solve("{p}. q(M):-M=#min{p:p}.")));
    CPPUNIT_ASSERT_EQUAL(S("[[p,q(p)],[q(#inf)]]"),IO::to_string(solve("{p}. q(M):-M=#max{p:p}.")));
    CPPUNIT_ASSERT_EQUAL(S(
        "[[p(1),p(2),q(1)],"
        "[p(1),p(3),q(1)],"
        "[p(1),p(4),q(1)],"
        "[p(2),p(3),q(2)],"
        "[p(2),p(4),q(2)],"
        "[p(3),p(4),q(3)]]"), 
        IO::to_string(solve("2{p(1..4)}2. q(M):-M=#min{X:p(X)}.")));
    CPPUNIT_ASSERT_EQUAL(S(
        "[[p(1),p(2),q(2)],"
        "[p(1),p(3),q(3)],"
        "[p(1),p(4),q(4)],"
        "[p(2),p(3),q(3)],"
        "[p(2),p(4),q(4)],"
        "[p(3),p(4),q(4)]]"), 
        IO::to_string(solve("2{p(1..4)}2. q(M):-M=#max{X:p(X)}.")));
}

void TestLparse::test_conjunction() {
    Gringo::Test::Messages msg;
    CPPUNIT_ASSERT_EQUAL(
        S("[]"),
        IO::to_string(solve(
            "a:-b:c.\n"
            "c:-a.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:4-5: warning: atom is undefined:\n  b\n]"), IO::to_string(msg));
    CPPUNIT_ASSERT_EQUAL(
        S("[[a,b,c]]"),
        IO::to_string(solve(
            "a:-b:c.\n"
            "c:-a.\n"
            "b:-c.\n", {"a", "b", "c"})));
    CPPUNIT_ASSERT_EQUAL(
        S("[[a,b,c,d]]"),
        IO::to_string(solve(
            "a:-b:c,d.\n"
            "c:-a.\n"
            "d:-a.\n"
            "b:-c.\n"
            "b:-d.\n", {"a","b","c","d"})));
    CPPUNIT_ASSERT_EQUAL(
        S("[[a(1),a(2),a(3),c,q],[a(3)]]"),
        IO::to_string(solve(
            "{c}.\n"
            "a(1):-c.\n"
            "a(2):-c.\n"
            "a(3).\n"
            "q:-a(X):X=1..3.\n")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[p],[q]]"),
        IO::to_string(solve(
            "p :- p:q.\n"
            "q :- q:p.\n")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[p,q]]"),
        IO::to_string(solve(
            "p :- p:q.\n"
            "q :- q:p.\n"
            "p :- q.\n"
            "q :- p.\n")));
}

void TestLparse::test_disjunction() {
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[a,b,c,d],[c,x,y]]"),
        IO::to_string(solve(
            "{ y; d } 1.\n"
            "c :- y.\n"
            "c :- d.\n"
            "b :- d.\n"
            "x:y | a:b :- c.\n"
            "b :- a.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[a,b,c,d,x5]]"),
        IO::to_string(solve(
            "x5:-b.\n"
            "x5:-not c.\n"
            "d:-c.\n"
            "c:-d.\n"
            "\n"
            "x5|d:-not not b.\n"
            "a:-x5.\n"
            "\n"
            "c:-a.\n"
            "b:-c.\n"
            )));

    CPPUNIT_ASSERT_EQUAL(
        S("[[a,b,c],[b],[b,c],[c]]"),
        IO::to_string(solve(
            "1{b;c}.\n"
            "a:b,c;not a.\n")));
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[p(1)],[p(1),p(2)],[p(2)]]"),
        IO::to_string(solve(
            "q(1..2).\n"
            "p(X); not p(X) :- q(X).\n", {"p("})));
    CPPUNIT_ASSERT_EQUAL(
        S("[[],[p(1),r(1)],[r(1)]]"),
        IO::to_string(solve(
            "q(1).\n"
            "p(X); not p(X); not r(X) :- q(X).\n"
            "r(X); not r(X) :- q(X).\n", {"p(", "r("})));
}

void TestLparse::test_show() {
    CPPUNIT_ASSERT_EQUAL(
        S(
            "["
            "[(1,2,3),-q(1),42],"
            "[(1,2,3),-q(1),42],"
            "[(1,2,3),-q(1),42,p(1)],"
            "[(1,2,3),42],"
            "[(1,2,3),42],"
            "[(1,2,3),42,boo(1)],"
            "[(1,2,3),42,boo(1)],"
            "[(1,2,3),42,boo(1),p(1)],"
            "[(1,2,3),42,p(1)]"
            "]"),
        IO::to_string(solve(
            "#show p/1.\n"
            "#show -q/1.\n"
            "#show boo(X):q(X).\n"
            "#show -p/-1.\n"
            "#show (1,2,3).\n"
            "\n"
            "{p(1); q(1); -p(1); -q(1)}.\n"
            "\n"
            "#const p=42.\n")));
    CPPUNIT_ASSERT_EQUAL(
        S(
            "[[a,c,x=1,y=1]]"),
        IO::to_string(solve(
            "a. b.\n"
            "$x $= 1. $y $= 1. $z $= 1.\n"
            "#show a/0.\n"
            "#show c.\n"
            "#show $x/0.\n"
            "#show $y.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S(
            "[[x=1],[y=1]]"),
        IO::to_string(solve(
            "{b}.\n"
            "$x $= 1. $y $= 1.\n"
            "#show.\n"
            "#show $x:b.\n"
            "#show $y:not b.\n"
            )));
}

void TestLparse::test_aggregates() {
    CPPUNIT_ASSERT_EQUAL(
        S(
            "["
            "[]"
            "]"),
        IO::to_string(solve(
            "#sum { 1:b; 2:c } < 1.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(S("[[p(1),p(2)],[p(1),p(3)],[p(2),p(3)]]"), IO::to_string(solve("{p(1..3)}.\n:-{p(X)}!=2.")));
    CPPUNIT_ASSERT_EQUAL(S("[[],[a,b],[b]]"), IO::to_string(solve("#sum { -1:a; 1:b } >= 0.")));
    CPPUNIT_ASSERT_EQUAL(S("[[],[a,b],[b]]"), IO::to_string(solve("#sum { 1:a; 2:b } != 1.")));
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve("a. {a} 0.")));
}

void TestLparse::test_minimize() {
    std::string prg;
    prg =
        "{a; b; c; d}.\n"
        "#minimize {1,a:a; 1,b:b; 1,c:c; 1,d:d}.\n"
        "ok :- a,    b,not c,not d.\n"
        "ok :- not a,b,    c,    d.\n"
        "ok :- a,not b,    c,    d.\n"
        ":- not ok.\n";
    CPPUNIT_ASSERT_EQUAL(S("[[a,b]]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {2})));
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {1})));
    prg =
        "{a; b; c; d}.\n"
        ":~ a. [1,a]\n"
        ":~ b. [1,b]\n"
        ":~ c. [1,c]\n"
        ":~ d. [1,d]\n"
        "ok :- a,    b,not c,not d.\n"
        "ok :- not a,b,    c,    d.\n"
        "ok :- a,not b,    c,    d.\n"
        ":- not ok.\n";
    CPPUNIT_ASSERT_EQUAL(S("[[a,b]]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {2})));
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {1})));
    prg =
        "{a; b; c; d}.\n"
        "#maximize {-1,a:a; -1,b:b; -1,c:c; -1,d:d}.\n"
        "ok :- a,    b,not c,not d.\n"
        "ok :- not a,b,    c,    d.\n"
        "ok :- a,not b,    c,    d.\n"
        ":- not ok.\n";
    CPPUNIT_ASSERT_EQUAL(S("[[a,b]]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {2})));
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {1})));
    prg = 
        "{a; b; c; d}.\n"
        "#minimize {3,a:a; 3,b:b; 1,x:c; 1,x:d}.\n"
        "ok :- a,    b,not c,not d.\n"
        "ok :- not a,b,    c,    d.\n"
        "ok :- a,not b,    c,    d.\n"
        ":- not ok.\n"
        ;
    CPPUNIT_ASSERT_EQUAL(S("[[a,c,d],[b,c,d]]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {4})));
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {3})));
    prg = 
        "{a; b; c; d}.\n"
        "#minimize {3@2,a:a; 3@2,b:b; 1@2,x:c; 1@2,x:d}.\n"
        "#minimize {1@1,x:a; 1@1,x:c; 1@1,y:b}.\n"
        "ok :- a,    b,not c,not d.\n"
        "ok :- not a,b,    c,    d.\n"
        "ok :- a,not b,    c,    d.\n"
        ":- not ok.\n"
        ;
    CPPUNIT_ASSERT_EQUAL(S("[[a,c,d]]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {4, 1})));
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve(S(prg), {"a", "b", "c", "d"}, {4, 0})));
}

void TestLparse::test_csp() {
    CPPUNIT_ASSERT_EQUAL(
        S("[[p(1)=1,p(2)=1,x=1],[p(1)=1,p(2)=2,x=1],[p(1)=2,p(2)=1,x=1],[p(1)=2,p(2)=2,x=1]]"),
        IO::to_string(solve(
            "1 $<= $p(1..2) $<= 2.\n"
            "$x $= 1.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=0,y=0,z=2],[x=0,y=0,z=3],[x=0,y=1,z=3],[x=1,y=0,z=3]]"),
        IO::to_string(solve(
            "0 $<= $(x;y;z) $<= 3.\n"
            "$x $+ $y $+ -1$*$z $<= -2.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=0,y=0,z=2],[x=0,y=0,z=3],[x=0,y=1,z=3],[x=1,y=0,z=3]]"),
        IO::to_string(solve(
            "0 $<= $(x;y;z) $<= 3.\n"
            "p:-$x $+ $y $+ -1$*$z $<= -2.\n"
            ":- not p.\n", {"x", "y", "z"}
            )));
}

void TestLparse::test_cspbound() {
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=1],[x=2],[x=3],[x=5],[x=7],[x=8],[x=9]]"),
        IO::to_string(solve(
            "X $<= $x $<= Y : b(X,Y).\n"
            "b(1,2).\n"
            "b(3,3).\n"
            "b(5,5).\n"
            "b(7,9).\n", {"x="}
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=10],[x=3],[x=7]]"),
        IO::to_string(solve(
            "X $<= $x $<= Y : b(A,X,Y) :- b(A).\n"
            "b(1;2).\n"
            "b(1,1,3).\n"
            "b(1,6,10).\n"
            "b(2,3,4).\n"
            "b(2,7,7).\n"
            "b(2,10,11).\n", {"x="}
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=4],[x=5]]"),
        IO::to_string(solve(
            "$x $<= 5.\n"
            ":- $x $<= 3, $x $<=4.\n", {"x="}
            )));
}

void TestLparse::test_disjoint() {
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=2]]"),
        IO::to_string(solve(
            "1 $<= $x $<= 2.\n"
            "#disjoint{ 1:1; 2:$x }.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[a,x=1],[b,x=1],[x=1]]"),
        IO::to_string(solve(
            "$x $= 1.\n"
            "{ a; b }.\n"
            "#disjoint{ a:$x:a; b:$x:b }.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[a,b,y=2],[a,y=2],[b,y=2],[y=1],[y=2]]"),
        IO::to_string(solve(
            "1 $<= $y $<= 2.\n"
            "{ a; b }.\n"
            "#disjoint{ 1:1:a; 1:1:b; 2:$y }.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[p(1)=1,p(2)=1,p(3)=1,q(1)=2,q(2)=2,q(3)=2],[p(1)=2,p(2)=2,p(3)=2,q(1)=1,q(2)=1,q(3)=1]]"),
        IO::to_string(solve(
            "1 $<= $(p(1..3);q(1..3)) $<= 2.\n"
            "#disjoint{ 1:$p(1..3); 2:$q(1..3) }.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=6,y=35]]"),
        IO::to_string(solve(
            "6  $<= $x $<=  7.\n"
            "35 $<= $y $<= 36.\n"
            "not #disjoint{ 1:6$*$y; 2:35$*$x }.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S(
            "[[x=1,y=1,z=1]"
            ",[x=2,y=2,z=2]"
            ",[x=3,y=3,z=3]]"),
        IO::to_string(solve(
            "1  $<= $(x;y;z) $<=  3.\n"
            "not #disjoint{ 1:2$*$x $+ 3$*$y; 2:2$*$y $+ 3$*$z; 3:2$*$z $+ 3$*$x }.\n"
            )));
    CPPUNIT_ASSERT_EQUAL(
        S("[[x=6,y=35]]"),
        IO::to_string(solve(
            "6  $<= $x $<=  7.\n"
            "35 $<= $y $<= 36.\n"
            "not #disjoint{ 1:6$*$y; 2:35$*$x }.\n"
            )));
}

void TestLparse::test_queens() {
    CPPUNIT_ASSERT_EQUAL(
        S("[[q(1,2),q(2,4),q(3,6),q(4,1),q(5,3),q(6,5)],"
          "[q(1,3),q(2,6),q(3,2),q(4,5),q(5,1),q(6,4)],"
          "[q(1,4),q(2,1),q(3,5),q(4,2),q(5,6),q(6,3)],"
          "[q(1,5),q(2,3),q(3,1),q(4,6),q(5,4),q(6,2)]]"),
        IO::to_string(solve(
            "#const n = 6.\n"
            "n(1..n).\n"
            "\n"
            "q(X,Y) :- n(X;Y), not not q(X,Y).\n"
            "\n"
            "        c(r,X; c,Y) :- q(X,Y).\n"
            "not not c(r,N; c,N) :- n(N).\n"
            "\n"
            "n(r,X,Y-1,X,Y; c,X-1,Y,X,Y; d1,X-1,Y-1,X,Y;     d2,X-1,Y+1,X,Y      ) :- n(X;Y).\n"
            "c(r,N,0;       c,0,N;       d1,N-1,0; d1,0,N-1; d2,N-1,n+1; d2,0,N+1) :- n(N).\n"
            "\n"
            "c(C,XX,YY) :-     c(C,X,Y), n(C,X,Y,XX,YY), not q(XX,YY).\n"
            "           :- not c(C,X,Y), n(C,X,Y,XX,YY),     q(XX,YY).\n", {"q("})));
    CPPUNIT_ASSERT_EQUAL( 
        48,
        (int)solve(
            "#const n=4.\n"
            "1 $<= $(row(X);col(X)) $<= n :- X=1..n.\n"
            "$row(X) $!= $row(Y) :- X=1..n, Y=1..n, X<Y.\n"
            "$col(X) $!= $col(Y) :- X=1..n, Y=1..n, X<Y.\n"
            "$row(X) $+ $col(Y) $!= $row(Y) $+ $col(X) :- X=1..n, Y=1..n, X<Y.\n"
            "$row(X) $+ $col(X) $!= $row(Y) $+ $col(Y) :- X=1..n, Y=1..n, X<Y.\n"
            ).size());
    S q5 = 
        "[[q(1)=1,q(2)=3,q(3)=5,q(4)=2,q(5)=4]"
        ",[q(1)=1,q(2)=4,q(3)=2,q(4)=5,q(5)=3]"
        ",[q(1)=2,q(2)=4,q(3)=1,q(4)=3,q(5)=5]"
        ",[q(1)=2,q(2)=5,q(3)=3,q(4)=1,q(5)=4]"
        ",[q(1)=3,q(2)=1,q(3)=4,q(4)=2,q(5)=5]"
        ",[q(1)=3,q(2)=5,q(3)=2,q(4)=4,q(5)=1]"
        ",[q(1)=4,q(2)=1,q(3)=3,q(4)=5,q(5)=2]"
        ",[q(1)=4,q(2)=2,q(3)=5,q(4)=3,q(5)=1]"
        ",[q(1)=5,q(2)=2,q(3)=4,q(4)=1,q(5)=3]"
        ",[q(1)=5,q(2)=3,q(3)=1,q(4)=4,q(5)=2]]";
    CPPUNIT_ASSERT_EQUAL( 
        S(q5),
        IO::to_string(solve(
            "#const n=5.\n"
            "1 $<= $q(1..n) $<= n.\n"
            "$q(X) $!= $q(Y) :- X=1..n, Y=1..n, X<Y.\n"
            "X $+ $q(Y) $!= Y $+ $q(X) :- X=1..n, Y=1..n, X<Y.\n"
            "X $+ $q(X) $!= Y $+ $q(Y) :- X=1..n, Y=1..n, X<Y.\n"
            )));
    CPPUNIT_ASSERT_EQUAL( 
        S(q5),
        IO::to_string(solve(
            "#const n=5.\n"
            "1 $<= $q(1..n) $<= n.\n"
            "#disjoint { X : $q(X)$+0 : X=1..n }.\n"
            "#disjoint { X : $q(X)$+X : X=1..n }.\n"
            "#disjoint { X : $q(X)$-X : X=1..n }.\n"
            )));
    CPPUNIT_ASSERT_EQUAL( 
        S(q5),
        IO::to_string(solve(
            "#const n=5.\n"
            "1 $<= $q(1..n) $<= n.\n"
            "#disjoint { X : $q(X)     : X=1..n }.\n"
            ":- not #disjoint { X : $q(X)$+ X : X=1..n }.\n"
            "not not #disjoint { X : $q(X)$+-X : X=1..n }.\n"
            )));
}

void TestLparse::test_python() {
#ifdef WITH_PYTHON
    CPPUNIT_ASSERT_EQUAL(
        S(
            "["
            "[p(39),q(\"a\"),q(1),q(2),q(a),r(2),r(3),s((1,2)),s((1,3)),s((2,1))]"
            "]"),
        IO::to_string(solve(
            "#script (python)\n"
            "import gringo\n"
            "def gcd(a, b): return b if a == 0 else gcd(b % a, a)\n"
            "def test():    return [1, 2, gringo.Fun(\"a\"), \"a\"]\n"
            "def match():   return [(1,2),(1,3),(2,1)]\n"
            "#end.\n"
            "\n"
            "p(@gcd(2*3*7*13,3*11*13)).\n"
            "q(@test()).\n"
            "r(X) :- (1,X)=@match().\n"
            "s(X) :- X=@match().\n"
            )));
#endif // WITH_PYTHON
}

void TestLparse::test_lua() {
#ifdef WITH_LUA
    CPPUNIT_ASSERT_EQUAL(
        S(
            "["
            "[p(39),q(\"a\"),q(1),q(2),q(a),r(2),r(3),s((1,2)),s((1,3)),s((2,1))]"
            "]"),
        IO::to_string(solve(
            "#script (lua)\n"
            "function gcd(a, b) if a == 0 then return b else return gcd(b % a, a) end end\n"
            "function test()    return {1, 2, gringo.Fun(\"a\"), \"a\"} end\n"
            "function match()   return {gringo.Tuple({1,2}),gringo.Tuple({1,3}),gringo.Tuple({2,1})} end\n"
            "#end.\n"
            "\n"
            "p(@gcd(2*3*7*13,3*11*13)).\n"
            "q(@test()).\n"
            "r(X) :- (1,X)=@match().\n"
            "s(X) :- X=@match().\n"
            )));
#endif // WITH_LUA
}

TestLparse::~TestLparse() { }

// }}}

CPPUNIT_TEST_SUITE_REGISTRATION(TestLparse);

} } } // namespace Test Output Gringo

