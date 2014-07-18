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

#include "gringo/ground/dependency.hh"
#include "gringo/input/nongroundparser.hh"

namespace Gringo { namespace Output { namespace Test {

// {{{ declaration of TestWarnings

class TestWarnings : public CppUnit::TestFixture {
    CPPUNIT_TEST_SUITE(TestWarnings);
        CPPUNIT_TEST(test_warnings);
    CPPUNIT_TEST_SUITE_END();
    using S = std::string;

public:
    virtual void setUp();
    virtual void tearDown();

    void test_warnings();

    virtual ~TestWarnings();
};

// }}}

// {{{ definition of TestWarnings

void TestWarnings::setUp() {
}

void TestWarnings::tearDown() {
}

void TestWarnings::test_warnings() {
    Gringo::Test::Messages msg;
    CPPUNIT_ASSERT_EQUAL(S("[[p(bot),q(0)]]"), IO::to_string(solve("p(bot).\nq(1/X):-p(X).\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:2:3-6: warning: operation undefined, a zero is substituted:\n  (1/X)\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[p(bot)],[p(bot),q(bot)]]"), IO::to_string(solve("p(bot).\n#sum{X:q(X):p(X)}.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:2:6-7: warning: #sum aggregate not defined for weight, tuple is ignored:\n  bot\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[p(bot)]]"), IO::to_string(solve("p(bot).\nx:-1#sum{X:p(X)}.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:2:10-11: warning: #sum aggregate not defined for weight, tuple is ignored:\n  bot\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[p(bot)]]"), IO::to_string(solve("p(bot).\nx:-1#sum{:p(X)}.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:2:4-16: warning: empty tuple in #sum aggregate, tuple is ignored\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("#show bot/0.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:1-13: warning: no matching occurrence for signature:\n  bot/0\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("x:-bot.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:4-7: warning: atom is undefined:\n  bot\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[p(bot)]]"), IO::to_string(solve("p(bot).\n:~ p(X). [X]\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:2:11-12: warning: weak constraint not defined for weight, tuple is ignored:\n  bot\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve("a:-#sum{-1:a;1:a}>=0.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:4-21: warning: negative weight in (potentially) recursive #sum aggregate:\n  (the applied translation might produce counter-intuitive results)\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve("a:-#sum{1:a;2:a}!=1.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:4-20: warning: holes in range of (potentially) recursive #sum aggregate:\n  (the applied translation might produce counter-intuitive results)\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(solve("a:-X=#sum{-1:a;1:a},X==0.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:4-20: warning: negative weight in (potentially) recursive #sum aggregate:\n  (the applied translation might produce counter-intuitive results)\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("#const a=b.\n#const b=a.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:1-12: warning: cyclic constant definition:\n  #const a=b.\n-:2:1-12: note: cycle involves definition:\n  #const b=a.\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("#const a=a.\n")));
    CPPUNIT_ASSERT_EQUAL(S("[]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[p(1)]]"), IO::to_string(solve("#const a=1.\n#const a=2.\np(a).\n")));
    CPPUNIT_ASSERT_EQUAL(S("[-:2:1-12: warning: redefinition of constant ignored:\n  #const a=2.\n-:1:1-12: note: constant first defined here\n]"), IO::to_string(msg));
    msg.clear();
    std::ofstream("/tmp/wincluded.lp");
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("#include \"/tmp/wincluded.lp\".#include \"/tmp/wincluded.lp\".")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:30-59: warning: already included file:\n  /tmp/wincluded.lp\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[x=1,y=-1,z=0]]"), IO::to_string(solve("$x $> 0.\n$y $< 0.\na:-$z $> 0.\n")));
    CPPUNIT_ASSERT_EQUAL(S(
        "[warning: unbounded constraint variable, domain is set to [1,1]:\n"
        "  x\n"
        ",warning: unbounded constraint variable, domain is set to [-1,-1]:\n"
        "  y\n"
        ",warning: unbounded constraint variable, domain is set to [0,0]:\n"
        "  z\n"
        "]"),
        IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("#show $y/0.")));
    CPPUNIT_ASSERT_EQUAL(S("[-:1:1-12: warning: no matching occurrence for signature:\n  $y/0\n]"), IO::to_string(msg));
    msg.clear();
    CPPUNIT_ASSERT_EQUAL(S("[[]]"), IO::to_string(solve("#show $y.")));
    CPPUNIT_ASSERT_EQUAL(S("[warning: trying to show constraint variable that does not occur in program:\n  $y\n]"), IO::to_string(msg));
    msg.clear();
}

TestWarnings::~TestWarnings() { }

// }}}

CPPUNIT_TEST_SUITE_REGISTRATION(TestWarnings);

} } } // namespace Test Output Gringo


