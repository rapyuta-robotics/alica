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

#include "gringo/bug.hh"
#include "tests/tests.hh"
#include "tests/term_helper.hh"

#include <climits>
#include <sstream>
#include <functional>

namespace Gringo { namespace Test {

// {{{ declaration of TestTerm

class TestTerm : public CppUnit::TestFixture {
    CPPUNIT_TEST_SUITE(TestTerm);
        CPPUNIT_TEST(test_hash);
        CPPUNIT_TEST(test_equal);
        CPPUNIT_TEST(test_eval);
        CPPUNIT_TEST(test_rewriteArithmetics);
        CPPUNIT_TEST(test_rewriteDots);
        CPPUNIT_TEST(test_simplify);
        CPPUNIT_TEST(test_unpool);
        CPPUNIT_TEST(test_rewrite);
        CPPUNIT_TEST(test_undefined);
        CPPUNIT_TEST(test_project);
        CPPUNIT_TEST(test_match);
    CPPUNIT_TEST_SUITE_END();

public:
    virtual void setUp();
    virtual void tearDown();

    void test_hash();
    void test_equal();
    void test_eval();
    void test_rewriteArithmetics();
    void test_rewriteDots();
    void test_unpool();
    void test_simplify();
    void test_rewrite();
    void test_undefined();
    void test_project();
    void test_match();

    virtual ~TestTerm();

    std::vector<std::string> messages;
    std::unique_ptr<MessagePrinter> oldPrinter;
};

// }}}

using namespace Gringo::IO;

// {{{ definition of helpers to shorten the tests

namespace {

typedef Value V;
typedef std::string S;

size_t hash(UTerm const &x) {
    return x->gterm()->hash();
}

UGTerm gterm(UTerm const &x) {
    return x->gterm();
}

std::tuple<UTerm, Term::DotsMap, Term::ScriptMap> rewriteDots(UTerm &&x) {
    Term::DotsMap dots;
    Term::ScriptMap scripts;
    unsigned int n = 0;
    x->simplify(dots, scripts, n, true, false).update(x);
    return std::make_tuple(std::move(x), std::move(dots), std::move(scripts));
}

std::string rewriteProject(UTerm &&x) {
    UTerm sig{make_locatable<ValTerm>(x->loc(), "#p") };
    Term::DotsMap dots;
    Term::ScriptMap scripts;
    unsigned int n = 0;
    x->simplify(dots, scripts, n, true, false).update(x);
    auto ret(x->project(sig.get(), n));
    Term::replace(x, std::move(std::get<0>(ret)));
    auto projected(std::move(std::get<1>(ret)));
    auto project(std::move(std::get<2>(ret)));
    return to_string(std::make_tuple(
        std::move(x), 
        projected ? std::move(projected) : val("#undef"), 
        project ? std::move(project) : val("#undef")));
}

std::string rewriteArithmetics(UTerm &&x) {
    Term::ArithmeticsMap vec;
    vec.emplace_back();
    unsigned int n = 0;
    UTerm y(x->rewriteArithmetics(vec, n));
    return to_string(std::make_pair(y ? std::move(y) : std::move(x), std::move(vec)));
}

UTerm simplify(UTerm &&x) {
    Term::DotsMap dots;
    Term::ScriptMap scripts;
    unsigned int n = 0;
    x->simplify(dots, scripts, n, true, false).update(x);
    return std::move(x);
}

std::vector<std::tuple<UTerm, Term::DotsMap, Term::ArithmeticsMap>> rewrite(UTerm &&x) {
    unsigned auxNum = 0;
    std::vector<std::tuple<UTerm, Term::DotsMap, Term::ArithmeticsMap>> res;
    for(auto &term : unpool(x)) {
        Term::DotsMap dots;
        Term::ScriptMap scripts;
        Term::ArithmeticsMap arith;
        arith.emplace_back();
        term->simplify(dots, scripts, auxNum, true, false).update(term);
        Term::replace(term, term->rewriteArithmetics(arith, auxNum));
        res.emplace_back(std::move(term), std::move(dots), std::move(arith));
    }
    return std::move(res);
}

UTerm bindVars(UTerm &&x) {
    unsigned auxNum = 0;
	Term::DotsMap dots;
    Term::ScriptMap scripts;
	Term::ArithmeticsMap arith;
	Term::VarSet set;
	arith.emplace_back();
	x->simplify(dots, scripts, auxNum, true, false).update(x);
	Term::replace(x, x->rewriteArithmetics(arith, auxNum));
	x->bind(set);
    return std::move(x);
}

} // namespace

// }}}
// {{{ definition of TestTerm

void TestTerm::setUp() {
    oldPrinter = std::move(message_printer());
    message_printer() = make_unique<TestMessagePrinter>(messages);
}

void TestTerm::tearDown() {
    std::swap(message_printer(), oldPrinter);
}

void TestTerm::test_hash() {
    char const *msg = "warning: hashes are very unlikely to collide";
    // term
    CPPUNIT_ASSERT_MESSAGE(msg, val(1)->hash() == val(1)->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(val(1)->hash() == val(2)->hash()));
    CPPUNIT_ASSERT_MESSAGE(msg, var("X")->hash() == var("X")->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(var("X")->hash() == var("Y")->hash()));
    CPPUNIT_ASSERT_MESSAGE(msg, binop(BinOp::ADD, val(1), val(2))->hash() == binop(BinOp::ADD, val(1), val(2))->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(binop(BinOp::ADD, val(1), val(2))->hash() == binop(BinOp::ADD, val(2), val(1))->hash()));
    CPPUNIT_ASSERT_MESSAGE(msg, unop(UnOp::NEG, val(1))->hash() == unop(UnOp::NEG, val(1))->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(unop(UnOp::NEG, val(1))->hash() == unop(UnOp::ABS, val(1))->hash()));
    CPPUNIT_ASSERT_MESSAGE(msg, fun("f", val(1), val(2))->hash() == fun("f", val(1), val(2))->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(fun("f", val(1), val(2))->hash() == fun("g", val(1), val(2))->hash()));
    CPPUNIT_ASSERT_MESSAGE(msg, lua("f", val(1), val(2))->hash() == lua("f", val(1), val(2))->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(lua("f", val(1), val(2))->hash() == lua("g", val(1), val(2))->hash()));
    CPPUNIT_ASSERT_MESSAGE(msg, pool(val(1), val(2))->hash() == pool(val(1), val(2))->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(pool(val(1), val(2))->hash() == pool(val(2), val(1))->hash()));
    CPPUNIT_ASSERT_MESSAGE(msg, dots(val(1), val(2))->hash() == dots(val(1), val(2))->hash());
    CPPUNIT_ASSERT_MESSAGE(msg, !(dots(val(1), val(2))->hash() == dots(val(2), val(1))->hash()));
    // gterm
    CPPUNIT_ASSERT_MESSAGE(msg, hash(val(1)) == hash(val(1)));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(val(1)) != hash(val(2)));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(lin("X",2,3)) == hash(lin("X",2,3)));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(lin("X",2,3)) == hash(lin("Y",2,3))); // Note: intended
    CPPUNIT_ASSERT_MESSAGE(msg, hash(lin("X",2,3)) != hash(lin("X",1,3)));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(lin("X",2,3)) != hash(lin("X",2,2)));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(var("X")) == hash(var("X")));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(var("X")) == hash(var("Y"))); // Note: intended
    CPPUNIT_ASSERT_MESSAGE(msg, hash(fun("f", val(1), val(2))) == hash(fun("f", val(1), val(2))));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(fun("f", val(1), val(2))) != hash(fun("g", val(1), val(2))));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(fun("f", val(1), val(2))) != hash(fun("f", val(2), val(2))));
    CPPUNIT_ASSERT_MESSAGE(msg, hash(fun("f", val(1), val(2))) != hash(fun("f", val(1), val(1))));
}

void TestTerm::test_equal() {
    // term
    CPPUNIT_ASSERT(*val(1) == *val(1));
    CPPUNIT_ASSERT(!(*val(1) == *val(2)));
    CPPUNIT_ASSERT(*var("X") == *var("X"));
    CPPUNIT_ASSERT(!(*var("X") == *var("Y")));
    CPPUNIT_ASSERT(*binop(BinOp::ADD, val(1), val(2)) == *binop(BinOp::ADD, val(1), val(2)));
    CPPUNIT_ASSERT(!(*binop(BinOp::ADD, val(1), val(2)) == *binop(BinOp::ADD, val(2), val(1))));
    CPPUNIT_ASSERT(*unop(UnOp::NEG, val(1)) == *unop(UnOp::NEG, val(1)));
    CPPUNIT_ASSERT(!(*unop(UnOp::NEG, val(1)) == *unop(UnOp::ABS, val(1))));
    CPPUNIT_ASSERT(*fun("f", val(1), val(2)) == *fun("f", val(1), val(2)));
    CPPUNIT_ASSERT(!(*fun("f", val(1), val(2)) == *fun("g", val(1), val(2))));
    CPPUNIT_ASSERT(*lua("f", val(1), val(2)) == *lua("f", val(1), val(2)));
    CPPUNIT_ASSERT(!(*lua("f", val(1), val(2)) == *lua("g", val(1), val(2))));
    CPPUNIT_ASSERT(*pool(val(1), val(2)) == *pool(val(1), val(2)));
    CPPUNIT_ASSERT(!(*pool(val(1), val(2)) == *pool(val(2), val(1))));
    CPPUNIT_ASSERT(!(*dots(val(1), val(2)) == *dots(val(1), val(2)))); // Note: intended
    CPPUNIT_ASSERT(!(*dots(val(1), val(2)) == *dots(val(2), val(1))));
    // gterm
    CPPUNIT_ASSERT(*gterm(val(1)) == *gterm(val(1)));
    CPPUNIT_ASSERT(*gterm(val(1)) != *gterm(val(2)));
    CPPUNIT_ASSERT(*gterm(lin("X",2,3)) == *gterm(lin("X",2,3)));
    CPPUNIT_ASSERT(*gterm(lin("X",2,3)) == *gterm(lin("Y",2,3))); // Note: intended
    CPPUNIT_ASSERT(*gterm(lin("X",2,3)) != *gterm(lin("X",1,3)));
    CPPUNIT_ASSERT(*gterm(lin("X",2,3)) != *gterm(lin("X",2,2)));
    CPPUNIT_ASSERT(*gterm(var("X")) == *gterm(var("X")));
    CPPUNIT_ASSERT(*gterm(fun("f", var("X"), var("X"))) == *gterm(fun("f", var("X"), var("X", 1)))); // Note: all levels=0
    CPPUNIT_ASSERT(*gterm(var("X")) == *gterm(var("Y"))); // Note: intended
    CPPUNIT_ASSERT(*gterm(fun("f", val(1), val(2))) == *gterm(fun("f", val(1), val(2))));
    CPPUNIT_ASSERT(*gterm(fun("f", val(1), val(2))) != *gterm(fun("g", val(1), val(2))));
    CPPUNIT_ASSERT(*gterm(fun("f", val(1), val(2))) != *gterm(fun("f", val(2), val(2))));
    CPPUNIT_ASSERT(*gterm(fun("f", val(1), val(2))) != *gterm(fun("f", val(1), val(1))));
}

void TestTerm::test_eval() {
    CPPUNIT_ASSERT_EQUAL(Value(1), val(1)->eval());

    CPPUNIT_ASSERT_EQUAL(Value(0),   binop(BinOp::DIV, val(7), val(0))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(1),   binop(BinOp::MOD, val(7), val(3))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(2),   binop(BinOp::DIV, val(7), val(3))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(3),   binop(BinOp::AND, val(7), val(3))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(4),   binop(BinOp::XOR, val(7), val(3))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(5),   binop(BinOp::SUB, val(7), val(2))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(7),   binop(BinOp::OR,  val(7), val(3))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(10),  binop(BinOp::ADD, val(7), val(3))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(21),  binop(BinOp::MUL, val(7), val(3))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(343), binop(BinOp::POW, val(7), val(3))->eval());

    CPPUNIT_ASSERT_EQUAL(Value(-1),  unop(UnOp::NEG, val(1))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(1),   unop(UnOp::ABS, val(-1))->eval());
    CPPUNIT_ASSERT_EQUAL(Value(-13), unop(UnOp::NOT, val(12))->eval());

    CPPUNIT_ASSERT_EQUAL(Value("f", {1, 5}), fun("f", val(1), binop(BinOp::ADD, val(2), val(3)))->eval());
}

void TestTerm::test_rewriteArithmetics() {
    CPPUNIT_ASSERT_EQUAL(std::string("(#Arith0,[{(X\\Y):#Arith0}])"), rewriteArithmetics(binop(BinOp::MOD, var("X"), var("Y"))));
    CPPUNIT_ASSERT_EQUAL(std::string("(f(#Arith0,1),[{(g(X)\\Y):#Arith0}])"), rewriteArithmetics(fun("f", binop(BinOp::MOD, fun("g", var("X")), var("Y")), val(1))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#Arith0,[{@f((@g(X)\\Y)):#Arith0}])"), rewriteArithmetics(lua("f", binop(BinOp::MOD, lua("g", var("X")), var("Y")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#Arith0,[{|(X\\Y)|:#Arith0}])"), rewriteArithmetics(unop(UnOp::ABS, binop(BinOp::MOD, var("X"), var("Y")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#Arith0,[{(-(X\\Y)):#Arith0}])"), rewriteArithmetics(unop(UnOp::NEG, binop(BinOp::MOD, var("X"), var("Y")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(f(#Arith0,#Arith0),[{(X\\Y):#Arith0}])"), rewriteArithmetics(fun("f", binop(BinOp::MOD, var("X"), var("Y")), binop(BinOp::MOD, var("X"), var("Y")))));
}

void TestTerm::test_rewriteDots() {
    CPPUNIT_ASSERT_EQUAL(std::string("(#Range0,[(#Range0,X,Y)],[])"), to_string(rewriteDots(dots(var("X"), var("Y")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#Range0,[(#Range0,1,2)],[])"), to_string(rewriteDots(dots(val(1), val(2)))));
    CPPUNIT_ASSERT_EQUAL(std::string("(f(#Range0,3),[(#Range0,g(1),2)],[])"), to_string(rewriteDots(fun("f", dots(fun("g", val(1)), val(2)), val(3)))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#Script2,[(#Range1,#Script0,2)],[(#Script0,g,[X]),(#Script2,f,[#Range1,3])])"), to_string(rewriteDots(lua("f", dots(lua("g", var("X")), val(2)), val(3)))));
    CPPUNIT_ASSERT_EQUAL(std::string("((1*#Range0+4),[(#Range0,3,3)],[])"), to_string(rewriteDots(binop(BinOp::ADD, dots(binop(BinOp::ADD, val(1), val(2)), val(3)), val(4)))));
    CPPUNIT_ASSERT_EQUAL(std::string("(|#Range0|,[(#Range0,1,2)],[])"), to_string(rewriteDots(unop(UnOp::ABS, dots(unop(UnOp::ABS, val(1)), val(2))))));
}

void TestTerm::test_unpool() {
    CPPUNIT_ASSERT_EQUAL(std::string("[X,Y]"), to_string(unpool(pool(var("X"), var("Y")))));
    CPPUNIT_ASSERT_EQUAL(std::string("[f(X),f(Y)]"), to_string(unpool(fun("f", pool(var("X"), var("Y"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("[f(g(X),h(A)),f(g(Y),h(A)),f(g(X),h(B)),f(g(Y),h(B))]"), to_string(unpool(fun("f", fun("g", pool(var("X"), var("Y"))), fun("h", pool(var("A"), var("B")))))));
    CPPUNIT_ASSERT_EQUAL(std::string("[@f(@g(X),@h(A)),@f(@g(Y),@h(A)),@f(@g(X),@h(B)),@f(@g(Y),@h(B))]"), to_string(unpool(lua("f", lua("g", pool(var("X"), var("Y"))), lua("h", pool(var("A"), var("B")))))));
    CPPUNIT_ASSERT_EQUAL(std::string("[(-X),(-Y)]"), to_string(unpool(unop(UnOp::NEG, pool(var("X"), var("Y"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("[(X+A),(X+B),(Y+A),(Y+B)]"), to_string(unpool(binop(BinOp::ADD, pool(var("X"), var("Y")), pool(var("A"), var("B"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("[(1..(2..4)),(1..(3..4)),5]"), to_string(unpool(pool(dots(val(1), dots(pool(val(2), val(3)), val(4))), val(5)))));
}

void TestTerm::test_simplify() {
    CPPUNIT_ASSERT_EQUAL(std::string("(1*X+1)"), to_string(simplify(binop(BinOp::ADD, val(1), var("X")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(1*X+1)"), to_string(simplify(binop(BinOp::ADD, var("X"), val(1)))));
    CPPUNIT_ASSERT_EQUAL(std::string("(-1*X+1)"), to_string(simplify(binop(BinOp::SUB, val(1), var("X")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(2*X+0)"), to_string(simplify(binop(BinOp::MUL, val(2), var("X")))));
    CPPUNIT_ASSERT_EQUAL(std::string("0"), to_string(simplify(binop(BinOp::MUL, val(0), var("X")))));
    CPPUNIT_ASSERT_EQUAL(std::string("0"), to_string(simplify(binop(BinOp::ADD, fun("f", val(1)), var("X")))));
    CPPUNIT_ASSERT_EQUAL(std::string("X"), to_string(simplify(binop(BinOp::SUB, val(1), binop(BinOp::SUB, val(1), var("X"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("(1*X+1)"), to_string(simplify(binop(BinOp::SUB, val(1), unop(UnOp::NEG, var("X"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("3"), to_string(simplify(binop(BinOp::ADD, val(1), val(2)))));
    CPPUNIT_ASSERT_EQUAL(std::string("1"), to_string(simplify(unop(UnOp::NEG, val(-1)))));
    CPPUNIT_ASSERT_EQUAL(std::string("f((1*X+1))"), to_string(simplify(fun("f", binop(BinOp::ADD, val(1), var("X"))))));
}

void TestTerm::test_rewrite() {
    CPPUNIT_ASSERT_EQUAL(std::string("[(f((1*#Range1+1)),[(#Range0,2,3),(#Range1,1,#Range0)],[{}]),(f((1*#Range3+1)),[(#Range2,2,4),(#Range3,1,#Range2)],[{}]),(f((-1*X+1)),[],[{}])]"), to_string(rewrite(fun("f", pool(binop(BinOp::ADD, dots(val(1), dots(val(2), pool(val(3), val(4)))), val(1)), binop(BinOp::SUB, val(1), var("X")))))));
}

void TestTerm::test_undefined() {
    CPPUNIT_ASSERT_EQUAL(Value(0), binop(BinOp::POW, val("a"), val(1))->eval());
    CPPUNIT_ASSERT_EQUAL(std::string("dummy:1:1: warning: operation undefined, a zero is substituted:\n  (a**1)\n"), messages.back());
    CPPUNIT_ASSERT_EQUAL(Value(0), unop(UnOp::NEG, val("a"))->eval());
    CPPUNIT_ASSERT_EQUAL(std::string("dummy:1:1: warning: operation undefined, a zero is substituted:\n  (-a)\n"), messages.back());
}

void TestTerm::test_project() {
    CPPUNIT_ASSERT_EQUAL(std::string("(#p_p(#p),#p_p(#p),p(#P0))"), to_string(rewriteProject(fun("p", var("_")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#p_p(#b(X),#p),#p_p(#b(#X0),#p),p(#X0,#P1))"), to_string(rewriteProject(fun("p", var("X"), var("_")))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#p_p(g(#p)),#p_p(g(#p)),p(g(#P0)))"), to_string(rewriteProject(fun("p", fun("g", var("_"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#p_p(#p,f(#b(X),#p),g(#p)),#p_p(#p,f(#b(#X1),#p),g(#p)),p(#P0,f(#X1,#P2),g(#P3)))"), to_string(rewriteProject(fun("p", var("_"), fun("f", var("X"), var("_")), fun("g", var("_"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#p_p(#p,f(h(#p),#b((1*X+2)),#p),g(#p)),#p_p(#p,f(h(#p),#b(#X2),#p),g(#p)),p(#P0,f(h(#P1),#X2,#P3),g(#P4)))"), to_string(rewriteProject(fun("p", var("_"), fun("f", fun("h", var("_")), binop(BinOp::ADD, var("X"), val(2)), var("_")), fun("g", var("_"))))));
    CPPUNIT_ASSERT_EQUAL(std::string("(#p_p(#b((1*#Anon0+1))),#p_p(#b(#X1)),p(#X1))"), to_string(rewriteProject(fun("p", binop(BinOp::ADD, var("_"), val(1))))));
}

void TestTerm::test_match() {
	CPPUNIT_ASSERT(bindVars(val(1))->match({1}));
	CPPUNIT_ASSERT(!bindVars(val(1))->match({2}));
	CPPUNIT_ASSERT(bindVars(binop(BinOp::ADD, val(1), var("X")))->match({1}));
	CPPUNIT_ASSERT(!bindVars(binop(BinOp::ADD, val(1), var("X")))->match({"a"}));
	CPPUNIT_ASSERT(bindVars(unop(UnOp::NEG, var("X")))->match({1}));
	CPPUNIT_ASSERT(!bindVars(unop(UnOp::NEG, var("X")))->match({"a"}));
	CPPUNIT_ASSERT(bindVars(fun("p", var("X"), var("X")))->match(Value("p", {1, 1})));
	CPPUNIT_ASSERT(!bindVars(fun("p", var("X"), var("X")))->match(Value("p", {1, 2})));
	CPPUNIT_ASSERT(bindVars(fun("p", binop(BinOp::SUB, val(4), binop(BinOp::MUL, val(3), var("X"))), unop(UnOp::NEG, var("X"))))->match(Value("p", {-2, -2})));
	CPPUNIT_ASSERT(bindVars(fun("p", binop(BinOp::SUB, val(4), binop(BinOp::MUL, val(3), var("X"))), unop(UnOp::NEG, var("X"))))->match(Value("p", {-5, -3})));
	CPPUNIT_ASSERT(!bindVars(fun("p", binop(BinOp::SUB, val(4), binop(BinOp::MUL, val(3), var("X"))), unop(UnOp::NEG, var("X"))))->match(Value("p", {2, 2})));
	CPPUNIT_ASSERT(!bindVars(fun("p", binop(BinOp::SUB, val(4), binop(BinOp::MUL, val(3), var("X"))), unop(UnOp::NEG, var("X"))))->match(Value("p", {1, 2})));
}

TestTerm::~TestTerm() { }

// }}}

CPPUNIT_TEST_SUITE_REGISTRATION(TestTerm);

} } // namespace Test Gringo

