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

#include "gringo/ground/literals.hh"
#include "gringo/scripts.hh"

#include "tests/tests.hh"
#include "tests/term_helper.hh"

#include <functional>

namespace Gringo { namespace Ground { namespace Test {

// {{{ declaration of TestLiteral

class TestLiteral : public CppUnit::TestFixture {
    CPPUNIT_TEST_SUITE(TestLiteral);
        CPPUNIT_TEST(test_range);
        CPPUNIT_TEST(test_relation);
        CPPUNIT_TEST(test_pred);
    CPPUNIT_TEST_SUITE_END();

public:
    virtual void setUp();
    virtual void tearDown();

    std::string evalRange(UTerm assign, UTerm l, UTerm r);
    std::string evalRelation(Relation rel, UTerm l, UTerm r);
    void test_range();
    void test_relation();
    void test_pred();
    
    virtual ~TestLiteral();

    Scripts scripts;
};

// }}}

// {{{ definition of auxiliary functions

namespace {

using namespace Gringo::IO;
using namespace Gringo::Test;

using S = std::string;
using V = Value;
template <class T>
using L = std::initializer_list<T>;
template <class A, class B>
using P = std::pair<A,B>;
template <class T>
using U = std::unique_ptr<T>;

} // namespace

// }}}
// {{{ definition of TestLiteral

void TestLiteral::setUp() {
}

void TestLiteral::tearDown() {
}

std::string TestLiteral::evalRange(UTerm assign, UTerm l, UTerm r) {
    RangeLiteral lit(get_clone(assign), get_clone(l), get_clone(r));
    Term::VarSet bound;
    UIdx idx(lit.index(scripts, BinderType::ALL, bound));
    ValVec vals;
    idx->match();
    while (idx->next()) { vals.emplace_back(assign->eval()); }
    return to_string(vals);
}
void TestLiteral::test_range() {
    CPPUNIT_ASSERT_EQUAL(S("[]"), evalRange(var("X"), val(1), val(0)));
    CPPUNIT_ASSERT_EQUAL(S("[1]"), evalRange(var("X"), val(1), val(1)));
    CPPUNIT_ASSERT_EQUAL(S("[1,2]"), evalRange(var("X"), val(1), val(2)));
    CPPUNIT_ASSERT_EQUAL(S("[]"), evalRange(var("X"), val(1), val("a")));
    CPPUNIT_ASSERT_EQUAL(S("[]"), evalRange(val(0), val(1), val(2)));
    CPPUNIT_ASSERT_EQUAL(S("[1]"), evalRange(val(1), val(1), val(2)));
    CPPUNIT_ASSERT_EQUAL(S("[2]"), evalRange(val(2), val(1), val(2)));
    CPPUNIT_ASSERT_EQUAL(S("[]"), evalRange(val(3), val(1), val(2)));
}

std::string TestLiteral::evalRelation(Relation rel, UTerm l, UTerm r) {
    Term::VarSet bound;
    RelationLiteral lit(rel, get_clone(l), get_clone(r));
    UIdx idx(lit.index(scripts, BinderType::ALL, bound));
    std::vector<S> vals;
    idx->match();
    while (idx->next()) { 
        vals.emplace_back();
        vals.back() += to_string(l->eval());
        vals.back() += to_string(rel);
        vals.back() += to_string(r->eval());
    }
    return to_string(vals);
}
void TestLiteral::test_relation() {
    CPPUNIT_ASSERT_EQUAL(S("[]"), evalRelation(Relation::LT, val(1), val(1)));
    CPPUNIT_ASSERT_EQUAL(S("[1<2]"), evalRelation(Relation::LT, val(1), val(2)));
    CPPUNIT_ASSERT_EQUAL(S("[1=1]"), evalRelation(Relation::ASSIGN, var("X"), val(1)));
    CPPUNIT_ASSERT_EQUAL(S("[f(1,g(1))=f(1,g(1))]"), evalRelation(Relation::ASSIGN, fun("f", var("X"), fun("g", var("X"))), val(V("f", {1, V("g", {V(1)})}))));
    CPPUNIT_ASSERT_EQUAL(S("[]"), evalRelation(Relation::ASSIGN, fun("f", var("X"), fun("g", var("X"))), val(V("f", {1, V("g", {V(2)})}))));
}

S evalPred(L<L<V>> vals, L<P<S,V>> bound, BinderType type, NAF naf, UTerm &&repr, bool recursive = false) {
    Scripts scripts;
    Term::VarSet boundSet;
    for (auto &x : bound) {
        U<VarTerm> v(var(x.first.c_str()));
        boundSet.emplace(x.first);
        *v->ref = x.second;
    }
    PredicateDomain dom;
    PredicateLiteral lit(dom, naf, get_clone(repr));
    if (recursive) { lit.setType(OccurrenceType::UNSTRATIFIED); }
    UIdx idx = lit.index(scripts, type, boundSet);
    std::vector<std::vector<S>> ret;
    dom.init();
    for (auto &x : vals) {
        ret.emplace_back();
        for (auto &y : x) { dom.insert(y, y < Value(0)); }
        dom.mark();
        dom.unmark();
        IndexUpdater *up{idx->getUpdater()};
        if (up) { up->update(); }
        dom.expire();
        idx->match();
        while (idx->next()) { 
            ret.back().emplace_back();
            ret.back().back() += to_string(lit.gLit.repr->first);
        }
        std::sort(ret.back().begin(), ret.back().end());
    }
    return to_string(ret);
}

void TestLiteral::test_pred() {
    // BIND + LOOKUP + POS + OLD/NEW/ALL
    CPPUNIT_ASSERT_EQUAL(S("[[f(1,1),f(1,2)],[f(1,1),f(1,2),f(1,3)]]"), evalPred({{V("f",{1,1}),V("f",{2,2}),V("f",{1,2})},{V("f",{1,3})}}, {{"X",1}}, BinderType::ALL, NAF::POS, fun("f",var("X"),var("Y")), true));
    CPPUNIT_ASSERT_EQUAL(S("[[],[f(1,1),f(1,2)]]"), evalPred({{V("f",{1,1}),V("f",{2,2}),V("f",{1,2})},{V("f",{1,3})}}, {{"X",1}}, BinderType::OLD, NAF::POS, fun("f",var("X"),var("Y")), true));
    CPPUNIT_ASSERT_EQUAL(S("[[f(1,1),f(1,2)],[f(1,3)]]"), evalPred({{V("f",{1,1}),V("f",{2,2}),V("f",{1,2})},{V("f",{1,3})}}, {{"X",1}}, BinderType::NEW, NAF::POS, fun("f",var("X"),var("Y")), true));
    // MATCH + POS + OLD/NEW/ALL + recursive
    CPPUNIT_ASSERT_EQUAL(S("[[],[1],[1]]"), evalPred({{2},{1},{3}}, {}, BinderType::ALL, NAF::POS, val(1), true));
    CPPUNIT_ASSERT_EQUAL(S("[[],[],[1]]"), evalPred({{2},{1},{3}}, {}, BinderType::OLD, NAF::POS, val(1), true));
    CPPUNIT_ASSERT_EQUAL(S("[[],[1],[]]"), evalPred({{2},{1},{3}}, {}, BinderType::NEW, NAF::POS, val(1), true));
    CPPUNIT_ASSERT_EQUAL(S("[[],[1],[1]]"), evalPred({{2},{1},{3}}, {}, BinderType::ALL, NAF::POS, val(1)));
    // MATCH NOT fact/unknown
    CPPUNIT_ASSERT_EQUAL(S("[[1],[1]]"), evalPred({{1,2},{3}}, {}, BinderType::ALL, NAF::NOT, val(1)));
    CPPUNIT_ASSERT_EQUAL(S("[[],[]]"), evalPred({{-1,2},{3}}, {}, BinderType::ALL, NAF::NOT, val(-1)));
    CPPUNIT_ASSERT_EQUAL(S("[[1],[1]]"), evalPred({{2},{3}}, {}, BinderType::ALL, NAF::NOT, val(1), true));
    CPPUNIT_ASSERT_EQUAL(S("[[#false],[#false]]"), evalPred({{2},{3}}, {}, BinderType::ALL, NAF::NOT, val(1)));
    // MATCH NOTNOT fact/unknown
    CPPUNIT_ASSERT_EQUAL(S("[[1],[1]]"), evalPred({{1,2},{3}}, {}, BinderType::ALL, NAF::NOTNOT, val(1)));
    CPPUNIT_ASSERT_EQUAL(S("[[-1],[-1]]"), evalPred({{-1,2},{3}}, {}, BinderType::ALL, NAF::NOTNOT, val(-1)));
    CPPUNIT_ASSERT_EQUAL(S("[[1],[1]]"), evalPred({{2},{3}}, {}, BinderType::ALL, NAF::NOTNOT, val(1), true));
    CPPUNIT_ASSERT_EQUAL(S("[[],[]]"), evalPred({{2},{3}}, {}, BinderType::ALL, NAF::NOTNOT, val(1)));
    // FULL + OLD/NEW/ALL
    CPPUNIT_ASSERT_EQUAL(S("[[1,2],[1,2,3]]"), evalPred({{1,2},{3}}, {}, BinderType::ALL, NAF::POS, var("X"), true));
    CPPUNIT_ASSERT_EQUAL(S("[[1,2],[3]]")    , evalPred({{1,2},{3}}, {}, BinderType::NEW, NAF::POS, var("X"), true));
    CPPUNIT_ASSERT_EQUAL(S("[[],[1,2]]")     , evalPred({{1,2},{3}}, {}, BinderType::OLD, NAF::POS, var("X"), true));
    // BIND + LOOKUP + POS + OLD/NEW/ALL
    CPPUNIT_ASSERT_EQUAL(S("[[f(1,1),f(1,2)],[f(1,1),f(1,2),f(1,3)]]"), evalPred({{V("f",{1,1}),V("f",{2,2}),V("f",{1,2})},{V("f",{1,3})}}, {{"X",1}}, BinderType::ALL, NAF::POS, fun("f",var("X"),var("Y")), true));
    CPPUNIT_ASSERT_EQUAL(S("[[],[f(1,1),f(1,2)]]"), evalPred({{V("f",{1,1}),V("f",{2,2}),V("f",{1,2})},{V("f",{1,3})}}, {{"X",1}}, BinderType::OLD, NAF::POS, fun("f",var("X"),var("Y")), true));
    CPPUNIT_ASSERT_EQUAL(S("[[f(1,1),f(1,2)],[f(1,3)]]"), evalPred({{V("f",{1,1}),V("f",{2,2}),V("f",{1,2})},{V("f",{1,3})}}, {{"X",1}}, BinderType::NEW, NAF::POS, fun("f",var("X"),var("Y")), true));
}

TestLiteral::~TestLiteral() { }

// }}}

CPPUNIT_TEST_SUITE_REGISTRATION(TestLiteral);

} } } // namespace Test Ground Gringo

