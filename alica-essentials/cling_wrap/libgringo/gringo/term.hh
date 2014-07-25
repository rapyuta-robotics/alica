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

#ifndef _GRINGO_TERM_HH
#define _GRINGO_TERM_HH

#include <gringo/bug.hh>
#include <gringo/value.hh>
#include <gringo/printable.hh>
#include <gringo/hashable.hh>
#include <gringo/locatable.hh>
#include <gringo/comparable.hh>
#include <gringo/clonable.hh>
#include <gringo/utility.hh>
#include <memory>

namespace Gringo {

// {{{ declaration of UnOp and BinOp

enum class BinOp : int { XOR, OR, AND, ADD, SUB, MUL, DIV, MOD, POW };
enum class UnOp : int { NEG, NOT, ABS };

int eval(UnOp op, int x);
int eval(BinOp op, int x, int y);

std::ostream &operator<<(std::ostream &out, BinOp op);
std::ostream &operator<<(std::ostream &out, UnOp op);

// }}}

 // {{{ declaration of Defines

struct Term;
using UTerm = std::unique_ptr<Term>;
class Defines {
public:
    using DefMap = std::unordered_map<FWString, std::tuple<bool, Location, UTerm>>;

    Defines();
    Defines(Defines &&x);
    //! Add a define. 
    //! Default defintions will not overwrite existing definitions and can be overwritten by other defines.
    void add(Location const &loc, FWString name, UTerm &&value, bool defaultDef);
    //! Evaluates layered definitions and checks for cycles.
    void init();
    bool empty() const;
    void apply(Value x, Value &retVal, UTerm &retTerm, bool replace);
    DefMap const &defs() const;
    ~Defines();

private:
    std::unordered_map<FWString, std::tuple<bool, Location, UTerm>> defs_;
};

// }}}

// {{{ declaration of GTerm

struct GRef;
struct GTerm;
struct GVarTerm;
struct GFunctionTerm;
struct GLinearTerm;
using UGTerm = std::unique_ptr<GTerm>;
struct GTerm : Printable, Hashable, Comparable<GTerm> {
    using EvalResult = std::pair<bool, Value>;
    virtual FWSignature sig() const = 0;
    virtual EvalResult eval() const = 0;
    virtual bool occurs(GRef &x) const = 0;
    virtual void reset() = 0;
    virtual bool match(Value const &x) = 0;
    virtual bool unify(GTerm &x) = 0;
    virtual bool unify(GFunctionTerm &x) = 0;
    virtual bool unify(GLinearTerm &x) = 0;
    virtual bool unify(GVarTerm &x) = 0;
    virtual ~GTerm() { }
};
using UGTermVec = std::vector<UGTerm>;
using SGRef     = std::shared_ptr<GRef>;

// }}}
// {{{ declaration of Term

struct VarTerm;
using UTermVec        = std::vector<UTerm>;
using UTermVecVec     = std::vector<UTermVec>;
using VarTermVec      = std::vector<std::reference_wrapper<VarTerm>>;
using VarTermBoundVec = std::vector<std::pair<VarTerm*,bool>>;
using VarTermSet      = std::unordered_set<std::reference_wrapper<VarTerm>, value_hash<std::reference_wrapper<VarTerm>>, value_equal_to<std::reference_wrapper<VarTerm>>>;

struct Term : public Printable, public Hashable, public Locatable, public Comparable<Term>, public Clonable<Term> {
    //! Return value of Term::project (replace, projected, project).
    using ProjectRet   = std::tuple<UTerm, UTerm, UTerm>;
    using SVal         = std::shared_ptr<Value>;
    using VarSet       = std::unordered_set<FWString>;
    using RenameMap    = std::unordered_map<FWString, std::pair<FWString, SVal>>;
    using ReferenceMap = std::unordered_map<Term*, SGRef, value_hash<Term*>, value_equal_to<Term*>>;
    //! Type that stores for each rewritten DotsTerm the associated variable and the lower and upper bound.
    using DotsMap = std::vector<std::tuple<UTerm, UTerm, UTerm>>;
    using ScriptMap = std::vector<std::tuple<UTerm, FWString, UTermVec>>;
    //! Type that stores for each rewritten arithmetic term (UnopTerm, BinopTerm, LuaTerm) the associated variable and the term itself.
    //! The indices of the vector correspond to the level of the term.
    using ArithmeticsMap = std::vector<std::unordered_map<UTerm, UTerm, value_hash<UTerm>, value_equal_to<UTerm>>>;
    //! The invertibility of a term. This may either be
    //! - CONSTANT for terms that do not contain variables,
    //! - INVERTIBLE for invertible terms (e.g. -X, 1+X, f(X,Y+Z))
    //! - NOT_INVERTIBLE for terms that are not invertible (e.g. arithmetic operations with two unknowns)
    enum Invertibility { CONSTANT = 0, INVERTIBLE = 1, NOT_INVERTIBLE = 2 };
    //! Whether the term contains a VarTerm.
    virtual bool hasVar() const = 0;
    //! Rename the outermost term.
    //! \pre the term must either be a function term or value term holding an identifier.
    //! \note unnecessary; can be deleted
    virtual void rename(FWString name) = 0;
    //! Removes all occurrences of PoolTerm instances. 
    //! Returns all unpooled incarnations of the term.
    //! \note The term becomes unusable after the method returns.
    //! \post The pool does not contain PoolTerm instances.
    virtual void unpool(UTermVec &x) const = 0;
    //! Somewhat complex result type of simplify.
    struct SimplifyRet;
    //! Removes all occurrences of DotsTerm instances, simplifies the term and sets the invertibility.
    //! To reduce the number of cases in later algorithms moves invertible terms to the left:
    //! - 1+X -> X+1
    //! - (1+(1-X)) -> ((-X)+1)+1
    //! Replaces undefined arithmetic operations with 0:
    //! - f(X)+0 -> 0
    //! \note The term is unusable if the method returned a non-zero replacement term.
    //! \pre Must be called after unpool.
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic) = 0;
    //! Removes anonymous variables in projectable positions (in outermost function symbols) from a term.
    //! The first element of the return value is a replacement term for the current term,
    //! which might be null if the term does not have to be replace..
    //! The second and third can be used to create a projection rule 
    //! where the second is the head and the third is the body element.
    //! \pre Must be called after simplify.
    //! \code{.cpp}
    //! num = 0; sig = base; lit = q(X,_)
    //! (lit', projected, project) = lit.project(&sig, num)
    //! assert(lit'      == #p_q(base,X,#p))
    //! assert(projected == #p_q(base,#X1,#p))
    //! assert(project   == #p_q(base,#X1,#Y2))
    //! \endcode
    virtual ProjectRet project(bool rename, unsigned &auxNum) = 0;
    //! Obtain the type of a term.
    //! \pre Must be called after simplify.
    //! Whether evaluation of a term is guaranteed to not produce numeric values.
    //! Basically, this means that the term is either a non-numeric constant or a function symbol.
    virtual bool isNotNumeric() const = 0;
    //! Obtain the invertibility of a term.
    //! \pre Must be called after simplify.
    virtual Invertibility getInvertibility() const = 0;
    //! Evaluates the term to a value.
    //! \pre Must be called after simplify.
    virtual Value eval() const = 0;
    //! Returns true if the term evaluates to zero.
    //! \pre Must be called after simplify.
	//! \pre Term is ground or 
    bool isZero() const;
    //! Collects variables in Terms.
    //! \pre Must be called after simplify and project to properly account for bound variables.
    // TODO: the way I am using these it would be nice to have a visitor for variables
    //       and implement the functions below using the visitor
    virtual void collect(VarTermBoundVec &vars, bool bound) const = 0;
    virtual void collect(VarTermSet &vars) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const = 0;
    virtual void collectIds(VarSet &vars) const = 0;
    //! Returns the nesting level of a term.
    //! That is the largest level of a nested variable or zero for ground terms.
    //! \pre Must be called after assignLevel.
    virtual unsigned getLevel() const = 0;
    //! Removes non-invertible arithmetics.
    //! \note The term is unusable if the method returned a non-zero replacement term.
    //! \pre Must be called after assignLevel.
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum) = 0;
	virtual bool match(Value const &val) const = 0;
    bool bind(VarSet &bound);
    virtual FWSignature getSig() const = 0;
    virtual UTerm renameVars(RenameMap &names) const = 0;
    SGRef _newRef(RenameMap &names, ReferenceMap &refs) const;
    UGTerm gterm() const;
    virtual bool hasPool() const = 0;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const = 0;
    virtual UTerm replace(Defines &defs, bool replace) = 0;
    virtual double estimate(double size, VarSet const &bound) const = 0;
    virtual Value isEDB() const = 0;

    virtual ~Term() { }

    //! Creates a unique variable name
    static FWString uniqueName(unsigned &auxNum, char const *prefix = "#X");
    //! Creates a unique variable using #aux(auxNum) as name.
    static UTerm uniqueVar(Location const &loc, unsigned &auxNum, unsigned level = 0, const char *prefix = "#X");
    static UTerm uniqueVal(Location const &loc, unsigned &auxNum, const char *prefix = "#X");
    //! Inserts a term into arith creating a new unique variable if necessary.
    static UTerm insert(ArithmeticsMap &arith, unsigned &auxNum, UTerm &&term);

    //! Set dst to src if src is non-zero.
    template <class T, class U>
    static void replace(std::unique_ptr<T> &dst, std::unique_ptr<U> &&src);
    template <class T, class U, class V>
    void replace(std::unique_ptr<T> &dst, std::unique_ptr<U> &&src, std::unique_ptr<V> &&alt);

    //! Unpools a, calling g on each obtained element.
    template <class A, class UnpoolA, class Callback>
    static void unpool(A const &a, UnpoolA const &fA, Callback const &g);
    //! Unpools a  and b, calculates cross product, calling g on each obtained tuple.
    template <class A, class B, class UnpoolA, class UnpoolB, class Callback>
    static void unpool(A const &a, B const &b, UnpoolA const &fA, UnpoolB const &fB, Callback const &g);
    //! Iterates of [begin, end] unpooling with f, calculates cross product, calling g on each obtained tuple.
    template <class It, class Unpool, class Callback>
    static void unpool(It const &begin, It const &end, Unpool const &f, Callback const &g);
    //! Unpools each element of vec using f, and move the union of all pools back to f.
    template <class Vec, class Unpool>
    static void unpoolJoin(Vec &vec, Unpool const &f);
};

UTermVec unpool(UTerm const &x);

struct LinearTerm;
struct Term::SimplifyRet {
    enum Type { UNTOUCHED, CONSTANT, LINEAR, REPLACE };
    SimplifyRet(SimplifyRet const &) = delete;
    SimplifyRet(SimplifyRet &&x);
    //! Reference to untouched term.
    SimplifyRet(Term &x, bool project);
    //! Indicate replacement with linear term.
    SimplifyRet(std::unique_ptr<LinearTerm> &&x);
    //! Indicate replacement with arbitrary term.
    SimplifyRet(UTerm &&x);
    //! Indicate replacement with value.
    SimplifyRet(Value const &x);
    bool notNumeric() const;
    bool constant() const;
    bool isZero() const;
    LinearTerm &lin();
    SimplifyRet &update(UTerm &x);
    ~SimplifyRet();
    Type  type;
    bool  project = false;
    union {
        Value val;
        Term *term;
    };
};

// }}}

// {{{ declaration of GRef

struct GRef {
    enum Type { EMPTY, VALUE, TERM };
    GRef(UTerm &&name);
    operator bool() const;
    void reset();
    GRef &operator=(Value const &x);
    GRef &operator=(GTerm &x);
    bool occurs(GRef &x) const;
    bool match(Value const &x);
    template <class T>
    bool unify(T &x);

    Type        type;
    UTerm       name;
    // Note: these two could be put into a union
    Value       value;
    GTerm      *term;
};
using SGRef = std::shared_ptr<GRef>;

// }}}
// {{{ declaration of GValTerm

struct GValTerm : GTerm {
    GValTerm(Value val);
    virtual bool operator==(GTerm const &other) const;
    virtual size_t hash() const;
    virtual void print(std::ostream &out) const;
    virtual FWSignature sig() const;
    virtual EvalResult eval() const;
    virtual bool occurs(GRef &x) const;
    virtual void reset();
    virtual bool match(Value const &x);
    virtual bool unify(GTerm &x);
    virtual bool unify(GFunctionTerm &x);
    virtual bool unify(GLinearTerm &x);
    virtual bool unify(GVarTerm &x);
    virtual ~GValTerm();

    Value val;
};

// }}}
// {{{ declaration of GFunctionTerm

struct GFunctionTerm : GTerm {
    GFunctionTerm(FWString name, UGTermVec &&args);
    virtual bool operator==(GTerm const &other) const;
    virtual size_t hash() const;
    virtual void print(std::ostream &out) const;
    virtual FWSignature sig() const;
    virtual EvalResult eval() const;
    virtual bool occurs(GRef &x) const;
    virtual void reset();
    virtual bool match(Value const &x);
    virtual bool unify(GTerm &x);
    virtual bool unify(GFunctionTerm &x);
    virtual bool unify(GLinearTerm &x);
    virtual bool unify(GVarTerm &x);
    virtual ~GFunctionTerm();

    FWString name;
    UGTermVec args;
};

// }}}
// {{{ declaration of GLinearTerm

struct GLinearTerm : GTerm {
    GLinearTerm(SGRef ref, int m, int n);
    virtual bool operator==(GTerm const &other) const;
    virtual size_t hash() const;
    virtual void print(std::ostream &out) const;
    virtual FWSignature sig() const;
    virtual EvalResult eval() const;
    virtual bool occurs(GRef &x) const;
    virtual void reset();
    virtual bool match(Value const &x);
    virtual bool unify(GTerm &x);
    virtual bool unify(GFunctionTerm &x);
    virtual bool unify(GLinearTerm &x);
    virtual bool unify(GVarTerm &x);
    virtual ~GLinearTerm();

    SGRef ref;
    int m;
    int n;
};

// }}}
// {{{ declaration of GVarTerm

struct GVarTerm : GTerm {
    GVarTerm(SGRef ref);
    virtual bool operator==(GTerm const &other) const;
    virtual size_t hash() const;
    virtual void print(std::ostream &out) const;
    virtual FWSignature sig() const;
    virtual EvalResult eval() const;
    virtual bool occurs(GRef &x) const;
    virtual void reset();
    virtual bool match(Value const &x);
    virtual bool unify(GTerm &x);
    virtual bool unify(GFunctionTerm &x);
    virtual bool unify(GLinearTerm &x);
    virtual bool unify(GVarTerm &x);
    virtual ~GVarTerm();

    SGRef ref;
};

// }}}

// {{{ declaration of PoolTerm

struct PoolTerm : public Term {
    PoolTerm(UTermVec &&terms);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual PoolTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~PoolTerm();

    UTermVec args;
};

// }}}
// {{{ declaration of ValTerm

struct ValTerm : public Term {
    ValTerm(Value value);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual ValTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~ValTerm();

    Value value;
};

// }}}
// {{{ declaration of VarTerm

struct VarTerm : Term {
    VarTerm(FWString name, SVal ref, unsigned level = 0, bool bindRef = false);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual VarTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~VarTerm();

    FWString name;
	SVal ref;
	bool bindRef;
    unsigned level;
};

// }}}
// {{{ declaration of UnOpTerm

struct UnOpTerm : public Term {
    UnOpTerm(UnOp op, UTerm &&arg);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual UnOpTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~UnOpTerm();

    UnOp const op;
    UTerm arg;
};

// }}}
// {{{ declaration of BinOpTerm

struct BinOpTerm : public Term {
    BinOpTerm(BinOp op, UTerm &&left, UTerm &&right);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual BinOpTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~BinOpTerm();

    BinOp op;
    UTerm left;
    UTerm right;
};

// }}}
// {{{ declaration of DotsTerm

struct DotsTerm : public Term {
    DotsTerm(UTerm &&left, UTerm &&right);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual DotsTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~DotsTerm();

    UTerm left;
    UTerm right;
};

// }}}
// {{{ declaration of LuaTerm

struct LuaTerm : public Term {
    LuaTerm(FWString name, UTermVec &&args);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual LuaTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~LuaTerm();

    FWString const name;
    UTermVec args;
};

// }}}
// {{{ declaration of FunctionTerm

struct FunctionTerm : public Term {
    FunctionTerm(FWString name, UTermVec &&args);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual FunctionTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~FunctionTerm();

    FWString name;
    UTermVec args;
    mutable ValVec cache;
};

// }}}
// {{{ declaration of LinearTerm

// TODO: if it holds a var it can as well use its location
struct LinearTerm : Term {
    using UVarTerm = std::unique_ptr<VarTerm>;
    LinearTerm(VarTerm const &var, int m, int n);
    LinearTerm(UVarTerm &&var, int m, int n);
    virtual void rename(FWString name);
    virtual SimplifyRet simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool arithmetic);
    virtual ProjectRet project(bool rename, unsigned &auxNum);
    virtual bool hasVar() const;
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void collect(VarSet &vars, unsigned minLevel = 0, unsigned maxLevel = std::numeric_limits<unsigned>::max()) const;
    virtual Value eval() const;
	virtual bool match(Value const &val) const;
    static std::unique_ptr<LinearTerm> create(Location const &loc, DotsMap &dots, unsigned &auxNum, UTerm &&left, UTerm &&right);
    static std::unique_ptr<LinearTerm> create(Location const &loc, ScriptMap &scripts, unsigned &auxNum, FWString name, UTermVec &&args);
    virtual FWSignature getSig() const;
    virtual UTerm renameVars(RenameMap &names) const;
    virtual UGTerm gterm(RenameMap &names, ReferenceMap &refs) const;
    virtual unsigned getLevel() const;
    virtual bool isNotNumeric() const;
    virtual Invertibility getInvertibility() const;
    virtual void print(std::ostream &out) const;
    virtual void unpool(UTermVec &x) const;
    virtual UTerm rewriteArithmetics(ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool operator==(Term const &other) const;
    virtual size_t hash() const;
    virtual LinearTerm *clone() const;
    virtual bool hasPool() const;
    virtual void collectIds(VarSet &vars) const;
    virtual UTerm replace(Defines &defs, bool replace = true);
    virtual double estimate(double size, VarSet const &bound) const;
    virtual Value isEDB() const;
    virtual ~LinearTerm();

    UVarTerm var;
    int m;
    int n;
};

// }}}

// {{{ definition of Term and auxiliary functions

// TODO: ugly

template <class T, class U>
void Term::replace(std::unique_ptr<T> &dst, std::unique_ptr<U> &&src) {
    if (src) { dst = std::move(src); }
}

template <class A, class UnpoolA, class Callback>
void Term::unpool(A const &a, UnpoolA const &fA, Callback const &g) {
    for (auto &termA : fA(a)) { g(std::move(termA)); }
}

template <class A, class B, class UnpoolA, class UnpoolB, class Callback>
void Term::unpool(A const &a, B const &b, UnpoolA const &fA, UnpoolB const &fB, Callback const &g) {
    auto poolB(fB(b));
    for (auto &termA : fA(a)) {
        for (auto &termB : poolB) { g(get_clone(termA), get_clone(termB)); }
    }
}

template <class It, class TermUnpool, class Callback>
void Term::unpool(It const &begin, It const &end, TermUnpool const &f, Callback const &g) {
    using R = decltype(f(*begin));
    std::vector<R> pools;
    for (auto it = begin; it != end; ++it) { pools.emplace_back(f(*it)); }
    cross_product(pools);
    for (auto &pooled : pools) { g(std::move(pooled)); }
}

template <class Vec, class TermUnpool>
void Term::unpoolJoin(Vec &vec, TermUnpool const &f) {
    Vec join;
    for (auto &x : vec) {
        auto pool(f(x));
        std::move(pool.begin(), pool.end(), std::back_inserter(join));
    }
    vec = std::move(join);
}

int toNum(UTerm const &x);

// }}}

} // namespace Gringo

GRINGO_HASH(Gringo::Term)
GRINGO_HASH(Gringo::VarTerm)
GRINGO_HASH(Gringo::GTerm)

#endif // _GRINGO_TERM_HH
