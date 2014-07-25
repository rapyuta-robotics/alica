// 
// Copyright (c) 2006-2012, Benjamin Kaufmann
// 
// This file is part of Clasp. See http://www.cs.uni-potsdam.de/clasp/ 
// 
// Clasp is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
// 
// Clasp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Clasp; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#include <clasp/clause.h>
#include <clasp/solver.h>
#include <clasp/util/misc_types.h>
#include <algorithm>

namespace Clasp { namespace Detail {
struct GreaterLevel {
	GreaterLevel(const Solver& s) : solver_(s) {}
	bool operator()(const Literal& p1, const Literal& p2) const {
		assert(solver_.value(p1.var()) != value_free && solver_.value(p2.var()) != value_free);
		return solver_.level(p1.var()) > solver_.level(p2.var());
	}
private:
	GreaterLevel& operator=(const GreaterLevel&);
	const Solver& solver_;
};

struct Sink { 
	explicit Sink(SharedLiterals* c) : clause(c) {}
	~Sink() { if (clause) clause->release(); }
	SharedLiterals* clause;   
};

void* alloc(uint32 size) {
	CLASP_PRAGMA_TODO("replace with CACHE_LINE_ALIGNED alloc")
	return ::operator new(size);
}
void free(void* mem) {
	::operator delete(mem);
}

} // namespace Detail

/////////////////////////////////////////////////////////////////////////////////////////
// SharedLiterals
/////////////////////////////////////////////////////////////////////////////////////////
SharedLiterals* SharedLiterals::newShareable(const Literal* lits, uint32 size, ConstraintType t, uint32 numRefs) {
	void* m = Detail::alloc(sizeof(SharedLiterals)+(size*sizeof(Literal)));
	return new (m) SharedLiterals(lits, size, t, numRefs);
}

SharedLiterals::SharedLiterals(const Literal* a_lits, uint32 size, ConstraintType t, uint32 refs) 
	: size_type_( (size << 2) + t ) {
	refCount_ = std::max(uint32(1),refs);
	std::memcpy(lits_, a_lits, size*sizeof(Literal));
}

uint32 SharedLiterals::simplify(Solver& s) {
	bool   removeFalse = unique();
	uint32   newSize   = 0;
	Literal* r         = lits_;
	Literal* e         = lits_+size();
	ValueRep v;
	for (Literal* c = r; r != e; ++r) {
		if ( (v = s.value(r->var())) == value_free ) {
			if (c != r) *c = *r;
			++c; ++newSize;
		}
		else if (v == trueValue(*r)) {
			newSize = 0; break;
		}
		else if (!removeFalse) ++c;
	}
	if (removeFalse && newSize != size()) {
		size_type_ = (newSize << 2) | (size_type_ & uint32(3));
	}
	return newSize;
}

void SharedLiterals::release() {
	if (--refCount_ == 0) {
		this->~SharedLiterals();
		Detail::free(this);
	}
}
SharedLiterals* SharedLiterals::share() {
	++refCount_;
	return this;
}

/////////////////////////////////////////////////////////////////////////////////////////
// ClauseCreator
/////////////////////////////////////////////////////////////////////////////////////////
ClauseCreator::ClauseCreator(Solver* s) 
	: solver_(s)
	, flags_(0){ 
}

ClauseCreator& ClauseCreator::start(ConstraintType t) {
	assert(solver_ && (solver_->decisionLevel() == 0 || t != Constraint_t::static_constraint));
	literals_.clear();
	extra_ = ClauseInfo(t);
	return *this;
}

uint32 ClauseCreator::watchOrder(const Solver& s, Literal p) {
	ValueRep value_p = s.value(p.var());
	// DL+1,  if isFree(p)
	// DL(p), if isFalse(p)
	// ~DL(p),if isTrue(p)
	uint32   abstr_p = value_p == value_free ? s.decisionLevel()+1 : s.level(p.var()) ^ -(value_p==trueValue(p));
	assert(abstr_p > 0 || (s.isFalse(p) && s.level(p.var()) == 0));
	return abstr_p;
}

ClauseRep ClauseCreator::prepare(Solver& s, const Literal* in, uint32 inSize, const ClauseInfo& e, uint32 flags, Literal* out, uint32 outMax) {
	assert(out && outMax > 2);
	ClauseRep ret  = ClauseRep::prepared(out, 0, e);
	uint32 abst_w1 = 0, abst_w2 = 0;
	bool simplify  = ((flags & clause_force_simplify) != 0) && inSize > 2 && outMax >= inSize;
	Literal tag    = ~s.tagLiteral();
	Var     vMax   = 0;
	for (uint32 i = 0, j = 0, MAX_OUT = outMax - 1; i != inSize; ++i) {
		Literal p     = in[i];
		uint32 abst_p = watchOrder(s, p);
		if ((abst_p + 1) > 1 && (!simplify || !s.seen(p.var()))) {
			out[j] = p;
			if (p == tag)         { ret.info.setTagged(true); }
			if (p.var() > vMax)   { vMax = p.var();}
			if (simplify)         { s.markSeen(p); }
			if (abst_p > abst_w1) { std::swap(abst_p, abst_w1); std::swap(out[0], out[j]); }
			if (abst_p > abst_w2) { std::swap(abst_p, abst_w2); std::swap(out[1], out[j]); }
			if (j != MAX_OUT)     { ++j;  }
			++ret.size;
		}
		else if (abst_p == UINT32_MAX || (simplify && abst_p && s.seen(~p))) {
			abst_w1 = UINT32_MAX;
			break;
		}
	}
	if (simplify) { 
		for (uint32 x = 0, end = ret.size; x != end; ++x) { s.clearSeen(out[x].var()); } 
	}
	if (abst_w1 == UINT32_MAX || (abst_w2 && out[0].var() == out[1].var())) {
		out[0]   = abst_w1 == UINT32_MAX || out[0] == ~out[1] ? posLit(0) : out[0]; 
		ret.size = 1;
	}
	ret.info.setAux(s.auxVar(vMax));
	return ret;
}


ClauseRep ClauseCreator::prepare(Solver& s, LitVec& lits, uint32 flags, const ClauseInfo& info) {
	if (lits.empty()) { lits.push_back(negLit(0)); }
	if ((flags & clause_no_prepare) == 0 || (flags & clause_force_simplify) != 0) {
		ClauseRep x = prepare(s, &lits[0], (uint32)lits.size(), info, flags, &lits[0]);
		shrinkVecTo(lits, x.size);
		return x;
	}
	return ClauseRep::prepared(&lits[0], (uint32)lits.size(), info);
}

ClauseRep ClauseCreator::prepare(bool forceSimplify) {
	return prepare(*solver_, literals_, forceSimplify ? clause_force_simplify : 0, extra_);
}

ClauseCreator::Status ClauseCreator::status(const Solver& s, const Literal* clause_begin, const Literal* clause_end) {
	if (clause_end <= clause_begin) { return status_empty; }
	Literal temp[3];
	ClauseRep x = prepare(const_cast<Solver&>(s), clause_begin, uint32(clause_end - clause_begin), ClauseInfo(), 0, temp, 3);
	return status(s, x);
}

ClauseCreator::Status ClauseCreator::status(const Solver& s, const ClauseRep& c) {
	uint32 dl = s.decisionLevel();
	uint32 fw = c.size     ? watchOrder(s, c.lits[0]) : 0;
	if (fw == UINT32_MAX) { return status_subsumed; }	
	uint32 sw = c.size > 1 ? watchOrder(s, c.lits[1]) : 0;
	uint32 st = status_open;
	if      (fw > varMax)   { st|= status_sat; fw = ~fw; }
	else if (fw <= dl)      { st|= (fw ? status_unsat : status_empty); }
	if (sw <= dl && fw > sw){ st|= status_unit;  }
	return static_cast<Status>(st);
}

bool ClauseCreator::ignoreClause(const Solver& s, const ClauseRep& c, Status st, uint32 flags) {
	uint32 x = (st & (status_sat|status_unsat));
	if (x == status_open)  { return false; }
	if (x == status_unsat) { return st != status_empty && (flags & clause_not_conflict) != 0; }
	return st == status_subsumed || (st == status_sat && ( (flags & clause_not_sat) != 0 || ((flags & clause_not_root_sat) != 0 && s.level(c.lits[0].var()) <= s.rootLevel())));
}

ClauseCreator::Result ClauseCreator::end(uint32 flags) {
	assert(solver_);
	flags |= flags_;
	return ClauseCreator::create_prepared(*solver_, prepare(*solver_, literals_, flags, extra_), flags);
}

ClauseHead* ClauseCreator::newProblemClause(Solver& s, const ClauseRep& clause, uint32 flags) {
	ClauseHead* ret;
	if (clause.size > 2 && s.strategy.initWatches != SolverStrategies::watch_first) {
		uint32 fw = 0, sw = 1;
		if (s.strategy.initWatches == SolverStrategies::watch_rand) {
			fw = s.rng.irand(clause.size);
			do { sw = s.rng.irand(clause.size); } while (sw == fw); 
		}
		else if (s.strategy.initWatches == SolverStrategies::watch_least) {
			uint32 cw1 = s.numWatches(~clause.lits[0]);
			uint32 cw2 = s.numWatches(~clause.lits[1]);
			if (cw1 > cw2) { std::swap(fw, sw); std::swap(cw1, cw2); }
			for (uint32 i = 2; i != clause.size && cw2; ++i) {
				uint32 p   = i;
				uint32 cwp = s.numWatches(~clause.lits[i]);
				if (cwp < cw1) { std::swap(cwp, cw1); std::swap(fw, p); }
				if (cwp < cw2) { std::swap(cwp, cw2); std::swap(sw, p); }
			}
		}
		std::swap(clause.lits[0], clause.lits[fw]);
		std::swap(clause.lits[1], clause.lits[sw]);
	}
	if (clause.size <= Clause::MAX_SHORT_LEN || !s.sharedContext()->physicalShareProblem()) {
		ret = Clause::newClause(s, clause);
	}
	else {
		ret = Clause::newShared(s, SharedLiterals::newShareable(clause.lits, clause.size, clause.info.type(), 1), clause.info, clause.lits, false);
	}
	if ( (flags & clause_no_add) == 0 ) {
		assert(!clause.info.aux());
		s.add(ret);
	}
	return ret;
}

ClauseHead* ClauseCreator::newLearntClause(Solver& s, const ClauseRep& clause, uint32 flags) {
	ClauseHead* ret;
	Detail::Sink sharedPtr(0);
	sharedPtr.clause = s.distribute(clause.lits, clause.size, clause.info);
	if (clause.size <= Clause::MAX_SHORT_LEN || sharedPtr.clause == 0) {
		if (!s.isFalse(clause.lits[1]) || !s.strategy.compress || clause.size < s.strategy.compress) {
			ret = Clause::newClause(s, clause);
		}
		else {
			ret = Clause::newContractedClause(s, clause, 2, true);
		}
	}
	else {
		ret              = Clause::newShared(s, sharedPtr.clause, clause.info, clause.lits, false);
		sharedPtr.clause = 0;
	}
	if ( (flags & clause_no_add) == 0 ) {
		s.addLearnt(ret, clause.size, clause.info.type());
	}
	return ret;
}

ClauseHead* ClauseCreator::newUnshared(Solver& s, SharedLiterals* clause, const Literal* w, const ClauseInfo& e) {
	LitVec temp; temp.reserve(clause->size());
	temp.assign(w, w+2);
	for (const Literal* x = clause->begin(), *end = clause->end(); x != end; ++x) {
		if (watchOrder(s, *x) > 0 && *x != temp[0] && *x != temp[1]) {
			temp.push_back(*x);
		}
	}
	return Clause::newClause(s, ClauseRep::prepared(&temp[0], (uint32)temp.size(), e));
}

ClauseCreator::Result ClauseCreator::create_prepared(Solver& s, const ClauseRep& clause, uint32 flags) {
	assert(s.decisionLevel() == 0 || (clause.info.learnt() && clause.prep));
	Status x = status(s, clause);
	if (ignoreClause(s, clause, x, flags)){ 
		return Result(0, x);
	}
	if (clause.size > 1) {
		Result ret(0, x);
		if (!clause.info.learnt() && s.satPrepro() && !s.sharedContext()->frozen()) {
			return Result(0, s.satPrepro()->addClause(clause.lits, clause.size) ? x : status_unsat);
		}
		if ((flags & clause_no_heuristic) == 0) { s.heuristic()->newConstraint(s, clause.lits, clause.size, clause.info.type()); }
		if (clause.size > 3 || (flags&clause_explicit) != 0 || !s.allowImplicit(clause)) {
			ret.local = clause.info.learnt() ? newLearntClause(s, clause, flags) : newProblemClause(s, clause, flags);
		}
		else {
			// add implicit short rep
			s.add(clause);
		}
		if ((x & (status_unit|status_unsat)) != 0) {
			Antecedent ante(ret.local);
			if (!ret.local){ ante = clause.size == 3 ? Antecedent(~clause.lits[1], ~clause.lits[2]) : Antecedent(~clause.lits[1]); }
			ret.status = s.force(clause.lits[0], s.level(clause.lits[1].var()), ante) ? status_unit : status_unsat;
		}
		return ret;
	}
	s.add(clause);
	return Result(0, !s.hasConflict() ? status_unit : status_unsat);
}

ClauseCreator::Result ClauseCreator::create(Solver& s, LitVec& lits, uint32 flags, const ClauseInfo& extra) {
	return create_prepared(s, prepare(s, lits, flags, extra), flags);
}

ClauseCreator::Result ClauseCreator::create(Solver& s, const ClauseRep& rep, uint32 flags) {
	return create_prepared(s, (rep.prep == 0 && (flags & clause_no_prepare) == 0 
		? prepare(s, rep.lits, rep.size, rep.info, flags, rep.lits)
		: ClauseRep::prepared(rep.lits, rep.size, rep.info)), flags);
}

ClauseCreator::Result ClauseCreator::integrate(Solver& s, SharedLiterals* clause, uint32 modeFlags, ConstraintType t) {
	assert(!s.hasConflict() && "ClauseCreator::integrate() - precondition violated!");
	Detail::Sink shared( (modeFlags & clause_no_release) == 0 ? clause : 0);
	// determine state of clause
	Literal temp[Clause::MAX_SHORT_LEN]; temp[0] = temp[1] = negLit(0);
	ClauseRep x    = prepare(s, clause->begin(), clause->size(), ClauseInfo(t), 0, temp, Clause::MAX_SHORT_LEN);
	uint32 impSize = (modeFlags & clause_explicit) != 0 || !s.allowImplicit(x) ? 1 : 3;
	Status xs      = status(s, x);
	if (ignoreClause(s, x, xs, modeFlags)) {
		return Result(0, xs);
	}
	Result result(0, xs);
	if ((modeFlags & clause_no_heuristic) == 0) { s.heuristic()->newConstraint(s, clause->begin(), clause->size(), t); }
	if (x.size > Clause::MAX_SHORT_LEN && s.sharedContext()->physicalShare(t)) {
		result.local  = Clause::newShared(s, clause, x.info, temp, shared.clause == 0);
		shared.clause = 0;
	}
	else if (x.size > impSize) {
		result.local  = x.size <= Clause::MAX_SHORT_LEN ? Clause::newClause(s, x) : newUnshared(s, clause, temp, x.info);
	}
	else {
		// unary clause or implicitly shared via binary/ternary implication graph;
		// only check for implication/conflict but do not create
		// a local representation for the clause
		s.stats.addLearnt(x.size, x.info.type());
		modeFlags    |= clause_no_add;
	}
	if ((modeFlags & clause_no_add) == 0) { s.addLearnt(result.local, x.size, x.info.type()); }
	if ((xs & (status_unit|status_unsat)) != 0) {
		Antecedent ante = result.local ? Antecedent(result.local) : Antecedent(~temp[1], ~temp[2]);
		uint32 impLevel = s.level(temp[1].var());
		result.status   = s.force(temp[0], impLevel, ante) ? status_unit : status_unsat;
		if (result.local && (modeFlags & clause_int_lbd) != 0 && result.status != status_unsat) {
			result.local->lbd(s.updateLearnt(negLit(0), clause->begin(), clause->end(), result.local->lbd(), true));
		}
	}
	return result;
}
ClauseCreator::Result ClauseCreator::integrate(Solver& s, SharedLiterals* clause, uint32 modeFlags) { 
	return integrate(s, clause, modeFlags, clause->type());
}
/////////////////////////////////////////////////////////////////////////////////////////
// Clause
/////////////////////////////////////////////////////////////////////////////////////////
void* Clause::alloc(Solver& s, uint32 lits, bool learnt) {
	if (lits <= Clause::MAX_SHORT_LEN) {
		if (learnt) { s.addLearntBytes(32); }
		return s.allocSmall();
	}
	uint32 extra = std::max((uint32)ClauseHead::HEAD_LITS, lits) - ClauseHead::HEAD_LITS; 
	uint32 bytes = sizeof(Clause) + (extra)*sizeof(Literal);
	if (learnt) { s.addLearntBytes(bytes); }
	return Detail::alloc(bytes);
}

ClauseHead* Clause::newClause(void* mem, Solver& s, const ClauseRep& rep) {
	assert(rep.size >= 2 && mem);
	return new (mem) Clause(s, rep);
}

ClauseHead* Clause::newShared(Solver& s, SharedLiterals* shared_lits, const ClauseInfo& e, const Literal* lits, bool addRef) {
	return mt::SharedLitsClause::newClause(s, shared_lits, e, lits, addRef);
}

ClauseHead* Clause::newContractedClause(Solver& s, const ClauseRep& rep, uint32 tailStart, bool extend) {
	assert(rep.size >= 2);
	if (extend) { std::stable_sort(rep.lits+tailStart, rep.lits+rep.size, Detail::GreaterLevel(s)); }
	return new (alloc(s, rep.size, rep.info.learnt())) Clause(s, rep, tailStart, extend);
}

Clause::Clause(Solver& s, const ClauseRep& rep, uint32 tail, bool extend) 
	: ClauseHead(rep.info) {
	assert(tail >= rep.size || s.isFalse(rep.lits[tail]));
	data_.local.init(rep.size);
	if (!isSmall()) {
		// copy literals
		std::memcpy(head_, rep.lits, rep.size*sizeof(Literal));
		tail = std::max(tail, (uint32)ClauseHead::HEAD_LITS);
		if (tail < rep.size) {       // contracted clause
			head_[rep.size-1].watch(); // mark last literal of clause
			Literal t = head_[tail];
			if (s.level(t.var()) > 0) {
				data_.local.markContracted();
				if (extend) {
					s.addUndoWatch(s.level(t.var()), this);
				}
			}
			data_.local.setSize(tail);
		}
	}
	else {
		std::memcpy(head_, rep.lits, std::min(rep.size, (uint32)ClauseHead::HEAD_LITS)*sizeof(Literal));
		data_.lits[0] = rep.size > ClauseHead::HEAD_LITS   ? rep.lits[ClauseHead::HEAD_LITS].asUint()   : negLit(0).asUint();
		data_.lits[1] = rep.size > ClauseHead::HEAD_LITS+1 ? rep.lits[ClauseHead::HEAD_LITS+1].asUint() : negLit(0).asUint();
		assert(isSmall() && Clause::size() == rep.size);
	}
	attach(s);
}

Clause::Clause(Solver& s, const Clause& other) : ClauseHead(ClauseInfo()) {
	info_.rep      = other.info_.rep;
	uint32 oSize   = other.size();
	data_.local.init(oSize);
	if      (!isSmall())      { std::memcpy(head_, other.head_, oSize*sizeof(Literal)); }
	else if (other.isSmall()) { std::memcpy(data_.lits, other.data_.lits, (ClauseHead::MAX_SHORT_LEN+1)*sizeof(Literal)); }
	else { // this is small, other is not
		std::memcpy(head_, other.head_, ClauseHead::HEAD_LITS*sizeof(Literal));
		std::memcpy(data_.lits, other.head_+ClauseHead::HEAD_LITS, 2*sizeof(Literal));
	}
	attach(s);
}

Constraint* Clause::cloneAttach(Solver& other) {
	assert(!learnt());
	return new (alloc(other, Clause::size(), false)) Clause(other, *this);
}

void Clause::destroy(Solver* s, bool detachFirst) {
	if (s) { 
		if (detachFirst) { Clause::detach(*s); }
		if (learnt())    { s->freeLearntBytes(computeAllocSize()); }
	}
	void* mem   = static_cast<Constraint*>(this);
	bool  small = isSmall();
	this->~Clause();
	if (!small) { Detail::free(mem); }
	else if (s) { s->freeSmall(mem); }
}

void Clause::detach(Solver& s) {
	if (contracted()) {
		Literal* eoc = longEnd();
		if (s.isFalse(*eoc) && s.level(eoc->var()) != 0) {
			s.removeUndoWatch(s.level(eoc->var()), this);
		}
	}
	ClauseHead::detach(s);
}

uint32 Clause::computeAllocSize() const {
	if (isSmall()) { return 32; }
	uint32 rt = sizeof(Clause) - (ClauseHead::HEAD_LITS*sizeof(Literal));
	uint32 sz = data_.local.size();
	uint32 nw = contracted() + strengthened();
	if (nw != 0u) {
		const Literal* eoc = head_ + sz;
		do { nw -= eoc++->watched(); } while (nw); 
		sz = static_cast<uint32>(eoc - head_);
	}
	return rt + (sz*sizeof(Literal));
}

bool Clause::updateWatch(Solver& s, uint32 pos) {
	uint32 idx = data_.local.idx;
	if (!isSmall()) {
		for (Literal* tailS = head_ + ClauseHead::HEAD_LITS, *end = longEnd();;) {
			for (Literal* it  = tailS + idx; it < end; ++it) {
				if (!s.isFalse(*it)) {
					std::swap(*it, head_[pos]);
					data_.local.idx = static_cast<uint32>(++it - tailS);
					return true;
				}
			}
			if (idx == 0) { break; }
			end = tailS + idx;
			idx = 0;
		}
	}
	else if (!s.isFalse(Literal::fromRep(data_.lits[idx=0])) || !s.isFalse(Literal::fromRep(data_.lits[idx=1]))) {
		std::swap(head_[pos].asUint(), data_.lits[idx]);
		return true;
	}
	return false;
}

void Clause::reason(Solver& s, Literal p, LitVec& out) {
	LitVec::size_type i = out.size();
	out.push_back(~head_[p == head_[0]]);
	if (!isSentinel(head_[2])) {
		out.push_back(~head_[2]);
		LitRange t = tail();
		for (const Literal* r = t.first; r != t.second; ++r) {
			out.push_back(~*r);
		}
		if (contracted()) {
			const Literal* r = t.second;
			do { out.push_back(~*r); } while (!r++->watched());
		}
	}
	if (learnt()) { 
		ClauseHead::bumpActivity();
		setLbd(s.updateLearnt(p, &out[0]+i, &out[0]+out.size(), lbd(), !hasLbd())); 
	}
}

bool Clause::minimize(Solver& s, Literal p, CCMinRecursive* rec) {
	ClauseHead::bumpActivity();
	uint32 other = p == head_[0];
	if (!s.ccMinimize(~head_[other], rec) || !s.ccMinimize(~head_[2], rec)) { return false; }
	LitRange t = tail();
	for (const Literal* r = t.first; r != t.second; ++r) {
		if (!s.ccMinimize(~*r, rec)) { return false; }
	}
	if (contracted()) {
		do {
			if (!s.ccMinimize(~*t.second, rec)) { return false; }
		} while (!t.second++->watched());
	}
	return true;
}

bool Clause::isReverseReason(const Solver& s, Literal p, uint32 maxL, uint32 maxN) {
	uint32 other   = p == head_[0];
	if (!isRevLit(s, head_[other], maxL) || !isRevLit(s, head_[2], maxL)) { return false; }
	uint32 notSeen = !s.seen(head_[other].var()) + !s.seen(head_[2].var());
	LitRange t     = tail();
	for (const Literal* r = t.first; r != t.second && notSeen <= maxN; ++r) {
		if (!isRevLit(s, *r, maxL)) { return false; }
		notSeen += !s.seen(r->var());
	}
	if (contracted()) {
		const Literal* r = t.second;
		do { notSeen += !s.seen(r->var()); } while (notSeen <= maxN && !r++->watched());
	}
	return notSeen <= maxN;
}

bool Clause::simplify(Solver& s, bool reinit) {
	assert(s.decisionLevel() == 0 && s.queueSize() == 0);
	if (ClauseHead::satisfied(s)) {
		detach(s);
		return true;
	}
	LitRange t = tail();
	Literal* it= t.first - !isSmall(), *j;
	// skip free literals
	while (it != t.second && s.value(it->var()) == value_free) { ++it; }
	// copy remaining free literals
	for (j = it; it != t.second; ++it) {
		if      (s.value(it->var()) == value_free) { *j++ = *it; }
		else if (s.isTrue(*it)) { Clause::detach(s); return true;}
	}
	// replace any false lits with sentinels
	for (Literal* r = j; r != t.second; ++r) { *r = negLit(0); }
	if (!isSmall()) {
		uint32 size     = std::max((uint32)ClauseHead::HEAD_LITS, static_cast<uint32>(j-head_));
		data_.local.idx = 0;
		data_.local.setSize(size);
		if (j != t.second && learnt() && !strengthened()) {
			// mark last literal so that we can recompute alloc size later
			t.second[-1].watch();
			data_.local.markStrengthened();
		}
		if (reinit && size > 3) {
			detach(s);
			std::random_shuffle(head_, j, s.rng);
			attach(s);	
		}
	}
	else if (s.isFalse(head_[2])) {
		head_[2]   = t.first[0];
		t.first[0] = t.first[1];
		t.first[1] = negLit(0);
		--j;
	}
	return j <= t.first && ClauseHead::toImplication(s);
}

uint32 Clause::isOpen(const Solver& s, const TypeSet& x, LitVec& freeLits) {
	if (!x.inSet(ClauseHead::type()) || ClauseHead::satisfied(s)) {
		return 0;
	}
	assert(s.queueSize() == 0 && "Watches might be false!");
	freeLits.push_back(head_[0]);
	freeLits.push_back(head_[1]);
	if (!s.isFalse(head_[2])) freeLits.push_back(head_[2]);
	ValueRep v;
	LitRange t = tail();
	for (Literal* r = t.first; r != t.second; ++r) {
		if ( (v = s.value(r->var())) == value_free) {
			freeLits.push_back(*r);
		}
		else if (v == trueValue(*r)) {
			std::swap(head_[2], *r);
			return 0;
		}
	}
	return ClauseHead::type();
}

void Clause::undoLevel(Solver& s) {
	assert(!isSmall());
	uint32   t = data_.local.size();
	Literal* r = head_+t;
	while (!r->watched() && s.value(r->var()) == value_free) {
		++t;
		++r;
	}
	if (r->watched() || s.level(r->var()) == 0) {
		r->clearWatch();
		t += !isSentinel(*r);
		data_.local.clearContracted();
	}
	else {
		s.addUndoWatch(s.level(r->var()), this);
	}
	data_.local.setSize(t);
}

void Clause::toLits(LitVec& out) const {
	out.insert(out.end(), head_, (head_+ClauseHead::HEAD_LITS)-isSentinel(head_[2]));
	LitRange t = const_cast<Clause&>(*this).tail();
	if (contracted()) { while (!t.second++->watched()) {;} }
	out.insert(out.end(), t.first, t.second);
}

ClauseHead::BoolPair Clause::strengthen(Solver& s, Literal p, bool toShort) {
	LitRange t  = tail();
	Literal* eoh= head_+ClauseHead::HEAD_LITS;
	Literal* eot= t.second;
	Literal* it = std::find(head_, eoh, p);
	BoolPair ret(false, false);
	if (it != eoh) {
		if (it != head_+2) { // update watch
			*it = head_[2];
			s.removeWatch(~p, this);
			Literal* best = it, *n;
			for (n = t.first; n != eot && s.isFalse(*best); ++n) {
				if (!s.isFalse(*n) || s.level(n->var()) > s.level(best->var())) { 
					best = n; 
				}
			}
			std::swap(*it, *best);
			s.addWatch(~*it, ClauseWatch(this));
			it = head_+2;
		}	
		// replace cache literal with literal from tail
		if ((*it  = *t.first) != negLit(0)) {
			eot     = removeFromTail(s, t.first, eot);
		}
		ret.first = true;
	}
	else if ((it = std::find(t.first, eot, p)) != eot) {
		eot       = removeFromTail(s, it, eot);
		ret.first = true;
	}
	else if (contracted()) {
		for (; *it != p && !it->watched(); ++it) { ; }
		ret.first = *it == p;
		eot       = *it == p ? removeFromTail(s, it, eot) : it + 1;
	}
	if (ret.first && ~p == s.tagLiteral()) {
		clearTagged();
	}
	ret.second = toShort && eot == t.first && toImplication(s);
	return ret;
}

Literal* Clause::removeFromTail(Solver& s, Literal* it, Literal* end) {
	assert(it != end || contracted());
	if (!contracted()) {
		*it  = *--end;
		*end = negLit(0);
		if (!isSmall()) { 
			data_.local.setSize(data_.local.size()-1);
			data_.local.idx = 0;
		}
	}
	else {
		uint32 uLev  = s.level(end->var());
		Literal* j   = it;
		while ( !j->watched() ) { *j++ = *++it; }
		*j           = negLit(0);
		uint32 nLev  = s.level(end->var());
		if (uLev != nLev && s.removeUndoWatch(uLev, this) && nLev != 0) {
			s.addUndoWatch(nLev, this);
		}
		if (j != end) { (j-1)->watch(); }
		else          { data_.local.clearContracted(); }
		end = j;
	}
	if (learnt() && !isSmall() && !strengthened()) {
		end->watch();
		data_.local.markStrengthened();
	}
	return end;
}
uint32 Clause::size() const {
	LitRange t = const_cast<Clause&>(*this).tail();
	return !isSentinel(head_[2])
		? 3u + static_cast<uint32>(t.second - t.first)
		: 2u;
}
/////////////////////////////////////////////////////////////////////////////////////////
// mt::SharedLitsClause
/////////////////////////////////////////////////////////////////////////////////////////
namespace mt {
ClauseHead* SharedLitsClause::newClause(Solver& s, SharedLiterals* shared_lits, const ClauseInfo& e, const Literal* lits, bool addRef) {
	return new (s.allocSmall()) SharedLitsClause(s, shared_lits, lits, e, addRef);
}

SharedLitsClause::SharedLitsClause(Solver& s, SharedLiterals* shared_lits, const Literal* w, const ClauseInfo& e, bool addRef) 
	: ClauseHead(e) {
	static_assert(sizeof(SharedLitsClause) <= 32, "Unsupported Padding");
	data_.shared = addRef ? shared_lits->share() : shared_lits;
	std::memcpy(head_, w, std::min((uint32)ClauseHead::HEAD_LITS, shared_lits->size())*sizeof(Literal));
	attach(s);
	if (learnt()) { s.addLearntBytes(32); }
}

Constraint* SharedLitsClause::cloneAttach(Solver& other) {
	return SharedLitsClause::newClause(other, data_.shared, ClauseInfo(this->type()), head_);
}

bool SharedLitsClause::updateWatch(Solver& s, uint32 pos) {
	Literal  other = head_[1^pos];
	for (const Literal* r = data_.shared->begin(), *end = data_.shared->end(); r != end; ++r) {
		// at this point we know that head_[2] is false so we only need to check 
		// that we do not watch the other watched literal twice!
		if (!s.isFalse(*r) && *r != other) {
			head_[pos] = *r; // replace watch
			// try to replace cache literal
			switch( std::min(static_cast<uint32>(8), static_cast<uint32>(end-r)) ) {
				case 8: if (!s.isFalse(*++r) && *r != other) { head_[2] = *r; return true; }
				case 7: if (!s.isFalse(*++r) && *r != other) { head_[2] = *r; return true; }
				case 6: if (!s.isFalse(*++r) && *r != other) { head_[2] = *r; return true; }
				case 5: if (!s.isFalse(*++r) && *r != other) { head_[2] = *r; return true; }
				case 4: if (!s.isFalse(*++r) && *r != other) { head_[2] = *r; return true; }
				case 3: if (!s.isFalse(*++r) && *r != other) { head_[2] = *r; return true; }
				case 2: if (!s.isFalse(*++r) && *r != other) { head_[2] = *r; return true; }
				default: return true;
			}
		}
	}
	return false;
}

void SharedLitsClause::reason(Solver& s, Literal p, LitVec& out) {
	LitVec::size_type i = out.size();
	for (const Literal* r = data_.shared->begin(), *end = data_.shared->end(); r != end; ++r) {
		assert(s.isFalse(*r) || *r == p);
		if (*r != p) { out.push_back(~*r); }
	}
	if (learnt()) { 
		ClauseHead::bumpActivity();
		setLbd(s.updateLearnt(p, &out[0]+i, &out[0]+out.size(), lbd(), !hasLbd())); 
	}
}

bool SharedLitsClause::minimize(Solver& s, Literal p, CCMinRecursive* rec) {
	ClauseHead::bumpActivity();
	for (const Literal* r = data_.shared->begin(), *end = data_.shared->end(); r != end; ++r) {
		if (*r != p && !s.ccMinimize(~*r, rec)) { return false; }
	}
	return true;
}

bool SharedLitsClause::isReverseReason(const Solver& s, Literal p, uint32 maxL, uint32 maxN) {
	uint32 notSeen = 0;
	for (const Literal* r = data_.shared->begin(), *end = data_.shared->end(); r != end; ++r) {
		if (*r == p) continue;
		if (!isRevLit(s, *r, maxL)) return false;
		if (!s.seen(r->var()) && ++notSeen > maxN) return false;
	}
	return true;
}

bool SharedLitsClause::simplify(Solver& s, bool reinit) {
	if (ClauseHead::satisfied(s)) {
		detach(s);
		return true;
	}
	uint32 optSize = data_.shared->simplify(s);
	if (optSize == 0) {
		detach(s);
		return true;
	}
	else if (optSize <= Clause::MAX_SHORT_LEN) {
		Literal lits[Clause::MAX_SHORT_LEN];
		Literal* j = lits;
		for (const Literal* r = data_.shared->begin(), *e = data_.shared->end(); r != e; ++r) {
			if (!s.isFalse(*r)) *j++ = *r;
		}
		uint32 rep= info_.rep;
		// detach & destroy but do not release memory
		detach(s);
		SharedLitsClause::destroy(0, false);
		// construct short clause in "this"
		ClauseHead* h = Clause::newClause(this, s, ClauseRep::prepared(lits, static_cast<uint32>(j-lits)));
		// restore extra data - safe because memory layout has not changed!
		info_.rep = rep;
		return h->simplify(s, reinit);
	}
	else if (s.isFalse(head_[2])) {
		// try to replace cache lit with non-false literal
		for (const Literal* r = data_.shared->begin(), *e = data_.shared->end(); r != e; ++r) {
			if (!s.isFalse(*r) && std::find(head_, head_+2, *r) == head_+2) {
				head_[2] = *r;
				break;
			}
		}
	}
	return false;
}

void SharedLitsClause::destroy(Solver* s, bool detachFirst) {
	if (s) {
		if (detachFirst) { ClauseHead::detach(*s); }
		if (learnt())    { s->freeLearntBytes(32); }
	}
	data_.shared->release();
	void* mem = this;
	this->~SharedLitsClause();
	if (s) { s->freeSmall(mem); }
}

uint32 SharedLitsClause::isOpen(const Solver& s, const TypeSet& x, LitVec& freeLits) {
	if (!x.inSet(ClauseHead::type()) || ClauseHead::satisfied(s)) {
		return 0;
	}
	Literal* head = head_;
	ValueRep v;
	for (const Literal* r = data_.shared->begin(), *end = data_.shared->end(); r != end; ++r) {
		if ( (v = s.value(r->var())) == value_free ) {
			freeLits.push_back(*r);
		}
		else if (v == trueValue(*r)) {
			head[2] = *r; // remember as cache literal
			return 0;
		}
	}
	return ClauseHead::type();
}

void SharedLitsClause::toLits(LitVec& out) const {
	out.insert(out.end(), data_.shared->begin(), data_.shared->end());
}

ClauseHead::BoolPair SharedLitsClause::strengthen(Solver&, Literal, bool) {
	return BoolPair(false, false);
}

uint32 SharedLitsClause::size() const { return data_.shared->size(); }

} // end namespace mt

/////////////////////////////////////////////////////////////////////////////////////////
// LoopFormula
/////////////////////////////////////////////////////////////////////////////////////////
LoopFormula* LoopFormula::newLoopFormula(Solver& s, Literal* bodyLits, uint32 numBodies, uint32 bodyToWatch, uint32 numAtoms, const Activity& act) {
	uint32 bytes = sizeof(LoopFormula) + (numBodies+numAtoms+3) * sizeof(Literal);
	void* mem    = Detail::alloc( bytes );
	s.addLearntBytes(bytes);
	return new (mem) LoopFormula(s, numBodies+numAtoms, bodyLits, numBodies, bodyToWatch, act);
}
LoopFormula::LoopFormula(Solver& s, uint32 size, Literal* bodyLits, uint32 numBodies, uint32 bodyToWatch, const Activity& act)
	: act_(act) {
	end_          = numBodies + 2;
	size_         = end_+1;
	other_        = end_-1;
	lits_[0]      = Literal();  // Starting sentinel
	lits_[end_-1] = Literal();  // Position of active atom
	lits_[end_]   = Literal();  // Ending sentinel - active part
	for (uint32 i = size_; i != size+3; ++i) {
		lits_[i] = Literal();
	}
	// copy bodies: S B1...Bn, watch one
	std::memcpy(lits_+1, bodyLits, numBodies * sizeof(Literal));
	s.addWatch(~lits_[1+bodyToWatch], this, ((1+bodyToWatch)<<1)+1);
	lits_[1+bodyToWatch].watch();
}

void LoopFormula::destroy(Solver* s, bool detach) {
	if (s) {
		if (detach) {
			for (uint32 x = 1; x != end_-1; ++x) {
				if (lits_[x].watched()) {
					s->removeWatch(~lits_[x], this);
					lits_[x].clearWatch();
				}
			}
			if (lits_[end_-1].watched()) {
				lits_[end_-1].clearWatch();
				for (uint32 x = end_+1; x != size_; ++x) {
					s->removeWatch(~lits_[x], this);
					lits_[x].clearWatch();
				}
			}
		}
		if (lits_[0].watched()) {
			while (lits_[size_++].asUint() != 3u) { ; }
		}
		s->freeLearntBytes(sizeof(LoopFormula) + (size_ * sizeof(Literal)));
	}
	void* mem = static_cast<Constraint*>(this);
	this->~LoopFormula();
	Detail::free(mem);
}


void LoopFormula::addAtom(Literal atom, Solver& s) {
	act_.bumpAct();
	uint32 pos = size_++;
	assert(isSentinel(lits_[pos]));
	lits_[pos] = atom;
	lits_[pos].watch();
	s.addWatch( ~lits_[pos], this, (pos<<1)+0 );
	if (isSentinel(lits_[end_-1])) {
		lits_[end_-1] = lits_[pos];
	}
}

void LoopFormula::updateHeuristic(Solver& s) {
	Literal saved = lits_[end_-1];
	for (uint32 x = end_+1; x != size_; ++x) {
		lits_[end_-1] = lits_[x];
		s.heuristic()->newConstraint(s, lits_+1, end_-1, Constraint_t::learnt_loop);
	}
	lits_[end_-1] = saved;
}

bool LoopFormula::watchable(const Solver& s, uint32 idx) {
	assert(!lits_[idx].watched());
	if (idx == end_-1) {
		for (uint32 x = end_+1; x != size_; ++x) {
			if (s.isFalse(lits_[x])) {
				lits_[idx] = lits_[x];
				return false;
			}
		}
	}
	return true;
}

bool LoopFormula::isTrue(const Solver& s, uint32 idx) {
	if (idx != end_-1) return s.isTrue(lits_[idx]);
	for (uint32 x = end_+1; x != size_; ++x) {
		if (!s.isTrue(lits_[x])) {
			lits_[end_-1] = lits_[x];
			return false;
		}
	}
	return true;
}

Constraint::PropResult LoopFormula::propagate(Solver& s, Literal, uint32& data) {
	if (isTrue(s, other_)) {          // ignore clause, as it is 
		return PropResult(true, true);  // already satisfied
	}
	uint32  pos   = data >> 1;
	uint32  idx   = pos;
	if (pos > end_) {
		// p is one of the atoms - move to active part
		lits_[end_-1] = lits_[pos];
		idx           = end_-1;
	}
	int     dir   = ((data & 1) << 1) - 1;
	int     bounds= 0;
	for (;;) {
		for (idx+=dir;s.isFalse(lits_[idx]);idx+=dir) {;} // search non-false literal - sentinels guarantee termination
		if (isSentinel(lits_[idx])) {             // Hit a bound,
			if (++bounds == 2) {                    // both ends seen, clause is unit, false, or sat
				if (other_ == end_-1) {
					uint32 x = end_+1;
					for (; x != size_ && s.force(lits_[x], this);  ++x) { ; }
					return Constraint::PropResult(x == size_, true);  
				}
				else {
					return Constraint::PropResult(s.force(lits_[other_], this), true);  
				}
			}
			idx   = std::min(pos, end_-1);          // halfway through, restart search, but
			dir   *= -1;                            // this time walk in the opposite direction.
			data  ^= 1;                             // Save new direction of watch
		}
		else if (!lits_[idx].watched() && watchable(s, idx)) { // found a new watchable literal
			if (pos > end_) {     // stop watching atoms
				lits_[end_-1].clearWatch();
				for (uint32 x = end_+1; x != size_; ++x) {
					if (x != pos) {
						s.removeWatch(~lits_[x], this);
						lits_[x].clearWatch();
					}
				}
			}
			lits_[pos].clearWatch();
			lits_[idx].watch();
			if (idx == end_-1) {  // start watching atoms
				for (uint32 x = end_+1; x != size_; ++x) {
					s.addWatch(~lits_[x], this, static_cast<uint32>(x << 1) + 0);
					lits_[x].watch();
				}
			}
			else {
				s.addWatch(~lits_[idx], this, static_cast<uint32>(idx << 1) + (dir==1));
			}
			return Constraint::PropResult(true, false);
		} 
		else if (lits_[idx].watched()) {          // Hit the other watched literal
			other_  = idx;                          // Store it in other_
		}
	}
}

// Body: all other bodies + active atom
// Atom: all bodies
void LoopFormula::reason(Solver& s, Literal p, LitVec& lits) {
	uint32 os = lits.size();
	// all relevant bodies
	for (uint32 x = 1; x != end_-1; ++x) {
		if (lits_[x] != p) {
			lits.push_back(~lits_[x]);
		}
	}
	// if p is a body, add active atom
	if (other_ != end_-1) {
		lits.push_back(~lits_[end_-1]);
	}
	act_.setLbd(s.updateLearnt(p, &lits[0]+os, &lits[0]+lits.size(), act_.lbd()));
	act_.bumpAct();
}

bool LoopFormula::minimize(Solver& s, Literal p, CCMinRecursive* rec) {
	act_.bumpAct();
	for (uint32 x = 1; x != end_-1; ++x) {
		if (lits_[x] != p && !s.ccMinimize(~lits_[x], rec)) {
			return false;
		}
	}
	return other_ == end_-1
		|| s.ccMinimize(~lits_[end_-1], rec);
}

uint32 LoopFormula::size() const {
	return size_ - 3;
}

bool LoopFormula::locked(const Solver& s) const {
	if (other_ != end_-1) {
		return s.isTrue(lits_[other_]) && s.reason(lits_[other_]) == this;
	}
	for (uint32 x = end_+1; x != size_; ++x) {
		if (s.isTrue(lits_[x]) && s.reason(lits_[x]) == this) {
			return true;
		}
	}
	return false;
}

uint32 LoopFormula::isOpen(const Solver& s, const TypeSet& x, LitVec& freeLits) {
	if (!x.inSet(Constraint_t::learnt_loop) || (other_ != end_-1 && s.isTrue(lits_[other_]))) {
		return 0;
	}
	for (uint32 x = 1; x != end_-1; ++x) {
		if (s.isTrue(lits_[x])) {
			other_ = x;
			return 0;
		}
		else if (!s.isFalse(lits_[x])) { freeLits.push_back(lits_[x]); }
	}
	for (uint32 x = end_+1; x != size_; ++x) {
		if      (s.value(lits_[x].var()) == value_free) { freeLits.push_back(lits_[x]); }
		else if (s.isTrue(lits_[x]))                    { return 0; }
	}
	return Constraint_t::learnt_loop;
}

bool LoopFormula::simplify(Solver& s, bool) {
	assert(s.decisionLevel() == 0);
	typedef std::pair<uint32, uint32> WatchPos;
	bool      sat = false;          // is the constraint SAT?
	WatchPos  bodyWatches[2];       // old/new position of watched bodies
	uint32    bw  = 0;              // how many bodies are watched?
	uint32    j   = 1, i;
	// 1. simplify the set of bodies:
	// - search for a body that is true -> constraint is SAT
	// - remove all false bodies
	// - find the watched bodies
	for (i = 1; i != end_-1; ++i) {
		assert( !s.isFalse(lits_[i]) || !lits_[i].watched() || s.isTrue(lits_[other_]) );
		if (!s.isFalse(lits_[i])) {
			sat |= s.isTrue(lits_[i]);
			if (i != j) { lits_[j] = lits_[i]; }
			if (lits_[j].watched()) { bodyWatches[bw++] = WatchPos(i, j); }
			++j;
		}
	}
	uint32  newEnd    = j + 1;
	uint32  numBodies = j - 1;
	j += 2;
	// 2. simplify the set of atoms:
	// - remove all determined atoms
	// - remove/update watches if necessary
	for (i = end_ + 1; i != size_; ++i) {
		if (s.value(lits_[i].var()) == value_free) {
			if (i != j) { lits_[j] = lits_[i]; }
			if (lits_[j].watched()) {
				if (sat || numBodies <= 2) {
					s.removeWatch(~lits_[j], this);
					lits_[j].clearWatch();
				}
				else if (i != j) {
					GenericWatch* w = s.getWatch(~lits_[j], this);
					assert(w);
					w->data = (j << 1) + 0;
				}
			}
			++j;
		}
		else if (lits_[i].watched()) {
			s.removeWatch(~lits_[i], this);
			lits_[i].clearWatch();
		}
	}
	if (j != size_ && !lits_[0].watched()) {
		lits_[size_-1].asUint() = 3u;
		lits_[0].watch();
	}
	size_         = j;
	end_          = newEnd;
	lits_[end_]   = Literal();
	lits_[end_-1] = lits_[end_+1];
	if (sat || numBodies < 3 || size_ == end_ + 1) {
		for (i = 0; i != bw; ++i) {
			s.removeWatch(~lits_[bodyWatches[i].second], this);
			lits_[bodyWatches[i].second].clearWatch();
		}
		if (sat || size_ == end_+1) { return true; }
		// replace constraint with short clauses
		LitVec temp(lits_+1, lits_+end_);
		for (i = end_+1; i != size_; ++i) {
			temp.back() = lits_[i];
			ClauseCreator::create(s, temp, ClauseCreator::clause_no_prepare);
		}
		return true;
	}
	other_ = 1;
	for (i = 0; i != bw; ++i) {
		if (bodyWatches[i].first != bodyWatches[i].second) {
			GenericWatch* w  = s.getWatch(~lits_[bodyWatches[i].second], this);
			assert(w);
			w->data = (bodyWatches[i].second << 1) + (w->data&1);
		}
	}
	return false;
}
}
