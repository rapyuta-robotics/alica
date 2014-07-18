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
#include <clasp/enumerator.h>
#include <clasp/solver.h>
namespace Clasp { 

/////////////////////////////////////////////////////////////////////////////////////////
// EnumerationConstraint
/////////////////////////////////////////////////////////////////////////////////////////
EnumerationConstraint::EnumerationConstraint(Solver&, MinimizeConstraint* min) : mini_(min), flags_(0), root_(0) {
	setDisjoint(false);
}
MinimizeConstraint* EnumerationConstraint::cloneMinimizer(Solver& s) const {
	return mini_ ? static_cast<MinimizeConstraint*>(mini_->cloneAttach(s)) : 0;
}
EnumerationConstraint::~EnumerationConstraint()             { }
void EnumerationConstraint::destroy(Solver* s, bool x)      { if (mini_) { mini_->destroy(s, x); mini_ = 0; } Constraint::destroy(s, x); }
bool EnumerationConstraint::simplify(Solver& s, bool reinit){ if (mini_) { mini_->simplify(s, reinit); } return false; }
bool EnumerationConstraint::valid(Solver& s)                { return !optimize() || mini_->valid(s); }
bool EnumerationConstraint::integrateBound(Solver& s) const { return !mini_ || mini_->integrate(s); }
bool EnumerationConstraint::optimize() const                { return mini_ && mini_->shared()->optimize(); }
void EnumerationConstraint::setDisjoint(bool x) {
	if (x) { flags_ |=  uint32(flag_path_disjoint); }
	else   { flags_ &= ~uint32(flag_path_disjoint); }
}
void EnumerationConstraint::end(Solver& s) {
	if (mini_) { mini_->relax(s, disjointPath()); }
	flags_ = 0;
	next_.clear();
	if (s.rootLevel() > root_) { s.popRootLevel(s.rootLevel() - root_); }
}
bool EnumerationConstraint::start(Solver& s, const LitVec& path, bool disjoint) {
	flags_ = 0;
	root_  = s.rootLevel();
	setDisjoint(disjoint);
	if (s.pushRoot(path)) {
		integrateBound(s);
		return true;
	}
	return false;
}
bool EnumerationConstraint::update(Solver& s) {
	ValueRep st = state();
	if (st == value_true) {
		if (s.strategy.restartOnModel) { s.undoUntil(0); }
		if (optimize())                { s.strengthenConditional(); }
	}
	else if (st == value_false && !s.pushRoot(next_)) {
		if (!s.hasConflict()) { s.setStopConflict(); }
		return false;
	}
	flags_ &= uint32(clear_state_mask);
	next_.clear();
	do {
		if (!s.hasConflict() && doUpdate(s) && integrateBound(s)) {
			if (st == value_true && optimize()) { mini_->shared()->heuristic(s, (s.strategy.optHeu & SolverStrategies::opt_model) != 0); }
			return true;
		}
	} while (st != value_free && s.hasConflict() && s.resolveConflict());
	return false;
}

bool EnumerationConstraint::commitModel(Enumerator& ctx, Solver& s) {
	if (state() == value_true)          { return !next_.empty() && (s.satPrepro()->extendModel(s.model, next_), true); }
	if (mini_ && !mini_->handleModel(s)){ return false; }
	if (!ctx.tentative())               { doCommitModel(ctx, s); }
	next_ = s.symmetric();
	flags_|= value_true;
	return true;
}
bool EnumerationConstraint::commitUnsat(Enumerator&, Solver& s) {
	next_.clear();
	flags_ |= value_false;
	return mini_ && mini_->handleUnsat(s, !disjointPath(), next_);
}
/////////////////////////////////////////////////////////////////////////////////////////
// Enumerator
/////////////////////////////////////////////////////////////////////////////////////////
Enumerator::Enumerator() : mini_(0) {}
Enumerator::~Enumerator()                            { if (mini_) mini_->release(); }
void Enumerator::setDisjoint(Solver& s, bool b)const { constraint(s)->setDisjoint(b);    }
void Enumerator::setIgnoreSymmetric(bool b)          { model_.sym = static_cast<uint32>(b == false); }
void Enumerator::end(Solver& s)                const { constraint(s)->end(s); }
int  Enumerator::init(SharedContext& ctx, SharedMinimizeData* min, int limit)  { 
	typedef MinimizeConstraint* MinPtr;
	ctx.master()->setEnumerationConstraint(0);
	if (mini_ && mini_ != min)                       { mini_->release(); }
	if (min && min->mode() == MinimizeMode_t::ignore){ min->release(); min = 0; }
	mini_        = min;
	model_.values= 0;
	model_.costs = min;
	model_.num   = 0;
	model_.opt   = 0;
	model_.sym   = 1;
	model_.type  = uint32(modelType());
	model_.sId   = 0;
	MinPtr mc    = mini_ ? mini_->attach(*ctx.master()) : 0;
	limit        = limit >= 0 ? limit : 1 - int(exhaustive());
	if (limit   != 1) { ctx.setPreserveModels(true); }
	ConPtr c     = doInit(ctx, mc, limit);
	bool optEnum = tentative();
	bool cons    = model_.consequences();
	if (limit) {
		if (optimize() && !optEnum){ ctx.report(warning(Event::subsystem_prepare, "#models not 0: optimality of last model not guaranteed.")); }
		if (cons)                  { ctx.report(warning(Event::subsystem_prepare, "#models not 0: last model may not cover consequences."));   }
	}
	if      (optEnum)            { model_.type = Model::model_sat; }
	else if (cons && optimize()) { ctx.report(warning(Event::subsystem_prepare, "Optimization: Consequences may depend on enumeration order.")); }
	ctx.master()->setEnumerationConstraint(c);
	return limit;
}
Enumerator::ConPtr Enumerator::constraint(const Solver& s) const {
	return static_cast<ConPtr>(s.enumerationConstraint());
}
bool Enumerator::start(Solver& s, const LitVec& path, bool disjointPath) const {
	return constraint(s)->start(s, path, disjointPath);
}
ValueRep Enumerator::commit(Solver& s) {
	if      (s.hasConflict() && s.decisionLevel() == s.rootLevel())         { return commitUnsat(s) ? value_free : value_false; }
	else if (s.numFreeVars() == 0 && s.queueSize() == 0 && !s.hasConflict()){ return commitModel(s) ? value_true : value_free;  }
	return value_free;
}
bool Enumerator::commitModel(Solver& s) {
	assert(s.numFreeVars() == 0 && !s.hasConflict() && s.queueSize() == 0 && constraint(s));
	if (constraint(s)->commitModel(*this, s)) {
		s.stats.addModel(s.decisionLevel());
		++model_.num;
		model_.values = &s.model;
		model_.sId    = s.id();
		return true;
	}
	return false;
}
bool Enumerator::commitSymmetric(Solver& s){ return model_.sym && !optimize() && commitModel(s); }
bool Enumerator::commitUnsat(Solver& s)    { return constraint(s)->commitUnsat(*this, s); }
bool Enumerator::commitComplete() {
	if (enumerated()) {
		if (tentative()) {
			mini_->markOptimal();
			model_.opt = 1;
			model_.num = 0;
			model_.type= uint32(modelType());
			return false;
		}
		else if (model_.consequences() || (!model_.opt && optimize())) {
			model_.opt = uint32(optimize());
			model_.num = 1;
		}
	}
	return true;
}

bool Enumerator::update(Solver& s) const {
	return constraint(s)->update(s);
}
/////////////////////////////////////////////////////////////////////////////////////////
// EnumOptions
/////////////////////////////////////////////////////////////////////////////////////////
Enumerator* EnumOptions::createEnumerator() const {
	if      (consequences())      { return createConsEnumerator(*this);  }
	else if (type <= enum_record) { return createModelEnumerator(*this); }
	else                          { return 0; }
}
Enumerator* EnumOptions::nullEnumerator() {
	struct NullEnum : Enumerator {
		ConPtr doInit(SharedContext& ctx, MinimizeConstraint* m, int) {
			struct Constraint : public EnumerationConstraint {
				Constraint(Solver& s, MinimizeConstraint* min) : EnumerationConstraint(s, min) {}
				bool        doUpdate(Solver& s)   { s.setStopConflict(); return false; }
				Constraint* cloneAttach(Solver& s){ return new Constraint(s, cloneMinimizer(s)); }
			};
			return new Constraint(*ctx.master(), m);
		}
	};
	return new NullEnum;
}
}
