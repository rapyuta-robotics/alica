// 
// Copyright (c) 2006-2011, Benjamin Kaufmann
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

#include <clasp/model_enumerators.h>
#include <clasp/solver.h>
#include <clasp/minimize_constraint.h>
#include <clasp/util/multi_queue.h>
#include <algorithm>
#include <cstdlib>
namespace Clasp {

/////////////////////////////////////////////////////////////////////////////////////////
// strategy_record
/////////////////////////////////////////////////////////////////////////////////////////
class ModelEnumerator::SolutionQueue : public mt::MultiQueue<SharedLiterals*, void (*)(SharedLiterals*)> {
public:
	typedef SharedLiterals SL;
	typedef mt::MultiQueue<SL*, void (*)(SL*)> base_type;
	SolutionQueue(uint32 m) : base_type(m, releaseLits) {}
	void addSolution(SL* solution, const ThreadId& id) {
		publish(solution, id);
	}
	static void releaseLits(SL* x) { x->release(); }
};

class ModelEnumerator::RecordFinder : public EnumerationConstraint {
public:
	typedef ModelEnumerator::QPtr   QPtr;
	typedef SolutionQueue::ThreadId ThreadId;
	typedef SolutionQueue::SL       SL;
	typedef ClauseCreator::Result   Result;
	typedef Solver::ConstraintDB    ConstraintDB;
	RecordFinder(Solver& s, MinimizeConstraint* min, SolutionQueue* q) : EnumerationConstraint(s, min), queue(q) {
		if (q) { id = q->addThread(); }
	}
	Constraint* cloneAttach(Solver& s) { return new RecordFinder(s, cloneMinimizer(s), queue); }
	void doCommitModel(Enumerator& ctx, Solver& s);
	void destroy(Solver* s, bool detach);
	bool doUpdate(Solver& s);
	bool simplify(Solver& s, bool) { EnumerationConstraint::simplify(s, false); simplifyDB(s, db, false); return false; }
	QPtr          queue;
	ThreadId      id;
	ConstraintDB  db;
	LitVec        solution;
};

void ModelEnumerator::RecordFinder::destroy(Solver* s, bool detach) {
	while (!db.empty()) {
		db.back()->destroy(s, detach);
		db.pop_back();
	}
	queue = 0;
	EnumerationConstraint::destroy(s, detach);
}

bool ModelEnumerator::RecordFinder::doUpdate(Solver& s) {
	if (queue) {
		uint32 f = ClauseCreator::clause_no_add | ClauseCreator::clause_no_release | ClauseCreator::clause_explicit;
		for (SL* clause; queue->tryConsume(id, clause); ) {
			ClauseCreator::Result res = ClauseCreator::integrate(s, clause, f);	
			if (res.local) { db.push_back(res.local); }
			if (!res.ok()) { return false; }
		}
	}
	else if (!solution.empty()) {
		ClauseInfo e(Constraint_t::learnt_other);
		ClauseCreator::Result ret = ClauseCreator::create(s, solution, ClauseCreator::clause_no_add, e);
		solution.clear();
		if (ret.local) { db.push_back(ret.local); }
		return ret.ok();
	}
	return true;
}

void ModelEnumerator::RecordFinder::doCommitModel(Enumerator& x, Solver& s) {
	ModelEnumerator& ctx = static_cast<ModelEnumerator&>(x);
	if (ctx.trivial()) { return; }
	assert(solution.empty() && "Update not called!");
	solution.clear();
	if (!ctx.projectionEnabled()) {
		for (uint32 x = s.decisionLevel(); x != 0; --x) {
			Literal d = s.decision(x);
			if      (!s.auxVar(d.var()))  { solution.push_back(~d); }
			else if (d != s.tagLiteral()) {
				// Todo: set of vars could be reduced to those having the aux var in their reason set.
				const LitVec& tr = s.trail();
				const uint32  end= x != s.decisionLevel() ? s.levelStart(x+1) : (uint32)tr.size();
				for (uint32 n = s.levelStart(x)+1; n != end; ++n) {
					if (!s.auxVar(tr[n].var())) { solution.push_back(~tr[n]); }
				}
			}
		}
	}
	else {
		solution.push_back(~s.sharedContext()->stepLiteral());
		for (uint32 i = 0, end = ctx.numProjectionVars(); i != end; ++i) {
			solution.push_back(~s.trueLit(ctx.projectVar(i)));
		}
	}
	if (queue) {
		assert(!queue->hasItems(id));
		// parallel solving active - share solution nogood with other solvers
		SL* shared = SL::newShareable(solution, Constraint_t::learnt_other);
		queue->addSolution(shared, id);
		solution.clear();
	}
	else if (solution.empty()) { 
		solution.push_back(negLit(0)); 
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
// strategy_backtrack
/////////////////////////////////////////////////////////////////////////////////////////
class ModelEnumerator::BacktrackFinder : public EnumerationConstraint {
public:
	BacktrackFinder(Solver& s, MinimizeConstraint* min, uint32 projOpts) : EnumerationConstraint(s, min), front(UINT32_MAX), back(0), btLev(0), opts(projOpts) {}
	bool hasModel() const { return front != UINT32_MAX; }
	
	// EnumerationConstraint interface
	void doCommitModel(Enumerator& ctx, Solver& s);
	bool doUpdate(Solver& s);
	// Constraint interface
	Constraint* cloneAttach(Solver& s){ return new BacktrackFinder(s, cloneMinimizer(s), opts); }
	void        destroy(Solver*, bool);
	PropResult  propagate(Solver&, Literal, uint32&);
	void        reason(Solver& s, Literal p, LitVec& x){
		for (uint32 i = 1, end = s.level(p.var()); i <= end; ++i) { 
			x.push_back(s.decision(i)); 
		}
	}
	typedef Solver::ConstraintDB ConstraintDB;
	ConstraintDB nogoods;
	LitVec       solution;
	uint32       front;
	uint32       back;
	uint32       btLev;
	uint32       opts;
};

void ModelEnumerator::BacktrackFinder::destroy(Solver* s, bool detach) {
	while (!nogoods.empty()) {
		Constraint* c = nogoods.back();
		nogoods.pop_back();
		if (c) c->destroy(s, detach);
	}
	EnumerationConstraint::destroy(s, detach);
}

Constraint::PropResult ModelEnumerator::BacktrackFinder::propagate(Solver& s, Literal, uint32& pos) {
	assert(pos < nogoods.size() && nogoods[pos] != 0);
	ClauseHead* c = static_cast<ClauseHead*>(nogoods[pos]);
	if (!c->locked(s)) {
		c->destroy(&s, true);
		nogoods[pos] = (c = 0);
		while (!nogoods.empty() && !nogoods.back()) {
			nogoods.pop_back();
		}
	}
	return PropResult(true, c != 0);
}

bool ModelEnumerator::BacktrackFinder::doUpdate(Solver& s) {
	if (hasModel()) {
		bool ok = true;
		assert(btLev <= s.decisionLevel());
		s.undoUntil(btLev);
		if (front == 0) { // The decision stack is already ordered.
			ok = s.backtrack();
		}
		else if (front == 1) { // The projection nogood is unit. Force the single remaining literal from the current DL.
			s.setBacktrackLevel(s.decisionLevel());
			ok = s.force(solution[0], this);
		}
		else {
			// Shorten the projection nogood by assuming one of its literals to false.
			LearntConstraint* c;
			assert(s.value(solution[1].var()) == value_free);
			bool heu  = (opts & ModelEnumerator::project_use_heuristic) != 0;
			Literal x = heu ? s.heuristic()->selectRange(s, &solution[0], &solution[0]+back) : solution[1];
			ClauseRep rep = ClauseCreator::prepare(s, solution, ClauseCreator::clause_no_prepare, Constraint_t::learnt_conflict);
			c = Clause::newContractedClause(s, rep, back, true);
			s.assume(~x);
			// Remember that we must backtrack the current decision
			// level in order to guarantee a different projected solution.
			s.setBacktrackLevel(s.decisionLevel());
			// Attach nogood to the current decision literal. 
			// Once we backtrack to ~x, the then obsolete nogood is destroyed 
			// keeping the number of projection nogoods linear in the number of (projection) atoms.
			s.addWatch(x, this, (uint32)nogoods.size());
			nogoods.push_back(c);
			ok = true;
		}
		solution.clear();
		front = UINT32_MAX;
		return ok;
	}
	if (optimize() || s.sharedContext()->concurrency() == 1 || disjointPath()) {
		return true;
	}
	s.setStopConflict();
	return false;
}

void ModelEnumerator::BacktrackFinder::doCommitModel(Enumerator& ctx, Solver& s) {
	ModelEnumerator& en = static_cast<ModelEnumerator&>(ctx);
	if (!en.projectionEnabled()) {
		s.setBacktrackLevel(btLev = s.decisionLevel());
		front = 0;
	}
	else {
		if ((opts & ModelEnumerator::project_save_progress) != 0 ) {
			s.strategy.saveProgress = 1;
		}
		// Projection is enabled - reorder stack such that
		// all decision vars up to the backtrack level are projection vars.
		// Find starting backtrack level.
		btLev  = s.backtrackLevel();
		for (uint32 r = btLev+1, end = s.decisionLevel(); r <= end && s.varInfo(s.decision(r).var()).project(); ++r) {
			++btLev;
		}
		// Store the current projected assignment as a nogood.
		solution.clear();
		front = 0;
		back  = en.numProjectionVars();
		Literal x;
		if (btLev != s.decisionLevel()) {
			solution.resize(en.numProjectionVars());
			for (uint32 i = 0, xDL; i != en.numProjectionVars(); ++i) {
				x   = ~s.trueLit(en.projectVar(i)); // Note: complement because we store the nogood as a clause!
				xDL = s.level(x.var());
				if      (xDL > btLev) { solution[front++] = x; }
				else if (xDL != 0)    { solution[--back]  = x; }
				else                               {
					solution[--back] = solution.back();
					solution.pop_back();
				}
			}
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
// class ModelEnumerator
/////////////////////////////////////////////////////////////////////////////////////////
ModelEnumerator::ModelEnumerator(Strategy st)
	: Enumerator()
	, queue_(0)
	, project_(0)
	, options_(st) {
}

Enumerator* EnumOptions::createModelEnumerator(const EnumOptions& opts) {
	ModelEnumerator*          e = new ModelEnumerator();
	ModelEnumerator::Strategy s = ModelEnumerator::strategy_auto;
	if (opts.type > (int)ModelEnumerator::strategy_auto && opts.type <= (int)ModelEnumerator::strategy_record) {
		s = static_cast<ModelEnumerator::Strategy>(opts.type);
	}
	e->setStrategy(s, opts.project);
	return e;
}

ModelEnumerator::~ModelEnumerator() {
	delete project_;
	delete queue_;
}

void ModelEnumerator::setStrategy(Strategy st, uint32 projection) {
	delete project_;
	options_ = st;
	project_ = 0;
	if (projection) { 
		options_ |= (((projection|1u) & 7u) << 4u);
		project_  = new VarVec();
	}
	if (st == strategy_auto) {
		options_ |= detect_strategy_flag;
	}
}

EnumerationConstraint* ModelEnumerator::doInit(SharedContext& ctx, MinimizeConstraint* min, int numModels) {
	delete queue_;
	queue_ = 0;
	if (project_) { 
		initProjection(ctx); 
	}
	uint32 st = strategy();
	if (detectStrategy() || (ctx.concurrency() > 1 && !ModelEnumerator::supportsParallel())) {
		st = 0;
	}
	bool optOne  = minimizer() && minimizer()->mode() == MinimizeMode_t::optimize;
	bool trivial = optOne || std::abs(numModels) == 1;
	if (optOne && project_) {
		const SharedMinimizeData* min = minimizer();
		for (const WeightLiteral* it = min->lits; !isSentinel(it->first) && trivial; ++it) {
			trivial = ctx.varInfo(it->first.var()).project();
		}
		if (!trivial) { ctx.report(warning(Event::subsystem_prepare, "Projection: Optimization may depend on enumeration order.")); }
	}
	if (st == strategy_auto) { st  = trivial || (project_ && ctx.concurrency() > 1) ? strategy_record : strategy_backtrack; }
	if (trivial)             { st |= trivial_flag; }
	if (ctx.concurrency() > 1 && !trivial && st != strategy_backtrack) {
		queue_ = new SolutionQueue(ctx.concurrency()); 
		queue_->reserve(ctx.concurrency() + 1);
	}
	options_ &= ~uint32(strategy_opts_mask);
	options_ |= st;
	Solver& s = *ctx.master();
	EnumerationConstraint* c = st == strategy_backtrack 
	  ? static_cast<ConPtr>(new BacktrackFinder(s, min, projectOpts()))
	  : static_cast<ConPtr>(new RecordFinder(s, min, queue_));
	if (projectionEnabled()) { setIgnoreSymmetric(true); }
	return c;
}

void ModelEnumerator::initProjection(SharedContext& ctx) {
	const SymbolTable& index = ctx.symbolTable();
	if (index.type() == SymbolTable::map_indirect) {
		for (SymbolTable::const_iterator it = index.curBegin(); it != index.end(); ++it) {
			if (!it->second.name.empty() && it->second.name[0] != '_') {
				addProjectVar(ctx, it->second.lit.var());
			}
		}
		std::sort(project_->begin(), project_->end());
		project_->erase(std::unique(project_->begin(), project_->end()), project_->end());
	}
	else {
		for (Var v = 1; v < index.size(); ++v) { addProjectVar(ctx, v); }
	}
	if (project_->empty()) { 
		// We project to the empty set. Add true-var so that 
		// we can distinguish this case from unprojected search
		project_->push_back(0);
	}
}

void ModelEnumerator::addProjectVar(SharedContext& ctx, Var v) {
	if (ctx.master()->value(v) == value_free) {
		project_->push_back(v);
		ctx.setFrozen(v, true);
		ctx.setProject(v, true);
	}
}

}
