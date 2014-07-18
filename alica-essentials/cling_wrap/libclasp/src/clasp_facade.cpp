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
#include <clasp/clasp_facade.h>
#include <clasp/lookahead.h>
#include <clasp/cb_enumerator.h>
#include <clasp/dependency_graph.h>
#include <clasp/minimize_constraint.h>
#include <clasp/util/timer.h>
#include <clasp/util/atomic.h>
#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include <limits>
#if WITH_THREADS
#include <clasp/util/mutex.h>
#endif
namespace Clasp {
/////////////////////////////////////////////////////////////////////////////////////////
// ClaspConfig
/////////////////////////////////////////////////////////////////////////////////////////
ClaspConfig::ClaspConfig() : tester_(0) { }
ClaspConfig::~ClaspConfig() {
	setSolvers(1);
	delete tester_;
}
void ClaspConfig::reset() {
	BasicSatConfig::reset();
	solve     = SolveOptions();
	enumerate = EnumOptions();
	asp       = AspOptions();
	if (tester_) { tester_->reset(); }
	setSolvers(1);
}
BasicSatConfig* ClaspConfig::addTesterConfig() {
	if (!tester_) { tester_ = new BasicSatConfig(); }
	return tester_;
}
void ClaspConfig::setSolvers(uint32 num) {
	if (!num) { num = 1; }
#if WITH_THREADS
	solve.algorithm.threads = num;
#endif
	if (num < numSolver()) { BasicSatConfig::resize(num, std::min(num, numSearch())); }
	if (testerConfig() && num < testerConfig()->numSolver()) { testerConfig()->resize(num, std::min(num, testerConfig()->numSearch())); }
}
void ClaspConfig::prepare(SharedContext& ctx) {
	uint32 numS = solve.numSolver();
	if (numS > solve.recommendedSolvers()) {
		ctx.report(warning(Event::subsystem_facade, clasp_format_error("Oversubscription: #Threads=%u exceeds logical CPUs=%u.", numS, solve.recommendedSolvers())));
	}
	if (numS > solve.supportedSolvers()) {
		ctx.report(warning(Event::subsystem_facade, "Too many solvers."));
		numS = solve.supportedSolvers();
	}
	if (numS > 1 && !solve.defaultPortfolio()) {
		for (uint32 i = 0, sWarn = 0; i != numS; ++i) {
			if (solver(i).optStrat >= SolverStrategies::opt_unsat) {
				if (++sWarn == 1) { ctx.report(warning(Event::subsystem_facade, "Splitting: Disabling unsat-core based optimization!")); }
				addSolver(i).optStrat = SolverStrategies::opt_dec;
			}
		}
	}
	if (std::abs(enumerate.numModels) != 1) { satPre.mode = SatPreParams::prepro_preserve_models; }
	setSolvers(numS);
	ctx.setConcurrency(solve.numSolver());
	for (uint32 i = 1; i != ctx.concurrency(); ++i) {
		if (!ctx.hasSolver(i)) { ctx.addSolver(); }
	}
	BasicSatConfig::prepare(ctx);
}
/////////////////////////////////////////////////////////////////////////////////////////
// ClaspFacade::SolveImpl/AsyncResult
/////////////////////////////////////////////////////////////////////////////////////////
static const ClaspFacade::Result result_unknown = {uint8(0),uint8(0)};
struct ClaspFacade::SolveImpl {
	typedef SingleOwnerPtr<SolveAlgorithm> AlgoPtr;
	enum State { state_done = 0, state_running = 1 };
	SolveImpl() : algo(0) { state = state_done; signal = 0; result = result_unknown; }
	~SolveImpl() { join(); }
	AlgoPtr            algo;
	Clasp::atomic<int> state;
	Clasp::atomic<int> signal;
	Result             result;
	bool ready()  const    { return state != state_running; }
	bool running()const    { return state == state_running; }
	void reset()           { state = signal = 0; result = result_unknown; algo->resetSolve(); }
	bool interrupt(int sig){
		if (running() && algo->interrupt()) {
			if (!signal || sig < signal) { signal = sig; }
			return true;
		}
		return false;
	}
	void solve(ClaspFacade& f, EventHandler* h, bool async) {
		assert(f.config_);
		CLASP_ASSERT_CONTRACT_MSG(ready(), "Concurrent solve operations not supported!");
		join(); // make sure that worker thread is ready
		if (!f.ok() || f.result().unsat()) { return; }
		if (h != f.ctx.eventHandler()) { f.stepHandler_ = h; }
		if (async) { algo->enableInterrupts(); }
		state      = SolveImpl::state_running;
		if (!async){ solveImpl(f, state_done);}
		else       { aSolve(f); }
	}
	void solveImpl(ClaspFacade& f, State done) {
		struct OnExit {
			OnExit(SolveImpl* s, ClaspFacade* x, int st) : self(s), facade(x), endState(st), more(true) {}
			~OnExit() { 
				self->result = facade->stopStep(self->signal, !more);
				self->state  = endState;
			}
			SolveImpl*    self;
			ClaspFacade*  facade;
			int           endState;
			bool          more;
		} scope(this, &f, done);
		f.step_.solveTime = f.step_.unsatTime = RealTime::getTime();
		scope.more = algo->solve(f.ctx, f.assume_, &f);
	}
#if WITH_THREADS
	static const int SIGASYNC = 32;
	void wait()            {
		for (unique_lock<Clasp::mutex> lock(mutex); !ready();) { condition.wait(lock); }
	}
	bool waitFor(double s) {
		for (unique_lock<Clasp::mutex> lock(mutex); !ready();) {
			try { condition.wait_for(lock, tbb::tick_count::interval_t(s)); break; }
			catch (const std::runtime_error&) {
				if (s < 0) { throw; }
				// Due to a bug in the computation of the wait time in some tbb versions 
				// for linux wait_for might fail with eid_condvar_wait_failed.
				// See: http://software.intel.com/en-us/forums/topic/280012
				// Ignore the error and retry the wait - the computed wait time will be valid, eventually.
			}
		}
		return ready();
	}
	void join() {
		if (task.joinable()) { task.join(); }
	}
	void aSolve(ClaspFacade& f) {
		// start solve in worker thread
		Clasp::thread(std::mem_fun(&SolveImpl::aSolveImpl), this, &f).swap(task);
	}
	void aSolveImpl(ClaspFacade* f) {
		try         { solveImpl(*f, state_running); }
		catch (...) { result.flags |= Result::EXT_ERROR; }
		{
			unique_lock<Clasp::mutex> lock(this->mutex);
			state = state_done;
		}
		condition.notify_one();
	}
	Clasp::thread             task;
	Clasp::mutex              mutex;
	Clasp::condition_variable condition;
#else
	void join() {}
	void aSolve(ClaspFacade& f) { solveImpl(f, state_done); }
#endif
};

#if WITH_THREADS
ClaspFacade::AsyncResult::AsyncResult(SolveImpl& x) : state_(&x) {}
int  ClaspFacade::AsyncResult::interrupted()        const { return state_->signal; }
bool ClaspFacade::AsyncResult::error()              const { return ready() && state_->result.error(); }
bool ClaspFacade::AsyncResult::ready()              const { return state_->ready(); }
bool ClaspFacade::AsyncResult::ready(Result& r)     const { if (ready()) { r = get(); return true; } return false; }
ClaspFacade::Result ClaspFacade::AsyncResult::get() const {
	wait();
	if (state_->result.error()) { throw std::runtime_error("Async operation failed!"); }
	return state_->result;
}
void ClaspFacade::AsyncResult::wait()  const { 
	state_->wait();
	state_->join();
}
bool ClaspFacade::AsyncResult::waitFor(double sec) const {
	if (state_->waitFor(sec)) {
		state_->join();
		return true;
	}
	return false;
}
bool ClaspFacade::AsyncResult::cancel()const { 
	if (state_->interrupt(SolveImpl::SIGASYNC)) {
		wait();
		return true;
	}
	return false;
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////
// ClaspFacade
/////////////////////////////////////////////////////////////////////////////////////////
ClaspFacade::ClaspFacade() : config_(0) {
}
ClaspFacade::~ClaspFacade() { }
void ClaspFacade::discardProblem() {
	config_  = 0;
	builder_ = 0;
	enum_    = 0;
	lpStats_ = 0;
	solve_   = new SolveImpl();
	accu_    = 0;
	step_.init(*this);
	if (ctx.numConstraints() || ctx.numVars()) { ctx.reset(); }
}
void ClaspFacade::init(ClaspConfig& config, bool discard) {
	if (discard) { discardProblem(); }
	ctx.setConfiguration(0, false); // force reload of configuration once done
	config_ = &config;
	if ((enum_ = config.enumerate.createEnumerator()).get() == 0) {
		enum_ = EnumOptions::nullEnumerator();
	}
	if (config.solve.numSolver() > 1 && !enum_->supportsParallel()) {
		ctx.report(warning(Event::subsystem_facade, "Selected reasoning mode implies #Threads=1."));
		config.setSolvers(1);
	}
	ctx.setConfiguration(&config, false); // prepare and apply config
	solve_->algo = config.solve.createSolveObject();
	solve_->algo->setEnumerator(*enum_);
	if (discard) { startStep(0); }
}

void ClaspFacade::initBuilder(ProgramBuilder* in, bool incremental) {
	builder_ = in;
	assume_.clear();
	builder_->startProgram(ctx);
	if (incremental) { 
		ctx.requestStepVar();
		builder_->updateProgram();
		solve_->algo->enableInterrupts();
		accu_ = new Summary();
		accu_->init(*this);
		accu_->step = UINT32_MAX;
	}
}

ProgramBuilder& ClaspFacade::start(ClaspConfig& config, ProblemType t, bool allowUpdate) {
	if      (t == Problem_t::SAT) { return startSat(config, allowUpdate); }
	else if (t == Problem_t::PB)  { return startPB(config, allowUpdate);  }
	else if (t == Problem_t::ASP) { return startAsp(config, allowUpdate); }
	else                          { throw std::domain_error("Unknown problem type!"); }
}

SatBuilder& ClaspFacade::startSat(ClaspConfig& config, bool allowUpdate) {
	init(config, true);
	initBuilder(new SatBuilder(config.enumerate.maxSat), allowUpdate);
	return static_cast<SatBuilder&>(*builder_.get());
}

PBBuilder& ClaspFacade::startPB(ClaspConfig& config, bool allowUpdate) {
	init(config, true);
	initBuilder(new PBBuilder(), allowUpdate);
	return static_cast<PBBuilder&>(*builder_.get());
}

Asp::LogicProgram& ClaspFacade::startAsp(ClaspConfig& config, bool allowUpdate) {
	init(config, true);
	Asp::LogicProgram* p = new Asp::LogicProgram();
	lpStats_             = new Asp::LpStats;
	p->accu              = lpStats_.get();
	initBuilder(p, allowUpdate);
	p->setOptions(config.asp);
	p->setNonHcfConfiguration(config.testerConfig());
	return *p;
}
ProgramBuilder& ClaspFacade::update(bool reloadConfig) {
	CLASP_ASSERT_CONTRACT(builder_.get() && solve_->ready());
	CLASP_ASSERT_CONTRACT_MSG(step_.result.signal != SIGINT, "Interrupt not handled!");
	solve_->reset();
	if (reloadConfig) { init(*config_, false); }
	if (builder_->frozen()) {
		startStep(step()+1);
		if (builder_->updateProgram()) { assume_.clear(); }
		else                           { stopStep(0, true); }
	}
	return *builder_;
}
bool ClaspFacade::terminate(int signal) {
	if (solve_.get() && solve_->interrupt(signal)) {
		return true;
	}
	// solving not active or not interruptible
	stopStep(signal, false);
	return false;
}

bool ClaspFacade::prepare(EnumMode enumMode) {
	CLASP_ASSERT_CONTRACT(config_ && solve_->ready());
	bool ok = this->ok();
	SharedMinimizeData* m = 0;
	EnumOptions& en       = config_->enumerate;
	if (builder_.get() && (ok = builder_->endProgram()) == true) {
		builder_->getAssumptions(assume_);
		if ((m  = en.opt != MinimizeMode_t::ignore ? builder_->getMinimizeConstraint(&en.bound) : 0) != 0) {
			ok = m->setMode(en.opt, en.bound);
			if (en.opt == MinimizeMode_t::enumerate && en.bound.empty()) {
				ctx.report(warning(Event::subsystem_facade, "opt-mode=enum: no bound given, optimize statement ignored"));
			}
		}
	}
	if (ok && (!ctx.frozen() || (ok = ctx.unfreeze()) == true)) {
		if (enumMode == enum_static) {
			// Step literal not needed in this step.
			ok = ctx.addUnary(ctx.stepLiteral());
		}
		if (ok) {
			int limit = enum_->init(ctx, m && m != enum_->minimizer() ? m->share() : m, en.numModels);
			solve_->algo->setEnumLimit(limit ? static_cast<uint64>(limit) : UINT64_MAX);
		}
	}
	if (!accu_.get()) { builder_ = 0; }
	return (ok && ctx.endInit()) || (stopStep(0, true), false);
}
void ClaspFacade::assume(Literal p) {
	assume_.push_back(p);
}
void ClaspFacade::assume(const LitVec& ext) {
	assume_.insert(assume_.end(), ext.begin(), ext.end());
}
bool ClaspFacade::solving() const { return solve_->running(); }

const ClaspFacade::Summary& ClaspFacade::shutdown() {
	if      (!config_) { step_.init(*this); }
	else if (solving()){ terminate(SIGINT); solve_->join(); }
	else               { stopStep(0, false); }
	return accu_.get() && accu_->step ? *accu_ : step_;
}

ClaspFacade::Result ClaspFacade::solve(EventHandler* handler) {
	solve_->solve(*this, handler, false);
	return result();
}

#if WITH_THREADS
ClaspFacade::AsyncResult ClaspFacade::solveAsync(EventHandler* handler) {
	solve_->solve(*this, handler, true);
	return AsyncResult(*solve_);
}
#endif

void ClaspFacade::startStep(uint32 n) {
	stepHandler_    = 0;
	step_.init(*this);
	step_.totalTime = -RealTime::getTime();
	step_.cpuTime   = -ProcessTime::getTime();
	step_.step      = n;
	ctx.report(StepStart(*this));
}
ClaspFacade::Result ClaspFacade::stopStep(int signal, bool complete) {
	if (step_.totalTime < 0) {
		double t = RealTime::getTime();
		step_.totalTime += t;
		step_.cpuTime   += ProcessTime::getTime();
		if (step_.solveTime) {
			step_.solveTime = t - step_.solveTime;
			step_.unsatTime = complete ? t - step_.unsatTime : 0;
		}
		Result res = {uint8(0), uint8(signal)};
		if (complete) { res.flags = uint8(step_.enumerated() ? Result::SAT : Result::UNSAT) | Result::EXT_EXHAUST; }
		else          { res.flags = uint8(step_.enumerated() ? Result::SAT : Result::UNKNOWN); }
		if (signal)   { res.flags|= uint8(Result::EXT_INTERRUPT); }
		step_.result = res;
		accuStep();
		if (stepHandler_) { stepHandler_->onEvent(StepReady(step_)); }
		ctx.report(StepReady(step_)); 
	}
	return result();
}
void ClaspFacade::accuStep() {
	if (accu_.get() && accu_->step != step_.step){
		if (step_.stats()) { ctx.accuStats(); }
		accu_->totalTime += step_.totalTime;
		accu_->cpuTime   += step_.cpuTime;
		accu_->solveTime += step_.solveTime;
		accu_->unsatTime += step_.unsatTime;
		accu_->numEnum   += step_.numEnum;
		// no aggregation
		if (step_.numEnum) { accu_->satTime = step_.satTime; }
		accu_->step       = step_.step;
		accu_->result     = step_.result;
	}
}
bool ClaspFacade::onModel(const Solver& s, const Model& m) {
	step_.unsatTime = RealTime::getTime();
	if (++step_.numEnum == 1) { step_.satTime = step_.unsatTime - step_.solveTime; }
	return !stepHandler_ || stepHandler_->onModel(s, m);
}
ExpectedQuantity::ExpectedQuantity(double d) : rep(d >= 0 ? d : -std::min(-d, 3.0)) {}
ExpectedQuantity::Error ExpectedQuantity::error() const {
	if (rep >= 0.0) { return error_none; }
	return static_cast<ExpectedQuantity::Error>(std::min(3, int(-rep)));
}
ExpectedQuantity::ExpectedQuantity(Error e) : rep(-double(int(e))) {}
ExpectedQuantity::operator double() const { return valid() ? rep : std::numeric_limits<double>::quiet_NaN(); }

ExpectedQuantity ClaspFacade::getStatImpl(const char* path, bool keys) const {
#define GET_KEYS(o, path) ( ExpectedQuantity((o).keys(path)) )
#define GET_OBJ(o, path)  ( ExpectedQuantity((o)[path]) )
#define COMMON_KEYS "ctx.\0solvers.\0solver.\0costs.\0totalTime\0cpuTime\0solveTime\0unsatTime\0satTime\0numEnum\0optimal\0step\0result\0"
	static const char* _keys[] = {"accu.\0hccs.\0hcc.\0lp.\0" COMMON_KEYS, 0, "accu.\0lp.\0" COMMON_KEYS, "accu.\0" COMMON_KEYS };
	enum ObjId { id_hccs, id_hcc, id_lp, id_ctx, id_solvers, id_solver, id_costs, id_total, id_cpu, id_solve, id_unsat, id_sat, id_num, id_opt, id_step, id_result };
	if (!path)  path = "";
	std::size_t kLen = 0;
	int         oId  = lpStats_.get() ? ((ctx.sccGraph.get() && ctx.sccGraph->numNonHcfs() != 0) ? id_hccs : id_lp) : id_ctx;
	const char* keyL = _keys[oId];
	bool        accu = matchStatPath(path, "accu");
	if (!*path) { 
		if (accu) { keyL += 6; }
		return keys ? ExpectedQuantity(keyL) : ExpectedQuantity::error_ambiguous_quantity;
	}
	accu = accu && step() != 0;
	for (const char* k = keyL + 6; (kLen = std::strlen(k)) != 0; k += kLen + 1, ++oId) {
		bool match = k[kLen-1] == '.' ? matchStatPath(path, k, kLen-1) : std::strcmp(path, k) == 0;
		if (match) {
			if (!*path && !keys){ return ExpectedQuantity::error_ambiguous_quantity; }
			switch(oId) {
				default: {
					const Summary* stats = accu && accu_.get() ? accu_.get() : &step_;
					if (oId == id_costs) {
						char* x;
						uint32 n = (uint32)std::strtoul(path, &x, 10);
						uint32 N = stats->model() && stats->optimize() ? stats->costs()->numRules() : 0u;
						if (x == path) {
							if (!*path) { return keys ? ExpectedQuantity("__len\0") : ExpectedQuantity::error_ambiguous_quantity; }
							if (!keys && std::strcmp(path, "__len") == 0) { return ExpectedQuantity(N); }
							return ExpectedQuantity::error_unknown_quantity;
						}
						if (*(path = x) == '.') { ++path; }
						if (*path)  { return ExpectedQuantity::error_unknown_quantity; }
						if (n >= N) { return ExpectedQuantity::error_not_available;    }
						if (!keys)  { return ExpectedQuantity((double)stats->costs()->optimum(n)); }
					}
					if (keys)            { return ExpectedQuantity(uint32(0)); }
					if (oId <= id_sat)   { return *((&stats->totalTime) + (oId - id_total)); }
					if (oId == id_num)   { return ExpectedQuantity(stats->numEnum); }
					if (oId == id_opt)   { return ExpectedQuantity(stats->optimal()); }
					if (oId == id_step)  { return ExpectedQuantity(stats->step);    }
					if (oId == id_result){ return ExpectedQuantity((double)stats->result); }
					return ExpectedQuantity::error_unknown_quantity; }
				case id_lp: if (!lpStats_.get()) { return ExpectedQuantity::error_not_available; }
					return keys ? GET_KEYS((*lpStats_), path) : GET_OBJ((*lpStats_), path);
				case id_ctx:     return keys ? GET_KEYS(ctx.stats(), path) : GET_OBJ(ctx.stats(), path);
				case id_solvers: return keys ? GET_KEYS(ctx.stats(*ctx.solver(0), accu), path) : getStat(ctx, path, accu, Range32(0, ctx.concurrency()));			
				case id_hccs: if (!ctx.sccGraph.get() || ctx.sccGraph->numNonHcfs() == 0) { return ExpectedQuantity::error_not_available; } else {
					ExpectedQuantity res(0.0);
					for (SharedDependencyGraph::NonHcfIter it = ctx.sccGraph->nonHcfBegin(), end = ctx.sccGraph->nonHcfEnd(); it != end; ++it) {
						const SharedContext& hCtx= it->second->ctx();
						if (keys) { return GET_KEYS(hCtx.stats(*hCtx.solver(0), accu), path); }
						ExpectedQuantity hccAccu = getStat(hCtx, path, accu, Range32(0, hCtx.concurrency()));
						if (!hccAccu.valid()) { return hccAccu; }
						res.rep += hccAccu.rep;
					}
					return res; }
				case id_solver:
				case id_hcc: 
					Range32 r(0, oId == id_solver ? ctx.concurrency() : ctx.sccGraph.get() ? ctx.sccGraph->numNonHcfs() : 0);
					char* x;
					r.lo = (uint32)std::strtoul(path, &x, 10);
					if (x == path) {
						if (!*path) { return keys ? ExpectedQuantity("__len\0") : ExpectedQuantity::error_ambiguous_quantity; }
						if (!keys && std::strcmp(path, "__len") == 0) { return ExpectedQuantity(r.hi); }
						return ExpectedQuantity::error_unknown_quantity;
					}
					if (r.lo >= r.hi)  { return ExpectedQuantity::error_not_available;    }
					if (*(path = x) == '.') { ++path; }
					const SharedContext* active = &ctx; 
					if (oId == id_hcc) {
						active = &(ctx.sccGraph->nonHcfBegin() + r.lo)->second->ctx();
						r      = Range32(0, active->concurrency());
					}
					return keys ? GET_KEYS(active->stats(*active->solver(r.lo), accu), path) : getStat(*active, path, accu, r);
			}
		}
	}
	return ExpectedQuantity::error_unknown_quantity; 
#undef GET_KEYS
#undef GET_OBJ
#undef COMMON_KEYS
}

ExpectedQuantity ClaspFacade::getStat(const char* path)const {
	return config_ && step_.totalTime >= 0.0 ? getStatImpl(path, false) : ExpectedQuantity::error_not_available;
}
const char*      ClaspFacade::getKeys(const char* path)const {
	ExpectedQuantity x =  config_ && step_.totalTime >= 0.0 ? getStatImpl(path, true) : ExpectedQuantity::error_not_available;
	if (x.valid()) { return (const char*)static_cast<uintp>(x.rep); }
	return x.error() == ExpectedQuantity::error_unknown_quantity ? 0 : "\0";
}
ExpectedQuantity ClaspFacade::getStat(const SharedContext& ctx, const char* key, bool accu, const Range<uint32>& r) const {
	if (!key || !*key) { return ExpectedQuantity::error_ambiguous_quantity; }
	ExpectedQuantity res(0.0);
	for (uint32 i = r.lo; i != r.hi && ctx.hasSolver(i); ++i) {
		ExpectedQuantity x = ctx.stats(*ctx.solver(i), accu)[key];
		if (!x.valid()) { return x; }
		res.rep += x.rep;
	}
	return res;	
}
/////////////////////////////////////////////////////////////////////////////////////////
// ClaspFacade::Summary
/////////////////////////////////////////////////////////////////////////////////////////
void ClaspFacade::Summary::init(ClaspFacade& f)  { std::memset(this, 0, sizeof(Summary)); facade = &f;}
int ClaspFacade::Summary::stats()          const { return ctx().master()->stats.level(); }
const Model* ClaspFacade::Summary::model() const { return facade->enum_.get() ? &facade->enum_->lastModel() : 0; }
const SharedMinimizeData* ClaspFacade::Summary::costs()const { return facade->enum_.get() ? facade->enum_->minimizer() : 0;  }
const char* ClaspFacade::Summary::consequences() const {
	int mt = facade->enum_.get() ? facade->enum_->modelType() : 0;
	if ( (mt & CBConsequences::brave_consequences)    == CBConsequences::brave_consequences)   { return "Brave"; }
	if ( (mt & CBConsequences::cautious_consequences) == CBConsequences::cautious_consequences){ return "Cautious"; }
	return 0;
}
bool ClaspFacade::Summary::optimize()  const { return (facade->enum_.get() && facade->enum_->optimize()) || (model() && model()->opt); }
bool ClaspFacade::Summary::optimum()   const { return (model() && model()->opt) || (costs() && complete()); }
uint64 ClaspFacade::Summary::optimal() const { 
	const Model* m = model();
	if (m && m->opt) {
		return !m->consequences() ? std::max(m->num, uint64(1)) : uint64(complete());
	}
	return 0;
}
}
