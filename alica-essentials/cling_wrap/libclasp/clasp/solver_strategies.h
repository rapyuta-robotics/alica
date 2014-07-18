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
#ifndef CLASP_SOLVER_STRATEGIES_H_INCLUDED
#define CLASP_SOLVER_STRATEGIES_H_INCLUDED
#ifdef _MSC_VER
#pragma once
#endif

#include <clasp/constraint.h>
#include <clasp/util/misc_types.h>

/*!
 * \file 
 * Contains strategies and options used to configure solvers and search.
 */
namespace Clasp {

//! Implements clasp's configurable schedule-strategies.
/*!
 * clasp currently supports the following basic strategies:
 *  - geometric sequence  : X = n1 * n2^k   (k >= 0)
 *  - arithmetic sequence : X = n1 + (n2*k) (k >= 0)
 *  - fixed sequence      : X = n1 + (0*k)  (k >= 0)
 *  - luby's sequence     : X = n1 * luby(k)(k >= 0) 
 *  .
 * Furthermore, an inner-outer scheme can be applied to the selected sequence.
 * In that case, the sequence is repeated every <limit>+j restarts, where
 * <limit> is the initial outer-limit and j is the number of times the
 * sequence was already repeated.
 *
 * \note For luby's seqeuence, j is not a repetition counter
 * but the index where the sequence grows to the next power of two.
 *
 * \see Luby et al. "Optimal speedup of las vegas algorithms."
 *
 */
struct ScheduleStrategy {
public:
	enum Type { geometric_schedule = 0, arithmetic_schedule = 1, luby_schedule = 2, user_schedule = 3 }; 
	
	ScheduleStrategy(Type t = geometric_schedule, uint32 b = 100, double g = 1.5, uint32 o = 0);
	//! Creates luby's sequence with unit-length unit and optional outer limit.
	static ScheduleStrategy luby(uint32 unit, uint32 limit = 0)              { return ScheduleStrategy(luby_schedule, unit, 0, limit);  }
	//! Creates geometric sequence base * (grow^k) with optional outer limit.
	static ScheduleStrategy geom(uint32 base, double grow, uint32 limit = 0) { return ScheduleStrategy(geometric_schedule, base, grow, limit);  }
	//! Creates arithmetic sequence base + (add*k) with optional outer limit.
	static ScheduleStrategy arith(uint32 base, double add, uint32 limit = 0) { return ScheduleStrategy(arithmetic_schedule, base, add, limit);  }
	//! Creates fixed sequence with length base.
	static ScheduleStrategy fixed(uint32 base)                               { return ScheduleStrategy(arithmetic_schedule, base, 0, 0);  }
	static ScheduleStrategy none()                                           { return ScheduleStrategy(geometric_schedule, 0); }
	static ScheduleStrategy def()                                            { return ScheduleStrategy(user_schedule, 0, 0.0); }
	uint64 current()  const;
	bool   disabled() const { return base == 0; }
	bool   defaulted()const { return base == 0 && type == user_schedule; }
	void   reset()          { idx  = 0;         }
	uint64 next();
	void   advanceTo(uint32 idx);
	uint32 base : 30; // base of sequence (n1)
	uint32 type :  2; // type of basic sequence
	uint32 idx;       // current index into sequence
	uint32 len;       // length of sequence (0 if infinite) (once reached, sequence is repeated and len increased)
	float  grow;      // update parameter n2
};

uint32 lubyR(uint32 idx);
double growR(uint32 idx, double g);
double addR(uint32 idx, double a);
inline uint32 log2(uint32 x) {
	uint32 ln = 0;
	if (x & 0xFFFF0000u) { x >>= 16; ln |= 16; }
	if (x & 0xFF00u    ) { x >>=  8; ln |=  8; }
	if (x & 0xF0u      ) { x >>=  4; ln |=  4; }
	if (x & 0xCu       ) { x >>=  2; ln |=  2; }
	if (x & 0x2u       ) {/*x>>=1*/; ln |=  1; }
	return ln;
}

class DecisionHeuristic;

struct SolverStrategies {
	//! Clasp's two general search strategies.
	enum SearchStrategy {
		use_learning = 0, /*!< Analyze conflicts and learn First-1-UIP-clause */
		no_learning  = 1  /*!< Don't analyze conflicts - chronological backtracking */
	};
	//! Default sign heuristic.
	enum SignHeu {
		sign_atom = 0, /*!< Prefer negative literal for atoms.                   */
		sign_no   = 1, /*!< Always prefer positive literal.                      */
		sign_yes  = 2, /*!< Always prefer negative literal.                      */
		sign_rnd  = 3, /*!< Prefer random literal.                               */
		sign_disj = 4, /*!< Prefer negative literal for atoms in hcf-components. */
	};
	//! Antecedents to consider during conflict clause minimization.
	enum CCMinAntes {
		no_antes     = 0,  /*!< Don't minimize first-uip-clauses. */
		all_antes    = 1,  /*!< Consider all antecedents.         */
		short_antes  = 2,  /*!< Consider only short antecedents.  */
		binary_antes = 3,  /*!< Consider only binary antecedents. */
	};
	//! Simplifications for long conflict clauses.
	enum CCRepMode {
		cc_no_replace  = 0,/*!< Don't replace literals in conflict clauses. */
		cc_rep_decision= 1,/*!< Replace conflict clause with decision sequence. */
		cc_rep_uip     = 2,/*!< Replace conflict clause with all uip clause. */
		cc_rep_dynamic = 3,/*!< Dynamically select between cc_rep_decision and cc_rep_uip. */
	};
	enum OptHeu {
		opt_sign     = 1,  /*!< Use optimize statements in sign heuristic. */ 
		opt_model    = 2,  /*!< Apply model heuristic when optimizing.     */
	};
	//! Strategy to use when optimization is active.
	enum OptStrategy {
		opt_def      = 0,  /*!< Use default branch and bound optimization.           */  
		opt_hier     = 1,  /*!< Use hierarchical branch and bound optimization.      */
		opt_inc      = 2,  /*!< Use hierarchical optimization with increasing steps. */
		opt_dec      = 3,  /*!< Use hierarchical optimization with decreasing steps. */
		opt_unsat    = 4,  /*!< Use unsat-core based optimization.                   */
		opt_unsat_pre= 5,  /*!< Use unsat-core based optimization with preprocessing.*/
	};
	enum WatchInit  { watch_first = 0, watch_rand = 1, watch_least = 2 };
	enum UpdateMode { update_on_propagate = 0, update_on_conflict  = 1 };

	SolverStrategies();
	void prepare();
	//----- 32 bit ------------	
	uint32    compress      : 16; /*!< If > 0, enable compression for learnt clauses of size > compress. */
	uint32    saveProgress  : 16; /*!< Enable progress saving if > 0. */
	//----- 32 bit ------------
	uint32    reverseArcs   : 2;  /*!< Use "reverse-arcs" during learning if > 0. */
	uint32    otfs          : 2;  /*!< Enable "on-the-fly" subsumption if > 0. */
	uint32    updateLbd     : 2;  /*!< Update lbds of antecedents during conflict analysis. */
	uint32    ccMinAntes    : 2;  /*!< Antecedents to look at during conflict clause minimization. */
	uint32    ccRepMode     : 2;  /*!< One of CCRepMode. */
	uint32    initWatches   : 2;  /*!< Initialize watches randomly in clauses. */
	uint32    optHeu        : 2;  /*!< Set of OptHeu values. */
	uint32    upMode        : 1;  /*!< One of UpdateMode. */
	uint32    bumpVarAct    : 1;  /*!< Bump activities of vars implied by learnt clauses with small lbd. */
	uint32    search        : 1;  /*!< Current search strategy. */
	uint32    restartOnModel: 1;  /*!< Do a restart after each model. */
	uint32    signDef       : 3;  /*!< Default sign heuristic.        */
	uint32    signFix       : 1;  /*!< Disable all sign heuristics and always use default sign. */
	uint32    loadCfg       : 1;  // (Re-)load config 
	uint32    id            : 6;  // Solver id - SHALL ONLY BE SET BY Shared Context!
	uint32    heuReserved   : 3;  // id of active heuristic - SHALL ONLY BE SET BY Solver!
};

//! Parameter-Object for configuring a solver.
struct SolverParams : SolverStrategies  {
	SolverParams();
	uint32 prepare();
	uint32 seed;          /*!< Seed for the random number generator.                        */ 
	// 32-bit
	uint32 heuParam : 16; /*!< Extra parameter for heuristic with meaning depending on type */
	uint32 lookOps  : 16; /*!< Max. number of lookahead operations (0: no limit).           */
	// 32-bit
	uint32 optStrat : 3;  /*!< Optimization strategy.*/
	uint32 heuId    : 3;  /*!< Type of decision heuristic.   */
	uint32 heuOther : 2;  /*!< Consider other learnt nogoods in heuristic (0=no, 1=loops, 2=all, 3=let heuristic decide). */
	uint32 heuReinit: 1;  /*!< Enable/disable reinitialization of existing vars in incremental setting */
	uint32 heuMoms  : 1;  /*!< Use MOMS-score as top-level heuristic */
	uint32 berkHuang: 1;  /*!< Only for Berkmin. */
	uint32 berkOnce : 1;  /*!< Only for Berkmin. */
	uint32 unitNant : 1;  /*!< Only for unit.    */
	uint32 lookType : 2;  /*!< Type of lookahead operations. */
	uint32 loopRep  : 2;  /*!< How to represent loops? */
	uint32 ccMinRec : 1;  /*!< If 1, use more expensive recursive nogood minimization.  */
	uint32 domPref  : 6;  /*!< Only for domain heuristic. */
	uint32 domMod   : 3;  /*!< Only for domain heuristic. */
	uint32 reserved : 5;
};

typedef Range<uint32> Range32;

//! Aggregates restart-parameters to configure restarts during search.
/*!
 * \see ScheduleStrategy
 */
struct RestartParams {
	RestartParams() : sched(), counterRestart(0), counterBump(9973), shuffle(0), shuffleNext(0), upRestart(0), cntLocal(0), dynRestart(0) {}
	enum SeqUpdate { seq_continue = 0, seq_repeat = 1, seq_disable = 2 };
	uint32    prepare(bool withLookback);
	void      disable();
	bool      dynamic() const { return dynRestart != 0; }
	bool      local()   const { return cntLocal   != 0; }
	SeqUpdate update()  const { return static_cast<SeqUpdate>(upRestart); }
	ScheduleStrategy sched;  /**< Restart schedule to use. */
	uint32 counterRestart:16;/**< Apply counter implication bump every counterRestart restarts (0: disable). */
	uint32 counterBump:16;   /**< Bump factor for counter implication restarts. */
	uint32 shuffle    :14;   /**< Shuffle program after shuffle restarts (0: disable). */
	uint32 shuffleNext:14;   /**< Re-Shuffle program every shuffleNext restarts (0: disable). */
	uint32 upRestart  : 2;   /**< How to update restart sequence after a model was found (one of SeqUpdate). */
	uint32 cntLocal   : 1;   /**< Count conflicts globally or relative to current branch? */
	uint32 dynRestart : 1;   /**< Dynamic restarts enabled? */
};

//! Reduce strategy used during solving.
/*!
 * A reduce strategy mainly consists of an algorithm and a scoring scheme
 * for measuring "activity" of learnt constraints.
 */
struct ReduceStrategy {
	//! Reduction algorithm to use during solving.
	enum Algorithm {
		reduce_linear   = 0, /*!< Linear algorithm from clasp-1.3.x. */
		reduce_stable   = 1, /*!< Sort constraints by score but keep order in learnt db. */
		reduce_sort     = 2, /*!< Sort learnt db by score and remove fraction with lowest score. */
		reduce_heap     = 3  /*!< Similar to reduce_sort but only partially sorts learnt db.  */
	};
	//! Score to measure "activity" of learnt constraints.
	enum Score {
		score_act  = 0, /*!< Activity only: how often constraint is used during conflict analysis. */
		score_lbd  = 1, /*!< Use literal block distance as activity. */
		score_both = 2  /*!< Use activity and lbd together. */
	};
	enum EstimateSize {
		est_dynamic         = 0,
		est_con_complexity  = 1,
		est_num_constraints = 2,
		est_num_vars        = 3
	};
	static uint32 scoreAct(const Activity& act)  { return act.activity(); }
	static uint32 scoreLbd(const Activity& act)  { return uint32(128)-act.lbd(); }
	static uint32 scoreBoth(const Activity& act) { return (act.activity()+1) * scoreLbd(act); }
	ReduceStrategy() : glue(0), fReduce(75), fRestart(0), score(0), algo(0), estimate(0), noGlue(0) {}
	static int    compare(Score sc, const Clasp::Activity& lhs, const Clasp::Activity& rhs) {
		int fs = 0;
		if      (sc == score_act) { fs = ((int)scoreAct(lhs)) - ((int)scoreAct(rhs)); }
		else if (sc == score_lbd) { fs = ((int)scoreLbd(lhs)) - ((int)scoreLbd(rhs)); }
		return fs != 0 ? fs : ((int)scoreBoth(lhs)) - ((int)scoreBoth(rhs)); 
	}
	static uint32 asScore(Score sc, const Clasp::Activity& act) {
		if (sc == score_act)  { return scoreAct(act); }
		if (sc == score_lbd)  { return scoreLbd(act); }
		/*  sc == score_both*/{ return scoreBoth(act);}
	}
	uint32 glue    : 8; /*!< Don't remove nogoods with lbd <= glue.    */
	uint32 fReduce : 7; /*!< Fraction of nogoods to remove in percent. */
	uint32 fRestart: 7; /*!< Fraction of nogoods to remove on restart. */
	uint32 score   : 2; /*!< One of Score.                             */
	uint32 algo    : 2; /*!< One of Algorithm.                         */
	uint32 estimate: 2; /*!< How to estimate problem size in init.     */
	uint32 noGlue  : 1; /*!< Do not count glue clauses in limit.       */
};

//! Aggregates parameters for the nogood deletion heuristic used during search.
/*!
 * S:delCond {yes,no}
 * no:del {0}[0]
 * no:del | delCond in {no}
 * deletion | delCond in {yes}
 * del-* | delCond in {yes}
 * {delCond=yes, del-grow=no, del-cfl=no}
 */
struct ReduceParams {
	ReduceParams() : cflSched(ScheduleStrategy::none()), growSched(ScheduleStrategy::def())
		, fInit(1.0f/3.0f)
		, fMax(3.0f)
		, fGrow(1.1f)
		, initRange(10, UINT32_MAX)
		, maxRange(UINT32_MAX)
	  , memMax(0) {}
	void    disable();
	uint32  prepare(bool withLookback);
	Range32 sizeInit(const SharedContext& ctx) const;
	uint32  cflInit(const SharedContext& ctx)  const;
	uint32  getBase(const SharedContext& ctx)  const;
	float   fReduce()  const { return strategy.fReduce / 100.0f; }
	float   fRestart() const { return strategy.fRestart/ 100.0f; }
	static uint32 getLimit(uint32 base, double f, const Range<uint32>& r);
	ScheduleStrategy cflSched;   /**< Conflict-based deletion schedule.               */
	ScheduleStrategy growSched;  /**< Growth-based deletion schedule.                 */
	ReduceStrategy   strategy;   /**< Strategy to apply during nogood deletion.       */
	float            fInit;      /**< Initial limit. X = P*fInit clamped to initRange.*/
	float            fMax;       /**< Maximal limit. X = P*fMax  clamped to maxRange. */
	float            fGrow;      /**< Growth factor for db.                           */
	Range32          initRange;  /**< Allowed range for initial limit.                */
	uint32           maxRange;   /**< Allowed range for maximal limit: [initRange.lo,maxRange]*/
	uint32           memMax;     /**< Memory limit in MB (0 = no limit).              */
};

//! Parameter-Object for grouping solve-related options.
/*!
 * \ingroup solver
 */
struct SolveParams {
	//! Creates a default-initialized object.
	/*!
	 * The following parameters are used:
	 * restart      : quadratic: 100*1.5^k / no restarts after first solution
	 * deletion     : initial size: vars()/3, grow factor: 1.1, max factor: 3.0, do not reduce on restart
	 * randomization: disabled
	 * randomProp   : 0.0 (disabled)
	 */
	SolveParams();
	uint32  prepare(bool withLookback);
	bool    randomize(Solver& s) const;
	RestartParams restart;
	ReduceParams  reduce;
	uint32        randRuns:16; /*!< Number of initial randomized-runs. */
	uint32        randConf:16; /*!< Number of conflicts comprising one randomized-run. */
	float         randProb;    /*!< Use random heuristic with given probability ([0,1]) */
	struct FwdCheck {          /*!< Options for partial checks in DLP-solving; */
		uint32 initHigh : 24;    /*!< Init high level to this DL (0 = max level) */
		uint32 highPct  :  7;    /*!< Check on low + (high - low) * highPct/100  */
		uint32 incHigh  :  1;    /*!< Inc high level when reached. */
		FwdCheck() { *reinterpret_cast<uint32*>(this) = 0; }
	}             fwdCheck;
};

class SharedContext;
class SatPreprocessor;

//! Parameters for (optional) Sat-preprocessing.
struct SatPreParams {
	enum Mode {
		prepro_preserve_sat    = 0, /**< Allow full preprocessing.                 */
		prepro_preserve_models = 1, /**< Only allow model-preserving preprocessing.*/
	};
	enum Type {
		sat_pre_no     = 0, /**< Disable sat-preprocessing.                            */
		sat_pre_ve     = 1, /**< Run variable elimination.                             */
		sat_pre_ve_bce = 2, /**< Run variable- and limited blocked clause elimination. */
		sat_pre_full   = 3, /**< Run variable- and full blocked clause elimination.    */
	};
	SatPreParams() : type(0u), mode(0u), limIters(0u), limTime(0u), limFrozen(0u), limClause(4000u), limOcc(0u) {}
	uint32 type     :  2; /**< Blocked clause elimination (0=off, 1=limited, 2=full).         */
	uint32 mode     :  1; /**< One of mode.                                                   */
	uint32 limIters : 10; /**< Max. number of iterations.                         (0=no limit)*/
	uint32 limTime  : 12; /**< Max. runtime in sec, checked after each iteration. (0=no limit)*/
	uint32 limFrozen:  7; /**< Run only if percent of frozen vars < maxFrozen.    (0=no limit)*/
	uint32 limClause: 16; /**< Run only if #clauses < (limClause*1000)            (0=no limit)*/ 
	uint32 limOcc   : 16; /**< Skip v, if #occ(v) >= limOcc && #occ(~v) >= limOcc.(0=no limit)*/
	bool clauseLimit(uint32 nc)           const { return limClause && nc > (limClause*1000u); }
	bool occLimit(uint32 pos, uint32 neg) const { return limOcc && pos > (limOcc-1u) && neg > (limOcc-1u); }
	uint32 bce()                          const { return type != sat_pre_no ? type - 1 : 0; }
	void   disableBce()                         { type = std::min(type, uint32(sat_pre_ve));}
	static SatPreprocessor* create(const SatPreParams&);
};
	
//! Parameters for a SharedContext object.
struct ContextParams {
	//! How to handle short learnt clauses.
	enum ShortMode  { 
		short_implicit = 0, /*!< Share short learnt clauses via short implication graph. */
		short_explicit = 1, /*!< Do not use short implication graph. */
	};
	//! How to handle physical sharing of (explicit) constraints.
	enum ShareMode  { 
		share_no      = 0, /*!< Do not physically share constraints (use copies instead). */
		share_problem = 1, /*!< Share problem constraints but copy learnt constraints.    */
		share_learnt  = 2, /*!< Copy problem constraints but share learnt constraints.    */
		share_all     = 3, /*!< Share all constraints.                                    */
		share_auto    = 4, /*!< Use share_no or share_all depending on number of solvers. */
	};
	ContextParams() : shareMode(share_auto), stats(0), shortMode(short_implicit), seed(0), reserved(0), cliConfig(0), cliId(0), cliMode(0) {}
	SatPreParams satPre;        /*!< Preprocessing options.                    */
	uint8        shareMode : 3; /*!< Physical sharing mode (one of ShareMode). */
	uint8        stats     : 2; /*!< See SharedContext::enableStats().         */
	uint8        shortMode : 1; /*!< One of ShortMode.                         */
	uint8        seed      : 1; /*!< Apply new seed when adding solvers.       */
	uint8        reserved  : 1; /*!< Reserved for future use.                  */
	uint8        cliConfig;     /*!< Reserved for command-line interface.      */
	uint8        cliId;         /*!< Reserved for command-line interface.      */
	uint8        cliMode;       /*!< Reserved for command-line interface.      */
};

//! Interface for configuring a SharedContext object and its associated solvers.
class Configuration {
public:
	typedef SolverParams  SolverOpts;
	typedef SolveParams   SearchOpts;
	typedef ContextParams CtxOpts;
	virtual ~Configuration();
	//! Prepares this configuration for the usage in the given context.
	virtual void               prepare(SharedContext&)    = 0;
	//! Returns the options for the shared context.
	virtual const CtxOpts&     context()            const = 0;
	//! Returns the number of solver options in this config.
	virtual uint32             numSolver()          const = 0;
	//! Returns the number of search options in this config.
	virtual uint32             numSearch()          const = 0;
	//! Returns the solver options for the i'th solver to be attached to the SharedContext.
	virtual const SolverOpts&  solver(uint32 i)     const = 0;
	//! Returns the search options for the i'th solver of the SharedContext.
	virtual const SearchOpts&  search(uint32 i)     const = 0;
	//! Returns the heuristic to be used in the i'th solver.
	/*!
	 * The function is called in Solver::startInit().
	 * \note The returned object is owned by the caller.
	 */
	virtual DecisionHeuristic* heuristic(uint32 i)  const = 0;
	//! Adds post propagators to the given solver.
	/*!
	 * The function is called during initialization of s.
	 * The default implementation adds a post propagator for unfounded set checking
	 * if necessary. 
	 */
	virtual bool               addPost(Solver& s)   const;
};

//! Base class for user-provided configurations.
class UserConfiguration : public Configuration {
public:
	//! Adds post propagators to the given solver.
	/*!
	 * The function is called during initialization of s.
	 * The default implementation calls Configuration::addPost(s)
	 * and adds a lookahead post propagator if necessary.
	 */
	virtual bool            addPost(Solver& s)   const;
	//! Returns the (modifiable) solver options for the i'th solver.
	virtual SolverOpts&     addSolver(uint32 i) = 0;
	//! Returns the (modifiable) search options for the i'th solver.
	virtual SearchOpts&     addSearch(uint32 i) = 0;
};

class BasicSatConfig : public UserConfiguration, public ContextParams {
public:
	BasicSatConfig();
	void               prepare(SharedContext&);
	const CtxOpts&     context()            const { return *this; }
	uint32             numSolver()          const { return solver_.size(); }
	uint32             numSearch()          const { return search_.size(); }
	const SolverOpts&  solver(uint32 i)     const { return solver_[i % solver_.size() ]; }
	const SearchOpts&  search(uint32 i)     const { return search_[i % search_.size() ]; }
	DecisionHeuristic* heuristic(uint32 i)  const;
	SolverOpts&        addSolver(uint32 i);
	SearchOpts&        addSearch(uint32 i);
	
	virtual void       reset();
	virtual void       resize(uint32 numSolver, uint32 numSearch);
private:
	typedef PodVector<SolverOpts>::type SolverVec;
	typedef PodVector<SearchOpts>::type SearchVec;
	SolverVec solver_;
	SearchVec search_;
};

//! Simple factory for decision heuristics.
struct Heuristic_t {
	enum Type { heu_default = 0, heu_berkmin = 1, heu_vsids = 2, heu_vmtf = 3, heu_domain = 4, heu_unit = 5, heu_none = 6  };
	static inline bool        isLookback(uint32 type) { return type >= (uint32)heu_berkmin && type <= (uint32)heu_vmtf; }
	static DecisionHeuristic* create(const SolverParams&);
};

}
#endif
