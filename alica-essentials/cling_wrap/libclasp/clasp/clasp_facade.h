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
#ifndef CLASP_CLASP_FACADE_H_INCLUDED
#define CLASP_CLASP_FACADE_H_INCLUDED

#ifdef _MSC_VER
#pragma warning (disable : 4200) // nonstandard extension used : zero-sized array
#pragma once
#endif

#if !defined(CLASP_VERSION)
#define CLASP_VERSION "3.0.1"
#endif
#if !defined(CLASP_LEGAL)
#define CLASP_LEGAL \
"Copyright (C) Benjamin Kaufmann\n"\
"License GPLv2+: GNU GPL version 2 or later <http://gnu.org/licenses/gpl.html>\n"\
"clasp is free software: you are free to change and redistribute it.\n"\
"There is NO WARRANTY, to the extent permitted by law."
#endif

#if !defined(WITH_THREADS)
#error Invalid thread configuration - use WITH_THREADS=0 for single-threaded or WITH_THREADS=1 for multi-threaded version of libclasp!
#endif

#if WITH_THREADS
#include <clasp/parallel_solve.h>
namespace Clasp { typedef Clasp::mt::ParallelSolveOptions SolveOptions; }
#else
#include <clasp/shared_context.h>
#include <clasp/solve_algorithms.h>
namespace Clasp { typedef Clasp::BasicSolveOptions SolveOptions; }
#endif

#include <clasp/program_builder.h>
#include <clasp/logic_program.h>
#include <clasp/enumerator.h>
/*!
 * \file 
 * This file provides a facade around the clasp library. 
 * I.e. a simplified interface for (incrementally) solving a problem using
 * some configuration (set of parameters).
 */
namespace Clasp {
/////////////////////////////////////////////////////////////////////////////////////////
// Configuration
/////////////////////////////////////////////////////////////////////////////////////////	
//! Configuration object for configuring solving via the ClaspFacade.
class ClaspConfig : public BasicSatConfig {
public:
	typedef BasicSatConfig UserConfig;
	typedef Solver**       SolverIt;
	typedef Asp::LogicProgram::AspOptions AspOptions;	
	ClaspConfig();
	~ClaspConfig();
	// Base interface
	void         prepare(SharedContext&);
	void         reset();
	// own interface
	UserConfig*  testerConfig() const { return tester_; } 
	UserConfig*  addTesterConfig();
	void         setSolvers(uint32 n);
	
	SolveOptions solve;    /*!< Options for solve algorithm.        */
	EnumOptions  enumerate;/*!< Options for enumerator.             */
	AspOptions   asp;      /*!< Options for asp preprocessing.      */
private:
	ClaspConfig(const ClaspConfig&);
	ClaspConfig& operator=(const ClaspConfig&);
	UserConfig* tester_;
};
/////////////////////////////////////////////////////////////////////////////////////////
// ClaspFacade
/////////////////////////////////////////////////////////////////////////////////////////
//! A (positive) numeric value.
struct ExpectedQuantity {
	enum Error { error_none = 0, error_unknown_quantity = 1, error_ambiguous_quantity = 2, error_not_available = 3 };
	ExpectedQuantity(double d);
	ExpectedQuantity(uint32 x) : rep(static_cast<double>(x)) {}
	ExpectedQuantity(uint64 x) : rep(static_cast<double>(x)) {}
	explicit ExpectedQuantity(const void* x) : rep(static_cast<double>(reinterpret_cast<uintp>(x))) {}
	ExpectedQuantity(Error e);
	bool     valid()  const { return error() == error_none; }
	Error    error()  const;
	operator double() const;
	double rep;
};

//! Provides a simplified interface to the services of the clasp library.
class ClaspFacade : public EventHandler {
	struct SolveImpl;
public:
	//! Result of a solving step.
	struct Result {
		//! Possible solving results.
		enum Base {
			UNKNOWN  = 0, /**< Satisfiability unknown - a given solve limit was hit.  */
			SAT      = 1, /**< Problem is satisfiable (a model was found).            */
			UNSAT    = 2, /**< Problem is unsatisfiable.                              */
		};
		enum Ext {
			EXT_EXHAUST  = 4, /**< Search space is exhausted.                             */
			EXT_INTERRUPT= 8, /**< The run was interrupted from outside.                  */
			EXT_ERROR    = 16,/**< The run was terminated because of an internal error.   */
		};
		bool sat()        const { return *this == SAT; }
		bool unsat()      const { return *this == UNSAT; }
		bool unknown()    const { return *this == UNKNOWN; }
		bool exhausted()  const { return (flags & EXT_EXHAUST)   != 0; }
		bool interrupted()const { return (flags & EXT_INTERRUPT) != 0; }
		bool error()      const { return (flags & EXT_ERROR)     != 0; }
		operator Base()   const { return static_cast<Base>(flags & 3u);}
		operator double() const { return (double(signal)*256.0) + flags; }
		uint8 flags;  // result flags
		uint8 signal; // term signal or 0
	};
	//! Type summarizing one or more solving steps.
	struct Summary {
		typedef SharedMinimizeData SharedMinData;
		void init(ClaspFacade& f);
		const SharedContext& ctx()          const { return facade->ctx; }	
		const Asp::LpStats*  lpStats()      const { return facade->lpStats_.get(); }
		uint64               enumerated()   const { return numEnum; }
		bool                 sat()          const { return result.sat();   }
		bool                 unsat()        const { return result.unsat(); }
		bool                 complete()     const { return result.exhausted(); }
		bool                 optimum()      const;
		uint64               optimal()      const; 
		bool                 optimize()     const;
		const SharedMinData* costs()        const;
		const char*          consequences() const;
		int                  stats()        const;
		const Model*         model()        const;
		const ClaspFacade* facade;    /**< Facade object of this run.          */
		double             totalTime; /**< Total wall clock time.              */
		double             cpuTime;   /**< Total cpu time.                     */
		double             solveTime; /**< Wall clock time for solving.        */
		double             unsatTime; /**< Wall clock time to prove unsat.     */
		double             satTime;   /**< Wall clock time to first model.     */
		uint64             numEnum;   /**< Total models enumerated.            */
		uint32             step;      /**< Step number (incremental solving).  */
		Result             result;    /**< Result of step.                     */
	};
	//! Event type used to signal that a solve step has started.
	struct StepStart : Event_t<StepStart> {
		explicit StepStart(const ClaspFacade& f) : Event_t<StepStart>(subsystem_facade, verbosity_quiet), facade(&f) {}
		const ClaspFacade* facade;
	};
	//! Event type used to signal that a solve step has terminated.
	struct StepReady : Event_t<StepReady> {
		explicit StepReady(const Summary& x) : Event_t<StepReady>(subsystem_facade, verbosity_quiet), summary(&x) {}
		const Summary* summary;
	};
	ClaspFacade();
	~ClaspFacade();
	SharedContext ctx; /*!< Context-object used to store problem. */

	/*!
	 * \name Start functions
	 * Functions for defining a problem. 
	 * Calling one of the start functions discards any previous problem.
	 * The allowUpdate parameter determines whether or not program updates
	 * are allowed once the problem is initially defined.
	 */
	//@{
	//! Starts definition of an ASP-problem.
	Asp::LogicProgram& startAsp(ClaspConfig& config, bool allowUpdate = false);
	//! Starts definition of a SAT-problem.
	SatBuilder&        startSat(ClaspConfig& config, bool allowUpdate = false);
	//! Starts definition of a PB-problem.
	PBBuilder&         startPB(ClaspConfig& config , bool allowUpdate = false);
	//! Starts definition of a problem of type t.
	ProgramBuilder&    start(ClaspConfig& config, ProblemType t, bool allowUpdate = false);
	
	enum EnumMode { enum_volatile, enum_static };
	//! Finishes the definition of a problem and prepares it for solving.
	/*!
	 * Once the problem is prepared, call solve() to solve it.
	 * \param m Mode to be used for all enumeration-related knowledge. If m is
	 *          enum_volatile, all enumeration knowledge is learnt under an assumption
	 *          that is retracted on program update. Otherwise, no special assumption is
	 *          used and enumeration-related knowledge might become unretractable.
	 */
	bool               prepare(EnumMode m = enum_volatile);
	//! Adds p to the list of assumptions under which solving should operate.
	void               assume(Literal p);
	//! Calls assume(p) for all literals p in ext.
	void               assume(const LitVec& ext);
	//! Starts update of the active problem.
	/*!
	 * \pre start() was called with allowUpdate and solving() is false.
	 */
	ProgramBuilder&    update(bool updateConfig = false);
	//@}
	
	//! Solves the current prepared problem.
	/*!
	 * \pre !solving()
	 */
	Result             solve(Clasp::EventHandler* onModel = 0);
#if WITH_THREADS
	//! A type for accessing the result of an asynchronous solve operation.
	class AsyncResult {
	public:
		explicit AsyncResult(SolveImpl& x);
		typedef  StepReady Ready;
		// blocking operations
		//! Waits until the result is ready and returns it.
		Result get()              const;
		//! Waits until the result is ready.
		void   wait()             const;
		//! Waits for at most sec seconds.
		bool   waitFor(double sec)const;
		//! Tries to cancel the active async operation.
		bool   cancel()           const;

		// non-blocking operations
		//! Tests whether the result is ready.
		bool   ready()            const;
		//! Returns the result in r if it is ready.
		bool   ready(Result& r)   const;
		//! Tests whether the operation was interrupted and if so returns the interruption signal.
		int    interrupted()      const;
		//! Tests whether the asynchronous result is ready and has a stored exception.
		bool   error()            const;
	private:
		SolveImpl* state_;
	};
	//! Asynchronously solves the current prepared problem.
	/*!
	 * \pre !solving()
	 */
	AsyncResult        solveAsync(Clasp::EventHandler* handler = 0);
#endif
	//! Returns whether the problem is still valid.
	bool               ok()                  const { return program() ? program()->ok() : ctx.ok(); }
	//! Tries to terminate an active solve operation.
	bool               terminate(int signal);
	const Summary&     shutdown();
	//! Returns whether a solve operation is currently active.
	bool               solving()             const;
	//! Returns the summary of the active step.
	const Summary&     summary()             const { return step_; }
	//! Returns the active configuration.
	const ClaspConfig* config()              const { return config_;}
	//! Returns the current incremental step (starts at 0).
	int                step()                const { return (int)step_.step;}
	//! Returns the result of the active step. (unknown if run is not yet completed).
	Result             result()              const { return step_.result; }
	//! Returns the active program or 0 if it was already released.
	ProgramBuilder*    program()             const { return builder_.get(); }
	
	ExpectedQuantity   getStat(const char* path)const;
	const char*        getKeys(const char* path)const;
private:
	typedef SingleOwnerPtr<ProgramBuilder>  BuilderPtr;
	typedef SingleOwnerPtr<Asp::LpStats>    LpStatsPtr;
	typedef SingleOwnerPtr<Enumerator>      EnumPtr;
	typedef SingleOwnerPtr<SolveImpl>       SolvePtr;
	typedef SingleOwnerPtr<Summary>         SummaryPtr;
	ExpectedQuantity getStatImpl(const char* path, bool keys)const;
	ExpectedQuantity getStat(const SharedContext& ctx, const char* key, bool accu, const Range<uint32>& r) const;
	void   init(ClaspConfig& cfg, bool discardProblem);
	void   initBuilder(ProgramBuilder* in, bool incremental);
	void   discardProblem();
	void   startStep(uint32 num);
	Result stopStep(int signal, bool complete);
	void   accuStep();
	bool   onModel(const Solver& s, const Model& m);
	ClaspConfig* config_;
	BuilderPtr   builder_;
	EnumPtr      enum_;
	LpStatsPtr   lpStats_;
	SolvePtr     solve_;
	EventHandler*stepHandler_;
	SummaryPtr   accu_;
	LitVec       assume_;
	Summary      step_;
};
}
#endif
