// 
// Copyright (c) 2009, 2012 Benjamin Kaufmann
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
#ifndef CLASP_LOOKAHEAD_H_INCLUDED
#define CLASP_LOOKAHEAD_H_INCLUDED

#ifdef _MSC_VER
#pragma once
#endif

/*!
 * \file 
 * Defines lookahead related types. Lookahead can be used
 * as a post propagator (e.g. failed-literal detection) and
 * optionally as an heuristic. 
 */

#include <clasp/solver.h>
#include <clasp/constraint.h>
namespace Clasp { 

//! Type used to store lookahead-information for one variable.
struct VarScore {
	VarScore() { clear(); }
	void clear() { std::memset(this, 0, sizeof(VarScore)); }
	//! Mark literal p as dependent.
	void setSeen( Literal p )    { seen_ |= uint32(p.sign()) + 1; }
	//! Is literal p dependent?
	bool seen(Literal p) const   { return (seen_ & (uint32(p.sign())+1)) != 0; }
	//! Is this var dependent?
	bool seen()          const   { return seen_ != 0; }
	//! Mark literal p as tested during lookahead.
	void setTested( Literal p )  { tested_ |= uint32(p.sign()) + 1; }
	//! Was literal p tested during lookahead?
	bool tested(Literal p) const { return (tested_ & (uint32(p.sign())+1)) != 0; }
	//! Was some literal of this var tested?
	bool tested()          const { return tested_ != 0; }
	//! Were both literals of this var tested?
	bool testedBoth()      const { return tested_  == 3; }

	//! Sets the score for literal p to value and marks p as tested.
	void setScore(Literal p, LitVec::size_type value) {
		if (value > (1U<<14)-1) value = (1U<<14)-1;
		if (p.sign()) nVal_ = uint32(value);
		else          pVal_ = uint32(value);
		setTested(p);
	}
	//! Sets the score of a dependent literal p to min(sc, current score).
	void setDepScore(Literal p, uint32 sc) {
		if (!seen(p) || score(p) > sc) {
			if (sc > (1U<<14)-1) sc = (1U<<14)-1;
			if (p.sign()) nVal_ = std::min(uint32(nVal_-(nVal_==0)), sc);
			else          pVal_ = std::min(uint32(pVal_-(pVal_==0)), sc);
		}
	}
	//! Returns the score for literal p.
	uint32 score(Literal p) const { return p.sign() ? nVal_ : pVal_; }
	//! Returns the scores of the two literals of a variable.
	/*!
	 * \param[out] mx The maximum score.
	 * \param[out] mn The minimum score.
	 */
	void score(uint32& mx, uint32& mn) const {
		if (nVal_ > pVal_) {
			mx = nVal_;
			mn = pVal_;
		}
		else {
			mx = pVal_;
			mn = nVal_;
		}
	}
	//! Returns the sign of the literal that has the higher score.
	bool prefSign() const { return nVal_ > pVal_; }

	uint32 nVal() const { return nVal_; }
	uint32 pVal() const { return pVal_; }
private:
	uint32 pVal_  : 14;
	uint32 nVal_  : 14;
	uint32 seen_  : 2;
	uint32 tested_: 2;
};

//! A small helper class used to score the result of a lookahead operation.
struct ScoreLook {
	enum Mode { score_max, score_max_min };
	typedef PodVector<VarScore>::type VarScores; /**< A vector of variable-scores */
	ScoreLook() : best(0), mode(score_max), addDeps(true), nant(false) {}
	bool    validVar(Var v) const { return v < score.size(); }
	void    scoreLits(const Solver& s, const Literal* b, const Literal *e);
	void    clearDeps();
	uint32  countNant(const Solver& s, const Literal* b, const Literal *e) const;
	bool    greater(Var lhs, Var rhs)const;
	bool    greaterMax(Var x, uint32 max) const { 
		return score[x].nVal() > max || score[x].pVal() > max; 
	}
	bool    greaterMaxMin(Var lhs, uint32 max, uint32 min) const {
		uint32 lhsMin, lhsMax;
		score[lhs].score(lhsMax, lhsMin);
		return lhsMin > min || (lhsMin == min && lhsMax > max);
	}
	VarScores score;  // score[v] stores lookahead score of v
	VarVec    deps;   // tested vars and those that follow from them
	VarType   types;  // var types to consider
	Var       best;   // var with best score among those in deps
	Mode      mode;   // score mode
	bool      addDeps;// add/score dependent vars?
	bool      nant;   // score only atoms in NAnt(P)?
};

class UnitHeuristic;

//! Lookahead extends propagation with failed-literal detection. 
/*!
 * The class provides different kinds of one-step lookahead.
 * Atom- and body-lookahead are uniform lookahead types, where
 * either only atoms or bodies are tested. Hybrid-lookahead tests
 * both types of vars but each only in a single phase. I.e. atoms
 * are only tested negatively while bodies are tested positively.
 */
class Lookahead : public PostPropagator {
public:
	//! Defines the supported lookahead-types
	enum Type { 
		no_lookahead=0,   /**< Dummy value */
		atom_lookahead,   /**< Test only atoms in both phases */
		body_lookahead,   /**< Test only bodies in both phases */
		hybrid_lookahead, /**< Test atoms and bodies but only their preferred decision literal */
	};
	struct Params {
		Params(Type t = atom_lookahead) : type(t), topLevelImps(true), restrictNant(false) {}
		Params& lookahead(Type t){ type         = t; return *this; }
		Params& addImps(bool b)  { topLevelImps = b; return *this; }
		Params& nant(bool b)     { restrictNant = b; return *this; }
		Type type;
		bool topLevelImps;
		bool restrictNant;
	};
	static bool isType(uint32 t) { return t != 0 && t <= hybrid_lookahead; }
	/*!
	 * \param t Lookahead-type to use.
	 */
	explicit Lookahead(const Params& p);
	~Lookahead();
	
	bool    init(Solver& s);
	//! Clears the lookahead list.
	void    clear();
	//! Returns true if lookahead list is empty.
	bool    empty() const { return head()->next == head_id; }
	//! Adds literal p to the lookahead list.
	void    append(Literal p, bool testBoth);
	//! Single-step lookahead on all vars in the loookahead list.
	bool    propagateFixpoint(Solver& s, PostPropagator*);
	//! Returns PostPropagator::priority_reserved_look.
	uint32  priority() const;
	void    destroy(Solver* s, bool detach);
	//! Updates state with lookahead result.
	ScoreLook score;
	//! Returns "best" literal w.r.t scoring of last lookahead or posLit(0) if no such literal exists.
	Literal heuristic(Solver& s);

	void    setLimit(UnitHeuristic* n);
protected:
	bool propagateLevel(Solver& s); // called by propagate
	void undoLevel(Solver& s);
	bool test(Solver& s, Literal p);
private:
	typedef uint32 NodeId;
	enum { head_id = NodeId(0), undo_id = NodeId(1) };
	struct LitNode {
		LitNode(Literal x) : lit(x), next(UINT32_MAX) {}
		Literal  lit;
		NodeId   next;
	};
	typedef PodVector<NodeId>::type  UndoStack;
	typedef PodVector<LitNode>::type LookList;
	typedef UnitHeuristic*           HeuPtr;
	void splice(NodeId n);
	LitNode* node(NodeId n) { return &nodes_[n]; }
	LitNode* head()         { return &nodes_[head_id]; } // head of circular candidate list
	LitNode* undo()         { return &nodes_[undo_id]; } // head of undo list
	bool     checkImps(Solver& s, Literal p);
	const LitNode* head() const { return &nodes_[head_id]; }
	LookList   nodes_; // list of literals to test
	UndoStack  saved_; // stack of undo lists
	LitVec     imps_;  // additional top-level implications 
	NodeId     last_;  // last candidate in list; invariant: node(last_)->next == head_id;
	NodeId     pos_;   // current lookahead start position
	uint32     top_;   // size of top-level
	HeuPtr     limit_; // 
};

//! Heuristic that uses the results of lookahead.
/*!
 * The heuristic creates and installs a Lookahead post propagator.
 * It then selects the literal with the highest score, 
 * where the score is determined by counting assignments made during
 * failed-literal detection. hybrid_lookahead simply selects the literal that
 * derived the most literals. uniform_lookahead behaves similar to the smodels
 * lookahead heuristic (the literal that maximizes the minimum is selected).
 * \see Patrik Simons: "Extending and Implementing the Stable Model Semantics" for a
 * detailed description of the lookahead heuristic.
 * 
 * \note The heuristic might itself apply some lookahead but only on variables that 
 *       did not fail in a previous call to Lookahead::propagateFixpoint(). I.e. if
 *       priorities are correct for all post propagators in s, the lookahead operations can't fail. 
 */
class UnitHeuristic : public DecisionHeuristic {
public:
	/*!
	 * \param p Lookahead parameters to apply during lookahead.
	 */
	explicit UnitHeuristic(const Lookahead::Params& p);
	//! Decorates the heuristic given in other with temporary lookahead of type t.
	static UnitHeuristic* restricted(const Lookahead::Params& p, uint32 numOps, DecisionHeuristic* other);
	void endInit(Solver& s);
	void updateVar(const Solver& s, Var v, uint32 n);
	Literal doSelect(Solver& s);
	virtual bool notify(Solver&) { return true; }
protected:
	typedef SingleOwnerPtr<Lookahead> LookPtr;
	LookPtr look_; // lookahead propagator
};

}
#endif
