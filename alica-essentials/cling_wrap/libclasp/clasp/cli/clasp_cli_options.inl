// 
// Copyright (c) 2013, Benjamin Kaufmann
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
/*!
 * \file 
 * Supermacros for defining clasp's command-line options.
 * An option consists of:
 *  - a name (string)
 *  - a key  (enumeration constant)
 *  - an arg description (ARG macro)
 *  - a description (string)
 *  - an action to be executed when the option is found in a source (macro or block)
 *  . 
 * OPTION("name", key, ARG(...), "help", action).
 *
 */
#if !defined(OPTION) || defined(SELF)
#error Invalid include context
#endif

#if !defined(GROUP_BEGIN)
#define GROUP_BEGIN(X)
#endif

#if !defined(GROUP_END)
#define GROUP_END(X)
#endif

//! Basic solver options.
#if defined(CLASP_SOLVER_BASIC_OPTIONS)
#define SELF CLASP_SOLVER_BASIC_OPTIONS
GROUP_BEGIN(SELF)
OPTION("restart-on-model", restart_on_model, ARG(flag()), "Restart after each model", STORE_FLAG(SELF.restartOnModel))
OPTION("lookahead!"      , lookahead       , ARG(implicit("atom")), \
       "Configure failed-literal detection (fld)\n" \
       "      %A: <type>[,<n {1..umax}>] / Implicit: %I\n" \
       "        <type>: Run fld via {atom|body|hybrid} lookahead\n" \
       "        <n>   : Disable fld after <n> applications ([-1]=no limit)\n" \
       "      --lookahead=atom is default if --no-lookback is used", { \
       PAIR(std::string, uint32) arg("",UINT32_MAX);\
       return (IS_OFF(VALUE) && SET(SELF.lookOps, 0) && SET(SELF.lookType,0))||\
       (CONVERT(VALUE, arg) && SET_ENUM_U(SELF.lookType,arg.first.c_str(),MAP("atom", Lookahead::atom_lookahead), MAP("body", Lookahead::body_lookahead),\
       MAP("hybrid", Lookahead::hybrid_lookahead)) && arg.second > 0 && SET_OR_ZERO(SELF.lookOps, arg.second));})
OPTION("heuristic"       , heuristic, NO_ARG, "Configure decision heuristic\n"  \
       "      %A: {Berkmin|Vmtf|Vsids|Domain|Unit|None}\n" \
       "        Berkmin: Apply BerkMin-like heuristic\n" \
       "        Vmtf   : Apply Siege-like heuristic\n" \
       "        Vsids  : Apply Chaff-like heuristic\n" \
       "        Domain : Apply domain knowledge to Vsids-like heuristic\n"\
       "        Unit   : Apply Smodels-like heuristic (Default if --no-lookback)\n" \
       "        None   : Select the first free variable", STORE_ENUM_U(SELF.heuId, \
       MAP("berkmin", Heuristic_t::heu_berkmin), MAP("vmtf", Heuristic_t::heu_vmtf), \
       MAP("vsids"  , Heuristic_t::heu_vsids)  , MAP("domain", Heuristic_t::heu_domain), \
       MAP("unit", Heuristic_t::heu_unit)      , MAP("none"   , Heuristic_t::heu_none)))
OPTION("init-moms!,@2" , init_moms    , ARG(flag())    , "Initialize heuristic with MOMS-score", STORE_FLAG(SELF.heuMoms))
OPTION("score-other"   , score_other  , ARG(arg("<n>")), "Score {0=no|1=loop|2=all} other learnt nogoods", STORE_LEQ(SELF.heuOther,2u))
OPTION("sign-def"      , sign_def     , ARG(arg("<n>")), "Default sign: {0=asp|1=no|2=yes|3=rnd|4=disj}", STORE_LEQ(SELF.signDef,4u))
OPTION("sign-fix!"     , sign_fix     , ARG(flag())    , "Disable sign heuristics and use default signs only", STORE_FLAG(SELF.signFix))
OPTION("berk-max,@2"   , berk_max     , ARG(arg("<n>")), "Consider at most %A nogoods in Berkmin heuristic", STORE_OR_ZERO(SELF.heuParam))
OPTION("berk-huang!,@2", berk_huang   , ARG(flag())    , "Enable/Disable Huang-scoring in Berkmin", STORE_FLAG(SELF.berkHuang))
OPTION("berk-once!,@2" , berk_once    , ARG(flag())    , "Score sets (instead of multisets) in Berkmin", STORE_FLAG(SELF.berkOnce))
OPTION("vmtf-mtf,@2"   , vmtf_mtf     , ARG(arg("<n>")), "In Vmtf move %A conflict-literals to the front", STORE_OR_FILL(SELF.heuParam))
OPTION("vsids-decay,@2", vsids_decay  , ARG(arg("<n>")), "In Vsids use 1.0/0.<n> as decay factor", STORE_OR_FILL(SELF.heuParam))
OPTION("nant!,@2"      , nant         , ARG(flag())    , "In Unit count only atoms in NAnt(P)", STORE_FLAG(SELF.unitNant))
OPTION("dom-pref,@2"   , dom_pref     , ARG(arg("<bitmask>")), "Prefer {1=atom|2=scc|4=hcc|8=disj|16=min|32=show}", STORE_LEQ(SELF.domPref, 63u))
OPTION("dom-mod,@2"    , dom_mod      , ARG(arg("{0..5}"))   , "Apply {1=level|2=sign+|3=true|4=sign-|5=false}", STORE_LEQ(SELF.domMod, 5u))
OPTION("opt-heuristic" , opt_heuristic, ARG(implicit("1")->arg("{0..3}")), "Use opt. in {1=sign|2=model|3=both} heuristics", STORE_LEQ(SELF.optHeu,  3u))
OPTION("opt-strategy"  , opt_strategy , ARG(arg("{0..5}")->implicit("1")),  "Configure optimization strategy\n" \
       "    Use:\n" \
       "      0: default branch and bound\n"      \
       "      1: hierarchical branch and bound\n" \
       "      2: hierarchical with exponentially increasing steps\n" \
       "      3: hierarchical with exponentially decreasing steps\n" \
       "      4: unsatisfiable-core based optimization\n"            \
       "      5: unsatisfiable-core based optimization with preprocessing\n",STORE_LEQ(SELF.optStrat, 5u))
OPTION("save-progress" , save_progress, ARG(implicit("1")->arg("<n>")), "Use RSat-like progress saving on backjumps > %A", STORE_OR_FILL(SELF.saveProgress))
OPTION("init-watches"  , init_watches , ARG(arg("{0..2}")->defaultsTo("1")->state(Value::value_defaulted)),\
       "Configure watched literal initialization [%D]\n" \
       "      Watch {0=first|1=random|2=least watched} literals in nogoods", STORE_LEQ(SELF.initWatches, 2u))
OPTION("update-mode,@2", update_mode  , NO_ARG, "Process messages on {0=propagation|1=conflict)", STORE_LEQ(SELF.upMode, 1u))
OPTION("seed"          , seed         , ARG(arg("<n>")),"Set random number generator's seed to %A\n", STORE(SELF.seed))
GROUP_END(SELF)
#undef CLASP_SOLVER_BASIC_OPTIONS
#undef SELF
#endif

//! Solver options requiring lookback.
#if defined(CLASP_SOLVER_LOOKBACK_OPTIONS)
#define SELF CLASP_SOLVER_LOOKBACK_OPTIONS
GROUP_BEGIN(SELF)
OPTION("no-lookback"   , no_lookback  , ARG(flag()), "Disable all lookback strategies\n", STORE_FLAG(SELF.search))
OPTION("strengthen!"   , strengthen   , NO_ARG     , "Use MiniSAT-like conflict nogood strengthening\n" \
       "      %A: <mode>[,<type>]\n" \
       "        <mode>: Use {local|recursive} self-subsumption check\n" \
       "        <type>: Follow {0=all|1=short|2=binary} antecedents  [0]", {\
       PAIR(std::string, uint32) arg("local", SolverParams::no_antes);\
       return (IS_OFF(VALUE) && SET(SELF.ccMinAntes,0) && SET(SELF.ccMinRec, 0)) || \
       (CONVERT(VALUE, arg) && ++arg.second <= 3u && SET(SELF.ccMinAntes, arg.second) && SET_ENUM_U(SELF.ccMinRec, arg.first.c_str(), MAP("local", 0), MAP("recursive", 1)));})
OPTION("otfs"          , otfs         , ARG(implicit("1")->arg("{0..2}")), "Enable {1=partial|2=full} on-the-fly subsumption", STORE_LEQ(SELF.otfs, 2u))
OPTION("update-lbd"    , update_lbd   , ARG(implicit("1")->arg("{0..3}")), "Update LBDs of learnt nogoods {1=<|2=strict<|3=+1<}", STORE_LEQ(SELF.updateLbd, 3u))
OPTION("update-act,@2" , update_act   , ARG(flag()), "Enable LBD-based activity bumping", STORE_FLAG(SELF.bumpVarAct))
OPTION("reverse-arcs"  , reverse_arcs , ARG(implicit("1")->arg("{0..3}")), "Enable ManySAT-like inverse-arc learning", STORE_LEQ(SELF.reverseArcs, 3u))
OPTION("contraction!"  , contraction  , NO_ARG, "Configure handling of long learnt nogoods\n"
       "      %A: <n>[,<rep>]\n"\
       "        <n>  : Contract nogoods if size > <n> (0=disable)\n"\
       "        <rep>: Replace literal blocks with {1=decisions|2=uips} ([0]=disable)\n", {UPair arg(0,0);\
       return (IS_OFF(VALUE) || (CONVERT(VALUE,arg) && arg.first != 0u)) && SET_OR_FILL(SELF.compress, arg.first) && SET_LEQ(SELF.ccRepMode, arg.second,3u);})
OPTION("loops"         , loops        , NO_ARG           , "Configure learning of loop nogoods\n" \
       "      %A: {common|distinct|shared|no}\n" \
       "        common  : Create loop nogoods for atoms in an unfounded set\n" \
       "        distinct: Create distinct loop nogood for each atom in an unfounded set\n" \
       "        shared  : Create loop formula for a whole unfounded set\n" \
       "        no      : Do not learn loop formulas", STORE_ENUM_U(SELF.loopRep, \
       MAP("common"  , DefaultUnfoundedCheck::common_reason), MAP("shared", DefaultUnfoundedCheck::shared_reason), \
       MAP("distinct", DefaultUnfoundedCheck::distinct_reason), MAP("no", DefaultUnfoundedCheck::only_reason)))
GROUP_END(SELF)
#undef CLASP_SOLVER_LOOKBACK_OPTIONS
#undef SELF
#endif

//! Basic search-related options.
#if defined(CLASP_SEARCH_BASIC_OPTIONS)
#define SELF CLASP_SEARCH_BASIC_OPTIONS
GROUP_BEGIN(SELF)
OPTION("partial-check", partial_check, ARG(implicit("50")), "Configure partial stability tests\n" \
       "      %A: <p>[,<h>][,<x>] / Implicit: %I\n" \
       "        <p>: Partial check percentage\n"    \
       "        <h>: Initial value for high bound (0 = umax)\n" \
       "        <x>: Increase (1) or keep (0) high bound once reached", {\
       PAIR(uint32, UPair) arg(0, UPair(0,0));\
       return (IS_OFF(VALUE) || (CONVERT(VALUE, arg) && arg.first != 0u)) && SET_LEQ(SELF.fwdCheck.highPct, arg.first, 100u) && SET_OR_ZERO(SELF.fwdCheck.initHigh, arg.second.first) && SET_LEQ(SELF.fwdCheck.incHigh, arg.second.second, 1u);})
OPTION("rand-freq"    , rand_freq, ARG(arg("<p>")), "Make random decisions with probability %A", {\
       double f = 0.0; \
       return (IS_OFF(VALUE) || CONVERT(VALUE, f)) && SET_R(SELF.randProb, (float)f, 0.0f, 1.0f);})
OPTION("rand-prob!"   , rand_prob, ARG(implicit("10,100")), "Configure random probing (Implicit: %I)\n" \
       "      %A: <n1>[,<n2>]\n" \
       "        Run <n1> random passes with at most <n2> conflicts each", {\
       UPair arg(0,100);\
       return (IS_OFF(VALUE)||CONVERT(VALUE, arg)) && SET_OR_FILL(SELF.randRuns, arg.first) && SET_OR_FILL(SELF.randConf, arg.second);})
GROUP_END(SELF)
#undef CLASP_SEARCH_BASIC_OPTIONS
#undef SELF
#endif

//! Options for configuring the restart strategy of a solver.
#if defined(CLASP_SEARCH_RESTART_OPTIONS)
#define SELF CLASP_SEARCH_RESTART_OPTIONS
GROUP_BEGIN(SELF)
OPTION("restarts!,r"     , restarts, ARG(arg("<sched>")), "Configure restart policy\n" \
       "      %A: <type {D|F|L|x|+}>,<n {1..umax}>[,<args>][,<lim>]\n"                    \
       "        F,<n>    : Run fixed sequence of <n> conflicts\n"                         \
       "        L,<n>    : Run Luby et al.'s sequence with unit length <n>\n"             \
       "        x,<n>,<f>: Run geometric seq. of <n>*(<f>^i) conflicts  (<f> >= 1.0)\n"   \
       "        +,<n>,<m>: Run arithmetic seq. of <n>+(<m>*i) conflicts (<m {0..umax}>)\n"\
       "        ...,<lim>: Repeat seq. every <lim>+j restarts           (<type> != F)\n"  \
       "        D,<n>,<f>: Restart based on moving LBD average over last <n> conflicts\n" \
       "                   Mavg(<n>,LBD)*<f> > avg(LBD)\n"                                \
       "                   use conflict level average if <lim> > 0 and avg(LBD) > <lim>\n"\
       "      no|0       : Disable restarts", { return IS_OFF(VALUE) ? (SELF.disable(),true) : \
       CONVERT(VALUE, SELF.sched) && SET(SELF.dynRestart, uint32(SELF.sched.type == ScheduleStrategy::user_schedule));})
OPTION("reset-restarts"  , reset_restarts  , ARG(arg("0..2")->implicit("1")), "{0=Keep|1=Reset|2=Disable} restart seq. after model", STORE_LEQ(SELF.upRestart, 2u))
OPTION("local-restarts"  , local_restarts  , ARG(flag()), "Use Ryvchin et al.'s local restarts", STORE_FLAG(SELF.cntLocal))
OPTION("counter-restarts", counter_restarts, ARG(arg("<n>")), "Do a counter implication restart every <n> restarts", STORE_OR_FILL(SELF.counterRestart))
OPTION("counter-bump,@2" , counter_bump    , ARG(arg("<n>"))    , "Set CIR bump factor to %A", STORE_OR_FILL(SELF.counterBump))
OPTION("shuffle!"        , shuffle         , ARG(arg("<n1>,<n2>")), "Shuffle problem after <n1>+(<n2>*i) restarts\n", {UPair arg(0,0);\
       return (IS_OFF(VALUE)||CONVERT(VALUE, arg)) && SET_OR_FILL(SELF.shuffle, arg.first) && SET_OR_FILL(SELF.shuffleNext, arg.second);})
GROUP_END(SELF)
#undef CLASP_SEARCH_RESTART_OPTIONS
#undef SELF
#endif

//! Options for configuring the deletion strategy of a solver.
#if defined(CLASP_SEARCH_REDUCE_OPTIONS)
#define SELF CLASP_SEARCH_REDUCE_OPTIONS
GROUP_BEGIN(SELF)
OPTION("deletion!,d" , deletion    , ARG(defaultsTo("basic,75,0")->state(Value::value_defaulted)), "Configure deletion algorithm [%D]\n" \
       "      %A: <algo>[,<n {1..100}>][,<sc>]\n"  \
       "        <algo>: Use {basic|sort|ipSort|ipHeap} algorithm\n" \
       "        <n>   : Delete at most <n>%% of nogoods on reduction    [75]\n" \
       "        <sc>  : Use {0=activity|1=lbd|2=combined} nogood scores [0]\n" \
       "      no      : Disable nogood deletion", {\
       PAIR(std::string, UPair) arg("", UPair(75,0));\
       return (IS_OFF(VALUE) && (SELF.disable(), true))||\
       (CONVERT(VALUE, arg) && SET_ENUM_U(SELF.strategy.algo, arg.first.c_str(), MAP("basic", ReduceStrategy::reduce_linear),\
       MAP("sort", ReduceStrategy::reduce_stable), MAP("ipSort", ReduceStrategy::reduce_sort), MAP("ipHeap", ReduceStrategy::reduce_heap)) &&\
       SET_R(SELF.strategy.fReduce, arg.second.first, 1, 100) && SET_LEQ(SELF.strategy.score, arg.second.second, 2));})
OPTION("del-grow!"   , del_grow    , NO_ARG, "Configure size-based deletion policy\n" \
       "      %A: <f>[,<g>][,<sched>] (<f> >= 1.0)\n"          \
       "        <f>     : Keep at most T = X*(<f>^i) learnt nogoods with X being the\n"\
       "                  initial limit and i the number of times <sched> fired\n"     \
       "        <g>     : Stop growth once T > P*<g> (0=no limit)      [3.0]\n"        \
       "        <sched> : Set grow schedule (<type {F|L|x|+}>) [grow on restart]", {\
       PAIR(PAIR(double, double), ScheduleStrategy) arg(std::make_pair(1.0, 3.0), SELF.growSched);\
       if (IS_OFF(VALUE)) { SELF.growSched = ScheduleStrategy::none(); SELF.fGrow = 0.0f; return true; }\
       return CONVERT(VALUE, arg) && (arg.second.defaulted() || arg.second.type != ScheduleStrategy::user_schedule) && SET_R(SELF.fGrow, (float)arg.first.first, 1.0f, FLT_MAX) && SET_R(SELF.fMax, (float)arg.first.second, 0.0f, FLT_MAX) && (SELF.growSched = arg.second, true);})
OPTION("del-cfl!"    , del_cfl     , ARG(arg("<sched>")), "Configure conflict-based deletion policy\n" \
       "      %A:   <type {F|L|x|+}>,<args>... (see restarts)",{\
       return IS_OFF(VALUE) ? (SELF.cflSched=ScheduleStrategy::none()).disabled() : CONVERT(VALUE, SELF.cflSched) && SELF.cflSched.type != ScheduleStrategy::user_schedule;})
OPTION("del-init"  , del_init  , ARG(defaultsTo("3.0")->state(Value::value_defaulted)), "Configure initial deletion limit\n"\
       "      %A: <f>[,<n>,<o>] (<f> > 0)\n" \
       "        <f>    : Set initial limit to P=estimated problem size/<f> [%D]\n" \
       "        <n>,<o>: Clamp initial limit to the range [<n>,<n>+<o>]" , {\
       PAIR(double, UPair) arg(3.0, UPair(SELF.initRange.lo, SELF.initRange.hi));\
       return CONVERT(VALUE, arg) && arg.first > 0 && (SELF.fInit = float(1.0 / arg.first)) > 0\
       && SET(SELF.initRange.lo, arg.second.first)\
       && SET_OR_FILL(SELF.initRange.hi, (uint64(SELF.initRange.lo)+arg.second.second));})
OPTION("del-estimate", del_estimate, ARG(arg("0..3")->implicit("1")), "Use estimated problem complexity in limits", STORE_LEQ(SELF.strategy.estimate, 3u))
OPTION("del-max"     , del_max     , ARG(arg("<n>,<X>")), "Keep at most <n> learnt nogoods taking up to <X> MB", { UPair arg(0,0); \
       return CONVERT(VALUE, arg) && SET_R(SELF.maxRange, arg.first, 1u, UINT32_MAX) && SET(SELF.memMax, arg.second);})
OPTION("del-glue"    , del_glue    , NO_ARG, "Configure glue clause handling\n" \
       "      %A: <n {0..127}>[,<m {0|1}>]\n"                                    \
       "        <n>: Do not delete nogoods with LBD <= <n>\n"                    \
       "        <m>: Count (0) or ignore (1) glue clauses in size limit [0]", {UPair arg(0, 0); \
       return CONVERT(VALUE, arg) && SET_LEQ(SELF.strategy.glue, arg.first, (uint32)Activity::MAX_LBD) && SET(SELF.strategy.noGlue, arg.second);})
OPTION("del-on-restart", del_on_restart, ARG(arg("<n>")->implicit("33")), "Delete %A%% of learnt nogoods on each restart\n", STORE_LEQ(SELF.strategy.fRestart, 100u))
GROUP_END(SELF)
#undef CLASP_SEARCH_REDUCE_OPTIONS
#undef SELF
#endif

//! Options for configuring a SharedContext object stored in a Clasp::ContextParams object.
#if defined(CLASP_CONTEXT_OPTIONS)
#define SELF CLASP_CONTEXT_OPTIONS
GROUP_BEGIN(SELF)
OPTION("stats,s"  , stats, ARG(implicit("1")->arg("{0..2}")), "Maintain {0=no|1=basic|2=extended} statistics", STORE_LEQ(SELF.stats,2u))
OPTION("share!,@1", share, ARG(defaultsTo("auto")->state(Value::value_defaulted)), "Configure physical sharing of constraints [%D]\n" \
       "      %A: {auto|problem|learnt|all}", STORE_ENUM_U(SELF.shareMode,           \
       MAP("no", ContextParams::share_no), MAP("all", ContextParams::share_all),             \
       MAP("auto", ContextParams::share_auto), MAP("problem", ContextParams::share_problem), \
       MAP("learnt", ContextParams::share_learnt)))
OPTION("learn-explicit,@1", learn_explicit, ARG(flag()), "Do not use Short Implication Graph for learning\n", STORE_FLAG(SELF.shortMode))
OPTION("sat-prepro,@1", sat_prepro, ARG(implicit("-1")), "Run SatELite-like preprocessing (Implicit: %I)\n" \
       "      %A: <n1>[,...][,<n5 {0..2}>] (-1=no limit)\n"                          \
       "        <n1>: Run for at most <n1> iterations\n"                             \
       "        <n2>: Run variable elimination with cutoff <n2>              [-1]\n" \
       "        <n3>: Run for at most <n3> seconds                           [-1]\n" \
       "        <n4>: Disable if <n4>%% of vars are frozen                    [-1]\n"\
       "        <n5>: Run blocked clause elimination  {0=no,1=limited,2=full} [1]\n",{\
       uint32 arg[6] = AGGREGATE(uint32(0),UINT32_MAX,UINT32_MAX,UINT32_MAX,uint32(1),SELF.satPre.limClause);\
       return CONVERT_EX(VALUE, arg, 6) && SET_OR_ZERO(SELF.satPre.limIters, arg[0])\
         && SET_OR_ZERO(SELF.satPre.limOcc, arg[1]) && SET_OR_ZERO(SELF.satPre.limTime, arg[2]) && SET_OR_ZERO(SELF.satPre.limFrozen,arg[3])\
         && ++arg[4] && SET_LEQ(SELF.satPre.type, arg[0] != 0 ? arg[4]:arg[0], 3u) && SET_OR_ZERO(SELF.satPre.limClause, arg[5]);})
GROUP_END(SELF)
#undef CLASP_CONTEXT_OPTIONS
#undef SELF
#endif

//! ASP-front-end options stored in an Clasp::Asp::LogicProgram::AspOptions object.
#if defined(CLASP_ASP_OPTIONS)
#define SELF CLASP_ASP_OPTIONS
GROUP_BEGIN(SELF)
OPTION("supp-models"  , supp_models, ARG(flag())    , "Compute supported models (no unfounded set check)", STORE_FLAG(SELF.suppMod))
OPTION("eq,@1"        , eq         , ARG(arg("<n>")), "Configure equivalence preprocessing\n" \
       "      Run for at most %A iterations (-1=run to fixpoint)", STORE_OR_FILL(SELF.iters))
OPTION("backprop!,@1" , backprop   , ARG(flag())    , "Use backpropagation in ASP-preprocessing", STORE_FLAG(SELF.backprop))
OPTION("no-gamma,@1"  , no_gamma   , ARG(flag())    , "Do not add gamma rules for non-hcf disjunctions", STORE_FLAG(SELF.noGamma))
OPTION("eq-dfs,@2"    , eq_dfs     , ARG(flag())    , "Enable df-order in eq-preprocessing", STORE_FLAG(SELF.dfOrder))
OPTION("trans-ext!,@1", trans_ext  , NO_ARG         , "Configure handling of Lparse-like extended rules\n"\
       "      %A: {all|choice|card|weight|integ|dynamic}\n"\
       "        all    : Transform all extended rules to basic rules\n"\
       "        choice : Transform choice rules, but keep cardinality and weight rules\n"\
       "        card   : Transform cardinality rules, but keep choice and weight rules\n"\
       "        weight : Transform cardinality and weight rules, but keep choice rules\n"\
       "        scc    : Transform \"recursive\" cardinality and weight rules\n"\
       "        integ  : Transform cardinality integrity constraints\n"\
       "        dynamic: Transform \"simple\" extended rules, but keep more complex ones", STORE_ENUM(SELF.erMode,\
       MAP("no"    , Asp::LogicProgram::mode_native)          , MAP("all" , Asp::LogicProgram::mode_transform),\
       MAP("choice", Asp::LogicProgram::mode_transform_choice), MAP("card", Asp::LogicProgram::mode_transform_card),\
       MAP("weight", Asp::LogicProgram::mode_transform_weight), MAP("scc" , Asp::LogicProgram::mode_transform_scc),\
       MAP("integ" , Asp::LogicProgram::mode_transform_integ) , MAP("dynamic", Asp::LogicProgram::mode_transform_dynamic)))
GROUP_END(SELF)
#undef CLASP_ASP_OPTIONS
#undef SELF
#endif

//! Options for the solving algorithm.
#if defined(CLASP_SOLVE_OPTIONS)
#define SELF CLASP_SOLVE_OPTIONS
GROUP_BEGIN(SELF)
OPTION("solve-limit" , solve_limit , ARG(arg("<n>[,<m>]")), "Stop search after <n> conflicts or <m> restarts\n", {\
       UPair arg(UINT32_MAX, UINT32_MAX);\
       return ((IS_OFF(VALUE) && *VALUE != '0') || CONVERT(VALUE,arg)) && (SELF.limit=SolveLimits(arg.first == UINT32_MAX ? UINT64_MAX : arg.first, arg.second == UINT32_MAX ? UINT64_MAX : arg.second), true);})
#if defined(WITH_THREADS) && WITH_THREADS == 1
OPTION("parallel-mode,t"   , parallel_mode, NO_ARG, "Run parallel search with given number of threads\n" \
       "      %A: <n {1..64}>[,<mode {compete|split}>]\n"                                           \
       "        <n>   : Number of threads to use in search\n"                                       \
       "        <mode>: Run competition or splitting based search [compete]\n", {\
       PAIR(uint32, const char*) arg(1,"compete");\
       return CONVERT(VALUE, arg) && SET_LEQ(SELF.algorithm.threads, arg.first, 64) && SET_ENUM(SELF.algorithm.mode, arg.second, MAP("compete", SolveOptions::Algorithm::mode_compete), MAP("split", SolveOptions::Algorithm::mode_split));})
OPTION("global-restarts,@1", global_restarts, ARG(implicit("5")->arg("<X>")), "Configure global restart policy\n" \
       "      %A: <n>[,<sched>] / Implicit: %I\n"                         \
       "        <n> : Maximal number of global restarts (0=disable)\n"    \
       "     <sched>: Restart schedule [x,100,1.5] (<type {F|L|x|+}>)\n",{\
       PAIR(uint32, Clasp::ScheduleStrategy) arg;\
       return (IS_OFF(VALUE) || (CONVERT(VALUE, arg) && arg.first != 0u)) && (SELF.restarts.sched=arg.second).type != ScheduleStrategy::user_schedule && SET(SELF.restarts.maxR, arg.first);})
OPTION("distribute!,@1"    , distribute, ARG(defaultsTo("conflict,4")), "Configure nogood distribution [%D]\n" \
       "      %A: <type>[,<lbd {0..127}>][,<size>]\n"                     \
       "        <type> : Distribute {all|short|conflict|loop} nogoods\n"  \
       "        <lbd>  : Distribute only if LBD  <= <lbd>  [4]\n"         \
       "        <size> : Distribute only if size <= <size> [-1]", {\
       PAIR(std::string, UPair) arg("", UPair(4, UINT32_MAX));\
       return (IS_OFF(VALUE) && (SELF.distribute=Distributor::Policy(0,0,0), true)) || (CONVERT(VALUE, arg)\
         && SET(SELF.distribute.lbd, arg.second.first) && SET_OR_FILL(SELF.distribute.size, arg.second.second) && SET_ENUM_U(SELF.distribute.types, arg.first.c_str(),\
         MAP("all", Distributor::Policy::all), MAP("short", Distributor::Policy::implicit), MAP("conflict", Distributor::Policy::conflict), MAP("loop" , Distributor::Policy::loop)));})
OPTION("integrate,@1"      , integrate, ARG(defaultsTo("gp")->state(Value::value_defaulted)), "Configure nogood integration [%D]\n" \
       "      %A: <pick>[,<n>][,<topo>]\n"                                           \
       "        <pick>: Add {all|unsat|gp(unsat wrt guiding path)|active} nogoods\n" \
       "        <n>   : Always keep at least last <n> integrated nogoods   [1024]\n" \
       "        <topo>: Accept nogoods from {all|ring|cube|cubex} peers    [all]\n",{\
       PAIR(std::string, PAIR(uint32, const char*)) arg("", std::make_pair(1024, "all"));\
       return CONVERT(VALUE, arg) && SET_ENUM_U(SELF.integrate.filter, arg.first.c_str(), MAP("all", SolveOptions::Integration::filter_no), MAP("gp", SolveOptions::Integration::filter_gp),\
         MAP("unsat", SolveOptions::Integration::filter_sat), MAP("active", SolveOptions::Integration::filter_heuristic)) && SET_OR_FILL(SELF.integrate.grace, arg.second.first)\
         && SET_ENUM_U(SELF.integrate.topo, arg.second.second, MAP("all" , SolveOptions::Integration::topo_all) , MAP("ring" , SolveOptions::Integration::topo_ring),\
         MAP("cube", SolveOptions::Integration::topo_cube), MAP("cubex", SolveOptions::Integration::topo_cubex));})
#endif	
GROUP_END(SELF)
#undef CLASP_SOLVE_OPTIONS
#undef SELF
#endif

//! Options controlling enumeration stored in a Clasp::EnumOptions object.
#if defined(CLASP_ENUM_OPTIONS)
#define SELF CLASP_ENUM_OPTIONS
GROUP_BEGIN(SELF)
OPTION("enum-mode,e" , enum_mode   , ARG(defaultsTo("auto")->state(Value::value_defaulted)), "Configure enumeration algorithm [%D]\n" \
       "      %A: {bt|record|brave|cautious|auto}\n" \
       "        bt      : Backtrack decision literals from solutions\n" \
       "        record  : Add nogoods for computed solutions\n" \
       "        brave   : Compute brave consequences (union of models)\n" \
       "        cautious: Compute cautious consequences (intersection of models)\n" \
       "        auto    : Use bt for enumeration and record for optimization", STORE_ENUM(SELF.type, \
       MAP("bt", EnumOptions::enum_bt), MAP("record", EnumOptions::enum_record), MAP("brave", EnumOptions::enum_brave), \
       MAP("cautious", EnumOptions::enum_cautious), MAP("auto", EnumOptions::enum_auto)))
OPTION("opt-mode"    , opt_mode    , NO_ARG, "Configure optimization algorithm\n"\
       "      %A: {opt|enum|optN|ignore}\n" \
       "        opt   : Find optimal model\n" \
       "        enum  : Find models with costs <= initial bound\n" \
       "        optN  : Find optimum, then enumerate optimal models\n"\
       "        ignore: Ignore optimize statements", STORE_ENUM(SELF.opt,\
       MAP("opt" , MinimizeMode_t::optimize), MAP("enum"  , MinimizeMode_t::enumerate),\
       MAP("optN", MinimizeMode_t::enumOpt) , MAP("ignore", MinimizeMode_t::ignore)))
OPTION("opt-bound!"  , opt_bound   , ARG(arg("<opt>...")), "Initialize objective function(s)", STORE(SELF.bound) || (IS_OFF(VALUE) && (SELF.bound.clear(), true)))
OPTION("opt-sat"     , opt_sat     , ARG(flag())         , "Treat DIMACS input as MaxSAT optimization problem", STORE(SELF.maxSat))
OPTION("project"     , project     , ARG(implicit("6"))  , "Project models to named atoms", STORE_LEQ(SELF.project,7u))
OPTION("number,n"    , number      , ARG(arg("<n>"))     , "Compute at most %A models (0 for all)\n", STORE(SELF.numModels))
GROUP_END(SELF)
#undef CLASP_ENUM_OPTIONS
#undef SELF
#endif

#undef GROUP_BEGIN
#undef GROUP_END
#undef OPTION
