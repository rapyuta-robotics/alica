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
 * Supermacros for defining clasp's default configurations.
 * A configuration consists of a name, a string of global options, and a string of solver options, i.e.
 * CONFIG(name, "global options", "solver options").
 *
 * \note Global options are ignored in the default portfolio.
 *
 */

#if !defined(CONFIG) || (!defined(CLASP_CLI_DEFAULT_CONFIGS) && !defined(CLASP_CLI_AUX_CONFIGS))
#error Invalid include context
#endif

//! Named default configurations accessible via option "--configuration".
#if defined(CLASP_CLI_DEFAULT_CONFIGS)
CLASP_CLI_DEFAULT_CONFIGS
CONFIG(tweety, "--eq=3 --trans-ext=dynamic", "--del-init=1000,17526 --del-max=2000000 --strengthen=recursive,0 --otfs=2 --heuristic=Vsids --vsids-decay=92 --init-moms --score-other=2 --deletion=basic,50,0 --del-cfl=+,2000,100,20 --del-grow=0 --del-glue=2,0 --update-lbd=1 --del-estimate=1 --save-progress=160 --init-watches=2 --restarts=L,60 --local-restarts --loops=shared")
CONFIG(trendy, "--sat-p=20,25,240,-1,1 --trans-ext=dynamic", "--heuristic=Vsids --restarts=D,100,0.7 --deletion=basic,50,0 --del-init=3.0,500,19500 --del-grow=1.1,20.0,x,100,1.5 --del-cfl=+,10000,2000 --del-glue=2 --strengthen=recursive --update-lbd --otfs=2 --save-p=75 --counter-restarts=3 --counter-bump=1023 --reverse-arcs=2  --contraction=250 --loops=common --opt-heu=1 --opt-strat=5")
CONFIG(frumpy, "--eq=5", "--heuristic=Berkmin --restarts=x,100,1.5 --deletion=basic,75 --del-init=3.0,200,40000 --del-max=400000 --contraction=250 --loops=common --save-p=180 --del-grow=1.1 --strengthen=local --sign-def=4")
CONFIG(crafty, "--sat-p=10,25,240,-1,1 --trans-ext=dynamic --backprop --save-p=180", "--heuristic=Vsids --restarts=x,128,1.5 --deletion=basic,75,0 --del-init=10.0,1000,9000 --del-grow=1.1,20.0 --del-cfl=+,10000,1000 --del-glue=2 --otfs=2 --reverse-arcs=1 --counter-restarts=3 --contraction=250 --opt-heu=1 --opt-strat=1")
CONFIG(jumpy , "--sat-p=20,25,240,-1,1 --trans-ext=dynamic", "--heuristic=Vsids --restarts=L,100 --deletion=basic,75,2 --del-init=3.0,1000,20000 --del-grow=1.1,25,x,100,1.5 --del-cfl=x,10000,1.1 --del-glue=2 --update-lbd=3 --strengthen=recursive --otfs=2 --save-p=70 --opt-heu=3 --opt-strat=2")
CONFIG(handy , "--sat-p=10,25,240,-1,1 --trans-ext=dynamic --backprop", "--heuristic=Vsids --restarts=D,100,0.7 --deletion=sort,50,2 --del-max=200000 --del-init=20.0,1000,14000 --del-cfl=+,4000,600 --del-glue=2 --update-lbd --strengthen=recursive --otfs=2 --save-p=20 --contraction=600 --loops=distinct --counter-restarts=7 --counter-bump=1023 --reverse-arcs=2")
#undef CLASP_CLI_DEFAULT_CONFIGS
#endif
//! Auxiliary configurations accessible via default portfolio ("--configuration=many").
#if defined(CLASP_CLI_AUX_CONFIGS)
CLASP_CLI_AUX_CONFIGS
CONFIG(strong, "","--heuristic=Berkmin --restarts=x,100,1.5 --deletion=basic,75 --del-init=3.0,200,40000 --del-max=400000 --contraction=250 --loops=common --berk-max=512 --del-grow=1.1,25 --otfs=2 --reverse-arcs=2 --strengthen=recursive --init-w=2 --lookahead=atom,10")
CONFIG(s2,     "","--heuristic=Vsids --reverse-arcs=1 --otfs=1 --local-restarts --save-progress=0 --contraction=250 --counter-restart=7 --counter-bump=200 --restarts=x,100,1.5 --del-init=3.0,800,-1 --deletion=basic,60,0 --strengthen=local --del-grow=1.0,1.0 --del-glue=4 --del-cfl=+,4000,300,100")
CONFIG(s4,     "","--heuristic=Vsids --restarts=L,256 --counter-restart=3 --strengthen=recursive --update-lbd --del-glue=2 --otfs=2 --deletion=ipSort,75,2 --del-init=20.0,1000,19000")
CONFIG(slow,   "","--heuristic=Berkmin --berk-max=512 --restarts=F,16000 --lookahead=atom,50")
CONFIG(vmtf,   "","--heuristic=Vmtf --strengthen=no --contr=0 --restarts=x,100,1.3 --del-init=3.0,800,9200")
CONFIG(simple, "","--heuristic=Vsids --strengthen=recursive --restarts=x,100,1.5,15 --contraction=0")
CONFIG(lubysp, "","--heuristic=Vsids --restarts=L,128 --save-p --otfs=1 --init-w=2 --contr=0 --opt-heu=3")
CONFIG(localr, "","--heuristic=Berkmin --berk-max=512 --restarts=x,100,1.5,6 --local-restarts --init-w=2 --contr=0")
CONFIG(nolearn,"","--no-lookback --heuristic=Unit --lookahead=atom --deletion=no --restarts=no")
#undef CLASP_CLI_AUX_CONFIGS
#endif
#undef CONFIG
