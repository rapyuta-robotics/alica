#script (lua) 
function main(prg)
    local step   = 0
    while true do
        if step > 0 then
            prg:ground("trans", {step})
            prg:releaseExternal(gringo.Fun("vol", {step-1}))
        else
            prg:ground("base", {})
        end
        prg:ground("state", {step})
        prg:assignExternal(gringo.Fun("vol", {step}), true)
        local ret = prg:solve()
        if ret == gringo.SolveResult.SAT then
            break
        end
        step = step+1
    end
end
#end.

#program base.

complement(F,neg(F)) :- fluent(F).
complement(neg(F),F) :- fluent(F).

condition(A,true,1)  :- action(A).
condition(A,N)       :- condition(A,L,N).

contradict(F)        :- fluent(F), effect(A,F,N1), effect(A,neg(F),N2),
                        #false : condition(A,M,N1), condition(A,L,N2), complement(L,M).
% Note: this is certainly never a good idea...
1 { holds(F,0); holds(neg(F),0) } 1 :- fluent(F).
holds(L,0) :- initially(L).

oneof(N,B) :- initially_oneof(N,L,B).
oneof(N)   :- oneof(N,B).
init(N,B)  :- oneof(N,B), holds(L,0) : initially_oneof(N,L,B).
 :- oneof(N), not 1 { init(N,B) : oneof(N,B) } 1.
 :- holds(L,0), initially_oneof(N,L,B), not init(N,B).

#program state(t).
#external vol(t).

holds(true,t).
goal_or(N,t) :- goal_or(N,L,B), vol(t).
 :- goal_or(N,t), not holds(L,t) : goal_or(N,L,B); vol(t).
 :- goal(L),      not holds(L,t),                  vol(t).

alt_holds(true,0..t,t) :- vol(t).

alt_holds(F,0,t); alt_holds(neg(F),0,t) :- fluent(F), not initially(F;neg(F)), vol(t).
alt_holds(L,0,t) :- initially(L), vol(t).

possible(T,t)         :- occurs(A,T), alt_holds(C,T-1,t) : executable(A,C); vol(t).
neg_applies(N,T,t)    :- occurs(A,T), condition(A,C,N), complement(C,D), alt_holds(D,T-1,t), vol(t).
neg_force(F,T,t)      :- occurs(A,T), fluent(F), neg_applies(N,T,t) : effect(A,F,N); vol(t).
neg_force(neg(F),T,t) :- occurs(A,T), fluent(F), neg_applies(N,T,t) : effect(A,neg(F),N); vol(t).
consistent(F,T,t)     :- contradict(F), neg_force(F,T,t), vol(t).
consistent(F,T,t)     :- contradict(F), neg_force(neg(F),T,t), vol(t).

alt_holds(L,T,t)      :- occurs(A,T), effect(A,L,N), condition(A,N), alt_holds(C,T-1,t) : condition(A,C,N); vol(t).
alt_holds(L,T,t)      :- alt_holds(L,T-1,t), neg_force(M,T,t), complement(L,M), vol(t).

bottom(t) :- fluent(F), initially(F;neg(F)), vol(t).

pos_init(N,B,t) :- vol(t), oneof(N,B),                              alt_holds(L,0,t) : initially_oneof(N,L,B).
neg_init(N,B,t) :- vol(t), initially_oneof(N,L,B), complement(L,M), alt_holds(M,0,t).
bottom(t) :- vol(t), oneof(N), 2 { pos_init(N,B,t) : oneof(N,B) }.
bottom(t) :- vol(t), oneof(N),     neg_init(N,B,t) : oneof(N,B).

alt_goal_or(N,t) :- goal_or(N,L,B), alt_holds(L,t,t), vol(t).
alt_goal_or(N,t) :- goal_or(N,L,B), goal(L), vol(t).
bottom(t) :- alt_goal_or(N,t)  : goal_or(N); 
             alt_holds(L,t,t)  : goal(L);
             possible(R,t)     : R = 1..t; 
             consistent(F,S,t) : contradict(F), S = 1..t;
             vol(t).

alt_holds(F,0,t)      :- bottom(t), fluent(F), not initially(F;neg(F)), vol(t).
alt_holds(neg(F),0,t) :- bottom(t), fluent(F), not initially(F;neg(F)), vol(t).

:- not bottom(t), vol(t).

#program trans(t).

1 { occurs(A,t) : action(A) } 1.
 :- occurs(A,t), executable(A,C), not holds(C,t-1).

holds(L,t)  :- occurs(A,t), effect(A,L,N), condition(A,N), holds(C,t-1) : condition(A,C,N).
holds(L,t)  :- holds(L,t-1), complement(L,M), not holds(M,t).
 :- holds((F;neg(F)),t), contradict(F).

#show occurs/2.

#program base.

fluent(cpa_started).
fluent(cpa_visited(cpa_n1)).
fluent(cpa_visited(cpa_n2)).
fluent(cpa_at(cpa_n1)).
fluent(cpa_at(cpa_n2)).

%% actions ------ 

action(cpa_start(cpa_n1)).
action(cpa_start(cpa_n2)).
action(cpa_travel(cpa_n1, cpa_n1)).
action(cpa_travel(cpa_n1, cpa_n2)).
action(cpa_travel(cpa_n2, cpa_n1)).
action(cpa_travel(cpa_n2, cpa_n2)).

%% executable ------ 

executable(cpa_start(cpa_n1),true).
executable(cpa_start(cpa_n2),true).
executable(cpa_travel(cpa_n1, cpa_n1),cpa_started).
executable(cpa_travel(cpa_n1, cpa_n2),cpa_started).
executable(cpa_travel(cpa_n2, cpa_n1),cpa_started).
executable(cpa_travel(cpa_n2, cpa_n2),cpa_started).

%% effects ------ 

effect(cpa_start(cpa_n1),cpa_started,1).
effect(cpa_start(cpa_n1),cpa_visited(cpa_n1),1).
condition(cpa_start(cpa_n1),cpa_at(cpa_n1),1).
effect(cpa_start(cpa_n2),cpa_started,1).
effect(cpa_start(cpa_n2),cpa_visited(cpa_n2),1).
condition(cpa_start(cpa_n2),cpa_at(cpa_n2),1).
effect(cpa_travel(cpa_n1, cpa_n2),cpa_visited(cpa_n2),1).
effect(cpa_travel(cpa_n1, cpa_n2),cpa_at(cpa_n2),1).
effect(cpa_travel(cpa_n1, cpa_n2),neg(cpa_at(cpa_n1)),1).
condition(cpa_travel(cpa_n1, cpa_n2),cpa_at(cpa_n1),1).
effect(cpa_travel(cpa_n2, cpa_n1),cpa_visited(cpa_n1),1).
effect(cpa_travel(cpa_n2, cpa_n1),cpa_at(cpa_n1),1).
effect(cpa_travel(cpa_n2, cpa_n1),neg(cpa_at(cpa_n2)),1).
condition(cpa_travel(cpa_n2, cpa_n1),cpa_at(cpa_n2),1).

%% initial state ------ 

initially(neg(cpa_visited(cpa_n1))).
initially(neg(cpa_visited(cpa_n2))).
initially(neg(cpa_started)).
initially_oneof(1,cpa_at(cpa_n1),1).
initially_oneof(1,cpa_at(cpa_n2),2).

%% goal state ---------- 

goal(cpa_visited(cpa_n1)).
goal(cpa_visited(cpa_n2)).

