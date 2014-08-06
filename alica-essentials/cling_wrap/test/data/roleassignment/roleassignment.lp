robot(r1;r2;r3).
role(ro1;ro2;ro3).
cap(grasp;speak).

hasCapability(r1,grasp,3).
hasCapability(r2,speak,2).

%-hasCapability(r1,speak).

requiresCapability(ro1,grasp,2).
requiresCapability(ro2,speak,3).

% guess

1 { hasRole(ROBOT,ROLE) : role(ROLE) } 1 :- robot(ROBOT).
	
% check
	
:- hasRole(ROBOT,ROLE), requiresCapability(ROLE,CAP,_), -hasCapability(ROBOT,CAP).
:- hasCapability(ROBOT,CAP,_), -hasCapability(ROBOT,CAP).
:- {hasRole(X,ro2) : robot(X)} 2.

% optimization

%hasCapability(ROBOT,CAP,0) :- not hasCapability(ROBOT, CAP, _), not -hasCapability(ROBOT, CAP), robot(ROBOT), cap(CAP).

matching(ROBOT, ROLE, CAPABILITY, SKILL - REQ_SKILL) :- hasRole(ROBOT,ROLE), 
	hasCapability(ROBOT,CAPABILITY,SKILL), requiresCapability(ROLE,CAPABILITY,REQ_SKILL).
matching(ROBOT, ROLE, CAPABILITY, -REQ_SKILL) :- hasRole(ROBOT,ROLE), 
	not hasCapability(ROBOT,CAPABILITY,_), requiresCapability(ROLE,CAPABILITY,REQ_SKILL). 

sum(S) :- S = #sum { V : matching(_,_,_,V)}.

#maximize { S : sum(S) }.
%#maximize { U, ROBOT, ROLE, CAP : matching(ROBOT,ROLE,CAP,U) }.

%#show sum/1.
%#show hasRole/2.
%#show matching/4.

#program hasCapability(robot,cap).

#external hasCapability(robot,cap).