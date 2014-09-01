% information/2	:		(entity,scope)
% stream/2		: 		(system,source,information)
% node/4		:		(system,nodeName,entity)
% output/2 		:		(nodeName,scope)
%--------------------------------------------------------------------------------------
% paramters
 #const priorityDelay=2.
 #const priorityAccuracy=2.
 #const priorityCost=1.
 #const priorityTransferCost=1.
 

%--------------------------------------------------------------------------------------
#program base.
%#show streamTransfer/7.
#show node/4.
%#show streamDelay/5.
%#show nodeTemplate/2.
%#show stream/4.
%#show transfer/4.
#show requiredStream/4.
#show system/2.
%--------------------------------------------------------------------------------------
#program query(k).

#external query(k).

% preprocessing
{ stream(SYSTEM,SOURCE,INFO,k) } :- inputStream(SYSTEM,SOURCE,INFO,_,_,_), query(k).
streamDelay(SYSTEM,SOURCE,INFO,k,DELAY) :- inputStream(SYSTEM,SOURCE,INFO,DELAY,_,_), query(k).
streamAccuracy(SYSTEM,SOURCE,INFO,k,ACCURACY) :- inputStream(SYSTEM,SOURCE,INFO,_,ACCURACY,_), query(k).
streamCost(SYSTEM,SOURCE,INFO,k,COST) :- inputStream(SYSTEM,SOURCE,INFO,_,_,COST), query(k).


% guessing
0 { node(SYSTEM,NODE,ENTITY,k) } 1 :- nodeTemplate(SYSTEM,NODE), entity(ENTITY), query(k).
MIN { connect(SYSTEM,NODE,SOURCE,information(ENTITY,SCOPE),k) : stream(SYSTEM,SOURCE,information(ENTITY,SCOPE),k) } MAX :- 
		input(SYSTEM,NODE,SCOPE,MIN,MAX), node(SYSTEM,NODE,ENTITY,k).
		
0 { streamTransfer(SYSTEM_SOURCE,SYSTEM_TARGET,SOURCE,INFO,k,TRANSFER_DELAY,TRANSFER_COST) : stream(SYSTEM_SOURCE,SOURCE,INFO,k), SYSTEM_TARGET!=SOURCE } 1
		:- transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST).
		
stream(SYSTEM_TARGET,SOURCE,INFO,k) :- streamTransfer(SYSTEM_SOURCE,SYSTEM_TARGET,SOURCE,INFO,k,TRANSFER_DELAY,TRANSFER_COST).
streamDelay(SYSTEM_TARGET,SOURCE,INFO,k,TRANSFER_DELAY+ DELAY) :- streamTransfer(SYSTEM_SOURCE,SYSTEM_TARGET,SOURCE,INFO,k,TRANSFER_DELAY,TRANSFER_COST), 
		streamDelay(SYSTEM_SOURCE,SOURCE,INFO,k,DELAY).
streamCost(SYSTEM_NAME_TARGET,SOURCE,INFO,k,TRANSFER_COST+COST) :- streamTransfer(SYSTEM_NAME_SOURCE,SYSTEM_NAME_TARGET,SOURCE,INFO,k,TRANSFER_DELAY,TRANSFER_COST),
		streamCost(SYSTEM_NAME_SOURCE,SOURCE,INFO,k,COST).

stream(SYSTEM,SYSTEM,information(ENTITY,SCOPE),k) :- node(SYSTEM,NODE,ENTITY,k), output(SYSTEM,NODE,SCOPE).

streamDelay(SYSTEM,SYSTEM,information(ENTITY,SCOPE),k,ND+SD) :- node(SYSTEM,NODE,ENTITY,k), output(SYSTEM,NODE,SCOPE), nodeDelay(SYSTEM,NODE,ND),
		SD = #max{D : connect(SYSTEM,NODE,SOURCE,INFO,k), streamDelay(SYSTEM,SOURCE,INFO,k,D)}.
streamCost(SYSTEM,SYSTEM,information(ENTITY,SCOPE),k,0) :- node(SYSTEM,NODE,ENTITY,k), output(SYSTEM,NODE,SCOPE).

% checking
% soft optimization contraint instead of a hard constraint in order so enable partial solitions
% funktioniert so nicht...
:~ requiredStream(SYSTEM,INFO,REQUIRED_DELAY,REQUIRED_ACCURACY), REQUIRED_DELAY > -1, REQUIRED_ACCURACY > -1, stream(SYSTEM,SOURCE,INFO,k), 
		streamDelay(SYSTEM,SOURCE,INFO,k,DELAY), streamAccuracy(SYSTEM,SOURCE,INFO,k,ACCURACY), R = REQUIRED_ACCURACY - ACCURACY, D = REQUIRED_DELAY - DELAY,
		R * D > 0, R + D > 0, query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO,REQUIRED_DELAY,REQUIRED_ACCURACY), REQUIRED_DELAY > -1, REQUIRED_ACCURACY < 0, stream(SYSTEM,SOURCE,INFO,k), 
		streamDelay(SYSTEM,SOURCE,INFO,k,DELAY), REQUIRED_DELAY < DELAY,query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO,REQUIRED_DELAY,REQUIRED_ACCURACY), REQUIRED_DELAY < 0, REQUIRED_ACCURACY > -1, stream(SYSTEM,SOURCE,INFO,k), 
		streamAccuracy(SYSTEM,SOURCE,INFO,k,ACCURACY), REQUIRED_ACCURACY < ACCURACY, query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO,DELAY,ACCURACY), not stream(SYSTEM,_,INFO,k), query(k). [1@5,requiredStream]
% checking cost points of the systems
:- system(SYSTEM,POINTS), SUM_STREAMS = #sum{COST : streamTransfer(SYSTEM,_,_,_,k,_,COST)}, SUM_NODES = #sum{COST : node(SYSTEM,NODE,ENTITY,k), nodeCost(SYSTEM,NODE,COST)}, 
		(SUM_NODES + SUM_STREAMS) > POINTS.

% optimize
#minimize{ TRANSFER_COST@priorityTransferCost : streamTransfer(_,_,_,_,k,_,TRANSFER_COST) }.
#minimize { DELAY@priorityDelay : requiredStream(SYSTEM,INFO,DELAY,ACCURACY), streamDelay(SYSTEM,_,INFO,k,DELAY) }.
%#maximize accuracy
#minimize { COST@priorityCost : requiredStream(SYSTEM,INFO,DELAY,ACCURACY), streamCost(SYSTEM,_,INFO,k,COST) }.


%--------------------------------------------------------------------------------------
#program entity(entity).

entity(entity).


%--------------------------------------------------------------------------------------
#program inputStream(system,source,information,delay,accuracy,cost).

#external inputStream(system,source,information,delay,accuracy,cost).


%--------------------------------------------------------------------------------------
#program requiredStream(system,information,delay,accuracy).

#external requiredStream(system,information,delay,accuracy).


%--------------------------------------------------------------------------------------
#program system(name,points).

#external system(name,points).


%--------------------------------------------------------------------------------------
#program transfer(system1,system2,delay,cost).

#external transfer(system1,system2,delay,cost).
transfer(system2,system1,delay,cost) :- transfer(system1,system2,delay,cost).


