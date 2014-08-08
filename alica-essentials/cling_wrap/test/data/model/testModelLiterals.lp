#program base.

sumRow(X, SUM) :- SUM = #count{ Y : player(X,Y) }, row(X).
sumColumn(X, SUM) :- SUM = #count{ Y : player(Y,X) }, column(X). 

sumPlayer(SUM) :- SUM = #count{ X, Y : player(X,Y), row(X), column(Y) }.


#program inc(i).
row(i).
column(i).

#program player(j,k).

#external player(j,k).

%#show row/1.