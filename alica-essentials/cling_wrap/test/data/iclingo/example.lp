#const istop  = "SAT".
#const imin   = 1.
%#const imax   = 11.
#const iquery = 1.

#program base.

{a;b;c}.

#program cumulative(k).

{ c(k) }.
q(k) :- c(k).

#program volatile(k).

#external query(k).
 :- not c(5), query(k).
