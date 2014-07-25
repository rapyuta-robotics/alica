#script (lua) 

function main(prg)
    prg:ground("pigeon", {})
    for i=1,2 do
        prg:assignExternal(gringo.Fun("p"), true)
        prg:solve()
        prg:assignExternal(gringo.Fun("p"), false)
        f = prg:asolve()
        f:wait(0.01)
        f:interrupt()
    end
end

#end.

#const n = 190.

#program pigeon.
#external p.

1 { p(X); q(X) } 1 :- X = 1..n, not p.
:- not n+1 { p(1..n); q(1..n) }, not p.
