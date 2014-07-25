#program step(k).

#external vol(k).
p(k).
:- vol(k), not 5 { p(K) : K <= k }.

#script (python)

from gringo import *

def main(prg):
    step = 0
    while True:
        if step > 0: prg.releaseExternal(Fun("vol", [step-1]))
        prg.ground("step", [step])
        prg.assignExternal(Fun("vol", [step]), True)
        ret = prg.solve()
        if ret == SolveResult.SAT:
            break
        step += 1

#end.
