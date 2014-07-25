#script (python)

import gringo

def onModel(x):
    print("Answer: 42")
    print(
          "hasA(" +        str(x.contains(gringo.Fun("a"))).lower() + ") "
        + "hasVolatile(" + str(x.contains(gringo.Fun("volatile", [9]))).lower() + ") "
        + "model(" +       ",".join(sorted(map(str, x.atoms()))) + ") ")

def onFinish(ret, interrupted):
    print("Answer: 42")
    print(str(ret) + "_" + str(interrupted).lower())

def main (prg):
    n = prg.getConst("n")
    prg.ground("base", [])
    prg.solve()
    prg.add("test", ["x"], "test(x).")
    prg.ground("test", [gringo.Fun("f", [1,2])])
    for i in range(1,n+1):
        prg.ground("cumulative", [i])
        f = prg.asolve(onModel, onFinish)
        f.wait(0)
        ret = f.get()
        print("Answer: 42")
        print(ret)

#end.

#const n = 10.

#program base.

a.

#program cumulative(k).

1 { c(k) } 1.
q(k) :- c(k).
r(k,X) :- test(X).

#external volatile(k).
 :- not c(5), not volatile(k).
volatile(k-1).

