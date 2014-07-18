#script (python)

import gringo

def main(prg):
    prg.ground("step1", [])
    prg.assignExternal(gringo.Fun("f"), True)
    prg.setConf("-n 1")
    prg.solve()
    prg.ground("step2", [])
    prg.assignExternal(gringo.Fun("f"), False)
    prg.setConf("-n 0")
    prg.solve()

#end.

#program step1.

#external f.
{ a; b; c }.

#show f/0.

#program step2.

#show a : a.
#show b : b.
#show c : c.

