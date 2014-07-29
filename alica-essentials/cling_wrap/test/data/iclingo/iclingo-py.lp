#script (python) 

from gringo import *

def onModel(model):
    print "Model: ", model.atoms()

def get(val, default):
    return val if val != None else default

def main(prg):
    imin   = get(prg.getConst("imin"), 1)
    imax   = prg.getConst("imax")
    istop  = get(prg.getConst("istop"), "SAT")
    iquery = get(prg.getConst("iquery"), 1)
    
    step = 1
    
    prg.ground("base", [])
    while True:
        if imax != None and step > imax: break
        prg.ground("cumulative", [step])
        if step >= iquery: 
            if step > iquery: prg.releaseExternal(Fun("query", [step-1]))
            prg.assignExternal(Fun("query", [step]), True)
            prg.ground("volatile", [step])
            
        ret = prg.solve(onModel)
        if step >= imin and ((istop == "SAT" and ret == SolveResult.SAT) or (istop == "UNSAT" and ret != SolveResult.SAT)): break
        step = step+1
#end.
