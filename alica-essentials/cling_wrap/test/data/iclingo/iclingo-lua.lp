#script (lua) 

function map(func, array)
    local new_array = {}
    for i,v in ipairs(array) do
        new_array[i] = func(v)
    end
    return new_array
end

function onModel(m)
    print("Model: {"..table.concat(map(tostring, m:atoms()), ",").."}")
end

function get(val, default)
    if val ~= nil then 
        return val 
    else 
        return default 
    end
end

function main(prg)
    imin   = get(prg:getConst("imin"), 1)
    imax   = prg:getConst("imax")
    istop  = get(prg:getConst("istop"), "SAT")
    iquery = get(prg:getConst("iquery"), 1)
    
    step = 1
    
    prg:ground("base", {})
    while true do
        if imax ~= nil and step > imax then break end
        prg:ground("cumulative", {step})
        if step >= iquery then
            if step > iquery then
                prg:releaseExternal(gringo.Fun("query", {step-1}))
            end
            prg:assignExternal(gringo.Fun("query", {step}), true)
            prg:ground("volatile", {step})
        end
        ret = prg:solve(onModel)
        if     ret == gringo.SolveResult.SAT     then print "SAT"
        elseif ret == gringo.SolveResult.UNSAT   then print "UNSAT"
        elseif ret == gringo.SolveResult.UNKNOWN then print "UNKNOWN"
        end
        if step >= imin and ((istop == "SAT" and ret == gringo.SolveResult.SAT) or (istop == "UNSAT" and ret ~= gringo.SolveResult.SAT)) then
            break
        end
        step = step+1
    end
end
#end.
