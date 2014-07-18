#script(lua)
function main(prg)
    n = 0
    for _, rng in pairs(prg:getConst("calls"):args()) do
        lower = rng:args()[1]
        upper = rng:args()[2]
        while n < upper do
            n = n + 1
            prg:ground("board", {n})
            if n >= lower then
                io.write(string.format('SIZE %d\n', n))
                prg:solve()
            end
        end
    end
end
#end.
