#script(python)
def main(prg):
    n = 0
    for lower, upper in prg.getConst("calls").args():
        while n < upper:
            n += 1
            prg.ground("board", [n])
            if n >= lower:
                print 'SIZE {0}'.format(n)
                prg.solve()
#end.
