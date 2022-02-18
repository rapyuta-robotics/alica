import sys
import os
from catkin_pkg.topological_order import topological_order

if len(sys.argv) != 2:
    raise RuntimeError('invalid call - missing workspace root')
if not os.path.isdir(sys.argv[1]):
    raise RuntimeError('invalid call - not a directory')

f = open("coverity-analysis.sh","w+")

prefix = 'cd ~/catkin_ws/build/'
suffix = ' && make clean && make'

f.write(prefix)
f.write((suffix + ' && ' + prefix).join(e[1].name for e in topological_order(sys.argv[1])))
f.write(suffix)

f.close()