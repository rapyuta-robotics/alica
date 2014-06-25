#!/bin/bash
ARCH=`uname -m`
if [ $ARCH == "i686" ]; then
       ARCH="x86"
fi
if [ $ARCH == "i386" ]; then
       ARCH="x86"
fi
#CPATH=`rospack find Planmodeller`"/bin/linux.gtk.${ARCH}/PlanDesigner"
CPATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )""/linux.gtk.${ARCH}/"
cd ${CPATH}
./PlanDesigner

