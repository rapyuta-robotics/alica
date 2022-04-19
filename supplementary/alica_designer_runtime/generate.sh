#!/bin/bash
echo ${PWD}

ALICA_APP_PATH=${ALICA_APP_PATH:-""}
CLANG_FORMATTER="/usr/bin/clang-format-10"

if [ ! -z $1 ]
then
    ALICA_APP_PATH="$1"
fi

if [ -z $ALICA_APP_PATH ]
then
   echo "Specify absolute path to the folder containing etc and Expr folders. Ex: ./generate.sh \"/home/rr/app/\""
   exit 1
fi

java -jar ${PWD}/codegen/PlanDesignerFX-Codegeneration-0.1.1.*.jar ${CLANG_FORMATTER} ${ALICA_APP_PATH}/Expr/ ${ALICA_APP_PATH}/etc/plans/ ${ALICA_APP_PATH}/etc/roles/ ${ALICA_APP_PATH}/etc/tasks/ ${PWD}/codegen/plugins/
