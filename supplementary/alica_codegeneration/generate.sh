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

if [ ! -z $2 ]
then
    PKG_NAME="$2"
else
    OLDIFS=$IFS
    IFS='/'
    read -a pkg_path <<< ${ALICA_APP_PATH}
    PKG_NAME=${pkg_path[-1]}
    echo "Extracting package_name \"${PKG_NAME}\". If incorrect, set it directly as an argument. Ex: ./generate.sh \"/home/rr/app/\" pkg_name"
    IFS=$OLDIFS
fi

java -jar ${PWD}/codegen/PlanDesignerFX-Codegeneration-0.1.1.*.jar ${CLANG_FORMATTER} ${ALICA_APP_PATH}/Expr/ ${ALICA_APP_PATH}/etc/plans/ ${ALICA_APP_PATH}/etc/roles/ ${ALICA_APP_PATH}/etc/tasks/ ${PWD}/codegen/plugins/ ${PKG_NAME}
