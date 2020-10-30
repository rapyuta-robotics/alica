#!/bin/bash

# absolut path to this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Variables that need to be overwritten in the Doxyfile
PROJECT_NAME=$1
OUTPUT_DIRECTORY=/tmp/${PROJECT_NAME}
INPUT=${DIR}/../../alica_engine/include/engine/AlicaContext.h
#INPUT=${DIR}/$2

echo "PROJECT_NAME=${PROJECT_NAME} OUTPUT_DIRECTORY=${OUTPUT_DIRECTORY} INPUT=${INPUT}"

# Call doxygen
(cat Doxyfile.in ; echo "INPUT=${INPUT}" ; echo "PROJECT_NAME=${PROJECT_NAME}" ; echo "OUTPUT_DIRECTORY=${OUTPUT_DIRECTORY}") | doxygen -
