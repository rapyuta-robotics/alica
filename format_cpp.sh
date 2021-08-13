#!/bin/sh
# Brief: This is a script to format C/C++ files.
# Usage:
#   ./path/to/format_cpp.sh in the folder that needs to be formatted

find . \
    -not \( -path "*/devel/*" -prune \) \
    -not \( -path "*/build/*" -prune \) \
    -not \( -path "*/install/*" -prune \) \
    \( -name *.h -o -name *.hpp -o -name *.c -o -name *.cc -o -name *.cpp \) \
    | xargs clang-format-6.0 -style=file -i
