#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail
IFS=$'\n\t'

# Don't format in third-party
readonly WHITELIST_REGEX='^src/third-party/'
# Matches *.cpp, *.hpp, *.c, *.h, and *.cu
readonly CPP_FILENAME_REGEX='.*\.[ch]((pp)|u)?$'

readonly SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
readonly GIT_ROOT="$(cd "${SCRIPT_DIR}" && git rev-parse --show-toplevel)"

function show_usage {
    cat <<EOF 1>&2
Usage: format-code.sh [-a | --all] [-d | --diff] [-h | --help]

Formats RTR source code, C (*.h, *.c), C++ (*.hpp, *.cpp), and CUDA (*.cu)

    -a | --all      Formats all sources (default)
    -d | --diff     Formats only stages sources, added and modified
    -h | --help     Shows this help message

Always remember to format your source code! ;-)

EOF
}

diff_mode=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        -a | --all )
            diff_mode=0
            ;;
        -d | --diff )
            diff_mode=1
            ;;
        -h | --help )
            show_usage
            exit 0
            ;;
        * )
            show_usage
            exit 1
            ;;
    esac
    shift
done

readonly emacs_lockfiles=$(find "${GIT_ROOT}" -regex '.*\.#.+\.[ch]pp$')
if [ -n "${emacs_lockfiles}" ]; then
    echo "Emacs Lockfiles detected! Save your files!" 1>&2
    exit 1
fi

if [ "${diff_mode}" -ne 0 ]; then
    (cd "${GIT_ROOT}"
     format_files="$(git diff --diff-filter=d --name-only HEAD \
                         | grep -E -e "${CPP_FILENAME_REGEX}" \
                         | grep -E -v -e "${WHITELIST_REGEX}")"
     if [ -n "$format_files" ]; then
         echo "$format_files" | xargs -P 4 clang-format-9 -i
     fi)
else
    (cd "${GIT_ROOT}"
     format_files="$(find "." -regextype egrep -regex "${CPP_FILENAME_REGEX}" \
                         | grep -E -v -e "${WHITELIST_REGEX}")"
     if [ -n "$format_files" ]; then
         echo "$format_files" | xargs -P 4 clang-format-9 -i
     fi)
fi

exit 0
