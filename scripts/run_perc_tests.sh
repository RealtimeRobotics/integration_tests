#!/bin/sh

usage() {
    cat <<EOF
Usage: $0 [OPTION]...
Run perception integration and regtests.

Options:
    -h    print usage and exit
    -d    disable time sensitives tests

EOF
}

################################################################################

disable_flakes=0

while getopts ":hd:" opt; do
    case "${opt}" in
        h)
            usage
            exit 0
            ;;
        d)
            disable_flakes=1
            ;;
        :) die "missing argument: -${OPTARG}" ;;
        \?) die "bad option: -${OPTARG}" ;;
    esac
done
shift "$((${OPTIND} - 1))"

readonly packages="$(catkin list | grep 'unit_test\|integ_test\|reg_test' | sed 's/- / /g' | tr -d '\n')"
echo "Running tests from the following packages: ${packages}"

if [ "${disable_flakes}" -eq 1 ]; then
    env DISABLE_RS_FLAKYTESTS=1 catkin run_tests --no-deps ${packages}
else
    env -u DISABLE_RS_FLAKYTESTS catkin run_tests --no-deps ${packages}
fi

echo "\n\nTEST RESULTS\n------------------------------------------------------------------"
readonly build_dir="$(catkin locate)/build/"
readonly results_dirs="${build_dir}$(echo ${packages} | sed "s| | ${build_dir}|g")"
for result_dir in ${results_dirs}
do
    catkin_test_results --all --verbose ${result_dir}
    echo ""
done

exit 1
