#!/bin/sh

################################################################################
#disable_flakes=0
#while getopts ":hd:" opt; do
#    case "${opt}" in
#        h)
#            usage
#            exit 0
#            ;;
#        d)
#            disable_flakes=1
#            ;;
#        :) die "missing argument: -${OPTARG}" ;;
#        \?) die "bad option: -${OPTARG}" ;;
#    esac
#done
#shift "$((${OPTIND} - 1))"

get_test_packages() {
    local packages1="$(catkin list | grep 'unit_test\|integ_test\|reg_test' | sed 's/- / /g' | tr -d '\n')"
    local  __resultvar=$1
    eval $__resultvar="'$packages1'"
}

################################################################################
get_build_dir() {
    local  __resultvar=$1
    local build_dir1="$(catkin locate -b)"
    eval $__resultvar="'$build_dir1/'"
}

# pass 0 to disable_flakes and 1 to run them
run_perc_tests() {
    local disable_flakes=$1
    get_test_packages packages
    echo "Running tests from the following packages: ${packages}"
    if [ "${disable_flakes}" -eq 1 ]; then
        env DISABLE_RS_FLAKYTESTS=1 catkin run_tests --no-deps ${packages}
    else
        env -u DISABLE_RS_FLAKYTESTS catkin run_tests --no-deps ${packages}
    fi
}

show_perc_results() {
    get_test_packages packages
    get_build_dir build_dir
    echo "showing tests from the following packages: ${packages} and ${build_dir}"
    echo "\n\nTEST RESULTS\n------------------------------------------------------------------"
    local results_dirs="${build_dir}$(echo ${packages} | sed "s| | ${build_dir}|g")"
    for result_dir in ${results_dirs}
    do
        echo $result_dir
        catkin_test_results --all --verbose ${result_dir}
        echo ""
    done
}

show_perc_results_summary() {
    get_test_packages packages
    get_build_dir build_dir
    local results_dirs="${build_dir}$(echo ${packages} | sed "s| | ${build_dir}|g")"
    echo "TEST RESULT SUMMARY"
    echo "-------------------"
    for result_dir in ${results_dirs}
    do
        echo $result_dir
        catkin_test_results --all ${result_dir}
        echo ""
    done
}

