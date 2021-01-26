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
    local disabled_tests=(reg_test_calibration)
    local packages1="$(catkin list | grep 'unit_test\|integ_test\|reg_test' | sed 's/- / /g' | tr -d '\n')"
    for test_pkg in ${disabled_tests}
    do
        local packages1="$(echo $packages1 | sed "s/ $test_pkg / /g")"
    done
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

    if [ -z "$RTR_PERCEPTION_TEST_DATA_ROOT" ]
    then
      echo "Missing rapidsense_testdata directory! Please set RTR_PERCEPTION_TEST_DATA_ROOT"
      return
    fi

    local disable_flakes=$1
    get_test_packages packages
    echo "Running tests from the following packages: ${packages}"
    if [ "${disable_flakes}" -eq 1 ]; then
        # run tests one at a time, because currently not dealing with state directories properly for parallel ops
        # TODO: fix this so we can run in parallel
        env DISABLE_RS_FLAKYTESTS=1 catkin run_tests --no-deps ${packages} -j1
    else
        # run the flaky tests one at a time
        env -u DISABLE_RS_FLAKYTESTS catkin run_tests --no-deps ${packages} -j1
    fi
}

show_perc_results() {
    get_test_packages packages
    get_build_dir build_dir
    echo "Showing tests from the following packages: ${packages} and ${build_dir}"
    echo "\n\nTEST RESULTS\n------------------------------------------------------------------"

    # catkin_test_results requires install to be sourced
    local root_dir=$(catkin locate)
    . ${root_dir}/install/setup.sh

    for result_dir in ${packages}
    do
        echo $result_dir
        catkin_test_results --all --verbose ${build_dir}${result_dir}
        echo ""
    done
}

show_perc_results_summary() {
    get_test_packages packages
    get_build_dir build_dir

    # catkin_test_results requires install to be sourced
    local root_dir=$(catkin locate)
    . ${root_dir}/install/setup.sh

    echo "TEST RESULT SUMMARY"
    echo "-------------------"
    for result_dir in ${packages}
    do
        echo $result_dir
        catkin_test_results --all ${build_dir}${result_dir}
        echo ""
    done
}

