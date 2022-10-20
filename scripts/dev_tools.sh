#!/bin/bash
LINE_COVERAGE_THRESHOLD=60
BRANCH_COVERAGE_THRESHOLD=0 # Don't care about Branch Coverage.
bin_dir="../../build"

function print_usage()
{
    echo "Usage Instructions"
    echo -e "<mode>:
        update: Update,
        code_coverage: Run Code Coverage Scan,
        plantuml: Generate plantuml Images
        regression: Run Regression Tests"
    exit 1
}
function update {
    python scripts/create_from_template.py -t auto_template/class/ -o sample/SampleClass/ -i 0
}
# Code Coverage Scan
function code_coverage_scan {
    coverage_dir="coverage"

    if [ -d "$coverage_dir" ]; then
    rm -r -f $coverage_dir
    fi
    mkdir $coverage_dir
    gcov_cmd="gcovr $bin_dir/eros -x $coverage_dir/coverage.xml --html-details -o $coverage_dir/coverage.html  --exclude ^src.*/test_[^/]*.cpp  --exclude ^nodes.*/test_[^/]*.cpp --exclude devel --fail-under-line $LINE_COVERAGE_THRESHOLD --fail-under-branch $BRANCH_COVERAGE_THRESHOLD --exclude-throw-branches --exclude-unreachable-branches"
    eval "$gcov_cmd"
    status=$?
    if [ "$status" -eq 0 ]; then
        echo "Coverage Scan OK!"
    else
        if [ "$status" -eq 2 ]; then
            echo "FAILED: Line Coverage!"
            exit 1
        elif [ "$status" -eq 4 ]; then
            echo "FAILED: Branch Coverage!"
            exit 1
        else
            echo "FAILED: Were Unit Tests Executed?"
            exit 1
        fi
    fi
}
function generate_plantuml {

    plantuml -tsvg -r -o output "doc*/**.puml"
    status=$?
    exit $status
}
function run_regression {
    ITERATION_COUNT=10
 
    status=0
    pushd ../../
    for (( c=1; c<=$ITERATION_COUNT; c++ ))
    do
        echo "Running Unit Test Iteration: "$c"/"$ITERATION_COUNT
        rm -r -f build/ devel/
        catkin_make
        status=$?
        if [ "$status" -ne 0 ]; then
            echo "Regression Test Failed at Iteration: "$c
            popd
            exit $status
        fi
        catkin_make tests
        status=$?
        if [ "$status" -ne 0 ]; then
            echo "Regression Test Failed at Iteration: "$c
            popd
            exit $status
        fi
        catkin_make run_tests
        status=$?
        if [ "$status" -ne 0 ]; then
            echo "Regression Test Failed at Iteration: "$c
            popd
            exit $status
        fi
        catkin_test_results build/test_results/
        status=$?
        if [ "$status" -ne 0 ]; then
            echo "Regression Test Failed at Iteration: "$c
            popd
            exit $status
        fi
    done
    popd

    if [ "$status" -eq 0 ]; then
        echo "Regression Tests Count: "$ITERATION_COUNT" Successfully Completed!"
    fi
    return $status
}
if [ $# -eq 0 ]; then
    print_usage
else
    case $1 in
        "update") update;;
        "code_coverage") code_coverage_scan;;
        "plantuml") generate_plantuml;;
        "regression") run_regression;;
    esac
fi
exit 0