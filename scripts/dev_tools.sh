#!/bin/bash
LINE_COVERAGE_THRESHOLD=10
BRANCH_COVERAGE_THRESHOLD=0 # Don't care about Branch Coverage.
bin_dir="../../build"

# Code Coverage Scan
function code_coverage_scan()
{
    coverage_dir="coverage"

    if [ -d "$coverage_dir" ]; then
    rm -r -f $coverage_dir
    fi
    mkdir $coverage_dir
    gcov_cmd="gcovr $bin_dir/eros -x $coverage_dir/coverage.xml --html-details -o $coverage_dir/coverage.html  --exclude ^src.*/test_[^/]*.cpp  --exclude ^nodes.*/test_[^/]*.cpp --exclude devel --fail-under-line $LINE_COVERAGE_THRESHOLD --fail-under-branch $BRANCH_COVERAGE_THRESHOLD --exclude-throw-branches --exclude-unreachable-branches"
    eval "$gcov_cmd"
    status=$?
    return $status
}

code_coverage_scan
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
exit 0