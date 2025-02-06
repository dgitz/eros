#!/bin/bash
FILES=$(find include/ src/ nodes/ -name "*.h" -o -name "*.c" -o -name "*.cpp")
function print_usage()
{
    echo "Usage Instructions"
    echo -e "<mode>:
        dry-run: Dry-Run"
    exit 1
}

function format_dryrun {
    # Run clang-format on the files and check for errors
    if clang-format --dry-run -Werror $FILES; then
        echo "No formatting errors found."
        return 0
    else
        echo "Formatting errors found. Please run clang-format to fix them."
        return 1
    fi
}

if [ $# -eq 0 ]; then
    print_usage
else
    case $1 in
        "dry-run") format_dryrun;;
    esac
fi