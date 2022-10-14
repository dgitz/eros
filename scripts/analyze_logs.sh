#!/bin/bash
source $(dirname "$0")/util/bag2csv.sh
function print_usage()
{
    echo "Usage Instructions"
    echo -e "<mode>:
        performance <BAG_DIRECTORY> <OUTPUT_DIRECTORY>: Run performance scans on a folder of bag files"
    exit 1
}
function performance()
{
    bag_directory=$1
    output_directory=$2

    #Clear Output Directory

    #Loop Through BAG Files
    for filePath in "$bag_directory"*
    do
        echo "loop"
        filename=$(basename -- "$filePath")
        extension="${filename##*.}"
        filename="${filename%.*}"

        if [ "$extension" == "bag" ]; then
            echo "Working on: "$filePath
            topicInfo=$(rosbag info -y -k topics $filePath)
            echo "NEXT: FILTER OUT TOPIC NAME/TYPES FROM THIS LIST"
            exit 1
            for info in $topicInfo
            do
                echo "next"
                echo $info
            done
            # Loop through topics of interest
           
        fi
    done

}

if [ $# -eq 0 ]; then
    print_usage
else
    case $1 in
        "performance") performance $2 $3;
    esac
fi
exit 0