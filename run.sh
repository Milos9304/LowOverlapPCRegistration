#!/bin/bash

#e.g. A1 high B1

INPUT_FOLDER='./8thFloor'

#for datasetName in ${INPUT_FOLDER}/*; do
#    [ -e "$datasetName" ] || continue
#    for 
#    echo $datasetName
#done

if [ -z "$1" ]
then
    echo "No input arguments"
    exit 1
fi

if [ -z "$2" ]
then #CORRUPTED
    ./bin/mapMerge ${INPUT_FOLDER}/${1}/pointcloud.ply output_data/edges/${1}_orig.ply output_data/lines/${1}_orig.data output_data/edges/${1}_orig_filtered.ply output_data/edges/${1}_orig_filtered2.ply mergedOutput.ply 
    python3 scripts/drawLines.py output_data/lines/${1}_orig.data output_data/lines
else
    ./bin/mapMerge ${INPUT_FOLDER}/${1}/pointcloud_post_${2}.ply ${INPUT_FOLDER}/${3}/pointcloud_post_${2}.ply output_data/edges/${1}_${2}.ply output_data/lines/${1}_${2}.data output_data/edges/${1}_${2}_filtered.ply output_data/edges/${1}_${2}_filtered2.ply mergedOutput.ply
    python3 scripts/drawLines.py output_data/lines/${1}_${2}.data output_data/lines
fi
