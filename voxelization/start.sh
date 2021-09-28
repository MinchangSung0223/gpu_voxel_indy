#!/bin/sh
##### Auto search and make Voxel Data####
searchdir=$1

for entry in $searchdir/*

do

        echo "$entry"

        filename="$entry"

        bash voxelize.sh $filename 100

done

