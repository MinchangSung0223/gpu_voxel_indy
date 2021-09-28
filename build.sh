#!/bin/bash

rm -r CMakeCache.txt ||
rm -r build &&
mkdir build &&

cp include/TemplateVoxelMap.hpp /root/workspace/gpu-voxels/packages/gpu_voxels/src/gpu_voxels/voxelmap/ &&
cd build && cmake .. -D icl_core_DIR=~/workspace/gpu-voxels/build/packages/icl_core/ -D gpu_voxels_DIR=~/workspace/gpu-voxels/build/packages/gpu_voxels
make -j16
cp gvl_ompl_planner ../
cp indyTest ../

export GPU_VOXELS_MODEL_PATH=models
