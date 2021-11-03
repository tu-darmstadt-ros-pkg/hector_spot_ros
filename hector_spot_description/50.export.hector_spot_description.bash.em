#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  add_rosrs_setup_env HECTOR_SIM_USE_GPU_LIDAR "true,false" "Use a GPU accelerated lidar simulation" 
fi
