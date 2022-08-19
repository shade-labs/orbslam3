#!/bin/bash

declare -a ROS_DISTROS=( "foxy" "galactic" "humble" "rolling" )


for VERSION in "${ROS_DISTROS[@]}"
do
  ROS_DISTRO="$VERSION"
  gcloud builds submit --config cloudbuild.yaml . --substitutions=_ROS_DISTRO="$ROS_DISTRO" --timeout=86399 &
  pids+=($!)
  echo Dispached ROS "$ROS_DISTRO"
done

for pid in ${pids[*]}; do
  wait "$pid"
done

echo "All builds finished"
