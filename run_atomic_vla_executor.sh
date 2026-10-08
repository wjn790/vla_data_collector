#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/svt/svtrobo_ws/install/setup.bash

cd /home/svt/lingbot_data_collector
exec /home/svt/miniconda3/bin/python \
  -m supervisor.atomic_vla_executor \
  "$@"
