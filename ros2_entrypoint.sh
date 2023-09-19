#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/aerosandbox_cessna/install/setup.bash 

exec "$@"
