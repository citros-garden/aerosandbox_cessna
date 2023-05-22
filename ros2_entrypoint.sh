#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/Spiceypy/install/setup.bash 

exec "$@"
