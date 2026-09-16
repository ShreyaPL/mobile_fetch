# Source this file: source ~/projects/mobile_fetch/scripts/activate.bash
# Exclude unrelated packages installed in ~/.local before starting Python.
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/../mobile-fetch/bin/activate"
