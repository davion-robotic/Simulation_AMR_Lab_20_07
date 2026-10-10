#!/usr/bin/env python3
"""Centralized system constants for AMR Robot."""

from pathlib import Path

# Path to directory where map .yaml and .pgm files are stored
MAPS_DIR = Path(
    '/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/'
    'src/mybot_navigation2/map'
)

MAPS_DIR_STR = str(MAPS_DIR)

DEFAULT_SAVE_MAP_TIMEOUT = 5.0
DEFAULT_USE_SIM_TIME = False
