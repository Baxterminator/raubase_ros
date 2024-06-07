"""
This file contains all constants related to files
"""

import os


class File:
    HOME_DIR = os.path.expanduser("~")
    CONFIG_DIR = os.path.join(HOME_DIR, "config")

    RASPBERRY_TAG = "ON_RASPBERRY"


class ConfigFiles:
    EXTENSION = ".yaml"  # Config file extension

    # Default config files
    DEFAULT_DIR = "default"
    DEFAULT_PREFIX = "default_"

    class Names:
        """
        This class contains the names of the configuration files.
        """

        LINE = "line"
        CAMERA = "camera"
        VCONTROL = "control"
        LOCALIZATION = "localization"
        TEENSY = "teensy"
