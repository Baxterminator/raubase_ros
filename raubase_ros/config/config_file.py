import os
from typing import Any, Dict, List, Tuple
from rclpy.logging import get_logger
from shutil import copy2
import yaml

from .static import get_config_file_path, get_default_config_file_path, CONFIG_DIR

logger = get_logger("ConfigFile")


class ConfigFile:
    def __init__(self, name: str, package: str = "raubase_ros") -> None:
        self.name = name
        self.path = get_config_file_path(name)
        self.default = get_default_config_file_path(name)

        self.params: List[Dict[str, Any]] = [{}]
        self.remaps: List[Tuple[str, str]] = []

        self.load_file()

    def load_file(self) -> None:
        """
        Load the YAML file from the file path
        """
        # Test if config file exist
        if not os.path.exists(self.path):
            logger.warn(f"No config file found for {self.path}")
            if not self.install_from_default():
                return

        # Open the file
        with open(self.path, "r") as config_file:
            try:
                data = yaml.safe_load(config_file)
                self.load_from_data(data)
            except yaml.YAMLError as e:
                logger.fatal("Error while loading config file")
                logger.fatal(e)

    def install_from_default(self) -> bool:
        """
        Install the config file inside the right directory by using a copy of the default config file.
        """
        logger.info(
            f"Installing config file {self.name} from default at location {self.default}"
        )

        # Test if file exist from package
        if not os.path.exists(self.default):
            logger.fatal("This config file does not exist !")
            return False

        # Test if dir exist
        if not os.path.exists(CONFIG_DIR):
            os.mkdir(CONFIG_DIR)

        # If exist, copy it inside the user config directory
        copy2(self.default, self.path)
        return True

    def load_from_data(self, data: dict, remaps: bool = False) -> None:
        # Sanity check if data is not dict
        if type(data) is not dict:
            return

        # Iterate over data keys
        for k, v in data.items():
            if remaps:
                self.remaps.append((k, v))
            else:
                # Prevent from using remapping as parameter
                if k == "remaps":
                    self.load_from_data(v, True)
                else:
                    # Load params
                    if type(v) is dict:
                        self.load_from_data(v)
                    else:
                        self.params[0][k] = v

    def get_parameters(self) -> List[Dict[str, Any]]:
        return self.params

    def get_remaps(self) -> List[Tuple[str, str]]:
        return self.remaps
