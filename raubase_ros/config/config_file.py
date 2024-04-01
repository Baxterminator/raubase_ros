import os, yaml, regex as re
from typing import Any, Dict, List, Tuple
from rclpy.logging import get_logger
from shutil import copy2

from .static import get_config_file_path, get_default_config_file_path
from raubase_ros.constants import PACKAGE_NAME, ConfigFiles

logger = get_logger("ConfigFile")


class ConfigFile:
    """
    Configuration file handler for the raubase stack.
    """

    def __init__(self, name: str, package: str = PACKAGE_NAME) -> None:
        self.name = name
        self.path = get_config_file_path(name)
        self.default = get_default_config_file_path(name, package)

        self.content: str = ""
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
                # Save the content for exporting
                self.content = config_file.read()
                config_file.seek(0, 0)

                # Load YAML data
                data = yaml.safe_load(config_file)
                logger.info(f"Loaded configuration file \"{self.name}\"")
                self.load_from_data(data)
            except yaml.YAMLError as e:
                logger.error("Error while loading config file")
                logger.error(e)

    def install_from_default(self) -> bool:
        """
        Install the config file inside the right directory by using a copy of the default config file.
        """
        logger.info(
            f"Installing config file {self.name} from default at location {self.default}"
        )

        # Test if file exist from package
        if not os.path.exists(self.default):
            logger.fatal("This default config file does not exist !")
            return False

        # Test if dir exist
        if not os.path.exists(ConfigFiles.DEFAULT_DIR):
            os.mkdir(ConfigFiles.DEFAULT_DIR)

        # If exist, copy it inside the user config directory
        copy2(self.default, self.path)
        return True

    def load_from_data(self, data: dict, remaps: bool = False) -> None:
        """
        Load the parameters and remapping from the loaded dict.
        """
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
        """
        Get the parameters name and values in this configuration file.
        """
        return self.params

    def get_remaps(self) -> List[Tuple[str, str]]:
        """
        Get the remappings in this configuration file.
        """
        return self.remaps

    def set_parameter(self, param_name: str, param_value: Any) -> None:
        """
        Set or add the given parameter in the loaded configuration file.
        To apply these changes on the disk, use the save_file() function.
        """
        # Try to find the parameter in the file
        self.content, n = re.subn(
            f"{param_name}: ([a-zA-z0-9,-.]*)",
            f"{param_name}: {param_value}",
            self.content,
        )

        # If the parameter wasn't in the file, add it at the end
        if n == 0:
            self.content += f"\n{param_name}: {param_value}"

    def save_file(self) -> None:
        """
        Save the configuration file locally.
        """
        if not os.path.exists(self.path):
            logger.warn("Trying to save inexistant configuration file !")
            return
        if len(self.content) == 0:
            logger.warn("Trying to save empty configuration file !")
            return
        with open(self.path, "w+") as conf_file:
            conf_file.write(self.content)
        logger.info(f"Saved file {self.path}!")
