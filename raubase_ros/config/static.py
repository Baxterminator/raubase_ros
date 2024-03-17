import os
import socket
from ament_index_python import get_package_share_directory

HOME_DIR = os.path.expanduser("~")
CONFIG_DIR = os.path.join(HOME_DIR, "config")
PACKAGE_NAME = "raubase_ros"
FILE_EXT = ".yaml"


def get_config_file_path(file: str):
    return os.path.join(CONFIG_DIR, file + FILE_EXT)


def get_default_config_file_path(file: str, package: str = PACKAGE_NAME):
    share_dir = get_package_share_directory(package)
    return os.path.join(share_dir, "default", f"default_{file}{FILE_EXT}")


def is_on_raspberry() -> bool:
    return os.path.exists(
        os.path.join(
            get_package_share_directory(PACKAGE_NAME),
            "ON_RASPBERRY",
        )
    )


def get_top_namespace() -> str:
    if is_on_raspberry():
        return socket.gethostname()
    return "robobot"
