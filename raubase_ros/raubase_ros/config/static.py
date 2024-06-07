import os
import socket

from ament_index_python import get_package_share_directory

from raubase_ros.constants import (
    DEFAULT_ROBOT_NAME,
    PACKAGE_NAME,
    ConfigFiles,
    File,
    Env,
)


def get_config_file_path(file: str):
    return os.path.join(File.CONFIG_DIR, file + ConfigFiles.EXTENSION)


def get_default_config_file_path(file: str, package: str = PACKAGE_NAME):
    share_dir = get_package_share_directory(package)
    return os.path.join(
        share_dir,
        ConfigFiles.DEFAULT_DIR,
        f"{ConfigFiles.DEFAULT_PREFIX}{file}{ConfigFiles.EXTENSION}",
    )


def is_on_raspberry() -> bool:
    return os.path.exists(
        os.path.join(
            get_package_share_directory(PACKAGE_NAME),
            File.RASPBERRY_TAG,
        )
    )


def get_top_namespace() -> str:
    robot_name = os.getenv(Env.ROBOT_NAME, "")
    if robot_name != "":
        return robot_name
    return socket.gethostname() if is_on_raspberry() else DEFAULT_ROBOT_NAME
