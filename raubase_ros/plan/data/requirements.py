from enum import unique, IntFlag
from typing import Tuple

ENUM_IDX = -1


@unique
class BaseRequirement(IntFlag):

    @staticmethod
    def next() -> int:
        """
        Get the next value for the enum.
        """

        global ENUM_IDX
        if ENUM_IDX > 0:
            ENUM_IDX *= 2
        elif ENUM_IDX == 0:
            ENUM_IDX = 1
        else:
            ENUM_IDX = 0
        return ENUM_IDX

    @staticmethod
    def find_in_subclasses(req: int) -> Tuple[str, str]:
        """
        Find the class and enum names corresponding to this input value.

        If no value is found, return "" for both field.
        """
        for cls in BaseRequirement.__subclasses__():
            # Iterate over its members
            for name, r in cls.__members__.items():
                if r == req:
                    return (cls.__name__, name)
        return ("", "")


@unique
class Requirement(BaseRequirement):
    NONE = BaseRequirement.next()

    # Sensor requirements
    ENCODERS = BaseRequirement.next()
    DISTANCE = BaseRequirement.next()
    ODOMETRY = BaseRequirement.next()

    # Camera requirements
    CAMERA = BaseRequirement.next()
    YOLO = BaseRequirement.next()
    ARUCO = BaseRequirement.next()

    # Control requirements
    MOVE = BaseRequirement.next()
    LINE = BaseRequirement.next()
