from enum import IntEnum, unique


ENUM_IDX = 1


@unique
class BaseRequirement(IntEnum):

    @staticmethod
    def next() -> int:
        global ENUM_IDX
        ENUM_IDX += 1
        return ENUM_IDX


@unique
class Requirement(BaseRequirement):
    NONE = BaseRequirement.next()

    # Sensor requirements
    ENCODERS = BaseRequirement.next()
    DISTANCE = BaseRequirement.next()

    # Camera requirements
    CAMERA = BaseRequirement.next()
    YOLO = BaseRequirement.next()
    ARUCO = BaseRequirement.next()

    # Control requirements
    MOVE = BaseRequirement.next()
    LINE = BaseRequirement.next()
