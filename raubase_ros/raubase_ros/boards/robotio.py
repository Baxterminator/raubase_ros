from typing import List

from .__board_interface import BoardInterface, PinDesc, PinType


class RobotIO_23_09_05(BoardInterface):
    """
    Description for the DTU RobotIO board (rev 230905)
    """

    def reserved(self) -> List[int]:
        return list(range(31, 41))

    def specials(self) -> List[PinDesc]:
        return [
            PinDesc(31, "STOP", PinType.DIGITAL_IN, "stop"),
            PinDesc(32, "P3_IO12", PinType.ON_FIRST, None),
            PinDesc(33, "START", PinType.DIGITAL_IN, "start"),
            PinDesc(35, "P5_IO19", PinType.ON_FIRST, None),
            PinDesc(36, "p4_IO16", PinType.ON_FIRST, None),
            PinDesc(37, "P6_IO26", PinType.ON_FIRST, None),
            PinDesc(38, "P8_IO20", PinType.ON_FIRST, None),
            PinDesc(40, "P7_IO21", PinType.ON_FIRST, None),
        ]
