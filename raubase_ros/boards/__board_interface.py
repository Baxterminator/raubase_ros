from abc import abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Tuple


class PinType(Enum):
    ON_FIRST = auto()  # Defining on first usage
    DIGITAL_OUT = auto()  # Output from the RPi towards the hardware
    DIGITAL_IN = auto()  # Input from the hardwards towards the RPi


@dataclass
class PinDesc:
    pin: int
    alias: str
    type: PinType
    special_topic: str | None
    period_ms: float = 20

    @staticmethod
    def default_pin_name(pin: int) -> str:
        return f"gpio{pin}"


class BoardInterface:

    @abstractmethod
    def reserved(self) -> List[int]:
        """
        Get the list of the reserved pins on this board (already wired to something) that people can't open.
        """
        pass

    @abstractmethod
    def specials(self) -> List[PinDesc]:
        """
        Returns the list of the special pins on this board.
        """
        pass
