from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from typing import Generic, Optional, TypeVar, cast
import typing

T = TypeVar("T")


class ParameterWrapper(Generic[T]):
    """
    Wrapper to simplify parameters in ROS2 Python nodes
    """

    def __init__(
        self,
        node: Node,
        name: str,
        value: Optional[T] = None,
        descriptor: Optional[ParameterDescriptor] = None,
        ignore_override: bool = False,
    ) -> None:
        super().__init__()

        self.__type = type(T)
        self.__param = node.declare_parameter(name, value, descriptor, ignore_override)

    def get(self) -> T:
        value = parameter_value_to_python(self.__param.get_parameter_value())
        if not type(value) is T:
            raise RuntimeError(
                f"Parameter {self.__param.name} has been declared of type {self.__type.__name__}, but rclpy is giving type {type(value).__name__}"
            )
        return cast(T, value)
