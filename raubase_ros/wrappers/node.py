from typing import Any, Dict, Generic, List, Optional, TypeVar, cast
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.time import Time

R = TypeVar("R")
T = TypeVar("T")


class ParameterWrapper(Generic[T]):
    """
    Wrapper to simplify parameters in ROS2 Python nodes
    """

    def __init__(
        self,
        node: "Node | NodeWrapper",
        name: str,
        value: Optional[T] = None,
        descriptor: Optional[ParameterDescriptor] = None,
        ignore_override: bool = False,
    ) -> None:
        super().__init__()

        self.__param: Parameter = node.declare_parameter(
            name, value, descriptor, ignore_override
        )

    def get(self) -> T:
        value = parameter_value_to_python(self.__param.get_parameter_value())
        # if not type(value) is T:
        #     raise RuntimeError(
        #         f"Parameter {self.__param.name} has been declared of type {self.__orig_class__.__args__[0]}, but rclpy is giving type {type(value).__name__}"
        #     )
        return cast(T, value)


class NodeWrapper(Node):
    """
    Custom ROS2 Node wrapper to allow some simplifications or custom implementations
    """

    def __init__(
        self,
        node_name: str,
        *,
        context: Context = None,  # type: ignore
        cli_args: List[str] = None,  # type: ignore
        namespace: str = None,  # type: ignore
        use_global_arguments: bool = True,
        enable_rosout: bool = True,
        start_parameter_services: bool = True,
        parameter_overrides: List[Parameter] = None,  # type: ignore
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False,
    ) -> None:
        super().__init__(
            node_name,
            context=context,
            cli_args=cli_args,
            namespace=namespace,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides,
        )

        self.__parameters: Dict[str, ParameterWrapper] = {}

    def declare_wparameter(
        self,
        name: str,
        value: R = None,
        descriptor: ParameterDescriptor | None = None,
        ignore_override: bool = False,
    ) -> ParameterWrapper[R]:
        wrapper = ParameterWrapper[R](self, name, value, descriptor, ignore_override)
        self.__parameters[name] = wrapper
        return wrapper

    def deltaTime(self, fr: Time, to: Time) -> float:
        """
        Get the difference in time in seconds between the two ROS clocks
        """
        fr_s, fr_ns = fr.seconds_nanoseconds()
        to_s, to_ns = to.seconds_nanoseconds()
        return float(to_s - fr_s) + float(to_ns - fr_ns) * 1e-9

    def since(self, fr: Time) -> float:
        """
        Get the difference in time in seconds since the given ROS clock
        """
        return self.deltaTime(fr, self.get_clock().now())
