from dataclasses import dataclass
from enum import unique
from raubase_msgs.msg import (
    DataDistance,
    DataEncoder,
    ResultYolo,
    ResultArUco,
    ResultOdometry,
)
from raubase_ros.constants import TRANSIENT_QOS
from sensor_msgs.msg import CompressedImage
from raubase_msgs.msg import (
    CmdMove,
    CmdLineFollower,
    SetControllerInput,
    CmdServoPosition,
)
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from typing import List, Dict, Callable, Any

from raubase_ros.wrappers import NodeWrapper

from .requirements import BaseRequirement, Requirement
from .shared_control import ControlWrapper, SharedControl
from .shared_data import SharedData
from .topics import Topics

import numpy as np


@dataclass
class IOWrapperConf:
    n_distance: int = 2


@unique
class InternalReq(BaseRequirement):
    CONTROLLER = BaseRequirement.next()


class IOWrapper:

    DEFAULT_QOS = 10

    InitFunction = Callable[[NodeWrapper], None]
    RequirementMapping = Dict[BaseRequirement, InitFunction]
    RequirementDependency = Dict[BaseRequirement, List[BaseRequirement]]

    # =========================================================================
    # Methods of interest (for adding new default mappings)
    # =========================================================================
    def __define_requirement_mappings(self) -> None:
        """
        Define the requirement mapping for the Raubase environnement.
        """
        # Sensors
        self.add_req(Requirement.ENCODERS, self.__init_encoders)
        self.add_req(Requirement.DISTANCE, self.__init_all_distance)
        self.add_req(Requirement.ODOMETRY, self.__init_odometry)

        # Camera
        self.add_req(Requirement.CAMERA, self.__init_camera)
        self.add_req(Requirement.ARUCO, self.__init_aruco)
        self.add_req(Requirement.YOLO, self.__init_yolo)

        # Control
        self.add_req(InternalReq.CONTROLLER, self.__init_controller_mode)
        self.add_req(Requirement.MOVE, self.__init_move, InternalReq.CONTROLLER)
        self.add_req(Requirement.LINE, self.__init_line_follow, InternalReq.CONTROLLER)
        self.add_req(Requirement.SERVO, self.__init_servos)

    # -------------------------
    # Sensors
    # -------------------------
    def __init_encoders(self, node: NodeWrapper) -> None:
        """
        Intitialize what is needed for the encoders data to be accessible.
        """

        def encoder_callback(msg: DataEncoder):
            self.state.encoders = msg

        self._sub(node, DataEncoder, Topics.ENCODERS, encoder_callback)

    def __init_all_distance(self, node: NodeWrapper) -> None:
        """
        Initialize all distance sensors
        """
        for i in range(1, self.__conf.n_distance + 1):
            self.__init_distance(node, i)

    def __init_distance(self, node: NodeWrapper, sensor_idx: int) -> None:
        """
        Intitialize what is needed for the IR distances sensors data to be accessible.
        """

        def distance_clbk(msg: DataDistance):
            self.state.ir[sensor_idx] = msg

        self._sub(node, DataEncoder, Topics.DISTANCE.format(sensor_idx), distance_clbk)

    # -------------------------
    # State
    # -------------------------
    def __init_odometry(self, node: NodeWrapper) -> None:
        """
        Setup for receiving the estimated position and speed.
        """

        def clbk(msg: ResultOdometry):
            if self.state.last_odometry is not None:
                # Update elapsed distance
                dx = msg.x - self.state.odometry.x
                dy = msg.y - self.state.odometry.y
                self.state.distance += np.sqrt(dx * dx + dy * dy)

            # Update odometry message
            self.state.last_odometry = msg
            self.state.odometry = msg

        self._sub(node, ResultOdometry, Topics.ODOMETRY, clbk)

    # -------------------------
    # Camera
    # -------------------------
    def __init_camera(self, node: NodeWrapper) -> None:
        """
        Setup for receiving the last img
        """

        def clbk(msg: CompressedImage):
            self.state.last_img = msg

        self._sub(node, CompressedImage, Topics.COMP_IMG, clbk)

    def __init_yolo(self, node: NodeWrapper) -> None:
        """
        Setup for receiving the YOLO processing data
        """

        def clbk(msg: ResultYolo):
            self.state.last_yolo = msg

        self._sub(node, ResultYolo, Topics.YOLO, clbk)

    def __init_aruco(self, node: NodeWrapper) -> None:
        """
        Setup for receiving the Aruco processing data
        """

        def clbk(msg: ResultArUco):
            self.state.last_aruco = msg

        self._sub(node, ResultArUco, Topics.ARUCO, clbk)

    # -------------------------
    # Controls
    # -------------------------
    def __init_controller_mode(self, node: NodeWrapper) -> None:
        """
        Setup for selecting the input source in the velocity controller.
        """
        self.__control.set_cmd = self._pub(node, SetControllerInput, Topics.CONTROLLER)

    def __init_move(self, node: NodeWrapper) -> None:
        """
        Setup for moving the robot.
        """
        self.__control.set_vel = self._pub(node, CmdMove, Topics.MOVE)

        # Initialize "plan" input method for the mixer
        pub = node.create_publisher(
            SetControllerInput, Topics.DECLARE_INPUT, TRANSIENT_QOS
        )
        msg = SetControllerInput()
        msg.input = SetControllerInput.PLAN
        pub.publish(msg)

    def __init_line_follow(self, node: NodeWrapper) -> None:
        """
        Setup for following the line
        """
        self.__control.follow_edge = self._pub(node, CmdLineFollower, Topics.LINE)

    def __init_servos(self, node: NodeWrapper) -> None:
        """
        Setup for using servos
        """
        self.__control.set_servo = self._pub(node, CmdServoPosition, Topics.SERVO)

    # =========================================================================
    # Internal work methods (should not be modified)
    # =========================================================================

    # -------------------------
    # Class methods
    # -------------------------
    def __init__(
        self,
        mapping: RequirementMapping = {},
        conf: IOWrapperConf = IOWrapperConf(),
    ) -> None:
        # Storing struct
        self.state = SharedData()
        self.__control = SharedControl()
        self.controls = ControlWrapper(self.__control)

        # Inner work members
        self.__conf = conf
        self.__subs: List[Subscription] = []  # For keeping a pointer
        self.__pubs: List[Publisher] = []  # For keeping a pointer
        self.__req_mapping: IOWrapper.RequirementMapping = mapping
        self.__req_deps: IOWrapper.RequirementDependency = {}
        self.__define_requirement_mappings()

    # -------------------------
    # Utils
    # -------------------------
    def _sub(
        self,
        node: NodeWrapper,
        T: type,
        topic: str,
        clb,
        qos: QoSProfile | int = DEFAULT_QOS,
    ) -> None:
        """
        Register a internal subscriber and configure its callback.
        """
        self.__subs.append(node.create_subscription(T, topic, clb, qos))

    def _pub(
        self,
        node: NodeWrapper,
        T: type,
        topic: str,
        qos: QoSProfile | int = DEFAULT_QOS,
    ) -> Callable[[Any], None]:
        """
        Register an internal publisher and configure its method
        """
        node.get_logger().warn(f"For topic {topic}, got QoS {qos}")
        pub = node.create_publisher(T, topic, qos)
        self.__pubs.append(pub)

        return self.__pubs[-1].publish

    # -------------------------
    # Requirement handling
    # -------------------------

    def add_req(
        self,
        req: BaseRequirement,
        clbk: InitFunction,
        *parents: BaseRequirement,
    ) -> None:
        """
        Add a callback to call for a requirement.

        Override the original callback if it existed!

        Params:
            - req: the requirement to add
            - clbk: the callback to map to the requirement
            - parents: the several other requirements that it needs to successfuly run
                (don't use the | operator to declare them)
        """
        self.__req_mapping[req] = clbk

        # Register requirements
        if len(parents) != 0:
            if req not in self.__req_deps.keys():
                self.__req_deps[req] = []
            self.__req_deps[req].extend(parents)

    def __recursive_dep_fetching(self, previous: int, need: BaseRequirement) -> int:
        """
        Fetch all needed dependencies recursively.
        """
        if need not in self.__req_deps.keys():
            return int(Requirement.NONE)
        out = previous
        for r in self.__req_deps[need]:
            if (r & out) == 0:
                out |= r | self.__recursive_dep_fetching(out, r)
        return out

    def init_requirements(
        self, node: NodeWrapper, req_group: BaseRequirement | int
    ) -> None:
        """
        Initialize all given requirements.
        """
        # Compute all requirements with their dependencies
        new_group = req_group
        for req in self.__req_deps.keys():
            if (req & req_group) != 0:
                new_group = self.__recursive_dep_fetching(new_group, req)

        # Setup requirements
        group_val = int(new_group)
        node.get_logger().info(f"Initializing requirements (code={int(req_group)})!")
        for req, init_func in self.__req_mapping.items():
            if (req & new_group) != 0:
                node.get_logger().info(
                    f"⊛ Initializing requirement #{int(req)} {req.name}!"
                )
                init_func(node)
                group_val -= int(req)

        # Check if requirements has not been made
        if group_val != int(Requirement.NONE):
            node.get_logger().warn(
                f"All asked requirements are not implemented (left: {group_val})!"
            )

            # Split in power of 2 and test them
            max_n = int(np.floor(np.log2(group_val)))
            while max_n > -1:
                item = np.power(2, max_n)
                if (item & group_val) != 0:
                    kls, enum = BaseRequirement.find_in_subclasses(item)
                    if kls == "" or enum == "":
                        node.get_logger().warn(f"⊛ No candidate found for #{item}")
                    else:
                        node.get_logger().warn(f"⊛ Candidate {kls}.{enum} for #{item}")
                max_n -= 1

        node.get_logger().info("Done for requirements initialization!")
