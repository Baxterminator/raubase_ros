from rclpy.qos import QoSProfile, DurabilityPolicy

TRANSIENT_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
