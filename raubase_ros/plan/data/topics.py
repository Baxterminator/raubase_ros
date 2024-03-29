class Topics:
    # Sensors data
    ENCODERS = "sensor/encoders"
    DISTANCE = "sensor/dist_{}"
    ODOMETRY = "state/odometry"

    # Camera / Images
    COMP_IMG = "camera/compressed"
    YOLO = "result/yolo"
    ARUCO = "camera/aruco"

    # Control
    DECLARE_INPUT = "control/declare_input"
    CONTROLLER = "control/set_input"
    MOVE = "control/move_plan"
    LINE = "control/line_conf"
    SERVO_CMD = "control/set_servos"
    SERVO_STATE = "sensor/servos"
