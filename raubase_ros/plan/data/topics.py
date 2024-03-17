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
    LINE = "line"
