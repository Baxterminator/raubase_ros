# Raubase Algorithm Result Messages

This page registers the custom results from diverse algorithms.

## Objects

### raubase_msgs::msg::ObjectArUco

This messages contains the data from one ArUco code.


=== "IDL"

    ```
    # ArUco object description
    std_msgs/Header header

    int32 id

    # Corners in images
    float32[4] corners_x
    float32[4] corners_y

    # Real world position and the three axes of the the ArUco marker.
    geometry_msgs/Point x
    geometry_msgs/Vector3 rx
    geometry_msgs/Vector3 ry
    geometry_msgs/Vector3 rz
    ```

=== "C++"

    ```c++
    struct ObjectArUco {
      std_msgs::msg::Header header;
      int id;

      float corners_x[4];
      float corners_y[4];

      geometry_msgs::msg::Point x;
      geometry_msgs::msg::Vector3 rx;
      geometry_msgs::msg::Vector3 ry;
      geometry_msgs::msg::Vector3 rz;
    };
    ```

=== "Python"

    ```Python
    class ObjectArUco:
        def __init__(self):
            self.header : std_msgs.msg.Header
            self.id : int
            self.corners_x : List[float]
            self.corners_y : List[float]
            self.x : geometry_msgs.msg.Point
            self.rx, self.ry, self.rz: geometry_msgs.msg.Vector3
    ```

### raubase_msgs::msg::ObjectBall

This messages contains the data from one detected ball.

=== "IDL"

    ```
    # Ball object description
    std_msgs/Header header

    # Position in image
    int32 x
    int32 y
    float64 r
    ```

=== "C++"

    ```c++
    struct ObjectBall {
      std_msgs::msg::Header header;
      int x, y;
      double r;
    };
    ```

=== "Python"

    ```Python
    class ObjectBall:
        def __init__(self):
            self.header : std_msgs.msg.Header
            self.x, self.y : int
            self.r : float
    ```

### raubase_msgs::msg::ObjectYolo

This messages contains the data from element of the YOLO classifier result.

=== "IDL"

    ```
    # Yolo object descriptor
    std_msgs/Header header

    # Position in image
    int32 xmin
    int32 xmax
    int32 ymin
    int32 ymax

    # Real world position (at center)
    geometry_msgs/Point robot_x

    string classifier
    float64 confidence
    ```

=== "C++"

    ```c++
    struct ObjectYolo {
      std_msgs::msg::Header header;

      int xmin, xmax;
      int ymin, ymax;

      geometry_msgs::msg::Point robot_x;

      std::string classifier;
      double confidence;
    };
    ```

=== "Python"

    ```Python
    class ObjectYolo:
        def __init__(self):
            self.header : std_msgs.msg.Header

            self.xmin, self.xmax : int
            self.ymin, self.ymax : int
            
            self.robot_x : geometry_msgs.msg.Point

            self.classifier : str
            self.confidence : float
    ```

## Results

### raubase_msgs::msg::ResultArUco

This messages contains the gathered data from the ArUco code analysis.


=== "IDL"

    ```
    ObjectArUco[] detected
    ```

=== "C++"

    ```c++
    struct ResultArUco {
        std::vecor<raubase_msgs::msg::ObjectArUco> detected;
    };
    ```

=== "Python"

    ```Python
    class ResultArUco:
        def __init__(self):
            self.detected : List[raubase_msgs.msg.ObjectArUco]
    ```

### raubase_msgs::msg::ResultBall

This messages contains the gathered data from the ball algorithm.

=== "IDL"

    ```
    ObjectBall[] detected
    ```

=== "C++"

    ```c++
    struct ResultBall {
        std::vecor<raubase_msgs::msg::ObjectBall> detected;
    };
    ```

=== "Python"

    ```Python
    class ResultBall:
        def __init__(self):
            self.detected : List[raubase_msgs.msg.ObjectBall]
    ```

### raubase_msgs::msg::ResultYolo

This messages contains the gathered data from the YOLO classifier algorithm.

=== "IDL"

    ```
    ObjectYolo[] detected
    ```

=== "C++"

    ```c++
    struct ResultYolo {
        std::vecor<raubase_msgs::msg::ObjectYolo> detected;
    };
    ```

=== "Python"

    ```Python
    class ResultYolo:
        def __init__(self):
            self.detected : List[raubase_msgs.msg.ObjectYolo]
    ```

### raubase_msgs::msg::ResultEdge

This messages contains the result of the line edge detection.

=== "IDL"

    ```
    bool valid_edge  # If the edge is valid (i.e. it exists)

    # Distances to both edges (in m)
    float32 left_edge  
    float32 right_edge

    # Width of the line (in m)
    float32 width
    ```

=== "C++"

    ```c++
    struct ResultEdge {
        bool valid_edge;
        float left_edge, right_edge;
        float width;
    };
    ```

=== "Python"

    ```Python
    class ResultEdge:
        def __init__(self):
            self.valid_edge : bool
            self.left_edge : float
            self.right_edge : float
            self.width : float
    ```

### raubase_msgs::msg::ResultOdometry

This messages contains the result of the odometry algorithm.

=== "IDL"

    ```
    # This message describe several odometry result of the robot
    builtin_interfaces/Time stamp

    # Left and right wheel velocities (in m/s)
    float64 v_right 
    float64 v_left

    float64 v_lin     # Linear velocity of the robot
    float64 turn_rate # Angular velocity of the robot

    # World position
    float64 x
    float64 y
    float64 heading
    ```

=== "C++"

    ```c++
    struct ResultOdometry {
        builtin_interfaces::msg::Time stamp;

        double v_right, v_left;
        double v_lin, turn_rate;

        double x, y, heading;
    };
    ```

=== "Python"

    ```Python
    class ResultOdometry:
        def __init__(self):
            self.stamp : builtin_interfaces.msg.Time
            self.v_right : float
            self.v_left : float
            self.v_lin : float
            self.turn_rate : float
            self.x, self.y : float
            self.heading : float
    ```